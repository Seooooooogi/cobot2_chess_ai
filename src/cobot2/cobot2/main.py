"""MainController node — chess workflow orchestrator (entry point: ``ros2 run cobot2 main``).

Role:
    Coordinates the end-to-end chess turn:
    sample board state → user verification (Firebase UI) → Stockfish best move → robot action.
    State machine: ``IDLE`` → ``SAMPLING`` → ``WAIT_DECISION`` → ``RUNNING`` → ``IDLE``
    (transitions guarded by ``self._state_lock``).

ROS2 Interfaces:
    Service: ``~/start_sampling`` (std_srvs/Trigger) — state-change trigger; IDLE→SAMPLING.
             Resolves to /main_controller/start_sampling.
    Service: ``~/user_decision`` (cobot2_interfaces/srv/UserDecision) — Phase 5 sub-phase D2.
             Replaces Firebase ui_control polling. Validates state==WAIT_DECISION and
             matching job_id, then APPROVED → RUNNING, RECHECKED → stay+update final_board,
             GAME_OVER → IDLE. Resolves to /main_controller/user_decision.
    Subscriber: Topic ``vision/board_state`` (cobot2_interfaces/msg/BoardState) —
                RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1). Cached latest is used as the
                board snapshot for SAMPLING and as the live fallback in RUNNING.
    Publisher:  Topic ``ui_status`` (cobot2_interfaces/msg/UIStatus) —
                RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1). main → UI 상태 토픽
                (Phase 5 sub-phase D1). FSM 전이 + verification/working/ai_suggested_move
                업데이트 시 latched publish.
    Client: Service ``StockfishMove``      (cobot2_interfaces/StockfishMove)
    Client: Action  ``move_chess_piece``  (cobot2_interfaces/MoveChessPiece)

Threads:
    - Daemon thread ``_job_make_and_publish_board`` — receives the latched ``vision/board_state``
      message (TRANSIENT_LOCAL) within ``VISION_RECEIVE_TIMEOUT_SEC``, no resampling/voting
      (single source of truth from vision node), spawned in ``_on_start_sampling``.
    - Daemon thread ``_job_stockfish_then_robot_then_wakeup`` — service call + action goal,
      spawned in ``_on_user_decision`` (APPROVED branch).

External Dependencies:
    - Firebase Realtime DB — read/write ``chess/ui_control`` (sub-phase E에서 일괄 제거).
      ``chess/board_state`` (D1)와 ``chess/chess_system`` (D3) 경로는 main 측에서 더 이상
      참조하지 않음.
    - cobot2_interfaces — ``StockfishMove.srv``, ``MoveChessPiece.action``,
      ``UIStatus.msg``, ``UserDecision.srv``, ``BoardState.msg``.

Issues (Phase 1-1 doc Node 1):
    - M1-3 RESOLVED 2026-05-01: ``FIREBASE_SERVICE_ACCOUNT_JSON`` env-ized via ``FIREBASE_SERVICE_ACCOUNT_PATH`` env var; ``FIREBASE_DB_URL`` via ``FIREBASE_DATABASE_URL`` env var.
    - M1-1 RESOLVED 2026-05-04: ``~/start_sampling`` (Trigger) 로 대체.
    - ~~M1-2: pub/sub QoS 미명시 (Rule 4)~~ **RESOLVED 2026-05-04**: voice 제거로 pub/sub 0건. service/action endpoint에 ``qos_profile_services_default`` / ``qos_profile_action_status_default`` 명시 (line 234-256).
    - M1-4 PARTIAL Phase 5 sub-phase D3 2026-05-10: vision→main, main→UI status, UI→main
      user_decision, UI→stockfish parameter 모두 ROS2로 이전 (``vision/board_state``,
      ``ui_status``, ``user_decision``, stockfish 노드 ``depth``/``skill_level``/
      ``default_turn`` parameter). Firebase ``ui_control`` writes는 additive 잔존
      (sub-phase E에서 일괄 제거 + game_logger 노드 신설).
    - M1-5 RESOLVED Phase 5 sub-phase D2 2026-05-10: ``_poll_ui_decision`` 0.2s timer
      제거. ``~/user_decision`` Service handler가 즉시 처리. workflow thread 내부
      Future polling (``_call_stockfish``, ``_send_robot_action_and_wait``)은 별도 트랙.
    - M1-6 RESOLVED 2026-05-04: Service 로 대체 — voice_control_node 미실행 무한 대기 해소.
    - M1-7 RESOLVED 2026-05-04: voice_status pub 제거 (옵션 a) — dead pub 해소.
"""

import json
import os
import time
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    qos_profile_services_default,
    qos_profile_action_status_default,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from std_srvs.srv import Trigger

import firebase_admin
from firebase_admin import credentials, db

from cobot2_interfaces.msg import BoardState, UIStatus
from cobot2_interfaces.srv import StockfishMove, UserDecision
from cobot2_interfaces.action import MoveChessPiece


# ================= [설정 상수: 클래스 밖] =================
FIREBASE_SERVICE_ACCOUNT_JSON = os.getenv("FIREBASE_SERVICE_ACCOUNT_PATH")
FIREBASE_DB_URL = os.getenv("FIREBASE_DATABASE_URL", "https://chess-43355-default-rtdb.asia-southeast1.firebasedatabase.app")

BOARD_STATE_PATH = "chess/board_state"
UI_CONTROL_PATH = "chess/ui_control"
# Phase 5 sub-phase D3: chess/chess_system Firebase 경로 제거. depth/skill_level/
# default_turn은 stockfish 노드의 ROS2 parameter로 직접 관리 (UI → rosbridge
# set_parameters). main은 Request에 빈 값 전달 → stockfish handler가 자기
# parameter로 fallback.

# Phase 5 sub-phase B: vision→main bus is now ROS2 topic (TRANSIENT_LOCAL).
# Relative topic name (Rule 5); resolves under main_controller's namespace.
VISION_BOARD_STATE_TOPIC = "vision/board_state"
VISION_RECEIVE_TIMEOUT_SEC = 3.0

# Phase 5 sub-phase D1: main → UI 상태 토픽. ``~`` private namespace prefix —
# 노드명(main_controller) 하위로 풀려 ``/main_controller/ui_status`` 경로가 됨.
# Rule 5 준수 (절대 경로 하드코딩 없음).
UI_STATUS_TOPIC = "~/ui_status"

# FSM 문자열 → UIStatus.STATE_* uint8 매핑. 신규 상태 추가 시 동기화 필요.
_STATE_NAME_TO_UINT = {
    "IDLE": UIStatus.STATE_IDLE,
    "SAMPLING": UIStatus.STATE_SAMPLING,
    "WAIT_DECISION": UIStatus.STATE_WAIT_DECISION,
    "RUNNING": UIStatus.STATE_RUNNING,
}

STOCKFISH_SERVICE_NAME = "StockfishMove"
SERVICE_TIMEOUT_SEC = 20.0
RESET_CHESS_STATE_SERVICE_NAME = "reset_chess_state"

ROBOT_ACTION_NAME = "move_chess_piece"
ROBOT_ACTION_SEND_TIMEOUT_SEC = 10.0
ROBOT_ACTION_RESULT_TIMEOUT_SEC = 180.0

# Phase 5 sub-phase D3: DEFAULT_DEPTH/DIFFICULTY/TURN 상수 제거 — chess_system
# Firebase 읽기 폐기. fallback 값은 stockfish 노드의 ROS2 parameter (declare_parameter
# 시점에 동일 default 적용). main은 빈 값(0/"")을 Request에 보낼 뿐.

# Firebase ui_control.user_decision "NONE" 리셋용 상수 — additive Firebase writes에서
# 사용. Phase 5 sub-phase D2에서 APPROVED/RECHECKED 상수는 UserDecision.srv로 이전.
DECISION_NONE = "NONE"

CMD_IDLE = "idle"

GAME_OVER_TEXT = "게임 종료"
# =========================================================


def now_iso_ms() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


class FirebaseClient:
    def __init__(self, service_account_json: str, db_url: str):
        self.service_account_json = service_account_json
        self.db_url = db_url
        self._initialized = False

    def init(self):
        if self._initialized:
            return
        if not firebase_admin._apps:
            cred = credentials.Certificate(self.service_account_json)
            firebase_admin.initialize_app(cred, {"databaseURL": self.db_url})
        self._initialized = True

    def get_board_dict(self, board_state_path: str) -> dict:
        self.init()
        data = db.reference(board_state_path).get()
        if data is None:
            return {}
        if isinstance(data, dict) and "board" in data and isinstance(data["board"], dict):
            return data["board"]
        if isinstance(data, dict):
            return data
        return {}

    def set_board_state(self, board_state_path: str, board_dict: dict, extra: dict = None):
        self.init()
        payload = {
            "updated_at": now_iso_ms(),
            "piece_count": len(board_dict),
            "board": board_dict,
        }
        if isinstance(extra, dict):
            payload.update(extra)
        db.reference(board_state_path).set(payload)

    def update_ui_control(self, ui_control_path: str, patch: dict):
        self.init()
        db.reference(ui_control_path).update(patch)

    def get_ui_control(self, ui_control_path: str) -> dict:
        self.init()
        data = db.reference(ui_control_path).get()
        return data if isinstance(data, dict) else {}


class MainController(Node):
    """Workflow orchestrator node.

    State machine:
        IDLE → SAMPLING → WAIT_DECISION → RUNNING → IDLE.

    Triggers:
        - Service ``~/start_sampling`` (Trigger): IDLE → SAMPLING.
          Returns success=False with message="busy: state=<state>" if not IDLE.
        - Firebase ``ui_control.user_decision == "APPROVED"`` (timer poll): WAIT_DECISION → RUNNING.
        - Firebase ``ui_control.user_decision == "RE-CHECKED"`` (timer poll): stays in WAIT_DECISION,
          updates ``final_board`` from ``corrected_board``.

    Concurrency:
        ``self._state_lock`` (mutex) guards ``self._state`` and ``self._job_id``.
        Two daemon worker threads (one per phase) run alongside the rclpy executor.
    """

    def __init__(self):
        super().__init__("main_controller")

        self.fb = FirebaseClient(FIREBASE_SERVICE_ACCOUNT_JSON, FIREBASE_DB_URL)

        # Vision board_state subscriber (Phase 5 sub-phase B): TRANSIENT_LOCAL latched
        # so a late-joining subscriber receives the publisher's most recent message.
        self._latest_board_state: dict | None = None
        self._latest_board_state_lock = threading.Lock()
        self._board_state_received_event = threading.Event()
        board_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.board_state_sub = self.create_subscription(
            BoardState,
            VISION_BOARD_STATE_TOPIC,
            self._on_board_state,
            board_state_qos,
        )

        # UIStatus publisher (Phase 5 sub-phase D1). 페이지 로드 직후 latched 메시지로
        # 최신 상태 즉시 전달 — board_state QoS와 동일.
        self.ui_status_pub = self.create_publisher(
            UIStatus,
            UI_STATUS_TOPIC,
            board_state_qos,
        )

        self.ai_client = self.create_client(
            StockfishMove,
            STOCKFISH_SERVICE_NAME,
            qos_profile=qos_profile_services_default,
        )
        self.reset_client = self.create_client(
            Trigger,
            RESET_CHESS_STATE_SERVICE_NAME,
            qos_profile=qos_profile_services_default,
        )
        self.robot_action_client = ActionClient(
            self,
            MoveChessPiece,
            ROBOT_ACTION_NAME,
            goal_service_qos_profile=qos_profile_services_default,
            result_service_qos_profile=qos_profile_services_default,
            cancel_service_qos_profile=qos_profile_services_default,
            feedback_sub_qos_profile=qos_profile_services_default,
            status_sub_qos_profile=qos_profile_action_status_default,
        )

        self.start_sampling_srv = self.create_service(
            Trigger,
            "~/start_sampling",
            self._on_start_sampling,
            qos_profile=qos_profile_services_default,
        )

        # Phase 5 sub-phase D2: UserDecision Service — Firebase ui_control polling 대체.
        # ``~`` private namespace → /main_controller/user_decision. 같은 callback group
        # (default MutuallyExclusive) 안에서 _on_start_sampling과 직렬 실행되어
        # FSM 전이가 race-free.
        self.user_decision_srv = self.create_service(
            UserDecision,
            "~/user_decision",
            self._on_user_decision,
            qos_profile=qos_profile_services_default,
        )

        self._state_lock = threading.Lock()
        self._state = "IDLE"
        self._job_id = ""

        # UIStatus tracking fields (write 보호: _state_lock).
        # FSM 전이 / 작업 진행 시점에 갱신 후 _publish_ui_status() 호출.
        self._verification = False
        self._working = False
        self._ai_suggested_move = ""
        self._final_board: dict = {}

        # Phase 5 sub-phase D2: _poll_ui_decision 타이머 제거 (M1-5 RESOLVED).
        # 사용자 결정은 ~/user_decision Service callback에서 즉시 처리.

        # 초기 IDLE 상태 latched publish — 페이지 로드 직후 UI 동기화.
        self._publish_ui_status()

        self.get_logger().info(
            "MainController ready. Services: /main_controller/start_sampling (Trigger), "
            "/main_controller/user_decision (UserDecision)."
        )

    def _on_board_state(self, msg: BoardState) -> None:
        # Subscriber callback runs on the rclpy executor thread; worker threads consume
        # the cached value via _wait_for_board_state.
        if len(msg.squares) != len(msg.pieces):
            self.get_logger().warn(
                f"BoardState arrays length mismatch: squares={len(msg.squares)} pieces={len(msg.pieces)} — discarding."
            )
            return
        board = dict(zip(msg.squares, msg.pieces))
        with self._latest_board_state_lock:
            self._latest_board_state = board
        self._board_state_received_event.set()

    def _wait_for_board_state(self, timeout_sec: float) -> dict:
        """Block until at least one ``BoardState`` message has been received.

        With ``TRANSIENT_LOCAL`` durability the publisher's most recent message is
        delivered to a late-joining subscriber, so the typical wait is sub-second.
        Subsequent calls return immediately (Event remains set) with whatever the
        most recent received message was — vision continues updating
        ``_latest_board_state`` as new frames arrive.

        Raises:
            TimeoutError: no message received within ``timeout_sec`` (vision
                node not running or topic unbound).
        """
        if not self._board_state_received_event.wait(timeout=timeout_sec):
            raise TimeoutError(
                f"No {VISION_BOARD_STATE_TOPIC} received within {timeout_sec:.1f}s "
                "(is vision node running?)"
            )
        with self._latest_board_state_lock:
            assert self._latest_board_state is not None
            return dict(self._latest_board_state)

    def _publish_ui_status(self) -> None:
        """Snapshot tracking fields under ``_state_lock`` and publish UIStatus.

        모든 FSM 전이 / verification·working·ai_suggested_move·final_board 업데이트
        직후 호출. 락은 read 동안만 잡고 publish() 자체는 락 밖에서 실행 — rclpy
        publisher는 thread-safe.
        """
        msg = UIStatus()
        stamp = self.get_clock().now().to_msg()
        msg.header.stamp = stamp
        msg.header.frame_id = ""

        with self._state_lock:
            state_name = self._state
            # 신규 FSM 상태 추가 시 _STATE_NAME_TO_UINT 동기화 누락 시에도 worker thread가
            # 죽지 않도록 IDLE로 fallback. 매핑 누락은 외부에 명시.
            msg.controller_state = _STATE_NAME_TO_UINT.get(state_name, UIStatus.STATE_IDLE)
            msg.verification = self._verification
            msg.working = self._working
            msg.ai_suggested_move = self._ai_suggested_move
            msg.job_id = self._job_id
            board_items = sorted(self._final_board.items())

        if state_name not in _STATE_NAME_TO_UINT:
            self.get_logger().error(
                f"_publish_ui_status: unknown FSM state '{state_name}' "
                "→ fallback IDLE in UIStatus. _STATE_NAME_TO_UINT 동기화 필요."
            )

        msg.final_board.header.stamp = stamp
        msg.final_board.header.frame_id = "chess_board"
        msg.final_board.squares = [k for k, _ in board_items]
        msg.final_board.pieces = [v for _, v in board_items]
        msg.final_board.piece_count = len(board_items)
        self.ui_status_pub.publish(msg)

    def _reset_ui_for_new_job(self, job_id: str):
        try:
            self.fb.update_ui_control(UI_CONTROL_PATH, {
                "verification": False,
                "user_decision": DECISION_NONE,
                "final_board": None,
                "corrected_board": None,
                "working": False,
                "job_id": job_id,
                "timestamp": datetime.now().isoformat(),
            })
        except Exception as e:
            self.get_logger().warn(f"UI reset failed: {e}")

        if self.reset_client.wait_for_service(timeout_sec=2.0):
            self.reset_client.call_async(Trigger.Request())
        else:
            self.get_logger().warn("reset_chess_state service unavailable; skipping chess state reset.")

    def _on_start_sampling(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        with self._state_lock:
            if self._state != "IDLE":
                response.success = False
                response.message = f"busy: state={self._state}"
                self.get_logger().warn(
                    f"start_sampling rejected (state={self._state})."
                )
                return response
            self._state = "SAMPLING"
            self._job_id = now_iso_ms()
            self._verification = False
            self._working = False
            self._ai_suggested_move = ""
            self._final_board = {}
            job_id = self._job_id

        self._publish_ui_status()
        self.get_logger().info(f"[start_sampling] triggered. job_id={job_id}")
        self._reset_ui_for_new_job(job_id)
        t = threading.Thread(
            target=self._job_make_and_publish_board, args=(job_id,), daemon=True
        )
        t.start()
        response.success = True
        response.message = "sampling started"
        return response

    def _job_make_and_publish_board(self, job_id: str):
        """Worker thread (daemon) — capture board state and publish for user verification.

        Args:
            job_id: str — timestamp-based identifier set when SAMPLING was entered.

        Side Effects:
            - Receives the latched ``vision/board_state`` message (TRANSIENT_LOCAL) within
              ``VISION_RECEIVE_TIMEOUT_SEC``. Single source of truth from vision node —
              vision is responsible for any internal smoothing / sample voting.
            - Writes ``ui_control.verification = True`` to Firebase + publishes
              ``UIStatus`` (verification=True, final_board, controller_state=WAIT_DECISION).
              Firebase ``chess/board_state.set()`` write removed in sub-phase D1 — UI consumes
              ``/vision/board_state`` directly.
            - On success: transitions ``self._state`` to ``WAIT_DECISION``.
            - On exception (incl. ``TimeoutError`` if vision is silent): transitions back to ``IDLE``.
        """
        try:
            self.get_logger().info(
                f"[SAMPLING] waiting for {VISION_BOARD_STATE_TOPIC} (timeout={VISION_RECEIVE_TIMEOUT_SEC:.1f}s)"
            )

            final_dict = self._wait_for_board_state(VISION_RECEIVE_TIMEOUT_SEC)

            self.get_logger().info(f"[SAMPLING] done. final pieces={len(final_dict)}")
            self.get_logger().info("[UI] uploading final_board and enabling verification")

            # sub-phase D1: chess/board_state.set() 제거 — UI는 ROS2 /vision/board_state 구독.
            self.fb.update_ui_control(UI_CONTROL_PATH, {
                "verification": True,
                "user_decision": DECISION_NONE,
                "final_board": final_dict,
                "corrected_board": None,
                "working": False,
                "timestamp": datetime.now().isoformat(),
                "job_id": job_id,
            })

            with self._state_lock:
                self._state = "WAIT_DECISION"
                self._verification = True
                self._working = False
                self._final_board = dict(final_dict)
            self._publish_ui_status()

        except Exception as e:
            self.get_logger().error(f"Failed to make/publish final_dict: {e}")
            with self._state_lock:
                self._state = "IDLE"
                self._job_id = ""
                self._verification = False
                self._working = False
                self._final_board = {}
            self._publish_ui_status()

    def _on_user_decision(
        self, request: UserDecision.Request, response: UserDecision.Response
    ) -> UserDecision.Response:
        """Service handler for ``~/user_decision`` (Phase 5 sub-phase D2).

        Replaces ``_poll_ui_decision`` Firebase polling. Handler runs in the rclpy
        executor thread (default MutuallyExclusiveCallbackGroup, same as
        ``_on_start_sampling`` — FSM transitions are serialized).

        Validation order:
            1. ``self._state == "WAIT_DECISION"`` — 다른 상태면 거부.
            2. ``self._job_id == request.job_id`` — stale 결정 거부.
            3. ``request.decision in {APPROVED, RECHECKED, GAME_OVER}`` — unknown 거부.

        ``request.corrected_board`` (BoardState) 가 비어있지 않으면 ``self._final_board``를
        해당 dict로 교체. APPROVED/RECHECKED 모두 동일 의미.
        """
        # 1. State + job_id 검증
        with self._state_lock:
            current_state = self._state
            current_job_id = self._job_id

        if current_state != "WAIT_DECISION":
            response.accepted = False
            response.message = f"wrong_state: {current_state}"
            self.get_logger().warn(
                f"user_decision rejected: state={current_state} (need WAIT_DECISION)"
            )
            return response

        if current_job_id != request.job_id:
            response.accepted = False
            response.message = f"stale_job: have={current_job_id} got={request.job_id}"
            self.get_logger().warn(
                f"user_decision rejected: stale job_id (have={current_job_id}, got={request.job_id})"
            )
            return response

        # 2. corrected_board 처리 — 비어있지 않으면 _final_board 갱신
        corrected = request.corrected_board
        if (
            corrected.squares
            and len(corrected.squares) == len(corrected.pieces)
        ):
            new_board = dict(zip(corrected.squares, corrected.pieces))
            with self._state_lock:
                self._final_board = new_board
            self.get_logger().info(
                f"[UI] corrected_board applied: {len(new_board)} pieces"
            )

        # 3. decision branching
        decision = int(request.decision)
        job_id = request.job_id

        if decision == UserDecision.Request.DECISION_APPROVED:
            self.get_logger().info("[UI] APPROVED. start stockfish/robot workflow")
            try:
                self.fb.update_ui_control(UI_CONTROL_PATH, {
                    "user_decision": DECISION_NONE,
                    "verification": False,
                    "working": True,
                })
            except Exception:
                pass
            with self._state_lock:
                if self._state != "WAIT_DECISION":
                    response.accepted = False
                    response.message = "state_changed_during_handling"
                    return response
                self._state = "RUNNING"
                self._verification = False
                self._working = True
            self._publish_ui_status()
            t = threading.Thread(
                target=self._job_stockfish_then_robot_then_wakeup,
                args=(job_id,),
                daemon=True,
            )
            t.start()
            response.accepted = True
            response.message = "approved"
            return response

        if decision == UserDecision.Request.DECISION_RECHECKED:
            self.get_logger().info("[UI] RECHECKED. final_board updated, staying in WAIT_DECISION")
            with self._state_lock:
                final_snapshot = dict(self._final_board)
            try:
                self.fb.update_ui_control(UI_CONTROL_PATH, {
                    "final_board": final_snapshot,
                    "corrected_board": None,
                    "user_decision": DECISION_NONE,
                    "timestamp": datetime.now().isoformat(),
                    "job_id": job_id,
                })
            except Exception:
                pass
            self._publish_ui_status()
            response.accepted = True
            response.message = "rechecked"
            return response

        if decision == UserDecision.Request.DECISION_GAME_OVER:
            # 현 UI(UI.html btn-ok/btn-check)에는 GAME_OVER 버튼이 배선되어 있지 않음.
            # 향후 "give up" 버튼 추가 시 callUserDecision(DECISION_GAME_OVER, {}) 사용.
            # 예약된 slot — main 핸들러는 이미 동작.
            self.get_logger().info("[UI] GAME_OVER. transitioning to IDLE")
            with self._state_lock:
                self._state = "IDLE"
                self._verification = False
                self._working = False
                self._ai_suggested_move = GAME_OVER_TEXT
                self._job_id = ""
                self._final_board = {}
            try:
                self.fb.update_ui_control(UI_CONTROL_PATH, {
                    "ai_suggested_move": GAME_OVER_TEXT,
                    "verification": False,
                    "working": False,
                    "user_decision": DECISION_NONE,
                    "timestamp": datetime.now().isoformat(),
                })
            except Exception:
                pass
            self._publish_ui_status()
            response.accepted = True
            response.message = "game_over"
            return response

        response.accepted = False
        response.message = f"unknown_decision: {decision}"
        self.get_logger().warn(f"user_decision rejected: unknown decision {decision}")
        return response

    def _job_stockfish_then_robot_then_wakeup(self, job_id: str):
        """Worker thread (daemon) — call Stockfish and send robot action.

        Args:
            job_id: str — timestamp-based identifier set when WAIT_DECISION was entered.

        Side Effects:
            - Reads Firebase ``ui_control`` to choose the input board (``corrected_board`` if present
              and non-empty, else ``final_board``, else live ``board_state``).
            - Calls service ``StockfishMove`` with the board only (sub-phase D3) — engine config
              (depth/skill_level/default_turn)은 stockfish 노드의 ROS2 parameter 단일 경로.
              On empty ``best_move`` writes ``ai_suggested_move = "게임 종료"`` to ``ui_control``
              and returns (game-over branch).
            - Sends an action goal to ``move_chess_piece`` and waits up to
              ``ROBOT_ACTION_RESULT_TIMEOUT_SEC`` (=180 s) for the result.
            - In the ``finally`` block: writes ``ui_control.working = False`` and transitions
              ``self._state`` to ``IDLE``.
        """
        try:
            ui = self.fb.get_ui_control(UI_CONTROL_PATH)

            corrected_board = ui.get("corrected_board")
            final_board = ui.get("final_board")

            if isinstance(corrected_board, dict) and corrected_board:
                board_dict = corrected_board
                try:
                    self.fb.update_ui_control(UI_CONTROL_PATH, {
                        "final_board": board_dict,
                        "corrected_board": None,
                        "job_id": job_id,
                        "timestamp": datetime.now().isoformat(),
                    })
                except Exception:
                    pass
                self.get_logger().info("[UI] Using corrected_board for stockfish/robot (APPROVED).")

            elif isinstance(final_board, dict) and final_board:
                board_dict = final_board
                self.get_logger().info("[UI] Using final_board for stockfish/robot (APPROVED).")
            else:
                # Live fallback (Phase 5 sub-phase B): pull the most recent ROS2 board_state
                # cached by the subscriber. If vision has been silent since startup this
                # raises TimeoutError, surfaced as a workflow exception (logged and
                # cleaned up by the outer ``finally``).
                board_dict = self._wait_for_board_state(VISION_RECEIVE_TIMEOUT_SEC)
                self.get_logger().info("[UI] Using board_state for stockfish/robot (APPROVED).")

            # sub-phase D3: 엔진 설정값은 stockfish 노드 parameter 단일 경로. UI가
            # set_parameters로 미리 설정한 값을 stockfish가 직접 읽어 사용.
            best_move = self._call_stockfish(board_dict)
            if not best_move:
                # ✅ GAME OVER 처리: best_move가 없으면 UI에 '게임 종료' 표시 후 종료 루틴으로 빠짐
                self.get_logger().error("No best_move from stockfish.")
                try:
                    self.fb.update_ui_control(UI_CONTROL_PATH, {
                        "ai_suggested_move": GAME_OVER_TEXT,
                        "ai_updated_at": datetime.now().isoformat(),
                        "job_id": job_id,
                    })
                except Exception:
                    pass
                with self._state_lock:
                    self._ai_suggested_move = GAME_OVER_TEXT
                self._publish_ui_status()
                return

            self.fb.update_ui_control(UI_CONTROL_PATH, {
                "ai_suggested_move": best_move,
                "ai_updated_at": datetime.now().isoformat(),
                "command": CMD_IDLE,
                "job_id": job_id,
            })
            with self._state_lock:
                self._ai_suggested_move = best_move
            self._publish_ui_status()

            ok = self._send_robot_action_and_wait(best_move, board_dict)
            if not ok:
                self.get_logger().error("Robot action failed or timed out.")
                return

            self.get_logger().info("Robot action completed.")

        except Exception as e:
            self.get_logger().error(f"Workflow failed: {e}")

        finally:
            try:
                self.fb.update_ui_control(UI_CONTROL_PATH, {"working": False})
            except Exception:
                pass

            with self._state_lock:
                self._state = "IDLE"
                self._job_id = ""
                self._working = False
                self._verification = False
            self._publish_ui_status()

    def _call_stockfish(self, board_dict: dict) -> str:
        # sub-phase D3: 엔진 설정값(depth/skill_level/turn)은 stockfish 노드 parameter로
        # 일원화. srv는 보드 데이터만 전달.
        if not self.ai_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SEC):
            self.get_logger().error("Stockfish service not available.")
            return ""

        req = StockfishMove.Request()
        req.pieces_data = json.dumps(board_dict)
        req.last_move = ""

        future = self.ai_client.call_async(req)

        start = time.time()
        while rclpy.ok() and not future.done():
            if (time.time() - start) > SERVICE_TIMEOUT_SEC:
                self.get_logger().error("Stockfish service call timeout.")
                return ""
            time.sleep(0.05)

        resp = future.result()
        if resp is None or (not resp.success) or (not resp.best_move):
            return ""
        return resp.best_move

    def _send_robot_action_and_wait(self, best_move: str, board_dict: dict) -> bool:
        if not self.robot_action_client.wait_for_server(timeout_sec=ROBOT_ACTION_SEND_TIMEOUT_SEC):
            self.get_logger().error("Robot action server not available.")
            return False

        goal = MoveChessPiece.Goal()
        goal.command = best_move
        goal.pieces_dict = json.dumps(board_dict)

        send_future = self.robot_action_client.send_goal_async(goal)

        start = time.time()
        while rclpy.ok() and not send_future.done():
            if (time.time() - start) > ROBOT_ACTION_SEND_TIMEOUT_SEC:
                self.get_logger().error("Action goal send timeout.")
                return False
            time.sleep(0.05)

        goal_handle = send_future.result()
        if goal_handle is None or (not goal_handle.accepted):
            self.get_logger().error("Action goal rejected.")
            return False

        result_future = goal_handle.get_result_async()

        start = time.time()
        while rclpy.ok() and not result_future.done():
            if (time.time() - start) > ROBOT_ACTION_RESULT_TIMEOUT_SEC:
                self.get_logger().error("Action result timeout.")
                return False
            time.sleep(0.05)

        result = result_future.result()
        if result is None:
            return False

        return bool(result.result.success)



def main(args=None):
    rclpy.init(args=args)
    node = MainController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
