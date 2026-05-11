"""MainController 노드 — 체스 워크플로 오케스트레이터 (entry point: ``ros2 run chess_ai main``).

역할:
    한 턴의 체스 흐름 전체를 조율한다:
    보드 상태 샘플링 → 사용자 검증 (rosbridge 경유 Web UI) → Stockfish best move → 로봇 액션.
    State machine: ``IDLE`` → ``SAMPLING`` → ``WAIT_DECISION`` → ``RUNNING`` → ``IDLE``
    (전이는 ``self._state_lock``으로 보호).

ROS2 Interfaces:
    Service: ``~/start_sampling`` (std_srvs/Trigger) — 상태 전이 트리거; IDLE→SAMPLING.
             ``/main_controller/start_sampling``으로 풀림.
    Service: ``~/user_decision`` (chess_ai_interfaces/srv/UserDecision) — Phase 5 sub-phase D2.
             Firebase ui_control polling 대체. state==WAIT_DECISION + job_id 일치 검증 후
             APPROVED → RUNNING, RECHECKED → 유지+final_board 갱신, GAME_OVER → IDLE.
             ``/main_controller/user_decision``으로 풀림.
    Subscriber: Topic ``vision/board_state`` (chess_ai_interfaces/msg/BoardState) —
                RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1). 캐시된 최신값을
                SAMPLING 시 보드 스냅샷, RUNNING 시 live fallback으로 사용.
    Publisher:  Topic ``ui_status`` (chess_ai_interfaces/msg/UIStatus) —
                RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1). main → UI 상태 토픽
                (Phase 5 sub-phase D1). FSM 전이 + verification/working/ai_suggested_move
                업데이트 시 latched publish.
    Client: Service ``StockfishMove``      (chess_ai_interfaces/StockfishMove)
    Client: Action  ``move_chess_piece``  (chess_ai_interfaces/MoveChessPiece)

Threads:
    - Daemon thread ``_job_make_and_publish_board`` — ``VISION_RECEIVE_TIMEOUT_SEC`` 내에
      latched ``vision/board_state`` (TRANSIENT_LOCAL)을 수신. 재샘플링/투표 없음
      (vision 노드가 single source of truth). ``_on_start_sampling``에서 spawn.
    - Daemon thread ``_job_stockfish_then_robot_then_wakeup`` — service call + action goal,
      ``_on_user_decision``의 APPROVED 분기에서 spawn.

외부 의존성:
    - chess_ai_interfaces — ``StockfishMove.srv``, ``UserDecision.srv``,
      ``MoveChessPiece.action``, ``BoardState.msg``, ``UIStatus.msg``, ``GameEvent.msg``.
    - Firebase 의존 0 (sub-phase E 2026-05-10). audit log는 game_logger 노드의
      SQLite append-only DB (Hard Rule #6).

Issues (Phase 1-1 doc Node 1):
    - M1-3 RESOLVED 2026-05-01: env-ized (sub-phase E에서 Firebase 의존 자체 제거되어 무효).
    - M1-1 RESOLVED 2026-05-04: ``~/start_sampling`` (Trigger) 로 대체.
    - ~~M1-2: pub/sub QoS 미명시 (Rule 4)~~ **RESOLVED 2026-05-04**: voice 제거로 pub/sub 0건. service/action endpoint에 ``qos_profile_services_default`` / ``qos_profile_action_status_default`` 명시.
    - M1-4 RESOLVED Phase 5 sub-phase E 2026-05-10: Firebase 의존 0. vision→main,
      main→UI, UI→main, UI→stockfish 모든 채널이 ROS2 native (board_state, ui_status,
      user_decision, set_parameters). audit log는 game_logger + SQLite (Hard Rule #6).
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

from chess_ai_interfaces.msg import BoardState, GameEvent, UIStatus
from chess_ai_interfaces.srv import StockfishMove, UserDecision
from chess_ai_interfaces.action import MoveChessPiece


# ================= [설정 상수: 클래스 밖] =================
# Phase 5 sub-phase E (2026-05-10): Firebase 의존 일괄 제거.
#   chess/board_state — D1에서 main read 제거.
#   chess/chess_system — D3에서 stockfish parameter로 이전.
#   chess/ui_control — E에서 read/write 모두 제거.
# audit log는 game_logger 노드 + SQLite (Hard Rule #6) 영속화.

# Phase 5 sub-phase B: vision→main 버스는 이제 ROS2 토픽(TRANSIENT_LOCAL).
# Relative topic 이름 (Rule 5); main_controller namespace 하위로 풀림.
VISION_BOARD_STATE_TOPIC = "vision/board_state"
VISION_RECEIVE_TIMEOUT_SEC = 3.0

# Phase 5 sub-phase D1: main → UI 상태 토픽. ``~`` private namespace prefix —
# 노드명(main_controller) 하위로 풀려 ``/main_controller/ui_status`` 경로가 됨.
# Rule 5 준수 (절대 경로 하드코딩 없음).
UI_STATUS_TOPIC = "~/ui_status"

# Phase 5 sub-phase E: main → game_logger 명시 게임 이벤트. UI에는 노출 X (audit
# 토픽 안전 경계). depth=10 — late-join logger도 최근 10개 이벤트 받아 갈 수 있게.
GAME_EVENT_TOPIC = "~/game_event"

# FSM 문자열 → UIStatus.STATE_* uint8 매핑. 신규 상태 추가 시 동기화 필요.
_STATE_NAME_TO_UINT = {
    "IDLE": UIStatus.STATE_IDLE,
    "SAMPLING": UIStatus.STATE_SAMPLING,
    "WAIT_DECISION": UIStatus.STATE_WAIT_DECISION,
    "RUNNING": UIStatus.STATE_RUNNING,
}

# Cross-node client 경로 (절대 경로) — owner는 stockfish.py의 chess_ai_node 노드.
# stockfish.py가 사설 네임스페이스 ``~/StockfishMove``로 등록 → 절대 경로
# ``/chess_ai_node/StockfishMove``. ``reset_chess_state``도 같은 패턴. (PB-4 fix.)
STOCKFISH_SERVICE_NAME = "/chess_ai_node/StockfishMove"
SERVICE_TIMEOUT_SEC = 20.0
RESET_CHESS_STATE_SERVICE_NAME = "/chess_ai_node/reset_chess_state"

ROBOT_ACTION_NAME = "move_chess_piece"
ROBOT_ACTION_SEND_TIMEOUT_SEC = 10.0
ROBOT_ACTION_RESULT_TIMEOUT_SEC = 180.0

# Phase 5 sub-phase D3: DEFAULT_DEPTH/DIFFICULTY/TURN 상수 제거 — chess_system
# Firebase 읽기 폐기. fallback 값은 stockfish 노드의 ROS2 parameter (declare_parameter
# 시점에 동일 default 적용). main은 빈 값(0/"")을 Request에 보낼 뿐.

GAME_OVER_TEXT = "게임 종료"
# =========================================================


def now_iso_ms() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


class MainController(Node):
    """워크플로 오케스트레이터 노드.

    State machine:
        IDLE → SAMPLING → WAIT_DECISION → RUNNING → IDLE.

    Triggers:
        - Service ``~/start_sampling`` (Trigger): IDLE → SAMPLING.
          IDLE이 아니면 success=False + message="busy: state=<state>"로 거부.
        - ``~/user_decision`` Service ``DECISION_APPROVED``: WAIT_DECISION → RUNNING.
        - ``~/user_decision`` Service ``DECISION_RECHECKED``: WAIT_DECISION 유지,
          ``corrected_board``로 ``final_board`` 갱신 (Service Request 안의 BoardState).
        - ``~/user_decision`` Service ``DECISION_GAME_OVER``: → IDLE + KIND_GAME_END 이벤트.

    동시성:
        ``self._state_lock`` (mutex) 가 ``self._state``와 ``self._job_id``를 보호.
        rclpy executor 옆에서 단계별 2개의 daemon worker thread가 동작.
    """

    def __init__(self):
        super().__init__("main_controller")

        # Vision board_state subscriber (Phase 5 sub-phase B): TRANSIENT_LOCAL latched
        # 이므로 늦게 가입한 subscriber도 publisher의 최신 메시지를 즉시 받는다.
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

        # GameEvent publisher (Phase 5 sub-phase E). game_logger가 단독 구독.
        # depth=10 — late-join logger에 최근 N개 이벤트 전달 가능.
        game_event_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.game_event_pub = self.create_publisher(
            GameEvent,
            GAME_EVENT_TOPIC,
            game_event_qos,
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

        # GameEvent game lifecycle tracking (sub-phase E). start_sampling 시 신규
        # 발급, GAME_OVER / 빈 best_move (체크메이트) 시 리셋. 빈 문자열 = 진행 중
        # 게임 없음.
        self._current_game_id: str = ""

        # Phase 5 sub-phase D2: _poll_ui_decision 타이머 제거 (M1-5 RESOLVED).
        # 사용자 결정은 ~/user_decision Service callback에서 즉시 처리.

        # 초기 IDLE 상태 latched publish — 페이지 로드 직후 UI 동기화.
        self._publish_ui_status()

        self.get_logger().info(
            "MainController ready. Services: /main_controller/start_sampling (Trigger), "
            "/main_controller/user_decision (UserDecision)."
        )

    def _on_board_state(self, msg: BoardState) -> None:
        # Subscriber callback은 rclpy executor 스레드에서 실행; worker thread는
        # 캐시된 값을 ``_wait_for_board_state``로 소비한다.
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
        """``BoardState`` 메시지가 최소 1회 수신될 때까지 블로킹.

        ``TRANSIENT_LOCAL`` durability 덕분에 publisher의 최신 메시지가 늦게 가입한
        subscriber에 전달되므로 일반적으로 sub-second 대기로 끝난다.
        이후 호출은 즉시 반환되며 (Event는 set 유지) 가장 최근 수신된 메시지를 받는다 —
        vision은 새 프레임이 들어올 때마다 ``_latest_board_state``를 계속 갱신한다.

        Raises:
            TimeoutError: ``timeout_sec`` 내에 메시지 미수신 (vision 노드가 실행
                중이지 않거나 토픽이 bind되지 않음).
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
        """``_state_lock`` 보호 하에 tracking 필드를 스냅샷한 뒤 UIStatus를 발행.

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

    def _publish_game_event(
        self,
        kind: int,
        game_id: str = "",
        job_id: str = "",
        uci: str = "",
        fen: str = "",
        result: str = "",
    ) -> None:
        """game_logger audit 토픽으로 명시 게임 이벤트 발행 (Phase 5 sub-phase E)."""
        msg = GameEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""
        msg.kind = kind
        msg.game_id = game_id
        msg.job_id = job_id
        msg.uci = uci
        msg.fen = fen
        msg.result = result
        self.game_event_pub.publish(msg)

    def _reset_ui_for_new_job(self, job_id: str):
        # Phase 5 sub-phase E: Firebase ui_control reset write 제거. UIStatus 토픽
        # 발행 (이미 _on_start_sampling에서 호출)이 UI 상태 동기화를 담당. stockfish
        # 게임 상태 (dict_memory + castling_rights) 리셋만 잔존.
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
            # Phase 5 sub-phase E: 게임이 진행 중이 아니면 신규 game_id 발급
            # (KIND_GAME_START 이벤트 발행 대상). 이미 진행 중이면 같은 game 안의
            # 다음 사용자 수 사이클로 간주.
            new_game = (self._current_game_id == "")
            if new_game:
                self._current_game_id = self._job_id
            game_id = self._current_game_id
            job_id = self._job_id

        self._publish_ui_status()
        if new_game:
            self._publish_game_event(
                kind=GameEvent.KIND_GAME_START,
                game_id=game_id,
                job_id=job_id,
            )
            self.get_logger().info(f"[GameEvent] GAME_START game_id={game_id}")
        self.get_logger().info(f"[start_sampling] triggered. job_id={job_id} game_id={game_id}")
        self._reset_ui_for_new_job(job_id)
        t = threading.Thread(
            target=self._job_make_and_publish_board, args=(job_id,), daemon=True
        )
        t.start()
        response.success = True
        response.message = "sampling started"
        return response

    def _job_make_and_publish_board(self, job_id: str):
        """Worker thread (daemon) — 보드 상태를 캡처해 사용자 검증을 위해 발행한다.

        Args:
            job_id: str — SAMPLING 진입 시점에 설정된 timestamp 기반 식별자.

        Side Effects:
            - ``VISION_RECEIVE_TIMEOUT_SEC`` 내에 latched ``vision/board_state``
              (TRANSIENT_LOCAL) 메시지를 수신. vision 노드가 single source of truth —
              내부 smoothing / sample voting은 vision의 책임.
            - 성공 시: ``self._state``를 ``WAIT_DECISION``으로 전이, ``UIStatus``
              (verification=True, final_board) + ``GameEvent`` (KIND_USER_BOARD_CONFIRMED) 발행.
            - 예외 시 (vision 침묵으로 인한 ``TimeoutError`` 포함): ``IDLE``로 복귀.
        """
        try:
            self.get_logger().info(
                f"[SAMPLING] waiting for {VISION_BOARD_STATE_TOPIC} (timeout={VISION_RECEIVE_TIMEOUT_SEC:.1f}s)"
            )

            final_dict = self._wait_for_board_state(VISION_RECEIVE_TIMEOUT_SEC)

            self.get_logger().info(f"[SAMPLING] done. final pieces={len(final_dict)}")

            with self._state_lock:
                self._state = "WAIT_DECISION"
                self._verification = True
                self._working = False
                self._final_board = dict(final_dict)
                game_id = self._current_game_id
            self._publish_ui_status()
            self._publish_game_event(
                kind=GameEvent.KIND_USER_BOARD_CONFIRMED,
                game_id=game_id,
                job_id=job_id,
            )

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
        """``~/user_decision`` Service handler (Phase 5 sub-phase D2).

        ``_poll_ui_decision`` Firebase polling을 대체. handler는 rclpy executor 스레드에서
        실행 (default MutuallyExclusiveCallbackGroup, ``_on_start_sampling``과 동일 그룹
        — FSM 전이가 직렬화).

        검증 순서:
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
                ended_game_id = self._current_game_id
                self._state = "IDLE"
                self._verification = False
                self._working = False
                self._ai_suggested_move = GAME_OVER_TEXT
                self._job_id = ""
                self._final_board = {}
                self._current_game_id = ""
            self._publish_ui_status()
            if ended_game_id:
                self._publish_game_event(
                    kind=GameEvent.KIND_GAME_END,
                    game_id=ended_game_id,
                    job_id=job_id,
                    result="resign",
                )
            response.accepted = True
            response.message = "game_over"
            return response

        response.accepted = False
        response.message = f"unknown_decision: {decision}"
        self.get_logger().warn(f"user_decision rejected: unknown decision {decision}")
        return response

    def _job_stockfish_then_robot_then_wakeup(self, job_id: str):
        """Worker thread (daemon) — Stockfish 호출 후 로봇 액션을 전송한다.

        Args:
            job_id: str — WAIT_DECISION 진입 시점에 설정된 timestamp 기반 식별자.

        Side Effects:
            - 보드 source: ``self._final_board`` (UserDecision으로 갱신된 최신값).
              비어 있으면 live ROS2 ``vision/board_state`` fallback (TimeoutError 가능).
            - ``StockfishMove`` Service 호출 (board만 전달) — engine config는 stockfish
              노드 parameter 단일 경로. 응답에 best_move + fen 포함.
            - 빈 ``best_move`` → 게임 종료 (KIND_GAME_END "checkmate") + game_id 리셋.
            - 정상 ``best_move`` → robot action 실행 + KIND_AI_MOVE 이벤트 발행.
            - ``move_chess_piece``에 action goal을 보내고 결과를 최대
              ``ROBOT_ACTION_RESULT_TIMEOUT_SEC`` (=180 s) 대기.
            - ``finally`` 블록: ``self._state``를 ``IDLE``로 복귀.
        """
        try:
            with self._state_lock:
                board_dict = dict(self._final_board)
                game_id = self._current_game_id

            if not board_dict:
                # final_board 비어 있으면 live vision board_state fallback.
                # vision이 침묵 중이면 TimeoutError → outer except에서 정리.
                board_dict = self._wait_for_board_state(VISION_RECEIVE_TIMEOUT_SEC)
                self.get_logger().info("[workflow] using live board_state fallback.")
            else:
                self.get_logger().info("[workflow] using cached final_board (D2 path).")

            # sub-phase D3: 엔진 설정값은 stockfish 노드 parameter 단일 경로.
            best_move, fen = self._call_stockfish(board_dict)
            if not best_move:
                # GAME OVER (체크메이트/스테일메이트): stockfish가 빈 best_move 반환.
                self.get_logger().error("No best_move from stockfish — game over.")
                with self._state_lock:
                    self._ai_suggested_move = GAME_OVER_TEXT
                    ended_game_id = self._current_game_id
                    self._current_game_id = ""
                self._publish_ui_status()
                if ended_game_id:
                    self._publish_game_event(
                        kind=GameEvent.KIND_GAME_END,
                        game_id=ended_game_id,
                        job_id=job_id,
                        result="checkmate",
                    )
                return

            with self._state_lock:
                self._ai_suggested_move = best_move
            self._publish_ui_status()

            ok = self._send_robot_action_and_wait(best_move, board_dict)
            if not ok:
                self.get_logger().error("Robot action failed or timed out.")
                return

            self.get_logger().info("Robot action completed.")
            # KIND_AI_MOVE는 robot action 성공 후 발행 — 실제로 둔 수만 audit에 남김.
            self._publish_game_event(
                kind=GameEvent.KIND_AI_MOVE,
                game_id=game_id,
                job_id=job_id,
                uci=best_move,
                fen=fen,
            )

        except Exception as e:
            self.get_logger().error(f"Workflow failed: {e}")

        finally:
            with self._state_lock:
                self._state = "IDLE"
                self._job_id = ""
                self._working = False
                self._verification = False
            self._publish_ui_status()

    def _call_stockfish(self, board_dict: dict) -> tuple[str, str]:
        """``StockfishMove`` Service를 호출.

        Returns: (best_move, fen). best_move=="" → game over 또는 실패.
        fen은 sub-phase E에서 추가 — game_logger audit에 사용.
        """
        # sub-phase D3: 엔진 설정값(depth/skill_level/turn)은 stockfish 노드 parameter로
        # 일원화. srv는 보드 데이터만 전달.
        if not self.ai_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SEC):
            self.get_logger().error("Stockfish service not available.")
            return "", ""

        req = StockfishMove.Request()
        req.pieces_data = json.dumps(board_dict)
        req.last_move = ""

        future = self.ai_client.call_async(req)

        start = time.time()
        while rclpy.ok() and not future.done():
            if (time.time() - start) > SERVICE_TIMEOUT_SEC:
                self.get_logger().error("Stockfish service call timeout.")
                return "", ""
            time.sleep(0.05)

        resp = future.result()
        if resp is None or (not resp.success) or (not resp.best_move):
            return "", ""
        return resp.best_move, getattr(resp, "fen", "") or ""

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
    except KeyboardInterrupt:
        pass
    finally:
        # rclpy.ok() 가드: SIGINT 시 상위 launch가 이미 context를 shutdown 한 경우
        # 'rcl_shutdown already called' 트레이스를 회피. (PB-6 fix.)
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
