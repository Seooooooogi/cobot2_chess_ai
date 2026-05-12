"""체스 워크플로 오케스트레이터 노드 (entry point: ``ros2 run chess_ai main``).

한 턴의 흐름 전체를 조율한다:
보드 상태 샘플링 → 사용자 검증 (rosbridge 경유 Web UI) → Stockfish best move → 로봇 액션.

FSM:
    ``IDLE → SAMPLING → WAIT_DECISION → RUNNING → IDLE``. 전이는 ``self._state_lock``
    (mutex)으로 보호된다.

ROS2 Interfaces:
    Services hosted:
        ~/start_sampling (std_srvs/Trigger): IDLE → SAMPLING.
        ~/user_decision (chess_ai_interfaces/srv/UserDecision): WAIT_DECISION 분기 처리.
    Subscribers:
        vision/board_state (chess_ai_interfaces/msg/BoardState):
            RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1). 최신값 캐시.
    Publishers:
        ~/ui_status (chess_ai_interfaces/msg/UIStatus):
            RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1). FSM 전이 시 latched publish.
        ~/game_event (chess_ai_interfaces/msg/GameEvent):
            RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(10). game_logger audit 입력.
    Clients:
        /chess_ai_node/StockfishMove (chess_ai_interfaces/srv/StockfishMove).
        /chess_ai_node/reset_chess_state (std_srvs/srv/Trigger).
        move_chess_piece (chess_ai_interfaces/action/MoveChessPiece).

Threads:
    Daemon worker ``_job_make_and_publish_board``: SAMPLING 단계에서 latched board_state를
        대기. ``_on_start_sampling``에서 spawn.
    Daemon worker ``_job_stockfish_then_robot_then_wakeup``: Stockfish service + robot
        action 직렬 호출. ``_on_user_decision`` APPROVED 분기에서 spawn.

Note:
    rclpy executor는 기본 MutuallyExclusiveCallbackGroup이라 service handler 둘
    (``_on_start_sampling``, ``_on_user_decision``)은 직렬 실행된다. 그래도 worker
    thread는 별도 OS thread이므로 ``self._state``는 lock 없이 읽을 수 없다.
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
# 노드 코드에서는 상대 토픽명, cross-node client는 절대 경로 — owner 노드의
# 사설 namespace로 풀리는 정식 경로를 명시한다
VISION_BOARD_STATE_TOPIC = "vision/board_state"
VISION_RECEIVE_TIMEOUT_SEC = 3.0

# ~/...prefix는 노드명(main_controller) 하위로 풀려 /main_controller/ui_status가 된다
UI_STATUS_TOPIC = "~/ui_status"

# audit 토픽 — UI에는 노출하지 않고 game_logger 단독 구독. depth=10은 late-join
# logger도 최근 N 이벤트를 받아 갈 수 있게 함
GAME_EVENT_TOPIC = "~/game_event"

# FSM 문자열 → UIStatus.STATE_* uint8 매핑. 신규 상태 추가 시 동기화 필요
_STATE_NAME_TO_UINT = {
    "IDLE": UIStatus.STATE_IDLE,
    "SAMPLING": UIStatus.STATE_SAMPLING,
    "WAIT_DECISION": UIStatus.STATE_WAIT_DECISION,
    "RUNNING": UIStatus.STATE_RUNNING,
}

# stockfish 노드가 사설 namespace ``~/StockfishMove``로 등록 → 절대 경로
# /chess_ai_node/StockfishMove. reset_chess_state도 같은 패턴
STOCKFISH_SERVICE_NAME = "/chess_ai_node/StockfishMove"
SERVICE_TIMEOUT_SEC = 20.0
RESET_CHESS_STATE_SERVICE_NAME = "/chess_ai_node/reset_chess_state"

ROBOT_ACTION_NAME = "move_chess_piece"
ROBOT_ACTION_SEND_TIMEOUT_SEC = 10.0
ROBOT_ACTION_RESULT_TIMEOUT_SEC = 180.0

GAME_OVER_TEXT = "게임 종료"
# =========================================================


def now_iso_ms() -> str:
    """현재 시각을 millisecond 정밀도 ISO-8601 문자열로 반환한다.

    ``job_id`` 발급과 timestamp 로깅에 사용한다.
    """
    return datetime.now().isoformat(timespec="milliseconds")


class MainController(Node):
    """체스 워크플로 오케스트레이터.

    Services:
        ~/start_sampling (std_srvs/srv/Trigger): IDLE → SAMPLING 전이 트리거.
            IDLE이 아니면 ``success=False, message="busy: state=<state>"``로 거부.
        ~/user_decision (chess_ai_interfaces/srv/UserDecision): WAIT_DECISION 결정 처리.
            ``DECISION_APPROVED`` → RUNNING (worker spawn),
            ``DECISION_RECHECKED`` → WAIT_DECISION 유지 + ``corrected_board``로 갱신,
            ``DECISION_GAME_OVER`` → IDLE + KIND_GAME_END 이벤트.

    Subscribes:
        vision/board_state (BoardState): RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1).
            최신값 캐시 — SAMPLING 단계 스냅샷 + RUNNING 단계 live fallback에 사용.

    Publishes:
        ~/ui_status (UIStatus): FSM 전이·verification/working/ai_suggested_move 업데이트
            직후 latched publish. 페이지 로드 직후 UI 동기화에 활용.
        ~/game_event (GameEvent): game_logger 단독 구독. UI 비노출.

    Concurrency:
        - ``self._state_lock`` (mutex): ``self._state``, ``self._job_id``,
          UIStatus tracking 필드, ``self._current_game_id``를 보호한다.
        - ``self._latest_board_state_lock`` (mutex): board_state 캐시 보호.
        - ``self._board_state_received_event`` (threading.Event): subscriber가 1회 이상
          메시지를 받았는지 worker thread에 신호.
        - 2개의 daemon worker thread가 rclpy executor와 병행 동작.

    Warning:
        ``rclpy.spin``은 기본 single-threaded MutuallyExclusive executor이므로 service
        handler 두 개는 직렬 실행되지만, worker thread는 OS thread이므로 ``self._state``
        등을 lock 없이 읽으면 race가 발생한다. 모든 worker 진입·종료에서 ``_state_lock``을
        잡을 것.
    """

    def __init__(self):
        super().__init__("main_controller")

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

        # UIStatus QoS는 board_state와 동일 — 페이지 로드 직후 latched 최신 상태 전달
        self.ui_status_pub = self.create_publisher(
            UIStatus,
            UI_STATUS_TOPIC,
            board_state_qos,
        )

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

        # 같은 default callback group이라 _on_start_sampling과 직렬 실행 — FSM 전이 race-free
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
        # FSM 전이 또는 작업 진행 시점에 갱신 후 _publish_ui_status() 호출.
        self._verification = False
        self._working = False
        self._ai_suggested_move = ""
        self._final_board: dict = {}

        # 게임 라이프사이클: start_sampling 시 신규 발급, GAME_OVER 또는 빈 best_move
        # (체크메이트)에서 리셋. 빈 문자열 = 진행 중 게임 없음.
        self._current_game_id: str = ""

        # IDLE 상태 latched publish — 페이지 로드 직후 UI 동기화
        self._publish_ui_status()

        self.get_logger().info(
            "MainController ready. Services: /main_controller/start_sampling (Trigger), "
            "/main_controller/user_decision (UserDecision)."
        )

    def _on_board_state(self, msg: BoardState) -> None:
        """``vision/board_state`` subscriber callback — 최신값을 캐시한다.

        rclpy executor 스레드에서 실행되며, worker thread는 ``_wait_for_board_state``로
        캐시된 값을 소비한다. squares·pieces 배열 길이가 다르면 데이터 무결성 위반으로
        discard 한다.
        """
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
        subscriber에도 전달되므로 일반적으로 sub-second에 끝난다. 이후 호출은 즉시
        반환되며 (Event가 set 유지) 가장 최근 캐시된 메시지를 반환한다.

        Args:
            timeout_sec (float): 첫 수신 대기 timeout(초).

        Returns:
            dict[str, str]: square → piece 매핑 (캐시의 얕은 copy).

        Raises:
            TimeoutError: ``timeout_sec`` 내에 메시지를 받지 못함 (vision 노드 미실행
                또는 토픽 바인딩 실패).
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
        """현재 tracking 필드를 스냅샷해 ``UIStatus``를 발행한다.

        모든 FSM 전이와 verification·working·ai_suggested_move·final_board 갱신 직후
        호출한다. ``_state_lock``은 read 동안만 잡고 ``publish()`` 자체는 락 밖에서
        실행한다 (rclpy publisher는 thread-safe).

        Note:
            ``_STATE_NAME_TO_UINT`` 매핑 누락 시 IDLE로 fallback 한 뒤 ERROR 로그를
            남긴다 (worker thread가 죽지 않도록).
        """
        msg = UIStatus()
        stamp = self.get_clock().now().to_msg()
        msg.header.stamp = stamp
        msg.header.frame_id = ""

        with self._state_lock:
            state_name = self._state
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
        """``GameEvent``를 audit 토픽으로 발행한다.

        Args:
            kind (int): ``GameEvent.KIND_*`` 상수.
            game_id (str): 게임 식별자 (KIND_GAME_START 시 발급).
            job_id (str): SAMPLING 사이클 식별자.
            uci (str): KIND_AI_MOVE 시 UCI move 문자열.
            fen (str): KIND_AI_MOVE 시 post-move FEN.
            result (str): KIND_GAME_END 시 ``"resign"``/``"checkmate"`` 등.
        """
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
        """stockfish의 게임 상태(dict_memory + castling_rights)를 리셋한다.

        ``reset_chess_state`` service가 2초 내 가용하지 않으면 warn 로그만 남기고
        skip — 워크플로 진행을 막지 않는다.

        Args:
            job_id (str): 호출 컨텍스트 표시용 (현재는 로그에 사용하지 않음).

        Warning:
            ``wait_for_service(2.0)``는 메인 스레드를 최대 2초 블로킹한다. service
            handler 안에서 호출되므로 그 동안 새 service 호출이 처리되지 않는다.
        """
        if self.reset_client.wait_for_service(timeout_sec=2.0):
            self.reset_client.call_async(Trigger.Request())
        else:
            self.get_logger().warn("reset_chess_state service unavailable; skipping chess state reset.")

    def _on_start_sampling(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """``~/start_sampling`` (Trigger) handler — IDLE → SAMPLING.

        IDLE이 아니면 ``success=False, message="busy: state=<state>"``로 거부.
        IDLE이면 새 ``job_id`` 발급, tracking 필드 초기화, KIND_GAME_START 이벤트
        (신규 게임 한정) 발행, board sampling worker thread spawn.

        Returns:
            Trigger.Response: ``success=True, message="sampling started"`` (정상) 또는
                ``success=False, message="busy: state=<state>"`` (거부).

        Note:
            ``_current_game_id``가 비어 있을 때만 신규 game_id를 발급한다. 같은 게임의
            다음 사용자 수 사이클은 같은 game_id를 유지한 채 새 job_id만 받는다.
        """
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
        """SAMPLING worker (daemon) — 보드 상태를 캡처해 사용자 검증 단계로 전이.

        ``VISION_RECEIVE_TIMEOUT_SEC`` 내에 latched ``vision/board_state``를 수신한 뒤
        ``self._final_board``에 저장하고 WAIT_DECISION으로 전이한다. 재샘플링이나 투표는
        없다 — vision 노드가 single source of truth.

        Args:
            job_id (str): SAMPLING 진입 시점의 timestamp 식별자.

        Note:
            성공 경로: ``UIStatus`` (verification=True, final_board 포함) +
                ``GameEvent`` (KIND_USER_BOARD_CONFIRMED) 발행.
            예외 경로 (TimeoutError 포함): IDLE로 복귀하고 tracking 필드 초기화.
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
        """``~/user_decision`` (UserDecision) handler — 사용자 결정 처리.

        검증 순서:
            1. ``self._state == "WAIT_DECISION"`` 아니면 거부.
            2. ``self._job_id == request.job_id`` 아니면 stale로 거부.
            3. ``request.decision``이 ``APPROVED`` / ``RECHECKED`` / ``GAME_OVER`` 중
               하나가 아니면 unknown으로 거부.

        ``request.corrected_board`` (BoardState)가 비어있지 않으면 ``self._final_board``를
        그 dict로 교체한다 (APPROVED/RECHECKED 공통).

        Decision별 동작:
            APPROVED: WAIT_DECISION → RUNNING. Stockfish + robot worker spawn.
            RECHECKED: WAIT_DECISION 유지, ``_final_board``만 갱신, UIStatus 재발행.
            GAME_OVER: IDLE 복귀 + KIND_GAME_END(``result="resign"``) 발행.

        Returns:
            UserDecision.Response: ``accepted`` (bool) + ``message`` (str). 거부 사유는
                message에 인코딩된다.
        """
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

        decision = int(request.decision)
        job_id = request.job_id

        if decision == UserDecision.Request.DECISION_APPROVED:
            self.get_logger().info("[UI] APPROVED. start stockfish/robot workflow")
            with self._state_lock:
                # double-check: lock 해제 사이에 외부 전이가 일어났을 수 있음
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
            # 현 UI에는 GAME_OVER 버튼이 배선되어 있지 않음. handler는 향후 "give up"
            # 버튼 추가를 위한 예약 slot — 핸들러 로직은 이미 동작 가능
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
        """RUNNING worker (daemon) — Stockfish 호출 후 robot action을 직렬 실행.

        Args:
            job_id (str): WAIT_DECISION 진입 시점에 발급된 식별자.

        Pipeline:
            1. ``self._final_board`` 스냅샷. 비어 있으면 live ``vision/board_state``로 fallback.
            2. ``StockfishMove`` 호출 (board만 전달, engine 설정은 stockfish parameter).
            3. 빈 ``best_move`` → KIND_GAME_END(``"checkmate"``) 발행 + game_id 리셋 후 종료.
            4. ``robot_action_client``에 goal 전송, 결과를 최대
               ``ROBOT_ACTION_RESULT_TIMEOUT_SEC`` (180 s) 대기.
            5. 성공 시 KIND_AI_MOVE 이벤트 발행 (실제로 둔 수만 audit에 남김).

        Note:
            ``finally`` 블록에서 항상 IDLE로 복귀하고 working/verification을 False로
            되돌린다. 호출 도중 발생하는 모든 예외는 ERROR 로그로 squash 된다.
        """
        try:
            with self._state_lock:
                board_dict = dict(self._final_board)
                game_id = self._current_game_id

            if not board_dict:
                board_dict = self._wait_for_board_state(VISION_RECEIVE_TIMEOUT_SEC)
                self.get_logger().info("[workflow] using live board_state fallback.")
            else:
                self.get_logger().info("[workflow] using cached final_board.")

            best_move, fen = self._call_stockfish(board_dict)
            if not best_move:
                # Stockfish가 빈 best_move를 반환 = 체크메이트/스테일메이트
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
        """``StockfishMove`` service를 동기적으로 호출한다.

        engine 설정값(depth/skill_level/turn)은 stockfish 노드의 ROS2 parameter 단일
        경로로 일원화되어 있어 request에는 ``pieces_data``만 채운다. polling은 50 ms
        sleep으로 future를 대기.

        Args:
            board_dict (dict[str, str]): square → piece code 매핑.

        Returns:
            tuple[str, str]: ``(best_move, fen)``. service 미가용·타임아웃·실패 또는 빈
                best_move 시 ``("", "")``. ``fen``은 적용 후 보드의 FEN — game_logger
                audit 입력으로 사용된다.
        """
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
        """``move_chess_piece`` action에 goal을 보내고 결과까지 동기 대기.

        2단계 timeout:
            1. ``ROBOT_ACTION_SEND_TIMEOUT_SEC`` (10 s) — server 준비 + goal accept.
            2. ``ROBOT_ACTION_RESULT_TIMEOUT_SEC`` (180 s) — 실제 동작 완료.

        Args:
            best_move (str): UCI move (e.g. ``"e2e4"``).
            board_dict (dict[str, str]): 현재 보드 (action의 ``pieces_dict``로 전달).

        Returns:
            bool: action result의 ``success`` 값. 어느 단계에서든 실패/타임아웃이면
                ``False``.
        """
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
    """Entry point: ``MainController`` 생성 후 ``rclpy.spin``.

    SIGINT는 ``KeyboardInterrupt``로 받아 정상 종료한다. ``rclpy.shutdown()``은
    ``rclpy.ok()`` 가드 후 호출 — supervisor가 이미 shutdown한 context를 재호출하면
    ``RCLError``가 발생한다.
    """
    rclpy.init(args=args)
    node = MainController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
