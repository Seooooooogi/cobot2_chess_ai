"""MainController node — chess workflow orchestrator (entry point: ``ros2 run cobot2 main``).

Role:
    Coordinates the end-to-end chess turn:
    sample board state → user verification (Firebase UI) → Stockfish best move → robot action.
    State machine: ``IDLE`` → ``SAMPLING`` → ``WAIT_DECISION`` → ``RUNNING`` → ``IDLE``
    (transitions guarded by ``self._state_lock``).

ROS2 Interfaces:
    Service: ``~/start_sampling`` (std_srvs/Trigger) — state-change trigger; IDLE→SAMPLING.
             Resolves to /main_controller/start_sampling.
    Subscriber: Topic ``vision/board_state`` (cobot2_interfaces/msg/BoardState) —
                RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1). Cached latest is used as the
                board snapshot for SAMPLING and as the live fallback in RUNNING.
    Client: Service ``StockfishMove``      (cobot2_interfaces/StockfishMove)
    Client: Action  ``move_chess_piece``  (cobot2_interfaces/MoveChessPiece)

Timer & Threads:
    - Timer ``_poll_ui_decision`` — 0.2 s, polls Firebase ``ui_control`` for APPROVED / RE-CHECKED
    - Daemon thread ``_job_make_and_publish_board`` — receives the latched ``vision/board_state``
      message (TRANSIENT_LOCAL) within ``VISION_RECEIVE_TIMEOUT_SEC``, no resampling/voting
      (single source of truth from vision node), spawned in ``_on_start_sampling``.
    - Daemon thread ``_job_stockfish_then_robot_then_wakeup`` — service call + action goal, spawned in ``_poll_ui_decision``

External Dependencies:
    - Firebase Realtime DB — read/write ``chess/board_state``, ``chess/ui_control``, ``chess/chess_system``
    - cobot2_interfaces — ``StockfishMove.srv``, ``MoveChessPiece.action``

Issues (Phase 1-1 doc Node 1):
    - M1-3 RESOLVED 2026-05-01: ``FIREBASE_SERVICE_ACCOUNT_JSON`` env-ized via ``FIREBASE_SERVICE_ACCOUNT_PATH`` env var; ``FIREBASE_DB_URL`` via ``FIREBASE_DATABASE_URL`` env var.
    - M1-1 RESOLVED 2026-05-04: ``~/start_sampling`` (Trigger) 로 대체.
    - ~~M1-2: pub/sub QoS 미명시 (Rule 4)~~ **RESOLVED 2026-05-04**: voice 제거로 pub/sub 0건. service/action endpoint에 ``qos_profile_services_default`` / ``qos_profile_action_status_default`` 명시 (line 234-256).
    - M1-4 PARTIAL Phase 5 sub-phase B 2026-05-10: vision→main bus migrated to ROS2 topic
      ``vision/board_state`` (TRANSIENT_LOCAL). UI↔main control flow still on Firebase
      (``ui_control``, ``chess_system``) — closed in sub-phases C/D.
    - MINOR M1-5: workflow threads use ``time.sleep`` polling on Futures inside ``_call_stockfish`` and ``_send_robot_action_and_wait`` — Future callbacks preferred.
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

from cobot2_interfaces.msg import BoardState
from cobot2_interfaces.srv import StockfishMove
from cobot2_interfaces.action import MoveChessPiece


# ================= [설정 상수: 클래스 밖] =================
FIREBASE_SERVICE_ACCOUNT_JSON = os.getenv("FIREBASE_SERVICE_ACCOUNT_PATH")
FIREBASE_DB_URL = os.getenv("FIREBASE_DATABASE_URL", "https://chess-43355-default-rtdb.asia-southeast1.firebasedatabase.app")

BOARD_STATE_PATH = "chess/board_state"
UI_CONTROL_PATH = "chess/ui_control"
CHESS_SYSTEM_PATH = "chess/chess_system"

# Phase 5 sub-phase B: vision→main bus is now ROS2 topic (TRANSIENT_LOCAL).
# Relative topic name (Rule 5); resolves under main_controller's namespace.
VISION_BOARD_STATE_TOPIC = "vision/board_state"
VISION_RECEIVE_TIMEOUT_SEC = 3.0

STOCKFISH_SERVICE_NAME = "StockfishMove"
SERVICE_TIMEOUT_SEC = 20.0
RESET_CHESS_STATE_SERVICE_NAME = "reset_chess_state"

ROBOT_ACTION_NAME = "move_chess_piece"
ROBOT_ACTION_SEND_TIMEOUT_SEC = 10.0
ROBOT_ACTION_RESULT_TIMEOUT_SEC = 180.0

DEFAULT_DEPTH = 15
DEFAULT_DIFFICULTY = 10
DEFAULT_TURN = "w"

DECISION_APPROVED = "APPROVED"
DECISION_RECHECKED = "RE-CHECKED"
DECISION_NONE = "NONE"

CMD_IDLE = "idle"
DECISION_POLL_SEC = 0.2

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

    def get_chess_system_params(self, chess_system_path: str):
        self.init()
        data = db.reference(chess_system_path).get()
        if not isinstance(data, dict):
            return DEFAULT_DEPTH, DEFAULT_DIFFICULTY, DEFAULT_TURN

        depth = data.get("depth", DEFAULT_DEPTH)
        difficulty = data.get("difficulty", DEFAULT_DIFFICULTY)
        turn = data.get("turn", DEFAULT_TURN)

        try:
            depth = int(depth)
        except Exception:
            depth = DEFAULT_DEPTH

        try:
            difficulty = int(difficulty)
        except Exception:
            difficulty = DEFAULT_DIFFICULTY

        if turn not in ["w", "b"]:
            turn = DEFAULT_TURN

        return depth, difficulty, turn


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

        self._state_lock = threading.Lock()
        self._state = "IDLE"
        self._job_id = ""

        self.timer = self.create_timer(DECISION_POLL_SEC, self._poll_ui_decision)

        self.get_logger().info("MainController ready. Service: /main_controller/start_sampling (std_srvs/Trigger).")

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
            job_id = self._job_id

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
            - Writes the captured board back to Firebase ``chess/board_state`` (UI mirror,
              removed in sub-phase E).
            - Sets ``ui_control.verification = True`` and uploads ``final_board``.
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

            self.fb.set_board_state(BOARD_STATE_PATH, final_dict, extra={"source": "main_final_dict"})

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

        except Exception as e:
            self.get_logger().error(f"Failed to make/publish final_dict: {e}")
            with self._state_lock:
                self._state = "IDLE"
                self._job_id = ""

    def _poll_ui_decision(self):
        with self._state_lock:
            if self._state != "WAIT_DECISION":
                return
            job_id = self._job_id

        try:
            ui = self.fb.get_ui_control(UI_CONTROL_PATH)
            decision = (ui.get("user_decision") or "").strip()
            ui_job_id = (ui.get("job_id") or "").strip()

            if ui_job_id and ui_job_id != job_id:
                return

            if decision == DECISION_RECHECKED:
                corrected = ui.get("corrected_board")
                if isinstance(corrected, dict):
                    self.get_logger().info("[UI] corrected_board received. updating final_board")
                    self.fb.set_board_state(BOARD_STATE_PATH, corrected, extra={"source": "manual_corrected"})
                    self.fb.update_ui_control(UI_CONTROL_PATH, {
                        "final_board": corrected,
                        "corrected_board": None,
                        "user_decision": DECISION_NONE,
                        "timestamp": datetime.now().isoformat(),
                        "job_id": job_id,
                    })
                else:
                    self.fb.update_ui_control(UI_CONTROL_PATH, {
                        "user_decision": DECISION_NONE,
                        "timestamp": datetime.now().isoformat(),
                        "job_id": job_id,
                    })
                return

            if decision != DECISION_APPROVED:
                return

            self.get_logger().info("[UI] APPROVED received. start stockfish/robot workflow")

            self.fb.update_ui_control(UI_CONTROL_PATH, {
                "user_decision": DECISION_NONE,
                "verification": False,
                "working": True,
            })

            with self._state_lock:
                if self._state != "WAIT_DECISION":
                    return
                self._state = "RUNNING"

            t = threading.Thread(target=self._job_stockfish_then_robot_then_wakeup, args=(job_id,), daemon=True)
            t.start()

        except Exception as e:
            self.get_logger().error(f"Decision polling error: {e}")

    def _job_stockfish_then_robot_then_wakeup(self, job_id: str):
        """Worker thread (daemon) — call Stockfish and send robot action.

        Args:
            job_id: str — timestamp-based identifier set when WAIT_DECISION was entered.

        Side Effects:
            - Reads Firebase ``ui_control`` to choose the input board (``corrected_board`` if present
              and non-empty, else ``final_board``, else live ``board_state``).
            - Calls service ``StockfishMove`` with the board + ``depth``/``difficulty``/``turn`` from
              ``chess_system``. On empty ``best_move`` writes ``ai_suggested_move = "게임 종료"`` to
              ``ui_control`` and returns (game-over branch).
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

            depth, difficulty, turn = self.fb.get_chess_system_params(CHESS_SYSTEM_PATH)

            best_move = self._call_stockfish(board_dict, depth, difficulty, turn)
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
                return

            self.fb.update_ui_control(UI_CONTROL_PATH, {
                "ai_suggested_move": best_move,
                "ai_updated_at": datetime.now().isoformat(),
                "command": CMD_IDLE,
                "job_id": job_id,
            })

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

    def _call_stockfish(self, board_dict: dict, depth: int, difficulty: int, turn: str) -> str:
        if not self.ai_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SEC):
            self.get_logger().error("Stockfish service not available.")
            return ""

        req = StockfishMove.Request()
        req.pieces_data = json.dumps(board_dict)
        req.depth = int(depth)
        req.skill_level = int(difficulty)
        req.turn = str(turn)

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
