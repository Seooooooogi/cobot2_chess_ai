import json
import time
import threading
from collections import Counter
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient

import firebase_admin
from firebase_admin import credentials, db

from cobot2_interfaces.srv import StockfishMove
from cobot2_interfaces.action import MoveChessPiece


# ================= [설정 상수: 클래스 밖] =================
FIREBASE_SERVICE_ACCOUNT_JSON = "/home/kyb/cobot_ws/src/cobot2/config/kybfirebase.json"
FIREBASE_DB_URL = "https://chess-43355-default-rtdb.asia-southeast1.firebasedatabase.app"

BOARD_STATE_PATH = "chess/board_state"
UI_CONTROL_PATH = "chess/ui_control"
CHESS_SYSTEM_PATH = "chess/chess_system"

SAMPLE_COUNT = 5
SAMPLE_INTERVAL_SEC = 1.0

VOICE_COMMAND_TOPIC = "voice_command"
VOICE_STATUS_TOPIC = "voice_status"
VOICE_UI_STATUS_TOPIC = "voice_ui_status"

PASS_COMMAND = "pass"
WAKE_UP_SIGNAL = "WAKE_UP"

STOCKFISH_SERVICE_NAME = "StockfishMove"
SERVICE_TIMEOUT_SEC = 20.0

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


class BoardStateSampler:
    def __init__(self, fb: FirebaseClient, board_state_path: str, sample_count: int, sample_interval_sec: float):
        self.fb = fb
        self.board_state_path = board_state_path
        self.sample_count = int(sample_count)
        self.sample_interval_sec = float(sample_interval_sec)

    @staticmethod
    def _majority_vote_dict(dict_list):
        all_keys = set()
        for d in dict_list:
            all_keys.update(d.keys())

        final_dict = {}
        for k in sorted(all_keys):
            values = [d[k] for d in dict_list if k in d]
            if not values:
                continue

            counter = Counter(values)
            max_count = max(counter.values())
            candidates = [v for v, c in counter.items() if c == max_count]

            if len(candidates) == 1:
                final_dict[k] = candidates[0]
            else:
                for d in reversed(dict_list):
                    if k in d and d[k] in candidates:
                        final_dict[k] = d[k]
                        break

        return final_dict

    def sample_final_dict(self, progress_cb=None) -> dict:
        samples = []
        for i in range(self.sample_count):
            d = self.fb.get_board_dict(self.board_state_path)
            samples.append(d)

            if callable(progress_cb):
                progress_cb(i + 1, self.sample_count, len(d))

            if i < self.sample_count - 1:
                time.sleep(self.sample_interval_sec)

        return BoardStateSampler._majority_vote_dict(samples)


class MainController(Node):
    def __init__(self):
        super().__init__("main_controller")

        self.fb = FirebaseClient(FIREBASE_SERVICE_ACCOUNT_JSON, FIREBASE_DB_URL)
        self.sampler = BoardStateSampler(self.fb, BOARD_STATE_PATH, SAMPLE_COUNT, SAMPLE_INTERVAL_SEC)

        self.ai_client = self.create_client(StockfishMove, STOCKFISH_SERVICE_NAME)
        self.robot_action_client = ActionClient(self, MoveChessPiece, ROBOT_ACTION_NAME)

        self.status_pub = self.create_publisher(String, VOICE_STATUS_TOPIC, 10)
        self.cmd_sub = self.create_subscription(String, VOICE_COMMAND_TOPIC, self._on_voice_command, 10)
        self.voice_ui_sub = self.create_subscription(String, VOICE_UI_STATUS_TOPIC, self._on_voice_ui_status, 10)

        self._state_lock = threading.Lock()
        self._state = "IDLE"
        self._job_id = ""

        self.timer = self.create_timer(DECISION_POLL_SEC, self._poll_ui_decision)

        self.get_logger().info("MainController ready. Waiting for voice_command='pass'.")

    def _on_voice_ui_status(self, msg: String):
        text = (msg.data or "").strip()
        if not text:
            return
        try:
            self.fb.update_ui_control(UI_CONTROL_PATH, {
                "voice_message": text,
                "voice_updated_at": datetime.now().isoformat(),
            })
        except Exception as e:
            self.get_logger().warn(f"Failed to update voice_message: {e}")

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

    def _on_voice_command(self, msg: String):
        cmd = (msg.data or "").strip().lower()
        if cmd != PASS_COMMAND:
            return

        with self._state_lock:
            if self._state != "IDLE":
                self.get_logger().warn(f"Ignoring trigger (state={self._state}).")
                return
            self._state = "SAMPLING"
            self._job_id = now_iso_ms()
            job_id = self._job_id

        self.get_logger().info(f"[PASS] received. job_id={job_id}")
        self._reset_ui_for_new_job(job_id)

        t = threading.Thread(target=self._job_make_and_publish_board, args=(job_id,), daemon=True)
        t.start()

    def _job_make_and_publish_board(self, job_id: str):
        try:
            self.get_logger().info("[SAMPLING] start: read board_state 5 times (1s interval)")

            def _progress(i, total, piece_count):
                self.get_logger().info(f"[SAMPLING] {i}/{total} (pieces={piece_count})")

            final_dict = self.sampler.sample_final_dict(progress_cb=_progress)

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
            self._publish_wake_up()

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
                board_dict = self.fb.get_board_dict(BOARD_STATE_PATH)
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

            self._publish_wake_up()
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

    def _publish_wake_up(self):
        try:
            wake = String()
            wake.data = WAKE_UP_SIGNAL
            self.status_pub.publish(wake)
        except Exception as e:
            self.get_logger().error(f"Failed to publish WAKE_UP: {e}")


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
