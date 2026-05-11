"""GameLoggerNode — append-only SQLite 로거 노드 (entry point: ``ros2 run chess_ai gamelogger``).

역할:
    Phase 5 sub-phase E. Firebase가 담당하던 audit trail (Hard Rule #6: append-only
    game/event logs)을 LAN-local SQLite DB로 이전. main_controller의 GameEvent
    토픽 + UIStatus 토픽 + vision board_state 토픽을 구독해서 영속화.

ROS2 Interfaces:
    Subscribers (모두 RELIABLE + TRANSIENT_LOCAL):
        - ``/main_controller/game_event``  (chess_ai_interfaces/msg/GameEvent), depth=10
        - ``/main_controller/ui_status``   (chess_ai_interfaces/msg/UIStatus),  depth=1
        - ``/vision/board_state``          (chess_ai_interfaces/msg/BoardState), depth=1

DB:
    Path:
        - 환경변수 ``CHESS_AI_LOG_DB_PATH`` 우선.
        - 기본 ``~/.local/share/cobot2_chess_ai/game_log.db`` (XDG_DATA_HOME 컨벤션).
        - 디렉토리 부재 시 ``os.makedirs(exist_ok=True)``.
    PRAGMA: ``journal_mode=WAL``, ``synchronous=NORMAL``, ``foreign_keys=ON``.
    Append-only: SQLite TRIGGER로 UPDATE/DELETE 차단 (Hard Rule #6 스키마 레벨 보장).

Schema:
    games          (game_id PK, started_at)            — INSERT-only.
    game_results   (game_id PK FK, ended_at, result)   — INSERT-only, 게임당 1행.
    moves          (id PK, game_id FK, ply, uci, side, fen, ts_ros) — INSERT-only.
    events         (id PK, ts_ros, ts_wall, game_id, kind, payload_json) — INSERT-only.

실패 모드:
    - DB open / DDL 실패 → ``RuntimeError`` (Rule 7 fail-loud, launch respawn).
    - 런타임 INSERT 실패 → ``ERROR`` 로그 + 카운터 증가, 게임은 계속 (audit gap만 발생).
"""

import json
import os
import sqlite3
import threading
from datetime import datetime
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from chess_ai_interfaces.msg import BoardState, GameEvent, UIStatus


DEFAULT_DB_PATH = os.path.expanduser(
    os.getenv(
        "CHESS_AI_LOG_DB_PATH",
        "~/.local/share/cobot2_chess_ai/game_log.db",
    )
)


# ============================================================================
# SQLite 스키마 — append-only를 TRIGGER로 강제 (Hard Rule #6 스키마 레벨 보장).
# ============================================================================
DDL_SCRIPT = """
PRAGMA journal_mode = WAL;
PRAGMA synchronous = NORMAL;
PRAGMA foreign_keys = ON;

CREATE TABLE IF NOT EXISTS games (
    game_id    TEXT PRIMARY KEY,
    started_at TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS game_results (
    game_id  TEXT PRIMARY KEY REFERENCES games(game_id),
    ended_at TEXT NOT NULL,
    result   TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS moves (
    id      INTEGER PRIMARY KEY AUTOINCREMENT,
    game_id TEXT NOT NULL REFERENCES games(game_id),
    ply     INTEGER NOT NULL,
    uci     TEXT NOT NULL,
    side    TEXT NOT NULL CHECK(side IN ('W', 'B')),
    fen     TEXT,
    ts_ros  REAL NOT NULL
);

CREATE TABLE IF NOT EXISTS events (
    id           INTEGER PRIMARY KEY AUTOINCREMENT,
    ts_ros       REAL NOT NULL,
    ts_wall      TEXT NOT NULL,
    game_id      TEXT,
    kind         TEXT NOT NULL,
    payload_json TEXT NOT NULL DEFAULT '{}'
);

CREATE INDEX IF NOT EXISTS idx_events_game_id ON events(game_id);
CREATE INDEX IF NOT EXISTS idx_moves_game_id  ON moves(game_id);

CREATE TRIGGER IF NOT EXISTS no_update_games BEFORE UPDATE ON games
BEGIN SELECT RAISE(ABORT, 'append-only: games UPDATE forbidden'); END;
CREATE TRIGGER IF NOT EXISTS no_delete_games BEFORE DELETE ON games
BEGIN SELECT RAISE(ABORT, 'append-only: games DELETE forbidden'); END;

CREATE TRIGGER IF NOT EXISTS no_update_game_results BEFORE UPDATE ON game_results
BEGIN SELECT RAISE(ABORT, 'append-only: game_results UPDATE forbidden'); END;
CREATE TRIGGER IF NOT EXISTS no_delete_game_results BEFORE DELETE ON game_results
BEGIN SELECT RAISE(ABORT, 'append-only: game_results DELETE forbidden'); END;

CREATE TRIGGER IF NOT EXISTS no_update_moves BEFORE UPDATE ON moves
BEGIN SELECT RAISE(ABORT, 'append-only: moves UPDATE forbidden'); END;
CREATE TRIGGER IF NOT EXISTS no_delete_moves BEFORE DELETE ON moves
BEGIN SELECT RAISE(ABORT, 'append-only: moves DELETE forbidden'); END;

CREATE TRIGGER IF NOT EXISTS no_update_events BEFORE UPDATE ON events
BEGIN SELECT RAISE(ABORT, 'append-only: events UPDATE forbidden'); END;
CREATE TRIGGER IF NOT EXISTS no_delete_events BEFORE DELETE ON events
BEGIN SELECT RAISE(ABORT, 'append-only: events DELETE forbidden'); END;
"""


_KIND_NAMES = {
    GameEvent.KIND_GAME_START: "GAME_START",
    GameEvent.KIND_GAME_END: "GAME_END",
    GameEvent.KIND_AI_MOVE: "AI_MOVE",
    GameEvent.KIND_USER_BOARD_CONFIRMED: "USER_BOARD_CONFIRMED",
}


def _now_iso_ms() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


class GameLoggerNode(Node):
    """append-only SQLite audit log을 호스팅하는 ROS2 노드."""

    def __init__(self, db_path: str = DEFAULT_DB_PATH):
        super().__init__("game_logger")

        self.db_path = db_path
        self._db_lock = threading.Lock()
        self._write_failures = 0
        self._ply_by_game: dict[str, int] = {}

        self._open_db()

        latched_qos_depth1 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        latched_qos_depth10 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            GameEvent,
            "/main_controller/game_event",
            self._on_game_event,
            latched_qos_depth10,
        )
        self.create_subscription(
            UIStatus,
            "/main_controller/ui_status",
            self._on_ui_status,
            latched_qos_depth1,
        )
        self.create_subscription(
            BoardState,
            "/vision/board_state",
            self._on_board_state,
            latched_qos_depth1,
        )

        self.get_logger().info(
            f"GameLogger ready. DB={self.db_path}. Subscribed to game_event, "
            f"ui_status, board_state."
        )

    def _open_db(self) -> None:
        try:
            os.makedirs(os.path.dirname(self.db_path) or ".", exist_ok=True)
        except OSError as e:
            raise RuntimeError(f"Cannot create DB directory: {e}") from e

        try:
            self._conn = sqlite3.connect(
                self.db_path,
                check_same_thread=False,
                isolation_level=None,
            )
        except sqlite3.Error as e:
            raise RuntimeError(f"Cannot open SQLite DB at {self.db_path}: {e}") from e

        try:
            self._conn.executescript(DDL_SCRIPT)
        except sqlite3.Error as e:
            raise RuntimeError(f"DDL failed on {self.db_path}: {e}") from e

    def _execute_safe(self, sql: str, params: tuple, ctx: str) -> None:
        with self._db_lock:
            try:
                self._conn.execute(sql, params)
            except sqlite3.Error as e:
                self._write_failures += 1
                self.get_logger().error(
                    f"DB write failed ({ctx}): {e} — total_failures={self._write_failures}"
                )

    def _ros_time_seconds(self, header_stamp) -> float:
        return float(header_stamp.sec) + float(header_stamp.nanosec) * 1e-9

    def _on_game_event(self, msg: GameEvent) -> None:
        kind_name = _KIND_NAMES.get(msg.kind, f"UNKNOWN({msg.kind})")
        ts_ros = self._ros_time_seconds(msg.header.stamp)
        ts_wall = _now_iso_ms()

        if msg.kind == GameEvent.KIND_GAME_START:
            if not msg.game_id:
                self.get_logger().warn("GAME_START with empty game_id — dropped.")
                return
            self._execute_safe(
                "INSERT OR IGNORE INTO games (game_id, started_at) VALUES (?, ?)",
                (msg.game_id, ts_wall),
                ctx=f"games INSERT game_id={msg.game_id}",
            )
            self._ply_by_game[msg.game_id] = 0

        elif msg.kind == GameEvent.KIND_GAME_END:
            if not msg.game_id:
                self.get_logger().warn("GAME_END with empty game_id — dropped.")
                return
            self._execute_safe(
                "INSERT OR IGNORE INTO game_results (game_id, ended_at, result) "
                "VALUES (?, ?, ?)",
                (msg.game_id, ts_wall, msg.result or "unknown"),
                ctx=f"game_results INSERT game_id={msg.game_id}",
            )
            self._ply_by_game.pop(msg.game_id, None)

        elif msg.kind == GameEvent.KIND_AI_MOVE:
            if not msg.game_id or not msg.uci:
                self.get_logger().warn(
                    f"AI_MOVE missing fields (game_id={msg.game_id!r}, "
                    f"uci={msg.uci!r}) — dropped."
                )
                return
            ply = self._ply_by_game.get(msg.game_id, 0) + 1
            self._ply_by_game[msg.game_id] = ply
            side = self._infer_side_from_fen(msg.fen)
            self._execute_safe(
                "INSERT INTO moves (game_id, ply, uci, side, fen, ts_ros) "
                "VALUES (?, ?, ?, ?, ?, ?)",
                (msg.game_id, ply, msg.uci, side, msg.fen or None, ts_ros),
                ctx=f"moves INSERT game_id={msg.game_id} ply={ply}",
            )

        payload = {
            "kind": kind_name,
            "game_id": msg.game_id,
            "job_id": msg.job_id,
            "uci": msg.uci,
            "fen": msg.fen,
            "result": msg.result,
        }
        self._execute_safe(
            "INSERT INTO events (ts_ros, ts_wall, game_id, kind, payload_json) "
            "VALUES (?, ?, ?, ?, ?)",
            (ts_ros, ts_wall, msg.game_id or None, f"game_event:{kind_name}", json.dumps(payload)),
            ctx="events INSERT (game_event)",
        )

    def _on_ui_status(self, msg: UIStatus) -> None:
        ts_ros = self._ros_time_seconds(msg.header.stamp)
        ts_wall = _now_iso_ms()
        payload = {
            "controller_state": int(msg.controller_state),
            "verification": bool(msg.verification),
            "working": bool(msg.working),
            "ai_suggested_move": msg.ai_suggested_move,
            "job_id": msg.job_id,
            "final_board_count": int(msg.final_board.piece_count),
        }
        self._execute_safe(
            "INSERT INTO events (ts_ros, ts_wall, game_id, kind, payload_json) "
            "VALUES (?, ?, ?, ?, ?)",
            (ts_ros, ts_wall, msg.job_id or None, "ui_status", json.dumps(payload)),
            ctx="events INSERT (ui_status)",
        )

    def _on_board_state(self, msg: BoardState) -> None:
        if len(msg.squares) != len(msg.pieces):
            self.get_logger().warn(
                f"board_state arrays mismatch (squares={len(msg.squares)}, "
                f"pieces={len(msg.pieces)}) — dropped."
            )
            return
        ts_ros = self._ros_time_seconds(msg.header.stamp)
        ts_wall = _now_iso_ms()
        board = dict(zip(msg.squares, msg.pieces))
        payload = {
            "frame_id": msg.header.frame_id,
            "piece_count": int(msg.piece_count),
            "board": board,
        }
        self._execute_safe(
            "INSERT INTO events (ts_ros, ts_wall, game_id, kind, payload_json) "
            "VALUES (?, ?, ?, ?, ?)",
            (ts_ros, ts_wall, None, "board_state", json.dumps(payload)),
            ctx="events INSERT (board_state)",
        )

    @staticmethod
    def _infer_side_from_fen(fen: str) -> str:
        # FEN format: "<placement> <turn> <castling> <ep> <halfmove> <fullmove>".
        # AI 수가 두어진 직후의 FEN이라 turn 필드는 "다음 차례" 색상 — AI가 둔 색상은 그 반대.
        if not fen:
            return "W"
        try:
            after_move_turn = fen.split()[1]
        except IndexError:
            return "W"
        return "B" if after_move_turn == "w" else "W"

    def cleanup(self) -> None:
        with self._db_lock:
            try:
                self._conn.close()
            except sqlite3.Error:
                pass


def main(args=None):
    rclpy.init(args=args)
    node: Optional[GameLoggerNode] = None
    try:
        node = GameLoggerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.cleanup()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
