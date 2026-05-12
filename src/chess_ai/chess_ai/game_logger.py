"""체스 게임 audit log를 append-only SQLite로 영속화하는 노드.

Entry point: ``ros2 run chess_ai gamelogger``.

main_controller의 GameEvent·UIStatus, vision의 BoardState topic을 구독해 LAN-local
SQLite DB로 기록한다. append-only 보장은 SQLite ``TRIGGER`` (UPDATE/DELETE에
``RAISE(ABORT)``)로 스키마 레벨에서 강제된다.

Subscribes (모두 RELIABLE + TRANSIENT_LOCAL):
    /main_controller/game_event (chess_ai_interfaces/msg/GameEvent): depth=10.
    /main_controller/ui_status (chess_ai_interfaces/msg/UIStatus): depth=1.
    /vision/board_state (chess_ai_interfaces/msg/BoardState): depth=1.

Environment:
    CHESS_AI_LOG_DB_PATH (str): SQLite DB 경로. 기본 ``~/.local/share/cobot2_chess_ai/game_log.db``.

Schema:
    games (game_id PK, started_at)
    game_results (game_id PK FK→games, ended_at, result)
    moves (id PK, game_id FK→games, ply, uci, side, fen, ts_ros)
    events (id PK, ts_ros, ts_wall, game_id, kind, payload_json)

Note:
    DB open / DDL 실패는 ``RuntimeError``로 fail-loud — launch supervisor가 노드 기동
    실패를 관찰할 수 있게 한다. 런타임 INSERT 실패는 ERROR 로그 + 카운터 증가로
    swallow 되며, 노드는 계속 살아남고 audit gap만 발생한다.
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
# append-only는 ORM 레벨이 아니라 SQLite TRIGGER로 강제 — 외부 도구가 직접 SQL을 쳐도
# UPDATE/DELETE가 ABORT 되어 audit 무결성이 깨지지 않는다
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
    """현재 시각을 millisecond 정밀도 ISO-8601 문자열로 반환한다."""
    return datetime.now().isoformat(timespec="milliseconds")


class GameLoggerNode(Node):
    """append-only SQLite audit logger.

    Subscribes:
        /main_controller/game_event (chess_ai_interfaces/msg/GameEvent): depth=10.
            게임 라이프사이클(GAME_START/GAME_END) + AI 수(AI_MOVE) + 사용자 보드 확정
            (USER_BOARD_CONFIRMED).
        /main_controller/ui_status (chess_ai_interfaces/msg/UIStatus): depth=1.
            FSM 상태 latched snapshot.
        /vision/board_state (chess_ai_interfaces/msg/BoardState): depth=1.
            카메라 인식 결과 latched snapshot.

    Args:
        db_path (str): SQLite DB 경로. 기본 ``DEFAULT_DB_PATH``
            (``CHESS_AI_LOG_DB_PATH`` env로 오버라이드 가능).

    Raises:
        RuntimeError: DB 디렉토리 생성·``sqlite3.connect``·DDL 실행 어느 단계든 실패하면
            노드 기동 자체를 중단한다 (fail-loud).

    Note:
        다른 노드의 리소스를 흡수하는 단방향 sink이므로 subscription topic 이름을
        절대 경로로 하드코딩한다 (외부 owner의 정식 경로). SQLite connection은
        ``check_same_thread=False`` + ``isolation_level=None`` (autocommit) +
        ``threading.Lock``으로 동시 write를 직렬화한다.

    Warning:
        INSERT 실패는 게임을 계속 진행시키며 ``_write_failures`` 카운터만 증가시킨다.
        DB가 영구 손상되면 이후 모든 audit이 누락되므로 카운터를 모니터링해야 한다.
    """

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
        """DB 파일을 열고 PRAGMA·DDL 스크립트를 실행한다.

        Raises:
            RuntimeError: 디렉토리 생성 / ``sqlite3.connect`` / ``executescript`` 어디서든
                실패 시. launch supervisor가 노드 재기동을 결정하도록 fail-loud로 전파한다.
        """
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
        """``_db_lock`` 보호 하에 INSERT를 시도하고, 실패는 squash 한다.

        autocommit 모드(``isolation_level=None``)이므로 별도 commit 호출 없이 즉시 영속화된다.
        실패 시 ``_write_failures`` 카운터를 증가시키고 ERROR 로그를 남길 뿐, 예외를
        bubble 하지 않는다 (게임 진행을 막지 않기 위함).

        Args:
            sql (str): 실행할 SQL.
            params (tuple): bind parameter.
            ctx (str): 실패 로그에 끼워 넣을 컨텍스트 문자열.
        """
        with self._db_lock:
            try:
                self._conn.execute(sql, params)
            except sqlite3.Error as e:
                self._write_failures += 1
                self.get_logger().error(
                    f"DB write failed ({ctx}): {e} — total_failures={self._write_failures}"
                )

    def _ros_time_seconds(self, header_stamp) -> float:
        """``builtin_interfaces/Time``을 float seconds로 변환한다.

        Args:
            header_stamp: ``sec`` (int), ``nanosec`` (int) 필드를 가진 stamp.

        Returns:
            float: ``sec + nanosec * 1e-9``.
        """
        return float(header_stamp.sec) + float(header_stamp.nanosec) * 1e-9

    def _on_game_event(self, msg: GameEvent) -> None:
        """GameEvent kind별로 적절한 테이블에 분기 INSERT 한다.

        Kind 처리:
            KIND_GAME_START: ``games``에 ``INSERT OR IGNORE``, ``_ply_by_game[game_id]=0``.
            KIND_GAME_END: ``game_results``에 ``INSERT OR IGNORE``, ply counter 제거.
            KIND_AI_MOVE: ply++ 후 ``moves``에 INSERT. ``side``는 FEN turn 필드에서 역추론.

        모든 kind는 추가로 ``events`` 테이블에 ``game_event:<KIND>`` row를 남겨 raw 시퀀스를
        보존한다.

        Args:
            msg (GameEvent): main_controller가 publish한 이벤트. 필수 필드 부재 시
                warn + drop (audit gap만 발생).

        Note:
            ``INSERT OR IGNORE``는 동일 game_id의 GAME_START가 두 번 들어와도 첫 번째만
            영속화한다 — latched topic 특성상 재구독 시 중복 수신이 가능하기 때문.
        """
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
        """UIStatus snapshot을 ``events``에 ``ui_status`` kind로 기록한다.

        ``payload_json``에 controller_state·verification·working·ai_suggested_move·job_id·
        final_board_count를 직렬화한다 — FSM 재구성에 필요한 최소 정보 셋.

        Args:
            msg (UIStatus): main_controller가 발행하는 latched 상태 메시지.
        """
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
        """BoardState를 ``events``에 ``board_state`` kind로 기록한다.

        ``squares``와 ``pieces`` 배열 길이가 다르면 데이터 무결성 위반으로 보고 drop 한다.
        payload는 frame_id, piece_count, ``dict(squares → pieces)``로 직렬화.

        Args:
            msg (BoardState): vision_db가 발행하는 인식 결과.
        """
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
        """AI 수 직후 FEN의 turn 필드에서 AI가 둔 색상을 역추론한다.

        Stockfish 노드는 수가 적용된 후의 보드를 ``다음 차례`` 색상으로 직렬화한다.
        따라서 FEN turn 필드가 ``"w"``이면 AI가 둔 색상은 ``"B"``, ``"b"``이면 ``"W"``.

        Args:
            fen (str): ``"<placement> <turn> <castling> <ep> <halfmove> <fullmove>"`` 형식.
                비어 있거나 형식 오류면 기본값 ``"W"``로 fallback.

        Returns:
            str: ``"W"`` 또는 ``"B"``.
        """
        if not fen:
            return "W"
        try:
            after_move_turn = fen.split()[1]
        except IndexError:
            return "W"
        return "B" if after_move_turn == "w" else "W"

    def cleanup(self) -> None:
        """DB connection을 lock 보호 하에 close 한다.

        close 자체가 실패해도 swallow — 노드 종료 경로에서 예외를 raise 하지 않는다.
        """
        with self._db_lock:
            try:
                self._conn.close()
            except sqlite3.Error:
                pass


def main(args=None):
    """Entry point: ``GameLoggerNode`` 생성 → ``rclpy.spin`` → cleanup.

    SIGINT는 ``KeyboardInterrupt``로 받아 정상 종료한다. ``rclpy.shutdown()``은
    ``rclpy.ok()`` 가드 후 호출 — supervisor가 이미 shutdown한 context를 재호출하면
    ``RCLError``가 발생한다.
    """
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
