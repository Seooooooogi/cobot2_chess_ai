#!/usr/bin/env python3
"""``game_logger`` SQLite audit pipeline의 standalone 시뮬레이션 테스트.

하드웨어·vision·카메라·rosbridge에 의존하지 않는 검증 harness — ROS2 + ``chess_ai`` +
``chess_ai_interfaces``만 source 되어 있으면 동작한다. 다음을 확인한다:

    - ``GameEvent`` / ``UIStatus`` / ``BoardState`` pub-sub 왕복.
    - SQLite append-only TRIGGER가 UPDATE/DELETE를 거부.
    - post-move FEN으로부터 side(W/B) 역추론.
    - 리팩토링 후 회귀 catch.

Usage:
    1. 워크스페이스 source::

        source /home/rokey/cobot2_chess_ai/install/setup.bash

    2. 실행::

        python3 /home/rokey/cobot2_chess_ai/scripts/sim_game_logger.py

스크립트는 임시 SQLite DB를 만들어 시뮬레이션을 돌리고, 각 검사에 대해 PASS/FAIL을
출력한 뒤 종료 시 temp DB를 삭제한다. exit code는 모든 검사 통과 시 0, 아니면 1.

Note:
    본 스크립트는 launch 파일을 띄우지 않는다 — ``GameLoggerNode`` + 합성 publisher
    노드를 직접 인스턴스화해 단일 ``MultiThreadedExecutor``에서 spin 한다.
    QoS는 production 파이프라인과 동일 (RELIABLE + TRANSIENT_LOCAL + KEEP_LAST). publisher와
    subscriber가 동시에 시작되므로 late-join은 검증 범위 밖이며, DDL + write 경로는 검증된다.
"""

from __future__ import annotations

import os
import sqlite3
import sys
import tempfile
import threading
import time
from typing import Any

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Header

from chess_ai_interfaces.msg import BoardState, GameEvent, UIStatus
from chess_ai.game_logger import GameLoggerNode


# ----------------------------------------------------------------------------
# 시나리오 상수: 합성 게임 2개, 각각 AI move 2수
# ----------------------------------------------------------------------------
GAME_IDS = ["sim-game-001", "sim-game-002"]
MOVES_PER_GAME = [
    # (uci, post_move_fen) — FEN turn 필드는 수 적용 직후이므로 다음 차례 색상
    ("e2e4", "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1"),
    ("d2d4", "rnbqkbnr/pppppppp/8/8/3PP3/8/PPP2PPP/RNBQKBNR w KQkq d3 0 2"),
]
EXPECTED_SIDES = ["W", "B"]  # FEN turn 필드 flip에서 역추론


# ============================================================================
# 합성 publisher
# ============================================================================
class SimPublisher(Node):
    """``GameEvent`` / ``UIStatus`` / ``BoardState`` 합성 메시지를 발행하는 테스트 노드.

    Publishes:
        /main_controller/game_event (GameEvent): depth=10, RELIABLE + TRANSIENT_LOCAL.
        /main_controller/ui_status (UIStatus): depth=1, RELIABLE + TRANSIENT_LOCAL.
        /vision/board_state (BoardState): depth=1, RELIABLE + TRANSIENT_LOCAL.

    Note:
        QoS는 production 파이프라인과 동일 — production subscriber인 ``GameLoggerNode``가
        같은 메시지를 받을 수 있도록 보장한다.
    """

    def __init__(self):
        super().__init__("sim_publisher")
        latched_d10 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        latched_d1 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub_event = self.create_publisher(
            GameEvent, "/main_controller/game_event", latched_d10
        )
        self._pub_ui = self.create_publisher(
            UIStatus, "/main_controller/ui_status", latched_d1
        )
        self._pub_board = self.create_publisher(
            BoardState, "/vision/board_state", latched_d1
        )

    def _stamp(self) -> Header:
        """현재 시각을 ``stamp``에 채운 새 ``Header``를 반환한다."""
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        return h

    def emit_game_start(self, game_id: str, job_id: str) -> None:
        """``KIND_GAME_START`` 이벤트를 발행한다."""
        msg = GameEvent()
        msg.header = self._stamp()
        msg.kind = GameEvent.KIND_GAME_START
        msg.game_id = game_id
        msg.job_id = job_id
        self._pub_event.publish(msg)

    def emit_user_board_confirmed(self, game_id: str, job_id: str) -> None:
        """``KIND_USER_BOARD_CONFIRMED`` 이벤트를 발행한다."""
        msg = GameEvent()
        msg.header = self._stamp()
        msg.kind = GameEvent.KIND_USER_BOARD_CONFIRMED
        msg.game_id = game_id
        msg.job_id = job_id
        self._pub_event.publish(msg)

    def emit_ai_move(self, game_id: str, job_id: str, uci: str, fen: str) -> None:
        """``KIND_AI_MOVE`` 이벤트를 발행한다.

        Args:
            game_id (str): 게임 식별자.
            job_id (str): 사이클 식별자.
            uci (str): UCI 수 문자열 (예: ``"e2e4"``).
            fen (str): 수 적용 직후 보드의 FEN. logger가 side 역추론에 사용.
        """
        msg = GameEvent()
        msg.header = self._stamp()
        msg.kind = GameEvent.KIND_AI_MOVE
        msg.game_id = game_id
        msg.job_id = job_id
        msg.uci = uci
        msg.fen = fen
        self._pub_event.publish(msg)

    def emit_game_end(self, game_id: str, result: str) -> None:
        """``KIND_GAME_END`` 이벤트를 발행한다.

        Args:
            game_id (str): 게임 식별자.
            result (str): ``"checkmate"``/``"resign"`` 등 결과 라벨.
        """
        msg = GameEvent()
        msg.header = self._stamp()
        msg.kind = GameEvent.KIND_GAME_END
        msg.game_id = game_id
        msg.result = result
        self._pub_event.publish(msg)

    def emit_ui_status(
        self, controller_state: int, working: bool, verification: bool, job_id: str
    ) -> None:
        """``UIStatus`` snapshot을 발행한다 (final_board는 비워둠).

        Args:
            controller_state (int): ``UIStatus.STATE_*`` 상수.
            working (bool): 작업 진행 여부.
            verification (bool): 사용자 검증 단계 여부.
            job_id (str): 사이클 식별자.
        """
        msg = UIStatus()
        msg.header = self._stamp()
        msg.controller_state = controller_state
        msg.working = working
        msg.verification = verification
        msg.ai_suggested_move = ""
        msg.job_id = job_id
        msg.final_board = BoardState()
        msg.final_board.header = self._stamp()
        msg.final_board.header.frame_id = "chess_board"
        msg.final_board.squares = []
        msg.final_board.pieces = []
        msg.final_board.piece_count = 0
        self._pub_ui.publish(msg)

    def emit_board_state(self, squares: list[str], pieces: list[str]) -> None:
        """``BoardState``를 발행한다.

        Args:
            squares (list[str]): square 이름 리스트 (``"A1"``..``"H8"``).
            pieces (list[str]): piece code 리스트. ``squares``와 길이 일치 필수.
        """
        msg = BoardState()
        msg.header = self._stamp()
        msg.header.frame_id = "chess_board"
        msg.squares = squares
        msg.pieces = pieces
        msg.piece_count = len(squares)
        self._pub_board.publish(msg)


# ============================================================================
# 검증 helper
# ============================================================================
def query(db_path: str, sql: str, params: tuple = ()) -> list[Any]:
    """짧은 connection을 열어 SELECT 결과를 반환한다.

    Args:
        db_path (str): SQLite DB 경로.
        sql (str): SELECT 문.
        params (tuple): bind parameter.

    Returns:
        list[Any]: ``fetchall()`` 결과.
    """
    conn = sqlite3.connect(db_path)
    try:
        return conn.execute(sql, params).fetchall()
    finally:
        conn.close()


def expect(label: str, actual: Any, expected: Any, results: list[bool]) -> None:
    """``actual == expected`` 검사를 수행하고 PASS/FAIL을 출력한다.

    Args:
        label (str): 검사 이름.
        actual (Any): 실제 값.
        expected (Any): 기대값.
        results (list[bool]): 누적 결과 리스트 — 호출마다 append 된다.
    """
    ok = actual == expected
    results.append(ok)
    mark = "PASS" if ok else "FAIL"
    print(f"  [{mark}] {label}: got={actual!r} expected={expected!r}")


def trigger_check(db_path: str, sql: str, params: tuple, label: str, results: list[bool]) -> None:
    """UPDATE/DELETE 문이 append-only TRIGGER에 의해 거부되는지 검증한다.

    SQL을 실행해 ``sqlite3.IntegrityError``의 메시지에 ``"append-only"``가 포함되면 PASS.
    아무 예외가 안 나거나 다른 사유로 실패하면 FAIL.

    Args:
        db_path (str): SQLite DB 경로.
        sql (str): UPDATE 또는 DELETE 문.
        params (tuple): bind parameter.
        label (str): 검사 이름.
        results (list[bool]): 누적 결과 리스트.
    """
    conn = sqlite3.connect(db_path)
    try:
        try:
            conn.execute(sql, params)
            conn.commit()
            results.append(False)
            print(f"  [FAIL] {label}: TRIGGER did NOT reject the statement")
        except sqlite3.IntegrityError as e:
            if "append-only" in str(e):
                results.append(True)
                print(f"  [PASS] {label}: rejected with '{e}'")
            else:
                results.append(False)
                print(f"  [FAIL] {label}: rejected but message was '{e}'")
    finally:
        conn.close()


# ============================================================================
# 메인 시뮬레이션
# ============================================================================
def run_simulation(db_path: str) -> bool:
    """전체 검증 시퀀스를 실행한다 — publish → flush → DB 검사 → TRIGGER 검사.

    Pipeline:
        1. ``GameLoggerNode`` + ``SimPublisher``를 ``MultiThreadedExecutor`` (2 threads)에서 spin.
        2. 2 게임 × (GAME_START → USER_BOARD_CONFIRMED → ui_status → board_state → 2 AI_MOVE → GAME_END) 발행.
        3. 1초 sleep으로 write flush.
        4. SQLite 직접 쿼리로 row count·ply·side 검증.
        5. 각 테이블에 UPDATE/DELETE를 시도해 TRIGGER 거부를 검증.

    Args:
        db_path (str): temp DB 경로.

    Returns:
        bool: 모든 검사 PASS 시 True.
    """
    print(f"\n=== sim_game_logger — DB={db_path} ===\n")

    rclpy.init()

    logger_node = GameLoggerNode(db_path=db_path)
    pub_node = SimPublisher()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(logger_node)
    executor.add_node(pub_node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(0.5)  # pub-sub matching이 안정될 시간 확보

    print("[STEP 1] publishing 2 games × 2 moves each + ui_status + board_state")
    for gi, game_id in enumerate(GAME_IDS):
        job_id = f"job-{gi+1}"
        pub_node.emit_game_start(game_id, job_id)
        time.sleep(0.05)
        pub_node.emit_user_board_confirmed(game_id, job_id)
        time.sleep(0.05)
        pub_node.emit_ui_status(
            controller_state=UIStatus.STATE_RUNNING,
            working=True,
            verification=False,
            job_id=job_id,
        )
        time.sleep(0.05)
        pub_node.emit_board_state(["E1", "E8"], ["WK", "BK"])
        time.sleep(0.05)
        for uci, fen in MOVES_PER_GAME:
            pub_node.emit_ai_move(game_id, job_id, uci, fen)
            time.sleep(0.05)
        pub_node.emit_game_end(game_id, "checkmate")
        time.sleep(0.10)

    # executor가 모든 write를 flush 할 시간 확보
    time.sleep(1.0)

    executor.shutdown()
    logger_node.cleanup()
    logger_node.destroy_node()
    pub_node.destroy_node()
    rclpy.shutdown()

    results: list[bool] = []

    print("\n[STEP 2] verifying SQLite contents")
    n_games = query(db_path, "SELECT COUNT(*) FROM games")[0][0]
    expect("games row count", n_games, len(GAME_IDS), results)

    n_results = query(db_path, "SELECT COUNT(*) FROM game_results")[0][0]
    expect("game_results row count", n_results, len(GAME_IDS), results)

    n_moves = query(db_path, "SELECT COUNT(*) FROM moves")[0][0]
    expect(
        "moves row count",
        n_moves,
        len(GAME_IDS) * len(MOVES_PER_GAME),
        results,
    )

    for game_id in GAME_IDS:
        plies = [
            row[0]
            for row in query(
                db_path,
                "SELECT ply FROM moves WHERE game_id=? ORDER BY ply",
                (game_id,),
            )
        ]
        expect(f"plies for {game_id}", plies, [1, 2], results)

        sides = [
            row[0]
            for row in query(
                db_path,
                "SELECT side FROM moves WHERE game_id=? ORDER BY ply",
                (game_id,),
            )
        ]
        expect(f"sides for {game_id}", sides, EXPECTED_SIDES, results)

    n_event_kinds = query(
        db_path,
        "SELECT COUNT(DISTINCT kind) FROM events",
    )[0][0]
    # 기대값 6: game_event:{GAME_START, USER_BOARD_CONFIRMED, AI_MOVE, GAME_END}
    #          + ui_status + board_state
    expect("distinct event kinds", n_event_kinds, 6, results)

    print("\n[STEP 3] verifying append-only TRIGGERs reject UPDATE/DELETE")
    trigger_check(
        db_path,
        "UPDATE games SET started_at='hacked' WHERE game_id=?",
        (GAME_IDS[0],),
        "UPDATE games rejected",
        results,
    )
    trigger_check(
        db_path,
        "DELETE FROM games WHERE game_id=?",
        (GAME_IDS[0],),
        "DELETE games rejected",
        results,
    )
    trigger_check(
        db_path,
        "UPDATE moves SET uci='hack' WHERE game_id=?",
        (GAME_IDS[0],),
        "UPDATE moves rejected",
        results,
    )
    trigger_check(
        db_path,
        "DELETE FROM moves WHERE game_id=?",
        (GAME_IDS[0],),
        "DELETE moves rejected",
        results,
    )
    trigger_check(
        db_path,
        "UPDATE game_results SET result='hack' WHERE game_id=?",
        (GAME_IDS[0],),
        "UPDATE game_results rejected",
        results,
    )
    trigger_check(
        db_path,
        "DELETE FROM game_results WHERE game_id=?",
        (GAME_IDS[0],),
        "DELETE game_results rejected",
        results,
    )
    trigger_check(
        db_path,
        "UPDATE events SET kind='hack' WHERE id=1",
        (),
        "UPDATE events rejected",
        results,
    )
    trigger_check(
        db_path,
        "DELETE FROM events WHERE id=1",
        (),
        "DELETE events rejected",
        results,
    )

    passed = sum(results)
    total = len(results)
    print(f"\n=== {passed}/{total} checks passed ===")
    return passed == total


def main() -> int:
    """Entry point — temp DB 생성, 시뮬레이션 실행, WAL/SHM 부속 파일까지 정리.

    Returns:
        int: 모든 검사 통과 시 ``0``, 실패가 하나라도 있으면 ``1``.
    """
    fd, db_path = tempfile.mkstemp(prefix="sim_game_logger_", suffix=".db")
    os.close(fd)
    try:
        ok = run_simulation(db_path)
    finally:
        # WAL journal mode이므로 ``.db-wal``·``.db-shm`` 부속 파일도 함께 정리
        for suffix in ("", "-wal", "-shm"):
            p = db_path + suffix
            if os.path.exists(p):
                os.remove(p)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
