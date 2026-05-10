#!/usr/bin/env python3
"""Standalone integration simulation for the game_logger SQLite audit pipeline.

Phase 5 sub-phase E verification harness. Runs without any hardware, vision,
camera, Firebase, or rosbridge dependency — only ROS2 + cobot2 + cobot2_interfaces
need to be sourced. Useful for:

  - Confirming GameEvent / UIStatus / BoardState pub-sub round-trip works.
  - Confirming SQLite append-only TRIGGERs reject UPDATE/DELETE attempts.
  - Confirming side inference (W/B) from post-move FEN.
  - Re-running after refactors to catch regressions.

Usage:
    # 1) Source the workspace.
    source /home/rokey/cobot2_chess_ai/install/setup.bash

    # 2) Run.
    python3 /home/rokey/cobot2_chess_ai/scripts/sim_game_logger.py

The script creates a throw-away temp SQLite DB, runs the simulation, prints
PASS/FAIL for every check, and deletes the temp DB on exit. Exit code 0 on
all-pass, 1 otherwise.

Notes:
  - This script does **not** start the launch file. It instantiates
    GameLoggerNode + a synthetic publisher node directly and spins them on
    a single MultiThreadedExecutor.
  - QoS matches the production pipeline (RELIABLE + TRANSIENT_LOCAL +
    KEEP_LAST). Late join is not exercised (publisher and subscriber start
    together) but the DDL + write path is.
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

from cobot2_interfaces.msg import BoardState, GameEvent, UIStatus
from cobot2.game_logger import GameLoggerNode


# ----------------------------------------------------------------------------
# Test scenario constants. Two synthetic games, each with two AI moves.
# ----------------------------------------------------------------------------
GAME_IDS = ["sim-game-001", "sim-game-002"]
MOVES_PER_GAME = [
    # (uci, post_move_fen)  -- turn field in FEN flips after each move.
    ("e2e4", "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1"),
    ("d2d4", "rnbqkbnr/pppppppp/8/8/3PP3/8/PPP2PPP/RNBQKBNR w KQkq d3 0 2"),
]
EXPECTED_SIDES = ["W", "B"]  # derived from FEN turn-field flip


# ============================================================================
# Synthetic publisher
# ============================================================================
class SimPublisher(Node):
    """Publishes synthetic GameEvent / UIStatus / BoardState messages."""

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
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        return h

    def emit_game_start(self, game_id: str, job_id: str) -> None:
        msg = GameEvent()
        msg.header = self._stamp()
        msg.kind = GameEvent.KIND_GAME_START
        msg.game_id = game_id
        msg.job_id = job_id
        self._pub_event.publish(msg)

    def emit_user_board_confirmed(self, game_id: str, job_id: str) -> None:
        msg = GameEvent()
        msg.header = self._stamp()
        msg.kind = GameEvent.KIND_USER_BOARD_CONFIRMED
        msg.game_id = game_id
        msg.job_id = job_id
        self._pub_event.publish(msg)

    def emit_ai_move(self, game_id: str, job_id: str, uci: str, fen: str) -> None:
        msg = GameEvent()
        msg.header = self._stamp()
        msg.kind = GameEvent.KIND_AI_MOVE
        msg.game_id = game_id
        msg.job_id = job_id
        msg.uci = uci
        msg.fen = fen
        self._pub_event.publish(msg)

    def emit_game_end(self, game_id: str, result: str) -> None:
        msg = GameEvent()
        msg.header = self._stamp()
        msg.kind = GameEvent.KIND_GAME_END
        msg.game_id = game_id
        msg.result = result
        self._pub_event.publish(msg)

    def emit_ui_status(
        self, controller_state: int, working: bool, verification: bool, job_id: str
    ) -> None:
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
        msg = BoardState()
        msg.header = self._stamp()
        msg.header.frame_id = "chess_board"
        msg.squares = squares
        msg.pieces = pieces
        msg.piece_count = len(squares)
        self._pub_board.publish(msg)


# ============================================================================
# Verification helpers
# ============================================================================
def query(db_path: str, sql: str, params: tuple = ()) -> list[Any]:
    conn = sqlite3.connect(db_path)
    try:
        return conn.execute(sql, params).fetchall()
    finally:
        conn.close()


def expect(label: str, actual: Any, expected: Any, results: list[bool]) -> None:
    ok = actual == expected
    results.append(ok)
    mark = "PASS" if ok else "FAIL"
    print(f"  [{mark}] {label}: got={actual!r} expected={expected!r}")


def trigger_check(db_path: str, sql: str, params: tuple, label: str, results: list[bool]) -> None:
    """Check that an UPDATE/DELETE statement is rejected by an append-only TRIGGER."""
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
# Main simulation
# ============================================================================
def run_simulation(db_path: str) -> bool:
    print(f"\n=== sim_game_logger — DB={db_path} ===\n")

    rclpy.init()

    logger_node = GameLoggerNode(db_path=db_path)
    pub_node = SimPublisher()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(logger_node)
    executor.add_node(pub_node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(0.5)  # let pub-sub matching settle

    # ----- publish two games, two moves each -----
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

    # let the executor flush all writes
    time.sleep(1.0)

    # ----- shutdown -----
    executor.shutdown()
    logger_node.cleanup()
    logger_node.destroy_node()
    pub_node.destroy_node()
    rclpy.shutdown()

    # ----- verify -----
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
    # expect: game_event:GAME_START, USER_BOARD_CONFIRMED, AI_MOVE, GAME_END
    #         + ui_status + board_state  → 6 distinct kinds
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
    fd, db_path = tempfile.mkstemp(prefix="sim_game_logger_", suffix=".db")
    os.close(fd)
    try:
        ok = run_simulation(db_path)
    finally:
        for suffix in ("", "-wal", "-shm"):
            p = db_path + suffix
            if os.path.exists(p):
                os.remove(p)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
