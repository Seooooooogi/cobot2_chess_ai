"""AIMoveServiceNode — Stockfish engine wrapper service (entry point: ``ros2 run cobot2 stockfish``).

Role:
    Exposes a ROS2 service that takes a chess board dict (``A1``..``H8`` → piece codes ``WP``/``BR``/...),
    converts it to FEN, and returns Stockfish's best move.

ROS2 Interfaces:
    Server: Service ``StockfishMove`` (cobot2_interfaces/StockfishMove) (line 73)

Internal State:
    - ``self.stockfish``  — ``Stockfish(path=STOCKFISH_PATH)`` instance, or ``None`` if engine binary missing (line 64-67).
    - ``self.dict_memory`` — last board dict used to infer ``last_move`` for en-passant heuristics (line 69; ``dict_to_fen`` line 114-129).

External Dependencies:
    - Stockfish binary at ``$STOCKFISH_PATH`` (env var, default ``/usr/games/stockfish``, line 38)
    - ``stockfish`` PyPI library
    - ``cobot2_interfaces.srv.StockfishMove``

Issues (Phase 1-1 doc Node 2):
    - ~~IMPORTANT S1-1: ``STOCKFISH_PATH`` is a module constant — Phase 4: env-ize.~~ **RESOLVED 2026-05-04**: ``os.getenv("STOCKFISH_PATH", ...)``.
    - ~~MINOR S1-2: Service QoS not explicitly declared (defaults used) → ROS2 Rule 4.~~ **RESOLVED 2026-05-04**: ``qos_profile=qos_profile_services_default`` 명시.
    # S1-3 RESOLVED 2026-05-05: castling rights now tracked via ``self.castling_rights`` (persisted to JSON); revoked on king/rook moves.
    # S1-4 RESOLVED 2026-05-05: ``dict_memory`` persisted to ``CHESS_AI_STATE_PATH`` JSON; loaded on node startup.
"""

import json
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default

from stockfish import Stockfish

from std_srvs.srv import Trigger

from cobot2_interfaces.srv import StockfishMove


# ================= [기본 설정: 클래스보다 먼저 정의] =================
STOCKFISH_PATH = os.getenv("STOCKFISH_PATH", "/usr/games/stockfish")
SERVICE_NAME = "StockfishMove"
RESET_SERVICE_NAME = "reset_chess_state"
CHESS_AI_STATE_PATH = os.path.expanduser(
    os.getenv("CHESS_AI_STATE_PATH", "~/.local/share/cobot2_chess_ai/chess_state.json")
)

DEFAULT_SKILL_LEVEL = 10
DEFAULT_DEPTH = 15
DEFAULT_TURN = "w"
# ===================================================================


class AIMoveServiceNode(Node):
    """ROS2 node hosting the ``StockfishMove`` and ``reset_chess_state`` services.

    Side Effects on construction:
        - Attempts to instantiate ``Stockfish(path=STOCKFISH_PATH)``; on failure stores ``None``
          and logs an error. Each request re-checks for ``None``.
        - Calls ``_load_state()`` to restore ``dict_memory`` and ``castling_rights`` from JSON
          (``CHESS_AI_STATE_PATH``). Defaults to empty dict / ``"KQkq"`` if file absent.
    """

    def __init__(self):
        super().__init__("chess_ai_node")

        try:
            self.stockfish = Stockfish(path=STOCKFISH_PATH)
        except Exception as e:
            self.stockfish = None
            self.get_logger().error(f"Stockfish engine not found: {e}")

        self._load_state()

        self.srv = self.create_service(
            StockfishMove,
            SERVICE_NAME,
            self.get_best_move_callback,
            qos_profile=qos_profile_services_default,
        )
        self.reset_srv = self.create_service(
            Trigger,
            RESET_SERVICE_NAME,
            self.reset_chess_state_callback,
            qos_profile=qos_profile_services_default,
        )
        self.get_logger().info(f"Stockfish service ready: {SERVICE_NAME}")

    def dict_to_fen(self, pieces_dict, turn):
        """Convert a board dict to a FEN string.

        Args:
            pieces_dict: dict[str, str] — keys ``"A1"``..``"H8"``, values ``"WP"``/``"BR"``/etc.
            turn:        ``"w"`` or ``"b"``.

        Returns:
            str — FEN with the form ``"<placement> <turn> <castling> <ep> 0 1"``.

        Side Effects:
            None (reads ``self.dict_memory`` but does not modify it here).

        Notes:
            Castling rights: uses ``self.castling_rights`` (persisted, revoked on king/rook moves).
            En-passant: inferred from ``self.dict_memory``-derived ``last_move``
            (single-removal/single-addition diff) — multi-piece changes yield ``last_move = None``.
        """
        last_move = None
        board = [["" for _ in range(8)] for _ in range(8)]

        piece_match = {
            "WR": "R",
            "WN": "N",
            "WB": "B",
            "WQ": "Q",
            "WK": "K",
            "WP": "P",
            "BR": "r",
            "BN": "n",
            "BB": "b",
            "BQ": "q",
            "BK": "k",
            "BP": "p",
        }

        # 이전 상태 기반 last_move 추론 (있으면)
        if self.dict_memory:
            removed = []
            added = []

            all_keys = set(self.dict_memory.keys()) | set(pieces_dict.keys())
            for pos in all_keys:
                old_val = self.dict_memory.get(pos)
                new_val = pieces_dict.get(pos)
                if old_val != new_val:
                    if old_val is not None:
                        removed.append(pos.lower())
                    if new_val is not None:
                        added.append(pos.lower())

            if len(removed) == 1 and len(added) == 1:
                last_move = removed[0] + added[0]

        for position, piece in pieces_dict.items():
            col = ord(position[0].upper()) - ord("A")
            row = 8 - int(position[1])
            board[row][col] = piece_match.get(piece, "")

        fen_rows = []
        for row in board:
            empty_count = 0
            row_str = ""
            for cell in row:
                if cell == "":
                    empty_count += 1
                else:
                    if empty_count > 0:
                        row_str += str(empty_count)
                        empty_count = 0
                    row_str += cell
            if empty_count > 0:
                row_str += str(empty_count)
            fen_rows.append(row_str)

        # castling rights — persisted by _load_state/_save_state; always a non-None string
        rights = self.castling_rights or "-"

        # en-passant (간단 추론)
        ep_square = "-"
        if last_move is not None and pieces_dict.get(last_move[2:4].upper()) in ["WP", "BP"]:
            if last_move[1] == "2" and last_move[3] == "4":
                ep_square = last_move[0] + "3"
            elif last_move[1] == "7" and last_move[3] == "5":
                ep_square = last_move[0] + "6"

        fen = f"{'/'.join(fen_rows)} {turn} {rights} {ep_square} 0 1"
        return fen

    def get_updated_dict(self, pieces_dict, move):
        """Apply a chess move to a board dict (used to update ``self.dict_memory``).

        Args:
            pieces_dict: dict[str, str] — current board.
            move:        UCI-style move string, e.g. ``"e2e4"`` or ``"e7e8q"`` (promotion suffix is ignored
                         here — only ``move[0:4]`` is consumed).

        Returns:
            dict[str, str] — updated board (input dict is copied, not mutated).

        Side Effects:
            None.

        Notes:
            Branches: pawn diagonal move into empty square → en-passant capture (removes the captured
            pawn at ``to_pos[0] + from_pos[1]``); king two-square move → castling (relocates rook
            from ``A``/``H`` to ``D``/``F``).
        """
        from_pos = move[0:2].upper()
        to_pos = move[2:4].upper()

        updated_dict = pieces_dict.copy()
        piece = updated_dict.pop(from_pos, None)
        if piece:
            updated_dict[to_pos] = piece

        # 앙파상(간단)
        if piece and piece[1] == "P":
            if from_pos[0] != to_pos[0] and to_pos not in pieces_dict:
                en_passant_pos = to_pos[0] + from_pos[1]
                updated_dict.pop(en_passant_pos, None)

        # 캐슬링(간단)
        if piece and piece[1] == "K":
            if abs(ord(from_pos[0]) - ord(to_pos[0])) == 2:
                if to_pos[0] == "G":
                    rook_from = "H" + from_pos[1]
                    rook_to = "F" + from_pos[1]
                else:
                    rook_from = "A" + from_pos[1]
                    rook_to = "D" + from_pos[1]
                rook_piece = updated_dict.pop(rook_from, None)
                if rook_piece:
                    updated_dict[rook_to] = rook_piece

        return updated_dict

    def get_best_move_callback(self, request, response):
        saved_rights = self.castling_rights  # snapshot for rollback on exception
        saved_memory = self.dict_memory
        try:
            if self.stockfish is None:
                raise RuntimeError("Stockfish engine is not initialized")

            pieces_dict = json.loads(request.pieces_data) if request.pieces_data else {}

            skill_level = int(request.skill_level) if int(request.skill_level) > 0 else DEFAULT_SKILL_LEVEL
            depth = int(request.depth) if int(request.depth) > 0 else DEFAULT_DEPTH
            turn = request.turn if request.turn in ["w", "b"] else DEFAULT_TURN

            # On first call (empty history): clamp persisted "KQkq" to what the
            # current position can actually support (prevents invalid FEN when
            # pieces for a right are absent, e.g. rook already captured/moved).
            if not self.dict_memory:
                inferred = ""
                if pieces_dict.get("E1") == "WK":
                    if pieces_dict.get("H1") == "WR": inferred += "K"
                    if pieces_dict.get("A1") == "WR": inferred += "Q"
                if pieces_dict.get("E8") == "BK":
                    if pieces_dict.get("H8") == "BR": inferred += "k"
                    if pieces_dict.get("A8") == "BR": inferred += "q"
                self.castling_rights = inferred

            # Update castling rights for the human's move (prev board → current board)
            self._revoke_castling_rights(self.dict_memory, pieces_dict)

            self.stockfish.set_skill_level(skill_level)
            self.stockfish.set_depth(depth)

            fen = self.dict_to_fen(pieces_dict, turn)
            self.get_logger().info(f"FEN: {fen}")

            if not self.stockfish.is_fen_valid(fen):
                raise ValueError("Invalid FEN generated")

            self.stockfish.set_fen_position(fen)
            best_move = self.stockfish.get_best_move()

            response.best_move = best_move if best_move else ""
            response.success = True if best_move else False

            if best_move:
                updated_dict = self.get_updated_dict(pieces_dict, best_move)
                # Update castling rights for the AI's move (current board → post-AI board)
                self._revoke_castling_rights(pieces_dict, updated_dict)
                self.dict_memory = updated_dict
                self._save_state()  # persist only on complete success

        except Exception as e:
            self.get_logger().error(f"Error in AI Calculation: {e}")
            response.success = False
            response.best_move = ""
            self.castling_rights = saved_rights  # rollback to pre-call state
            self.dict_memory = saved_memory

        return response


    def _load_state(self) -> None:
        self._state_path = CHESS_AI_STATE_PATH
        try:
            with open(self._state_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.dict_memory = data.get("dict_memory", {})
            self.castling_rights = data.get("castling_rights", "KQkq")
            self.get_logger().info(f"Chess state loaded from {self._state_path}")
        except FileNotFoundError:
            self.dict_memory = {}
            self.castling_rights = "KQkq"
            self.get_logger().info("No prior chess state file; starting fresh.")
        except Exception as e:
            self.dict_memory = {}
            self.castling_rights = "KQkq"
            self.get_logger().warn(f"Chess state load failed ({e}); starting fresh.")

    def _save_state(self) -> None:
        try:
            os.makedirs(os.path.dirname(self._state_path), exist_ok=True)
            data = {
                "dict_memory": self.dict_memory,
                "castling_rights": self.castling_rights,
            }
            with open(self._state_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            self.get_logger().warn(f"Chess state save failed: {e}")

    def _revoke_castling_rights(self, prev_dict: dict, new_dict: dict) -> None:
        rights = self.castling_rights or ""
        if prev_dict.get("E1") == "WK" and new_dict.get("E1") != "WK":
            rights = rights.replace("K", "").replace("Q", "")
        if prev_dict.get("H1") == "WR" and new_dict.get("H1") != "WR":
            rights = rights.replace("K", "")
        if prev_dict.get("A1") == "WR" and new_dict.get("A1") != "WR":
            rights = rights.replace("Q", "")
        if prev_dict.get("E8") == "BK" and new_dict.get("E8") != "BK":
            rights = rights.replace("k", "").replace("q", "")
        if prev_dict.get("H8") == "BR" and new_dict.get("H8") != "BR":
            rights = rights.replace("k", "")
        if prev_dict.get("A8") == "BR" and new_dict.get("A8") != "BR":
            rights = rights.replace("q", "")
        self.castling_rights = rights

    def reset_chess_state_callback(self, request, response):
        try:
            self.dict_memory = {}
            self.castling_rights = "KQkq"
            try:
                os.remove(self._state_path)
            except FileNotFoundError:
                pass
            self.get_logger().info("Chess state reset for new game.")
            response.success = True
            response.message = "reset ok"
        except Exception as e:
            self.get_logger().error(f"Chess state reset failed: {e}")
            response.success = False
            response.message = str(e)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AIMoveServiceNode()
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
