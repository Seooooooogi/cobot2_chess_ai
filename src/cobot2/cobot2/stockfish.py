import json

import rclpy
from rclpy.node import Node

from stockfish import Stockfish

from cobot2_interfaces.srv import StockfishMove


# ================= [기본 설정: 클래스보다 먼저 정의] =================
STOCKFISH_PATH = "/usr/games/stockfish"
SERVICE_NAME = "StockfishMove"

DEFAULT_SKILL_LEVEL = 10
DEFAULT_DEPTH = 15
DEFAULT_TURN = "w"
# ===================================================================


class AIMoveServiceNode(Node):
    def __init__(self):
        super().__init__("chess_ai_node")

        try:
            self.stockfish = Stockfish(path=STOCKFISH_PATH)
        except Exception as e:
            self.stockfish = None
            self.get_logger().error(f"Stockfish engine not found: {e}")

        self.dict_memory = {}

        self.srv = self.create_service(StockfishMove, SERVICE_NAME, self.get_best_move_callback)
        self.get_logger().info(f"Stockfish service ready: {SERVICE_NAME}")

    def dict_to_fen(self, pieces_dict, turn):
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

        # castling rights(간단 추론)
        rights = ""
        if pieces_dict.get("E1") == "WK":
            if pieces_dict.get("H1") == "WR":
                rights += "K"
            if pieces_dict.get("A1") == "WR":
                rights += "Q"
        if pieces_dict.get("E8") == "BK":
            if pieces_dict.get("H8") == "BR":
                rights += "k"
            if pieces_dict.get("A8") == "BR":
                rights += "q"
        if not rights:
            rights = "-"

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
        try:
            if self.stockfish is None:
                raise RuntimeError("Stockfish engine is not initialized")

            pieces_dict = json.loads(request.pieces_data) if request.pieces_data else {}

            skill_level = int(request.skill_level) if int(request.skill_level) > 0 else DEFAULT_SKILL_LEVEL
            depth = int(request.depth) if int(request.depth) > 0 else DEFAULT_DEPTH
            turn = request.turn if request.turn in ["w", "b"] else DEFAULT_TURN

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
                self.dict_memory = self.get_updated_dict(pieces_dict, best_move)

        except Exception as e:
            self.get_logger().error(f"Error in AI Calculation: {e}")
            response.success = False
            response.best_move = ""

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
