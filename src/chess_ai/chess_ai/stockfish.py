"""AIMoveServiceNode — Stockfish engine wrapper service (entry point: ``ros2 run chess_ai stockfish``).

Role:
    Exposes a ROS2 service that takes a chess board dict (``A1``..``H8`` → piece codes ``WP``/``BR``/...),
    converts it to FEN, and returns Stockfish's best move.

ROS2 Interfaces:
    Server: Service ``StockfishMove`` (chess_ai_interfaces/StockfishMove)
    Server: Service ``reset_chess_state`` (std_srvs/Trigger)
    ROS2 parameters (Phase 5 sub-phase D3, Web UI가 rosbridge set_parameters로 설정):
        - ``depth`` (int, 1–30, default 15)            — Stockfish search depth.
        - ``skill_level`` (int, 0–20, default 10)      — Stockfish skill level.
        - ``default_turn`` (string, "w" or "b", default "w") — AI가 두는 색상.
        Range validation은 ``add_on_set_parameters_callback``에서 수행. 위반 시 set 실패.
        엔진은 항상 이 parameter 값을 사용 — StockfishMove.srv는 보드 데이터만 전달.

Internal State:
    - ``self.stockfish``  — ``Stockfish(path=STOCKFISH_PATH)`` instance, or ``None`` if engine binary missing.
    - ``self.dict_memory`` — last board dict used to infer ``last_move`` for en-passant heuristics.

External Dependencies:
    - Stockfish binary at ``$STOCKFISH_PATH`` (env var, default ``/usr/games/stockfish``)
    - ``stockfish`` PyPI library
    - ``chess_ai_interfaces.srv.StockfishMove``

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
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_services_default
from rcl_interfaces.msg import SetParametersResult

from stockfish import Stockfish

from std_srvs.srv import Trigger

from chess_ai_interfaces.srv import StockfishMove


# ================= [기본 설정: 클래스보다 먼저 정의] =================
STOCKFISH_PATH = os.getenv("STOCKFISH_PATH", "/usr/games/stockfish")
# 사설 네임스페이스(~/...) 사용 — 노드 코드 노드명 (chess_ai_node) 하위로 풀림.
# Rule 5 (resource = node owns) 준수. 절대 경로(`/`로 시작) 또는 namespace-relative
# (`StockfishMove`) 사용 금지 — 후자는 root namespace에 매핑됨 (PB-4 회귀 방지).
SERVICE_NAME = "~/StockfishMove"
RESET_SERVICE_NAME = "~/reset_chess_state"
CHESS_AI_STATE_PATH = os.path.expanduser(
    os.getenv("CHESS_AI_STATE_PATH", "~/.local/share/cobot2_chess_ai/chess_state.json")
)

DEFAULT_SKILL_LEVEL = 10
DEFAULT_DEPTH = 15
DEFAULT_TURN = "w"

# ROS2 parameter range validation (sub-phase D3).
# Stockfish 라이브러리 원래 범위 그대로. sentinel 설계 폐기로 0도 자연 허용.
DEPTH_MIN, DEPTH_MAX = 1, 30
SKILL_LEVEL_MIN, SKILL_LEVEL_MAX = 0, 20
VALID_TURNS = ("w", "b")
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

        # Phase 5 sub-phase D3: engine config를 ROS2 parameter로 노출 (Web UI →
        # rosbridge set_parameters). Range validation은 _on_set_parameters_callback.
        self.declare_parameter("depth", DEFAULT_DEPTH)
        self.declare_parameter("skill_level", DEFAULT_SKILL_LEVEL)
        self.declare_parameter("default_turn", DEFAULT_TURN)
        self.add_on_set_parameters_callback(self._on_set_parameters)

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

    def _on_set_parameters(self, params):
        """ROS2 parameter set 검증 콜백.

        UI(rosbridge)에서 set_parameters 호출 시 트리거. depth/skill_level 정수
        범위, default_turn 'w'/'b' 검증. 위반 시 successful=False로 거부 + 노드
        로그 — Rule 7 (silent failure 방지).
        """
        for p in params:
            reason = None
            if p.name == "depth":
                if p.type_ != Parameter.Type.INTEGER or not (DEPTH_MIN <= p.value <= DEPTH_MAX):
                    reason = f"depth out of range [{DEPTH_MIN}, {DEPTH_MAX}]: got {p.value}"
            elif p.name == "skill_level":
                if p.type_ != Parameter.Type.INTEGER or not (SKILL_LEVEL_MIN <= p.value <= SKILL_LEVEL_MAX):
                    reason = f"skill_level out of range [{SKILL_LEVEL_MIN}, {SKILL_LEVEL_MAX}]: got {p.value}"
            elif p.name == "default_turn":
                if p.type_ != Parameter.Type.STRING or p.value not in VALID_TURNS:
                    reason = f"default_turn must be one of {VALID_TURNS}: got {p.value!r}"
            if reason is not None:
                self.get_logger().warn(f"parameter set rejected: {reason}")
                return SetParametersResult(successful=False, reason=reason)
        return SetParametersResult(successful=True)

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

        # board[row][col]: row=0이 8랭크(흑 뒤쪽), row=7이 1랭크(백 뒤쪽)
        # "A1" → col=0, row=7 / "H8" → col=7, row=0
        for position, piece in pieces_dict.items():
            col = ord(position[0].upper()) - ord("A")  # 'A'=0 ~ 'H'=7
            row = 8 - int(position[1])                  # '1'→7, '8'→0
            board[row][col] = piece_match.get(piece, "")

        # FEN 랭크 직렬화: 빈 칸은 연속 빈 칸 수를 숫자로 압축 (예: "rnbqkbnr/pppppppp/8/...")
        fen_rows = []
        for row in board:
            empty_count = 0
            row_str = ""
            for cell in row:
                if cell == "":
                    empty_count += 1
                else:
                    if empty_count > 0:
                        row_str += str(empty_count)  # 이전 빈 칸 수 flush
                        empty_count = 0
                    row_str += cell
            if empty_count > 0:
                row_str += str(empty_count)  # 행 끝 빈 칸 flush
            fen_rows.append(row_str)

        # 캐슬링 권한 — _load_state/_save_state 로 영속화; "KQkq" → "KQ" → "-" 형태로 축소
        rights = self.castling_rights or "-"

        # 앙파상 타겟 칸 추론: 폰이 2칸 전진했으면 그 사이 칸을 ep_square로 지정
        # (Stockfish이 앙파상 가능 여부 판단에 사용)
        ep_square = "-"
        if last_move is not None and pieces_dict.get(last_move[2:4].upper()) in ["WP", "BP"]:
            if last_move[1] == "2" and last_move[3] == "4":  # 백 폰 2칸 전진
                ep_square = last_move[0] + "3"
            elif last_move[1] == "7" and last_move[3] == "5":  # 흑 폰 2칸 전진
                ep_square = last_move[0] + "6"

        # FEN 최종 조합: "<랭크> <차례> <캐슬링> <앙파상> <반수> <수 번호>"
        # 반수(halfmove clock)와 수 번호는 단순화를 위해 "0 1" 고정.
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

            # Phase 5 sub-phase D3: 엔진 설정값 단일 경로 — ROS2 parameter만 사용.
            # StockfishMove.srv는 보드 데이터만 전달. depth/skill_level/turn 필드 폐기.
            skill_level = int(self.get_parameter("skill_level").value)
            depth = int(self.get_parameter("depth").value)
            turn = str(self.get_parameter("default_turn").value)

            # 첫 호출(dict_memory 비어있을 때): 보존된 "KQkq"를 현재 보드로 클램프.
            # 룩/킹이 이미 없거나 움직인 상태라면 해당 캐슬링 권한을 제거해 유효한 FEN 보장.
            # (체스판 재배치 후 상태 복원 시나리오 대응 — PB-4 이후 설계)
            if not self.dict_memory:
                inferred = ""
                if pieces_dict.get("E1") == "WK":
                    if pieces_dict.get("H1") == "WR": inferred += "K"  # 백 킹사이드
                    if pieces_dict.get("A1") == "WR": inferred += "Q"  # 백 퀸사이드
                if pieces_dict.get("E8") == "BK":
                    if pieces_dict.get("H8") == "BR": inferred += "k"  # 흑 킹사이드
                    if pieces_dict.get("A8") == "BR": inferred += "q"  # 흑 퀸사이드
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
            response.fen = ""  # 기본값. AI 수 성공 시 아래에서 post-move FEN으로 채움.

            if best_move:
                updated_dict = self.get_updated_dict(pieces_dict, best_move)
                # Update castling rights for the AI's move (current board → post-AI board)
                self._revoke_castling_rights(pieces_dict, updated_dict)
                self.dict_memory = updated_dict
                self._save_state()  # persist only on complete success
                # AI 수 직후의 보드를 다음 차례 색상으로 FEN 직렬화 — game_logger
                # audit (sub-phase E)에 활용. 차례 flip: 'w' ↔ 'b'.
                next_turn = "b" if turn == "w" else "w"
                response.fen = self.dict_to_fen(updated_dict, next_turn)

        except Exception as e:
            self.get_logger().error(f"Error in AI Calculation: {e}")
            response.success = False
            response.best_move = ""
            response.fen = ""
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
