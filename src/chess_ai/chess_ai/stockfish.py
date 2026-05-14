"""Stockfish 엔진을 ROS2 service로 노출하는 노드 (entry point: ``ros2 run chess_ai stockfish``).

보드 dict (``"A1"``..``"H8"`` → ``"WP"``/``"BR"`` piece code)를 입력받아 FEN 문자열로
변환한 뒤 Stockfish의 best move를 반환한다. ``dict_memory``와 ``castling_rights``는
JSON 파일로 영속화돼 노드 재기동 시 복원된다.

Environment:
    STOCKFISH_PATH (str): Stockfish 바이너리 경로. 기본 ``/usr/games/stockfish``.
    CHESS_AI_STATE_PATH (str): 상태 JSON 경로. 기본
        ``~/.local/share/cobot2_chess_ai/chess_state.json``.

Note:
    Service 이름은 사설 namespace ``~/...`` — 노드 이름 ``chess_ai_node`` 하위로 풀린다
    (예: ``/chess_ai_node/StockfishMove``). 절대 경로 또는 namespace-relative 형태로 쓰면
    root namespace로 빠지므로 의도적으로 ``~/`` prefix를 강제한다.
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
# 사설 namespace (~/...) 강제 — root namespace로 빠지는 회귀를 차단 (절대 경로·
# namespace-relative 형태 금지)
SERVICE_NAME = "~/StockfishMove"
RESET_SERVICE_NAME = "~/reset_chess_state"
CHESS_AI_STATE_PATH = os.path.expanduser(
    os.getenv("CHESS_AI_STATE_PATH", "~/.local/share/cobot2_chess_ai/chess_state.json")
)

DEFAULT_SKILL_LEVEL = 10
DEFAULT_DEPTH = 15
DEFAULT_TURN = "w"

# Stockfish 라이브러리 원래 범위 그대로 (sentinel 미사용)
DEPTH_MIN, DEPTH_MAX = 1, 30
SKILL_LEVEL_MIN, SKILL_LEVEL_MAX = 0, 20
VALID_TURNS = ("w", "b")
# ===================================================================


class AIMoveServiceNode(Node):
    """Stockfish 엔진을 wrapping 하는 ROS2 서비스 노드.

    Services:
        ~/StockfishMove (chess_ai_interfaces/srv/StockfishMove): 보드 dict → best move +
            적용 후 보드의 FEN.
        ~/reset_chess_state (std_srvs/srv/Trigger): ``dict_memory`` + ``castling_rights``
            초기화 및 영속 파일 삭제.

    Parameters (rosbridge ``set_parameters``로 runtime 변경 가능):
        depth (int, default 15): Stockfish 탐색 깊이. 유효 범위 ``[1, 30]``.
        skill_level (int, default 10): Stockfish skill level. 유효 범위 ``[0, 20]``.
        default_turn (string, default ``"w"``): AI가 두는 색상. ``"w"`` 또는 ``"b"``.

    Internal state:
        self.stockfish (stockfish.Stockfish | None): 엔진 핸들. 바이너리 부재·로드 실패 시
            ``None``으로 유지된다 (노드 자체는 생존).
        self.dict_memory (dict[str, str]): 직전에 응답한 보드 상태. en-passant 추론용
            ``last_move`` 계산에 사용.
        self.castling_rights (str): ``"KQkq"`` 표기 castling 권한. 킹/룩 이동으로 revoke.

    Warning:
        Stockfish 인스턴스화는 try/except로 감싼다. 호출 측은 ``response.success == False``로
        엔진 부재 상황을 식별해야 한다 — service call은 RuntimeError 대신 빈 best_move를
        반환한다.
    """

    def __init__(self):
        super().__init__("chess_ai_node")

        try:
            self.stockfish = Stockfish(path=STOCKFISH_PATH)
        except Exception as e:
            self.stockfish = None
            self.get_logger().error(f"Stockfish engine not found: {e}")

        self._load_state()

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
        """``set_parameters`` 요청을 type·range로 검증하는 callback.

        rosbridge/CLI에서 들어온 parameter 변경을 첫 위반에서 reject 하고 warn 로그를 남긴다.
        조용한 실패를 방지하기 위해 거부 사유를 reason 필드에 채운다.

        Args:
            params (list[rclpy.parameter.Parameter]): 적용 시도 중인 parameter 목록.

        Returns:
            rcl_interfaces.msg.SetParametersResult: 첫 위반에서 reject, 모두 통과 시 ok.
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
        """보드 dict을 FEN 문자열로 직렬화한다.

        En-passant target square는 ``self.dict_memory`` 와의 single-removal /
        single-addition diff로 last_move를 추론한 뒤, 폰의 2칸 전진 패턴에서만 채운다.
        2개 이상 칸이 변하면 last_move 추론을 포기해 en-passant는 ``-``로 둔다.
        Castling rights는 ``self.castling_rights``를 그대로 사용 — revoke는 caller 책임.

        Args:
            pieces_dict (dict[str, str]): square → piece code (``"WP"``/``"BR"``/...).
            turn (str): ``"w"`` 또는 ``"b"`` — FEN 차례 필드.

        Returns:
            str: ``"<placement> <turn> <castling> <ep> 0 1"`` 형식 FEN.

        Note:
            halfmove clock과 fullmove number는 단순화를 위해 ``0 1`` 고정. 50수 룰/
            반복국면 판정이 필요하면 추후 별도 추적이 필요하다.
            ``self.dict_memory``는 read-only 참조 — side effect 없음.
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

        # FEN 랭크 인덱스: row=0이 8랭크(흑 뒤쪽), row=7이 1랭크(백 뒤쪽)
        for position, piece in pieces_dict.items():
            col = ord(position[0].upper()) - ord("A")
            row = 8 - int(position[1])
            board[row][col] = piece_match.get(piece, "")

        # 빈 칸 압축: 연속 빈 칸 수를 숫자로 (FEN 사양)
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

        rights = self.castling_rights or "-"

        # 폰이 2칸 전진했을 때만 ep_square 채움 — Stockfish이 en-passant 합법성 판단에 사용
        ep_square = "-"
        if last_move is not None and pieces_dict.get(last_move[2:4].upper()) in ["WP", "BP"]:
            if last_move[1] == "2" and last_move[3] == "4":
                ep_square = last_move[0] + "3"
            elif last_move[1] == "7" and last_move[3] == "5":
                ep_square = last_move[0] + "6"

        fen = f"{'/'.join(fen_rows)} {turn} {rights} {ep_square} 0 1"
        return fen

    def get_updated_dict(self, pieces_dict, move):
        """UCI 수를 보드 dict에 적용해 새 dict을 반환한다 (입력은 mutate 하지 않음).

        En-passant: 폰이 대각선 빈 칸으로 이동하면 ``to_pos[0] + from_pos[1]``의 폰을 제거.
        Castling: 킹이 2칸 옆으로 이동하면 룩을 ``A``/``H`` ↔ ``D``/``F``로 재배치.

        Args:
            pieces_dict (dict[str, str]): 적용 전 보드.
            move (str): UCI 수 (예: ``"e2e4"`` 또는 ``"e7e8q"``). 프로모션 suffix는 무시되며
                ``move[0:4]``만 소비된다.

        Returns:
            dict[str, str]: 수 적용 후 보드 (얕은 copy).

        Note:
            프로모션 시 piece type 교체 로직은 없다 — 폰이 ``e8``에 그대로 남는다.
            FEN 직렬화 단계에서는 영향이 없지만 후속 이동 추적에 영향을 줄 수 있다.
        """
        from_pos = move[0:2].upper()
        to_pos = move[2:4].upper()

        updated_dict = pieces_dict.copy()
        piece = updated_dict.pop(from_pos, None)
        if piece:
            updated_dict[to_pos] = piece

        if piece and piece[1] == "P":
            if from_pos[0] != to_pos[0] and to_pos not in pieces_dict:
                en_passant_pos = to_pos[0] + from_pos[1]
                updated_dict.pop(en_passant_pos, None)

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
        """``~/StockfishMove`` service handler.

        Pipeline: ``dict_memory``가 비어 있으면 현재 보드로부터 castling rights를 추론 →
        prev/new diff로 revoke 적용 → Stockfish에 FEN set → ``get_best_move`` 호출 →
        적용 후 보드를 ``response.fen``으로 직렬화 → 영속 상태 갱신.

        Args:
            request (StockfishMove.Request): ``pieces_data`` 필드에 JSON-serialized 보드 dict.
            response (StockfishMove.Response): ``best_move`` (UCI), ``success``, ``fen``을
                채워 반환한다.

        Returns:
            StockfishMove.Response: 성공·실패 모든 경로에서 동일 객체 반환.

        Note:
            엔진 호출 중 발생하는 모든 예외는 ``response.success=False``로 squash 되며,
            ``self.castling_rights``·``self.dict_memory``는 호출 직전 스냅샷으로 롤백된다.
            영속 ``_save_state()``는 best_move 획득 + dict 갱신이 모두 성공한 경로에서만
            호출된다.
        """
        saved_rights = self.castling_rights
        saved_memory = self.dict_memory
        try:
            if self.stockfish is None:
                raise RuntimeError("Stockfish engine is not initialized")

            pieces_dict = json.loads(request.pieces_data) if request.pieces_data else {}

            # 엔진 설정값 단일 경로 — ROS2 parameter만 사용 (StockfishMove.srv는 보드만 전달)
            skill_level = int(self.get_parameter("skill_level").value)
            depth = int(self.get_parameter("depth").value)
            turn = str(self.get_parameter("default_turn").value)

            # 콜드 스타트(dict_memory 빈 상태)에는 보존된 "KQkq"가 실제 보드와 불일치할 수
            # 있으므로 현재 보드로부터 자연스러운 castling rights를 재추론한다
            if not self.dict_memory:
                inferred = ""
                if pieces_dict.get("E1") == "WK":
                    if pieces_dict.get("H1") == "WR": inferred += "K"
                    if pieces_dict.get("A1") == "WR": inferred += "Q"
                if pieces_dict.get("E8") == "BK":
                    if pieces_dict.get("H8") == "BR": inferred += "k"
                    if pieces_dict.get("A8") == "BR": inferred += "q"
                self.castling_rights = inferred

            # 사용자 수 적용분의 revoke (직전 보드 → 새 보드)
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
            response.fen = ""  # 실패 경로 기본값; 성공 시 아래에서 post-move FEN으로 갱신

            if best_move:
                updated_dict = self.get_updated_dict(pieces_dict, best_move)
                # AI 수 적용분의 revoke (사용자 수 적용 보드 → AI 수 적용 보드)
                self._revoke_castling_rights(pieces_dict, updated_dict)
                self.dict_memory = updated_dict
                self._save_state()
                # 다음 차례(상대 색)로 FEN 직렬화 — game_logger audit 입력으로 사용
                next_turn = "b" if turn == "w" else "w"
                response.fen = self.dict_to_fen(updated_dict, next_turn)

        except Exception as e:
            self.get_logger().error(f"Error in AI Calculation: {e}")
            response.success = False
            response.best_move = ""
            response.fen = ""
            self.castling_rights = saved_rights
            self.dict_memory = saved_memory

        return response


    def _load_state(self) -> None:
        """영속 JSON에서 ``dict_memory``와 ``castling_rights``를 복원한다.

        파일 부재 시 빈 dict / ``"KQkq"``로 fresh-start 한다. JSON 파싱 실패도 동일하게
        fresh-start 하되 warn 로그를 남긴다 (silent corruption 방지).
        """
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
        """현재 ``dict_memory``와 ``castling_rights``를 JSON에 덮어쓴다.

        상위 디렉토리는 ``os.makedirs(exist_ok=True)``로 보장한다. write 실패는 warn 로그로만
        남기고 노드 동작은 유지한다.

        Warning:
            non-atomic write — ``open(..., "w")``를 직접 사용한다. SIGKILL/디스크 full
            상황에서 파일이 잘려 다음 기동 시 fresh-start 분기로 빠질 수 있다.
        """
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
        """두 보드 사이의 변동에 따라 ``self.castling_rights``를 in-place 갱신한다.

        킹이 ``E1``/``E8``에서 사라지면 해당 색의 양쪽 castling을 모두 제거.
        룩이 ``A1``/``H1``/``A8``/``H8``에서 사라지면 해당 방향만 제거.

        Args:
            prev_dict (dict[str, str]): 변동 전 보드.
            new_dict (dict[str, str]): 변동 후 보드.
        """
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
        """``~/reset_chess_state`` (Trigger) handler — 신규 게임용 상태 초기화.

        ``dict_memory``를 비우고 ``castling_rights``를 ``"KQkq"``로 되돌린 뒤, 영속 JSON
        파일을 삭제한다 (파일 없으면 무시). 작업 자체가 실패하면 ``success=False``와 함께
        예외 메시지를 반환한다.

        Returns:
            std_srvs.srv.Trigger.Response: 성공 시 ``success=True, message="reset ok"``.
        """
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
    """Entry point: ``AIMoveServiceNode`` 생성 후 ``rclpy.spin``.

    SIGINT는 ``KeyboardInterrupt``로 받아 정상 종료한다. ``rclpy.shutdown()``은
    ``rclpy.ok()`` 가드 후 호출 — supervisor가 이미 shutdown한 context를 재호출하면
    ``RCLError``가 발생한다.
    """
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
