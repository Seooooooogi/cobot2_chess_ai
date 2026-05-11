# MainController → game_logger 명시 게임 이벤트 (Phase 5 sub-phase E).
#
# Hard Rule #6 (append-only audit log)을 SQLite로 영속화하기 위해 main_controller가
# FSM 경계마다 발행. game_logger 노드만 구독 (UI rosbridge에는 노출하지 않음 — 안전
# 경계 외 정보).
#
# QoS (publisher 측): RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(10).
# late-join하는 logger가 최근 10개 이벤트는 받아 갈 수 있도록 depth=10.
#
# 발행 시점:
#   KIND_GAME_START           — start_sampling 호출 시 game_id가 없으면 신규 발급 + 발행.
#   KIND_USER_BOARD_CONFIRMED — _job_make_and_publish_board에서 final_board 확정 후.
#   KIND_AI_MOVE              — _job_stockfish_then_robot_then_wakeup에서 best_move
#                                 확정 + robot action 성공 직후.
#   KIND_GAME_END             — UserDecision GAME_OVER 또는 stockfish empty best_move
#                                 (체크메이트/스테일메이트). game_id 리셋 동반.

std_msgs/Header header

uint8 KIND_GAME_START           = 0
uint8 KIND_GAME_END             = 1
uint8 KIND_AI_MOVE              = 2
uint8 KIND_USER_BOARD_CONFIRMED = 3

uint8 kind

# 게임 식별자. main_controller가 게임 시작 시 발급 (ISO timestamp 등 unique 문자열).
# 한 game 안에 여러 SAMPLING job (사용자 수마다 1회) 존재.
string game_id

# SAMPLING 사이클 식별자. start_sampling 호출 시점 timestamp.
string job_id

# KIND_AI_MOVE에서만 의미. 그 외엔 빈 문자열. 예: "e2e4", "a7a8q" (promotion).
string uci

# 디버깅용 FEN 스냅샷. AI 수 직후 보드 상태. 비어 있으면 미기록.
string fen

# KIND_GAME_END에서만 의미. "checkmate" / "stalemate" / "resign" / "unknown".
string result
