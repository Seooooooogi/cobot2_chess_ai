# MainController → Web UI 상태 토픽 (Phase 5 sub-phase D1).
#
# QoS (publisher 측): RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(depth=1).
# 페이지 로드 직후 latched 메시지로 최신 상태 즉시 표시.
#
# header.stamp:    상태 변경 시점 (FSM transition / 필드 업데이트 발생 시각).
# header.frame_id: 사용 안 함 (빈 문자열 허용).

std_msgs/Header header

# 사용자 검증 단계 (WAIT_DECISION). True인 동안 UI는 final_board 표시 + decision 입력 대기.
bool verification

# 로봇 / 엔진 작업 진행 중 (RUNNING). UI는 working 애니메이션 표시.
bool working

# stockfish 추천 수 (e.g. "e2e4"). 비어 있으면 미확정. "게임 종료" = checkmate/stalemate.
string ai_suggested_move

# SAMPLING 시점 ISO timestamp. UI는 user_decision Service 호출 시 첨부 (D2).
string job_id

# 검증/실행에 사용된 보드. squares=[] 이면 "아직 없음" (IDLE/SAMPLING 진입 직후).
BoardState final_board

# FSM 상태 enum. 문자열 매핑은 main.py와 일치.
uint8 STATE_IDLE = 0
uint8 STATE_SAMPLING = 1
uint8 STATE_WAIT_DECISION = 2
uint8 STATE_RUNNING = 3
uint8 controller_state
