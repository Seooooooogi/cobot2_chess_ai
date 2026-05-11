# cobot2_chess_ai 코드 리뷰

> **대상**: cobot2_chess_ai v1.0 (Phase 5 완료, Phase 6 실기 검증 진입 가능)
> **작성일**: 2026-05-11
> **인수인계 컨텍스트**: 협동2 A-2 팀(박윤헌·김연빈·김하균·문규혁·서재우) 프로젝트 인계물.
> 현 작업자는 원작자가 아님 — **코드 이해 → 주석 → 동작 검증 → 리팩토링** 순으로 진행.

---

## 0. 시스템 한 페이지 요약

단일 PC에서 5개 ROS2 노드가 동작:

```
[Web UI (UI.html + rosbridge:9090)]
    │  /main_controller/ui_status  (UIStatus — 상태 latched publish)
    │  /main_controller/user_decision  (UserDecision Service — 사용자 결정)
    │  /chess_ai_node/StockfishMove   (set_parameters — 엔진 설정)
    ▼
[main_controller]  (main.py)
    ├── 보드 상태 수신  ← /vision/board_state  (BoardState TRANSIENT_LOCAL)
    ├── Stockfish 호출  → /chess_ai_node/StockfishMove  (Service)
    ├── 로봇 명령       → move_chess_piece  (Action)
    └── 감사 이벤트 발행 → /main_controller/game_event  (GameEvent)
        │
        ▼
[chess_ai_node]  (stockfish.py)
    └── Stockfish 바이너리 → 최선 수 반환 (FEN 변환 + en-passant/castling 추론)

[vision_db]  (vision_db.py)
    └── Camera → YOLO → grid hit-test → HSV 색상 → ResNet → /vision/board_state 발행

[robot_action_server]  (robot_action.py)
    └── MoveChessPiece Action → Doosan M0609 (DR_init/DSR_ROBOT2) + RG2 Modbus TCP

[game_logger]  (game_logger.py)
    └── game_event + ui_status + board_state 구독 → SQLite append-only audit DB
```

### 파일별 책임 요약

| 파일 | 노드명 | 라인 | 책임 |
|------|--------|------|------|
| `chess_ai/main.py` | `main_controller` | ~730 | FSM 오케스트레이터 — 체스 턴 전체 흐름 |
| `chess_ai/robot_action.py` | `robot_action_server` | ~940 | Doosan M0609 + RG2 pick-and-place |
| `chess_ai/stockfish.py` | `chess_ai_node` | ~420 | Stockfish 엔진 래퍼 서비스 |
| `chess_ai/vision_db.py` | `vision_db` | ~420 | YOLO+ResNet 비전 파이프라인 |
| `chess_ai/game_logger.py` | `game_logger` | ~356 | SQLite 감사 로그 (append-only) |
| `chess_ai/onrobot.py` | (라이브러리) | ~210 | RG2 Modbus TCP 드라이버 |
| `chess_ai_interfaces/` | (메시지 정의) | — | 커스텀 msg/srv/action 정의 5종 |

---

## 1. 노드별 상세

<details>
<summary><strong>1.1 <code>main.py</code> — MainController [핵심 오케스트레이터]</strong></summary>

**책임**: 체스 한 턴의 전체 흐름(FSM) 조율. UI ↔ vision ↔ stockfish ↔ robot 사이의 중앙 허브.

**FSM 상태 전이**:

```
IDLE
  │ ~/start_sampling (Trigger Service)
  ▼
SAMPLING  ← daemon thread: vision/board_state 수신 대기 (3.0s timeout)
  │ board_state 수신 완료 → final_board 확정
  ▼
WAIT_DECISION  ← UI에 UIStatus 발행 (verification=True, final_board 첨부)
  │ ~/user_decision (UserDecision Service) APPROVED
  ▼
RUNNING  ← daemon thread: Stockfish 호출 → robot action 전송
  │ robot action 완료 (성공/실패 모두)
  ▼
IDLE
```

**ROS2 인터페이스**:

| 종류 | 이름 | 메시지 | 역할 |
|------|------|--------|------|
| Service (서버) | `~/start_sampling` | `Trigger` | IDLE → SAMPLING 진입 트리거 |
| Service (서버) | `~/user_decision` | `UserDecision` | WAIT_DECISION → RUNNING / RECHECKED / GAME_OVER |
| Subscriber | `vision/board_state` | `BoardState` | TRANSIENT_LOCAL latched 구독 |
| Publisher | `~/ui_status` | `UIStatus` | 상태 변경 시마다 latched publish |
| Publisher | `~/game_event` | `GameEvent` | game_logger 감사 토픽 |
| Service (클라이언트) | `/chess_ai_node/StockfishMove` | `StockfishMove` | 최선 수 요청 |
| Service (클라이언트) | `/chess_ai_node/reset_chess_state` | `Trigger` | 게임 시작 시 엔진 상태 초기화 |
| Action (클라이언트) | `move_chess_piece` | `MoveChessPiece` | 로봇 동작 요청 |

**핵심 설계 포인트**:
- `_state_lock` (mutex) 으로 FSM 전이 직렬화. 두 daemon worker thread 와 rclpy executor callback 이 동시에 state 를 건드리지 않도록 보호.
- Vision 은 TRANSIENT_LOCAL 로 latest 보드 상태를 항상 latched — main 이 SAMPLING 진입 시 이미 수신된 최신 값을 즉시 소비. 재촬영 없음 (단일 진실 원천).
- Stockfish 서비스 폴링은 `while not future.done()` 0.05s sleep 루프 — SingleThreadedExecutor 환경에서 안전하나 향후 MultiThreadedExecutor 전환 시 재검토 필요.
- `DECISION_GAME_OVER` 버튼은 UI에 아직 미배선 (주석 예약). handler 로직 자체는 완성.

**verify needed**:
- `_reset_ui_for_new_job` 의 `wait_for_service(2.0)` 이 main thread 에서 2초 블로킹 — service callback 과 같은 executor 라면 데드락 가능성.

</details>

<details>
<summary><strong>1.2 <code>stockfish.py</code> — AIMoveServiceNode [체스 엔진 래퍼]</strong></summary>

**책임**: 보드 딕셔너리(A1→H8 : WP/BK/...)를 FEN으로 변환 후 Stockfish에게 최선 수를 묻는다. 캐슬링 권한·en-passant 상태를 JSON 파일로 영속화.

**ROS2 인터페이스**:

| 종류 | 이름 | 메시지 | 역할 |
|------|------|--------|------|
| Service (서버) | `~/StockfishMove` → `/chess_ai_node/StockfishMove` | `StockfishMove` | 최선 수 응답 |
| Service (서버) | `~/reset_chess_state` → `/chess_ai_node/reset_chess_state` | `Trigger` | dict_memory + castling_rights 초기화 |
| Parameters | `depth` (1–30, default 15) | int | Stockfish 탐색 깊이 |
| Parameters | `skill_level` (0–20, default 10) | int | 엔진 실력 |
| Parameters | `default_turn` ('w'/'b', default 'w') | string | AI 착수 색상 |

**FEN 변환 흐름**:

```
pieces_dict (A1→H8 dict)
    │ dict_to_fen()
    ├─ piece_match 로 WP→P, BR→r 등 FEN 기물 코드 변환
    ├─ board[row][col] 배치 (row=0=8랭크, row=7=1랭크)
    ├─ 연속 빈 칸 숫자 압축 (FEN 직렬화)
    ├─ castling_rights (영속화, 킹/룩 이동 시 revoke)
    └─ en-passant 칸 추론 (dict_memory diff로 마지막 폰 2칸 전진 감지)
    ▼
FEN string → Stockfish.set_fen_position() → get_best_move()
    ▼
best_move (UCI 형식, 예: "e2e4") + post-move FEN
```

**영속화 (`CHESS_AI_STATE_PATH`)**:
- `dict_memory`: AI가 둔 마지막 보드 상태 — en-passant 추론의 베이스.
- `castling_rights`: "KQkq" → "K" → "-" 형태로 점차 축소.
- 정상 응답 완료 시에만 `_save_state()`. 예외 발생 시 rollback (saved_rights, saved_memory 사용).

**팀장 질문 가능 영역**:
- "왜 FEN 을 직접 생성하는가?" → stockfish PyPI 라이브러리가 직접 dict 입력을 받지 않으므로.
- "halfmove clock, fullmove number 가 항상 '0 1' 인 이유?" → 게임 이력 추적 없이 현재 보드만 분석하기 때문. 반복 수 규칙(threefold repetition)은 미지원.

**MINOR 미해결**:
- `_save_state()` non-atomic write: tmp+os.replace 패턴 권장 (OS crash 시 truncated 파일 위험).
- `SingleThreadedExecutor` 가정 미문서화.

</details>

<details>
<summary><strong>1.3 <code>vision_db.py</code> — VisionNode [비전 파이프라인]</strong></summary>

**책임**: 카메라 프레임 → YOLO 검출 → 격자 매핑 → 색상 분류 → ResNet 기물 인식 → ROS2 topic publish.

**파이프라인**:

```
VideoCapture(CAMERA_SOURCE)
    │ 매 프레임
    ▼
analyze_interval_sec (기본 0.2s) rate limit
    ▼
YOLO(conf=0.5, iou=0.3) → bounding boxes
    │ 각 박스
    ├─ foot point = (cx, y2) — 말의 발 위치
    ├─ grid polygon hit-test (cv2.pointPolygonTest) → 칸 이름 (A1~H8)
    ├─ HSV V채널 임계값 → White/Black/Unknown
    └─ ResNet18 (6-class: Pawn/Rook/Knight/Bishop/Queen/King) → 기물 종류
    ▼
board_dict (예: {"E4": "WP", "D8": "BQ"})
    │ normalize (알파벳 정렬, 문자열화)
    ├─ only_publish_on_change: 이전과 동일하면 skip
    └─ publish_min_interval_sec rate limit
    ▼
/vision/board_state (BoardState, RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1))
```

**ROS2 인터페이스**:

| 종류 | 이름 | 메시지 | 역할 |
|------|------|--------|------|
| Publisher | `vision/board_state` | `BoardState` | 인식된 보드 상태 latched publish |
| Parameters | `analyze_interval_sec` (default 0.2) | double | YOLO+ResNet 실행 간격 |
| Parameters | `publish_min_interval_sec` (default 0.2) | double | 발행 최소 간격 |
| Parameters | `only_publish_on_change` (default True) | bool | 변경 없으면 발행 생략 |
| Parameters | `frame_id` (default "chess_board") | string | BoardState Header frame_id |

**HSV 색상 분류 (`get_piece_color_improved`)**:
- ROI: 바운딩 박스의 y ∈ [+20%h, +40%h], x ∈ [+42%w, +58%w] 상단 중앙 줄기 부분
- V < 80 픽셀이 30% 초과 OR 중앙값 V < 105 → Black

**주의사항**:
- `run()` 은 blocking OpenCV 루프. `rclpy.spin()` 없음 → 파라미터 런타임 변경 미반영.
- 필수 환경변수(`YOLO_MODEL_PATH`, `RESNET_MODEL_PATH`, `CHESS_GRID_PATH`) 미설정 시 `_load_models()` 에서 `RuntimeError` fail-loud (PB-5 fix).

**verify needed**:
- V1-7: HSV 임계값 (80, 105) 조명 조건별 robustness 미검증.
- V1-8: `CHESS_GRID_PATH` 가 실제 체스판 배치와 맞는지 실측 검증 필요.

</details>

<details>
<summary><strong>1.4 <code>robot_action.py</code> — RobotActionServer [로봇 모션]</strong></summary>

**책임**: `MoveChessPiece` Action 목표를 받아 Doosan M0609 + RG2 그리퍼로 실제 pick-and-place 수행. 안전(L0/L1) 페일세이프 내장.

**클래스 구조**:
- `MovingChessPiece` — 모션 로직, 그리퍼 제어, data.json 로드, 좌표 계산
- `RobotActionServer(Node)` — ROS2 Action 서버, 목표 검증, 페일세이프 FSM
- `MockGripper` — 가상 그리퍼 (fault injection 테스트용, `GRIPPER_FAULT_MODE` env)

**목표 검증 순서 (goal_callback)**:

```
F1: _degraded 플래그 확인 (L1 페일세이프 진입 후)
    │
    ▼
V1-V13: _validate_goal()
    V1: command str, len 4 또는 5
    V2-V5: from/to 파일/랭크 유효성 (A-H, 1-8)
    V6: promotion 문자 (q/r/b/n)
    V7: from ≠ to
    V13: 5글자(promotion) HARD REJECT (execute_callback이 [0:4]만 소비 → 보드 desync 방지)
    V8-V9: pieces_dict JSON 파싱 + 비어있지 않음
    V10: from_sq에 기물 존재
    V11: 기물 코드 유효 (WP/WR/.../BK 12종)
    │
    ▼
F2: Modbus pre-flight ping (is_socket_open())
    │
    ▼
V12: atomic concurrency claim (_execution_lock, _is_executing=True)
    │
    ▼ ACCEPT
```

**페일세이프 레이어**:
- **L0**: 교시 펜던트 하드웨어 E-stop (항상 가용, SW 독립)
- **L1**: Modbus 단절 감지 → `set_safety_mode(RECOVERY, STOP)` → `_degraded=True` → 이후 목표 전체 REJECT
- **복구**: `~/reset` Service → 그리퍼 재연결 + `set_safety_mode(AUTONOMOUS, ENTER)` → `_degraded=False`

**좌표 계산 (`calculate`)**:
- A1 좌표를 기준으로 열(A~H), 행(1~8) 증분을 더해 64개 칸 전체 좌표 사전 계산.
- 세 레벨: board (착지), over (+z_interval, 안전 접근 높이), under (+3mm, 살짝 눌러잡기).

**모션 분기 (`perform_task`)**:
1. **앙파상**: 폰이 대각 이동 + 목적지 비어있음 → 잡히는 폰 위치(`to열+from행`) 먼저 tomb으로
2. **캐슬링**: 킹 2칸 이동 → 대응 룩 먼저 이동 (킹사이드: H→F, 퀸사이드: A→D)
3. **잡기**: 목적지에 기물 존재 → tomb으로 먼저
4. **공통**: from → to pick & place

**verify needed**:
- `dsr_robot_node` 가 executor에 추가되지 않음 — DR_init 이 spin 없이 정상 작동하는지 실기 검증 필요.
- `set_safety_mode(AUTONOMOUS, ENTER)` 이 SW 단독으로 성공하는지 vs 펜던트 수동 확인 필요한지.

**DEFERRED**:
- R1-3: `TOOLCHARGER_IP/PORT` 하드코딩 유지 결정 (단일 호스트 고정 IP 시나리오).

</details>

<details>
<summary><strong>1.5 <code>game_logger.py</code> — GameLoggerNode [감사 로그]</strong></summary>

**책임**: Hard Rule #6 (append-only 게임/이벤트 로그) 를 SQLite 로 영속화. Firebase 제거(Phase 5) 후 audit trail 역할을 담당.

**ROS2 인터페이스**:

| 종류 | 이름 | 메시지 | QoS |
|------|------|--------|-----|
| Subscriber | `/main_controller/game_event` | `GameEvent` | RELIABLE + TRANSIENT_LOCAL, depth=10 |
| Subscriber | `/main_controller/ui_status` | `UIStatus` | RELIABLE + TRANSIENT_LOCAL, depth=1 |
| Subscriber | `/vision/board_state` | `BoardState` | RELIABLE + TRANSIENT_LOCAL, depth=1 |

**SQLite 스키마**:

```sql
games          (game_id PK, started_at)            -- INSERT-only
game_results   (game_id PK FK, ended_at, result)   -- INSERT-only, 게임당 1행
moves          (id PK, game_id FK, ply, uci, side, fen, ts_ros)  -- INSERT-only
events         (id PK, ts_ros, ts_wall, game_id, kind, payload_json) -- INSERT-only
```

**append-only 보장**: `BEFORE UPDATE / BEFORE DELETE` TRIGGER 로 스키마 레벨에서 차단 (Hard Rule #6 강제).

**오류 처리**:
- DB open/DDL 실패 → `RuntimeError` (fail-loud, launch respawn 트리거)
- 런타임 INSERT 실패 → ERROR 로그 + `_write_failures` 카운터 증가, 게임 계속

**side 추론 (`_infer_side_from_fen`)**:
- AI가 둔 직후의 FEN 에서 turn 필드("다음 차례") 를 읽어 AI가 둔 색상 역추론.
- FEN turn="w" → AI는 "B" (흑), FEN turn="b" → AI는 "W" (백).

</details>

<details>
<summary><strong>1.6 <code>onrobot.py</code> — RG 드라이버 [그리퍼 Modbus]</strong></summary>

**책임**: OnRobot RG2/RG6 그리퍼와 Modbus TCP 로 직접 통신. ROS2 노드/서비스 없이 동기 호출만 제공.

**왜 vendor `onrobot_rg_control` 패키지를 쓰지 않는가?**
- vendor 패키지는 ROS2 node + service 인터페이스 제공 (별도 노드 실행 필요).
- `robot_action.py` 는 모션 실행 중 동기적 그리퍼 호출만 필요 → Modbus 직접 래핑이 단순.

**레지스터 맵 (slave=65)**:

| address | 내용 |
|---------|------|
| 0 | 목표 force (1/10 N) |
| 1 | 목표 width (1/10 mm) |
| 2 | control (1=grip, 8=stop, 16=grip_w_offset) |
| 258 | fingertip offset (signed, 1/10 mm) |
| 267 | 현재 width (fingertip offset 미포함) |
| 268 | status 비트 (bit0=busy, bit1=grip detected, bit2-6=safety) |
| 275 | 현재 width (fingertip offset 포함) |

**`get_status()` 반환**: `[busy, grip_detected, s1_pushed, s1_trigged, s2_pushed, s2_trigged, safety_error]`
- `robot_action.py` 의 `_wait_gripper_idle()` 은 `[0]` (busy) 만 사용.

**주의**: `get_status()` 내부에서 `print()` 직접 사용 — vendor 코드 원형 보존. ROS2 logger 미사용.

</details>

---

## 2. ROS2 인터페이스 전체 맵

```
[vision_db]
    publisher /vision/board_state (BoardState, RELIABLE+TRANSIENT_LOCAL, depth=1)
        ↓
[main_controller]
    subscriber /vision/board_state
    publisher /main_controller/ui_status (UIStatus, RELIABLE+TRANSIENT_LOCAL, depth=1)
        ↓
[rosbridge → UI.html]
    subscriber /main_controller/ui_status
    service-call /main_controller/user_decision (UserDecision)
        ↓
[main_controller]
    service-server ~/user_decision
    service-server ~/start_sampling (Trigger)
    publisher ~/game_event (GameEvent, RELIABLE+TRANSIENT_LOCAL, depth=10)
        ↓
[game_logger]
    subscriber /main_controller/game_event
    subscriber /main_controller/ui_status
    subscriber /vision/board_state

[main_controller]
    service-client /chess_ai_node/StockfishMove (StockfishMove)
        ↓
[chess_ai_node]
    service-server ~/StockfishMove
    service-server ~/reset_chess_state (Trigger)

[main_controller]
    action-client move_chess_piece (MoveChessPiece)
        ↓
[robot_action_server]
    action-server move_chess_piece
    service-server ~/reset (Trigger)  ← L1 페일세이프 복구
```

### 커스텀 메시지/서비스/액션 정의 (chess_ai_interfaces)

| 파일 | 타입 | 핵심 필드 |
|------|------|-----------|
| `BoardState.msg` | msg | `header`, `squares[]`, `pieces[]`, `piece_count` |
| `UIStatus.msg` | msg | `header`, `controller_state`, `verification`, `working`, `ai_suggested_move`, `job_id`, `final_board(BoardState)` |
| `GameEvent.msg` | msg | `header`, `kind(uint8)`, `game_id`, `job_id`, `uci`, `fen`, `result` |
| `StockfishMove.srv` | srv | req: `pieces_data`, `last_move` / res: `best_move`, `success`, `fen` |
| `UserDecision.srv` | srv | req: `decision(uint8)`, `job_id`, `corrected_board(BoardState)` / res: `accepted`, `message` |
| `MoveChessPiece.action` | action | goal: `command`, `pieces_dict` / result: `success`, `message` / feedback: `status` |

---

## 3. 핵심 비즈니스 흐름 — UI에서 로봇 그리퍼까지

```
사용자가 ros2 service call /main_controller/start_sampling 또는 Web UI 버튼
    │
    ▼ MainController._on_start_sampling()
    │   IDLE → SAMPLING
    │   새 game_id 발급 + KIND_GAME_START 이벤트 → game_logger
    │   _reset_ui_for_new_job() → /chess_ai_node/reset_chess_state
    │   daemon thread: _job_make_and_publish_board() 시작
    │
    ▼ [daemon] _job_make_and_publish_board()
    │   vision/board_state TRANSIENT_LOCAL 수신 대기 (3.0s timeout)
    │   final_board 확정 → SAMPLING → WAIT_DECISION
    │   UIStatus 발행 (verification=True, final_board) → UI 표시
    │   KIND_USER_BOARD_CONFIRMED 이벤트 → game_logger
    │
    ▼ [UI] 사용자 보드 확인 → callUserDecision(APPROVED, {})
    │
    ▼ MainController._on_user_decision()
    │   job_id 검증 + state 검증
    │   corrected_board 있으면 final_board 교체
    │   WAIT_DECISION → RUNNING
    │   UIStatus 발행 (working=True) → UI 로딩 표시
    │   daemon thread: _job_stockfish_then_robot_then_wakeup() 시작
    │
    ▼ [daemon] _job_stockfish_then_robot_then_wakeup()
    │   ├─ final_board 비면 live board_state fallback
    │   ├─ _call_stockfish(board_dict) → /chess_ai_node/StockfishMove
    │   │     Stockfish: dict→FEN→get_best_move() → best_move (예: "e2e4")
    │   │     응답: best_move + post-move FEN
    │   ├─ best_move 비어있으면 → GAME_OVER (checkmate) + KIND_GAME_END → IDLE
    │   ├─ UIStatus 발행 (ai_suggested_move=best_move)
    │   └─ _send_robot_action_and_wait(best_move, board_dict)
    │         RobotActionServer.goal_callback()
    │           → F1(degraded?) → V1-V13 검증 → F2(Modbus ping) → V12(동시성)
    │         RobotActionServer.execute_callback()
    │           → MovingChessPiece.perform_task()
    │                 [분기: 앙파상 / 캐슬링 / 잡기 / 기본 이동]
    │                 DSR_ROBOT2: movej / movel / mwait / wait
    │                 RG2: close_gripper() / open_gripper()
    │         result.success=True → goal_handle.succeed()
    │
    ▼ [daemon] 계속
    │   robot action 성공 → KIND_AI_MOVE 이벤트 (uci + fen) → game_logger
    │   finally: RUNNING → IDLE, UIStatus 발행
    │
    ▼ [game_logger] 이벤트 수신 → SQLite INSERT
    │   games, game_results, moves, events 테이블 append-only
    │
    ▼ UI: ui_status 구독 → controller_state=IDLE 감지 → 다음 턴 준비
```

모든 daemon thread 의 `finally` 블록이 IDLE 복귀를 보장 — 예외 발생 시에도 FSM 이 RUNNING에 갇히지 않음.

---

## 4. 팀장 질문 가능 / 설명 준비 영역

| # | 위치 | 영역 | 답변 준비 |
|---|------|------|-----------|
| 1 | `stockfish.py:dict_to_fen` | halfmove/fullmove가 항상 "0 1"인 이유 | 게임 이력 없이 현재 보드만 분석. 반복 수 규칙(threefold repetition) 미지원 — 경쟁 체스가 아닌 시연용이므로 수용 |
| 2 | `stockfish.py:get_best_move_callback` | "첫 호출 castling rights clamping"이 왜 필요한가 | 저장된 "KQkq" 가 실제 보드(킹/룩 위치)와 불일치 시 Stockfish가 invalid FEN 거부. 첫 호출에만 현재 보드로 클램프 |
| 3 | `vision_db.py:get_piece_color_improved` | HSV V채널 임계값 (80, 105) 근거 | 조명 조건 최적화 실험값 (원작자). 실기 검증 미완료 (V1-7) |
| 4 | `vision_db.py:analyze_frame` | foot point를 y2(하단 중앙)로 쓰는 이유 | 체스 말 머리가 인접 칸 위로 넘어올 때 머리 중심 사용 시 오매핑 발생 — 발 위치가 어느 칸에 있는지 더 정확 |
| 5 | `robot_action.py:_init_gripper` | `is_socket_open()` 직후 확인이 왜 필요한가 | pymodbus 2.x가 connect 실패를 조용히 삼킴. 이후 write_register 에서 AttributeError 발생 — fail-loud 선행 검증 (Rule 7) |
| 6 | `robot_action.py:main()` | `dsr_robot_node` 가 executor에 없는 이유 | DR_init이 node를 "소유자" 로만 참조 (g_node). 실제 spin은 `RobotActionServer` 만. Phase 6 실기 검증 필요 (verify needed) |
| 7 | `onrobot.py` | vendor `onrobot_rg_control` 과 별도 driver를 두는 이유 | vendor는 ROS2 node + service 제공 — 별도 노드 실행 필요. 동기 Modbus 직접 호출이 모션 시퀀스 안에서 더 단순 |
| 8 | `robot_action.py:calculate` | `under` 위치가 `+3mm` 고정인 이유 | 기물 착지 시 살짝 눌러서 놓기 위한 값. 실기 calibration 으로 조정 필요 (Phase 6) |
| 9 | `main.py:_call_stockfish` | `while not future.done()` 루프가 싱글스레드에서 안전한가 | main 노드는 SingleThreadedExecutor. future 콜백이 spin 중 처리됨. 단, `time.sleep(0.05)` 중 executor 가 idle — service response callback 처리 안 됨. `rclpy.spin_until_future_complete` 패턴 검토 가능 |
| 10 | `game_logger.py:_infer_side_from_fen` | FEN turn 필드에서 side를 역산하는 이유 | AI가 둔 직후 FEN이라 turn 은 "다음 차례". AI 착수색 = next_turn 의 반대 |

---

## 5. 발견된 잠재 이슈

<details>
<summary><strong>MINOR — stockfish.py _save_state() non-atomic write</strong></summary>

**위치**: `stockfish.py:_save_state()`
**현상**: `json.dump()` 가 파일을 직접 쓰다가 OS crash 발생 시 truncated JSON → 다음 시작 시 로드 실패.
**권장 수정**: tmp 파일에 쓴 후 `os.replace()` 로 원자적 교체.

```python
import tempfile
tmp = self._state_path + ".tmp"
with open(tmp, "w", encoding="utf-8") as f:
    json.dump(data, f, indent=2)
os.replace(tmp, self._state_path)
```

</details>

<details>
<summary><strong>MINOR — main.py _reset_ui_for_new_job() 2초 메인 스레드 블로킹</strong></summary>

**위치**: `main.py:_reset_ui_for_new_job()` → `wait_for_service(2.0)`
**현상**: `_on_start_sampling` service callback 내에서 2초 블로킹. rclpy executor 가 SingleThreadedExecutor 라면 이 2초 동안 다른 콜백이 처리되지 않음.
**권장**: `wait_for_service` 를 daemon thread로 분리하거나 timeout을 0으로 줄이고 unavailable 시 warn만.

</details>

<details>
<summary><strong>verify needed — dsr_robot_node executor 미등록</strong></summary>

**위치**: `robot_action.py:main()` L919
**현상**: `dsr_robot_node = Node('dsr_robot_node')` 를 생성해 `DR_init.__dsr__node` 에 할당하나 `executor.add_node()` 를 호출하지 않음. DR_init 내부 코드가 이 노드를 spin 없이 정상 작동하는지 가상 모드에서는 미검증.
**Phase 6 실기 검증 항목**.

</details>

<details>
<summary><strong>verify needed — pymodbus get_status hang (half-open socket)</strong></summary>

**위치**: `robot_action.py:_wait_gripper_idle()` → `onrobot.py:get_status()`
**현상**: deadline 루프 안에서 `get_status()` 자체가 hang 하면 deadline 이 영구 무시됨. pymodbus 소켓 레벨 timeout 기본값이 있으나 `onrobot.RG` 에서 명시적으로 설정하지 않음.
**권장**: `ModbusTcpClient(timeout=...)` 명시 또는 `client.set_timeout()` 추가 (Phase 6 실기 검증).

</details>

<details>
<summary><strong>verify needed — V1-7 / V1-8 (비전 실기 calibration)</strong></summary>

**V1-7**: `get_piece_color_improved()` HSV 임계값 (80, 105) 과 ROI 비율 (0.2/0.4/0.42/0.58) — 조명 조건 robustness 미검증.
**V1-8**: `CHESS_GRID_PATH` 가 실제 새 체스판 위치에 맞게 설정되어 있는지 확인. Phase 6 calibration 시 `chess_grid.json` 갱신 필요.

</details>

<details>
<summary><strong>INFO — 환경변수 / 설정 정리 완료 항목</strong></summary>

Phase 5 완료로 정리된 dead 항목들 (이미 제거됨):
- `VOICE_INPUT_ENABLED` — voice stack 제거 (2026-05-04)
- `LLM_CHESS_LOGIC_ENABLED` — Input Policy (신규 추가 금지)
- `FIREBASE_SERVICE_ACCOUNT_PATH` / `FIREBASE_DATABASE_URL` — Phase 5 Firebase 의존 제거
- `OPENAI_API_KEY` — Input Policy

</details>

<details>
<summary><strong>INFO — 보안 0건 (시크릿 하드코딩 없음)</strong></summary>

- `.env` gitignored, `.env.example` placeholder만 포함. ✓
- Firebase 키 코드 내 하드코딩 없음 (Phase 5 완전 제거). ✓
- TOOLCHARGER_IP/PORT 하드코딩 — **사용자 결정으로 유지** (단일 고정 IP 시나리오, Phase 6 멀티호스트 확장 시 재검토). ✓

</details>

---

## 6. 실행 환경 및 빌드 참조

### source 순서 (모든 셸)

```bash
source /opt/ros/humble/setup.bash
source /home/rokey/cobot2_chess_ai/install/setup.bash
source /home/rokey/cobot2_chess_ai/.venv/bin/activate
set -a && source src/chess_ai/.env && set +a   # vision_db 필수 env 주입
```

### 빌드 + shebang 패치

```bash
cd /home/rokey/cobot2_chess_ai
colcon build --packages-select chess_ai --symlink-install
for ep in main stockfish robotaction object gamelogger; do
  sed -i "1s|^#!.*|#!/home/rokey/cobot2_chess_ai/.venv/bin/python|" \
    install/chess_ai/lib/chess_ai/$ep
done
```

### 노드 실행 (각 별도 셸)

```bash
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=virtual          # arm bringup
ros2 run chess_ai stockfish                                                # 체스 엔진
ros2 run chess_ai robotaction                                              # 로봇 동작
ros2 run chess_ai object                                                   # 비전 인식
ros2 run chess_ai gamelogger                                               # 감사 로그 (선택)
ros2 run chess_ai main                                                     # 오케스트레이터
```

또는 launch 파일로 일괄 실행:

```bash
ros2 launch chess_ai chess_system.launch.py
```

---

## 7. 한 줄 정리

각 노드는 **단일 책임** + **명확한 ROS2 인터페이스 계약** + **공유 상태는 FSM lock + daemon thread** 로 분리. 안전 정지는 Action goal 검증(V1-V13) + L0 하드웨어 E-stop + L1 SW 페일세이프(Modbus 단절 → RECOVERY) 3중 레이어. Firebase 의존은 Phase 5 에서 완전 제거되어 LAN-only SQLite 감사 로그로 대체. Phase 6 실기 검증이 남은 유일한 필수 관문.
