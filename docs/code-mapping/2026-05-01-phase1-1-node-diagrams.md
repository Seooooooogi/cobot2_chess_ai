# Phase 1-1 — Node Diagrams (4 entry points)

> 작성일: 2026-05-01 · 작성자: 인계자 (현 작업자)
> 목적: ROS2 노드 4개의 인터페이스·의존성·이슈를 코드 직독으로 매핑.
> **원칙**: 검증된 동작만 기록. 미검증은 `# verify needed` 표기 (CLAUDE.md II.1).

## Source Files

| Entry point | File | LoC | Node 여부 |
|-------------|------|-----|-----------|
| `ros2 run cobot2 main` | `src/cobot2/cobot2/main.py` | 493 | Yes — `MainController(Node)` |
| `ros2 run cobot2 stockfish` | `src/cobot2/cobot2/stockfish.py` | 201 | Yes — `AIMoveServiceNode(Node)` |
| `ros2 run cobot2 robotaction` | `src/cobot2/cobot2/robot_action.py` | 283 | Yes — `RobotActionServer(Node)` |
| `ros2 run cobot2 object` | `src/cobot2/cobot2/vision_db.py` | 217 | **No** — script-style, ROS2 미사용 |

`vision_db.py`는 ROS2 노드가 아니라 Firebase 직결 클라이언트. ROS2 메시지 버스에 참여하지 않음 (verify by grep: `from rclpy` 부재).

---

## System Interaction (cross-node)

```
                                  ┌──────────────────────────┐
                                  │   Camera (V4L2 src=3)    │
                                  └────────────┬─────────────┘
                                               │ cv2.VideoCapture
                                               ▼
                                ┌──────────────────────────────┐
                                │   vision_db.py (NOT a Node)  │
                                │   YOLO + ResNet18 추론       │
                                └────────────┬─────────────────┘
                                             │ Firebase write
                                             │ chess/board_state
                                             ▼
   ┌──────────────────────┐         ┌──────────────────────────┐
   │  CLI / 외부 스크립트  │ service │   main.py                │
   │                      │ ──────▶ │   MainController         │
   │  ros2 service call   │~/start_ │   node: main_controller  │
   └──────────────────────┘ sampling│                          │
                                    │   Firebase read/write    │
                                    │   (board_state,          │
                                    │    ui_control,           │
                                    │    chess_system)         │
                                    └──┬─────────────────┬─────┘
                                       │                 │
                                       │ srv             │ action
                                       │ StockfishMove   │ MoveChessPiece
                                       ▼                 ▼
                            ┌─────────────────┐  ┌──────────────────────┐
                            │ stockfish.py    │  │ robot_action.py      │
                            │ chess_ai_node   │  │ robot_action_server  │
                            │ Stockfish bin   │  │ DR_init, DSR_ROBOT2  │
                            │ /usr/games/...  │  │ + RG2 (Modbus 192.   │
                            └─────────────────┘  │   168.1.1:502)       │
                                                 └──────────────────────┘
                                                       │
                                                       ▼
                                                 두산 M0609 + RG2
```

**핵심 관찰**: vision → main 의 경로는 **Firebase가 메시지 버스**. ROS2 토픽 미사용 → Rule 2 위반 (지속 흐름 + 최신값 = Topic이 옳다). 외부 DB를 ROS 노드 간 통신에 쓰는 건 Rule 9 / Rule 7 명시성 측면에서도 취약.

---

## Node 1 — `MainController` (main.py)

### Identity
- **File**: `src/cobot2/cobot2/main.py:182`
- **Node name**: `main_controller`
- **Role**: 워크플로 오케스트레이터 (sample → verify → stockfish → robot → wakeup)

### ROS2 Interfaces

| Direction | Type | Name | Msg/Srv/Action | QoS | 위치 |
|-----------|------|------|----------------|-----|------|
| Server | Service | `~/start_sampling` → `/main_controller/start_sampling` | `std_srvs/Trigger` | `rmw_qos_profile_services_default` | `_on_start_sampling` |
| Client | Service | `StockfishMove` | `cobot2_interfaces/StockfishMove` | default | |
| Client | Action | `move_chess_piece` | `cobot2_interfaces/MoveChessPiece` | default | |

### Timer
- `_poll_ui_decision` — 0.2s 주기, Firebase ui_control 폴링 (line 200)

### Threads
- `_job_make_and_publish_board` (daemon, line 246) — 보드 5회 샘플링 + 다수결
- `_job_stockfish_then_robot_then_wakeup` (daemon, line 333) — stockfish 호출 → 액션 전송 → wake-up publish

### State Machine
- `_state ∈ {IDLE, SAMPLING, WAIT_DECISION, RUNNING}` 보호: `_state_lock` (mutex)
- 트리거: Service `~/start_sampling` (Trigger) 호출 → SAMPLING 진입 (`_on_start_sampling`)
- 전이: 샘플 완료 → WAIT_DECISION → UI APPROVED → RUNNING → IDLE

### External Dependencies
- **Firebase** (line 12-13, 186):
  - 읽기: `chess/board_state`, `chess/ui_control`, `chess/chess_system`
  - 쓰기: `chess/board_state`, `chess/ui_control`
  - **하드코딩 (Phase 1+ 처리)**: `FIREBASE_SERVICE_ACCOUNT_JSON = "/home/kyb/..."` (line 20), `FIREBASE_DB_URL = "https://chess-43355-..."` (line 21)
- `cobot2_interfaces`: `StockfishMove.srv`, `MoveChessPiece.action`

### Issues / Concerns

| # | Severity | 내용 | Rule |
|---|----------|------|------|
| M1-1 | ~~IMPORTANT~~ | ~~`voice_command` Topic으로 상태 변경 트리거~~ → **RESOLVED 2026-05-04**: `~/start_sampling` (Trigger) Service로 교체. | ROS2 Rule 2 |
| M1-2 | IMPORTANT | QoS 단축 표기(`10`) 사용. `QoSProfile` 명시 필요 — voice Sub QoS 제거 완료. Service는 `rmw_qos_profile_services_default` 적용. | ROS2 Rule 4 |
| M1-3 | CRITICAL | `FIREBASE_SERVICE_ACCOUNT_JSON` 하드코딩 (`/home/kyb/...`) — 다른 사용자 시스템 잔재 | CLAUDE.md II.5 |
| M1-4 | IMPORTANT | Firebase가 vision↔main 메시지 버스 — ROS2 외부 채널을 통신 경로로 사용 | ROS2 Rule 7 |
| M1-5 | MINOR | 워크플로 thread 내 `time.sleep` 폴링 (line 424,447,461) — Future 콜백 활용 권장 | ROS2 Rule 7 |
| M1-6 | ~~`# verify needed`~~ | ~~`voice_command='pass'` 수신 동작 — voice_control_node 실행 없이 main만 띄우면 무한 대기~~ → **RESOLVED 2026-05-04**: Service로 대체, voice_control_node 의존 제거. | — |
| M1-7 | ~~`# verify needed`~~ | ~~`WAKE_UP_SIGNAL` publish가 voice 노드에서 어떻게 처리되는지 미검증~~ → **RESOLVED 2026-05-04**: voice_status Pub + `_publish_wake_up()` 제거 (dead pub 해소, 옵션 a). | — |

---

## Node 2 — `AIMoveServiceNode` (stockfish.py)

### Identity
- **File**: `src/cobot2/cobot2/stockfish.py:21`
- **Node name**: `chess_ai_node`
- **Role**: Stockfish 엔진 wrapper service server

### ROS2 Interfaces

| Direction | Type | Name | Msg/Srv | QoS | 위치 |
|-----------|------|------|---------|-----|------|
| Server | Service | `StockfishMove` | `cobot2_interfaces/StockfishMove` | default | line 33 |

### Internal State
- `self.stockfish` — `Stockfish(path="/usr/games/stockfish")` 인스턴스 (line 26). 실패 시 `None`, 매 요청마다 None 체크 (line 152)
- `self.dict_memory` — 직전 board_state, last_move 추론용

### External Dependencies
- **Stockfish 바이너리**: `/usr/games/stockfish` (line 12). **하드코딩 module 상수** — Phase 1+ env 화 후보.
- 라이브러리: `stockfish` (PyPI), `cobot2_interfaces.srv.StockfishMove`

### Logic Notes
- `dict_to_fen()` (line 36-118): board_dict → FEN 변환. **castling rights / en-passant 추론은 휴리스틱** (line 95-107, 109-115).
- `get_updated_dict()` (line 120-148): best_move 적용해서 dict_memory 갱신.

### Issues / Concerns

| # | Severity | 내용 | Rule |
|---|----------|------|------|
| S1-1 | IMPORTANT | `STOCKFISH_PATH` 모듈 상수 하드코딩 — env 화 필요 | CLAUDE.md II.5 |
| S1-2 | MINOR | 서비스 QoS 미명시 (default 사용) — 의도 명시 권장 | ROS2 Rule 4 |
| S1-3 | `# verify needed` | castling/en-passant 휴리스틱 정확성 — 엣지 케이스 (king/rook 이동 후 권리 소실 추적 안 됨) | — |
| S1-4 | `# verify needed` | `dict_memory`가 노드 재시작 시 초기화 → 중간 게임에서 stockfish 노드 재시작 시 last_move 추론 실패 | — |

---

## Node 3 — `RobotActionServer` (robot_action.py)

### Identity
- **File**: `src/cobot2/cobot2/robot_action.py:214`
- **Node name**: `robot_action_server` (네임스페이스 없음 — `RobotActionServer.__init__` line 216에서 `super().__init__('robot_action_server')`만)
- **보조 노드**: `dsr_robot_node` (namespace=`dsr01`, line 265) — DR_init 글로벌 변수 주입 전용. **`executor.add_node` 호출 없음** → 실제 spin 안 함. `# verify needed`: DR_init이 spin 없는 노드만으로 작동하는지.
- **Role**: 두산 M0609 + RG2 그리퍼 동작 액션 서버

### ROS2 Interfaces

| Direction | Type | Name | Action | QoS | 위치 |
|-----------|------|------|--------|-----|------|
| Server | Action | `move_chess_piece` | `cobot2_interfaces/MoveChessPiece` | default | line 222-229 |

### Module-level State (BAD — 보고됨)
```python
# line 28 (import 시점에 Modbus connect 시도)
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
```
- `RG.__init__` 내부에서 `pymodbus.client.sync.ModbusTcpClient` 연결 시도. **import만으로 하드웨어 통신 발생** → Tier 0 callback blocking + Rule 9 안전 위반 의심.

### Hardware
- **Doosan M0609**: `DR_init.__dsr__id = "dsr01"`, `__dsr__model = "m0609"` (line 266-268, in `main()`)
- **RG2 gripper**: Modbus TCP `192.168.1.1:502` (line 26-27, **하드코딩**)
- **Tool**: TCP `GripperDA_v1_1`, weight `Tool Weight` (line 19-20)

### Goal Callback Behavior
- `goal_callback` (line 232): **무조건 ACCEPT** — command 유효성 검증 없음. 주석 line 235 "추가할 수 있습니다" 표기.
- `cancel_callback` (line 238): 무조건 ACCEPT.
- `execute_callback` (line 243, async): `chess_mover.perform_task(goal_handle)` 위임.

### Movement Logic — `MovingChessPiece`
- `__init__`: `data.json` 로드(`load_initial_config`, line 59-80) → 보드 좌표 미리 계산(`calculate`, line 94-111).
- `perform_task` (line 113-211): command 파싱 (e.g. "e2e4") → 폰 앙파상 / 킹 캐슬링 / 일반 이동 분기.
- 그리퍼 동작: `grip()` / `release()` — `gripper.get_status()[0]` 폴링 + `time.sleep(0.25)` (blocking).
- `from DSR_ROBOT2 import movej, movel, mwait, wait` — **함수 내부에서 lazy import** (line 124).

### Threading
- `MultiThreadedExecutor` (line 272) — 액션 콜백 + 로봇 상태 보고 동시 처리.

### Configuration
- `data.json` (`src/cobot2/cobot2/data.json`) — 한국어 키 (`속도`, `가속도`, `시간`, `홈_관절좌표`, `A1_좌표`, `무덤_관절좌표` 등) → Phase 1-4 인벤토리 대상.

### Issues / Concerns

| # | Severity | 내용 | Rule |
|---|----------|------|------|
| R1-1 | ~~CRITICAL~~ **RESOLVED** (2026-05-01) | Module-level `gripper = RG(...)` 제거 → `MovingChessPiece._init_gripper()` 지연 + ROBOT_MODE 분기 (default virtual) + `is_socket_open()` 가드 (Rule 7 명시적 실패). 별도 cycle 미해결: ROBOT_MODE↔DSR mode 통합 (Phase 2). | ROS2 Rule 9, Tier 0 |
| R1-2 | IMPORTANT | `goal_callback` 무조건 ACCEPT — command 유효성 검증 없음 (잘못된 chess move도 ACCEPT) | ROS2 Rule 7 |
| R1-3 | IMPORTANT | `TOOLCHARGER_IP/PORT` 하드코딩 (line 26-27) — 환경 의존 값 노드 파라미터로 외부화 필요 | ROS2 Rule 8 |
| R1-4 | IMPORTANT | E-stop / 비상정지 로직 부재 — Modbus 단절 시 페일세이프 미정의 | ROS2 Rule 9 |
| R1-5 | IMPORTANT | 액션 서버 QoS 미명시 | ROS2 Rule 4 |
| R1-6 | MINOR | `from DSR_ROBOT2 import ...` 함수 내부 import (line 124) — 이유 미문서화 | — |
| R1-7 | `# verify needed` | `data.json` 한국어 키 — virtual 모드에서 좌표 정확성 미검증 | — |
| R1-8 | `# verify needed` | `feedback_msg` 생성하나 publish 안 함 (line 248) — 액션 Feedback 미사용 의심 | ROS2 Rule 2 |

---

## Node 4 — `vision_db.py` (NOT a ROS2 Node)

### Identity
- **File**: `src/cobot2/cobot2/vision_db.py`
- **Type**: 스탠드얼론 Python 스크립트. `rclpy` import 없음.
- **Entry point**: `cobot2/object` (setup.py 매핑)
- **Role**: 카메라 → YOLO + ResNet18 보드 인식 → Firebase 직접 write

### Pipeline
```
cv2.VideoCapture(SOURCE=3)
    │
    ▼
analyze_frame():
  1. yolo_model(frame, conf=0.5, iou=0.3) → bbox 추출
  2. foot_point = ((x1+x2)//2, y2) → grid_polygons 매칭으로 square 결정
  3. get_piece_color_improved() → HSV V채널 임계값으로 White/Black
  4. ResNet18 분류 (6 클래스: Pawn/Rook/Knight/Bishop/Queen/King)
  5. board_dict[square] = "WP", "BR" 등
    │
    ▼
normalize_board_dict() → 정렬된 dict
    │
    ▼
Firebase: chess/board_state set({updated_at, piece_count, board})
```

### External Files
| 자원 | 경로 (현 하드코딩) | 상태 |
|------|------------------|------|
| YOLO weights | `/home/kyb/cobot_ws/.../best.pt` (line 18) | **부재** (Phase 1+ 처리) |
| ResNet weights | `/home/kyb/cobot_ws/.../classifier.pt` (line 19) | **부재** (Phase 1+ 처리) |
| Chess grid | `/home/kyb/cobot_ws/.../config/chess_grid.json` (line 20) | repo에 `src/cobot2/config/chess_grid.json` 존재 — 경로 불일치 |
| Firebase service account | env `FIREBASE_SERVICE_ACCOUNT_PATH` | **env 화 완료 (Phase 1 first task)** |

### Configuration
- `SOURCE = 3` (line 21) — 카메라 인덱스 하드코딩
- `ANALYZE_INTERVAL_SEC = 0.20`, `FIREBASE_UPDATE_MIN_INTERVAL_SEC = 0.20`
- `ONLY_UPDATE_ON_CHANGE = True` — board 변경 시에만 Firebase write
- `SAVE_DIR = "./captured_boards"` — `os.makedirs` at import time (line 24, **side effect at import**)

### Issues / Concerns

| # | Severity | 내용 | Rule |
|---|----------|------|------|
| V1-1 | CRITICAL | ROS2 노드 아닌데 ROS2 entry point로 등록 — vision은 ROS2 메시지 버스에서 격리됨. main과 통신은 Firebase 통해서만. | ROS2 Rule 2 |
| V1-2 | IMPORTANT | `YOLO_PATH`, `RESNET_PATH`, `GRID_PATH` 하드코딩 (`/home/kyb/...`) | CLAUDE.md II.5 |
| V1-3 | IMPORTANT | YOLO/ResNet weights 부재 — 실행 시 즉시 fail (best.pt, classifier.pt 미보유) | — |
| V1-4 | IMPORTANT | `SOURCE = 3` 카메라 인덱스 하드코딩 — env 화 또는 노드 파라미터 필요 | ROS2 Rule 8 |
| V1-5 | MINOR | `os.makedirs(SAVE_DIR)` import 시점 side effect (line 24) | — |
| V1-6 | MINOR | `cv2.imshow` 무한 루프 — headless/CI 환경 비호환 | — |
| V1-7 | `# verify needed` | piece color 분류(HSV V채널 임계값 80, 105) — 조명 변동 robustness 미검증 | — |
| V1-8 | `# verify needed` | `chess_grid.json`이 repo에 존재하지만 GRID_PATH는 `/home/kyb/...` 가리킴 → 둘이 동일한지 미검증 | — |

---

## Tier 0 / Hard Rules — Summary

| 규칙 | 위반 의심 노드 | 대응 |
|------|--------------|------|
| no fabrication | stockfish (last_move 휴리스틱), vision (color heuristic), robot (data.json 좌표) | Phase 1-2 검증, 필요 시 verify needed 마킹 |
| virtual mode first | robot_action (DR_init mode 분기 미구현 — verify) | Phase 1-3 외부 의존성 매핑에서 확인 |
| no hardcoded secrets | main.py:20, vision_db (대부분 해소), robot_action:26-27 | Phase 1-3, Phase 2 env 화 |
| append-only Firebase | main.py가 `db.reference().set()` 사용 (덮어쓰기) — UI 제어용은 OK, board_state는 history 손실 | Phase 4 마이그레이션 시 history append 설계 |

## ROS2 Rule Violations — Summary

| Rule | 노드 | 비고 |
|------|------|------|
| Rule 1 (메시지 의미) | OK | 커스텀 .action/.srv 사용. String 토픽은 voice 명령용 — Phase 1+ Service 전환 검토 |
| Rule 2 (통신 패턴) | main (voice_command Topic), vision_db (ROS2 부재) | 재설계 필요 |
| Rule 4 (QoS 명시) | 전 노드 default 사용 | Phase 2+ QoSProfile 명시 |
| Rule 7 (실패 가시성) | main (Firebase 단절 silent), robot (Modbus 단절 silent) | 페일세이프 정의 필요 |
| Rule 8 (확장성) | robot (IP 하드코딩), vision (path 하드코딩) | Phase 1+ env 화 |
| Rule 9 (안전) | robot (e-stop 부재, module-level connect) | **CRITICAL — 실제 로봇 연결 전 필수** |

---

## Phase 1 후속 작업 우선순위 (제안)

1. **Phase 1-2** (다음): 토픽/액션/서비스 인벤토리 — `ros2 topic list` 실측, voice_command 흐름 끊기 방안 설계 (M1-6 해소).
2. **Phase 1-3**: 외부 의존성 매핑 (Firebase, YOLO, Stockfish, DR_init) — 하드코딩 일괄 정리 가능 여부.
3. **Phase 1-4**: `data.json`, `config/*.json` 인벤토리.
4. **Phase 2** (식별된 우선 이슈):
   - ~~R1-1 (module-level gripper)~~ **RESOLVED 2026-05-01** (옵션 C — env 분기 + Rule 7 + 로깅)
   - **R1-1 후속**: ROBOT_MODE↔DSR mode 통합 (`m0609_rg2_bringup` 패턴 — DeclareLaunchArgument + IfCondition + onrobot ROS2 service client). launch 파일 도입 필요.
   - V1-1 (vision ROS2 미참여) — 아키텍처 결정 (Firebase 마이그레이션과 연계)
   - M1-3 (main.py Firebase 하드코딩) — 1라인 env 화

---

*문서 갱신 정책*: 검증 완료 항목은 `# verify needed` 제거. 새 발견은 해당 노드 섹션에 추가.
