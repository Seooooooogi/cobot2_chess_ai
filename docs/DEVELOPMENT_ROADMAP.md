# cobot2_chess_ai — Development Roadmap

**Stance**: 인계자 모드 (원작자 X). Hard Rules `no speculation in comments` / `baseline before refactor` / `virtual mode first`를 모든 phase에 적용.

**구조 변경 (2026-05-10)**: 인계 시점 1주일 Phase 0~4 스코프 → Phase 5 (Firebase → rosbridge 마이그레이션) + Phase 6 (실기 검증) 신설로 재구성. Phase 3 (Virtual Verification) 마일스톤 폐지 → 상시 규율로 통합. R/S/M/V 이슈 추적 표 명시.

---

## Status Summary

| Phase | 제목 | 상태 |
|-------|------|------|
| 0 | Environment Bootstrap | ✅ COMPLETE |
| 1 | Code Mapping | ✅ COMPLETE |
| 2 | Annotation & Documentation | ✅ COMPLETE |
| 3 | ~~Virtual Verification~~ | 폐지 → 상시 규율 |
| 4 | Refactoring (R/S/M/V) | ◐ 17/21 RESOLVED. R1-3, R1-4 OPEN |
| 5 | Firebase → rosbridge 마이그레이션 | ⚪ NEW (Phase 4 잔여 RESOLVED 후 진입) |
| 6 | 실기 검증 `mode:=real` | ⚪ NEW (Phase 5 완료 후 진입) |

---

## Phase 0: Environment Bootstrap — ✅ COMPLETE

- [x] 0-1. `.env.example` → `src/cobot2/.env` 복사 + 실제 키 채움.
- [x] 0-2. pip 의존성 설치 (`ultralytics`, `stockfish`, `firebase-admin`).
- [x] 0-3. `colcon build --packages-select cobot2 cobot2_interfaces` 성공.
- [x] 0-4. 4 entry point 기동 검증.

해결 커밋: `abe8830 chore(phase-0): bootstrap complete + memory updates`.

---

## Phase 1: Code Mapping — ✅ COMPLETE

- [x] 1-1. 노드 다이어그램 (4 entry point) — `docs/code-mapping/2026-05-01-phase1-1-node-diagrams.md`.
- [x] 1-2. ROS2 토픽/액션/서비스 인벤토리 — `docs/code-mapping/2026-05-01-phase1-2-topic-inventory.md`.
- [x] 1-3. 외부 의존성 매핑 — `docs/code-mapping/2026-05-01-phase1-3-external-deps.md`.
- [x] 1-4. 설정 파일 인벤토리 — `docs/code-mapping/2026-05-01-phase1-4-config-inventory.md`.

해결 커밋: `f5d53a7 docs(phase-1): node interaction diagrams (1-1 mapping)` 외 Phase 1 doc 4종.

---

## Phase 2: Annotation & Documentation — ✅ COMPLETE

- [x] 2-1, 2-2. 노드 docstring (module/class/function): `68b02ec` (stockfish), `490c825` (robot_action), `bf3b6a5` (vision_db).
- [x] 2-3. README — Phase 0 부트스트랩 / 실행 시퀀스 (인계자 흐름).
- [△] 2-4. `# verify needed` 마커 — Phase 5 진입 시 또는 Phase 6 실기 검증과 함께 처리.

---

## Phase 3: ~~Virtual Verification~~ → 상시 규율 (마일스톤 폐지)

**재정의**: Hard Rule #3 (virtual mode first)는 PR/refactor 시 virtual 회귀 점검 필수 — **상시 규율**. 통합 baseline 1회 기록은 풀스택(vision+Firebase+UI) 가용 시점에만 가능 → Phase 5 완료 후 Phase 6 진입 직전에 통합 baseline 시도.

`demo/virtual-rviz` 브랜치 (영구 보존, 영상 촬영 후 보류)가 robot+stockfish virtual 통합 일부 검증 역할 수행.

---

## Phase 4: Refactoring (R/S/M/V) — ◐ MOSTLY COMPLETE

우선순위: **안전성 > 가독성 > 구조 > 성능**.

### Status 표

| ID | 의미 | 파일 | 상태 | 해결 커밋 |
|----|------|------|------|----------|
| R1-1 | 모듈 수준 gripper Modbus 연결 | `robot_action.py` | ✅ RESOLVED | `ccff5d0` |
| R1-2 | goal_callback 검증 부재 (Rule 7) | `robot_action.py` | ✅ RESOLVED | `8c4dcd4` |
| R1-3 | TOOLCHARGER_IP/PORT 하드코딩 (Rule 8) | `robot_action.py:26-27` | ◐ **DEFERRED** (의식적 유지, 2026-05-10) | — |
| R1-4 | Modbus 단절 페일세이프 (Rule 9 ⚠) | `robot_action.py` | ⚪ **OPEN** | — |
| R1-5 | 액션 서버 QoS 미명시 (Rule 4) | `robot_action.py` | ✅ RESOLVED | `f6a4955` |
| R1-8 | 미사용 feedback_msg 제거 | `robot_action.py` | ✅ RESOLVED | `fbacec1` |
| S1-1 | STOCKFISH_PATH 하드코딩 | `stockfish.py` | ✅ RESOLVED | `bca2bec` |
| S1-2 | 서비스 QoS 미명시 (Rule 4) | `stockfish.py` | ✅ RESOLVED | `7ee4722` |
| S1-3 | castling 휴리스틱 부정확 | `stockfish.py` | ✅ RESOLVED | `43e8f0a` |
| S1-4 | dict_memory 재시작 손실 | `stockfish.py` | ✅ RESOLVED | `43e8f0a` |
| M1-1 | voice_command 무한 대기 (Rule 2) | `main.py` | ✅ RESOLVED | `90078c5` |
| M1-2 | 서비스/액션 QoS 미명시 (Rule 4) | `main.py` | ✅ RESOLVED | `cec0ed0` |
| M1-3 | Firebase 경로 하드코딩 | `main.py` | ✅ RESOLVED | `edd15bc` |
| M1-4 | Firebase가 메시지 버스 (Rule 7) | `main.py` | ⚪ OPEN → Phase 5 |
| M1-5 | workflow time.sleep 폴링 | `main.py` | ⚪ OPEN → Phase 5 부분 중첩 |
| M1-6 | voice_control_node 의존 | `main.py` | ✅ RESOLVED | `90078c5` |
| M1-7 | voice_status dead pub | `main.py` | ✅ RESOLVED | `90078c5` |
| V1-1 | ROS2 노드 미등록 (Rule 2) | `vision_db.py` | ⚪ OPEN → Phase 5 |
| V1-2 | 모델 경로 하드코딩 | `vision_db.py` | ✅ RESOLVED | `70ab813` |
| V1-4 | CAMERA SOURCE 하드코딩 | `vision_db.py` | ✅ RESOLVED | `70ab813` |

해결 17 / 전체 21. 추가 미체크 커밋: `4-voice` voice stack 삭제 (M1-1/M1-6/M1-7 동반 RESOLVED, `90078c5`).

### Phase 4 잔여 (Phase 5 진입 전 처리)

- ~~**R1-3** TOOLCHARGER_IP / TOOLCHARGER_PORT 환경변수화. Rule 8 (확장성) 위반.~~ **DEFERRED 2026-05-10** — 사용자 결정으로 하드코딩 유지. 이유: 단일 호스트 + 고정 그리퍼 IP 시나리오에서 환경변수화 이득 < 변경 비용. Phase 6 실기 다호스트 확장 시 재검토.
- **R1-4** Modbus 단절 시 페일세이프 (E-stop 또는 hold) 정의. **Rule 9 Tier 0 동급 안전**. 옵션 검토 진행 중 (대화 참조).

### Phase 4 OPEN → Phase 5에서 자동 RESOLVED

- **M1-4** (Firebase as message bus) — Phase 5 sub-phase B/C에서 ROS2 토픽/Service로 전환되며 해결.
- **V1-1** (vision_db not a node) — Phase 5 sub-phase A에서 `rclpy.Node` 노드화로 해결.
- **M1-5** 부분 중첩 — Phase 5 sub-phase D에서 `_poll_ui_decision` 폴링 제거.

### MINOR 미해결 (handoff 임시 넘버링, 별도 트랙)

`stockfish.py` 임시 #3, #4, #6 — 공식 ID 아님. 우선순위 낮음.
- atomic write (`_save_state` non-atomic)
- `wait_for_service(2.0)` 메인 스레드 블로킹
- SingleThreadedExecutor 가정 미문서

---

## Phase 5: Firebase → rosbridge 마이그레이션 (NEW)

**Why**: V1-1 (Rule 2) + M1-4 (Rule 7) 위반 해소. 외부 클라우드 의존 제거. `UI.html:114` 하드코딩 apiKey 제거. 0.2s 폴링 → push (latency/CPU 개선). Hard Rule #6 (append-only logs)은 별도 영속 레이어로 보존. ADR-002 참조.

**전제 조건**: Phase 4 R1-3, R1-4 RESOLVED.

### A. vision_db → ROS2 Publisher 노드화

> 본 sub-phase는 venv 가정. **Open Decision "YOLO Runtime"** 결과에 따라 재설계 가능.

- `src/cobot2/cobot2/vision_db.py` 재작성: `rclpy.Node` 상속, `/vision/board_state` Publisher.
- `src/cobot2_interfaces/msg/BoardState.msg` 신규 — 8×8 board dict + `std_msgs/Header`.
- QoS: RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1) (Rule 4 명시, late join 대응).
- Frame: `chess_board` (REP-105 컨벤션 검토 필요 — `# verify needed`).
- V1-1 RESOLVED.

### B. main.py 수신 경로 전환

- `BOARD_STATE_PATH` Firebase 읽기 → `/vision/board_state` Subscriber.
- `_sample_and_decide()` (5회 sample) 로직: ROS2 토픽 5회 수신 후 vote, 또는 단일 latched + timeout (재설계 필요).
- 일부 M1-5 폴링 제거.

### C. rosbridge_server launch + UI board_state 전환

- `src/cobot2/launch/chess_system.launch.py`에 `rosbridge_websocket_launch.xml` 또는 직접 노드 추가 (port 9090, LAN only bind).
- `UI.html`: Firebase Web SDK board_state 구독 → `roslibjs.Topic('/vision/board_state')` 구독으로 교체 (이중 운용 단계 — ui_control은 아직 Firebase).
- 회귀 검증: 기존 Firebase 흐름 보존 + ROS2 경로 추가.

### D. ui_control + chess_system → ROS2 Service / Parameter

- `src/cobot2_interfaces/srv/UserDecision.srv` 신규 — Request: `decision` ("APPROVED"/"RE-CHECKED"/"GAME-OVER"), `corrected_board` (str JSON, optional) / Response: `accepted` (bool).
- `src/cobot2_interfaces/srv/CorrectBoard.srv` 신규 — Request: `board` (str JSON) / Response: `success`, `message`.
- `chess_system` (depth/difficulty/turn) → stockfish 노드의 ROS2 parameter (`depth`, `skill_level`, `default_turn`). UI는 rosbridge `set_parameters` API 사용.
- `main.py`: `_poll_ui_decision` 타이머 제거 → Service handler. M1-5 RESOLVED.
- `main.py` FSM 전이: Service 호출 기반.
- `UI.html`: roslibjs Service 호출 + parameter API.

### E. Firebase 코드 제거 + 영속 로그 신규 노드

- `firebase_admin` import / `FirebaseClient` 클래스 / Firebase Web SDK 전체 제거.
- `.env`에서 `FIREBASE_*` 키 제거. `.env.example` 동반 갱신.
- 신규: `src/cobot2/cobot2/game_logger.py` — ROS2 logger 노드.
  - 구독: `/vision/board_state`, `/chess/ui_status`, `/chess/move_executed`, `/chess/game_event`.
  - 영속: SQLite (`~/.local/share/cobot2_chess_ai/game_log.db`, env: `CHESS_AI_LOG_DB_PATH`).
  - 스키마 (초안):
    - `events` (`id INTEGER PK, ts_ros REAL, ts_wall TEXT, kind TEXT, payload_json TEXT`)
    - `moves` (`game_id INTEGER, ply INTEGER, uci TEXT, side TEXT, fen TEXT, ts REAL`)
    - `games` (`id INTEGER PK, started_at TEXT, ended_at TEXT, result TEXT`)
  - **Hard Rule #6 (append-only)**: UPDATE/DELETE 금지. WAL 모드, atomic transaction.
- M1-4 RESOLVED. V1-1 confirmed RESOLVED.

**Exit**: V1-1, M1-4 RESOLVED + virtual 모드 풀 스택 통합 시퀀스 1회 성공 + baseline 기록 (`outputs/baseline/phase5-integration/`).

---

## Phase 6: 실기 검증 `mode:=real` (NEW, handoff Priority 5)

**전제 조건**: Phase 4 OPEN 모두 RESOLVED + Phase 5 완료 + virtual baseline 기록 완료.

- [ ] 6-1. Hardware preflight — `ping 192.168.1.100` (M0609), `ping 192.168.1.1` (RG2 Modbus).
- [ ] 6-2. 새 체스판 좌표 calibration — `data.json` `posnumx_interval` 등 실측.
- [ ] 6-3. V1-7 — HSV color robustness (조명/색상 변동 검증).
- [ ] 6-4. V1-8 — `chess_grid` 좌표 매칭 (calibration → 그리드 좌표 변환 검증).
- [ ] 6-5. 통합 시퀀스 1회 (`mode:=real`) — vision → main → stockfish → robot_action → 실 그리퍼 동작.
- [ ] 6-6. baseline 기록 (`outputs/baseline/phase6-real/`).

**Exit**: 통합 실기 시퀀스 1회 성공 + baseline 아티팩트 저장.

---

## Open Decisions

### YOLO Inference Runtime — venv vs Docker (Phase 5 진입 전 결정 필요)

**현 상태**: vision_db.py가 `.venv` 내 `ultralytics` + `torch 2.11.0+cu130` + `torchvision` 직접 import. RTX 4070 Laptop (8GB) + driver 580.142. nvidia container runtime 등록됨.

**배경**: 본 리포는 인계물. 인계 시 재현성 중요. 한편 venv 경로는 동작 중이며 Phase 5 마이그레이션과 직교. 본 결정은 Phase 5 sub-phase A 구현 형태에 영향.

| 측면 | A: venv (현 상태) | B: 하이브리드 (inference만 Docker) | C: 풀 컨테이너 |
|------|-----|-----|-----|
| 재현성 | 호스트 CUDA/glibc 의존 | 추론 환경 고정 | 모든 환경 고정 (최고) |
| GPU 접근 | 직접 (간단) | nvidia-container-toolkit 필요 (✅ 등록됨) | 동일 |
| 추론 성능 | zero overhead | IPC 1회 비용 | zero overhead |
| 메모리 (8GB GPU) | venv 단일 프로세스 공유 | 컨테이너 분리, OOM 격리 | 동일 |
| 의존성 충돌 | ROS2 + ultralytics + DR_init 공존 | inference 격리 | 가장 깨끗 |
| 개발 iteration | edit → colcon build → run (가장 빠름) | inference 변경 시 docker rebuild | 모든 변경 시 rebuild |
| 빌드 복잡도 | uv pip + colcon + shebang patch | + Dockerfile 1개 + IPC 정의 | + Dockerfile 다수 + compose |
| Phase 5 영향 | sub-phase A: rclpy 추가만 | sub-phase A: wrapper + 컨테이너 | sub-phase A 외 + 오케스트레이션 |
| DRCF Docker와 일관성 | 이중 패러다임 | 자연스러운 컨테이너 사용 (3개) | 가장 일관 |
| 인계자 재현성 | `.venv` 재구축 + GPU 환경 일치 | 이미지 pull → ROS2 venv만 재구축 | `docker compose up` 1회 |
| 공급망 보안 | ultralytics 호스트 직접 접근 | inference 격리 | 모두 격리 |
| 모델 swap | `.pt` 파일 교체 | volume mount 교체 | 동일 |
| Hard Rule 6 (재현성) | 약함 | 강함 | 가장 강함 |
| 추가 작업량 | 0 | Dockerfile 1 + IPC + wrapper | 다수 + compose + base image |

**결정 가이드**

- **A 채택**: 단일 개발자 + 단일 호스트 + 시연 일정 임박 + 인계 후 추가 작업 가능. Phase 5 단순화.
- **B 채택**: 재현성 / 의존성 충돌 우려 결정적. inference 환경 동결. Phase 5에 +1 sub-phase 추가 (vision 컨테이너 + thin ROS2 wrapper).
- **C 채택**: 다개발자 / CI 도입 / GPU 박스 분리. **Phase 7 신설** 권장 (Phase 5/6 venv로 마치고 컨테이너화는 독립 트랙).

**Default 가정**: A (venv). 변경 시 Phase 5 sub-phase A 재설계 필요.

**결정 (2026-05-10)**: **A 채택**. Phase 5 sub-phase A는 `vision_db.py`에 `rclpy.Node` 상속 + `/vision/board_state` Publisher 추가 (단순). B/C는 Phase 6 또는 그 이후 별도 트랙으로 재검토 가능.

### Other

- **shebang 영속화** — wrapper / setup.py hook / cmake flag. Phase 5 진입 시 결정.
- **ROBOT_MODE↔DSR mode 통합** — m0609_rg2_bringup 패턴. 임시 mitigation = startup WARN.
- **DR_init bringup 통합** — 시점 미정.
- **vendored 패키지 freeze 정책** — README 명시 (`git clone <commit>`) 미작성.
- **외부 인증/원격 접근** — LAN only 결정. 원격 노출 요구 시 nginx + WSS + auth 별도 검토.

---

## Remaining Issues (Phase 5 외 별도 트랙)

- **`onrobot_rg_control/package.xml`** ROS1 `message_runtime` 키 — `src/onrobot-ros2/` gitignored, 커밋 불가. 별도 처리 필요.
- **`.env` dead key 정리** — `VOICE_INPUT_ENABLED`, `LLM_CHESS_LOGIC_ENABLED`, `OPENAI_*`.
- **`UI.html:114`** Firebase Web SDK apiKey 하드코딩 — Phase 5 sub-phase E에서 자동 정리.

### 환경 quirk
- `ros2 topic hz /dsr01/joint_states` hang on TRANSIENT_LOCAL pub — 데이터 흐름은 정상.
- `dsr_hw_interface2` "Depreciated API" 경고 — 동작 무영향.

---

## Backlog (스코프 외)

- 실제 로봇 데모 영상 — Phase 6 완료 후.
- pytest 테스트 스위트 구축.
- CI (colcon build + lint) 설정.
- v2 도메인 확장 (체스 외).
- 다른 팀 멤버 코드 의도 인터뷰 — `verify needed` → 사실 승격.
- `demo/virtual-rviz` 브랜치 — 영상 촬영 후 보류 (사용자 결정, 삭제 안 함).

---

## Reference

- ADR: `docs/decisions/README.md` (ADR-001 인계자 모드, ADR-002 Firebase → rosbridge).
- Hard Rules: `CLAUDE.md` / `.claude/rules/ai-constitution.md`.
- ROS2 설계 원칙: `~/.claude/rules/ros2-principles.md` (Rule 9 Tier 0 동급 안전).
- handoff: `~/.claude/projects/-home-rokey-cobot2-chess-ai/memory/session-handoff.md`.
