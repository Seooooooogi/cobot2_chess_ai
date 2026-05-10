# cobot2_chess_ai — Development Roadmap

**Stance**: 인계자 모드 (원작자 X). Hard Rules `no speculation in comments` / `baseline before refactor` / `virtual mode first`를 모든 phase에 적용.

**구조 변경 (2026-05-10)**: 인계 시점 1주일 Phase 0~4 스코프 → Phase 5 (Firebase → rosbridge 마이그레이션) + Phase 6 (실기 검증) 신설로 재구성. Phase 3 (Virtual Verification) 마일스톤 폐지 → 상시 규율로 통합. R/S/M/V 이슈 추적 표 명시.

**Phase 5 완료 (2026-05-10)**: sub-phase A→E 13 커밋 (`3653be8..0630f29`)으로 master 머지. M1-4 / M1-5 / V1-1 RESOLVED. Firebase 의존성 (admin SDK + Web SDK + apiKey + 0.2s 폴링) 완전 제거. game_logger SQLite append-only TRIGGER로 Hard Rule #6 스키마 레벨 보장. **Phase 6 진입 가능 상태**.

---

## Status Summary

| Phase | 제목 | 상태 |
|-------|------|------|
| 0 | Environment Bootstrap | ✅ COMPLETE |
| 1 | Code Mapping | ✅ COMPLETE |
| 2 | Annotation & Documentation | ✅ COMPLETE |
| 3 | ~~Virtual Verification~~ | 폐지 → 상시 규율 |
| 4 | Refactoring (R/S/M/V) | ✅ COMPLETE (M1-4/M1-5/V1-1 RESOLVED via Phase 5; R1-3 DEFERRED) |
| 5 | Firebase → rosbridge 마이그레이션 | ✅ COMPLETE (sub-phase A→E 모두 완료, master `0630f29`) |
| 6 | 실기 검증 `mode:=real` | ⚪ NEXT (Phase 5 완료 → 진입 가능) |

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
| R1-4 | Modbus 단절 페일세이프 (Rule 9 ⚠) | `robot_action.py` | ✅ RESOLVED | `765082c` |
| R1-5 | 액션 서버 QoS 미명시 (Rule 4) | `robot_action.py` | ✅ RESOLVED | `f6a4955` |
| R1-8 | 미사용 feedback_msg 제거 | `robot_action.py` | ✅ RESOLVED | `fbacec1` |
| S1-1 | STOCKFISH_PATH 하드코딩 | `stockfish.py` | ✅ RESOLVED | `bca2bec` |
| S1-2 | 서비스 QoS 미명시 (Rule 4) | `stockfish.py` | ✅ RESOLVED | `7ee4722` |
| S1-3 | castling 휴리스틱 부정확 | `stockfish.py` | ✅ RESOLVED | `43e8f0a` |
| S1-4 | dict_memory 재시작 손실 | `stockfish.py` | ✅ RESOLVED | `43e8f0a` |
| M1-1 | voice_command 무한 대기 (Rule 2) | `main.py` | ✅ RESOLVED | `90078c5` |
| M1-2 | 서비스/액션 QoS 미명시 (Rule 4) | `main.py` | ✅ RESOLVED | `cec0ed0` |
| M1-3 | Firebase 경로 하드코딩 | `main.py` | ✅ RESOLVED | `edd15bc` |
| M1-4 | Firebase가 메시지 버스 (Rule 7) | `main.py` | ✅ RESOLVED | `f0f58f3` (Phase 5 E) |
| M1-5 | workflow time.sleep 폴링 | `main.py` | ✅ RESOLVED | `620a34b` (Phase 5 D2) |
| M1-6 | voice_control_node 의존 | `main.py` | ✅ RESOLVED | `90078c5` |
| M1-7 | voice_status dead pub | `main.py` | ✅ RESOLVED | `90078c5` |
| V1-1 | ROS2 노드 미등록 (Rule 2) | `vision_db.py` | ✅ RESOLVED | `3653be8` (Phase 5 B) + `1a3b428` (C) + `7460208` (E) |
| V1-2 | 모델 경로 하드코딩 | `vision_db.py` | ✅ RESOLVED | `70ab813` |
| V1-4 | CAMERA SOURCE 하드코딩 | `vision_db.py` | ✅ RESOLVED | `70ab813` |

해결 17 / 전체 21. 추가 미체크 커밋: `4-voice` voice stack 삭제 (M1-1/M1-6/M1-7 동반 RESOLVED, `90078c5`).

### Phase 4 잔여

- ~~**R1-3** TOOLCHARGER_IP / TOOLCHARGER_PORT 환경변수화.~~ **DEFERRED 2026-05-10** — 사용자 결정으로 하드코딩 유지. 단일 호스트 + 고정 그리퍼 IP 시나리오에서 환경변수화 이득 < 변경 비용. Phase 6 실기 다호스트 확장 시 재검토.
- ~~**R1-4** Modbus 단절 페일세이프.~~ **RESOLVED 2026-05-10** (`765082c`) — L0 하드웨어 E-stop + L1 SW 페일세이프 (Option 1 STOP + Option 3 HOLD: `set_safety_mode(RECOVERY, STOP)` bounded). 회복: `~/reset` Service 두 단계 (gripper reconnect + safety mode AUTONOMOUS 복귀). MockGripper로 가상 검증. 실기 검증은 Phase 6.

### Phase 4 OPEN → Phase 5에서 자동 RESOLVED (✅ 완료)

- **M1-4** (Firebase as message bus) — Phase 5 sub-phase B (`3653be8`) + D1 (`830284e`) + E (`f0f58f3`)에서 ROS2 토픽/Service로 전환 완료.
- **V1-1** (vision_db not a node) — Phase 5 sub-phase B (`3653be8`)에서 `rclpy.Node` 노드화 + E (`7460208`)에서 Firebase dual-write 제거로 완료.
- **M1-5** (`_poll_ui_decision` 폴링) — Phase 5 sub-phase D2 (`620a34b`)에서 UserDecision Service로 대체 완료.

### MINOR 미해결 (handoff 임시 넘버링, 별도 트랙)

`stockfish.py` 임시 #3, #4, #6 — 공식 ID 아님. 우선순위 낮음.
- atomic write (`_save_state` non-atomic)
- `wait_for_service(2.0)` 메인 스레드 블로킹
- SingleThreadedExecutor 가정 미문서

---

## Phase 5: Firebase → rosbridge 마이그레이션 — ✅ COMPLETE (2026-05-10)

**Why**: V1-1 (Rule 2) + M1-4 (Rule 7) 위반 해소. 외부 클라우드 의존 제거. `UI.html:114` 하드코딩 apiKey 제거. 0.2s 폴링 → push (latency/CPU 개선). Hard Rule #6 (append-only logs)은 별도 영속 레이어로 보존. ADR-002 참조.

**전제 조건**: Phase 4 R1-3, R1-4 RESOLVED. ✅ 충족.

**완료 시점**: master HEAD `0630f29` (2026-05-10). 13 커밋 (`3653be8..0630f29`). Sub-phase별 feat 브랜치에서 작업 후 master 머지.

### A+B. vision_db → ROS2 Publisher 노드화 + main.py 수신 경로 전환 — ✅ DONE

- ✅ `src/cobot2/cobot2/vision_db.py` `VisionNode(Node)`로 재작성. `vision/board_state` Publisher (private namespace, Rule 5 상대 경로).
- ✅ `src/cobot2_interfaces/msg/BoardState.msg` — squares[]/pieces[] 평행 배열 + Header + piece_count.
- ✅ QoS: RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1) (late join 대응).
- ✅ Frame: `chess_board`.
- ✅ `main.py`: Firebase polling 제거 → `/vision/board_state` Subscriber. `_sample_and_decide()` 5-frame vote 보존.
- 커밋: `3653be8` (Phase 5 sub-phase B).

### C. rosbridge_server launch + UI board_state 전환 — ✅ DONE

- ✅ `chess_system.launch.py`에 `rosbridge_websocket` 노드 추가 (port 9090, LAN bind).
- ✅ `UI.html`: roslibjs `/vision/board_state` 구독으로 board 표시.
- 커밋: `1a3b428` (Phase 5 sub-phase C).

### D. ui_control + chess_system → ROS2 Service / Parameter — ✅ DONE

- ✅ **D1**: `msg/UIStatus.msg` (controller_state STATE_* 4종, verification, working, ai_suggested_move, job_id, BoardState final_board). `main_controller/ui_status` 토픽 latched 발행. UI roslibjs 구독. 커밋 `830284e`.
- ✅ **D2**: `srv/UserDecision.srv` (uint8 decision APPROVED/RECHECKED/GAME_OVER + job_id + corrected_board). `main.py` `_poll_ui_decision` 타이머 제거 → Service handler. **M1-5 RESOLVED**. 커밋 `620a34b`.
- ✅ **D3**: stockfish 노드 ROS2 parameter (`depth`, `skill_level`, `default_turn`). UI는 rosbridge `set_parameters`. **`StockfishMove.srv` 단순화**: depth/skill_level/turn Request 필드 제거 (sentinel 0 충돌 회피, parameter 단일 경로). 커밋 `e80407e`.
- ✅ **D4**: UI Firebase ui_control listener + voice_message dead 필드 제거. 커밋 `4ca3cf4`.
- *Note*: `srv/CorrectBoard.srv`는 별도 정의하지 않고 `UserDecision.srv`의 `corrected_board(BoardState)` 필드로 통합 (단일 채널 단순화).

### E. Firebase 코드 제거 + 영속 로그 신규 노드 — ✅ DONE

- ✅ `firebase_admin` import / `FirebaseClient` 클래스 / Firebase Web SDK 전체 제거. 커밋 `f0f58f3` (main) + `7460208` (vision_db) + `d4c6c00` (UI).
- ✅ `.env`에서 `FIREBASE_*` 키 제거 + `.env.example` 갱신 + ADR-002 IMPLEMENTED 상태 기록. 커밋 `e2f2601`.
- ✅ 신규 `src/cobot2/cobot2/game_logger.py` — ROS2 logger 노드. 커밋 `3720557` (구현) + `365efe0` (launch+setup).
  - 구독: `/main_controller/game_event` (depth=10, latched) + `/main_controller/ui_status` (depth=1) + `/vision/board_state` (depth=1).
  - 영속: SQLite (`~/.local/share/cobot2_chess_ai/game_log.db`, env: `CHESS_AI_LOG_DB_PATH`).
  - 실제 스키마 (4 테이블):
    - `games` (`game_id PK, started_at`)
    - `game_results` (`game_id PK FK, ended_at, result`)
    - `moves` (`id PK, game_id FK, ply, uci, side, fen, ts_ros`)
    - `events` (`id PK, ts_ros, ts_wall, game_id, kind, payload_json`)
  - **Hard Rule #6**: SQLite TRIGGER 8개 (4 테이블 × UPDATE/DELETE 거부) — 스키마 레벨 보장. WAL + foreign_keys.
- ✅ 신규 `msg/GameEvent.msg` — kind enum (GAME_START/GAME_END/AI_MOVE/USER_BOARD_CONFIRMED) + game_id/job_id/uci/fen/result. 커밋 `485d94f`.
- ✅ `srv/StockfishMove.srv`에 post-move `fen` Response 필드 추가 (audit 용). 커밋 `ee71f71`.
- ✅ `main.py`: GameEvent를 FSM 경계에서 발행 + `_current_game_id` 라이프사이클 관리.
- ✅ docstring stale Firebase 언급 정리. 커밋 `0630f29`.
- **M1-4 RESOLVED. V1-1 confirmed RESOLVED**.

### 통합 테스트 — `scripts/sim_game_logger.py`

- 하드웨어/Firebase/rosbridge/카메라/Stockfish 의존성 0.
- tempfile DB + 합성 GameEvent/UIStatus/BoardState publisher로 16건 검증 (행 수 / ply 증가 / FEN 기반 side 추론 / event kinds / TRIGGER 8 거부).
- `feat/sim-game-logger` 브랜치 (cc5552f) — master 머지 대기.

**Exit (충족)**: V1-1, M1-4, M1-5 RESOLVED + 통합 테스트 16/16 PASS. 풀스택 virtual baseline은 Phase 6 진입 직전 별도 기록 예정 (`outputs/baseline/phase5-integration/`).

---

## Phase 6: 실기 검증 `mode:=real` — ⚪ NEXT

**전제 조건**: Phase 4 OPEN 모두 RESOLVED ✅ + Phase 5 완료 ✅ + virtual baseline 기록 (Phase 5 → 6 전환 직전 1회).

- [ ] 6-0. virtual 풀스택 baseline 기록 (`outputs/baseline/phase5-integration/`) — 풀 launch + UI + sim_game_logger 통합 1회. **Phase 6 진입 직전 게이트**.
- [ ] 6-1. Hardware preflight — `ping 192.168.1.100` (M0609), `ping 192.168.1.1` (RG2 Modbus).
- [ ] 6-2. 새 체스판 좌표 calibration — `data.json` `posnumx_interval` 등 실측.
- [ ] 6-3. V1-7 — HSV color robustness (조명/색상 변동 검증).
- [ ] 6-4. V1-8 — `chess_grid` 좌표 매칭 (calibration → 그리드 좌표 변환 검증).
- [ ] 6-5. M0609 RECOVERY → AUTONOMOUS 전이 SW alone 가능 vs 티치펜던트 manual 필요 검증 (R1-4 실기 검증).
- [ ] 6-6. DSR `set_safety_mode` 실기 round-trip 동작.
- [ ] 6-7. pymodbus `get_status` 자체 hang 가능성 (onrobot.RG socket timeout 미설정).
- [ ] 6-8. 통합 시퀀스 1회 (`mode:=real`) — vision → main → stockfish → robot_action → 실 그리퍼 동작.
- [ ] 6-9. baseline 기록 (`outputs/baseline/phase6-real/`).

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

**결정 (2026-05-10)**: **A 채택**. Phase 5는 venv 가정으로 완료됨. B/C는 Phase 6 또는 그 이후 별도 트랙으로 재검토 가능 (인계자 재현성 / GPU 박스 분리 요건 발생 시).

### Other

- **shebang 영속화** — wrapper / setup.py hook / cmake flag. 미결.
- **ROBOT_MODE↔DSR mode 통합** — m0609_rg2_bringup 패턴. 임시 mitigation = startup WARN.
- **DR_init bringup 통합** — 시점 미정.
- **vendored 패키지 freeze 정책** — README 명시 (`git clone <commit>`) 미작성.
- **외부 인증/원격 접근** — LAN only 결정. 원격 노출 요구 시 nginx + WSS + auth 별도 검토.
- **demo/virtual-rviz 처리** — 영상 촬영 후 보류/삭제 결정.
- **머지된 feat 브랜치 정리** — 7개 (`feat/cleanup-voice-msg`, `feat/firebase-removal-game-logger`, `feat/stockfish-params`, `feat/ui-status-topic`, `feat/user-decision-srv`, `feat/web-ui-ros2-bridge`, 머지 후 `feat/sim-game-logger`). 사용자 명시 승인 필요.

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
