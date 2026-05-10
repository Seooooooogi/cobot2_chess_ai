# Architecture Decision Records

이 코드베이스에 영향을 준 결정을 기록한다. 다음 작업 시 항목 추가:
새 의존성 도입, 기존 패턴 교체, 데이터 모델 변경, 노드 구조 재구성, Hard Rules 변경.

## Template
```markdown
# ADR-NNN: [Decision Title]
## Context: 왜 이 결정이 필요한가
## Decision: 무엇을 선택했는가
## Consequences: 트레이드오프, 알려진 제약
```

## Decisions

### ADR-001: 인계자 모드 — Hard Rules 채택 및 작업 흐름 정의 (2026-04-30)

**Context**:
- 본 리포(`cobot2_chess_ai`)는 협동2 A-2 팀(박윤헌·김연빈·김하균·문규혁·서재우)
  프로젝트의 인계물. 원작자가 아닌 사용자가 코드 이해 → 주석 → 검증 → 리팩토링을 수행.
- 글로벌 `~/.claude/rules/ai-constitution.md`는 별도 프로젝트(M0609_RG2_Integration /
  LeRobot 데이터셋 빌드)용 규칙(예: `no data leakage`, `baseline required` for VLA model)을
  포함 — 본 프로젝트에 그대로 적용 시 mismatch.

**Decision**:
- **Stack** — Language: Python 3 / ROS2 ament_python, DB: Firebase, AI: YOLOv8/v11 +
  Stockfish + OpenAI API + 음성 인식, Interface: Web UI (HTML/CSS/Firebase) + ROS2 노드,
  Deployment: Local Ubuntu PC + Doosan M0609.
- **Hard Rules** — 글로벌 참조 대신 프로젝트 로컬 7개 항목 채택 (`CLAUDE.md` 참조).
  - 인계자 특화: `no speculation in comments`, `baseline before refactor`
  - 글로벌 공통: `virtual mode first`, `no fabrication`, `no hardcoded secrets`,
    `no AI attribution in git`
  - 프로젝트 특화: `append-only Firebase logs`
- **Scope** — 1주일 이내 작업 사이클. `docs/DEVELOPMENT_ROADMAP.md`로 인계자 흐름을
  Phase 0(부트스트랩) → 1(매핑) → 2(주석) → 3(virtual 검증) → 4(리팩토링) 단계화.

**Consequences**:
- 글로벌 `ai-constitution.md` 변경 시 자동 반영되지 않음 — 수동 동기화 필요.
- test suite 생략(Backlog로 이동) — 1주일 스코프 내 baseline 기록(Phase 3-6)이
  회귀 검증의 1차 수단. 본격적 pytest 도입은 차기 사이클.
- `baseline before refactor` 규칙으로 모든 리팩토링 전 동작 기록(로그/스모크 시퀀스)
  의무화. 기록 없는 변경은 회귀 검증 불가.
- `setup.py`가 `.env`를 share 데이터 파일로 등록하여, `.env` 부재 시 colcon build 실패.
  신규 환경에선 `.env.example` → `src/cobot2/.env` 복사 필수 (ROADMAP Phase 0-1).

---

### ADR-002: Firebase → rosbridge 마이그레이션 + SQLite 영속 로그 분리 (2026-05-10)

**Context**:
- 현 master 아키텍처는 Firebase Realtime DB를 **세 가지 역할 동시 수행**으로 사용:
  1. 메시지 버스 — `chess/board_state` (vision↔main), `chess/ui_control` (main↔UI)
  2. 엔진 설정 동기화 — `chess/chess_system` (UI 슬라이더 ↔ stockfish 호출)
  3. 게임 기록 영속 (Hard Rule #6 append-only logs)
- 이로 인한 ROS2 설계 위반:
  - **V1-1 (Rule 2)**: `vision_db.py`가 ROS2 노드가 아니며 (rclpy import 0), Firebase write로만 main과 통신.
  - **M1-4 (Rule 7)**: Firebase가 vision↔main 메시지 버스 역할 — 외부 채널을 transport로 사용 = 명시성·실패 가시성 위반.
  - `UI.html:114` Firebase Web SDK apiKey 하드코딩 — `no hardcoded secrets` 위반 (Hard Rule #5).
- 추가 비용: 외부 클라우드 의존, 0.2s 폴링 (`_poll_ui_decision` 타이머 + M1-5 워크플로 폴링), 200ms write throttle, service account JSON 관리.
- `ros-humble-rosbridge-suite 2.0.5` 시스템 의존성 이미 설치됨 — 추가 시스템 패키지 불요.

**Decision**:

1. **실시간 메시지 버스**: Firebase → ROS2 native (Topic/Service/Action) + 브라우저 ↔ ROS2는 `rosbridge_websocket` (port 9090, LAN only bind).
   - `chess/board_state` (read latest) → `/vision/board_state` Topic, RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1).
   - `chess/ui_control.user_decision` (poll) → `/chess/user_decision` Service (Rule 2 — 응답 필요).
   - `chess/ui_control.corrected_board` → `/chess/correct_board` Service.
   - `chess/ui_control.{working, verification, status}` → `/chess/ui_status` Topic, RELIABLE + TRANSIENT_LOCAL.

2. **엔진 설정 (Rule 8)**: `chess/chess_system` (depth/difficulty/turn) → stockfish 노드의 ROS2 declared parameters. UI는 rosbridge `set_parameters` API 호출. 메시지가 아닌 노드 파라미터로 표현 — 환경 의존 값은 parameter라는 Rule 8 원칙 부합.

3. **영속 로그 분리** (Hard Rule #6 충족): 신규 ROS2 노드 `game_logger.py`가 `/vision/board_state`, `/main_controller/ui_status`, `/main_controller/game_event`를 구독하여 SQLite (`~/.local/share/cobot2_chess_ai/game_log.db`)에 append-only 기록. 스키마 4 테이블 (`games`, `game_results`, `moves`, `events`). WAL 모드 + SQLite TRIGGER 8개로 UPDATE/DELETE 스키마 레벨 차단 (코드 컨벤션이 아닌 강제). `move_executed`는 별도 토픽 신설 대신 `GameEvent.KIND_AI_MOVE`로 통합 — 토픽 표면 최소화.

   **2026-05-10 Rule 5 정정**: 초안의 `/chess/move_executed`, `/chess/game_event`, `/chess/ui_status`는 기능 분류 글로벌 네임스페이스로 Rule 5 위반. 실제 구현은 발행자 노드 namespace 하위로 정정 — `~/ui_status` (main_controller), `~/game_event` (main_controller). 코드는 처음부터 정정된 형태로 머지됨.

   **SQLite 스키마 (구현 완료)**:
   ```sql
   games          (game_id PK, started_at)               -- INSERT-only
   game_results   (game_id PK FK, ended_at, result)      -- INSERT-only, 게임당 1행
   moves          (id PK, game_id FK, ply, uci, side, fen, ts_ros)
   events         (id PK, ts_ros, ts_wall, game_id, kind, payload_json)
   -- + 8 TRIGGER (no_update_*, no_delete_*) for append-only enforcement
   ```

4. **인증/접근 범위**: LAN only 가정. rosbridge_server 무인증. 외부 노출 요구 시 별도 nginx + WSS + auth 검토 (본 ADR 범위 외).

5. **마이그레이션 전략**: 5 sub-phase 점진 (DEVELOPMENT_ROADMAP.md Phase 5 A→E). 단계별 회귀 검증.

**Consequences**:

- **Positive**:
  - V1-1, M1-4, M1-5 (부분) RESOLVED — ROS2 설계 위반 해소.
  - 외부 클라우드 의존 제거 — 오프라인/LAN 동작.
  - `UI.html:114` 하드코딩 apiKey 자동 제거.
  - `firebase_admin` Python 의존성 제거 (`vision_db.py`, `main.py`).
  - 0.2s 폴링 → push (latency / CPU 개선).
  - 200ms write throttle 제거 — QoS로 backpressure.

- **Negative**:
  - **NAT/원격 접근 상실** — Firebase는 인터넷 어디서든 접근 가능, rosbridge는 LAN only. 원격 데모/모니터링 시나리오 발생 시 재검토 필요.
  - **Hard Rule #6 충족 비용** — 영속 로그가 별도 노드/스키마/SQLite로 분리됨. UI ↔ Firebase 단일 경로의 단순함 손실 → 노드 1개 + DB 파일 1개 추가.
  - **rosbridge 무인증** — LAN 범위 외 노출 시 별도 인증 레이어 필요.
  - **마이그레이션 비용** — `UI.html` JS 재작성 (Firebase Web SDK → roslibjs), `main.py` FirebaseClient 제거, `vision_db.py` 노드화. 5 sub-phase 회귀 검증 필요.

- **Constraints**:
  - Hard Rule #2 (`baseline before refactor`) — 각 sub-phase 진입 전 동작 baseline 기록.
  - Hard Rule #3 (`virtual mode first`) — 각 sub-phase virtual 모드 회귀 검증 후 다음 단계.
  - Hard Rule #6 — `game_logger.py` 코드에서 UPDATE/DELETE SQL 금지 (assertion / 코드 리뷰 enforce).

- **Reversal cost**: high — UI/main/vision 3 컴포넌트 코드 변경 + 인터페이스 파일 신규 (`BoardState.msg`, `UserDecision.srv`, `CorrectBoard.srv`). Sub-phase 단위 점진이므로 단일 단계 롤백은 가능.

- **Open subordinate decision**: YOLO inference runtime — venv (현 default) / Docker 하이브리드 / 풀 컨테이너 중 결정 보류. Phase 5 sub-phase A 구현 형태에 영향 (ROADMAP Open Decisions 참조).

**Status (2026-05-10): IMPLEMENTED** — Phase 5 sub-phase A→E 모두 master에 머지.
- A (vision_db ROS2 node + BoardState.msg): commit b46a1b6.
- B (main board_state subscriber): commit 3653be8.
- C (rosbridge_websocket + UI roslibjs): commit 1a3b428.
- D1 (UIStatus topic + board_state.set 제거): commit 830284e.
- D2 (UserDecision Service): commit 620a34b.
- D3 (stockfish parameter 단일 경로): commit e80407e.
- D4 (voice_message dead field): commit 4ca3cf4.
- E (Firebase 일괄 제거 + game_logger + SQLite): 8 commits — GameEvent.msg, game_logger 노드, launch+setup, StockfishMove.fen 추가, main.py refactor, vision_db Firebase 제거, UI Firebase Web SDK 제거, .env+ADR 갱신.
