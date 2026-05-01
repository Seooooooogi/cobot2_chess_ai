# cobot2_chess_ai v1.0

ROS2 + 두산 M0609 협동로봇 기반 AI 체스 시스템. 비전(YOLO) + 음성 인식 + Stockfish/OpenAI 엔진 + Firebase 로그 + Web UI.
본 리포지토리는 협동2 A-2 팀(박윤헌·김연빈·김하균·문규혁·서재우) 프로젝트의 **인계물**이며,
현 작업자는 원작자가 아니므로 **코드 이해 → 주석 → 동작 검증 → 리팩토링** 순서를 지킨다.

## Hard Rules (never bend)

> **동일 내용**이 [.claude/rules/ai-constitution.md](.claude/rules/ai-constitution.md)에도 정의됨 (단일 source 동기화 필수).
> ROS2 설계 원칙은 추가로 `~/.claude/rules/ros2-principles.md` 자동 적용 (특히 Rule 9 안전).

1. **no speculation in comments** — 인계받은 코드의 미검증 동작을 주석/docstring으로 단정 금지.
   확인되지 않은 부분은 `# verify needed` 또는 docstring에 "확인 필요" 명시.

2. **baseline before refactor** — 리팩토링 전 현재 동작을 기록(로그/스크린샷/스모크 시퀀스).
   변경 후 회귀 발견 시 즉시 롤백.

3. **virtual mode first** — 실제 M0609(`mode:=real`) 연결 전 virtual 모드에서
   동일 시퀀스가 에러 없이 완료되어야 한다. virtual 검증 없이 실제 하드웨어 동작 금지.

4. **no fabrication** — vision 추론 실패, 음성 인식 실패, Firebase 로그 결측은
   명시적으로 표기/마킹. 임의 보간/대체값 채우기 금지.

5. **no hardcoded secrets** — Firebase service account JSON, OpenAI API key,
   로봇 IP/credentials는 환경변수/`.env`로만 로드. 리포 내 `.env`,
   `*-firebase-adminsdk-*.json` 커밋 금지.

6. **append-only Firebase logs** — 게임 기록·이벤트 로그 덮어쓰기 금지 (감사 추적성).

7. **no AI attribution in git** — commit message, PR description, AUTHORS 등 git 추적
   attribution에 Claude/Copilot/GPT 등 AI assistant를 `Co-Authored-By`,
   contributor, footer로 추가 금지. 위반 시 git 히스토리에 비가역으로 남는다.

## Quick Ref

- Build: `colcon build --packages-select cobot2 cobot2_interfaces`
- Source: `source install/setup.bash`
- Entry points (`cobot2` 패키지):
  - `ros2 run cobot2 main` — 통합 실행 (`cobot2/main.py`)
  - `ros2 run cobot2 stockfish` — 체스 엔진 노드 (`cobot2/stockfish.py`)
  - `ros2 run cobot2 robotaction` — 로봇 동작 노드 (`cobot2/robot_action.py`)
  - `ros2 run cobot2 object` — 비전 인식 노드 (`cobot2/vision_db.py`)
- Voice: `cobot2/run_voice.sh`
- Configs: `src/cobot2/config/*.json`, `src/cobot2/cobot2/data.json`
- Models: `src/cobot2/models/`
- Interfaces: `src/cobot2_interfaces/{action,srv}/`
- Tests: pytest (현재 미정 — `docs/DEVELOPMENT_ROADMAP.md` Backlog).
- Roadmap: `docs/DEVELOPMENT_ROADMAP.md` (Phase 0~4)
- Decisions: `docs/decisions/README.md` (ADR-001)

## Secrets Policy

- `.env`, Firebase service account JSON, OpenAI API key, 두산 로봇 IP/credentials는
  환경변수로만 로드 — 코드 하드코딩 금지.
- `.env`는 커밋 금지 — `.env.example`이 템플릿(실제 값 없음).
- 새 키 추가 시 `.env.example` placeholder + 환경변수 로더 코드 동시 업데이트.
- 주의: 현 `setup.py`는 `share/cobot2/.env`를 데이터 파일로 등록. `.env`가 없으면
  빌드 실패 — 신규 환경에서는 `.env.example`을 복사해 `src/cobot2/.env` 생성 필요.

## Dev Conventions

- **인계자 작업 흐름**: ① 코드 매핑 (노드/토픽/액션/서비스 다이어그램) → ② 주석·docstring 추가
  (검증된 동작만) → ③ virtual 모드 동작 검증 → ④ 식별된 개선점 리팩토링.
- 새 기능/리팩토링: opt-in 환경변수, 기본 OFF.
- 로그: append-only — Firebase, 로컬 파일 모두 덮어쓰기 금지.
- 커밋: 한 논리적 변경 = 한 커밋(독립 revert 가능). 명시적 요청 시에만 생성.
- 커밋 메시지: subject(첫 줄)는 영어, body는 한국어/영어 혼용 가능.
- `--no-verify` 사용 금지.

## Compact Instructions

세션 압축 시 보존할 항목:
1. Hard Rules
2. 현재 활성 브랜치 / 미커밋 파일 목록
3. 진행 중 태스크와 상태
4. 조사 중인 활성 에러/버그
5. Dev Conventions (인계자 작업 흐름 4단계)
6. 이번 세션에 수정한 파일 경로
