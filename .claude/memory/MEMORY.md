# MEMORY — cobot2_chess_ai

> 프로젝트 지식 베이스. 인계자 작업 중 발견·검증한 사실 누적.
> 항목은 한 줄 인덱스 + 하위 파일 (memory가 길어지면 분할). 현재는 인라인.

## Project Snapshot (as of 2026-05-01)

- **Stage**: 인계 직후 (Phase 0 진입). 원작자 부재. 코드 이해·검증 단계.
- **Stack**: ROS2 + 두산 M0609 + RG2 그리퍼 + YOLO(ultralytics) + Stockfish + Firebase + Web UI
- **Input policy**: 텍스트 + Stockfish만. OpenAI/음성 신규 금지 (Tier 0 III).
- **Build**: `colcon build --packages-select cobot2 cobot2_interfaces`
- **Source**: `source install/setup.bash`

## Entry Points (`cobot2` 패키지)

| 명령 | 진입점 | 역할 (확인 필요) |
|------|--------|----------------|
| `ros2 run cobot2 main` | `cobot2/main.py` | 통합 실행 — verify needed |
| `ros2 run cobot2 stockfish` | `cobot2/stockfish.py` | 체스 엔진 노드 — verify needed |
| `ros2 run cobot2 robotaction` | `cobot2/robot_action.py` | 로봇 동작 노드 — verify needed |
| `ros2 run cobot2 object` | `cobot2/vision_db.py` | 비전 인식 노드 — verify needed |

> 위 "역할" 컬럼은 파일명에서 추정. **Phase 1 (Code Mapping) 완료 전까지 단정 금지** (Tier 0 #1).

## External Dependencies (확인 필요)

- **Firebase**: service account JSON (`.env`로 경로 주입). 게임 기록·이벤트 로그 저장. Append-only.
- **YOLO model**: `src/cobot2/models/`에 가중치 파일. 체스말 8x8 인식.
- **Stockfish 바이너리**: 시스템 설치 또는 Python `stockfish` 패키지 — 위치 verify needed.
- **두산 M0609**: TCP/Modbus 연결. 로봇 IP / 그리퍼 IP는 `.env` 환경변수.
- ~~OpenAI API~~: **scope 제외** (Tier 0 III). 기존 호출 부분은 Phase 4 검토 대상.
- ~~음성 인식~~: **scope 제외**. `run_voice.sh` + 관련 노드는 Phase 4 검토.

## Configs

- `src/cobot2/config/*.json` — 설정 파일군. 키별 용도 verify needed (Phase 1-4).
- `src/cobot2/cobot2/data.json` — 데이터 파일. 용도 verify needed.
- `src/cobot2/.env` — secrets (Firebase JSON 경로, 로봇 IP). 커밋 금지.
- `src/cobot2/.env.example` — 템플릿 (placeholder만).

## Interfaces

- `src/cobot2_interfaces/action/` — 커스텀 Action 정의 (verify needed: 어느 노드가 사용?)
- `src/cobot2_interfaces/srv/` — 커스텀 Service 정의

## Current Active Concerns

- 인계 직후 — virtual 모드 빌드·기동 미검증.
- `setup.py`가 `share/cobot2/.env`를 데이터 파일로 등록 → `.env` 부재 시 빌드 실패.
- ROS2 토픽/Action/Service 인벤토리 미작성 (Phase 1-2).
- baseline 시퀀스 미기록 (Phase 3-6).

## Known Risks (인계 시점)

- **Tier 0 위반 의심**: vision/음성/Firebase 결측 시 fabrication 코드 존재 가능성 — Phase 1-3에서 점검.
- **하드코딩 의심**: 로봇 IP, Firebase 키 경로 등 — Phase 1-3에서 grep.
- **ROS2 설계 위반 의심**: `ros2-principles.md` Rule 1(메시지 의미), Rule 4(QoS 명시), Rule 9(안전 신호 패턴) 점검 필요.

## Roadmap

- 단기 (1주, ~2026-05-08): `docs/DEVELOPMENT_ROADMAP.md` Phase 0~4 순차 진행.
- 중기 (Backlog): 실제 M0609 데모, pytest 스위트, CI(colcon + lint).

## Quick References

- Hard Rules: `.claude/rules/ai-constitution.md` (프로젝트) + `~/.claude/rules/ai-constitution.md` (전역)
- ROS2 Design: `~/.claude/rules/ros2-principles.md`
- Workflow: `~/.claude/rules/development-workflow.md`
- Roadmap: `docs/DEVELOPMENT_ROADMAP.md`
- Decisions: `docs/decisions/README.md` (ADR-001)

## Update Discipline

- 새 사실 검증 → 즉시 추가, "verify needed" 마커 제거.
- 잘못된 추정 발견 → 메모리 즉시 정정 (전역 ai-constitution.md VI.3).
- 항목 5개 초과로 늘면 토픽별 파일 분리 (`memory/<topic>.md`).
