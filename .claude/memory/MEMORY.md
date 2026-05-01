# MEMORY — cobot2_chess_ai

> 프로젝트 지식 베이스. 인계자 작업 중 발견·검증한 사실 누적.

## Project Snapshot (as of 2026-05-01)

- **Stage**: **Phase 0 완료, Phase 1 진입 직전**. 인계자 모드 (원작자 부재).
- **Stack**: ROS2 Humble + 두산 M0609 + RG2 그리퍼 + YOLO(ultralytics) + Stockfish 14.1 + Firebase + Web UI
- **Input policy**: 텍스트 + Stockfish만. OpenAI/음성 신규 금지 (Tier 0 III). 기존 코드는 Phase 1+ 처리.
- **Git**: 초기 커밋 `4e7e53e` 존재 (`master` 브랜치). Author: Seooooooogi <dlwotjraks@gmail.com>.

## Build & Run Environment (검증됨, Phase 0 산출물)

### Source 순서 (반드시 이 순서)

```bash
source /opt/ros/humble/setup.bash
source /home/rokey/M0609_RG2_Integration/install/setup.bash
source /home/rokey/cobot2_chess_ai/install/setup.bash
source /home/rokey/cobot2_chess_ai/.venv/bin/activate
```

### Build

```bash
cd /home/rokey/cobot2_chess_ai
rm -rf build install log
colcon build --packages-select cobot2_interfaces cobot2 --symlink-install
# 재빌드마다 shebang 패치 필요 (영속 해결 Phase 1+ 미정):
for ep in main stockfish robotaction object; do
  sed -i "1s|^#!.*|#!/home/rokey/cobot2_chess_ai/.venv/bin/python|" \
    install/cobot2/lib/cobot2/$ep
done
```

### venv 정보

- 위치: `/home/rokey/cobot2_chess_ai/.venv` (uv venv + `--system-site-packages`)
- venv 내 pip 미포함 — 항상 `uv pip install` 사용 (`pip install`은 system pip 잡힘)
- 설치된 패키지: ultralytics, stockfish, firebase-admin, **pymodbus<3** (현재 2.5.3 — 3.x 금지)

## Entry Points (`cobot2` 패키지)

| 명령 | 진입점 | 검증 상태 |
|------|--------|----------|
| `ros2 run cobot2 main` | `cobot2/main.py` | import-only smoke PASS, 실행은 `voice_command='pass'` 대기로 멈춤 (verify needed) |
| `ros2 run cobot2 stockfish` | `cobot2/stockfish.py` | import-only smoke PASS, 실행 미검증 |
| `ros2 run cobot2 robotaction` | `cobot2/robot_action.py` | import-only smoke PASS, 실행 시 `gripper = RG(...)` Modbus 연결 시도 |
| `ros2 run cobot2 object` | `cobot2/vision_db.py` | import-only smoke PASS, 실행 시 YOLO 가중치 부재 + Firebase 미설정으로 즉시 fail |

## External Dependencies

- **DR_init (Doosan SDK)**: `/home/rokey/M0609_RG2_Integration/install/dsr_common2/lib/python3.10/site-packages/DR_init.py`. chess_ai 자체에 미포함 — M0609 워크스페이스 source 필수. **통합 방향**: M0609_RG2_Integration의 bringup과 통합 (사용자 결정 2026-05-01).
- **Firebase**: service account JSON `/home/rokey/secrets/kybfirebase.json` (외부 보관, chmod 600). **Phase 1+에서 로컬 DB로 마이그레이션 예정**.
- **YOLO 가중치**: `models/hello_rokey_8332_32.tflite` (wakeword 모델) 1개만 존재. **`best.pt` 부재** — `vision_db.py:18` 하드코딩 경로(`/home/kyb/.../best.pt`)로 참조 → 실행 시 fail.
- **Stockfish**: `/usr/games/stockfish` (apt install stockfish, 14.1).
- **두산 M0609**: `DOOSAN_ROBOT_IP=192.168.137.100` (.env). RG2 그리퍼 Modbus: `192.168.1.1:502` (`robot_action.py:28-29` 하드코딩).
- ~~OpenAI API~~: scope 제외 (Input Policy). 기존 호출 위치: `STT.py`, `voice_control_node.py`, `run_voice.sh`. Phase 1+ 비활성화 cycle 대상.
- ~~음성 인식~~: scope 제외. `STT.py` (Whisper), `voice_control_node.py` (wakeword), `run_voice.sh`. venv_voice 삭제됨, 재구성 시 `docs/baseline/voice_venv_freeze.txt` 참조.

## Hardcoded Paths (Phase 1+ 처리 대상)

| 파일 | 라인 | 하드코딩 | 처리 우선순위 |
|------|------|---------|------------|
| `vision_db.py` | 30 | `FIREBASE_SERVICE_ACCOUNT_JSON = "/home/kyb/cobot_ws/..."` | **최우선** (code-reviewer IMPORTANT) |
| `vision_db.py` | 18 | `YOLO_PATH = "/home/kyb/cobot_ws/.../best.pt"` | 높음 (가중치 부재 + env 화) |
| `main.py` | 20-21 | `FIREBASE_SERVICE_ACCOUNT_JSON`, `FIREBASE_DB_URL` | 높음 |
| `voice_control_node.py` | 22 | `WAKEWORD_MODEL_PATH = "/home/kyb/.../hello_rokey_8332_32.tflite"` | 음성 비활성화 cycle 대상 |
| `stockfish.py` | 12 | `STOCKFISH_PATH = "/usr/games/stockfish"` | 낮음 (표준 경로, externalize만) |
| `robot_action.py` | 28-29 | `TOOLCHARGER_IP = "192.168.1.1"`, port 502 | 중간 |
| `UI.html` | 114 | Firebase Web SDK apiKey | Firebase migration 시 정리 |

## Configs

- `src/cobot2/config/chess_grid.json` — 체스 보드 8x8 픽셀 좌표 (시크릿 아님)
- `src/cobot2/cobot2/data.json` — 데이터 파일, 용도 verify needed (Phase 1)
- `src/cobot2/.env` — 활성 키 7개: DOOSAN_ROBOT_IP, ROBOT_MODE, VOICE_INPUT_ENABLED, LLM_CHESS_LOGIC_ENABLED, LOG_LEVEL, YOLO_MODEL_PATH, STOCKFISH_PATH. OpenAI/Firebase 미활성. **커밋 금지**.
- `src/cobot2/.env.example` — 템플릿 (placeholder만, 커밋됨).

## Topics / Actions / Services (확인된 일부)

| 토픽/리소스 | 정의 | 사용 |
|------------|------|------|
| `voice_command` (String) | `voice_control_node.py:25`, `main.py:30` | `voice_control_node` publish → `main.py:193` subscribe |
| `voice_status`, `voice_ui_status` (String) | `voice_control_node.py` | (verify needed) |
| `MoveChessPiece` (Action) | `cobot2_interfaces/action/MoveChessPiece.action` | `main.py` ActionClient → `robot_action.py` ActionServer (verify needed) |
| `StockfishMove` (Service) | `cobot2_interfaces/srv/StockfishMove.srv` | `main.py` 호출 → `stockfish.py` 응답 (verify needed) |

## Known Risks (carried over)

- **shebang 비영속**: 재빌드 시 system python으로 회귀 (Phase 1+ wrapper 미적용)
- **Tier 0 위반 의심**: vision/Firebase 결측 시 fabrication 코드 존재 가능성 — Phase 1-3에서 점검
- **ROS2 설계 위반 의심**: `ros2-principles.md` Rule 1(메시지 의미), Rule 4(QoS 명시), Rule 9(안전 신호 패턴) 점검 필요 — Phase 1
- **module-level hardware connect**: `robot_action.py:30` `gripper = RG(...)` import 시 Modbus 연결 시도 — Tier 0 callback blocking 의심

## Roadmap

- 단기 (1주, ~2026-05-08): `docs/DEVELOPMENT_ROADMAP.md` Phase 0~4. **Phase 0 완료 (2026-05-01)**.
- 중기 (Backlog): 실제 M0609 데모, pytest, CI(colcon + lint).

## Quick References

- Hard Rules: `.claude/rules/ai-constitution.md` (프로젝트) + `~/.claude/rules/ai-constitution.md` (전역)
- ROS2 Design: `~/.claude/rules/ros2-principles.md`
- Workflow: `~/.claude/rules/development-workflow.md`
- Roadmap: `docs/DEVELOPMENT_ROADMAP.md`
- Phase 0 산출물: `docs/plans/2026-05-01-phase0-bootstrap-{design,bootstrap}.md`
- venv_voice 백업: `docs/baseline/voice_venv_freeze.txt`
- Firebase JSON: `/home/rokey/secrets/kybfirebase.json` (외부)
- Decisions: `docs/decisions/README.md` (ADR-001)

## Update Discipline

- 새 사실 검증 → 즉시 추가, "verify needed" 마커 제거
- 잘못된 추정 발견 → 즉시 정정 (전역 ai-constitution VI.3)
- 항목 5개 초과로 늘면 토픽별 파일 분리 (`memory/<topic>.md`)
