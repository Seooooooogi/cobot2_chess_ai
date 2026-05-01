# Phase 0 Bootstrap — Design (env-only scope)

**Status**: Approved 2026-05-01 (option A — env-only, voice/OpenAI 코드 변경은 Phase 1 이후 분리)
**Roadmap**: `docs/DEVELOPMENT_ROADMAP.md` Phase 0
**Gate**: brainstorming → writing-plans → impl → code-reviewer → verification → done

## Decision Context

User decisions (대화 시점 2026-05-01):
- **Q1 OpenAI 처리**: option C — key 제거 + 기존 OpenAI 코드 비활성화 → **(축소) Phase 0에서는 key 제거만, 코드 비활성화는 Phase 1+로 이연**
- **Q2 Firebase**: 향후 제거 예정, 로컬 DB 대체 → **별도 cycle**
- **Q3 로봇**: `DOOSAN_ROBOT_IP=192.168.137.100`, `ROBOT_MODE=virtual` 고정
- **Q4 Stockfish**: 미설치 → apt install 필요. YOLO 가중치 → `.tflite`만 존재 (wakeword), `best.pt` 부재
- **Q5 venv**: uv 설치 + `--system-site-packages` 로 Humble rclpy와 공존
- **추가 확정** (advisor 권고로 도출): 음성/OpenAI/Firebase 코드 변경은 **Phase 0에서 분리** — voice→main 결합(`main.py:193` voice_command subscribe) 발견 후 재고

## Goal

`docs/DEVELOPMENT_ROADMAP.md` Phase 0 exit criteria 충족:
- `colcon build` 에러 없이 성공
- 4개 entry point(`main`, `stockfish`, `robotaction`, `object`) **import error 없이 기동** (= process spawn + import 통과; ros2 run 실제 실행 결과는 Phase 0 외)

## Out of Scope (Phase 1+ 이연)

코드 변경을 수반하는 다음 항목은 **Phase 0에서 다루지 않음**:
- 음성 노드(`STT.py`, `voice_control_node.py`, `run_voice.sh`) 비활성화/제거
- main.py가 `voice_command` 토픽 대기로 영구 정지하는 문제 — 통합 시퀀스에서만 노출
- Firebase 코드 제거 / 로컬 DB 마이그레이션
- 하드코딩된 `/home/kyb/...` 절대 경로 4건 (main.py:20-21, vision_db.py:18, voice_control_node.py:22) → .env 화
- 모듈 상수 외부화 (stockfish.py:12 `STOCKFISH_PATH`)
- YOLO 가중치 확보 (`best.pt` 부재)
- `docs/DEVELOPMENT_ROADMAP.md` Phase 0-1의 "OpenAI key 채움" 문구 패치 (Input Policy와 모순)

각 항목은 Phase 1 코드 매핑 완료 후 별도 brainstorming cycle로 재진입.

## Approach

### Tier 0 / 안전 체크
- **baseline before refactor**: git init + 첫 커밋으로 비가역 작업 사전 롤백 보장
- **no fabrication**: venv_voice 삭제 전 `pip freeze` 백업 (필요 시 재구성)
- **no hardcoded secrets**: 신규로 secret을 추가하지 않음 (.env 통합 과정에서 OpenAI key 제거)
- **virtual mode first**: ROBOT_MODE=virtual 고정, 본 cycle에서 mode:=real 진입 없음
- **Input Policy**: 신규 OpenAI/STT 코드 추가 없음, 기존 코드는 보존(Phase 1+에서 처리)
- **append-only**: Firebase 미사용 — append-only 위반 가능성 없음

### Sequence (10 steps)

| # | 작업 | 누가 | 비고 |
|---|------|------|------|
| 1 | `git init` + `.gitignore` + 첫 커밋 | 나 | rollback 보장. AI attribution 금지 (Tier 0) |
| 2 | uv 설치 (없으면) + `.venv` 구성 (`--system-site-packages`) | 나 | rclpy(Humble) 공존 |
| 3 | venv_voice `pip freeze` → `docs/baseline/voice_venv_freeze.txt` | 나 | 489MB 삭제 전 보존 |
| 4 | `rm -rf venv_voice` + 중복 `.env` 2개 + `openai.env` 삭제 | 나 | 사용자 명시 승인됨 |
| 5 | `.env` 통합 → `src/cobot2/.env` 1개 | 나 | OpenAI key 제거, Firebase 키 미포함 |
| 6 | `sudo apt install stockfish` | 나 | sudo 비밀번호는 사용자 터미널에서 입력 |
| 7 | `pip install ultralytics stockfish firebase-admin` | 나 | venv 활성 상태에서 |
| 8 | `colcon build --packages-select cobot2 cobot2_interfaces` | 나 | ROS2 sourced 필요 |
| 9 | shebang 검증 (`head -1 install/cobot2/lib/cobot2/main`) | 나 | venv python 가리키는지 |
| 10 | import-only smoke: `python -c "import cobot2.main, ..."` | 나 | Phase 0-4 정의 명확화 |

### `.env` 통합 명세 (Step 5)

`src/cobot2/.env` 단일 파일:
```
# === Robot Connection (Doosan M0609) ===
DOOSAN_ROBOT_IP=192.168.137.100
ROBOT_MODE=virtual

# === Feature Flags (default OFF) ===
VOICE_INPUT_ENABLED=0
LLM_CHESS_LOGIC_ENABLED=0

# === Firebase (deferred — 로컬 DB 마이그레이션 예정) ===
# FIREBASE_SERVICE_ACCOUNT_PATH=
# FIREBASE_DATABASE_URL=

# === OpenAI (Input Policy: 신규 추가 금지) ===
# OPENAI_API_KEY=  ← 의도적 미설정

# === App Config ===
LOG_LEVEL=INFO
YOLO_MODEL_PATH=src/cobot2/models/best.pt   # ← 파일 부재 (Phase 1+ 처리)
STOCKFISH_PATH=/usr/games/stockfish
```

주의:
- 본 .env 파일이 `src/cobot2/.env`에 존재해야 `setup.py`의 `data_files` 빌드 통과 (CLAUDE.md Secrets Policy 명시)
- `.env` 자체는 git 커밋 금지 (`.gitignore`에 등록)

### `.gitignore` 명세 (Step 1)

최소 항목:
```
# venv
.venv/
venv/
**/venv_voice/

# Secrets
.env
*.env
!.env.example
**/firebase-adminsdk-*.json
**/*serviceAccount*.json

# ROS2 build artifacts
build/
install/
log/

# Python
__pycache__/
*.pyc
*.pyo
.pytest_cache/

# Models (large weights — 별도 관리)
*.pt
*.pth
*.tflite
!src/cobot2/models/.gitkeep

# IDE
.vscode/
.idea/
```

### Verification (Step 10)

import-only smoke 정의:
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source .venv/bin/activate
python -c "
import cobot2.main
import cobot2.stockfish
import cobot2.robot_action
import cobot2.vision_db
print('all imports OK')
"
```
출력 `all imports OK` → Phase 0-4 PASS.

ros2 run 실제 실행은 Phase 0 외 (`vision_db`는 YOLO 가중치 부재로 즉시 fail, `main`은 voice_command 무한 대기 — Phase 1 매핑에서 식별).

## Risks

| Risk | 대응 |
|------|------|
| uv `--system-site-packages`가 ament_python venv-installed package를 못 찾음 | colcon build 후 shebang 검증 (Step 9). venv python에서 `import rclpy` 통과 확인 |
| pip install 실패 (네트워크/권한) | venv 활성 상태 재확인, `pip install --no-deps` 분리 진단 |
| .env 통합 시 사용자 secret 손실 | Step 4 삭제 전 3개 파일 키 비교 출력 — 모두 `OPENAI_API_KEY` 동일 (확인됨) |
| git init이 기존 비-git 환경에 영향 | 영향 없음 — 디렉터리에 `.git` 추가만. 사용자 동의 후 진행 |
| colcon build가 venv_voice 삭제 후에도 stale 캐시 참조 | `rm -rf build install log` 후 build (이미 build 산출물 없음 확인됨) |

## Exit Criteria

- [ ] git 초기 커밋 존재 (rollback 가능)
- [ ] `.venv` 활성화 가능 + `import rclpy` 통과
- [ ] `src/cobot2/.env` 1개로 통합, OpenAI key 부재 확인
- [ ] 중복 `.env` / `venv_voice` 삭제 완료
- [ ] `stockfish --version` 동작
- [ ] `colcon build --packages-select cobot2 cobot2_interfaces` 0 error
- [ ] `python -c "import cobot2.main, cobot2.stockfish, cobot2.robot_action, cobot2.vision_db"` 통과
- [ ] code-reviewer 통과 (`SHIP IT` 또는 `MINOR`만)
- [ ] verification 통과

## Approval Required

다음 단계: **writing-plans** — 본 설계를 원자 태스크 리스트로 분해 (파일 경로/명령/검증 단계까지 명시).
