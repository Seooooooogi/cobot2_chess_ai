# Phase 0 Bootstrap — Atomic Task Plan

**Source design**: `docs/plans/2026-05-01-phase0-bootstrap-design.md`
**Approved**: 2026-05-01
**Scope**: env-only (코드 변경 없음). 모든 단계 내가 실행 (T7 sudo는 사용자 터미널 비밀번호 입력만).

---

## Task 1 — Git 초기화 + .gitignore + 첫 커밋

**File**: `/home/rokey/cobot2_chess_ai/.gitignore` (신규)
**Change**:
1. `git init` (project root)
2. `.gitignore` 작성 — 설계 문서 § `.gitignore` 명세 그대로
3. 현재 상태 모두 stage 후 첫 커밋:
   - subject (영어): `chore: initial import — pre-Phase 0 baseline`
   - body 한국어 허용
   - **AI attribution 금지**: Co-Authored-By, Generated with X 절대 추가 금지 (Tier 0)
4. `.env`, `*.json` (firebase) 등 secret 파일은 `.gitignore` 적용 후 stage 됐는지 확인

**Verify**:
- `git log --oneline` → 1 commit 존재
- `git status` clean
- `git ls-files | grep -E '\.env$|venv_voice|firebase-adminsdk' | wc -l` → 0
**Depends on**: none

---

## Task 2 — uv 설치 확인 / 설치

**File**: 시스템 (사용자 home)
**Change**:
- `which uv` 결과 없음 → `curl -LsSf https://astral.sh/uv/install.sh | sh` 또는 `pip install uv` (사용자 환경에 따라)
- 설치 후 `uv --version` 통과 확인
- 이미 설치되어 있으면 skip

**Verify**: `uv --version` 출력 존재
**Depends on**: none (병렬 가능)

---

## Task 3 — .venv 생성 (--system-site-packages)

**File**: `/home/rokey/cobot2_chess_ai/.venv/` (신규 디렉터리)
**Change**:
- `uv venv .venv --system-site-packages --python python3.10` (Humble = Python 3.10)
- 활성화: `source .venv/bin/activate`
- `python -c "import rclpy; print(rclpy.__file__)"` → `/opt/ros/humble/...` 경로 확인 (system-site-packages 동작 검증)

**Verify**: `.venv/bin/python` 존재 + rclpy import 통과
**Depends on**: Task 2

---

## Task 4 — venv_voice pip freeze 백업

**File**: `/home/rokey/cobot2_chess_ai/docs/baseline/voice_venv_freeze.txt` (신규)
**Change**:
- `src/cobot2/cobot2/venv_voice/bin/pip freeze` 출력을 백업 파일로 저장
- venv_voice가 손상되어 pip freeze 실패 시: `find venv_voice/lib/python3.10/site-packages -maxdepth 1 -type d` 디렉터리 목록만이라도 저장

**Verify**: `wc -l docs/baseline/voice_venv_freeze.txt` ≥ 1
**Depends on**: none (Task 5 전에만 완료)

---

## Task 5 — venv_voice + 중복 .env 삭제

**Files (삭제 대상)**:
- `/home/rokey/cobot2_chess_ai/src/cobot2/cobot2/venv_voice/` (디렉터리, ~489MB)
- `/home/rokey/cobot2_chess_ai/src/cobot2/cobot2/.env`
- `/home/rokey/cobot2_chess_ai/src/cobot2/cobot2/openai.env`

**Change**:
1. `rm -rf src/cobot2/cobot2/venv_voice/`
2. `rm src/cobot2/cobot2/.env src/cobot2/cobot2/openai.env`

**Verify**:
- `[ ! -d src/cobot2/cobot2/venv_voice ]` 통과
- `find src -name '.env' -o -name 'openai.env'` → `src/cobot2/.env` 1개만
**Depends on**: Task 4 (백업 후 삭제)

---

## Task 6 — src/cobot2/.env 통합 작성

**File**: `/home/rokey/cobot2_chess_ai/src/cobot2/.env` (덮어쓰기)
**Change**: 설계 문서 § `.env 통합 명세` 그대로 작성 — OpenAI key 제거, Firebase 키 미설정, 로봇 IP 기본값.

**Verify**:
- `wc -l src/cobot2/.env` ≥ 10
- `grep -c "^OPENAI_API_KEY=" src/cobot2/.env` → 0 (활성 키 없음, 주석 형태로만)
- `grep "^DOOSAN_ROBOT_IP=" src/cobot2/.env` → 값 존재
- `grep "^ROBOT_MODE=virtual" src/cobot2/.env` → 매칭

**Depends on**: Task 5

---

## Task 7 — Stockfish 설치

**Change**: `sudo apt update && sudo apt install -y stockfish` (사용자 터미널에서 sudo 비밀번호 입력)

**Verify**:
- `which stockfish` → `/usr/games/stockfish` 또는 동등 경로
- `stockfish --version` 통과 (또는 `echo 'quit' | stockfish` 응답)

**Depends on**: none (병렬 가능)

---

## Task 8 — pip 의존성 설치 (venv 안에)

**Change**:
- `.venv` 활성 상태 확인 (`which python` → `.venv/bin/python`)
- `pip install ultralytics stockfish firebase-admin`

**Verify**:
- `python -c "import ultralytics; import stockfish; import firebase_admin; print('OK')"` → `OK`

**Depends on**: Task 3

---

## Task 9 — colcon build

**Change**:
- ROS2 source: `source /opt/ros/humble/setup.bash`
- venv 활성: `source .venv/bin/activate` (이 순서 — venv가 ROS2 위에 올라가야 함)
- 클린 빌드: `rm -rf build install log` (있으면)
- `colcon build --packages-select cobot2_interfaces cobot2 --symlink-install`
  - interfaces 먼저 빌드되도록 명시적 순서

**Verify**:
- `colcon build` 종료 코드 0
- `install/setup.bash` 존재
- `install/cobot2/share/cobot2/.env` 존재 (data_files 등록 확인)

**Depends on**: Task 6, Task 8

---

## Task 10 — Shebang 검증

**Change**: 빌드 산출물 entry point 4개의 shebang 확인 (advisor 권고 5)

**Verify**:
```bash
for ep in main stockfish robotaction object; do
  head -1 install/cobot2/lib/cobot2/$ep
done
```
모두 `.venv/bin/python` 또는 venv 경로 가리키는지 확인. system python (`/usr/bin/python3`) 가리키면 → pip 설치 패키지 로드 실패 가능 → 사용자에게 surface.

**Depends on**: Task 9

---

## Task 11 — Import-only Smoke Test (Phase 0-4)

**Change**:
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

**Verify**: `all imports OK` 출력 + 종료 코드 0

**Depends on**: Task 9, Task 10

---

## Sequence Diagram

```
T1 (git init) ──┐
                ├─→ (모든 작업의 rollback 점)
T2 (uv) ────────┘
       │
       ▼
T3 (.venv) ───────┬───→ T8 (pip deps) ───┐
                  │                       │
T4 (pip freeze) ──┼───→ T5 (rm) ──→ T6 (.env) ─┤
                                                ▼
T7 (apt stockfish) ────────────────────→ T9 (colcon build)
                                                │
                                                ▼
                                          T10 (shebang)
                                                │
                                                ▼
                                          T11 (smoke)
```

병렬 가능: T1, T2, T7 — 의존성 없음
직렬: T3 → T8 / T4 → T5 → T6 / T9 → T10 → T11

## Rollback Plan

| 단계 실패 | 롤백 |
|----------|------|
| T2-T6 실패 | `git reset --hard HEAD` (T1 baseline 복귀) — venv_voice 삭제 후라면 백업 파일 + 재구성 가이드는 `docs/baseline/voice_venv_freeze.txt` 참조 |
| T7 실패 | apt 상태 영향 없음 (uninstall: `sudo apt remove stockfish`) |
| T8 실패 | `rm -rf .venv && uv venv .venv ...` 재시도 |
| T9 실패 | build-error-resolver 에이전트 호출 (orchestrator 자동 트리거) |
| T11 실패 | code-reviewer + verification 게이트로 진단 |

## Exit Criteria (Phase 0-4 정의)

T1-T11 전부 PASS + code-reviewer 통과 + verification 통과 → Phase 0 종료, Phase 1 진입 가능.
