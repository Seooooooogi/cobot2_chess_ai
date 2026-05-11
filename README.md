# cobot2_chess_ai

ROS2 + 두산 M0609 협동로봇 기반 AI 체스 시스템. 비전(YOLO + ResNet18) → Stockfish 엔진 → 로봇 액션 → rosbridge WebSocket + SQLite 감사 로그 (Phase 5 완료, Firebase 의존 0).

본 리포는 협동2 A-2 팀(박윤헌·김연빈·김하균·문규혁·서재우) 프로젝트의 **인계물**이며, 현 작업자는 원작자가 아니다. 따라서 작업 흐름은 **코드 매핑 → 주석 → 동작 검증 → 리팩토링** 순서를 따른다 (`CLAUDE.md` Dev Conventions).

## Status

- **Phase 0** (부트스트랩) 완료 — `.env.example`, secrets policy, vendored 패키지 freeze.
- **Phase 1** (코드 매핑) 완료 — `docs/code-mapping/2026-05-01-phase1-{1,2,3,4}-*.md`.
- **Phase 2** (주석 / 문서화) 진행 중 — module/class docstring 추가, `# verify needed` 인덱스 (`outputs/verify-needed.md`).
- **Phase 3** (동작 검증) 미진입.
- **Phase 4** (리팩토링) 미진입 — Open Decisions는 `.claude/memory/session-handoff-LATEST.md`.

---

## 1. Phase 0 부트스트랩

### 1.1 의존성

- ROS2 Humble (`/opt/ros/humble`)
- Python venv (`.venv/`, `uv` 사용 가정 — `pip` 대신 `uv pip install`)
- Vendored ROS2 패키지 (커밋 freeze):
  - `src/doosan-robot2` — `ec92425`
  - `src/onrobot-ros2` — `c6e3903`
  - `src/m0609_rg2_bringup` — `7d6aa3c`
- 외부 의존성 (자세히는 `docs/code-mapping/2026-05-01-phase1-3-external-deps.md`):
  - `stockfish`, `ultralytics`, `torch`, `torchvision`, `opencv-python`, `pymodbus==2.5.3` (PyPI)
  - 시스템: `stockfish` 바이너리 (`/usr/games/stockfish`)

### 1.2 시크릿 / `.env`

`setup.py`가 `share/chess_ai/.env`를 데이터 파일로 등록 → **`.env`가 없으면 빌드 실패**한다.

```bash
cd /home/rokey/cobot2_chess_ai
cp .env.example src/chess_ai/.env
# 그런 다음 src/chess_ai/.env 안의 값들을 채운다 (ROBOT_MODE, 모델 경로 등)
```

`.env` 는 절대 커밋하지 않는다 (CLAUDE.md Hard Rule 5).

### 1.3 빌드

```bash
cd /home/rokey/cobot2_chess_ai

# (a) 신규 환경 — vendored 패키지까지 포함한 클린 빌드
rm -rf build install log
colcon build \
  --packages-up-to dsr_bringup2 dsr_controller2 onrobot_rg_control m0609_rg2_bringup chess_ai \
  --symlink-install

# (b) chess_ai 코드만 수정한 경우
colcon build --packages-select chess_ai --symlink-install
```

### 1.4 Shebang 패치 (재빌드마다 필요)

`ament_python` 빌드는 venv 인식이 없어 system python shebang을 박는다. 빌드 직후 패치:

```bash
for ep in main stockfish robotaction object gamelogger; do
  sed -i "1s|^#!.*|#!/home/rokey/cobot2_chess_ai/.venv/bin/python|" \
    install/chess_ai/lib/chess_ai/$ep
done
```

→ 영속 해결책 (wrapper / setup.py / cmake flag) 미정 (`.claude/memory/session-handoff-LATEST.md` Open Decisions).

---

## 2. 실행 시퀀스

### 2.1 source 3줄 (모든 셸에서)

```bash
source /opt/ros/humble/setup.bash
source /home/rokey/cobot2_chess_ai/install/setup.bash      # vendored DSR/onrobot 포함
source /home/rokey/cobot2_chess_ai/.venv/bin/activate      # chess_ai 패키지 실행 시
```

### 2.2 Bringup (별도 셸)

가상 모드:
```bash
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=virtual
```

실기 모드 (CLAUDE.md Tier 0: virtual 검증 후에만):
```bash
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100
```

### 2.3 chess_ai 노드

각 노드 별도 셸:
```bash
ros2 run chess_ai stockfish     # Stockfish 서비스 (chess_ai/stockfish.py)
ros2 run chess_ai robotaction   # Doosan M0609 + RG2 액션 서버 (chess_ai/robot_action.py)
ros2 run chess_ai object        # 비전 인식 + /vision/board_state publish (chess_ai/vision_db.py)
ros2 run chess_ai gamelogger    # SQLite 감사 로그 (chess_ai/game_logger.py) — 선택
ros2 run chess_ai main          # 워크플로 오케스트레이터 (chess_ai/main.py)
```

### 게임 시작 트리거

main 노드 startup 로그에서 "Service: /main_controller/start_sampling" 확인 후 호출:

```bash
ros2 service call /main_controller/start_sampling std_srvs/srv/Trigger {}
```

`ROBOT_MODE` 환경변수는 `robot_action.py`에서 직접 읽으며 (default `"virtual"`), 반드시 bringup의 `mode:=` 인자와 일치시킨다 (Rule 9 안전 mismatch 경고가 노드 시작 시 출력됨).

---

## 3. 환경변수 표

`.env.example`이 템플릿. 자세한 코드 참조 위치는 `docs/code-mapping/2026-05-01-phase1-3-external-deps.md`.

| 키 | 용도 | 코드 참조 | 필수 여부 |
|-----|------|-----------|-----------|
| `YOLO_MODEL_PATH` | YOLO 가중치 경로 | `vision_db.py` | vision 실행 시 필수 |
| `RESNET_MODEL_PATH` | ResNet18 분류기 가중치 경로 | `vision_db.py` | vision 실행 시 필수 |
| `CHESS_GRID_PATH` | 체스 보드 grid JSON 절대경로 | `vision_db.py` | vision 실행 시 필수 |
| `CAMERA_SOURCE` | 카메라 인덱스 | `vision_db.py` (default `3`) | 선택 |
| `DOOSAN_ROBOT_IP` | M0609 IP | bringup launch 인자 | 실기 모드 시 |
| `ROBOT_MODE` | `virtual` 또는 `real` | `robot_action.py` | 권장 (default `virtual`) |
| `LOG_LEVEL` | 로깅 수준 | (현재 미연결) | 선택 |
| `STOCKFISH_PATH` | Stockfish 바이너리 경로 | `stockfish.py` | 선택 (default `/usr/games/stockfish`) |
| `CHESS_AI_STATE_PATH` | Stockfish 국면 상태 파일 경로 | `stockfish.py` | 선택 (default `~/.local/share/cobot2_chess_ai/chess_state.json`) |
| `CHESS_AI_LOG_DB_PATH` | game_logger SQLite DB 경로 | `game_logger.py` | 선택 (default `~/.local/share/cobot2_chess_ai/game_log.db`) |


---

## 4. 인계자 작업 흐름 (`CLAUDE.md` Dev Conventions)

1. **코드 매핑** — `docs/code-mapping/2026-05-01-phase1-{1,2,3,4}-*.md` (4개 doc, 1054줄). 노드/토픽/액션/외부 의존성/설정 사전.
2. **주석·docstring** — 검증된 동작만 기술. 미검증은 `# verify needed` 또는 docstring "확인 필요". Tier 0 II.1.
3. **virtual 모드 동작 검증** — 실기 (`mode:=real`) 진입 전 동일 시퀀스가 virtual에서 에러 없이 완료되어야 한다. CLAUDE.md Hard Rule 3.
4. **식별된 개선점 리팩토링** — 동작 변경은 마지막. opt-in 환경변수, 기본 OFF.

---

## 5. 알려진 제약

### Tier 0 (절대 위반 금지)

- 미검증 동작 단정 금지 (`CLAUDE.md` Hard Rule 1)
- 리팩토링 전 baseline 기록 (Hard Rule 2)
- virtual mode first (Hard Rule 3)
- vision 결측 명시적 표기 — 보간 금지 (Hard Rule 4)
- 시크릿 하드코딩 금지 (Hard Rule 5)
- game_logger SQLite append-only (Hard Rule 6 — TRIGGER로 스키마 레벨 보장)
- git attribution에 AI 이름 추가 금지 (Hard Rule 7)
- ROS2 Rule 9 (안전 신호 Topic 금지, 페일세이프 수렴) — `~/.claude/rules/ros2-principles.md`

### Phase 4 deferred (Open Decisions / Remaining Issues)

- `data.json:칸_간격` (50.2) **dead key** — 삭제 vs 매핑 복원 결정.
- `chess_grid.json` byte-identical 사본 2개 (`chess_ai/`, `config/`) — 단일화.
- `launch/cv_chess_recognition.launch.py` **dead launch** — `executable=cv_chess_recognition_node`가 setup.py entry_points 미등록.
- 모델 파일 (`best.pt` 19MB, `classifier.pt` 43MB, `hello_rokey_8332_32.tflite` 203KB) `setup.py:data_files` 미패키징.
- ~~음성/OpenAI 코드 처리~~ — 2026-05-04 옵션 A(파일 삭제) 완료. voice stack 제거, ~/start_sampling Service로 전환.
- ROS2 Rule 위반:
  - QoS depth-only shorthand (Rule 4)
  - ~~`voice_command` Topic 명령~~ — 2026-05-04 ~/start_sampling Service로 RESOLVED (Rule 2)
  - `goal_callback` 무조건 ACCEPT (Rule 7)
  - `feedback_msg` 미사용 (Rule 2 — Action 재설계 후보)
- shebang 영속 해결 — wrapper / setup.py / cmake flag 미결정.
- `package.xml` rosdep key 오류 — `chess_ai`의 `ament_python`, `onrobot_rg_control`의 `message_runtime` (vendored, Rule 6에 따라 수정 금지).
- vendored 패키지의 `ros2 topic hz /dsr01/joint_states` hang on TRANSIENT_LOCAL pub.

### 보안 (carry over)

- `.env` 노출 사고 발생 → 키 회전 권장 (`.claude/memory/session-handoff-LATEST.md` 참조).

---

## 6. 추가 문서

- `CLAUDE.md` — Hard Rules, Quick Ref, Compact Instructions.
- `.claude/memory/MEMORY.md` — 의미적 메모리 (단계, entry points, configs, 외부 의존성 요약).
- `.claude/memory/session-handoff-LATEST.md` — 다음 세션 진입 정보 (Open Decisions, Remaining Issues).
- `docs/DEVELOPMENT_ROADMAP.md` — Phase 0~6 계획 (Phase 5 완료, Phase 6 실기 검증 진입 가능).
- `docs/decisions/README.md` — ADR 인덱스.
- `outputs/verify-needed.md` — `# verify needed` 마커 통합 인덱스 (Phase 3 인풋).
