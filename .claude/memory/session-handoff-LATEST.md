# Session Handoff — LATEST

> 다음 세션이 처음 읽는 파일. **forward-looking only** (이번 세션에서 한 일은 적지 않는다).
> 압축/세션 종료 직전 갱신.

## Last Updated

2026-05-01 — Phase 0 완료, Phase 1 진입 직전.

## Next Actions (priority order)

1. **Phase 0 변경 사항 git commit** (사용자 명시 요청 후 진행).
   포함:
   - `.gitignore` 패치 (firebase/모델 가중치/venv_voice 패턴)
   - `docs/plans/2026-05-01-phase0-bootstrap-design.md`
   - `docs/plans/2026-05-01-phase0-bootstrap.md`
   - `docs/baseline/voice_venv_freeze.txt`
   - `.claude/memory/{MEMORY.md,session-handoff-LATEST.md,context-log.md}` 갱신
   - `tasks/lessons.md` 갱신
   - 삭제: `src/cobot2/cobot2/{venv_voice,.env,openai.env}` (이미 디스크에서 삭제됨, git에서 stage 필요)
   - 삭제: `src/cobot2/config/kybfirebase.json` (이미 외부로 이동됨)
   - 추가 build artifact 발생 시 .gitignore로 제외됨 (확인 필요)

2. **Phase 1 진입** — `docs/DEVELOPMENT_ROADMAP.md` § Phase 1 Code Mapping.
   - **최우선**: `vision_db.py:30` `FIREBASE_SERVICE_ACCOUNT_JSON` 하드코딩(`/home/kyb/...`) → `os.getenv("FIREBASE_SERVICE_ACCOUNT_PATH")` 교체 (code-reviewer가 IMPORTANT로 마킹)
   - 1-1. 노드 다이어그램 4개 (`main`, `stockfish`, `robot_action`, `vision_db`)
   - 1-2. 토픽/액션/서비스 인벤토리 (`ros2 topic/service/action list` 실측 — 실행 환경 source 순서 주의, Context Notes 참조)
   - 1-3. 외부 의존성 매핑 (Firebase, YOLO, Stockfish, 음성, DR_init)
   - 1-4. 설정 파일 인벤토리 (`config/*.json`, `data.json`)

## Open Decisions

- **shebang 영속성**: wrapper 스크립트 / `setup.py` 수정 / cmake flag — Phase 1 진입 시 결정 (현재는 매번 재빌드 후 sed 패치 임시 방편)
- **Firebase → 로컬 DB**: SQLite / JSON file / 기타 — 별도 cycle. 사용자 lean: 미정
- **음성/OpenAI 코드 처리**: 파일 삭제 / `_archive/` 이동 / import guard — Phase 1 매핑 후 결정
- **DR_init 통합 방향**: M0609_RG2_Integration의 bringup과 통합 (방향 확정, 구현 미정)

## Remaining Issues

- `vision_db.py:30` — `FIREBASE_SERVICE_ACCOUNT_JSON = "/home/kyb/cobot_ws/..."` 하드코딩 (code-reviewer IMPORTANT 1건, Phase 1 진입 첫 처리)
- 하드코딩 `/home/kyb/...` 추가 3건: `main.py:20-21`, `vision_db.py:18`, `voice_control_node.py:22`
- 모듈 상수 외부화: `stockfish.py:12` `STOCKFISH_PATH`
- YOLO 가중치 `best.pt` 부재 — 현재 `models/`에 `hello_rokey_8332_32.tflite` (wakeword) 1개만
- `main.py:193` `voice_command` subscribe → `voice_control_node` 미구동 시 무한 대기
- `robot_action.py:30` module-level `gripper = RG(...)` (Modbus 인스턴스 import 시 생성 — Tier 0 callback blocking 의심)
- `UI.html:114` Firebase Web SDK apiKey 하드코딩 (Web SDK는 공개 식별자 카테고리지만 Firebase migration 시 정리)
- `.gitignore`에 `!src/cobot2/models/.gitkeep` 누락 (설계 명세 대비)
- shebang 영속 해결 미적용 (재빌드 시 system python으로 회귀)

## Context Notes (needed next session)

### 실행 환경 source 순서 (반드시 이 순서, 누락 시 import 실패)

```bash
source /opt/ros/humble/setup.bash
source /home/rokey/M0609_RG2_Integration/install/setup.bash
source /home/rokey/cobot2_chess_ai/install/setup.bash
source /home/rokey/cobot2_chess_ai/.venv/bin/activate
```

- ROS2 → M0609 → cobot2 → venv 순서. 마지막 활성이 venv (PYTHONPATH 우선순위 보장).
- M0609 source 누락 시 `DR_init` 모듈 부재로 `cobot2.robot_action` import 실패.
- venv 활성 누락 시 `ultralytics`, `firebase_admin`, `stockfish`, `pymodbus` 모듈 부재.

### 빌드 절차

```bash
# 위 source 4줄 + 아래
cd /home/rokey/cobot2_chess_ai
rm -rf build install log  # 클린 빌드 권장
colcon build --packages-select cobot2_interfaces cobot2 --symlink-install
# 빌드 후 shebang 패치 (재빌드마다 필요 — 영속 해결 미적용):
for ep in main stockfish robotaction object; do
  sed -i "1s|^#!.*|#!/home/rokey/cobot2_chess_ai/.venv/bin/python|" \
    install/cobot2/lib/cobot2/$ep
done
```

### 검증된 환경 사실 (Phase 0 산출물)

- DR_init 출처: `/home/rokey/M0609_RG2_Integration/install/dsr_common2/lib/python3.10/site-packages/DR_init.py`
- pymodbus 버전 고정: `<3` (현재 2.5.3) — 3.x로 올리면 `from pymodbus.client.sync import ModbusTcpClient` 깨짐 (`onrobot.py:3`)
- Stockfish 14.1 설치 위치: `/usr/games/stockfish`
- venv 위치: `/home/rokey/cobot2_chess_ai/.venv` (uv venv + `--system-site-packages`)
- Firebase JSON 외부 보관: `/home/rokey/secrets/kybfirebase.json` (chmod 600, 디렉터리 700)
- Phase 0-4 정의: import-only smoke (`python -c "import cobot2.main, cobot2.stockfish, cobot2.robot_action, cobot2.vision_db"`)
- `firebase_admin.initialize_app()` 호출 위치: 메서드 안 (`main.py:74`, `vision_db.py:52`) — import만 통과 가능

### Failed approaches (반복 금지)

- `pip install <pkg>` (venv 활성 중에도 `which pip` = `/usr/bin/pip` 가능 — uv venv는 pip 미포함). 항상 `uv pip install` 사용.
- `--system-site-packages` 만으로 rclpy 접근 시도 — 불가. ROS2 source 필요.
- venv_voice 의 pip 직접 실행 — shebang이 `/home/kyb/...` 가리켜 작동 불가 (다른 사용자 시스템 잔재).

### 보안 사고 (이번 cycle 발생, 차후 회피)

- `.env` 파일을 Read 도구로 그대로 열어 OpenAI key가 transcript에 노출됨 → 사용자가 키 회전 권장 받음
- 사용자가 sudo 비밀번호를 chat에 직접 입력 → transcript 영구 보관 → 비번 회전 권장
- 향후: `.env`는 sed 마스킹 후 확인. sudo는 `! sudo ...` 또는 NOPASSWD 사전 설정.

## Current Focus

Top priority: **Phase 1 Code Mapping 진입**. 첫 작업 = `vision_db.py:30` Firebase 경로 하드코딩 → env 화 (IMPORTANT 1건 해소).

## Resume Instruction

다음 세션 시작 시:
1. 본 핸드오프 + `.claude/memory/MEMORY.md` + `.claude/memory/context-log.md` 읽기
2. `tasks/lessons.md` 검토 (AI 행동 교정 규칙 — 보안 사고 lesson 2건)
3. **Hard Rules 재확인**: `.claude/rules/ai-constitution.md` + `~/.claude/rules/ros2-principles.md`
4. 실행 환경 source 4줄 (위 Context Notes 참조)
5. Phase 1 진입 또는 사용자 지시 대기
