# Phase 1-4 — Configuration File Inventory

> 작성일: 2026-05-01 · 작성자: 인계자 (현 작업자)
> 목적: chess_ai 패키지가 참조하는 JSON 설정 파일의 키별 용도·참조 위치 매핑.
> 범위: `data.json`, `chess_grid.json` (2개 사본).
> **원칙**: 코드 직독으로 확인된 참조만 기록. 키 의미·단위 추정은 `# verify needed`.

## File List

| # | 경로 | 크기 | 형식 | 용도 |
|---|------|------|------|------|
| 1 | `src/chess_ai/chess_ai/data.json` | 12 줄 | JSON object (Korean keys) | DSR 모션 파라미터 + 좌표 calibration |
| 2 | `src/chess_ai/chess_ai/chess_grid.json` | 1 줄 (minified) | JSON object (cell→polygon) | 카메라 frame chess board cell polygon |
| 3 | `src/chess_ai/config/chess_grid.json` | 1 줄 (minified) | 동일 (#2와 byte-identical, `diff -q` empty) | **중복 파일** |

---

## 1. `data.json` — DSR 모션 파라미터 (Korean keys, 11개)

### Load 위치

```python
# robot_action.py:23
JSON_PATH = os.path.join(BASE_DIR, "data.json")     # chess_ai/chess_ai/data.json

# robot_action.py:84-105 (MovingChessPiece.load_initial_config)
if not os.path.exists(JSON_PATH): return
with open(JSON_PATH, "r") as f: data = json.load(f)
self.vel = data.get("속도", self.vel)
... (10개 키 매핑)
```

### 키 매핑 표

| 키 (data.json) | 값 (data.json) | 단위 | code default | code field | 매핑 위치 (robot_action.py) |
|---------------|---------------|------|--------------|-----------|---------------------------|
| `속도` | 60 | DSR vel (`%`?) | 60 | `self.vel` | line 91 |
| `가속도` | 60 | DSR acc (`%`?) | 60 | `self.acc` | line 92 |
| `시간` | 2.5 | sec | 2 | `self.time` | line 93 |
| `mwait_시간` | 0.5 | sec | 2 | `self.mwait_time` | line 94 |
| `wait_시간` | 1.5 | sec | 1 | `self.wait_time` | line 95 |
| `홈_관절좌표` | `[0,0,45,0,135,-90]` | deg (J1~J6) | 동일 | `self.basic_posj` | line 96 |
| `A1_좌표` | `[245.56, 179.81, 27.62, 75.24, -180, -55.21]` | mm,mm,mm,deg,deg,deg (X,Y,Z,A,B,C) | `[244.44, 176.01, 27.62, 75.24, -180, -55.21]` | `self.posx_A1` | line 98 |
| `무덤_관절좌표` | `[0,0,-90,0,-90,0]` | deg | 동일 | `self.posj_tomb` | line 99 |
| `무덤_관절좌표_오버` | `[0,0,0,0,0,0]` | deg | 동일 | `self.posj_tomb_over` | line 100 |
| `z축_간격` | 150 | mm | 150 | `self.z_posx_interval` | line 101 |
| `칸_간격` | 50.2 | mm? | — | **(없음)** | — **`# verify needed` — Dead key** |

### 발견 사항

#### Dead key: `칸_간격`

- `data.json`에만 존재. `robot_action.py`에서 참조하는 위치 0개 (grep 확인).
- 코드의 cell 간격 상수는 별도 hardcoded:
  ```python
  # robot_action.py:48-51 (모듈 상수, JSON 매핑 안 됨)
  self.poscharx_interval = 0.082857143
  self.poschary_interval = 50.648571429
  self.posnumx_interval = 50.858571429
  self.posnumy_interval = 0.3
  ```
- **추정**: 원작자가 `칸_간격`(50.2)을 추가한 후 `posnumx_interval`(50.858571429) / `poschary_interval`(50.648571429)로 분리하면서 매핑 누락. # verify needed
- **Phase 4 제안**: (a) 키 삭제 또는 (b) `poschary_interval`/`posnumx_interval` 매핑 복원.

#### Calibration 흔적

- `A1_좌표` 차이: data.json `[245.56, 179.81, ...]` vs code default `[244.44, 176.01, ...]` → ~1.1mm + ~3.8mm 차이. 실측 보정값으로 추정. # verify needed
- `mwait_시간`: 2 → 0.5 (단축), `wait_시간`: 1 → 1.5 (연장), `시간`: 2 → 2.5 — calibration 단계에서 튜닝.

#### Korean 키 사용

- 11개 키 모두 한글. ROS2 인터페이스 표준은 영문이지만, 본 파일은 노드 외부 호출 안 함(local read-only) → Rule 1·8 위반 아님. **단 cross-team 협업 시 영문화 권장** (Phase 4 검토).

#### Tier 0 검토 — 단위 명시 부재

- `속도`/`가속도` 단위: DSR `movej` API는 vel/acc 입력 단위가 deg/sec, deg/sec² 또는 percentage 모드. **본 코드에서 DSR API 호출 시 어느 모드인지 미확인** — `# verify needed`. Rule 3 (SI 단위) 위반 후보.

---

## 2. `chess_grid.json` — Chess board cell polygons

### 구조

```json
{
  "A1": [[128, 79], [118, 110], [167, 110], [174, 79]],
  "B1": [[118, 110], [109, 143], [160, 144], [167, 110]],
  ...
  "H8": [[509, 377], [519, 441], [592, 442], [576, 377]]
}
```

- **64개 cell** (A1~H8) × 각 cell **4개 vertex** × **(x, y) pixel pair** = 512 정수 좌표.
- 단위: **pixel** in camera frame (카메라 입력 이미지 좌표계).
- 카메라 해상도: `# verify needed` — 좌표 최댓값 ~592x442로 미루어 **640x480 추정** (USB 웹캠 default).

### Load 위치

```python
# vision_db.py:20
GRID_PATH = "/home/kyb/cobot_ws/src/chess_ai/config/chess_grid.json"   # 하드코딩 (Rule 8 위반)

# vision_db.py:155
grid_polygons = load_chess_grid(GRID_PATH)
```

- `load_chess_grid` 함수 내용 확인 미실시 — 폴리곤 in-test로 detection 결과를 cell에 매핑한다고 추정. `# verify needed`

### 사본 중복

- `chess_ai/chess_ai/chess_grid.json` ↔ 'chess_ai/config/chess_grid.json': byte-identical (`diff -q` empty).
- **추정 의도**:
  - `config/`: setup.py가 `share/chess_ai/config/`로 install (`setup.py:15` `glob('config/*.json')`)
  - 'chess_ai/': import 시 module 경로 fallback 또는 dev 편의 사본
- **현재 GRID_PATH는 absolute /home/kyb/...** — 둘 다 안 봄. 절대경로 하드코딩이 우선 (Phase 4 환경변수화 + 사본 단일화).

---

## 3. Packaging (setup.py)

```python
# setup.py:11-17
data_files=[
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.json')),    # config/chess_grid.json
    (os.path.join('share', package_name, 'models'), glob('models/*')),          # models/hello_rokey_*.tflite
    (os.path.join('share', package_name), ['.env', 'chess_ai/data.json']),         # ↑ .env + data.json (직접)
]
```

### 설치 결과 (`install/chess_ai/share/chess_ai/`)

| Source | Install dest |
|--------|--------------|
| `launch/*.launch.py` | `share/chess_ai/launch/` |
| `config/chess_grid.json` | `share/chess_ai/config/chess_grid.json` |
| `models/hello_rokey_8332_32.tflite` | `share/chess_ai/models/` |
| `.env` | `share/chess_ai/.env` (**필수** — 누락 시 빌드 실패, CLAUDE.md Secrets Policy 참조) |
| `chess_ai/data.json` | `share/chess_ai/data.json` |

### Tier 0 (.env) 검토

- `setup.py:17`이 `.env`를 data file로 등록 → `.env` 파일 자체는 **빌드 의존성**. 신규 환경에서 `.env.example` → `src/chess_ai/.env` 복사 필수.
- 환경변수 값은 코드에서 `os.getenv(...)`로 로드(예: `ROBOT_MODE`, `FIREBASE_SERVICE_ACCOUNT_PATH`) — 파일 내용은 secret, **커밋 금지** (`.gitignore` 등록됨). (`OPENAI_API_KEY`, `VOICE_INPUT_ENABLED`, `WAKEWORD_MODEL_PATH` 는 2026-05-04 .env.example에서 제거 완료)

### 미설치 자산

- `chess_ai/chess_ai/train_pt/best.pt` (19MB), `classifier.pt` (43MB) — `setup.py:data_files`에 등록 **안 됨** → `install/`에 복사 안 됨.
  - 영향: `vision_db.py:18-19` 절대경로(`/home/kyb/...`)로 직접 읽음 → install 위치 무관, **소스 트리 직접 의존**.
  - Phase 4 후보: model files도 `data_files` 추가 + 환경변수화.

---

## 4. Dead Launch File (Phase 1-2 후속)

### `launch/cv_chess_recognition.launch.py`

```python
# 19-23
home_dir = os.path.expanduser('~')
default_yolo_path   = os.path.join(home_dir, 'assembly_yolo11/runs/detect/chess_10k_result/weights/best.pt')
default_resnet_path = os.path.join(home_dir, 'classifier.pt')
default_grid_path   = os.path.join(home_dir, 'chess_grid.json')

# 64-68
Node(package='chess_ai', executable='cv_chess_recognition_node', ...)
```

**문제**:
1. `executable='cv_chess_recognition_node'` — `setup.py:entry_points` 에 등록 **안 됨**. `ros2 run chess_ai cv_chess_recognition_node` → "executable not found".
2. Default 경로 3종(`~/assembly_yolo11/...`, `~/classifier.pt`, `~/chess_grid.json`) 모두 **실제 파일 위치와 불일치** (실파일은 `train_pt/`와 `config/`에 있음).

**결론**: **Dead launch** — 등록되지 않은 노드 + 잘못된 경로. Phase 4에서 (a) 삭제 또는 (b) `executable=object` (=vision_db) + 정확한 default 경로로 재작성.

---

## Open Issues (Phase 4 후속)

### 키/파일 정리

1. **`data.json:칸_간격`(50.2) Dead key** — 코드 미참조. 삭제 또는 매핑 복원 결정 필요.
2. **`chess_grid.json` 사본 중복** — 'chess_ai/'와 `config/` 두 곳. 단일화 (config 측 보존 권장 — setup.py가 install 대상으로 등록).
3. **YOLO/ResNet 모델 미패키징** — `train_pt/*.pt`가 `setup.py:data_files`에 없음. install/ 미복사 → src/ 절대경로 의존.

### 하드코딩 (Phase 1-3과 중복)

4. `vision_db.py:20` `GRID_PATH` 절대경로 (`/home/kyb/...`).
5. data.json 키들의 단위 미명시 (`속도`, `가속도` — DSR 단위 모드 검증 필요, Rule 3).

### Naming convention

6. **Korean keys**: 11개 키 모두 한글. cross-team 협업 시 영문화 권장. ROS2 외부 인터페이스가 아니므로 Rule 위반은 아님.

### Dead launch

7. `launch/cv_chess_recognition.launch.py` 삭제 또는 entry_point 등록 + 경로 수정.

---

## Phase 1 Summary (1-1 ~ 1-4 완료)

| Phase | 산출물 | Status |
|-------|--------|--------|
| 1-1 | 노드 다이어그램 (4 entry points) | ✅ `2026-05-01-phase1-1-node-diagrams.md` |
| 1-2 | ROS2 endpoint 인벤토리 (10 노드 실측) | ✅ `2026-05-01-phase1-2-topic-inventory.md` |
| 1-3 | 외부 의존성 매핑 (7종) | ✅ `2026-05-01-phase1-3-external-deps.md` |
| 1-4 | 설정 파일 인벤토리 (data.json 11키 + chess_grid.json) | ✅ 본 문서 |

**Exit criteria** (DEVELOPMENT_ROADMAP.md Phase 1 Exit):
- [x] ① 노드 그래프 — Phase 1-1 다이어그램
- [x] ② 외부 의존성 맵 — Phase 1-3
- [x] ③ 설정 파일 사전 — 본 문서

→ Phase 1 완료. 다음: **Phase 2 — 주석/문서화** (`# verify needed` 마커 수집 → Phase 3 검증 대상).

---

## Cross-References

- Phase 1-1: `docs/code-mapping/2026-05-01-phase1-1-node-diagrams.md`
- Phase 1-2: `docs/code-mapping/2026-05-01-phase1-2-topic-inventory.md`
- Phase 1-3: `docs/code-mapping/2026-05-01-phase1-3-external-deps.md`
- DEVELOPMENT_ROADMAP: `docs/DEVELOPMENT_ROADMAP.md`
- Tier 0 Hard Rules: `CLAUDE.md`, `.claude/rules/ai-constitution.md`
