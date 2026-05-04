# Phase 1-3 — External Dependency Map

> 작성일: 2026-05-01 · 작성자: 인계자 (현 작업자)
> 목적: cobot2 패키지가 ROS2 외부에서 호출하는 의존성을 위치별로 매핑.
> 범위: Firebase, OpenAI, YOLO/ResNet, Stockfish, Modbus(RG2), 카메라, 음성/오디오 — 7종.
> **원칙**: 코드 직독으로 확인된 호출 위치만 기록. 동작 추정은 `# verify needed`.

## Summary

| # | 의존성 | 호출 위치 (file:line) | 범주 | Input Policy |
|---|--------|---------------------|------|--------------|
| 1 | Firebase Admin SDK | `vision_db.py:13`, `main.py:12` | 데이터 백엔드 | 허용 |
| 1' | Firebase Web SDK | `UI.html:110-123` | UI 데이터 | 허용 (Web UI) |
| 2 | ~~OpenAI API (Whisper STT)~~ | ~~`voice_control_node.py:10`, `STT.py:1`~~ | 음성 STT | **REMOVED 2026-05-04** — 파일 삭제(옵션 A) |
| 3a | YOLO (ultralytics) | `vision_db.py:5,153` | 비전 추론 | 허용 |
| 3b | ResNet18 | `vision_db.py` (경로만 — `train_pt/classifier.pt`) | 비전 분류 | 허용 |
| 4 | Stockfish (Python lib + bin) | `stockfish.py:6,26`, bin: `/usr/games/stockfish` | 체스 엔진 | 허용 |
| 5 | OnRobot RG2 (pymodbus) | `onrobot.py:3-11`, `robot_action.py:8,55-73` | 그리퍼 IO | 허용, virtual에서 skip |
| 6 | OpenCV (V4L2 카메라) | `vision_db.py:163,207` | 영상 입력 | 허용 |
| 7a | ~~openWakeWord (TFlite)~~ | ~~`voice_control_node.py:11,67`~~ | wakeword 감지 | **REMOVED 2026-05-04** |
| 7b | ~~PyAudio~~ | ~~`miccheck.py:1`, `miccontroller.py:1`, `voice_control_node.py`~~ | 마이크 스트림 | **REMOVED 2026-05-04** |
| 7c | ~~sounddevice~~ | ~~`STT.py:2`~~ | 마이크 스트림 (대안) | **REMOVED 2026-05-04** |
| 8 | DR_init / DSR_ROBOT2 | `robot_action.py:6,155,304-306` | 두산 DRCF Python 래퍼 | 허용 (vendored 통합) |

> **Input Policy 근거**: `.claude/rules/ai-constitution.md` III — 음성 인식(STT)·OpenAI API 신규 추가 금지. **기존 코드는 Phase 4에서 비활성화 / 제거 검토.**

---

## 1. Firebase Realtime Database

### 1.1 `firebase_admin` (Server-side, Python)

**호출 노드**: `vision_db.py`(스크립트), `main.py`(`MainController`)

**Service Account JSON 경로** (Tier 0 위반 — 하드코딩):

```python
# main.py:20
FIREBASE_SERVICE_ACCOUNT_JSON = "/home/kyb/cobot_ws/src/cobot2/config/kybfirebase.json"
# vision_db.py: 동일 패턴, 환경변수 처리됨 (이전 세션 commit edd15bc 참조)
```

**DB URL** (양쪽 동일):

```
https://chess-43355-default-rtdb.asia-southeast1.firebasedatabase.app
```

**Reference paths** (호출 위치):

| 경로 | Operation | 호출 위치 |
|------|-----------|-----------|
| `chess/board_state` | `set` (write) | `vision_db.py:145,151` (YOLO 추론 → board snapshot) |
| `chess/board_state` | `get` (read) | `main.py:79` (board_dict 폴링) |
| `chess/board_state` | `set` (write) | `main.py:97` (board state + extra metadata) |
| `chess/ui_control` | `update` | `main.py:101` (UI 명령 반영) |
| `chess/ui_control` | `get` | `main.py:105` (UI 상태 폴링) |
| `chess/chess_system` | `get` | `main.py:110` (depth/difficulty/turn 파라미터) |

**Throttling**:
- `vision_db.py:36`: `FIREBASE_UPDATE_MIN_INTERVAL_SEC = 0.20` (5 Hz cap on board write)

**Tier 0 (CLAUDE.md #6) — append-only**:
- `set`/`update` 호출 위치는 모두 board 전체 덮어쓰기. 게임 이벤트 로그(append-only) 기능은 코드상 부재 → **Phase 4 추가 검토**.

### 1.2 Firebase Web SDK (Browser-side)

**위치**: `UI.html:110-123`

```js
import { initializeApp }    from "https://www.gstatic.com/firebasejs/10.7.1/firebase-app.js";
import { getDatabase, ref, onValue, update }
                            from "https://www.gstatic.com/firebasejs/10.7.1/firebase-database.js";

const firebaseConfig = {
  apiKey: "...",            # ← 하드코딩 (UI.html:114)
  authDomain: "chess-43355.firebaseapp.com",
  databaseURL: "https://chess-43355-default-rtdb.asia-southeast1.firebasedatabase.app",
  storageBucket: "chess-43355.firebasestorage.app",
  ...
};
```

**Tier 0 (no hardcoded secrets) 검토**:
- Firebase Web SDK `apiKey`는 RTDB security rules + Auth로 보호하는 게 정석이라 단독으론 secret이 아니지만, 본 시스템은 RTDB rules 검증 미실시 → **Phase 4 (rules audit + apiKey rotation 또는 환경변수화) 검토 가치**.

---

## 2. OpenAI API (Whisper STT) — **REMOVED 2026-05-04 (옵션 A: 파일 삭제)**

### 위치

| 파일 | 줄 | 용도 |
|------|----|------|
| `voice_control_node.py:10` | `from openai import OpenAI` | Whisper 호출 클라이언트 |
| `voice_control_node.py:38` | `WHISPER_MODEL = "whisper-1"` | 모델명 |
| `voice_control_node.py:61-65` | `os.getenv("OPENAI_API_KEY")` 후 `OpenAI(api_key=...)` | 클라이언트 생성. 키 없으면 `RuntimeError`. |
| `STT.py:1,17-19` | 동일 패턴 (별도 standalone STT 클래스) | `voice_control_node.py`와 중복 — Phase 4 통폐합 검토 |

### `.env` 흐름 (`run_voice.sh`)

```
run_voice.sh:
  1) WS_DIR 탐색 (src/cobot2/setup.py 또는 install/setup.bash 존재)
  2) venv 활성화: $WS_DIR/src/cobot2/cobot2/venv_voice/bin/activate
  3) .env 우선순위: $WS_DIR/src/cobot2/.env → $WS_DIR/.env
  4) OPENAI_API_KEY 미설정 시 ERROR exit
  5) ROS env source + python -m cobot2.voice_control_node
```

**Input Policy 정합성**:
- `.claude/rules/ai-constitution.md` III에 따라 **음성/OpenAI 신규 추가 금지**.
- 기존 코드 처리 방향 (handoff Open Decisions): 파일 삭제 / `_archive/` / import guard 중 미결정 → **Phase 4 결정**.
- 본 launch(`bringup.launch.py`) 및 cobot2 entry point 4종에 `voice_control_node`는 포함되지 않음 — `run_voice.sh`로 별도 기동해야만 활성화. 즉 **현 시스템 시작점에선 OpenAI 호출 비활성**.

---

## 3. YOLO + ResNet18 (비전)

### 호출 위치 (`vision_db.py`)

```python
# vision_db.py:5
from ultralytics import YOLO

# vision_db.py:18-20  (이전 세션 commit edd15bc로 일부 환경변수화)
YOLO_PATH   = ".../train_pt/best.pt"
RESNET_PATH = ".../train_pt/classifier.pt"
GRID_PATH   = ".../config/chess_grid.json"

# vision_db.py:153
yolo_model = YOLO(YOLO_PATH)
# ResNet load: # verify needed (위치 미확인 — 추가 grep 필요)
```

### 모델 파일 — **존재 확인** (handoff stale 정정)

```
src/cobot2/cobot2/train_pt/best.pt        19 MB  (YOLO)
src/cobot2/cobot2/train_pt/classifier.pt  43 MB  (ResNet)
```

> Handoff `## Remaining Issues` 의 "### 모델 가중치 부재" 항목은 **stale**. 두 파일 모두 file system에 존재. **Phase 4에서 handoff 갱신.**

---

## 4. Stockfish — 허용 (Input Policy)

### Python lib + 바이너리

```python
# stockfish.py:6
from stockfish import Stockfish

# stockfish.py:12
STOCKFISH_PATH = "/usr/games/stockfish"

# stockfish.py:26
self.stockfish = Stockfish(path=STOCKFISH_PATH)

# stockfish.py:33  (ROS2 service)
self.srv = self.create_service(StockfishMove, "StockfishMove", self.get_best_move_callback)
```

### 사용 패턴 (`stockfish.py:152-171`)

```
ROS2 service 호출:
  pieces_data + turn + last_move + skill_level + depth
    ↓
  set_skill_level → set_depth → is_fen_valid → set_fen_position → get_best_move
    ↓
  best_move(uci string) 응답
```

**시스템 의존성**: apt `stockfish` 패키지 (`/usr/games/stockfish`). venv 내 Python `stockfish 5.2.0` (handoff carry-over). 둘 다 검증됨.

---

## 5. OnRobot RG2 (Modbus TCP)

### `onrobot.py` (8행)

```python
# onrobot.py:3
from pymodbus.client.sync import ModbusTcpClient as ModbusClient

# onrobot.py:8-11
class RG:
    def __init__(self, gripper, ip, port):
        self.client = ModbusClient(host=ip, port=port, ...)
```

### `robot_action.py` 사용

```python
# robot_action.py:8
from .onrobot import RG

# robot_action.py:26-27 (모듈 상수 — 하드코딩)
TOOLCHARGER_IP   = "192.168.1.1"
TOOLCHARGER_PORT = "502"

# robot_action.py:58-73 (defer pattern, 이전 commit ccff5d0)
if ROBOT_MODE == "virtual":
    self.log("[VIRTUAL] Skipping RG2 Modbus connect")
else:
    # 실제 Modbus connect 시도
```

**virtual 모드 동작**:
- 실제 RG2 Modbus 호출 skip → DR_init 통합된 `gripper_virtual_node`(first-party) 의 `/onrobot/sendCommand` ROS2 service로 대체 (Phase 1-2 인벤토리 참조).

**하드코딩 secrets 검토**:
- `192.168.1.1:502`은 IP — secret이 아니지만 **환경 의존 값**(Rule 8 위반 후보). Phase 4에서 ROS2 파라미터화 검토.

---

## 6. 카메라 (OpenCV / V4L2)

### `vision_db.py`

```python
# vision_db.py:21 (이전 세션 환경변수화 후보)
SOURCE = ...   # V4L2 카메라 인덱스 또는 device path

# vision_db.py:163
cap = cv2.VideoCapture(SOURCE)

# vision_db.py:207
cv2.imshow("Chess Vision Tracker", display_frame)
```

**의존성**:
- 시스템: V4L2 device (`/dev/video*`), GUI display (X server) — `cv2.imshow`는 headless 환경에서 실패.
- venv: `opencv-python 4.13.0.92` (handoff carry-over).

**bringup_camera.launch.py** (RealSense 변형):
- 별도 launch 변형. 본 Phase에선 미기동. RealSense는 ROS2 토픽(`/camera/...`)으로 발행하지만 cobot2 코드는 cv2 직접 사용 중 → **개념 충돌**(Rule 7 명시성). Phase 4에서 통합 검토.

---

## 7. 음성 / 오디오 stack — **REMOVED 2026-05-04 (옵션 A: 파일 삭제 완료)**

| 파일 | 의존성 | 용도 | 비고 |
|------|--------|------|------|
| `voice_control_node.py:11,67` | `openwakeword.model.Model` | wakeword 감지 (TFlite) | 모델: `models/hello_rokey_8332_32.tflite` (203KB, 존재) |
| `voice_control_node.py:22` | `WAKEWORD_MODEL_PATH` 하드코딩 (`/home/kyb/...`) | — | M1-3과 동일 — Phase 4 환경변수화 |
| `miccheck.py:1`, `miccontroller.py:1,11,17` | `pyaudio.PyAudio()` | 마이크 캡처 (16-bit int) | venv `pyaudio` 필요 — handoff 미언급, **install verify needed** |
| `STT.py:2` | `sounddevice as sd` | 마이크 캡처 (대안) | `voice_control_node.py`와 중복 stack |

**호출 흐름** (`voice_control_node.py`):

```
PyAudio stream
  → wakeword detection (openWakeWord, hello_rokey)
  → trigger Whisper recording window
  → OpenAI Whisper API (model="whisper-1")
  → publish /voice_command (std_msgs/String)
```

**중복 코드**:
- `STT.py`도 같은 OpenAI Whisper 호출. `voice_control_node.py`와 어느 한쪽이 dead — **Phase 4에서 식별**.

---

## 8. DR_init / DSR_ROBOT2 (vendored 통합)

### `robot_action.py`

```python
# robot_action.py:6
import DR_init

# robot_action.py:155 (callback 내부 import, lazy)
from DSR_ROBOT2 import movej, movel, mwait, wait

# robot_action.py:304-306
DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
DR_init.__dsr__node  = robot_node
```

**제공 패키지**: `dsr_bringup2` (vendored, ec92425 commit).

**Phase 1-2 연결**:
- `cobot2/robot_action.py`는 `/dsr01/dsr_controller2` 의 ROS2 service들을 **직접 호출 안 함** — DR_init Python 래퍼(DRCF 직결) 사용. Phase 1-2 doc의 "streaming command 토픽 publisher 0" 관찰과 일치.

**Tier 0 (CLAUDE.md #1) — virtual mode first**:
- `robot_action.py:32` `ROBOT_MODE = os.getenv("ROBOT_MODE", "virtual")` — 안전 default = virtual. 환경변수 미설정 시 hardware 활성 안 됨. ✓

---

## Open Issues (Phase 4 후속)

### 하드코딩 secrets / 환경 의존 값

1. `main.py:20` Firebase Service Account JSON 경로 (`/home/kyb/...`) — env화 미적용. (M1-3, 이전 세션부터 carry-over)
3. `robot_action.py:26-27` `TOOLCHARGER_IP`, `TOOLCHARGER_PORT` 모듈 상수 — Rule 8 (환경 의존 값) ROS2 파라미터화 후보.
4. `stockfish.py:12` `STOCKFISH_PATH` 모듈 상수 — 환경변수화 후보.
5. `vision_db.py:21` `SOURCE` (카메라 인덱스) — 환경변수화 후보.
6. `UI.html:114` Firebase Web SDK `apiKey` 하드코딩 — RTDB rules audit + 환경변수 주입 검토.

### 음성/OpenAI 처리 — **RESOLVED 2026-05-04 (옵션 A 실행)**

- STT.py, voice_control_node.py, miccheck.py, miccontroller.py, run_voice.sh, voice_control.launch.py 삭제 완료.
- voice_command Topic → /main_controller/start_sampling (std_srvs/Trigger) Service 교체.
- voice_status dead pub, _publish_wake_up() 제거 완료.
- 중복 코드(STT.py ↔ voice_control_node.py) 삭제로 함께 해소.

### Stale handoff 정정 대상

- `## Remaining Issues > 모델 가중치 부재`: best.pt(19MB) + classifier.pt(43MB) 모두 존재. 갱신 필요.

---

## Cross-References

- Phase 1-1 (코드 직독): `docs/code-mapping/2026-05-01-phase1-1-node-diagrams.md`
- Phase 1-2 (런타임 endpoint): `docs/code-mapping/2026-05-01-phase1-2-topic-inventory.md`
- Input Policy: `.claude/rules/ai-constitution.md` III
- Tier 0 Hard Rules: `CLAUDE.md` Hard Rules 1~7
