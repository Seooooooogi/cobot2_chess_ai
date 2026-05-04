# Plan: Voice/OpenAI Removal + Trigger Service Migration

> 2026-05-04. Phase 4 잔여. brainstorming → writing-plans 게이트 통과 후 atomic task list.
> 옵션 A(파일 삭제) + 옵션 1(`std_srvs/Trigger` Service) + 옵션 a(voice_status dead pub 제거).

## Settled Design

| 항목 | 결정 |
|---|---|
| .srv 정의 | `std_srvs/Trigger` 표준 재사용 (cobot2_interfaces 변경 없음) |
| Service 이름 | `~/start_sampling` → `/main_controller/start_sampling` |
| Service QoS | `rmw_qos_profile_services_default` (ROS2 Rule 4, create_service 기본) |
| 동시성 | `state != IDLE` 시 `success=False, message="busy: state=<현재>"` 반환. `_state_lock` 재사용 |
| voice_command 매직 스트링 | `PASS_COMMAND` 상수 제거 |
| voice_ui_status 처리 | sub + 콜백 + Firebase voice_message 갱신 코드 전량 제거. UI.html 손대지 않음 |
| voice_status 처리 (옵션 a) | Pub + `VOICE_STATUS_TOPIC` + `WAKE_UP_SIGNAL` 상수 + `_publish_wake_up()` 메서드 + 호출 2개 전량 제거 |
| 커밋 분할 | C1(main.py 변경 + 파일 삭제 + launch/setup 정리) → C2(문서) — 2개 |
| baseline | `outputs/baseline/2026-05-04-voice-removal/` (변경 *전* 캡처) |
| Ready 신호 | 없음. README에 "main 노드 startup 로그 확인 후 service call" 운영 메모 |

## Scope Notes

- **chess_system.launch.py**는 entry_points가 이미 실행파일명과 불일치한 dead 상태 — 본 plan은 voice 블록만 정리. 나머지 dead 문제는 별도 트랙.
- **cobot2_interfaces 변경 없음** — `std_srvs/Trigger` 재사용. `colcon build --packages-select cobot2`만으로 충분.
- **String import 제거 가능** — `String`은 voice_status Pub과 `_on_voice_*` 콜백에서만 사용. 옵션 a 적용 후 다른 사용처 없음 → import 제거.

---

## Phase A — Pre-flight (read-only, 변경 0)

### Task A1: voice 관련 심볼 전수 grep
- File: repo 루트
- Action:
  ```bash
  grep -rn \
    "voice_control_node\|run_voice\.sh\|voice_command\|voice_ui_status\|voice_status\|VOICE_COMMAND_TOPIC\|VOICE_UI_STATUS_TOPIC\|VOICE_STATUS_TOPIC\|PASS_COMMAND\|WAKE_UP_SIGNAL\|OPENAI_API_KEY\|WAKEWORD_MODEL_PATH" \
    --include="*.py" --include="*.sh" --include="*.md" --include="*.html" \
    /home/rokey/cobot2_chess_ai/
  ```
- Validation: 출력 결과를 기록. design 문서가 나열한 6개 파일 + 문서 5개 외 hits가 있으면 task 범위에 추가.
- Depends on: 없음

### Task A2: .env.example 현재 상태 확인
- File: `src/cobot2/.env.example`
- Action: `git diff src/cobot2/.env.example` — `OPENAI_API_KEY`, `WAKEWORD_MODEL_PATH`, `VOICE_INPUT_ENABLED`의 줄 번호 기록.
- Validation: 위치 기록 완료.
- Depends on: 없음

### Task A3: chess_system.launch.py voice 블록 line 범위 확인
- File: `src/cobot2/launch/chess_system.launch.py`
- Action: voice_control_node Node 블록의 정확한 line 범위 확인 (주석 포함).
- Validation: 라인 범위 기록.
- Depends on: 없음

### Task A4: String import 잔존 사용처 확인
- File: `src/cobot2/cobot2/main.py`
- Action: `grep -n "String" src/cobot2/cobot2/main.py`
- Validation: `String` 참조가 voice_status pub + `_on_voice_*` 콜백 외에 없음을 확인. 외부 사용처 없으면 C1에서 `from std_msgs.msg import String` import 제거 대상에 포함.
- Depends on: 없음

---

## Phase B — Baseline 기록 (커밋 0, 변경 *전*)

### Task B1: baseline 디렉터리 생성
- Action: `mkdir -p /home/rokey/cobot2_chess_ai/outputs/baseline/2026-05-04-voice-removal/`
- Validation: 디렉터리 존재 확인.
- Depends on: A1~A4

### Task B2: main 노드 ready 로그 캡처
- File: `outputs/baseline/2026-05-04-voice-removal/b-main-ready.log`
- Action:
  ```bash
  source install/setup.bash
  ros2 run cobot2 main 2>&1 | head -20 \
    > outputs/baseline/2026-05-04-voice-removal/b-main-ready.log &
  sleep 5; kill %1
  ```
- Validation: `grep "Waiting for voice_command" b-main-ready.log` 1줄 이상.
- Depends on: B1

### Task B3: voice_command pub → SAMPLING 진입 로그 캡처
- File: `outputs/baseline/2026-05-04-voice-removal/b-sampling-trigger.log`
- Action:
  ```bash
  ros2 run cobot2 main 2>&1 > b-sampling-trigger.log &
  sleep 5
  ros2 topic pub --once /voice_command std_msgs/msg/String "data: 'pass'"
  sleep 3; kill %1
  ```
- Validation: `grep "\[PASS\]\|\[SAMPLING\]" b-sampling-trigger.log` 결과 존재.
- Depends on: B2

---

## Phase C — main.py 변경 + 파일 삭제 + launch/setup 정리 (단일 커밋 C1)

### Task C1: import 정리
- File: `src/cobot2/cobot2/main.py`
- Action: `from std_srvs.srv import Trigger` 추가. `from std_msgs.msg import String` 제거 (A4에서 외부 사용처 없음 확인된 경우).
- Validation: `grep -n "from std_msgs\|from std_srvs" main.py` — Trigger 1줄, String 0줄.
- Depends on: A4

### Task C2: 모듈 docstring 갱신 (lines 9-31 영역)
- File: `src/cobot2/cobot2/main.py`
- Action: 모듈 docstring의 ROS2 Interfaces / Issues 섹션 교체:
  - `Pub: voice_status` 줄 제거 (line 10)
  - `Sub: voice_command` 줄 제거 (line 11)
  - `Sub: voice_ui_status` 줄 제거 (line 12)
  - 새 줄 삽입: `Service: ~/start_sampling (std_srvs/Trigger) — state-change trigger; IDLE→SAMPLING. Resolves to /main_controller/start_sampling.`
  - line 18 (`spawned in _on_voice_command`) → `spawned in _on_start_sampling`
  - line 27 (M1-1) → `M1-1 RESOLVED 2026-05-04: ~/start_sampling (Trigger) 로 대체.`
  - line 28 (M1-2) → `M1-2 PARTIAL: voice sub QoS 제거 완료. status_pub 항목은 voice_status 제거로 함께 해소.`
  - line 31 (M1-6) → `M1-6 RESOLVED 2026-05-04: Service 로 대체 — voice_control_node 미실행 무한 대기 해소.`
  - 추가 1줄: `M1-7 RESOLVED 2026-05-04: voice_status pub 제거 (옵션 a) — dead pub 해소.`
- Validation: `python3 -c "import ast; ast.parse(open('main.py').read())"` 통과.
- Depends on: C1

### Task C3: 모듈 상수 정리
- File: `src/cobot2/cobot2/main.py`
- Action: 다음 상수 5개 모두 제거:
  - `VOICE_COMMAND_TOPIC = "voice_command"`
  - `VOICE_STATUS_TOPIC = "voice_status"`
  - `VOICE_UI_STATUS_TOPIC = "voice_ui_status"`
  - `PASS_COMMAND = "pass"`
  - `WAKE_UP_SIGNAL = "WAKE_UP"`
- Validation: `grep -n "VOICE_COMMAND_TOPIC\|VOICE_STATUS_TOPIC\|VOICE_UI_STATUS_TOPIC\|PASS_COMMAND\|WAKE_UP_SIGNAL" main.py` → 0줄.
- Depends on: C2

### Task C4: `__init__` Pub/Sub 제거 + Service 서버 추가
- File: `src/cobot2/cobot2/main.py`
- Action: `MainController.__init__` 내에서:
  - 제거: `self.status_pub = self.create_publisher(String, VOICE_STATUS_TOPIC, 10)`
  - 제거: `self.cmd_sub = self.create_subscription(String, VOICE_COMMAND_TOPIC, self._on_voice_command, 10)`
  - 제거: `self.voice_ui_sub = self.create_subscription(String, VOICE_UI_STATUS_TOPIC, self._on_voice_ui_status, 10)`
  - 추가: `self.start_sampling_srv = self.create_service(Trigger, "~/start_sampling", self._on_start_sampling)`
- Validation: syntax OK + 위 3개 멤버 부재 확인.
- Depends on: C3

### Task C5: ready 로그 메시지 교체
- File: `src/cobot2/cobot2/main.py`
- Action: `"MainController ready. Waiting for voice_command='pass'."` → `"MainController ready. Service: /main_controller/start_sampling (std_srvs/Trigger)."`
- Validation: `grep "MainController ready" main.py` — Service: 형태.
- Depends on: C4

### Task C6: `_publish_wake_up()` 메서드 + 2개 호출 제거
- File: `src/cobot2/cobot2/main.py`
- Action:
  - `_publish_wake_up()` 메서드 전체 제거 (line 556 영역).
  - line 345 `self._publish_wake_up()` 호출 제거.
  - line 488 `self._publish_wake_up()` 호출 제거.
  - 메서드 호출 컨텍스트의 docstring(line 418 `voice_status` 언급)도 함께 갱신 — voice_status 언급 제거, "transitions self._state to IDLE"만 유지.
- Validation: `grep -n "_publish_wake_up\|voice_status" main.py` → 0줄.
- Depends on: C5

### Task C7: `_on_voice_ui_status` 메서드 제거
- File: `src/cobot2/cobot2/main.py`
- Action: `_on_voice_ui_status` 메서드 전체 제거 (line 255-265, Firebase voice_message 갱신 포함).
- Validation: `grep -n "_on_voice_ui_status\|voice_message" main.py` → 0줄.
- Depends on: C6

### Task C8: `_on_voice_command` → `_on_start_sampling` 교체
- File: `src/cobot2/cobot2/main.py`
- Action: `_on_voice_command` 메서드 전체 제거 후 동일 위치에 `_on_start_sampling(self, request, response)` 삽입:
  ```python
  def _on_start_sampling(
      self, request: Trigger.Request, response: Trigger.Response
  ) -> Trigger.Response:
      with self._state_lock:
          if self._state != "IDLE":
              response.success = False
              response.message = f"busy: state={self._state}"
              self.get_logger().warn(
                  f"start_sampling rejected (state={self._state})."
              )
              return response
          self._state = "SAMPLING"
          self._job_id = now_iso_ms()
          job_id = self._job_id

      self.get_logger().info(f"[start_sampling] triggered. job_id={job_id}")
      self._reset_ui_for_new_job(job_id)
      t = threading.Thread(
          target=self._job_make_and_publish_board, args=(job_id,), daemon=True
      )
      t.start()
      response.success = True
      response.message = "sampling started"
      return response
  ```
- Validation: `grep -n "_on_voice_command\|PASS_COMMAND" main.py` → 0줄. syntax OK.
- Depends on: C7

### Task C9: MainController 클래스 docstring Triggers 섹션 갱신
- File: `src/cobot2/cobot2/main.py`
- Action: 클래스 docstring의 Triggers 섹션에서 `voice_command="pass"` 언급 제거 후 다음으로 교체:
  ```
  Triggers:
      - Service ``~/start_sampling`` (Trigger): IDLE → SAMPLING.
        Returns success=False with message="busy: state=<state>" if not IDLE.
  ```
- Validation: syntax OK.
- Depends on: C8

### Task C10: line drift 자가 검사
- File: `src/cobot2/cobot2/main.py`
- Action: `grep -n "line [0-9]\+" main.py` — 모든 `line N` 인용을 sed/Read로 cross-check. 옵션 a로 코드량이 ~30줄 줄어들었으므로 모든 docstring 내 라인 번호가 shift됨. lessons.md 2026-05-01 교훈.
- Validation: 각 인용된 line 번호가 실제 expected symbol을 가리키는지 확인.
- Depends on: C9

### Task C11: 음성 관련 파일 6개 삭제
- File: 다음 6개 파일
  - `src/cobot2/cobot2/voice_control_node.py`
  - `src/cobot2/cobot2/STT.py`
  - `src/cobot2/cobot2/miccheck.py`
  - `src/cobot2/cobot2/miccontroller.py`
  - `src/cobot2/cobot2/run_voice.sh`
  - `src/cobot2/launch/voice_control.launch.py`
- Action: `rm` (절대경로).
- Validation: `ls` 결과 6개 파일 부재.
- Depends on: C10

### Task C12: chess_system.launch.py voice 블록 제거
- File: `src/cobot2/launch/chess_system.launch.py`
- Action: lines 55-67 (주석 `# 4. Voice Control Node`부터 Node 블록 닫힘 `),`까지) 제거. 제거 후 `# 5. Chess Integration Node` → `# 4.` 주석 번호 재정렬.
- Validation: syntax OK + `grep "voice_control_node" launch.py` → 0줄.
- Depends on: C11

### Task C13: setup.py 정리
- File: `src/cobot2/setup.py`
- Action: 2줄 제거:
  - `(os.path.join('share', package_name), glob('cobot2/*.sh')),` (run_voice.sh 삭제 후 빈 결과)
  - `(os.path.join('lib', package_name), ['cobot2/run_voice.sh']),`
- Validation: `grep "run_voice\|\.sh" setup.py` → 0줄. syntax OK.
- Depends on: C12

### Task C14: build/cobot2 stale 아티팩트 제거 후 colcon build
- Action:
  ```bash
  rm -rf /home/rokey/cobot2_chess_ai/build/cobot2 /home/rokey/cobot2_chess_ai/install/cobot2
  colcon build --packages-select cobot2
  source install/setup.bash
  ```
- Validation: `colcon build` 종료 코드 0. (`feedback_build_artifact_trap` 메모리: 파일 삭제 시 stale 빌드 아티팩트 trap 회피.)
- Depends on: C13

### Task C15: 런타임 Service 검증
- Action:
  ```bash
  ros2 run cobot2 main &
  sleep 5
  ros2 service list | grep start_sampling
  # 예상: /main_controller/start_sampling
  ros2 service call /main_controller/start_sampling std_srvs/srv/Trigger {}
  # 예상: success=True, message='sampling started'
  ros2 service call /main_controller/start_sampling std_srvs/srv/Trigger {}
  # 예상: success=False, message='busy: state=SAMPLING' (또는 IDLE 복귀 후 True)
  kill %1
  ```
- Validation: service 등록 1줄 + 첫 call success=True + ready 로그 갱신 확인.
- Depends on: C14

### Task C16: C1 커밋 (main.py + 파일 삭제 + launch/setup)
- Action:
  ```bash
  git add src/cobot2/cobot2/main.py \
          src/cobot2/cobot2/voice_control_node.py \
          src/cobot2/cobot2/STT.py \
          src/cobot2/cobot2/miccheck.py \
          src/cobot2/cobot2/miccontroller.py \
          src/cobot2/cobot2/run_voice.sh \
          src/cobot2/launch/voice_control.launch.py \
          src/cobot2/launch/chess_system.launch.py \
          src/cobot2/setup.py
  git commit -m "refactor(main): replace voice_command Topic with Trigger Service
  
  - voice_command (std_msgs/String) Sub → std_srvs/Trigger Service ~/start_sampling
  - voice_status (std_msgs/String) Pub + _publish_wake_up() 제거 (dead pub, 옵션 a)
  - voice_ui_status (std_msgs/String) Sub + Firebase voice_message 갱신 제거
  - state != IDLE 시 success=False + message='busy: state=<state>' 반환
  - voice_control_node.py / STT.py / miccheck.py / miccontroller.py / run_voice.sh / voice_control.launch.py 삭제
  - chess_system.launch.py: voice_control_node 블록 제거
  - setup.py: run_voice.sh / cobot2/*.sh 설치 항목 제거
  
  ROS2 Rule 2 (M1-1) RESOLVED: 상태 변경 트리거를 Service로 교체.
  ROS2 Rule 7 (M1-7) RESOLVED: voice_status dead pub 제거.
  음성 stack 외부 의존성(PyAudio, openWakeWord, OpenAI Whisper) 제거."
  ```
- **No AI attribution** — `Co-Authored-By: Claude` 등 금지 (Hard Rule 7).
- Validation: `git log --oneline -1` 확인.
- Depends on: C15

---

## Phase D — 문서 갱신 (단일 커밋 C2)

### Task D1: CLAUDE.md
- File: `CLAUDE.md`
- Action: Quick Ref 섹션의 `- Voice: cobot2/run_voice.sh` 줄 제거.
- Depends on: C16

### Task D2: README.md
- File: `README.md`
- Action: 음성 관련 인용 제거. service call 운영 메모 추가:
  ```
  ### 게임 시작 트리거
  
  main 노드 startup 로그에서 "Service: /main_controller/start_sampling" 확인 후 호출:
  
  ```bash
  ros2 service call /main_controller/start_sampling std_srvs/srv/Trigger {}
  ```
  ```
- Depends on: C16

### Task D3: phase1-1-node-diagrams.md
- File: `docs/code-mapping/2026-05-01-phase1-1-node-diagrams.md`
- Action:
  - System Interaction 다이어그램에서 voice_control_node 박스 + voice_command/voice_status 화살표 제거. `~/start_sampling` Service 화살표 추가.
  - MainController ROS2 Interfaces 표에서 Pub voice_status / Sub voice_command / Sub voice_ui_status 행 제거. Service `~/start_sampling` 행 추가.
  - Issues 표 M1-1, M1-2(voice 부분), M1-6, M1-7 모두 `RESOLVED 2026-05-04` 처리.
- Depends on: C16

### Task D4: phase1-2-topic-inventory.md
- File: `docs/code-mapping/2026-05-01-phase1-2-topic-inventory.md`
- Action: voice_command / voice_status / voice_ui_status 토픽 행 제거. Service `~/start_sampling` 추가. Open Issues item 4 해소 처리.
- Depends on: C16

### Task D5: phase1-3-external-deps.md
- File: `docs/code-mapping/2026-05-01-phase1-3-external-deps.md`
- Action:
  - Summary 표에서 OpenAI / openWakeWord / PyAudio / sounddevice 관련 행 `REMOVED 2026-05-04` 처리.
  - 섹션 2 / 7 본문에 `Status: 2026-05-04 삭제 완료 (옵션 A)` 추가.
  - Open Issues "음성/OpenAI 처리 결정 (Phase 4)" → `RESOLVED 2026-05-04`.
- Depends on: C16

### Task D6: .env.example
- File: `src/cobot2/.env.example`
- Action: A2 결과 기반으로 다음 제거:
  - `VOICE_INPUT_ENABLED=0`
  - OpenAI 섹션 (`# === OpenAI ... ===` + `# OPENAI_API_KEY=`)
  - `WAKEWORD_MODEL_PATH` (존재 시)
- 상단 주석에 `# voice/OpenAI stack removed 2026-05-04` 1줄 추가.
- Validation: `grep "OPENAI\|VOICE_INPUT\|WAKEWORD" .env.example` → 0줄.
- Depends on: C16

### Task D7: DEVELOPMENT_ROADMAP.md
- File: `docs/DEVELOPMENT_ROADMAP.md`
- Action:
  - Phase 4에 완료 항목 추가: `- [x] 4-voice. voice_command Topic → std_srvs/Trigger Service 전환 (2026-05-04). voice stack 삭제 (옵션 A). M1-1/M1-6/M1-7 RESOLVED.`
  - Phase 3-2 항목(`음성 인식 노드 단독`) 취소선 + 주석 `(voice stack 삭제 — 2026-05-04)`.
- Depends on: C16

### Task D8: phase1-4-config-inventory.md
- File: `docs/code-mapping/2026-05-01-phase1-4-config-inventory.md`
- Action: env/config 표에서 `OPENAI_API_KEY`, `WAKEWORD_MODEL_PATH`, `VOICE_INPUT_ENABLED` 항목 제거 (등재 시).
- Depends on: C16

### Task D9: C2 커밋 (문서 갱신)
- Action:
  ```bash
  git add CLAUDE.md README.md \
          docs/code-mapping/2026-05-01-phase1-1-node-diagrams.md \
          docs/code-mapping/2026-05-01-phase1-2-topic-inventory.md \
          docs/code-mapping/2026-05-01-phase1-3-external-deps.md \
          docs/code-mapping/2026-05-01-phase1-4-config-inventory.md \
          docs/DEVELOPMENT_ROADMAP.md \
          src/cobot2/.env.example
  git commit -m "docs: update diagrams and roadmap for voice removal (Phase 4)
  
  - CLAUDE.md: run_voice.sh Quick Ref 제거
  - README.md: service call 운영 메모 추가 (~/start_sampling)
  - phase1-1-node-diagrams.md: voice_control_node 다이어그램 제거, start_sampling Service 반영. M1-1/M1-2/M1-6/M1-7 RESOLVED.
  - phase1-2-topic-inventory.md: voice_* 토픽 제거, Service 추가
  - phase1-3-external-deps.md: OpenAI/음성 의존성 REMOVED 처리
  - phase1-4-config-inventory.md: OPENAI_API_KEY/VOICE_INPUT_ENABLED/WAKEWORD_MODEL_PATH 제거
  - DEVELOPMENT_ROADMAP.md: Phase 4-voice 완료 + Phase 3-2 취소선
  - .env.example: 위 env 키 제거"
  ```
- Validation: `git log --oneline -2` C1·C2 두 커밋 확인.
- Depends on: D1~D8

---

## Phase E — Verification 게이트

### Task E1: 자가 체크리스트
- Action:
  ```bash
  rm -rf build/cobot2 install/cobot2
  colcon build --packages-select cobot2
  source install/setup.bash
  ros2 run cobot2 main &
  sleep 5
  ros2 service list | grep start_sampling
  ros2 service call /main_controller/start_sampling std_srvs/srv/Trigger {}
  grep -rn "VOICE_COMMAND_TOPIC\|VOICE_STATUS_TOPIC\|VOICE_UI_STATUS_TOPIC\|PASS_COMMAND\|WAKE_UP_SIGNAL\|_on_voice_command\|_on_voice_ui_status\|_publish_wake_up\|voice_control_node" \
    --include="*.py" src/
  python3 -c "from cobot2.main import MainController; print('import OK')"
  kill %1
  ```
- Validation: 빌드 0, service list 1줄, call success=True, grep 0줄, import OK.
- Depends on: D9

### Task E2: verification agent 호출
- Action: E1 통과 후 verification agent 호출. 전달 항목:
  - C1·C2 커밋 hash + diff 요약
  - E1 자가 체크 로그
  - 본 plan 파일 경로
- Validation: agent가 `DONE` 또는 `DONE_WITH_CONCERNS` 반환.
- Depends on: E1

---

## Total: 30 tasks

A:4 + B:3 + C:16 + D:9 + E:2 — Phase F는 verification agent 결과에 따라 추가 작업 분기.

## Hard Rules 적용

- Tier 0: no AI attribution in git (모든 커밋 메시지 검증), virtual mode first (실기 검증 X — virtual만), no fabrication (line ref drift 자가 검사), baseline before refactor (Phase B), no speculation (검증된 동작만 docstring 기록).
- ROS2 Rule 2 / 4 / 7 / 10 모두 충족.
