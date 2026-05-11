# Phase 1-2 — Topic / Service / Action Inventory (실측)

> 작성일: 2026-05-01 · 작성자: 인계자 (현 작업자)
> 목적: virtual mode `bringup.launch.py` 기동 상태에서 실측한 ROS2 endpoint 인벤토리.
> 보완: Phase 1-1 (코드 직독 mapping)은 chess_ai 4개 entry point만 다룸. 본 문서는 **DRCF + DSR controller + gripper_virtual** 의 실제 노출 표면.
> **원칙**: 캡처 시점의 daemon 응답을 그대로 기록. 미검증 추정은 `# verify needed`.

## Capture Baseline

```bash
source /opt/ros/humble/setup.bash
source /home/rokey/cobot2_chess_ai/install/setup.bash
ros2 launch m0609_rg2_bringup bringup.launch.py     # mode=virtual default
```

- 시점: 2026-05-01 ~19:10 KST
- DRCF 컨테이너: `dsr01_emulator` (`doosanrobot/dsr_emulator:3.0.1`)
- ros2_control_node 상태: `STATE_STANDBY` 도달, `joint_state_broadcaster` + `dsr_controller2` configured & activated
- Raw capture artifacts: `/tmp/inventory/` (nodes.txt / topics_typed.txt / services_typed.txt / actions_typed.txt / node_*.txt / qos_profiles.txt)
- **Out of scope**: `chess_ai` package 노드(`main`, `stockfish`, `robotaction`, `object`) — 본 launch는 bringup만 띄우며 chess_ai entry point는 별도 실행 필요. 본 문서는 bringup 노출 표면 + `chess_ai_interfaces` 정의 표면을 함께 기록.

---

## Node Inventory (10)

| 노드 | 패키지 | 역할 | 비고 |
|------|--------|------|------|
| `/dsr01/controller_manager` | controller_manager (vendored) | 컨트롤러 lifecycle / load·unload | ros2_control_node spawn |
| `/dsr01/dsr_controller2` | dsr_controller2 (vendored) | 두산 motion 명령 게이트웨이 | 모든 `/dsr01/motion/*`, `/dsr01/system/*`, `/dsr01/io/*`, action server 2건 보유 |
| `/dsr01/joint_state_broadcaster` | joint_state_broadcaster (vendored) | 하드웨어 joint 상태 → topic | publishes `/dsr01/joint_states` (RELIABLE+TRANSIENT_LOCAL — 후술) |
| `/dsr01/virtual_node` | dsr_bringup2 (vendored) | DRCF 에뮬레이터 lifecycle 관리 | 외부 publisher/subscriber 없음 (parameter only) |
| `/gripper_virtual_node` | m0609_rg2_bringup (**first-party**, Seooooooogi 작성) | virtual mode RG2 그리퍼 | `/onrobot/sendCommand` 서비스 + `/gripper_joint_states` 50Hz |
| `/joint_state_publisher` | joint_state_publisher_gui (apt) | `/dsr01/joint_states` + `/gripper_joint_states` 합성 → `/joint_states` | RViz 시각화 백본 |
| `/robot_state_publisher` | robot_state_publisher (apt) | URDF + joint_states → TF tree | publishes `/tf`, `/tf_static`, `/robot_description` |
| `/rviz2` | rviz2 (apt) | 시각화 | RViz config from m0609_rg2_bringup/rviz/ |
| `/static_transform_publisher` | tf2_ros (apt) | world → base_link 정적 TF | bringup.launch.py 인자로 정의 |
| `/transform_listener_impl_…` | tf2_ros (RViz 내부) | RViz의 TF listener 인스턴스 | 노드 이름은 hex addr — 의미 없음 |

---

## Topic Inventory (30개, 시스템 토픽 제외)

### 명령(command) 토픽 — `/dsr01/dsr_controller2`가 구독

DSR controller가 streaming 입력으로 받는 motion 명령:

| 토픽 | 타입 | Pub/Sub | 비고 |
|------|------|---------|------|
| `/dsr01/alter_motion_stream` | `dsr_msgs2/msg/AlterMotionStream` | sub:1 / pub:0 | alter_motion 활성 시 사용 |
| `/dsr01/servoj_stream` | `dsr_msgs2/msg/ServojStream` | sub:1 / pub:0 | joint space servoing |
| `/dsr01/servoj_rt_stream` | `dsr_msgs2/msg/ServojRtStream` | sub:1 / pub:0 | RT realtime variant |
| `/dsr01/servol_stream` | `dsr_msgs2/msg/ServolStream` | sub:1 / pub:0 | task space servoing |
| `/dsr01/servol_rt_stream` | `dsr_msgs2/msg/ServolRtStream` | sub:1 / pub:0 | |
| `/dsr01/speedj_stream` | `dsr_msgs2/msg/SpeedjStream` | sub:1 / pub:0 | joint velocity |
| `/dsr01/speedj_rt_stream` | `dsr_msgs2/msg/SpeedjRtStream` | sub:1 / pub:0 | |
| `/dsr01/speedl_stream` | `dsr_msgs2/msg/SpeedlStream` | sub:1 / pub:0 | task velocity |
| `/dsr01/speedl_rt_stream` | `dsr_msgs2/msg/SpeedlRtStream` | sub:1 / pub:0 | |
| `/dsr01/torque_rt_stream` | `dsr_msgs2/msg/TorqueRtStream` | sub:1 / pub:0 | RT torque |

> **Rule 9 검토**: 위 streams는 모두 motion 명령(연속 흐름) — Topic 사용은 Rule 2 (지속적 흐름 + 최신값)에 부합. 단, **사용자 측 publisher가 0개** → 본 launch에서 활용 안 됨. chess_ai `robot_action.py`가 DR_init API로 호출하는지 별도 검증 필요. # verify needed

### 상태(state) 토픽 — DSR controller가 발행

| 토픽 | 타입 | Pub/Sub | QoS | 비고 |
|------|------|---------|-----|------|
| `/dsr01/joint_states` | `sensor_msgs/msg/JointState` | pub:`joint_state_broadcaster` / sub:`joint_state_publisher` | **pub: RELIABLE+TRANSIENT_LOCAL** / sub: RELIABLE+VOLATILE | **Durability 비대칭 — 후술 Rule 4 위반** |
| `/dsr01/dynamic_joint_states` | `control_msgs/msg/DynamicJointState` | pub:1 / sub:0 | RELIABLE+TRANSIENT_LOCAL | controller 표준, 본 launch에서 미사용 |
| `/dsr01/error` | `dsr_msgs2/msg/RobotError` | pub:`dsr_controller2` / sub:0 | RELIABLE+VOLATILE | 에러 이벤트 — 구독자 없음 |
| `/dsr01/robot_disconnection` | `dsr_msgs2/msg/RobotDisconnection` | pub:`dsr_controller2` / sub:0 | (미캡처) | 연결 단절 이벤트 — 구독자 없음 |
| `/dsr01/io/ctrl_box_digital_input_state` | `std_msgs/msg/UInt8MultiArray` | pub:`dsr_controller2` / sub:0 | (미캡처) | **Rule 1 위반: `UInt8MultiArray` 사용** (의미 표현 불가). vendored이므로 직접 수정 불가 (Rule 6). |
| `/dsr01/joint_state_broadcaster/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | pub:1 / sub:0 | (미캡처) | controller lifecycle |
| `/dsr01/dsr_controller2/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | pub:1 / sub:0 | (미캡처) | controller lifecycle |

### 그리퍼 / 합성 / TF / 카메라

| 토픽 | 타입 | Pub/Sub | QoS | 비고 |
|------|------|---------|-----|------|
| `/gripper_joint_states` | `sensor_msgs/msg/JointState` | pub:`gripper_virtual_node` / sub:`joint_state_publisher` | RELIABLE+VOLATILE (양쪽 일치) | 50Hz 시뮬 — 컨테이너 그리퍼 상태 |
| `/joint_states` | `sensor_msgs/msg/JointState` | pub:`joint_state_publisher` / sub:`robot_state_publisher` | pub: RELIABLE+VOLATILE / sub: BEST_EFFORT+VOLATILE | **Reliability 비대칭** — 후술 |
| `/robot_description` | `std_msgs/msg/String` | pub:`robot_state_publisher` / sub:`joint_state_publisher`, `rviz2` | RELIABLE+TRANSIENT_LOCAL (latched 정적 설정) | URDF — Rule 4 권장과 일치 |
| `/tf` | `tf2_msgs/msg/TFMessage` | pub:`robot_state_publisher` / sub:RViz TF listener | RELIABLE+VOLATILE | 표준 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | pub:2 (`robot_state_publisher`, `static_transform_publisher`) / sub:1 | RELIABLE+TRANSIENT_LOCAL | 표준 |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` (예상) | pub:0 / sub:0 | — | RViz config가 expects, 실제 publisher 없음 (RealSense 미기동) |
| `/camera/color/image_raw` | (동일) | pub:0 / sub:0 | — | |
| `/camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` (예상) | pub:0 / sub:0 | — | |

> **카메라 토픽**: `bringup.launch.py`엔 RealSense 미포함 → `bringup_camera.launch.py`에서만 발행. 본 launch에선 RViz가 빈 토픽 expects만 함.

### RViz 인터랙션 (수동)

| 토픽 | 타입 | 비고 |
|------|------|------|
| `/clicked_point` | `geometry_msgs/msg/PointStamped` (예상) | RViz 클릭 |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | RViz 2D pose 추정 |
| `/move_base_simple/goal` | `geometry_msgs/msg/PoseStamped` | RViz 2D goal — 본 시스템 미사용 |

---

## Service Inventory (실 비-parameter 서비스만, ~140개 중 발췌)

### `/dsr01/dsr_controller2`가 노출 — 호출 후보 ★ 표시

#### Motion (`/dsr01/motion/*` — 27개)

★ `move_home`, ★ `move_joint`, ★ `move_jointx`, ★ `move_line`, ★ `move_blending`, ★ `move_circle`, ★ `move_periodic`, ★ `move_spiral`, ★ `move_spline_joint`, ★ `move_spline_task`, ★ `move_pause`, ★ `move_resume`, ★ `move_stop`, ★ `move_wait`, `alter_motion`, `change_operation_speed`, `check_motion`, `disable_alter_motion`, `enable_alter_motion`, `fkin`, `ikin`, `jog`, `jog_multi`, `set_ref_coord`, `set_singular_handling_force`, `set_singularity_handling`, `trans`

> **추정**: `chess_ai/robot_action.py`는 DR_init Python 래퍼(DRCF) 직호출이지 ROS2 service 호출이 아닐 가능성 — Phase 1-1 doc 확인 필요. # verify needed

#### Aux Control / Force / Realtime (~50개)

상태 조회(`get_current_posj`, `get_current_posx`, `get_external_torque` 등 19개) + 컴플라이언스(`task_compliance_ctrl`, `set_stiffnessx` 등 23개) + RT 제어(`connect_rt_control`, `set_velj_rt` 등 16개).

#### System (`/dsr01/system/*` — 13개)

`get_robot_state`, `get_robot_mode`, `set_robot_mode`, `set_safety_mode`, `servo_off`, `change_collision_sensitivity` 등.

> **Rule 9 (안전) 검토**: `set_safety_mode`, `servo_off`, `set_safe_stop_reset_type`은 안전 신호 — Service로 노출되어 Rule 9 (Topic 금지) 부합. 다만 QoS 캡처 미실시.

#### IO / DRL / TCP / Tool / Modbus / PLC (~50개)

GPIO·디지털 IO·Modbus·PLC 레지스터·TCP/Tool 정의 — 본 launch 미사용.

### `/gripper_virtual_node`가 노출

| 서비스 | 타입 | 용도 |
|--------|------|------|
| `/onrobot/sendCommand` | `onrobot_rg_msgs/srv/SetCommand` | OnRobot 드라이버와 동일 인터페이스 (실기 코드 무수정 호환). virtual에선 애니메이션 완료까지 blocking 응답. |

### Controller manager

`/dsr01/controller_manager/{configure,switch,load,unload,list}_*` (16개) — 표준 ros2_control 인터페이스.

---

## Action Inventory (2)

| 액션 | 타입 | 서버 | 클라이언트 |
|------|------|------|-----------|
| `/dsr01/motion/movej_h2r` | `dsr_msgs2/action/MovejH2r` | `dsr_controller2` | (없음) |
| `/dsr01/motion/movel_h2r` | `dsr_msgs2/action/MovelH2r` | `dsr_controller2` | (없음) |

> 두 action 모두 본 launch에서 client 0 — `chess_ai/robot_action.py`가 사용하는지 미검증. # verify needed

---

## `chess_ai_interfaces` 정의 (코드 표면, 실행 시 노출)

위 인벤토리는 bringup만 띄운 상태. chess_ai 패키지가 띄우면 추가로 다음이 노출됨:

| 종류 | 이름 | 정의 | 비고 |
|------|------|------|------|
| Action | `MoveChessPiece` | goal: `string command, string pieces_dict` (JSON) / result: `bool success, string message` / feedback: `string status` | `robot_action.py` 서버, `main.py` 클라이언트 |
| Service | `StockfishMove` | request: `string pieces_data, string turn, string last_move, int32 skill_level, int32 depth` / response: `string best_move, bool success` | `stockfish.py` 서버, `main.py` 클라이언트 |
| ~~Topic (sub)~~ | ~~`/voice_command`~~ | ~~`main.py` 구독~~ | **REMOVED 2026-05-04**: voice_command Topic 제거. |
| Service (server) | `/main_controller/start_sampling` (`std_srvs/srv/Trigger`) | `main.py` `_on_start_sampling` | IDLE→SAMPLING 트리거. state!=IDLE 시 success=False. |

> Phase 1-1 다이어그램에 따르면 `vision_db.py`는 ROS2 노드가 아님 (rclpy import 없음) — 실행해도 endpoint 추가 0.

---

## QoS 발견 (Rule 4 관점)

### 비대칭/위반 사례

| 토픽 | Pub QoS | Sub QoS | 영향 | 위반 주체 |
|------|---------|---------|------|-----------|
| `/dsr01/joint_states` | RELIABLE + **TRANSIENT_LOCAL** | RELIABLE + VOLATILE | 데이터 흐름은 정상(durability mismatch는 강한 쪽이 약한 쪽 receive 허용). **`ros2 topic hz` default subscriber(VOLATILE)와 호환되어 보이나 실제로 hang됨** — depth UNKNOWN + lifecycle 타이밍이 원인일 가능성. | publisher(`joint_state_broadcaster`, vendored ros2_control 표준 — Rule 6에 따라 직접 수정 금지) |
| `/joint_states` | RELIABLE + VOLATILE | **BEST_EFFORT** + VOLATILE | publisher가 RELIABLE이면 BEST_EFFORT subscriber도 수신 가능(downgrade) — 실용적 동작 OK. 단 Rule 4 권장(로봇 상태 = RELIABLE+VOLATILE)에 subscriber만 어긋남. | subscriber(`robot_state_publisher`, apt 패키지) |
| Streaming command 토픽들 | n/a (pub 0) | RELIABLE + VOLATILE | publisher 부재라 흐름 검증 불가 | n/a |

### 일치 사례 (Rule 4 권장과 부합)

| 토픽 | QoS | 부합 항목 |
|------|-----|-----------|
| `/gripper_joint_states` | RELIABLE+VOLATILE 양쪽 | "로봇 상태" 권장 일치 |
| `/robot_description`, `/tf_static` | RELIABLE+TRANSIENT_LOCAL | "정적 설정/latched" 권장 일치 |
| `/tf` | RELIABLE+VOLATILE | 표준 일치 |

### Quirk: `ros2 topic hz /dsr01/joint_states` hang

- **사실**: `topic echo --once /dsr01/joint_states`는 즉시 메시지 수신 (데이터 흐름 정상).
- **사실**: `ros2 topic hz /dsr01/joint_states`는 응답 없이 hang (이전 세션 + 본 세션 모두 재현).
- **추정 원인**: publisher TRANSIENT_LOCAL durability + history depth UNKNOWN + `topic hz` 내부 subscriber QoS 호환 비대칭. # verify needed
- **워크어라운드**: `ros2 topic echo --no-arr --field header.stamp /dsr01/joint_states` + 자체 카운팅 스크립트.

---

## Open Issues (verification gaps)

1. **streaming command 토픽 사용 주체**: `chess_ai/robot_action.py`가 `/dsr01/servoj_stream` 등을 발행하는지, 아니면 DR_init Python 래퍼 직호출인지 확인 필요. Phase 1-1 매핑 + grep 권장.
2. **action client 부재**: `/dsr01/motion/movej_h2r`, `/movel_h2r` 사용 코드 식별 필요 (vendored DSR 예제 또는 chess_ai 측인지).
3. **`/dsr01/io/ctrl_box_digital_input_state`** Rule 1 위반(`UInt8MultiArray`) — vendored이므로 본 프로젝트 외 위반. 기록만.
4. ~~**`/voice_command` 토픽**~~: **RESOLVED 2026-05-04** — Topic 제거. `/main_controller/start_sampling` (std_srvs/Trigger) Service로 교체. voice_control_node 미기동 무한 대기 해소.
5. **`/dsr01/robot_disconnection`, `/dsr01/error`**: 구독자 0 — 연결 단절 / 에러 이벤트가 무시되고 있음. chess_ai 측 구독자 추가 검토 가치 있음 (Rule 7 silent failure 방지).

---

## Cross-References

- Phase 1-1 (코드 직독): `docs/code-mapping/2026-05-01-phase1-1-node-diagrams.md`
- Raw capture: `/tmp/inventory/{nodes,topics_typed,services_typed,actions_typed,qos_profiles}.txt` + `/tmp/inventory/node_*.txt`
- DSR launch arg 요약: `~/.claude/projects/-home-rokey-cobot2-chess-ai/memory/reference_m0609_rg2_integration_repo.md`
- `gripper_virtual_node` 출처: `~/.claude/projects/-home-rokey-cobot2-chess-ai/memory/project_gripper_virtual_node_origin.md`
