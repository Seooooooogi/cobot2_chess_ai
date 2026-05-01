# ROS2 Design Principles — AGENTS.md

> ROS2 패키지 작성·수정 시 에이전트가 준수해야 하는 핵심 규칙. 충돌 시 번호가 낮은 쪽이 우선하며, Rule 9(안전)와 Rule 10(메타)은 별도 우선순위를 가진다.

---

## Rule 1. 메시지는 의미를 표현한다

- 대응하는 표준 메시지(`geometry_msgs`, `sensor_msgs`, `nav_msgs` 등)가 있으면 **반드시 사용**한다. `std_msgs` 기본 타입(`Int32`, `Float64`, `String`, `Bool`)이나 `*MultiArray`로 대체 금지.
- `*MultiArray`는 **동질 수치 데이터 전용**. 의미·단위·타입이 다른 필드가 2개 이상이면 커스텀 `.msg` 정의.
- 공간 좌표·시간 의존·센서 데이터는 `std_msgs/Header` 필수. `Pose`/`Twist`처럼 Header가 없는 타입은 `*Stamped` 버전 우선 검토.
- 커스텀 메시지 필드명은 의미를 서술(`data`, `value`, `arr` 금지). 단위는 주석으로 명시(`# meters`, `# radians`). 이산 카테고리는 `uint8 MODE_X = 0` 형태의 정수 상수로 표현.

---

## Rule 2. 통신 패턴은 상호작용 성격에 따라 고른다

순차 판정:

1. 지속적 흐름 + 최신값만 필요 → **Topic**
2. 명령/질의 + 1초 이내 응답 → **Service**
3. 장기 실행·진행률·취소·피드백 필요 → **Action**

- Topic 금지 용도: 상태 변경 명령(시작/정지/리셋), 응답이 필요한 질의, **안전 신호**, 성공·실패가 구분되어야 하는 트랜잭션.
- Action은 Goal·Feedback·Result 모두 의미 있게 정의. Feedback이 비거나 Result가 단순 성공 플래그뿐이면 Service로 재설계.

---

## Rule 3. 단위는 SI 표준(REP-103)을 따른다

- 노드 간 통신은 **무조건 SI**: m, rad, s, kg, m/s, rad/s 등. mm, cm, deg, rpm, inch의 노드 간 전달 금지.
- 비-SI는 **시스템 경계에서만** 존재: 외부 입력(ingress)에서 즉시 SI로 변환, 외부 출력(egress)에서만 비-SI로 변환. 중간 경로에서 변환 금지.
- 커스텀 메시지 수치 필드·노드 파라미터에 단위 명시(`max_velocity_mps`, `timeout_sec`).

---

## Rule 4. QoS는 명시적으로 선언한다

- Publisher/Subscriber 생성 시 `10` 같은 **큐 사이즈 단축 표기 금지**. `QoSProfile`로 reliability·durability·history·depth 모두 명시.
- 데이터 성격별 기본:
  - 고빈도 센서 스트림 → `BEST_EFFORT` + `VOLATILE` (`rmw_qos_profile_sensor_data`)
  - 로봇 상태(joint_states, odom) → `RELIABLE` + `VOLATILE`
  - 명령·이벤트, 정적 설정(map, robot_description) → `RELIABLE` + `TRANSIENT_LOCAL`
  - 서비스 → `rmw_qos_profile_services_default`
- Pub/Sub QoS 호환성은 **작성 시점에 검증**. 비호환 조합은 경고 없이 연결 실패하므로 금지.
- rosbag2 녹화 대상은 QoS 문서화, 필요 시 `qos_override.yaml` 제공.

---

## Rule 5. 네임스페이스는 소유권을 표현한다

- 네임스페이스는 **"이 리소스가 누구에게 속하는가"**. 기능 분류(`/sensors/...`, `/control/...`)로 쓰지 않는다.
- 글로벌(루트) 배치는 다음만 허용:
  1. ROS2·RMW 표준 글로벌 리소스(`/tf`, `/tf_static`, `/clock`, `/rosout`, `/parameter_events`)
  2. 시스템 전체 공유 단일 인스턴스(전역 진단, 단일 맵)
  3. 오퍼레이터 ↔ 시스템 경계 단일 입력(단일 조이스틱)
- 리소스 이름만 보고 주체가 안 드러나면 글로벌 금지. 동일 종류가 2개 이상 존재 가능하면 글로벌 금지.
- 노드 코드는 **상대 경로로만 선언**. 절대 경로(`/`로 시작) 하드코딩 금지. 네임스페이스·리매핑은 launch 파일에서 적용.

---

## Rule 6. 재현성과 의존성 관리

- 단일 시스템 내 모든 노드는 **동일한 ROS2 배포판**. EOL 배포판 금지. 혼용이 불가피하면 브리지 전략 명시.
- 외부 ROS2 패키지 의존성은 `.repos`(vcstool)로 선언, **커밋 해시 또는 태그로 고정**. `main`/`master`/`develop` 지정 금지.
- rosdistro 등록 패키지는 `package.xml`의 `<depend>` + `rosdep`로 관리.
- 외부(third-party) 패키지 소스 **직접 수정 금지**. 필요하면 (1) 자체 패키지로 복사 후 수정, (2) 포크 후 의존성 지정, (3) 래퍼 작성 중 택일.
- 빌드 명령은 README/스크립트에 완전한 형태로 기록. 암묵적 빌드 단계 금지.

---

## Rule 7. 명시성·실패 가시성

- 노드 간 계약(토픽 이름, 메시지 필드, 단위, 프레임, QoS, 파라미터)은 **코드 또는 메시지 정의에서 드러나야** 함. 주석·구두 합의에만 의존 금지.
- **조용한 실패(silent failure) 방지**:
  - QoS 불일치 → 노드 초기화 시 구독 상태 검증
  - TF 변환 실패 → 명시적 타임아웃 + 에러 로그
  - 서비스 서버 미존재 → 타임아웃 후 명확한 에러
  - 필수 파라미터 누락 → 기본값 없이 선언해 초기화 실패 유도
- 공간 데이터 메시지는 **`frame_id` 항상 채움**(빈 문자열 발행 금지). REP-105 관례 준수(`base_link`, `odom`, `map`).
- `stamp`는 **측정 시점**(발행 시점 아님). `use_sim_time`은 전역 일관 적용(시뮬/실기 혼용 금지). 다중 센서 동기화는 `message_filters::TimeSynchronizer` 등 사용.

---

## Rule 8. 확장성·재사용성

- 설계 시 **동일 시스템 다중 인스턴스**를 전제. 고정 IP·고정 노드명·고정 토픽 경로 하드코딩 금지.
- 하드웨어 주소, 프레임 이름, 속도 제한, 타임아웃 등 환경 의존 값은 **노드 파라미터**로 노출, YAML로 관리, launch에서 주입.
- Launch 파일은 선언적 구성이 목표. 조건부 로직이 많아지면 상위/하위로 분리. 변형은 `DeclareLaunchArgument`로 제어.

---

## Rule 9. 안전(Safety) 강제 규정 — 최우선

- 비상정지·잠금·권한 해제 등 안전 신호는 **Topic 금지**. Service 또는 Action으로 구현하여 수신 확인·성공 여부를 보장.
- 안전 메시지 QoS 기본: **`RELIABLE` + `TRANSIENT_LOCAL`**, depth 충분히 확보.
- 통신 단절·타임아웃·예외 시 **페일세이프 상태로 수렴**(stop/neutral/hold 중 시스템에 맞게 명시적 정의).
- 웹 UI·외부 API 등 신뢰 경계 외부 명령을 ROS2 내부 안전 신호에 직결 금지. 인증·검증·rate limiting 레이어 필수.

---

## Rule 10. 메타 원칙 (충돌 해결)

- **안전 > 정확성 > 성능 > 편의성**. Rule 9는 모든 규칙에 우선.
- **생태계 호환성(Rule 1~5) > 프로젝트 편의**.
- **명시성 > 간결성**. 의도가 드러나는 쪽 선택, 단축 표기 지양.
- **생태계 관례 > 독자 컨벤션**. REP·ROS 2 Design Docs·공식 튜토리얼 우선. 독자 컨벤션은 이유 문서화.
- **좁게 시작 > 넓게 시작**. 네임스페이스·QoS·스키마는 변경 비용이 크므로 초기 보수적 선택.
- **실패 시 큰 소리로**. 조기에 명확히 실패시키는 설계 채택.

---

## 적용 방식

1. **코드 생성 시**: 해당 Rule 조건 검토, 위반 소지 있으면 생성 거부 또는 경고.
2. **코드 리뷰 시**: Rule 번호 인용(예: "Rule 1.2 위반: `Float64MultiArray`에 이종 데이터").
3. **애매한 경우**: Rule 10의 우선순위에 따라 결정.
4. **예외 적용 시**: 근거를 주석 또는 문서에 명시.

## 참고

- REP-103 (단위·좌표), REP-105 (프레임), ROS 2 Design Docs, QoS Concepts.
