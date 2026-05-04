# cobot2_chess_ai — Development Roadmap

**Stance**: 인계자 모드 (원작자 X). Hard Rules `no speculation in comments` /
`baseline before refactor` / `virtual mode first`를 모든 phase에 적용.

**Scope**: 1주일 (≈ 5 working days). Phase 0 → 4 순차 진행.
각 phase는 다음 phase 진입 전 exit criteria를 만족해야 한다.

---

## Phase 0: Environment Bootstrap (goal: 빌드·기동 검증, 반나절)
- [ ] 0-1. `.env.example` → `src/cobot2/.env` 복사 + 실제 키(Firebase JSON 경로,
       OpenAI API key, 로봇 IP) 채움. `.env` 자체는 절대 커밋 금지.
- [ ] 0-2. pip 의존성 설치 — `ultralytics`, `stockfish`, `firebase-admin`
       (`package.xml` 주석 참조).
- [ ] 0-3. `colcon build --packages-select cobot2 cobot2_interfaces` 에러 없이 성공.
- [ ] 0-4. `source install/setup.bash` 후 각 entry point가 import error 없이
       기동만 되는지 확인 (`ros2 run cobot2 main` 등).

**Exit**: 빌드 성공 + 4개 entry point 기동 가능.

---

## Phase 1: Code Mapping (goal: 시스템 구조 파악, 1-1.5일)
- [ ] 1-1. 노드 다이어그램 작성 — `cobot2/main.py`, `stockfish.py`, `robot_action.py`,
       `vision_db.py` 4개의 역할·실행 순서 매핑.
- [ ] 1-2. ROS2 토픽/액션/서비스 인벤토리 — `ros2 topic/service/action list`로
       실측. `cobot2_interfaces/{action,srv}` 정의와 대조.
- [ ] 1-3. 외부 의존성 매핑 — Firebase 클라이언트 호출 위치, OpenAI API 호출 위치,
       YOLO 모델 로드 위치(`models/`), Stockfish 바이너리 호출, 음성 인식
       (`run_voice.sh`) 진입점.
- [ ] 1-4. 설정 파일 인벤토리 — `config/*.json`, `cobot2/data.json` 각 키의 용도와
       참조 위치 (확인 안 된 키는 "verify needed" 표기).

**Exit**: docs/ARCHITECTURE.md 또는 동등한 메모에 ① 노드 그래프 ② 외부 의존성 맵
③ 설정 파일 사전 — 3종 정리 완료.

---

## Phase 2: Annotation & Documentation (goal: 검증된 동작에 한해 주석 추가, 1.5-2일)
- [ ] 2-1. 각 노드 파일 상단 module docstring — 책임, publish/subscribe 토픽,
       의존 외부 서비스. **확인 안 된 동작은 절대 단정하지 않음** (Hard Rule #1).
- [ ] 2-2. 주요 클래스/함수 docstring — 입력·출력·side effect만. 추측·예시 동작 금지.
- [ ] 2-3. README 업데이트 — 부트스트랩 절차(Phase 0), 실행 시퀀스, 환경 변수 표.
- [ ] 2-4. `# verify needed` 마커 수집 → Phase 3 검증 대상으로 인계.

**Exit**: 4개 entry point + 핵심 클래스 docstring 완료. `verify needed` 목록 작성.

---

## Phase 3: Virtual-Mode Verification (goal: 단독·통합 동작 확인, 1-1.5일)
- [ ] 3-1. 비전 노드 단독 — 카메라 입력에서 체스말 8x8 좌표 인식 결과 로그/시각화.
- ~~[ ] 3-2. 음성 인식 노드 단독 — 명령어 인식 정확도 sample 측정.~~ (voice stack 삭제 — 2026-05-04)
- [ ] 3-3. Stockfish 엔진 노드 단독 — 입력 FEN/move에 대한 응답 확인.
- [ ] 3-4. 로봇 동작 노드 단독 (virtual) — 좌표 입력 → 그리퍼 + 이동 시퀀스 완료.
- [ ] 3-5. 통합 시퀀스 (virtual) — 사람 차례 인식 → 엔진 응답 → 로봇 동작이
       최소 1회 에러 없이 완료. **이 단계 통과 없이 `mode:=real` 진입 금지** (Hard Rule #3).
- [ ] 3-6. baseline 기록 — 통합 시퀀스의 로그·터미널 출력·(가능하면) 영상을
       `outputs/baseline/`에 저장. Phase 4 회귀 비교 기준.
- [ ] 3-7. Phase 2의 `verify needed` 항목 결과 docstring에 반영.

**Exit**: 통합 virtual 시퀀스 1회 성공 + baseline 아티팩트 저장.

---

## Phase 4: Refactoring (goal: 식별된 개선점 적용, 잔여 시간)
우선순위: **안전성 > 가독성 > 구조 > 성능**.

- [ ] 4-1. 개선 후보 목록화 — Phase 1-3에서 발견한 이슈를 severity로 분류
       (CRITICAL: 하드코딩된 secrets / 안전 미체크 / `mode:=real` 무방비 호출 등).
- [ ] 4-2. CRITICAL 항목 우선 수정 — 변경 1건당 1 커밋, virtual 모드 회귀 확인.
- [ ] 4-3. 그 외 개선은 가독성·구조 위주. 동작 변경 시 baseline(3-6) 대조 의무.
- [ ] 4-4. 회귀 발생 시 즉시 롤백 (Hard Rule #2).
- [x] 4-voice. voice_command Topic → std_srvs/Trigger Service 전환 (2026-05-04). voice stack 삭제 (옵션 A). M1-1/M1-6/M1-7 RESOLVED.

**Exit**: CRITICAL 0건, baseline 회귀 0건.

---

## Backlog (unscheduled — 본 1주일 스코프 외)
- [ ] 실제 로봇(`mode:=real`) 데모 — 별도 사이클로 분리 (Phase 0-3 모두 통과 후).
- [ ] pytest 테스트 스위트 구축.
- [ ] CI(colcon build + lint) 설정.
- [ ] v2 확장 (체스 외 도메인 적용, 추가 음성 명령 등).
- [ ] 다른 팀 멤버 코드 의도 인터뷰(가능 시) — `verify needed`를 사실로 승격.
