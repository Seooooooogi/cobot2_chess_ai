# AI Constitution — cobot2_chess_ai

> 프로젝트 레벨 Tier 0. 전역 `~/.claude/rules/ai-constitution.md`를 **확장**(약화 아님)한다.
> 본 리포지토리는 협동2 A-2 팀의 **인계물**이며 현 작업자는 원작자가 아니다.
> 코드 이해 → 주석 → 동작 검증 → 리팩토링 순서를 지킨다.

## I. Inherited Rules (전역에서 자동 적용)

다음 전역 규칙이 본 프로젝트에도 동일하게 적용됨 (재서술 불필요):

- 전역 Tier 0 (`~/.claude/rules/ai-constitution.md`):
  - no fabrication, virtual mode first, no AI attribution in git
- 전역 ROS2 규칙 (`~/.claude/rules/ros2-principles.md`):
  - Rule 1~10. 특히 **Rule 9 (안전 Tier 0 동급)**: 비상정지/안전 신호 Topic 금지, 페일세이프 수렴, 웹 UI → ROS2 안전 신호 직결 금지.

## II. Project-Specific Tier 0 (chess_ai 전용 — never bend)

1. **no speculation in comments** — 인계받은 코드의 미검증 동작을 주석/docstring으로 단정 금지.
   확인되지 않은 부분은 `# verify needed` 또는 docstring에 "확인 필요" 명시.
   *Why*: 원작자가 아니므로 추측 주석이 향후 혼란·잘못된 리팩토링 유발.

2. **baseline before refactor** — 리팩토링 전 현재 동작을 기록(로그/스크린샷/스모크 시퀀스 → `outputs/baseline/`).
   변경 후 회귀 발견 시 즉시 롤백.
   *Why*: 검증되지 않은 동작을 무자각 상태로 깨뜨릴 위험 차단.

3. **virtual mode first** — 실제 M0609(`mode:=real`) 연결 전 virtual 모드에서
   동일 시퀀스가 에러 없이 완료되어야 한다. virtual 검증 없이 실제 하드웨어 동작 금지.
   *Why*: 하드웨어·인적 안전. 전역 규칙 강화.

4. **no fabrication (chess_ai scope)** — vision 추론 실패, Firebase 로그 결측은
   명시적으로 표기/마킹. 임의 보간/대체값 채우기 금지.
   *Why*: 게임 기록의 감사 추적성 + 비전 fallback이 잘못된 수를 둘 위험.

5. **no hardcoded secrets** — Firebase service account JSON, 두산 로봇 IP/credentials는
   환경변수/`.env`로만 로드. 리포 내 `.env`, `*-firebase-adminsdk-*.json` 커밋 금지.
   *Why*: 자격 증명 유출 방지. `setup.py`가 `share/cobot2/.env`를 데이터 파일로 등록하므로
   `.env` 파일 자체는 빌드에 필요하지만 그 내용물은 환경변수에서 주입.

6. **append-only Firebase logs** — 게임 기록·이벤트 로그 덮어쓰기 금지.
   *Why*: 게임 결과 분쟁 시 감사 추적, 학습/분석 데이터 무결성.

7. **no AI attribution in git** — commit message, PR description, AUTHORS 등 git 추적
   attribution에 Claude/Copilot/GPT 등 AI assistant를 `Co-Authored-By`, contributor, footer로 추가 금지.
   *Why*: 위반 시 git 히스토리에 비가역으로 남음. 전역 규칙 강화.

## III. Input Policy (Scope Lock)

본 프로젝트는 입력 채널을 단순화한다:

- **허용**: 텍스트 입력(체스 수, 명령 등) + Stockfish 엔진
- **금지**: OpenAI API/LLM 엔진 연동 신규 추가, 음성 인식(STT) 신규 추가
- **사유**: 외부 API 의존성 축소, 음성 인식 실패 fabrication 위험 차단, 인계 단계에서 표면적 최소화
- **기존 OpenAI/음성 코드 처리**: 인계자 작업 흐름의 ② 주석 단계에서 `# verify needed` 표기 + Phase 4에서 비활성화 또는 제거 검토. **신규 기능 추가 시 위 채널만 사용**.

## IV. Invalidation Conditions

- 사용자가 문서화된 이유와 함께 명시적 override 요청 시
- 전역 ai-constitution.md / ros2-principles.md와 충돌 시 → **전역이 우선 (project extends, never weakens)**

## V. Memory Discipline (전역 VI 적용)

- `MEMORY.md`, `session-handoff-LATEST.md`는 과거 시점 스냅샷. 행동 전 현재 상태 검증.
- 메모리에 기록된 파일 경로·함수명은 Glob/Grep으로 재확인 후 사용.
- 메모리와 현재 상태 충돌 시 → 현재 상태가 이긴다.

## VI. Cross-Reference

- 본 파일은 프로젝트 루트 `CLAUDE.md`의 Hard Rules와 **동일 내용**(드리프트 방지: 둘 중 하나가 변경되면 다른 하나도 동기화).
- ROS2 설계 원칙: `~/.claude/rules/ros2-principles.md` (Rule 9는 Tier 0 동급).
- 인계자 작업 흐름·검증 체크리스트: `CLAUDE.md` Dev Conventions + `~/.claude/rules/development-workflow.md`.
