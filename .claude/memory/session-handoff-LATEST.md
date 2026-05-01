# Session Handoff — LATEST

> 세션 간 상태 전달. 다음 세션이 처음 읽는 파일.
> 압축 직전 / 세션 종료 직전 업데이트. 매일 백업: `session-handoff-YYYY-MM-DD.md`.

## Last Updated

2026-05-01 — harness-init 완료 후 초기 핸드오프.

## Current Branch / Working State

- 작업 디렉터리: `/home/rokey/cobot2_chess_ai`
- Git 추적 여부: **현재 비-git** (CLAUDE.md 시스템 정보 기반). 추후 git 초기화 필요.
- 미커밋 파일: harness-init 산출물 (`.claude/`, `tasks/lessons.md`, `docs/harness-tests.md`).

## Active Task

- **방금 완료**: `/harness-init` 프로젝트 레벨 harness 생성.
  - `.claude/rules/ai-constitution.md` (chess_ai Tier 0)
  - `.claude/memory/MEMORY.md`, `session-handoff-LATEST.md`
  - `tasks/lessons.md`
  - `~/.claude/rules/ros2-principles.md` (글로벌화)
  - `~/.claude/rules/ai-constitution.md` 확장 (ROS2 참조 추가)

- **다음 작업**: `docs/DEVELOPMENT_ROADMAP.md` Phase 0 진입.
  - [ ] 0-1. `.env.example` → `src/cobot2/.env` 복사 + 실제 키 채움 (Firebase JSON 경로, 로봇 IP). **OpenAI key 제외 (Input Policy)**.
  - [ ] 0-2. pip 의존성 설치 (`ultralytics`, `stockfish`, `firebase-admin`)
  - [ ] 0-3. `colcon build --packages-select cobot2 cobot2_interfaces` 성공
  - [ ] 0-4. 4개 entry point 기동 import error 없이 확인

## Open Investigations

- 없음 (인계 직후, Phase 0 진입 전).

## Decisions This Session

1. **Input Policy 변경**: OpenAI/음성 배제, 텍스트 + Stockfish만 허용 (project ai-constitution.md III).
2. **CLAUDE.md 형태**: as-is 유지 + ai-constitution.md 동일 내용 동기화 (slim 안 함). 다중 AI 도구 대응.
3. **ros2-principles 글로벌화**: `~/.claude/rules/ros2-principles.md`. 모든 ROS2 프로젝트가 자동 참조.

## Files Modified This Session

- 신규: `~/.claude/rules/ros2-principles.md`
- 신규: `/home/rokey/cobot2_chess_ai/.claude/rules/ai-constitution.md`
- 신규: `/home/rokey/cobot2_chess_ai/.claude/memory/MEMORY.md`
- 신규: `/home/rokey/cobot2_chess_ai/.claude/memory/session-handoff-LATEST.md`
- 신규: `/home/rokey/cobot2_chess_ai/tasks/lessons.md`
- 신규 (예정): `/home/rokey/cobot2_chess_ai/docs/harness-tests.md` (Phase 4 위반 테스트 결과)
- 수정: `~/.claude/rules/ai-constitution.md` (ROS2 프로젝트 적용 섹션 추가)
- 수정 (예정): `/home/rokey/cobot2_chess_ai/CLAUDE.md` (ai-constitution 참조 라인 1줄 추가)

## Known Risks Carried Over

- Phase 0 시작 전: virtual 모드 미검증.
- 기존 OpenAI/음성 코드 존재 — Phase 1 매핑 시 표시, Phase 4 검토 대상.
- ROS2 설계 원칙 (`ros2-principles.md` Rule 1~9) 준수 여부 미점검.

## Resume Instruction

다음 세션 시작 시:
1. 본 핸드오프 파일 + `.claude/memory/MEMORY.md` 읽기
2. `tasks/lessons.md` 검토 (AI 행동 교정 규칙)
3. `docs/DEVELOPMENT_ROADMAP.md` Phase 0 진행 또는 사용자 지시 대기
4. **Hard Rules 재확인**: `.claude/rules/ai-constitution.md` + `~/.claude/rules/ros2-principles.md`
