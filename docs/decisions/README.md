# Architecture Decision Records

이 코드베이스에 영향을 준 결정을 기록한다. 다음 작업 시 항목 추가:
새 의존성 도입, 기존 패턴 교체, 데이터 모델 변경, 노드 구조 재구성, Hard Rules 변경.

## Template
```markdown
# ADR-NNN: [Decision Title]
## Context: 왜 이 결정이 필요한가
## Decision: 무엇을 선택했는가
## Consequences: 트레이드오프, 알려진 제약
```

## Decisions

### ADR-001: 인계자 모드 — Hard Rules 채택 및 작업 흐름 정의 (2026-04-30)

**Context**:
- 본 리포(`cobot2_chess_ai`)는 협동2 A-2 팀(박윤헌·김연빈·김하균·문규혁·서재우)
  프로젝트의 인계물. 원작자가 아닌 사용자가 코드 이해 → 주석 → 검증 → 리팩토링을 수행.
- 글로벌 `~/.claude/rules/ai-constitution.md`는 별도 프로젝트(M0609_RG2_Integration /
  LeRobot 데이터셋 빌드)용 규칙(예: `no data leakage`, `baseline required` for VLA model)을
  포함 — 본 프로젝트에 그대로 적용 시 mismatch.

**Decision**:
- **Stack** — Language: Python 3 / ROS2 ament_python, DB: Firebase, AI: YOLOv8/v11 +
  Stockfish + OpenAI API + 음성 인식, Interface: Web UI (HTML/CSS/Firebase) + ROS2 노드,
  Deployment: Local Ubuntu PC + Doosan M0609.
- **Hard Rules** — 글로벌 참조 대신 프로젝트 로컬 7개 항목 채택 (`CLAUDE.md` 참조).
  - 인계자 특화: `no speculation in comments`, `baseline before refactor`
  - 글로벌 공통: `virtual mode first`, `no fabrication`, `no hardcoded secrets`,
    `no AI attribution in git`
  - 프로젝트 특화: `append-only Firebase logs`
- **Scope** — 1주일 이내 작업 사이클. `docs/DEVELOPMENT_ROADMAP.md`로 인계자 흐름을
  Phase 0(부트스트랩) → 1(매핑) → 2(주석) → 3(virtual 검증) → 4(리팩토링) 단계화.

**Consequences**:
- 글로벌 `ai-constitution.md` 변경 시 자동 반영되지 않음 — 수동 동기화 필요.
- test suite 생략(Backlog로 이동) — 1주일 스코프 내 baseline 기록(Phase 3-6)이
  회귀 검증의 1차 수단. 본격적 pytest 도입은 차기 사이클.
- `baseline before refactor` 규칙으로 모든 리팩토링 전 동작 기록(로그/스모크 시퀀스)
  의무화. 기록 없는 변경은 회귀 검증 불가.
- `setup.py`가 `.env`를 share 데이터 파일로 등록하여, `.env` 부재 시 colcon build 실패.
  신규 환경에선 `.env.example` → `src/cobot2/.env` 복사 필수 (ROADMAP Phase 0-1).
