# tasks/lessons.md — AI 행동 교정 규칙

> 반복 실수 발생 시 여기에 기록 → 다음 세션 시작 시 SessionStart hook이 자동으로 읽어 리뷰.
> 규칙 형식: `날짜 — 짧은 제목` + 문제 / 교정 / 근거.

## Format

```
### YYYY-MM-DD — <짧은 제목>
**문제**: <어떤 행동이 잘못되었나>
**교정**: <앞으로 어떻게 할 것인가>
**근거**: <왜 이 규칙인가, 어느 Tier 0 / 원칙과 연결되나>
```

## Active Lessons

### 2026-05-01 — `.env` 파일 Read 시 시크릿 transcript 노출

**문제**: Edit 도구는 사전 Read 필수. `.env` 파일을 그대로 Read 호출하여 OpenAI API key 전체(`sk-proj-...`)가 transcript에 평문으로 박힘. transcript는 Anthropic 측 로그에 보관될 수 있어 비가역 노출.

**교정**:
1. `.env` 등 시크릿 가능성 있는 파일을 Read 하기 전, **반드시 `sed 's/=.*/=<REDACTED>/'` 마스킹된 출력으로 먼저 확인**
2. 키 존재/위치 확인이 목적이면 `grep -E '^[A-Z_]+=' file | sed 's/=.*/=<value>/'` 형태 사용
3. Edit 도구가 사전 Read를 강제하지만, 불가피하게 Read 후 노출되면 **즉시 사용자에게 키 회전 권장**
4. .env Write 시에는 동일 파일을 Read 해야 하므로 — Edit 대신 Write 사용 시 사전 Read 우회 고려

**근거**: 프로젝트 ai-constitution.md II.5 (no hardcoded secrets), 글로벌 ai-constitution VI (memory discipline), execution discipline III.3 (불확실하면 말한다 — 추측 전에 마스킹).

---

### 2026-05-01 — 사용자가 chat에 sudo 비밀번호 직접 입력 → 영구 노출

**문제**: 비대화형 sudo 실행이 막혔을 때 사용자가 chat에 평문 비밀번호를 입력함. transcript에 평문 박힘 → 비가역 노출. AI 측에서 비번 입력 방식을 사전 차단하지 않음.

**교정**:
1. sudo 명령이 필요할 때 **첫 시도부터 다음 옵션을 명시 권장**:
   - **A**: 사용자 터미널 직접 실행 (chat 노출 0)
   - **B**: 본 세션 `! sudo ...` 명령 (사용자 터미널 비번 입력, output만 chat 들어옴)
   - **금지**: chat에 비번 직접 입력
2. 사용자가 비번을 chat에 입력하면 **즉시 비번 회전 권장 + 향후 옵션 안내**
3. sudo 명령 실행 전 NOPASSWD 사전 설정 가능성 안내 (해당 명령만 한정 권장)

**근거**: 프로젝트 ai-constitution.md II.5 (시크릿 정책), 보안 사고 사전 차단 책임은 AI 측에 있음. 사용자가 안전한 옵션을 모를 수 있다는 가정.

---

## Archived (해결됨 / 더 이상 적용 불필요)

(없음)
