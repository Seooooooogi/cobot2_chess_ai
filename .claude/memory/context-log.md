# Context Log — cobot2_chess_ai

> 시간 순 episode 로그. TTL 만료 시 정리. `[ref:N]` 카운트 ≥ 3 → MEMORY.md 승격.
> 형식: `[YYYY-MM-DD] [TYPE] [ttl:Nd|permanent] [ref:0] description`

## Episodes

[2026-05-01] [DECISION] [ttl:permanent] [ref:0] Input Policy 확정: 텍스트 + Stockfish만. OpenAI/음성 신규 금지 (project ai-constitution III).

[2026-05-01] [DECISION] [ttl:permanent] [ref:0] Firebase → 로컬 DB 마이그레이션 예정 (별도 cycle). 사용자 lean: 미정 (SQLite/JSON/기타).

[2026-05-01] [DECISION] [ttl:permanent] [ref:0] DR_init은 M0609_RG2_Integration의 bringup과 통합하는 방향 (구현 시점 미정).

[2026-05-01] [DECISION] [ttl:permanent] [ref:0] orchestrator 글로벌은 JIUM 도메인 유지, chess_ai는 프로젝트 로컬 오버라이드 (Light) 사용.

[2026-05-01] [COMPLETION] [ttl:90d] [ref:0] Phase 0 bootstrap 완료. T1-T11 PASS. code-reviewer SHIP IT, verification 10/10 PASS. git 첫 커밋 `4e7e53e`.

[2026-05-01] [COMPLETION] [ttl:90d] [ref:0] /team-init Standard tier 유지, 프로젝트 로컬 orchestrator (Light) 1개 신규 생성: `.claude/agents/orchestrator.md`.

[2026-05-01] [INCIDENT] [ttl:90d] [ref:0] OpenAI API key (`sk-proj-...`) 가 `.env` Read 시 transcript 노출. 사용자에게 키 즉시 revoke 권장.

[2026-05-01] [INCIDENT] [ttl:90d] [ref:0] sudo 비밀번호 `rokey1234` 가 chat에 평문 입력되어 transcript 영구 보관. 사용자에게 시스템 비번 회전 권장.

[2026-05-01] [DISCOVERY] [ttl:permanent] [ref:0] DR_init은 별도 워크스페이스 `/home/rokey/M0609_RG2_Integration/install/`에 빌드되어 있음. chess_ai 실행 전 source 필수.

[2026-05-01] [DISCOVERY] [ttl:permanent] [ref:0] pymodbus 3.x는 `from pymodbus.client.sync import ModbusTcpClient` 미지원. cobot2/onrobot.py는 2.x API 사용 → `pymodbus<3` 고정 필요.

[2026-05-01] [DISCOVERY] [ttl:permanent] [ref:0] uv venv는 기본적으로 pip 미포함. `which pip`이 system pip를 잡을 수 있어 venv 격리 깨짐. `uv pip install` 명시 사용 필수.

[2026-05-01] [DISCOVERY] [ttl:permanent] [ref:0] colcon build의 ament_python builder가 venv 활성 중에도 system python shebang을 생성하는 경우 있음. 재빌드마다 shebang sed 패치 필요 (영속 해결 Phase 1+ 미정).

[2026-05-01] [DISCOVERY] [ttl:permanent] [ref:0] `main.py:193`이 `voice_command` 토픽 subscribe → voice_control_node 미구동 시 무한 대기. 음성 노드 단순 삭제 시 main 정지.

[2026-05-01] [PLAN] [ttl:90d] [ref:0] Phase 1 첫 작업: `vision_db.py:30` Firebase 경로 하드코딩 → env 화 (code-reviewer IMPORTANT 1건 해소).

## Cleanup Log

(아직 정리된 episode 없음)
