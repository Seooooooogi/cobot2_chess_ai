#!/usr/bin/env bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# ros2 run으로 실행되면 SCRIPT_DIR이 install/.../lib/cobot2 가 될 수 있음.
# 그래서 워크스페이스 루트를 2단계로 탐색:
# 1) src에서 직접 실행하는 경우: SCRIPT_DIR=.../src/cobot2/cobot2 -> ../../.. = WS
# 2) install에서 실행하는 경우: SCRIPT_DIR=.../install/cobot2/lib/cobot2 -> ../../.. = install (WS 아님)
#    => install 기준으로는 ../.. 로 올라가도 WS 루트가 안 나올 수 있어, 다음 fallback로 home/cobot_ws 사용.
WS_DIR_CANDIDATE_1="$( realpath "$SCRIPT_DIR/../../.." 2>/dev/null || true )"

# 워크스페이스 판별: src/<pkg> 또는 install/setup.bash가 있는지로 체크
if [ -f "$WS_DIR_CANDIDATE_1/src/cobot2/setup.py" ] || [ -f "$WS_DIR_CANDIDATE_1/install/setup.bash" ]; then
  WS_DIR="$WS_DIR_CANDIDATE_1"
else
  # fallback: 현재 스크립트가 install 아래에서 실행되는 케이스
  # 일반적으로 ws는 ~/cobot_ws
  WS_DIR="$HOME/cobot_ws"
fi

VENV_PATH="$WS_DIR/src/cobot2/cobot2/venv_voice/bin/activate"
if [ ! -f "$VENV_PATH" ]; then
  echo "[ERROR] venv activate not found: $VENV_PATH"
  echo "        해결: 다음 중 하나를 확인"
  echo "        1) venv를 생성: python3 -m venv $WS_DIR/src/cobot2/cobot2/venv_voice"
  echo "        2) 필요한 패키지 설치: $WS_DIR/src/cobot2/cobot2/venv_voice/bin/pip install -r requirements.txt"
  exit 1
fi

# venv 활성화
source "$VENV_PATH"

python - <<'EOF'
import numpy
print("numpy version:", numpy.__version__)
EOF

trap 'deactivate; exit 0' INT TERM

# =========================================================
# .env 로드 (OPENAI_API_KEY export)
# - 우선순위: src/cobot2/.env -> ws 루트 .env
# =========================================================
ENV_FILE_1="$WS_DIR/src/cobot2/.env"
ENV_FILE_2="$WS_DIR/.env"

if [ -f "$ENV_FILE_1" ]; then
  set -a
  source "$ENV_FILE_1"
  set +a
  echo "[INFO] Loaded env from: $ENV_FILE_1"
elif [ -f "$ENV_FILE_2" ]; then
  set -a
  source "$ENV_FILE_2"
  set +a
  echo "[INFO] Loaded env from: $ENV_FILE_2"
else
  echo "[WARN] .env not found at:"
  echo "       - $ENV_FILE_1"
  echo "       - $ENV_FILE_2"
fi

if [ -z "${OPENAI_API_KEY:-}" ]; then
  echo "[ERROR] OPENAI_API_KEY is not set."
  echo "        해결:"
  echo "        1) $ENV_FILE_1 또는 $ENV_FILE_2 에 다음 라인 추가"
  echo '           OPENAI_API_KEY="sk-xxxx..."'
  echo "        2) 또는 실행 전에 export OPENAI_API_KEY=..."
  deactivate
  exit 1
fi
# =========================================================

# ROS 환경
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

# 패키지로 인식시키기 위해 PYTHONPATH에 src/cobot2 추가
export PYTHONPATH="$WS_DIR/src/cobot2:${PYTHONPATH}"

# 모듈 형태로 실행 (상대 import 정상 동작)
python -m cobot2.voice_control_node

deactivate
