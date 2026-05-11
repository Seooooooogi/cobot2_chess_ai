"""RobotActionServer — Doosan M0609 + RG2 모션 액션 서버 (entry point: ``ros2 run chess_ai robotaction``).

역할:
    체스 수 (예: ``"e2e4"``)와 현재 보드 dict이 담긴 ``MoveChessPiece`` action goal을 받아
    pick-and-place 시퀀스를 실행한다 (앙파상/캐슬링 분기 포함).

ROS2 Interfaces:
    Server: Action ``move_chess_piece`` (chess_ai_interfaces/MoveChessPiece)

    Auxiliary 노드 ``dsr_robot_node`` (namespace=``dsr01``)는 DR_init이 참조하는 글로벌 변수
    호스팅용으로 ``main()``에서 생성된다. **executor에 add하지 않으며** ``RobotActionServer``만
    spin됨 — # verify needed: bound 노드를 spin하지 않은 상태에서 DR_init이 정상 동작하는지.

하드웨어 & 모션 API:
    - DR_init / DSR_ROBOT2 Python wrapper, ``__dsr__id="dsr01"`` / ``__dsr__model="m0609"`` 바인딩.
    - 모션 호출은 DSR API 직접 호출 (``movej``, ``movel``, ``mwait``, ``wait``) — ROS2 service
      호출(``/dsr01/dsr_controller2``)이 **아님**. Phase 1-2 capture에서 ``/dsr01/servoj_stream`` 등의
      publisher가 0이었던 원인.
    - RG2 그리퍼는 OnRobot Modbus TCP ``192.168.1.1:502`` (하드코딩).
    - Tool: TCP ``GripperDA_v1_1``, weight ``Tool Weight``.

모드 선택:
    - ``ROBOT_MODE`` env var (default ``"virtual"``). DSR launch의 ``mode:=`` 인자와 반드시 일치해야 함 —
      불일치는 Rule 9 안전 리스크 (노드 시작 시 warn 출력).
    - ``virtual`` 모드: ``MovingChessPiece._init_gripper``가 Modbus connect를 skip.
    - ``real`` 모드: connect를 시도한 뒤 ``is_socket_open()``으로 fail-loud 검증
      (Rule 7) — pymodbus 2.x의 silent connect failure 회피.

외부 의존성:
    - DR_init / DSR_ROBOT2 (vendored doosan-robot2)
    - ``chess_ai.onrobot.RG`` (pymodbus 래퍼)
    - ``data.json`` (이 파일 옆에 위치) — 모션 파라미터 + 체스 보드 좌표
      (``JSON_PATH``, ``MovingChessPiece.load_initial_config``에서 로드)
    - ``chess_ai_interfaces.action.MoveChessPiece``

Issues (Phase 1-1 doc Node 3):
    - RESOLVED R1-1: module-level ``gripper = RG(...)`` 제거 (2026-05-01) —
      ``MovingChessPiece._init_gripper``로 이동 + ``ROBOT_MODE`` 분기 + ``is_socket_open()`` 가드.
    - ~~IMPORTANT R1-2: ``goal_callback`` 무조건 ACCEPT — 명령 검증 부재 → Rule 7.~~ **RESOLVED 2026-05-04**: ``_validate_goal``이 V1-V13 (UCI 형식, pieces_dict 무결성, from_pos 피스 존재·코드, 동시성) 구현. HARD REJECT 시 robot/gripper 미동작 보장 (Rule 9 모션 전 차단).
    - DEFERRED R1-3: ``TOOLCHARGER_IP/PORT`` 하드코딩 — 사용자 결정으로 유지 (2026-05-10):
      single-host + 고정 그리퍼 IP 시나리오, env화 비용 > 이득. Phase 6 다호스트 확장 시 재검토.
    - RESOLVED R1-4 (2026-05-10): 레이어드 페일세이프 (Rule 9 명시).
      L0 = teach pendant 하드웨어 E-stop (operator 주도, 모터 차단, 상시 사용 가능).
      L1 = software failsafe — Modbus 단절 검출 → DR_init ``set_safety_mode(RECOVERY, STOP)``
      (Option 1 STOP + Option 3 HOLD fallback: 모션 halt + 서보 유지). 트리거:
      grip/release pymodbus exception, status polling timeout (5.0s), pre-flight ping fail (D).
      복구: ``~/reset`` (std_srvs/Trigger) — operator 수동 re-init (B1).
      Virtual fault injection (V2): ``GRIPPER_FAULT_MODE`` env로 ``MockGripper`` 경유.
    - ~~IMPORTANT R1-5: action server QoS 미명시 → Rule 4.~~ **RESOLVED 2026-05-04**: 5종 QoS 명시 (goal/result/cancel/feedback = ``qos_profile_services_default``, status = ``qos_profile_action_status_default``).
    # verify needed (Phase 1-1): ``dsr_robot_node``가 executor에 add되지 않음 — bound 노드를
        spin하지 않아도 DR_init이 정상 동작하는지 확인 필요.
    # verify needed R1-7: ``data.json`` 한글 키 — virtual 모드에선 좌표 정확성 미검증.
    - R1-8 RESOLVED 2026-05-01: 미사용 ``feedback_msg`` (``MoveChessPiece.Feedback()``) ``execute_callback``에서 제거.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_services_default, qos_profile_action_status_default

from std_srvs.srv import Trigger

import DR_init
import time
from .onrobot import RG

import json
import os
import threading

from datetime import datetime

from chess_ai_interfaces.action import MoveChessPiece # 커스텀 액션 임포트


class FailsafeError(RuntimeError):
    """R1-4: L1 failsafe (Modbus 단절)이 트리거되었을 때 발생하는 예외.

    호출자(``execute_callback``)는 action server를 degraded로 표시하고 ABORT해야 한다.
    일반 예외와 구분되므로, 페일세이프 대상이 아닌 에러로는 degraded 모드가 트리거되지 않는다.
    """


# R1-4: bounded safety-mode 호출 timeout (초). DSR_ROBOT2.set_safety_mode 내부의
# wait_for_service는 timeout이 없어 — M0609 controller가 unreachable이면 영구 hang
# (Rule 7 silent-fail). Daemon thread + join(timeout)으로 ``_enter_failsafe`` /
# ``restore_safety_mode``가 bounded 시간 내에 반드시 반환되도록 한다.
# Nominal ROS2 service 왕복은 sub-100ms; 2.0s는 정상 fault response를 지연시키지 않으면서
# deadlock을 잡아낸다.
_SAFETY_CALL_TIMEOUT_SEC = 2.0


class MockGripper:
    """R1-4 V2: failsafe 테스트용 fault-injectable virtual 그리퍼.

    ``ROBOT_MODE == "virtual"`` AND ``GRIPPER_FAULT_MODE`` env가 설정되어 있을 때 활성화.
    ``MovingChessPiece``가 호출하는 ``onrobot.RG``의 일부 API를 노출.

    Fault modes (env value):
        - ``ping_fail``           — ``is_socket_open()``이 False 반환 (pre-flight ping 실패)
        - ``disconnect_on_grip``  — ``close_gripper()``에서 ConnectionException 발생
        - ``disconnect_on_release`` — ``open_gripper()``에서 ConnectionException 발생
        - ``disconnect_on_status`` — ``get_status()``에서 ConnectionException 발생
        - ``hang_on_status``      — ``get_status()``가 항상 busy=True 반환 (timeout 테스트)
        - 기타 / 미설정          — no-op virtual 그리퍼처럼 동작 (legacy ``None``과 동등)
    """

    def __init__(self, fault_mode: str, log_fn):
        self.fault_mode = fault_mode
        self._log = log_fn
        self._log(f"[MOCK] MockGripper instantiated (fault_mode={fault_mode!r})")

    def _maybe_raise(self, trigger: str):
        if self.fault_mode != trigger:
            return
        try:
            from pymodbus.exceptions import ConnectionException
            raise ConnectionException(f"MockGripper fault: {trigger}")
        except ImportError:
            raise ConnectionError(f"MockGripper fault: {trigger}")

    def is_socket_open(self) -> bool:
        return self.fault_mode != "ping_fail"

    def close_gripper(self):
        self._log("[MOCK] close_gripper")
        self._maybe_raise("disconnect_on_grip")

    def open_gripper(self):
        self._log("[MOCK] open_gripper")
        self._maybe_raise("disconnect_on_release")

    def get_status(self):
        self._maybe_raise("disconnect_on_status")
        if self.fault_mode == "hang_on_status":
            return [True]
        return [False]

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1_1"

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
JSON_PATH = os.path.join(BASE_DIR, "data.json")

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

# CLAUDE.md Tier 0: virtual mode first. 안전을 위해 default는 virtual —
# ROBOT_MODE 설정을 깜빡해도 하드웨어가 silently 활성화되지 않도록.
ROBOT_MODE = os.getenv("ROBOT_MODE", "virtual")

# R1-2 goal_callback 검증용 (Rule 7+9 defense-in-depth).
# Source: stockfish.py piece_match dict (12 entries).
VALID_PIECES = frozenset({"WP", "WR", "WN", "WB", "WQ", "WK",
                          "BP", "BR", "BN", "BB", "BQ", "BK"})
VALID_FILES = frozenset("ABCDEFGH")
VALID_RANKS = frozenset("12345678")
VALID_PROMOTIONS = frozenset("qrbn")


class MovingChessPiece:
    """모션 로직 owner — config 로드 + 보드 좌표 사전 계산 + pick-and-place 실행.

    Construction 순서:
        1. ``log()``용 ``logger_node`` 저장.
        2. 모션 파라미터(vel/acc/time/wait) + 기준 pose 기본값 설정.
        3. ``_init_gripper()`` — virtual 모드는 None, real 모드는 RG2 Modbus 클라이언트.
        4. ``load_initial_config()`` — ``data.json`` (한글 키)로 기본값 override.
        5. ``calculate()`` — ``posx_A1`` + interval 상수로부터 64개 칸 전체에 대한
           ``posx_board_list`` / ``posx_over_list`` / ``posx_under_list`` 사전 계산.

    Side Effects:
        - real 모드: ``_init_gripper``가 ``192.168.1.1:502``로 Modbus TCP 소켓을 연다.
        - virtual 모드: 하드웨어 접촉 없음.
    """

    def __init__(self, logger_node: Node, grip_status_timeout_sec: float = 5.0):
        """R1-4 / Rule 8: ``grip_status_timeout_sec``는 호출자의 ROS2 parameter
        (RobotActionServer ``grip_status_timeout_sec``)로부터 주입받는다.
        action server 외부에서 인스턴스화될 경우 (tests / scripts) default 5.0s 적용.

        RG2 close/open은 nominal < 1s 완료; 5.0s는 정상 모션을 지연시키지 않으면서
        Modbus stall을 잡아낸다.
        """
        self.logger_node = logger_node
        self._grip_status_timeout_sec = grip_status_timeout_sec

        self.vel = 60
        self.acc = 60
        self.time = 2
        self.mwait_time = 2
        self.wait_time = 1
        self.basic_posj = [0, 0, 45, 0, 135, -90]

        self.posx_A1 = [244.44, 176.01, 27.62, 75.24, -180, -55.21]
        self.posj_tomb = [0, 0, -90, 0, -90, 0]
        self.posj_tomb_over = [0, 0, 0, 0, 0, 0]

        self.poscharx_interval = 0.082857143
        self.poschary_interval = 50.648571429
        self.posnumx_interval = 50.858571429
        self.posnumy_interval = 0.3
        self.z_posx_interval = 150

        self.gripper = self._init_gripper()

        self.load_initial_config()
        self.calculate()

    def _init_gripper(self):
        """``ROBOT_MODE``에 따라 RG2 그리퍼를 초기화한다.

        Returns:
            real 모드 → ``RG``. virtual 모드 + ``GRIPPER_FAULT_MODE`` env 설정 →
            ``MockGripper`` (R1-4 V2). 그 외 virtual 모드 → ``None`` (legacy).

        Raises:
            RuntimeError — real 모드에서 ``rg.client.is_socket_open()``이 False인 경우
            (pymodbus 2.x는 connect failure를 silently 삼키므로 Rule 7에 따라 fail-loud).

        Side Effects:
            real 모드만: ``TOOLCHARGER_IP:TOOLCHARGER_PORT`` (``192.168.1.1:502``)로
            Modbus TCP 소켓을 연다.
        """
        if ROBOT_MODE == "virtual":
            fault_mode = os.getenv("GRIPPER_FAULT_MODE", "")
            if fault_mode:
                self.log(f"[VIRTUAL] Routing to MockGripper (GRIPPER_FAULT_MODE={fault_mode})")
                return MockGripper(fault_mode, self.log)
            self.log("[VIRTUAL] Skipping RG2 Modbus connect")
            return None

        self.log(
            f"[REAL] Connecting to RG2 gripper at "
            f"{TOOLCHARGER_IP}:{TOOLCHARGER_PORT}"
        )
        rg = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
        # ROS2 Rule 7 (fail-loud): pymodbus 2.x는 connect failure를 silently
        # 삼키므로 소켓이 실제로 열렸는지 명시 검증.
        if not rg.client.is_socket_open():
            raise RuntimeError(
                f"RG2 gripper Modbus connect failed: "
                f"{TOOLCHARGER_IP}:{TOOLCHARGER_PORT}"
            )
        self.log(f"[REAL] RG2 gripper connected")
        return rg

    def check_modbus_alive(self) -> bool:
        """R1-4 D: pre-flight 그리퍼 liveness 체크 (``goal_callback``에서 호출).

        # verify needed: ``is_socket_open()``은 네트워크 I/O 없이 pymodbus 연결 상태만
        읽음 — non-blocking 기대. pymodbus 버전 drift로 blocking 동작이 들어가면
        ``goal_callback``도 blocking이 되어 Rule 2 위반. Phase 6 검증 시 timing 측정.

        Returns:
            virtual 모드 (Modbus 없음) 또는 소켓 열림 → True.
            단절 / 예외 / MockGripper ``ping_fail`` 모드 → False.
        """
        if self.gripper is None:
            return True
        try:
            return bool(self.gripper.is_socket_open())
        except Exception as e:
            self.log(f"[FAILSAFE] Modbus liveness check exception: {e}")
            return False

    def reconnect_gripper(self) -> tuple[bool, str]:
        """R1-4 B1: L1 failsafe 복구를 위해 ``~/reset`` Service에서 호출.

        virtual 모드: 재인스턴스화 (MockGripper 상태 clear). real 모드:
        ``_init_gripper``로 Modbus 소켓 재open. degraded flag 해제 여부는
        반환 tuple로 호출자가 결정한다.

        Side effect: 재초기화 전에 ``self.gripper``를 None으로 clear — 다음 실패 시
        stale 깨진 인스턴스가 아닌 known-empty 상태가 남도록.
        """
        # MINOR fix: stale 참조를 먼저 정리.
        self.gripper = None
        try:
            self.gripper = self._init_gripper()
            if ROBOT_MODE == "virtual":
                return True, "virtual gripper reset"
            return True, f"gripper reconnected at {TOOLCHARGER_IP}:{TOOLCHARGER_PORT}"
        except Exception as e:
            return False, f"reconnect failed: {type(e).__name__}: {e}"

    def _call_safety_mode_bounded(self, safety_mode: int, safety_event: int, label: str) -> bool:
        """R1-4 헬퍼: hard timeout으로 DR_init ``set_safety_mode``를 호출.

        Vendored ``DSR_ROBOT2.set_safety_mode``에는 timeout 없는 ``wait_for_service``
        루프가 들어 있다. Daemon thread + ``join(timeout)``으로 controller
        reachability와 무관하게 호출자가 ``_SAFETY_CALL_TIMEOUT_SEC`` 내에 반드시
        반환되도록 보장 — Rule 7 fail-loud 의미를 유지.

        Returns: 호출 완료 → True. timeout / 예외 → False (로그 기록).

        # verify needed: real M0609에서의 실제 왕복 latency. virtual DRCF emulator는
        set_safety_mode를 honor하지 않을 수도 있음 (Phase 6 실기 검증 항목).
        """
        outcome = {"completed": False, "error": None}

        def _worker():
            try:
                from DSR_ROBOT2 import set_safety_mode
                set_safety_mode(safety_mode, safety_event)
                outcome["completed"] = True
            except Exception as e:
                outcome["error"] = e

        t = threading.Thread(target=_worker, daemon=True)
        t.start()
        t.join(timeout=_SAFETY_CALL_TIMEOUT_SEC)

        if t.is_alive():
            self.log(f"[FAILSAFE] set_safety_mode({label}) TIMEOUT after "
                     f"{_SAFETY_CALL_TIMEOUT_SEC}s — controller unreachable. "
                     f"Hardware E-stop (L0) remains the operator's recourse.")
            return False
        if outcome["error"] is not None:
            self.log(f"[FAILSAFE] set_safety_mode({label}) EXCEPTION: "
                     f"{type(outcome['error']).__name__}: {outcome['error']}")
            return False
        return True

    def _enter_failsafe(self, reason: str):
        """R1-4 L1: DR_init ``set_safety_mode``로 모션 STOP + 서보 HOLD.

        L0 (teach pendant 하드웨어 E-stop)는 operator 주도로 이 path와 독립 —
        본 메서드는 software graceful-degradation 레이어이며 catastrophic-safety
        레이어가 아니다 (Rule 9 명시).

        복구: 물리적 보드 점검 → ``~/reset`` Service.

        Notes:
            ``set_safety_mode(SAFETY_MODE_RECOVERY=2, SAFETY_MODE_EVENT_STOP=2)`` —
            ``DSR_ROBOT2.py`` line 2016 / ``DRFC.py`` line 127-141 참조.
            M0609 controller도 unreachable일 때 indefinite hang을 방지하기 위해
            ``_call_safety_mode_bounded``로 bounded 호출.
        """
        self.log(f"[FAILSAFE] L1 entry — {reason}")
        try:
            from DRFC import SAFETY_MODE_RECOVERY, SAFETY_MODE_EVENT_STOP
            self._call_safety_mode_bounded(
                SAFETY_MODE_RECOVERY, SAFETY_MODE_EVENT_STOP, "RECOVERY+STOP"
            )
        except ImportError as e:
            self.log(f"[FAILSAFE] DRFC import failed: {e} — safety mode call skipped")
        self.log("[FAILSAFE] Motion stop requested, servo retained. "
                 "Recovery: hardware E-stop (L0) if unsafe, else ~/reset Service after board check.")

    def restore_safety_mode(self) -> tuple[bool, str]:
        """R1-4 B1: reset 시점에 safety mode를 RECOVERY → AUTONOMOUS로 전환 시도.

        # verify needed: M0609는 software AUTONOMOUS 진입이 성공하기 전에 teach pendant
        수동 확인 (E-stop reset / mode key)을 요구할 수 있음. software 호출만으로는
        충분하지 않을 수 있으며, operator가 pendant에서 safety 상태를 해제한 후
        ``~/reset``을 호출해야 할 수도 있다. Phase 6 실기 검증 항목.

        Returns: (called_ok, message). False라고 무조건 unsafe는 아님 —
        operator pendant interaction이 먼저 필요할 수도 있음.
        """
        try:
            from DRFC import SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_ENTER
        except ImportError as e:
            return False, f"DRFC import failed: {e}"
        ok = self._call_safety_mode_bounded(
            SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_ENTER, "AUTONOMOUS+ENTER"
        )
        if ok:
            return True, "safety mode restored to AUTONOMOUS"
        return False, ("safety mode restore TIMEOUT or FAILED — "
                       "verify on teach pendant (manual reset may be required)")

    def log(self, msg: str):
        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        full = f"[{now}] {msg}"
        self.logger_node.get_logger().info(full)
    
    def load_initial_config(self):
        """``data.json`` (모듈 옆 위치)으로 모션 기본값을 override한다.

        파일의 14개 키 전부를 읽는다 — 모션 한글 키 + cell-interval 키 (Phase 4 2026-05-01):
            ``속도``, ``가속도``, ``시간``, ``mwait_시간``, ``wait_시간``, ``홈_관절좌표``,
            ``A1_좌표``, ``무덤_관절좌표``, ``무덤_관절좌표_오버``, ``z축_간격``,
            ``posnumx_interval``, ``poschary_interval``, ``posnumy_interval``, ``poscharx_interval``.

        Side Effects:
            인스턴스 속성을 in-place 갱신. 파일 부재 / 파싱 실패 시 생성자 default 유지.

        Notes:
            # verify needed R1-7: data.json 한글 키 좌표는 virtual 모드에서 미검증.
        """
        if not os.path.exists(JSON_PATH):
            return
        try:
            with open(JSON_PATH, "r") as f:
                data = json.load(f)

            self.vel = data.get("속도", self.vel)
            self.acc = data.get("가속도", self.acc)
            self.time = data.get("시간", self.time)
            self.mwait_time = data.get("mwait_시간", self.mwait_time)
            self.wait_time = data.get("wait_시간", self.wait_time)
            self.basic_posj = data.get("홈_관절좌표", self.basic_posj)

            self.posx_A1 = data.get("A1_좌표", self.posx_A1)
            self.posj_tomb = data.get("무덤_관절좌표", self.posj_tomb)
            self.posj_tomb_over = data.get("무덤_관절좌표_오버", self.posj_tomb_over)
            self.z_posx_interval = data.get("z축_간격", self.z_posx_interval)
            self.posnumx_interval = data.get("posnumx_interval", self.posnumx_interval)
            self.poschary_interval = data.get("poschary_interval", self.poschary_interval)
            self.posnumy_interval = data.get("posnumy_interval", self.posnumy_interval)
            self.poscharx_interval = data.get("poscharx_interval", self.poscharx_interval)

            self.log("JSON sync done")
        except Exception as e:
            self.log(f"JSON load error: {e}")

    def grip(self): # 10mm
        if self.gripper is None:
            self.log("[VIRTUAL] grip (no-op)")
            return
        try:
            self.gripper.close_gripper()
            self._wait_gripper_idle("grip")
        except Exception as e:
            self._enter_failsafe(f"grip(): {type(e).__name__}: {e}")
            raise FailsafeError(f"grip(): {e}") from e

    # (0,1):50mm

    def release(self): # 35mm
        if self.gripper is None:
            self.log("[VIRTUAL] release (no-op)")
            return
        try:
            self.gripper.open_gripper()
            self._wait_gripper_idle("release")
        except Exception as e:
            self._enter_failsafe(f"release(): {type(e).__name__}: {e}")
            raise FailsafeError(f"release(): {e}") from e

    def _wait_gripper_idle(self, op_label: str):
        """R1-4: parameter 기반 timeout으로 ``get_status()[0]``이 idle이 될 때까지 polling.

        원래 루프엔 timeout이 없어 — pymodbus 2.x는 degraded 연결에서 hang이나
        stale truthy 값을 반환할 수 있다. Timeout으로 indefinite hang을
        ``TimeoutError``로 전환 — 호출자의 ``_enter_failsafe`` 경로가 작동하도록.

        # verify needed (MINOR): deadline은 ``get_status()`` 호출 사이에 검사. pymodbus
        ``get_status()`` 자체가 half-open 소켓에서 hang하면 deadline에 도달하지 못함.
        pymodbus는 default socket-level timeout이 있으나 ``onrobot.RG``에서 명시 설정
        되어 있지 않음. Real-mode 동작은 Phase 6 검증 대기; 필요 시 명시적
        ``client.set_timeout`` 검토 (R1-3 onrobot 래퍼 영역).

        Raises:
            TimeoutError — polling이 ``_grip_status_timeout_sec``를 초과.
            Exception — ``get_status()``의 pymodbus / socket 에러 (호출자가 wrap).
        """
        deadline = time.monotonic() + self._grip_status_timeout_sec
        while self.gripper.get_status()[0]:
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"{op_label} status polling exceeded {self._grip_status_timeout_sec}s"
                )
            time.sleep(0.25)

    def calculate(self):
        """64개 체스 칸 전체에 대한 로봇 TCP 좌표를 미리 계산해 딕셔너리에 저장.

        좌표 체계 (data.json 기반):
            posx_A1   : A1 칸 위치 [x, y, z, rx, ry, rz] (mm, deg)
            posnumx_interval : 숫자(1→8) 방향 x 증분 — 각 숫자 행 사이의 거리
            posnumy_interval : 숫자 방향 y 미세 보정 — 체스판이 완전 수직이 아닌 경우 보정
            poscharx_interval: 문자(A→H) 방향 x 미세 보정
            poschary_interval: 문자 방향 y 증분 — 각 문자 열 사이의 거리
            z_posx_interval  : pick/place 시 접근 높이 오프셋 (over 위치)

        생성하는 세 딕셔너리:
            posx_board_list : 각 칸의 보드 레벨 좌표 (말이 놓이는 높이)
            posx_over_list  : 보드 레벨 + z_posx_interval (수직 접근 전 안전 높이)
            posx_under_list : 보드 레벨 + 3 mm (pick 시 살짝 눌러잡는 낮은 위치)

        # verify needed R1-7: data.json Korean-keyed 좌표 실측 검증 미완료.
        """
        self.posx_board_list = {}
        self.posx_over_list = {}
        self.posx_under_list = {}
        characters = ('A', 'B', 'C', 'D', 'E', 'F', 'G', 'H')
        for c in range(8):   # 열 (A~H): c=0은 A, c=7은 H
            for j in range(8):  # 행 (1~8): j=0은 1, j=7은 8
                # 보드 레벨 좌표: A1 기준으로 열/행 증분을 더한다.
                # x 방향: 숫자(행) 증분 + 문자(열) x 미세 보정
                # y 방향: 문자(열) y 증분 + 미세 보정
                # z 및 rx,ry,rz: A1과 동일 (보드 평면 가정)
                posx_a = [self.posx_A1[0]+self.posnumx_interval*j+self.posnumy_interval*j, self.posx_A1[1]+self.poscharx_interval*c-self.poschary_interval*c, self.posx_A1[2]]
                posx_a.extend(self.posx_A1[3:6])  # rx, ry, rz 복사
                self.posx_board_list[f"{characters[c]}{j+1}"] = posx_a

                # over: 수직 접근 전 안전 높이 (z만 증가)
                posx_over = posx_a.copy()
                posx_over[2] += self.z_posx_interval
                self.posx_over_list[f"{characters[c]}{j+1}"] = posx_over

                # under: 살짝 눌러잡는 낮은 위치 (z + 3 mm)
                # # verify needed: 3 mm 고정값의 적정성 (기물 높이에 따라 달라질 수 있음)
                posx_under = posx_a.copy()
                posx_under[2] += 3
                self.posx_under_list[f"{characters[c]}{j+1}"] = posx_under
                
    def perform_task(self, goal_handle):
        """한 체스 수에 대한 pick-and-place 시퀀스를 실행한다.

        Args:
            goal_handle: rclpy ActionServer goal handle. ``goal_handle.request``는
                ``command`` (UCI move, 예: ``"e2e4"``)와 ``pieces_dict`` (JSON 직렬화된
                ``A1``..``H8`` → ``WP``/``BR``/... 매핑)을 운반.

        함수 내 분기:
            - 폰의 대각선 빈 칸 이동 (``piece_from[1] == 'P'`` + 열 변경 + target is None)
              → 앙파상: 잡힌 폰을 lift → ``posj_tomb``에 deposit → return.
            - 킹의 2칸 이동 (``piece_from[1] == 'K'`` + column delta == 2)
              → 캐슬링: 킹 이동 전에 해당 룩 먼저 옮김.
            - ``target is not None`` → pre-capture: 목적지 기물을 ``posj_tomb``으로 옮김.
            - 그 뒤 이동 기물을 ``from_pos`` → ``to_pos``로 이동.

        모션 API:
            ``DSR_ROBOT2``에서 ``movej``, ``movel``, ``mwait``, ``wait``를 직접 import.
            ROS2 service 호출이 **아님**.

        Side Effects:
            - DR_init / DSR_ROBOT2 경유로 M0609 controller에 모션 명령 발행.
            - RG2 그리퍼 open / close (virtual 모드는 no-op).

        Notes:
            R1-8 RESOLVED 2026-05-01: 미사용 ``Feedback`` 인스턴스 ``execute_callback``에서 제거.
        """
        command = goal_handle.request.command
        pieces_dict = json.loads(goal_handle.request.pieces_dict)
        from_pos = command[0:2].upper()  # 예: "E2"
        to_pos = command[2:4].upper()    # 예: "E4"

        piece_from = pieces_dict.get(from_pos)  # 이동할 기물 코드 (예: "WP")
        target = pieces_dict.get(to_pos)         # 목적지 기물 코드 (있으면 잡기)
        self.log(f"Moving piece: {piece_from} from {from_pos} to {to_pos}, target: {target}")

        self.log("Moving piece starts")
        # DSR_ROBOT2 모션 함수: function-level import (global import 시 g_node 사전 할당 필요)
        from DSR_ROBOT2 import movej, movel, mwait, wait

        # 홈 위치로 이동 + 그리퍼 열기 (이전 동작 상태 초기화)
        movej(self.basic_posj, vel=self.vel, acc=self.acc)
        mwait(self.mwait_time)
        self.release()

        # ── 분기 1: 앙파상 (en-passant) ──────────────────────────────
        # 폰이 대각선으로 이동하는데 목적지가 비어있으면 앙파상.
        # 잡히는 폰의 위치: to_pos의 열 + from_pos의 행 (예: E5 → E5에 없고 E4에 있던 폰)
        if piece_from[1] == "P":
            if from_pos[0] != to_pos[0] and target is None:
                en_passant = ''.join([to_pos[0], from_pos[1]])  # 잡히는 폰의 칸
                # 잡히는 폰 lift → tomb
                movel(self.posx_over_list[en_passant], time=self.time)
                mwait(self.mwait_time)
                movel(self.posx_board_list[en_passant], time=self.time)
                wait(self.wait_time)
                self.grip()
                movel(self.posx_over_list[en_passant], time=self.time)
                mwait(self.mwait_time)
                movej(self.basic_posj, vel=self.vel, acc=self.acc)
                mwait(self.mwait_time)
                movej(self.posj_tomb_over, vel=self.vel, acc=self.acc)
                mwait(self.mwait_time)
                movej(self.posj_tomb, vel=self.vel, acc=self.acc)
                wait(self.wait_time)
                self.release()
                movej(self.posj_tomb_over, vel=self.vel, acc=self.acc)
                mwait(self.mwait_time)
                movej(self.basic_posj, vel=self.vel, acc=self.acc)
                mwait(self.mwait_time)
                # 앙파상: 잡힌 기물 처리만 하고 폰 자체 이동은 아래 공통 섹션에서.
                # (early return 없이 fall-through → 공통 pick&place 섹션 실행)

        # ── 분기 2: 캐슬링 ────────────────────────────────────────────
        # 킹이 2칸 이동하면 캐슬링 — 대응하는 룩을 먼저 옮긴다.
        # 킹사이드(G 파일): 룩 H→F. 퀸사이드(C 파일): 룩 A→D.
        elif piece_from[1] == "K":
            if abs(ord(from_pos[0])-ord(to_pos[0])) == 2:
                if to_pos[0] == "G":
                    castling_from = "H" + from_pos[1]  # 킹사이드 룩
                    castling_to = "F" + from_pos[1]
                else:
                    castling_from = "A" + from_pos[1]  # 퀸사이드 룩
                    castling_to = "D" + from_pos[1]
                # 룩 이동 시퀀스: over → board(grip) → over → target_over → target_under(release)
                movel(self.posx_over_list[castling_from], time=self.time)
                mwait(self.mwait_time)
                movel(self.posx_board_list[castling_from], time=self.time)
                wait(self.wait_time)
                self.grip()
                movel(self.posx_over_list[castling_from], time=self.time)
                mwait(self.mwait_time)
                movel(self.posx_over_list[castling_to], time=self.time)
                mwait(self.mwait_time)
                movel(self.posx_under_list[castling_to], time=self.time)
                wait(self.wait_time)
                self.release()
                movel(self.posx_over_list[castling_to], time=self.time)
                mwait(self.mwait_time)
                # 룩 처리 완료 → 공통 섹션에서 킹 이동

        # ── 공통: 잡기(capture) 전처리 ───────────────────────────────
        # 목적지에 기물이 있으면 먼저 tomb(무덤)으로 옮긴다.
        if target is not None:
            movel(self.posx_over_list[to_pos], time=self.time)
            mwait(self.mwait_time)
            movel(self.posx_board_list[to_pos], time=self.time)
            wait(self.wait_time)
            self.grip()
            movel(self.posx_over_list[to_pos], time=self.time)
            mwait(self.mwait_time)
            movej(self.basic_posj, vel=self.vel, acc=self.acc)
            mwait(self.mwait_time)
            movej(self.posj_tomb_over, vel=self.vel, acc=self.acc)
            mwait(self.mwait_time)
            movej(self.posj_tomb, vel=self.vel, acc=self.acc)
            wait(self.wait_time)
            self.release()
            movej(self.posj_tomb_over, vel=self.vel, acc=self.acc)
            mwait(self.mwait_time)
            movej(self.basic_posj, vel=self.vel, acc=self.acc)
            mwait(self.mwait_time)

        # ── 공통: 실제 기물 이동 pick & place ─────────────────────────
        # from_pos 에서 pick → to_pos 에 place (under 위치로 내려 release)
        movel(self.posx_over_list[from_pos], time=self.time)
        mwait(self.mwait_time)
        movel(self.posx_board_list[from_pos], time=self.time)
        wait(self.wait_time)
        self.grip()
        movel(self.posx_over_list[from_pos], time=self.time)
        mwait(self.mwait_time)
        movel(self.posx_over_list[to_pos], time=self.time)
        mwait(self.mwait_time)
        movel(self.posx_under_list[to_pos], time=self.time)  # 살짝 눌러놓기
        wait(self.wait_time)
        self.release()
        movel(self.posx_over_list[to_pos], time=self.time)
        mwait(self.mwait_time)
        movej(self.basic_posj, vel=self.vel, acc=self.acc)
        mwait(self.mwait_time)
        self.log("Moving piece completed")


class RobotActionServer(Node):
    """``move_chess_piece`` action을 호스팅하는 ROS2 action server.

    Construction 순서:
        1. ``MovingChessPiece`` 인스턴스화 (``data.json`` 로드 + 64칸 좌표 테이블 사전 계산;
           real 모드에선 Modbus 연결도 open).
        2. ``goal_callback`` / ``cancel_callback`` / ``execute_callback``과 함께 ``ActionServer`` 생성.
        3. ``ROBOT_MODE``를 Rule 9 mismatch warn과 함께 로그.

    Notes:
        - ``goal_callback``은 ``_validate_goal`` 경유 검증 (R1-2 RESOLVED 2026-05-04).
        - ``execute_callback``은 ``self._execution_lock`` 보호 하에 ``self._is_executing``을
          toggle — V12 (``_validate_goal``의 concurrent-goal REJECT) 활성화.
        - ``main()``에서 생성되는 auxiliary 노드 ``dsr_robot_node``는 ``DR_init.__dsr__node``에
          바인딩되지만 **executor에 add되지 않음** — 이 노드 (``RobotActionServer``)만 spin됨.
          # verify needed (Phase 1-1).
    """

    def __init__(self):
        super().__init__('robot_action_server')

        # Rule 8: 하드웨어 의존 timing은 ROS2 parameter로 노출.
        self.declare_parameter('grip_status_timeout_sec', 5.0)
        grip_timeout = self.get_parameter('grip_status_timeout_sec').value

        # 로직 클래스 초기화
        self.chess_mover = MovingChessPiece(self, grip_status_timeout_sec=grip_timeout)

        # 액션 서버 설정 (Rule 4: QoS 명시 — rclpy 기본값과 동일하나 의도 명시)
        self._action_server = ActionServer(
            self,
            MoveChessPiece,
            'move_chess_piece',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            goal_service_qos_profile=qos_profile_services_default,
            result_service_qos_profile=qos_profile_services_default,
            cancel_service_qos_profile=qos_profile_services_default,
            feedback_pub_qos_profile=qos_profile_services_default,
            status_pub_qos_profile=qos_profile_action_status_default,
        )
        self.get_logger().info(
            f"Robot Action Server started for {ROBOT_ID} ({ROBOT_MODEL}, "
            f"ROBOT_MODE={ROBOT_MODE})"
        )
        self.get_logger().warn(
            "ROBOT_MODE must match DSR launch 'mode:=' arg "
            "(arm/gripper mode mismatch is a Rule 9 safety risk)"
        )

        # R1-2 V12: 동시 실행 추적. _is_executing 은 _execution_lock 으로 보호.
        self._execution_lock = threading.Lock()
        self._is_executing = False

        # R1-4 L1 failsafe 상태. ``_degraded == True``인 동안 후속 goal은 모두 REJECT
        # — ``~/reset`` Service가 호출될 때까지 유지.
        self._degraded_lock = threading.Lock()
        self._degraded = False
        self._degraded_reason = ""

        # R1-4 B1: 수동 복구 Service (Trigger). Resolved name: ``~/reset``
        # → ``/robot_action_server/reset`` (Rule 5 — 이 노드가 소유).
        self._reset_srv = self.create_service(
            Trigger,
            '~/reset',
            self._reset_callback,
            qos_profile=qos_profile_services_default,
        )

    def _validate_goal(self, goal_request) -> tuple[bool, str]:
        """ACCEPT 전 MoveChessPiece goal을 검증한다 (R1-2 / Rule 7+9).

        Returns: (is_valid, reason). reason은 ``V<N>:`` 형식으로 시작 — 설계 문서의
        rule 인덱스와 매칭. 호출자(``goal_callback``)가 prefix로 로그 severity 결정.
        """
        cmd = goal_request.command

        # V1: command 타입과 길이
        if not isinstance(cmd, str) or len(cmd) not in (4, 5):
            cmd_len = len(cmd) if isinstance(cmd, str) else "N/A"
            return False, f"V1: command must be str of length 4 or 5, got {type(cmd).__name__} len={cmd_len}"

        cmd_upper = cmd.upper()
        from_sq = cmd_upper[0:2]
        to_sq = cmd_upper[2:4]

        # V2-V3: from-square 형식
        if from_sq[0] not in VALID_FILES:
            return False, f"V2: from-file '{cmd[0]}' not in a-h"
        if from_sq[1] not in VALID_RANKS:
            return False, f"V3: from-rank '{cmd[1]}' not in 1-8"

        # V4-V5: to-square 형식
        if to_sq[0] not in VALID_FILES:
            return False, f"V4: to-file '{cmd[2]}' not in a-h"
        if to_sq[1] not in VALID_RANKS:
            return False, f"V5: to-rank '{cmd[3]}' not in 1-8"

        # V6: 프로모션 piece 문자 (길이 5일 때만 검사)
        if len(cmd) == 5 and cmd[4].lower() not in VALID_PROMOTIONS:
            return False, f"V6: promotion piece '{cmd[4]}' not in qrbn"

        # V7: from != to
        if from_sq == to_sq:
            return False, f"V7: from == to ({from_sq})"

        # V13: 5-char UCI HARD REJECT (OQ-1=A: execute_callback이 [0:4]만 소비,
        # 프로모션 미지원 → ACCEPT 시 board dict desync 위험).
        if len(cmd) == 5:
            return False, f"V13: promotion moves not supported by execute_callback (consumes [0:4] only)"

        # V8: pieces_dict JSON 파싱
        try:
            d = json.loads(goal_request.pieces_dict)
        except (json.JSONDecodeError, TypeError) as e:
            return False, f"V8: pieces_dict JSON parse failed: {e}"

        # V9: dict + 비어있지 않음
        if not isinstance(d, dict) or not d:
            return False, f"V9: pieces_dict must be non-empty dict, got {type(d).__name__}"

        # V10: from_sq에 기물 존재 (Rule 9: execute_callback의 homing 모션 전에 차단)
        piece = d.get(from_sq)
        if piece is None:
            return False, f"V10: source square {from_sq} is empty in pieces_dict"

        # V11: from-piece가 유효한 코드
        if piece not in VALID_PIECES:
            return False, f"V11: unknown piece code '{piece}' at {from_sq}"

        # V11 (extended, OQ-3=A): to-piece가 None이 아니면 그것도 유효해야 함
        target = d.get(to_sq)
        if target is not None and target not in VALID_PIECES:
            return False, f"V11: unknown piece code '{target}' at {to_sq}"

        # V12 (concurrent goal)는 의도적으로 ``_validate_goal`` 밖 ``goal_callback``으로 이동 —
        # check-then-act가 아닌 atomic claim-and-set이어야 함.

        return True, ""

    def goal_callback(self, goal_request):
        """액션 목표 수락 여부 결정 (R1-2 V1-V13 + R1-4 F1/F2 + V12 atomic claim).

        REJECT 시 robot/gripper 미동작 (Rule 9: 모션 전 차단).
        순서: F1 (degraded) → V1-V11+V13 → F2 (Modbus ping) → V12 (atomic claim).
        V12는 반드시 마지막 게이트여야 함 — ``_execution_lock`` 보호 하의 atomic
        claim-and-set이 동시 callback 2개가 모두 ``_is_executing=False``를 관찰해
        둘 다 ACCEPT 되는 TOCTOU race를 방지.
        ``execute_callback``이 ``finally`` 블록에서 slot을 clear.
        """
        log = self.get_logger()
        cmd_repr = repr(goal_request.command)

        # F1: degraded 모드 REJECT (B1 수동 reset 필요).
        with self._degraded_lock:
            if self._degraded:
                log.error(
                    f"Goal REJECTED [F1: degraded — {self._degraded_reason}]. "
                    f"Call ~/reset Service to recover. command={cmd_repr}"
                )
                return GoalResponse.REJECT

        # V1-V11, V13 (V12는 atomic 의미를 위해 아래로 이동).
        is_valid, reason = self._validate_goal(goal_request)
        if not is_valid:
            log.error(f"Goal REJECTED [{reason}]. command={cmd_repr}")
            return GoalResponse.REJECT

        # F2: pre-flight Modbus ping (D). 실패 시 L1 failsafe 자동 진입 — 다음 goal도
        # 수동 reset 전까지 F1에서 REJECT.
        if not self.chess_mover.check_modbus_alive():
            with self._degraded_lock:
                self._degraded = True
                self._degraded_reason = "F2: pre-flight Modbus ping failed"
            self.chess_mover._enter_failsafe("F2: pre-flight Modbus ping failed")
            log.error(
                f"Goal REJECTED [F2: pre-flight Modbus ping failed]. "
                f"Entered L1 failsafe. command={cmd_repr}"
            )
            return GoalResponse.REJECT

        # V12: atomic concurrency claim — ACCEPT 전 마지막 게이트.
        # ``_is_executing=True``를 여기서 (execute_callback이 아닌 곳에서) set —
        # 그래야 두 번째 concurrent goal_callback이 True를 관찰하고 REJECT.
        with self._execution_lock:
            if self._is_executing:
                log.warn(f"Goal REJECTED [V12: concurrent goal in progress]. command={cmd_repr}")
                return GoalResponse.REJECT
            self._is_executing = True

        log.info(f"Goal ACCEPTED: command={cmd_repr}")
        return GoalResponse.ACCEPT

    def _reset_callback(self, request, response):
        """R1-4 B1: L1 failsafe에서 수동 복구.

        Two-phase 복구:
            (1) 그리퍼 재연결 — Modbus 소켓 re-open / MockGripper 상태 clear.
            (2) safety mode 복원 — ``set_safety_mode(AUTONOMOUS, ENTER)``.
        다음 모션 goal 성공을 위해 둘 다 필요: phase (1)은 그리퍼 I/O 복원;
        phase (2)는 ``_enter_failsafe``가 남긴 M0609 controller의 RECOVERY 모드 해제.
        phase (2) 없이는 F1/F2/V-checks를 통과해도 다음 ``movel``이 controller에서 거부됨.

        # verify needed: M0609는 AUTONOMOUS 진입이 성공하기 전에 teach pendant 수동
        확인을 요구할 수 있음. phase (2)가 failure / timeout 보고 시 operator가
        pendant에서 safety 상태를 해제한 후 ``~/reset``을 재시도해야 한다.
        Phase 6 실기 검증 항목.

        호출 전에 operator가 보드 / 로봇을 물리적으로 점검할 것이 전제.
        """
        log = self.get_logger()
        with self._degraded_lock:
            if not self._degraded:
                response.success = True
                response.message = "not in degraded mode (no-op)"
                log.info("Reset called but not in degraded mode")
                return response

            # Phase (1): 그리퍼 재연결.
            grip_ok, grip_msg = self.chess_mover.reconnect_gripper()
            if not grip_ok:
                log.error(f"Reset phase (1) gripper FAILED: {grip_msg}")
                response.success = False
                response.message = f"gripper reconnect failed: {grip_msg}"
                return response

            # Phase (2): safety mode 복원 — RECOVERY → AUTONOMOUS.
            safety_ok, safety_msg = self.chess_mover.restore_safety_mode()

            if safety_ok:
                self._degraded = False
                self._degraded_reason = ""
                log.info(f"Reset OK — gripper={grip_msg}; safety={safety_msg}")
                response.success = True
                response.message = f"reset OK: {grip_msg}; {safety_msg}"
            else:
                # 그리퍼는 복구됐지만 safety mode 복원이 inconclusive — degraded flag를
                # 자동으로 clear하지 않는다. operator가 teach pendant에서 확인 후
                # ~/reset 재호출, 또는 별도 Service로 override (향후 범위).
                log.warn(
                    f"Reset phase (2) safety mode unconfirmed: {safety_msg}. "
                    f"Degraded flag retained. Verify teach pendant + re-call ~/reset."
                )
                response.success = False
                response.message = (
                    f"gripper OK ({grip_msg}); safety unconfirmed ({safety_msg}); "
                    f"degraded flag retained — verify pendant"
                )
        return response

    def cancel_callback(self, goal_handle):
        """액션 취소 요청 처리"""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """실제 로봇 동작 수행.

        - V12 enabling: ``_is_executing``은 ``goal_callback``에서 atomic하게 claim됨
          (R1-2 V12 atomic-fix). 이 callback은 clear만 담당.
        - R1-4: ``FailsafeError`` (Modbus 단절) → degraded 모드 진입 + ABORT.
          일반 ``Exception`` → ABORT만 (degraded 진입 안 함 — 모션 자체 실패는
          페일세이프 대상 아님; 통신 장애만 degraded 처리).
        """
        self.get_logger().info("Executing goal...")

        result = MoveChessPiece.Result()
        try:
            # goal_handle을 통째로 넘겨준다.
            self.chess_mover.perform_task(goal_handle)

            goal_handle.succeed()
            result.success = True
        except FailsafeError as e:
            self.get_logger().error(f"L1 FAILSAFE during execution: {e}")
            with self._degraded_lock:
                self._degraded = True
                self._degraded_reason = f"L1 failsafe: {e}"
            goal_handle.abort()
            result.success = False
            result.message = "L1 failsafe — recovery via ~/reset"
        except Exception as e:
            self.get_logger().error(f"Task failed: {e}")
            goal_handle.abort()
            result.success = False
            result.message = f"task failed: {type(e).__name__}: {e}"
        finally:
            with self._execution_lock:
                self._is_executing = False

        return result

def main(args=None):
    rclpy.init(args=args)
    robot_node = Node('dsr_robot_node', namespace=ROBOT_ID)
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = robot_node
    # MultiThreadedExecutor 사용: 액션 서버가 동작 중이어도 로봇 상태 보고 및
    # 기타 callback이 원활하게 동작하도록 한다.
    node = RobotActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # rclpy.ok() 가드: SIGINT 시 상위 launch가 이미 context를 shutdown 한 경우
        # 'rcl_shutdown already called' 트레이스를 회피. (PB-6 fix.)
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()