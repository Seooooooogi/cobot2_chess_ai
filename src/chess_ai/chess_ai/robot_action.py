"""Doosan M0609 + RG2 모션 액션 서버 (entry point: ``ros2 run chess_ai robotaction``).

체스 수(예: ``"e2e4"``)와 보드 dict이 담긴 ``MoveChessPiece`` action goal을 받아
pick-and-place 시퀀스(앙파상·캐슬링 분기 포함)를 실행한다.

ROS2 Interfaces:
    Action server: ``move_chess_piece`` (chess_ai_interfaces/action/MoveChessPiece).
    Service: ``~/reset`` (std_srvs/srv/Trigger) — L1 failsafe 수동 복구.

Motion API:
    DSR_ROBOT2 Python wrapper (``movej``/``movel``/``mwait``/``wait``)를 직접 호출.
    ROS2 service (``/dsr01/dsr_controller2``) 경로는 사용하지 않는다.
    RG2 그리퍼는 OnRobot Modbus TCP ``192.168.1.1:502``로 직접 통신.
    Tool: TCP ``GripperDA_v1_1``, weight ``Tool Weight``.

Robot mode:
    ``ROBOT_MODE`` env var (default ``"virtual"``). DSR launch의 ``mode:=``와 반드시
    일치해야 한다. virtual에서는 ``_init_gripper``가 Modbus connect를 skip 하고,
    real에서는 소켓 open 후 ``is_socket_open()``으로 fail-loud 검증한다.

Safety layers:
    L0 (hardware E-stop): teach pendant operator 주도. 모터 차단, 상시 가용. 본 모듈과 독립.
    L1 (software graceful degradation): Modbus 단절·status polling timeout·pre-flight ping
        실패에서 트리거. ``set_safety_mode(RECOVERY, STOP)``로 모션 halt + 서보 유지.
        복구는 ``~/reset`` Service (operator의 보드/로봇 점검 선행 가정).

Environment:
    ROBOT_MODE (str, default "virtual"): "virtual" 또는 "real".
    GRIPPER_FAULT_MODE (str, optional): virtual 모드에서만 의미. ``MockGripper``의 fault
        injection 트리거 — failsafe 회로 테스트용.
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

from chess_ai_interfaces.action import MoveChessPiece


class FailsafeError(RuntimeError):
    """L1 software failsafe (Modbus 단절·status timeout)가 트리거됐을 때 raise.

    ``execute_callback``은 본 예외를 받아 action server를 degraded 상태로 표시하고
    ABORT한다. 일반 ``Exception``과 구분되므로, 페일세이프 대상이 아닌 모션 에러로는
    degraded 모드가 lock 되지 않는다.
    """


# vendored DSR_ROBOT2.set_safety_mode 내부 wait_for_service에는 timeout이 없어 M0609
# controller unreachable 시 영구 hang. daemon thread + join(timeout)으로 호출자가
# bounded 시간 안에 반드시 반환되도록 강제 — 정상 ROS2 service 왕복은 sub-100ms이므로
# 2.0s는 nominal response를 막지 않으면서 deadlock을 잡아낸다
_SAFETY_CALL_TIMEOUT_SEC = 2.0


class MockGripper:
    """Failsafe 회로 테스트용 fault-injectable 가상 그리퍼.

    ``ROBOT_MODE == "virtual"`` AND ``GRIPPER_FAULT_MODE`` env가 설정된 경우에만 활성화.
    ``MovingChessPiece``가 호출하는 ``onrobot.RG``의 일부 API만 노출한다.

    Args:
        fault_mode (str): 아래 모드 중 하나, 또는 빈 문자열.
        log_fn (Callable[[str], None]): 로그 출력 함수.

    Fault modes (``GRIPPER_FAULT_MODE`` 값별):
        ``ping_fail``: ``is_socket_open()``이 False — pre-flight ping 실패.
        ``disconnect_on_grip``: ``close_gripper()``에서 ConnectionException raise.
        ``disconnect_on_release``: ``open_gripper()``에서 ConnectionException raise.
        ``disconnect_on_status``: ``get_status()``에서 ConnectionException raise.
        ``hang_on_status``: ``get_status()``가 항상 busy=True — timeout 경로 테스트.
        그 외 / 미설정: no-op (legacy ``None`` 그리퍼와 동등).
    """

    def __init__(self, fault_mode: str, log_fn):
        self.fault_mode = fault_mode
        self._log = log_fn
        self._log(f"[MOCK] MockGripper instantiated (fault_mode={fault_mode!r})")

    def _maybe_raise(self, trigger: str):
        """``self.fault_mode == trigger``이면 ConnectionException(가능 시) / ConnectionError를 raise."""
        if self.fault_mode != trigger:
            return
        try:
            from pymodbus.exceptions import ConnectionException
            raise ConnectionException(f"MockGripper fault: {trigger}")
        except ImportError:
            raise ConnectionError(f"MockGripper fault: {trigger}")

    def is_socket_open(self) -> bool:
        """``ping_fail`` 모드에서만 False — 그 외 항상 True."""
        return self.fault_mode != "ping_fail"

    def close_gripper(self):
        """no-op + 로그. ``disconnect_on_grip``이면 raise."""
        self._log("[MOCK] close_gripper")
        self._maybe_raise("disconnect_on_grip")

    def open_gripper(self):
        """no-op + 로그. ``disconnect_on_release``이면 raise."""
        self._log("[MOCK] open_gripper")
        self._maybe_raise("disconnect_on_release")

    def get_status(self):
        """``onrobot.RG.get_status``와 같은 list[int]를 반환한다.

        ``disconnect_on_status``면 raise. ``hang_on_status``이면 busy=True를 영구 반환.
        그 외 idle ``[False]``.
        """
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

# default를 virtual로 고정 — ROBOT_MODE 설정 누락 시에도 하드웨어가 silently 활성화되지
# 않도록 (Tier 0 safety policy)
ROBOT_MODE = os.getenv("ROBOT_MODE", "virtual")

# goal validation 상수 — stockfish.py의 piece_match dict와 동일 셋
VALID_PIECES = frozenset({"WP", "WR", "WN", "WB", "WQ", "WK",
                          "BP", "BR", "BN", "BB", "BQ", "BK"})
VALID_FILES = frozenset("ABCDEFGH")
VALID_RANKS = frozenset("12345678")
VALID_PROMOTIONS = frozenset("qrbn")


class MovingChessPiece:
    """모션 로직 owner — config 로드, 보드 좌표 사전 계산, pick-and-place 실행.

    Args:
        logger_node (rclpy.node.Node): ``log()``가 사용할 logger source.
        grip_status_timeout_sec (float): RG2 status polling deadline(초). RobotActionServer의
            ROS2 parameter ``grip_status_timeout_sec``에서 주입. 외부에서 인스턴스화될 때
            (테스트) 기본 5.0.

    Construction 순서:
        1. logger / timeout 저장.
        2. 모션 파라미터(vel/acc/time/wait) + 기준 pose 기본값 설정.
        3. ``_init_gripper()`` — virtual 모드에서 None, real 모드에서 RG2 Modbus 클라이언트.
        4. ``load_initial_config()`` — ``data.json``으로 기본값 override.
        5. ``calculate()`` — 64 칸 전체에 대해 ``posx_board_list``/``posx_over_list``/
           ``posx_under_list`` 사전 계산.

    Note:
        real 모드 시 ``_init_gripper``가 ``192.168.1.1:502``로 Modbus TCP 소켓을 open.
        기본 ``grip_status_timeout_sec`` 5.0초는 nominal RG2 close/open(<1s)을 막지 않으면서
        stall을 잡아내는 보수적 값이다.
    """

    def __init__(self, logger_node: Node, grip_status_timeout_sec: float = 5.0):
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
            ``onrobot.RG`` (real 모드) | ``MockGripper`` (virtual + ``GRIPPER_FAULT_MODE``
            설정) | ``None`` (그 외 virtual — no-op 동작).

        Raises:
            RuntimeError: real 모드에서 ``client.is_socket_open()``이 False일 때.
                pymodbus 2.x는 connect 실패를 silently 삼키므로 명시 검증으로 fail-loud.

        Note:
            real 모드만 ``TOOLCHARGER_IP:TOOLCHARGER_PORT`` (192.168.1.1:502)로
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
        # pymodbus 2.x가 connect 실패를 silent하게 삼키므로 명시 검증으로 fail-loud
        if not rg.client.is_socket_open():
            raise RuntimeError(
                f"RG2 gripper Modbus connect failed: "
                f"{TOOLCHARGER_IP}:{TOOLCHARGER_PORT}"
            )
        self.log(f"[REAL] RG2 gripper connected")
        return rg

    def check_modbus_alive(self) -> bool:
        """Pre-flight 그리퍼 liveness 체크 — ``goal_callback``의 F2 게이트에서 호출.

        Returns:
            bool: virtual 모드(그리퍼 ``None``) 또는 소켓 열림 → True. 단절·예외·
                ``MockGripper`` ``ping_fail`` 모드 → False.
        """
        if self.gripper is None:
            return True
        try:
            return bool(self.gripper.is_socket_open())
        except Exception as e:
            self.log(f"[FAILSAFE] Modbus liveness check exception: {e}")
            return False

    def reconnect_gripper(self) -> tuple[bool, str]:
        """L1 failsafe 복구를 위해 그리퍼를 재초기화한다 (``~/reset``에서 호출).

        재초기화 직전 ``self.gripper = None``으로 stale 핸들을 비운다 — 다음 실패 시
        깨진 인스턴스가 아니라 known-empty 상태가 남도록.

        Returns:
            tuple[bool, str]: ``(ok, message)``. virtual / Modbus 재open 성공 / 예외별로
                사람이 읽을 수 있는 메시지를 함께 반환한다.
        """
        self.gripper = None
        try:
            self.gripper = self._init_gripper()
            if ROBOT_MODE == "virtual":
                return True, "virtual gripper reset"
            return True, f"gripper reconnected at {TOOLCHARGER_IP}:{TOOLCHARGER_PORT}"
        except Exception as e:
            return False, f"reconnect failed: {type(e).__name__}: {e}"

    def _call_safety_mode_bounded(self, safety_mode: int, safety_event: int, label: str) -> bool:
        """Daemon thread + ``join(timeout)``으로 ``set_safety_mode``를 bounded하게 호출한다.

        vendored ``DSR_ROBOT2.set_safety_mode`` 내부 ``wait_for_service`` 루프엔 timeout이
        없어 M0609 controller unreachable 시 영구 hang. 본 헬퍼는 호출자가
        ``_SAFETY_CALL_TIMEOUT_SEC`` 안에 반드시 반환되도록 강제한다.

        Args:
            safety_mode (int): DRFC ``SAFETY_MODE_*`` 상수.
            safety_event (int): DRFC ``SAFETY_MODE_EVENT_*`` 상수.
            label (str): 로그 메시지에 끼울 라벨.

        Returns:
            bool: 호출 완료 → True. timeout/예외 → False (로그 기록).
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
        """L1 failsafe로 진입 — ``set_safety_mode(RECOVERY, STOP)``로 모션 halt + 서보 유지.

        L0 (teach pendant 하드웨어 E-stop)는 operator 주도이며 본 경로와 독립이다.
        본 메서드는 software graceful-degradation 레이어이지 catastrophic-safety 레이어가
        아니다.

        Args:
            reason (str): 페일세이프 트리거 원인 (로그용).

        Note:
            ``DRFC`` import 실패 시에는 로그만 남기고 safety 호출은 skip 한다.
            controller unreachable로 인한 indefinite hang은 ``_call_safety_mode_bounded``로
            차단된다. 복구 경로는 ``~/reset`` Service (operator의 물리 점검 선행 가정).
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
        """RECOVERY → AUTONOMOUS 전환 시도 — ``~/reset``의 phase (2)에서 호출.

        Returns:
            tuple[bool, str]: ``(called_ok, message)``. False라고 무조건 unsafe는 아니다 —
                M0609는 software AUTONOMOUS 진입이 성공하기 전에 teach pendant의 수동
                확인을 요구할 수 있으며, operator가 pendant에서 safety 상태를 해제한 뒤
                ``~/reset``을 재호출해야 할 수도 있다.
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
        """``[YYYY-MM-DD HH:MM:SS]`` prefix를 붙여 logger_node 경유로 INFO 로그를 출력한다."""
        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        full = f"[{now}] {msg}"
        self.logger_node.get_logger().info(full)

    def load_initial_config(self):
        """``data.json`` (모듈 옆 위치)으로 모션 기본값을 in-place 갱신한다.

        한글 key 14개를 읽는다:
            ``속도``, ``가속도``, ``시간``, ``mwait_시간``, ``wait_시간``, ``홈_관절좌표``,
            ``A1_좌표``, ``무덤_관절좌표``, ``무덤_관절좌표_오버``, ``z축_간격``,
            ``posnumx_interval``, ``poschary_interval``, ``posnumy_interval``,
            ``poscharx_interval``.

        Note:
            파일 부재 / 파싱 실패 시 생성자 default가 유지된다 (silent fallback). 인스턴스
            속성은 in-place 갱신.
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

    def grip(self):
        """RG2를 닫는다 — ``close_gripper`` + idle polling.

        Raises:
            FailsafeError: Modbus 단절(pymodbus exception) 또는 status polling timeout 시.
                raise 직전에 ``_enter_failsafe``가 자동 실행되어 L1 모드로 진입한다.

        Note:
            virtual 모드(``self.gripper is None``)에서는 no-op 로그만 남기고 즉시 반환.
        """
        if self.gripper is None:
            self.log("[VIRTUAL] grip (no-op)")
            return
        try:
            self.gripper.close_gripper()
            self._wait_gripper_idle("grip")
        except Exception as e:
            self._enter_failsafe(f"grip(): {type(e).__name__}: {e}")
            raise FailsafeError(f"grip(): {e}") from e

    def release(self):
        """RG2를 연다 — ``open_gripper`` + idle polling.

        Raises:
            FailsafeError: ``grip()``과 동일 — Modbus 단절 / status polling timeout.

        Note:
            virtual 모드(``self.gripper is None``)에서는 no-op 로그만 남기고 즉시 반환.
        """
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
        """``get_status()[0]`` (busy 비트)이 0이 될 때까지 ``_grip_status_timeout_sec`` 한도 안에서 polling.

        원래 루프엔 timeout이 없어 pymodbus 2.x가 degraded 연결에서 hang하거나 stale truthy
        값을 반환하면 무한 대기에 빠진다. 본 메서드는 indefinite hang을 ``TimeoutError``로
        승격해 호출자(``grip``/``release``)의 페일세이프 경로가 트리거되도록 한다.

        Args:
            op_label (str): 에러 메시지에 끼울 작업 이름(예: ``"grip"``).

        Raises:
            TimeoutError: polling이 ``_grip_status_timeout_sec``를 초과.
            Exception: ``get_status()``의 pymodbus / socket 에러 — 호출자가 wrap 한다.

        Warning:
            deadline은 ``get_status()`` 호출 사이에만 검사된다. ``get_status()`` 자체가
            half-open 소켓에서 hang하면 deadline에 도달하지 못한다 (pymodbus의 socket-level
            timeout은 ``onrobot.RG`` 래퍼에서 명시 설정되지 않음).
        """
        deadline = time.monotonic() + self._grip_status_timeout_sec
        while self.gripper.get_status()[0]:
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"{op_label} status polling exceeded {self._grip_status_timeout_sec}s"
                )
            time.sleep(0.25)

    def calculate(self):
        """64 칸 전체에 대한 로봇 TCP 좌표를 사전 계산해 dict 3종에 저장한다.

        좌표 체계 (``data.json`` 기반):
            ``posx_A1``: A1 칸 좌표 ``[x, y, z, rx, ry, rz]`` (mm, deg).
            ``posnumx_interval`` / ``posnumy_interval``: 숫자(1→8) 방향 x 증분 + y 미세 보정.
            ``poscharx_interval`` / ``poschary_interval``: 문자(A→H) 방향 x 미세 보정 + y 증분.
            ``z_posx_interval``: pick/place 시 수직 접근 안전 높이 오프셋.

        생성하는 dict 3종:
            ``posx_board_list``: 보드 레벨 좌표 (말이 놓이는 높이).
            ``posx_over_list``: 보드 레벨 + ``z_posx_interval`` (수직 접근 전 안전 높이).
            ``posx_under_list``: 보드 레벨 + 3 mm (pick 시 살짝 눌러잡는 위치).

        Note:
            보드 평면 가정 — 모든 칸의 z·rx·ry·rz는 A1과 동일. 미세 보정 항목으로 비수직
            체스판을 일부 보상한다.
        """
        self.posx_board_list = {}
        self.posx_over_list = {}
        self.posx_under_list = {}
        characters = ('A', 'B', 'C', 'D', 'E', 'F', 'G', 'H')
        for c in range(8):
            for j in range(8):
                # A1 기준으로 행 증분(j) + 열 미세 보정(c)을 합성
                posx_a = [self.posx_A1[0]+self.posnumx_interval*j+self.posnumy_interval*j, self.posx_A1[1]+self.poscharx_interval*c-self.poschary_interval*c, self.posx_A1[2]]
                posx_a.extend(self.posx_A1[3:6])
                self.posx_board_list[f"{characters[c]}{j+1}"] = posx_a

                posx_over = posx_a.copy()
                posx_over[2] += self.z_posx_interval
                self.posx_over_list[f"{characters[c]}{j+1}"] = posx_over

                # 3 mm 고정 — 기물 높이에 따라 적정성이 달라질 수 있음
                posx_under = posx_a.copy()
                posx_under[2] += 3
                self.posx_under_list[f"{characters[c]}{j+1}"] = posx_under

    def perform_task(self, goal_handle):
        """한 체스 수에 대한 pick-and-place 시퀀스를 실행한다.

        Args:
            goal_handle: rclpy ActionServer goal handle. ``goal_handle.request``는
                ``command`` (UCI, 예: ``"e2e4"``)와 ``pieces_dict`` (JSON 직렬화된
                ``A1``..``H8`` → piece code 매핑)을 운반한다.

        분기:
            앙파상 (폰 + 대각선 이동 + 목적지 빈 칸): 잡힌 폰을 lift → ``posj_tomb``에 deposit
                한 뒤 공통 섹션으로 fall-through (폰 자체 이동 수행).
            캐슬링 (킹 + column delta == 2): 대응 룩을 먼저 ``H↔F`` 또는 ``A↔D``로 옮긴 뒤
                공통 섹션에서 킹 이동.
            일반 capture (``target ≠ None``): 목적지 기물을 ``posj_tomb``으로 옮기는
                pre-capture 후 from→to 이동.
            일반 이동: from → to pick-and-place.

        Note:
            모션은 DSR_ROBOT2 wrapper(``movej``/``movel``)를 직접 호출 — ROS2 service 경로
            사용 없음. 그리퍼는 ``grip()``/``release()``로 동기 제어. ``DSR_ROBOT2``는
            function-level import (global import 시 ``DR_init.__dsr__node`` 사전 할당 필요).
        """
        command = goal_handle.request.command
        pieces_dict = json.loads(goal_handle.request.pieces_dict)
        from_pos = command[0:2].upper()
        to_pos = command[2:4].upper()

        piece_from = pieces_dict.get(from_pos)
        target = pieces_dict.get(to_pos)
        self.log(f"Moving piece: {piece_from} from {from_pos} to {to_pos}, target: {target}")

        self.log("Moving piece starts")
        from DSR_ROBOT2 import movej, movel, mwait, wait

        # 홈 위치로 이동 + 그리퍼 열기 — 이전 동작 상태 초기화
        movej(self.basic_posj, vel=self.vel, acc=self.acc)
        mwait(self.mwait_time)
        self.release()

        # ── 분기 1: 앙파상 ─────────────────────────────────────────────
        # 폰이 대각선으로 이동하는데 목적지가 비어있으면 앙파상.
        # 잡히는 폰의 위치: to_pos의 열 + from_pos의 행 (예: 백 폰이 e5→d6이면 d5의 흑 폰)
        if piece_from[1] == "P":
            if from_pos[0] != to_pos[0] and target is None:
                en_passant = ''.join([to_pos[0], from_pos[1]])
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
                # 잡힌 기물 처리만 하고 폰 자체 이동은 아래 공통 섹션에서 fall-through

        # ── 분기 2: 캐슬링 ────────────────────────────────────────────
        # 킹사이드(G 파일): 룩 H→F. 퀸사이드(C 파일): 룩 A→D
        elif piece_from[1] == "K":
            if abs(ord(from_pos[0])-ord(to_pos[0])) == 2:
                if to_pos[0] == "G":
                    castling_from = "H" + from_pos[1]
                    castling_to = "F" + from_pos[1]
                else:
                    castling_from = "A" + from_pos[1]
                    castling_to = "D" + from_pos[1]
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
        # 목적지에 기물이 있으면 먼저 tomb으로 옮긴다
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

        # ── 공통: 실제 기물 이동 (pick & place) ───────────────────────
        # from_pos에서 pick → to_pos의 under(살짝 눌러놓기) 위치로 place
        movel(self.posx_over_list[from_pos], time=self.time)
        mwait(self.mwait_time)
        movel(self.posx_board_list[from_pos], time=self.time)
        wait(self.wait_time)
        self.grip()
        movel(self.posx_over_list[from_pos], time=self.time)
        mwait(self.mwait_time)
        movel(self.posx_over_list[to_pos], time=self.time)
        mwait(self.mwait_time)
        movel(self.posx_under_list[to_pos], time=self.time)
        wait(self.wait_time)
        self.release()
        movel(self.posx_over_list[to_pos], time=self.time)
        mwait(self.mwait_time)
        movej(self.basic_posj, vel=self.vel, acc=self.acc)
        mwait(self.mwait_time)
        self.log("Moving piece completed")


class RobotActionServer(Node):
    """``move_chess_piece`` action server + ``~/reset`` failsafe 복구 service 호스트.

    Action server:
        move_chess_piece (chess_ai_interfaces/action/MoveChessPiece): goal accept → execute →
            result. accept는 다중 검증 게이트(F1 → V1-V11+V13 → F2 → V12)를 통과해야 한다.

    Services:
        ~/reset (std_srvs/srv/Trigger): L1 degraded 상태에서 수동 복구. two-phase
            (gripper reconnect + safety mode 복원).

    Parameters:
        grip_status_timeout_sec (double, default 5.0): RG2 status polling deadline(초).
            ``MovingChessPiece``에 주입.

    Concurrency:
        ``_execution_lock``: ``_is_executing`` 보호. ``goal_callback``의 V12 게이트(atomic
            claim)에서 set, ``execute_callback`` finally에서 clear.
        ``_degraded_lock``: ``_degraded``/``_degraded_reason`` 보호. F1(goal 거부)·
            ``execute_callback``(failsafe entry)·``_reset_callback``(복구)에서 access.

    Note:
        ``main()``이 별도로 생성한 auxiliary 노드 ``dsr_robot_node``는 ``DR_init`` 변수
        호스팅용이며 executor에 add 되지 않는다. spin되는 노드는 본 ``RobotActionServer``뿐.
    """

    def __init__(self):
        super().__init__('robot_action_server')

        # 하드웨어 의존 timing은 ROS2 parameter로 노출 — 환경별 튜닝 가능
        self.declare_parameter('grip_status_timeout_sec', 5.0)
        grip_timeout = self.get_parameter('grip_status_timeout_sec').value

        self.chess_mover = MovingChessPiece(self, grip_status_timeout_sec=grip_timeout)

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
            "(arm/gripper mode mismatch is a safety risk)"
        )

        # 동시 실행 추적 — _is_executing은 _execution_lock으로 보호
        self._execution_lock = threading.Lock()
        self._is_executing = False

        # L1 failsafe 상태. _degraded=True 인 동안 후속 goal은 모두 REJECT — ~/reset이
        # 호출될 때까지 유지
        self._degraded_lock = threading.Lock()
        self._degraded = False
        self._degraded_reason = ""

        # ~/reset → /robot_action_server/reset (소유 노드의 사설 namespace)
        self._reset_srv = self.create_service(
            Trigger,
            '~/reset',
            self._reset_callback,
            qos_profile=qos_profile_services_default,
        )

    def _validate_goal(self, goal_request) -> tuple[bool, str]:
        """ACCEPT 전 goal을 다중 규칙으로 검증한다.

        검사 항목 (``V<n>`` 라벨은 reason 문자열 prefix에 포함):
            V1: command 타입과 길이 (str, 4 또는 5).
            V2/V3: from-file ∈ a-h, from-rank ∈ 1-8.
            V4/V5: to-file/to-rank 동일 검사.
            V6: 5자 UCI 5번째 문자가 ``q/r/b/n``.
            V7: from ≠ to.
            V13: 5자 UCI hard reject — ``execute_callback``이 ``[0:4]``만 소비해 프로모션
                미지원이므로 ACCEPT 시 board dict desync 위험.
            V8: pieces_dict JSON 파싱.
            V9: 비어 있지 않은 dict.
            V10: from square에 기물 존재 — execute의 homing 모션 전에 차단.
            V11: from/to piece가 유효 코드.

        Args:
            goal_request (MoveChessPiece.Goal): action goal.

        Returns:
            tuple[bool, str]: ``(is_valid, reason)``. valid면 reason은 빈 문자열.

        Note:
            V12(concurrent goal)는 의도적으로 본 메서드 밖, ``goal_callback`` 안에서 처리한다.
            check-then-act가 아닌 lock 보호 하의 atomic claim-and-set이어야 두 callback이
            ``_is_executing=False``를 동시에 관찰하는 TOCTOU race가 차단된다.
        """
        cmd = goal_request.command

        if not isinstance(cmd, str) or len(cmd) not in (4, 5):
            cmd_len = len(cmd) if isinstance(cmd, str) else "N/A"
            return False, f"V1: command must be str of length 4 or 5, got {type(cmd).__name__} len={cmd_len}"

        cmd_upper = cmd.upper()
        from_sq = cmd_upper[0:2]
        to_sq = cmd_upper[2:4]

        if from_sq[0] not in VALID_FILES:
            return False, f"V2: from-file '{cmd[0]}' not in a-h"
        if from_sq[1] not in VALID_RANKS:
            return False, f"V3: from-rank '{cmd[1]}' not in 1-8"

        if to_sq[0] not in VALID_FILES:
            return False, f"V4: to-file '{cmd[2]}' not in a-h"
        if to_sq[1] not in VALID_RANKS:
            return False, f"V5: to-rank '{cmd[3]}' not in 1-8"

        if len(cmd) == 5 and cmd[4].lower() not in VALID_PROMOTIONS:
            return False, f"V6: promotion piece '{cmd[4]}' not in qrbn"

        if from_sq == to_sq:
            return False, f"V7: from == to ({from_sq})"

        if len(cmd) == 5:
            return False, f"V13: promotion moves not supported by execute_callback (consumes [0:4] only)"

        try:
            d = json.loads(goal_request.pieces_dict)
        except (json.JSONDecodeError, TypeError) as e:
            return False, f"V8: pieces_dict JSON parse failed: {e}"

        if not isinstance(d, dict) or not d:
            return False, f"V9: pieces_dict must be non-empty dict, got {type(d).__name__}"

        piece = d.get(from_sq)
        if piece is None:
            return False, f"V10: source square {from_sq} is empty in pieces_dict"

        if piece not in VALID_PIECES:
            return False, f"V11: unknown piece code '{piece}' at {from_sq}"

        target = d.get(to_sq)
        if target is not None and target not in VALID_PIECES:
            return False, f"V11: unknown piece code '{target}' at {to_sq}"

        return True, ""

    def goal_callback(self, goal_request):
        """ACCEPT / REJECT를 결정한다 — F1 → V1-V11+V13 → F2 → V12 순서.

        Gate 순서와 의도:
            F1 (degraded): ``_degraded == True``이면 REJECT (operator의 ``~/reset`` 필요).
            V1-V11+V13: ``_validate_goal`` 호출.
            F2 (pre-flight Modbus ping): ``check_modbus_alive`` 실패 시 L1 failsafe 진입 +
                REJECT. 다음 goal은 F1에서 거부된다.
            V12 (atomic concurrency claim): ``_execution_lock`` 보호 하에 ``_is_executing``을
                check-and-set. 이미 실행 중이면 REJECT.

        REJECT 시 모션·그리퍼 명령은 일절 발행되지 않는다 — 안전 게이트는 모션 발생 이전이다.
        """
        log = self.get_logger()
        cmd_repr = repr(goal_request.command)

        with self._degraded_lock:
            if self._degraded:
                log.error(
                    f"Goal REJECTED [F1: degraded — {self._degraded_reason}]. "
                    f"Call ~/reset Service to recover. command={cmd_repr}"
                )
                return GoalResponse.REJECT

        is_valid, reason = self._validate_goal(goal_request)
        if not is_valid:
            log.error(f"Goal REJECTED [{reason}]. command={cmd_repr}")
            return GoalResponse.REJECT

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

        # V12: atomic claim. _is_executing=True를 여기서 set 해야 동시 호출되는 두 번째
        # goal_callback이 True를 관찰하고 REJECT 한다
        with self._execution_lock:
            if self._is_executing:
                log.warn(f"Goal REJECTED [V12: concurrent goal in progress]. command={cmd_repr}")
                return GoalResponse.REJECT
            self._is_executing = True

        log.info(f"Goal ACCEPTED: command={cmd_repr}")
        return GoalResponse.ACCEPT

    def _reset_callback(self, request, response):
        """L1 degraded에서 수동 복구 — two-phase 절차.

        Phase 1: 그리퍼 재연결 (``reconnect_gripper``). 실패 시 즉시 fail return.
        Phase 2: safety mode 복원 (``restore_safety_mode``) — ``set_safety_mode(AUTONOMOUS, ENTER)``.

        Phase 2 없이는 F1/F2/V 검사를 통과해도 다음 ``movel``이 M0609 controller의 RECOVERY
        모드 잔존으로 거부될 수 있다. Phase 2가 inconclusive이면 ``_degraded`` flag는 유지된
        채 operator가 teach pendant에서 확인 후 재시도해야 한다.

        Returns:
            Trigger.Response: ``success``는 두 phase 모두 confirmed일 때만 True.

        Warning:
            호출 전에 operator가 보드/로봇을 물리적으로 점검할 것이 전제다 — 본 service는
            software state만 정리할 뿐 하드웨어 안전 검증 책임은 operator에게 있다.
        """
        log = self.get_logger()
        with self._degraded_lock:
            if not self._degraded:
                response.success = True
                response.message = "not in degraded mode (no-op)"
                log.info("Reset called but not in degraded mode")
                return response

            grip_ok, grip_msg = self.chess_mover.reconnect_gripper()
            if not grip_ok:
                log.error(f"Reset phase (1) gripper FAILED: {grip_msg}")
                response.success = False
                response.message = f"gripper reconnect failed: {grip_msg}"
                return response

            safety_ok, safety_msg = self.chess_mover.restore_safety_mode()

            if safety_ok:
                self._degraded = False
                self._degraded_reason = ""
                log.info(f"Reset OK — gripper={grip_msg}; safety={safety_msg}")
                response.success = True
                response.message = f"reset OK: {grip_msg}; {safety_msg}"
            else:
                # 그리퍼는 복구됐지만 safety mode는 inconclusive — degraded flag는
                # 자동으로 clear 하지 않는다. operator pendant 확인 후 ~/reset 재호출 필요
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
        """현재 goal에 대한 cancel 요청을 무조건 ACCEPT 한다."""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Goal에 대한 모션 시퀀스를 실행하고 result를 채워 반환한다.

        예외 처리:
            ``FailsafeError`` (Modbus 단절·status timeout) → degraded 모드 lock + ABORT.
            그 외 ``Exception`` → ABORT만 (degraded 미진입). 모션 자체 실패는 페일세이프
                대상이 아니며, 통신 장애만 degraded로 lock 한다.

        Note:
            ``_is_executing`` flag는 ``goal_callback``의 V12 게이트에서 이미 set 되어 있다.
            본 callback은 ``finally``에서 lock 보호 하에 clear만 담당한다.
        """
        self.get_logger().info("Executing goal...")

        result = MoveChessPiece.Result()
        try:
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
    """Entry point — auxiliary ``dsr_robot_node``로 DR_init 변수 바인딩 후 ``RobotActionServer`` spin.

    ``MultiThreadedExecutor``를 사용해 action server 실행 중에도 ``~/reset`` Service와 기타
    callback이 병행 처리되도록 한다. SIGINT는 ``KeyboardInterrupt``로 받아 정상 종료한다.

    Note:
        auxiliary 노드는 executor에 add 되지 않는다 — DR_init이 참조하는 글로벌 변수의 호스트
        역할만 한다. spin되는 노드는 ``RobotActionServer``뿐. ``rclpy.shutdown()``은 ``rclpy.ok()``
        가드 후 호출 — supervisor가 이미 shutdown한 context를 재호출하면 ``RCLError`` 발생.
    """
    rclpy.init(args=args)
    robot_node = Node('dsr_robot_node', namespace=ROBOT_ID)
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = robot_node
    node = RobotActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
