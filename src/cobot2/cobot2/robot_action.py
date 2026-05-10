"""RobotActionServer — Doosan M0609 + RG2 motion action server (entry point: ``ros2 run cobot2 robotaction``).

Role:
    Receives ``MoveChessPiece`` action goals containing a chess move (e.g. ``"e2e4"``) plus the current
    board dict, and drives the robot through the corresponding pick-and-place sequence (with branches for
    en-passant and castling).

ROS2 Interfaces:
    Server: Action ``move_chess_piece`` (cobot2_interfaces/MoveChessPiece) (line 402-414)

    Auxiliary node ``dsr_robot_node`` (namespace=``dsr01``) is constructed at line 552 to host the global
    references DR_init reads from. **It is not added to the executor** (line 563 only spins
    ``RobotActionServer``) — # verify needed: whether DR_init can operate without spinning that node.

Hardware & Motion API:
    - DR_init / DSR_ROBOT2 Python wrapper bound to ``__dsr__id="dsr01"``, ``__dsr__model="m0609"`` (line 553-555).
    - Motion calls are direct DSR API (``movej``, ``movel``, ``mwait``, ``wait``) — **not** ROS2 service calls
      against ``/dsr01/dsr_controller2``. This is why ``/dsr01/servoj_stream`` etc. observed zero publishers
      in Phase 1-2 capture.
    - RG2 gripper via OnRobot Modbus TCP at ``192.168.1.1:502`` (line 75-76, hardcoded).
    - Tool: TCP ``GripperDA_v1_1``, weight ``Tool Weight`` (line 69-70).

Mode Selection:
    - ``ROBOT_MODE`` env var (default ``"virtual"``, line 80). Must match the DSR launch ``mode:=`` arg —
      mismatch is a Rule 9 safety risk (warned at line 419-422).
    - In ``virtual`` mode, ``MovingChessPiece._init_gripper`` skips the Modbus connect (line 138-140).
    - In ``real`` mode, the connect is attempted lazily and ``is_socket_open()`` is checked to fail loudly
      (Rule 7) instead of pymodbus 2.x's silent connect failure (line 149-154).

External Dependencies:
    - DR_init / DSR_ROBOT2 (vendored doosan-robot2)
    - ``cobot2.onrobot.RG`` wrapping pymodbus
    - ``data.json`` (alongside this file) for motion parameters and chess-board coordinates
      (``JSON_PATH`` line 71, loaded by ``MovingChessPiece.load_initial_config``)
    - ``cobot2_interfaces.action.MoveChessPiece``

Issues (Phase 1-1 doc Node 3):
    - RESOLVED R1-1: module-level ``gripper = RG(...)`` removed (2026-05-01) — moved into
      ``MovingChessPiece._init_gripper`` with ``ROBOT_MODE`` branch + ``is_socket_open()`` guard.
    - ~~IMPORTANT R1-2: ``goal_callback`` accepts unconditionally — no command validation → Rule 7.~~ **RESOLVED 2026-05-04**: ``_validate_goal`` (line 428) implements V1-V13 (UCI 형식, pieces_dict 무결성, from_pos 피스 존재·코드, 동시성). HARD REJECT 시 robot/gripper 미동작 보장 (Rule 9 모션 전 차단).
    - DEFERRED R1-3: ``TOOLCHARGER_IP/PORT`` hardcoded — kept by user decision (2026-05-10):
      single-host + fixed gripper IP scenario, env-ization cost > benefit. Revisit at Phase 6 multi-host.
    - RESOLVED R1-4 (2026-05-10): layered failsafe (Rule 9 명시).
      L0 = teach pendant hardware E-stop (operator-driven, motor cutoff, always available).
      L1 = software failsafe — Modbus disconnect detection → DR_init ``set_safety_mode(RECOVERY, STOP)``
      (Option 1 STOP + Option 3 HOLD fallback: motion halt with servo retained). Triggers:
      grip/release pymodbus exception, status polling timeout (5.0s), pre-flight ping fail (D).
      Recovery: ``~/reset`` (std_srvs/Trigger) — manual operator-initiated re-init (B1).
      Virtual fault injection (V2): ``GRIPPER_FAULT_MODE`` env routes ``MockGripper`` for testing.
    - ~~IMPORTANT R1-5: action server QoS not declared → Rule 4.~~ **RESOLVED 2026-05-04**: 5종 QoS 명시 (goal/result/cancel/feedback = ``qos_profile_services_default``, status = ``qos_profile_action_status_default``).
    # verify needed (Phase 1-1 line 156): ``dsr_robot_node`` is not added to the executor — does DR_init
        function correctly when its bound node is never spun?
    # verify needed R1-7: ``data.json`` Korean keys — coordinate accuracy unverified in virtual mode.
    - R1-8 RESOLVED 2026-05-01: unused ``feedback_msg`` (``MoveChessPiece.Feedback()``) removed from ``execute_callback``.
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

from cobot2_interfaces.action import MoveChessPiece # 커스텀 액션 임포트


class FailsafeError(RuntimeError):
    """R1-4: raised when L1 failsafe (Modbus disconnect) has been triggered.

    Caller (``execute_callback``) must mark the action server degraded and ABORT.
    Distinct from generic exceptions so non-failsafe errors don't trigger degraded mode.
    """


# R1-4 bounded safety-mode call timeout (s). DSR_ROBOT2.set_safety_mode wraps
# wait_for_service WITHOUT a timeout — if the M0609 controller is unreachable the
# call hangs forever (Rule 7 silent-fail). Daemon-thread + join(timeout) ensures
# _enter_failsafe / restore_safety_mode always return within bounded time.
# Nominal ROS2 service round-trip is sub-100ms; 2.0s catches deadlocks without
# delaying legitimate fault response.
_SAFETY_CALL_TIMEOUT_SEC = 2.0


class MockGripper:
    """R1-4 V2: fault-injectable virtual gripper for failsafe testing.

    Activated when ``ROBOT_MODE == "virtual"`` AND ``GRIPPER_FAULT_MODE`` env is set.
    Exposes the subset of ``onrobot.RG`` API that ``MovingChessPiece`` calls.

    Fault modes (env value):
        - ``ping_fail``           — ``is_socket_open()`` returns False (pre-flight ping fails)
        - ``disconnect_on_grip``  — ``close_gripper()`` raises ConnectionException
        - ``disconnect_on_release`` — ``open_gripper()`` raises ConnectionException
        - ``disconnect_on_status`` — ``get_status()`` raises ConnectionException
        - ``hang_on_status``      — ``get_status()`` always returns busy=True (timeout test)
        - any other / unset       — behaves as no-op virtual gripper (matches legacy ``None``)
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

# CLAUDE.md Tier 0: virtual mode first. Default virtual for safety —
# forgetting to set ROBOT_MODE never silently activates hardware.
ROBOT_MODE = os.getenv("ROBOT_MODE", "virtual")

# R1-2 goal_callback validation (Rule 7+9 defense-in-depth).
# Source: stockfish.py piece_match dict (12 entries).
VALID_PIECES = frozenset({"WP", "WR", "WN", "WB", "WQ", "WK",
                          "BP", "BR", "BN", "BB", "BQ", "BK"})
VALID_FILES = frozenset("ABCDEFGH")
VALID_RANKS = frozenset("12345678")
VALID_PROMOTIONS = frozenset("qrbn")


class MovingChessPiece:
    """Motion logic owner — loads config, pre-computes board coordinates, executes pick-and-place.

    Construction sequence:
        1. Stores ``logger_node`` for ``log()``.
        2. Sets default motion params (vel/acc/time/wait) and reference poses.
        3. ``_init_gripper()`` — None in virtual mode, RG2 Modbus client in real mode.
        4. ``load_initial_config()`` — overrides defaults from ``data.json`` (Korean keys).
        5. ``calculate()`` — pre-computes ``posx_board_list`` / ``posx_over_list`` /
           ``posx_under_list`` for all 64 cells from ``posx_A1`` + interval constants.

    Side Effects:
        - In real mode, ``_init_gripper`` opens a Modbus TCP socket to ``192.168.1.1:502``.
        - In virtual mode, no hardware contact.
    """

    def __init__(self, logger_node: Node, grip_status_timeout_sec: float = 5.0):
        """R1-4 / Rule 8: ``grip_status_timeout_sec`` injected from caller's ROS2
        parameter (RobotActionServer ``grip_status_timeout_sec``). Default 5.0s
        applies if instantiated outside the action server (tests / scripts).

        RG2 close/open completes < 1s nominally; 5.0s catches Modbus stalls
        without slowing legitimate motion.
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
        """Initialize the RG2 gripper based on ``ROBOT_MODE``.

        Returns:
            ``RG`` in real mode. ``MockGripper`` in virtual mode if ``GRIPPER_FAULT_MODE``
            env is set (R1-4 V2). ``None`` in virtual mode otherwise (legacy).

        Raises:
            RuntimeError — if real mode and ``rg.client.is_socket_open()`` is False
            (pymodbus 2.x silently swallows connect failures, so this guard fails loudly per Rule 7).

        Side Effects:
            Real mode only: opens a Modbus TCP socket to ``TOOLCHARGER_IP:TOOLCHARGER_PORT``
            (``192.168.1.1:502``).
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
        # ROS2 Rule 7: fail loudly. pymodbus 2.x silently swallows
        # connect failures; verify socket actually opened.
        if not rg.client.is_socket_open():
            raise RuntimeError(
                f"RG2 gripper Modbus connect failed: "
                f"{TOOLCHARGER_IP}:{TOOLCHARGER_PORT}"
            )
        self.log(f"[REAL] RG2 gripper connected")
        return rg

    def check_modbus_alive(self) -> bool:
        """R1-4 D: pre-flight gripper liveness check (called from goal_callback).

        # verify needed: ``is_socket_open()`` reads pymodbus connection state
        without performing network I/O — expected non-blocking. If pymodbus
        version drift introduces blocking behavior here, ``goal_callback`` becomes
        blocking too (Rule 2 violation). Phase 6 검증 시 timing 측정.

        Returns:
            True if virtual mode (no Modbus), or socket open. False on disconnect /
            exception / MockGripper ``ping_fail`` mode.
        """
        if self.gripper is None:
            return True
        try:
            return bool(self.gripper.is_socket_open())
        except Exception as e:
            self.log(f"[FAILSAFE] Modbus liveness check exception: {e}")
            return False

    def reconnect_gripper(self) -> tuple[bool, str]:
        """R1-4 B1: called by ``~/reset`` Service to recover from L1 failsafe.

        In virtual mode, re-instantiates (clears MockGripper state). In real mode,
        re-opens the Modbus socket via ``_init_gripper``. Caller decides whether to
        clear the degraded flag based on the returned tuple.

        Side effect: clears ``self.gripper`` to None before re-init so a subsequent
        failure leaves a known-empty state rather than a stale broken instance.
        """
        # MINOR fix: clear stale reference first.
        self.gripper = None
        try:
            self.gripper = self._init_gripper()
            if ROBOT_MODE == "virtual":
                return True, "virtual gripper reset"
            return True, f"gripper reconnected at {TOOLCHARGER_IP}:{TOOLCHARGER_PORT}"
        except Exception as e:
            return False, f"reconnect failed: {type(e).__name__}: {e}"

    def _call_safety_mode_bounded(self, safety_mode: int, safety_event: int, label: str) -> bool:
        """R1-4 helper: invoke DR_init ``set_safety_mode`` with hard timeout.

        Vendored ``DSR_ROBOT2.set_safety_mode`` (line 2031) contains an unbounded
        ``wait_for_service`` loop. Daemon thread + ``join(timeout)`` ensures the
        caller always returns within ``_SAFETY_CALL_TIMEOUT_SEC`` regardless of
        controller reachability — preserving Rule 7 fail-loud semantics.

        Returns True on completed call, False on timeout / exception (logged).

        # verify needed: actual round-trip latency on real M0609. Virtual DRCF
        emulator may not honor set_safety_mode at all (Phase 6 실기 검증 항목).
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
        """R1-4 L1: motion STOP + servo HOLD via DR_init ``set_safety_mode``.

        L0 (teach pendant hardware E-stop) is operator-driven and independent of this
        path — this method is the software graceful-degradation layer, not the
        catastrophic-safety layer (Rule 9 명시).

        Recovery: physical board check → ``~/reset`` Service.

        Notes:
            ``set_safety_mode(SAFETY_MODE_RECOVERY=2, SAFETY_MODE_EVENT_STOP=2)`` —
            see ``DSR_ROBOT2.py`` line 2016 / ``DRFC.py`` line 127-141.
            Bounded call via ``_call_safety_mode_bounded`` to prevent indefinite hang
            when M0609 controller is also unreachable.
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
        """R1-4 B1: attempt to transition safety mode RECOVERY → AUTONOMOUS as part of reset.

        # verify needed: M0609 may require teach pendant manual confirmation
        (E-stop reset / mode key) before software AUTONOMOUS entry succeeds.
        Software call alone may not be sufficient — operator should clear safety
        state on the pendant before invoking ``~/reset``. Phase 6 실기 검증 항목.

        Returns (called_ok, message). False does not necessarily mean unsafe —
        operator may need pendant interaction first.
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
        """Override motion defaults from ``data.json`` (alongside this module).

        Reads all 14 keys in the file — Korean motion params plus cell-interval keys (Phase 4 2026-05-01):
            ``속도``, ``가속도``, ``시간``, ``mwait_시간``, ``wait_시간``, ``홈_관절좌표``,
            ``A1_좌표``, ``무덤_관절좌표``, ``무덤_관절좌표_오버``, ``z축_간격``,
            ``posnumx_interval``, ``poschary_interval``, ``posnumy_interval``, ``poscharx_interval``.

        Side Effects:
            Updates instance attributes in place. If the file does not exist or fails to parse,
            the constructor's defaults remain.

        Notes:
            # verify needed R1-7: data.json Korean-keyed coordinates have not been verified in virtual mode.
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
        """R1-4: poll ``get_status()[0]`` until idle, with parameter-driven timeout.

        Original loop had no timeout — pymodbus 2.x can hang or return stale truthy
        values on degraded connections. Timeout converts indefinite hang into
        ``TimeoutError`` so caller's ``_enter_failsafe`` path engages.

        # verify needed (MINOR): the deadline is checked between ``get_status()``
        calls. If pymodbus ``get_status()`` itself hangs on a half-open socket the
        deadline is never reached. pymodbus has a default socket-level timeout but
        it is not explicitly configured by ``onrobot.RG``. Real-mode behavior
        pending Phase 6 검증; consider explicit ``client.set_timeout`` if needed
        (touches R1-3 onrobot wrapper territory).

        Raises:
            TimeoutError — polling exceeds ``_grip_status_timeout_sec``.
            Exception — pymodbus / socket error from ``get_status()`` (caller wraps).
        """
        deadline = time.monotonic() + self._grip_status_timeout_sec
        while self.gripper.get_status()[0]:
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"{op_label} status polling exceeded {self._grip_status_timeout_sec}s"
                )
            time.sleep(0.25)

    def calculate(self):
        self.posx_board_list = {}
        self.posx_over_list = {}
        self.posx_under_list = {}
        characters = ('A', 'B', 'C', 'D', 'E', 'F', 'G', 'H')
        for c in range(8):
            for j in range(8):
                posx_a = [self.posx_A1[0]+self.posnumx_interval*j+self.posnumy_interval*j, self.posx_A1[1]+self.poscharx_interval*c-self.poschary_interval*c, self.posx_A1[2]]
                posx_a.extend(self.posx_A1[3:6])
                self.posx_board_list[f"{characters[c]}{j+1}"] = posx_a

                posx_over = posx_a.copy()
                posx_over[2] += self.z_posx_interval
                self.posx_over_list[f"{characters[c]}{j+1}"] = posx_over

                posx_under = posx_a.copy()
                posx_under[2] += 3
                self.posx_under_list[f"{characters[c]}{j+1}"] = posx_under
                
    def perform_task(self, goal_handle):
        """Execute the pick-and-place sequence for one chess move.

        Args:
            goal_handle: rclpy ActionServer goal handle. ``goal_handle.request`` carries
                ``command`` (UCI move, e.g. ``"e2e4"``) and ``pieces_dict`` (JSON-serialized
                ``A1``..``H8`` → ``WP``/``BR``/... map).

        Branches inside the function:
            - Pawn diagonal into empty cell (``piece_from[1] == 'P'`` and column changes and target is None)
              → en-passant: lift captured pawn, deposit at ``posj_tomb``, return.
            - King two-column move (``piece_from[1] == 'K'`` and column delta == 2)
              → castling: relocate corresponding rook before moving the king.
            - ``target is not None`` → pre-capture: lift target piece to ``posj_tomb`` first.
            - Then move the moving piece from ``from_pos`` to ``to_pos``.

        Motion API:
            Imports ``movej``, ``movel``, ``mwait``, ``wait`` from ``DSR_ROBOT2`` directly.
            **Not** ROS2 service calls.

        Side Effects:
            - Issues motion commands through DR_init / DSR_ROBOT2 to the M0609 controller.
            - Opens / closes the RG2 gripper (no-op in virtual mode).

        Notes:
            R1-8 RESOLVED 2026-05-01: unused ``Feedback`` instance removed from ``execute_callback``.
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

        movej(self.basic_posj, vel=self.vel, acc=self.acc)
        mwait(self.mwait_time)
        self.release()

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
    """ROS2 action server hosting ``move_chess_piece``.

    Construction sequence:
        1. Instantiates ``MovingChessPiece`` (which loads ``data.json`` and pre-computes
           the 64-cell coordinate tables; in real mode also opens the Modbus connection).
        2. Creates ``ActionServer`` with ``goal_callback`` / ``cancel_callback`` / ``execute_callback``.
        3. Logs ``ROBOT_MODE`` with a Rule 9 mismatch warning.

    Notes:
        - ``goal_callback`` validates via ``_validate_goal`` (R1-2 RESOLVED 2026-05-04).
        - ``execute_callback`` toggles ``self._is_executing`` under ``self._execution_lock``
          to enable V12 (concurrent-goal REJECT in ``_validate_goal``).
        - The auxiliary node ``dsr_robot_node`` constructed in ``main()`` (line 552) is bound to
          ``DR_init.__dsr__node`` but **not** added to the executor at line 563 — only this node
          (``RobotActionServer``) is spun. # verify needed (Phase 1-1 line 156).
    """

    def __init__(self):
        super().__init__('robot_action_server')

        # Rule 8: hardware-dependent timing exposed as ROS2 parameter.
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

        # R1-4 L1 failsafe state. _degraded means subsequent goals are REJECTed
        # until ``~/reset`` Service is called.
        self._degraded_lock = threading.Lock()
        self._degraded = False
        self._degraded_reason = ""

        # R1-4 B1: manual recovery Service (Trigger). Resolved name: ``~/reset``
        # → ``/robot_action_server/reset`` (Rule 5 — owned by this node).
        self._reset_srv = self.create_service(
            Trigger,
            '~/reset',
            self._reset_callback,
            qos_profile=qos_profile_services_default,
        )

    def _validate_goal(self, goal_request) -> tuple[bool, str]:
        """Validate MoveChessPiece goal before accepting (R1-2 / Rule 7+9).

        Returns (is_valid, reason). Reason starts with ``V<N>:`` matching
        the rule index in the design doc. Caller (``goal_callback``) decides
        log severity based on prefix.
        """
        cmd = goal_request.command

        # V1: command type and length
        if not isinstance(cmd, str) or len(cmd) not in (4, 5):
            cmd_len = len(cmd) if isinstance(cmd, str) else "N/A"
            return False, f"V1: command must be str of length 4 or 5, got {type(cmd).__name__} len={cmd_len}"

        cmd_upper = cmd.upper()
        from_sq = cmd_upper[0:2]
        to_sq = cmd_upper[2:4]

        # V2-V3: from-square format
        if from_sq[0] not in VALID_FILES:
            return False, f"V2: from-file '{cmd[0]}' not in a-h"
        if from_sq[1] not in VALID_RANKS:
            return False, f"V3: from-rank '{cmd[1]}' not in 1-8"

        # V4-V5: to-square format
        if to_sq[0] not in VALID_FILES:
            return False, f"V4: to-file '{cmd[2]}' not in a-h"
        if to_sq[1] not in VALID_RANKS:
            return False, f"V5: to-rank '{cmd[3]}' not in 1-8"

        # V6: promotion piece char (only checked if length 5)
        if len(cmd) == 5 and cmd[4].lower() not in VALID_PROMOTIONS:
            return False, f"V6: promotion piece '{cmd[4]}' not in qrbn"

        # V7: from != to
        if from_sq == to_sq:
            return False, f"V7: from == to ({from_sq})"

        # V13: 5-char UCI HARD REJECT (OQ-1=A: execute_callback consumes [0:4] only,
        # promotion not yet supported → board dict desync risk if accepted).
        if len(cmd) == 5:
            return False, f"V13: promotion moves not supported by execute_callback (consumes [0:4] only)"

        # V8: pieces_dict JSON parse
        try:
            d = json.loads(goal_request.pieces_dict)
        except (json.JSONDecodeError, TypeError) as e:
            return False, f"V8: pieces_dict JSON parse failed: {e}"

        # V9: dict and non-empty
        if not isinstance(d, dict) or not d:
            return False, f"V9: pieces_dict must be non-empty dict, got {type(d).__name__}"

        # V10: from_sq has piece (Rule 9: block before homing motion at execute_callback line 285)
        piece = d.get(from_sq)
        if piece is None:
            return False, f"V10: source square {from_sq} is empty in pieces_dict"

        # V11: from-piece is valid code
        if piece not in VALID_PIECES:
            return False, f"V11: unknown piece code '{piece}' at {from_sq}"

        # V11 (extended, OQ-3=A): to-piece if non-None must also be valid
        target = d.get(to_sq)
        if target is not None and target not in VALID_PIECES:
            return False, f"V11: unknown piece code '{target}' at {to_sq}"

        # V12 (concurrent goal) intentionally moved out of _validate_goal to
        # ``goal_callback`` — must be atomic claim-and-set, not check-then-act.

        return True, ""

    def goal_callback(self, goal_request):
        """액션 목표 수락 여부 결정 (R1-2 V1-V13 + R1-4 F1/F2 + V12 atomic claim).

        REJECT 시 robot/gripper 미동작 (Rule 9: 모션 전 차단).
        Order: F1 (degraded) → V1-V11+V13 → F2 (Modbus ping) → V12 (atomic claim).
        V12 must be the LAST gate — atomic claim-and-set under ``_execution_lock``
        prevents the TOCTOU race where two concurrent callbacks both observe
        ``_is_executing=False`` and both ACCEPT.
        ``execute_callback`` clears the slot in its ``finally`` block.
        """
        log = self.get_logger()
        cmd_repr = repr(goal_request.command)

        # F1: degraded mode REJECT (B1 manual reset required).
        with self._degraded_lock:
            if self._degraded:
                log.error(
                    f"Goal REJECTED [F1: degraded — {self._degraded_reason}]. "
                    f"Call ~/reset Service to recover. command={cmd_repr}"
                )
                return GoalResponse.REJECT

        # V1-V11, V13 (V12 moved below for atomic semantics).
        is_valid, reason = self._validate_goal(goal_request)
        if not is_valid:
            log.error(f"Goal REJECTED [{reason}]. command={cmd_repr}")
            return GoalResponse.REJECT

        # F2: pre-flight Modbus ping (D). Failure auto-enters L1 failsafe so
        # next goal also REJECTs at F1 until manual reset.
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

        # V12: atomic concurrency claim — last gate before ACCEPT.
        # Set ``_is_executing=True`` here (not in execute_callback) so a second
        # concurrent goal_callback observes True and REJECTs.
        with self._execution_lock:
            if self._is_executing:
                log.warn(f"Goal REJECTED [V12: concurrent goal in progress]. command={cmd_repr}")
                return GoalResponse.REJECT
            self._is_executing = True

        log.info(f"Goal ACCEPTED: command={cmd_repr}")
        return GoalResponse.ACCEPT

    def _reset_callback(self, request, response):
        """R1-4 B1: manual recovery from L1 failsafe.

        Two-phase recovery:
            (1) gripper reconnect — re-open Modbus socket / clear MockGripper state.
            (2) safety mode restore — ``set_safety_mode(AUTONOMOUS, ENTER)``.
        Both are required for the next motion goal to succeed: phase (1) restores
        gripper I/O; phase (2) lifts the M0609 controller out of RECOVERY mode left
        by ``_enter_failsafe``. Without phase (2) the next ``movel`` is rejected
        by the controller even though F1/F2/V-checks pass.

        # verify needed: M0609 may require teach pendant manual confirmation
        before AUTONOMOUS entry succeeds. If phase (2) reports failure / timeout,
        operator should clear safety state on pendant before retrying ``~/reset``.
        Phase 6 실기 검증 항목.

        Operator is expected to physically inspect the board / robot before calling.
        """
        log = self.get_logger()
        with self._degraded_lock:
            if not self._degraded:
                response.success = True
                response.message = "not in degraded mode (no-op)"
                log.info("Reset called but not in degraded mode")
                return response

            # Phase (1): gripper reconnect.
            grip_ok, grip_msg = self.chess_mover.reconnect_gripper()
            if not grip_ok:
                log.error(f"Reset phase (1) gripper FAILED: {grip_msg}")
                response.success = False
                response.message = f"gripper reconnect failed: {grip_msg}"
                return response

            # Phase (2): safety mode restore — RECOVERY → AUTONOMOUS.
            safety_ok, safety_msg = self.chess_mover.restore_safety_mode()

            if safety_ok:
                self._degraded = False
                self._degraded_reason = ""
                log.info(f"Reset OK — gripper={grip_msg}; safety={safety_msg}")
                response.success = True
                response.message = f"reset OK: {grip_msg}; {safety_msg}"
            else:
                # Gripper recovered but safety mode restore inconclusive — DO NOT clear
                # degraded flag automatically. Operator must verify on teach pendant
                # and re-call ~/reset, or override via separate Service (future scope).
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

        - V12 enabling: ``_is_executing`` was claimed atomically in
          ``goal_callback`` (R1-2 V12 atomic-fix). This callback only clears it.
        - R1-4: ``FailsafeError`` (Modbus 단절) → degraded 모드 진입 + ABORT.
          Generic ``Exception`` → ABORT only (degraded 진입 안 함 — 모션 자체 실패는
          페일세이프 대상 아님; 통신 장애만 degraded 처리).
        """
        self.get_logger().info("Executing goal...")

        result = MoveChessPiece.Result()
        try:
            # goal_handle을 통째로 넘겨줍니다.
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
    # 멀티스레드 실행기를 사용하여 액션 서버가 동작하는 동안에도 
    # 로봇 상태 보고나 기타 콜백이 원활하게 작동하도록 합니다.
    node = RobotActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()