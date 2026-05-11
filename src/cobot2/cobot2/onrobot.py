#!/usr/bin/env python3
"""OnRobot RG2/RG6 그리퍼 Modbus TCP 드라이버.

역할:
    pymodbus를 통해 OnRobot RG 시리즈 그리퍼와 직접 통신한다.
    ROS2 노드나 서비스 없이 순수 Modbus TCP 레벨에서 동작한다.

사용처:
    robot_action.py 의 MovingChessPiece._init_gripper() 에서 인스턴스를 생성.
    grip() / release() 는 각각 close_gripper() / open_gripper() 를 호출한다.

Modbus 레지스터 맵 (OnRobot RG 시리즈, slave=65):
    address 0  : 목표 force (1/10 N 단위)
    address 1  : 목표 width (1/10 mm 단위)
    address 2  : control (1=grip, 8=stop, 16=grip_w_offset)
    address 258: fingertip offset (1/10 mm, signed two's complement)
    address 267: 현재 width (1/10 mm, fingertip offset 미포함)
    address 268: status 비트 필드 (bit0=busy, bit1=grip detected, bit2-6=safety)
    address 275: 현재 width (1/10 mm, fingertip offset 포함)

출처:
    vendor 코드 직접 카피 (pick2build/onrobot.py, JIUM 프로젝트).
    vendor ROS2 패키지(onrobot_rg_control)는 ROS2 node+service 제공 — 본 파일은
    그리퍼를 동기 호출로 쓰기 위해 Modbus를 직접 감싼 경량 래퍼.

주의:
    get_status()가 print()를 직접 호출함 (vendor 코드 원형 보존).
    robot_action.py 의 _wait_gripper_idle() 이 get_status()[0] (busy 플래그) 만 소비.
"""

from pymodbus.client.sync import ModbusTcpClient as ModbusClient


class RG():
    """OnRobot RG2 / RG6 그리퍼 Modbus TCP 클라이언트.

    생성자가 Modbus 연결을 즉시 시도한다 (open_connection()).
    robot_action.py 의 _init_gripper() 에서 is_socket_open() 으로 연결 성공 여부를
    재확인한다 (pymodbus 2.x는 연결 실패를 조용히 삼키므로 명시 확인 필요).

    Args:
        gripper: 'rg2' 또는 'rg6'. 그 외 값이면 즉시 return (연결 없음).
        ip:      그리퍼 컨트롤러 IP 주소.
        port:    Modbus TCP 포트 (OnRobot 기본 502).
    """

    def __init__(self, gripper, ip, port):
        self.client = ModbusClient(
            ip,
            port=port,
            stopbits=1,
            bytesize=8,
            parity='E',        # 짝수 패리티 — OnRobot 통신 규격
            baudrate=115200,   # 실제로는 TCP라 무시되나 라이브러리 파라미터 요구
            timeout=1)
        if gripper not in ['rg2', 'rg6']:
            print("Please specify either rg2 or rg6.")
            return
        self.gripper = gripper  # RG2/6
        # RG2와 RG6의 최대 개도/힘 (1/10 mm, 1/10 N 단위)
        if self.gripper == 'rg2':
            self.max_width = 400   # 40.0 mm
            self.max_force = 400   # 40.0 N
        elif self.gripper == 'rg6':
            self.max_width = 1600  # 160.0 mm
            self.max_force = 1200  # 120.0 N
        self.open_connection()

    def open_connection(self):
        """Opens the connection with a gripper."""
        self.client.connect()

    def close_connection(self):
        """Closes the connection with the gripper."""
        self.client.close()

    def get_fingertip_offset(self):
        """Reads the current fingertip offset in 1/10 millimeters.
        Please note that the value is a signed two's complement number.
        """
        result = self.client.read_holding_registers(
            address=258, count=1, unit=65)
        offset_mm = result.registers[0] / 10.0
        return offset_mm

    def get_width(self):
        """Reads current width between gripper fingers in 1/10 millimeters.
        Please note that the width is provided without any fingertip offset,
        as it is measured between the insides of the aluminum fingers.
        """
        result = self.client.read_holding_registers(
            address=267, count=1, unit=65)
        width_mm = result.registers[0] / 10.0
        return width_mm

    def get_status(self):
        """Reads current device status.
        This status field indicates the status of the gripper and its motion.
        It is composed of 7 flags, described in the table below.

        Bit      Name            Description
        0 (LSB): busy            High (1) when a motion is ongoing,
                                  low (0) when not.
                                  The gripper will only accept new commands
                                  when this flag is low.
        1:       grip detected   High (1) when an internal- or
                                  external grip is detected.
        2:       S1 pushed       High (1) when safety switch 1 is pushed.
        3:       S1 trigged      High (1) when safety circuit 1 is activated.
                                  The gripper will not move
                                  while this flag is high;
                                  can only be reset by power cycling.
        4:       S2 pushed       High (1) when safety switch 2 is pushed.
        5:       S2 trigged      High (1) when safety circuit 2 is activated.
                                  The gripper will not move
                                  while this flag is high;
                                  can only be reset by power cycling.
        6:       safety error    High (1) when on power on any of
                                  the safety switch is pushed.
        10-16:   reserved        Not used.

        Returns:
            list[int] — 7개 요소. [0]=busy, [1]=grip detected, [2-6]=safety bits.
            robot_action.py 의 _wait_gripper_idle() 은 [0](busy)만 소비.
        """
        # address 268 = status 레지스터, slave=65 (OnRobot 고정)
        result = self.client.read_holding_registers(
            address=268, count=1, unit=65)
        # 16비트를 이진 문자열로 변환 후 하위 비트부터 해석
        status = format(result.registers[0], '016b')
        status_list = [0] * 7
        if int(status[-1]):
            print("A motion is ongoing so new commands are not accepted.")
            status_list[0] = 1
        if int(status[-2]):
            print("An internal- or external grip is detected.")
            status_list[1] = 1
        if int(status[-3]):
            print("Safety switch 1 is pushed.")
            status_list[2] = 1
        if int(status[-4]):
            print("Safety circuit 1 is activated so it will not move.")
            status_list[3] = 1
        if int(status[-5]):
            print("Safety switch 2 is pushed.")
            status_list[4] = 1
        if int(status[-6]):
            print("Safety circuit 2 is activated so it will not move.")
            status_list[5] = 1
        if int(status[-7]):
            print("Any of the safety switch is pushed.")
            status_list[6] = 1

        return status_list

    def get_width_with_offset(self):
        """Reads current width between gripper fingers in 1/10 millimeters.
        The set fingertip offset is considered.
        """
        result = self.client.read_holding_registers(
            address=275, count=1, unit=65)
        width_mm = result.registers[0] / 10.0
        return width_mm

    def set_control_mode(self, command):
        """The control field is used to start and stop gripper motion.
        Only one option should be set at a time.
        Please note that the gripper will not start a new motion
        before the one currently being executed is done
        (see busy flag in the Status field).
        The valid flags are:

        1 (0x0001):  grip
                      Start the motion, with the target force and width.
                      Width is calculated without the fingertip offset.
                      Please note that the gripper will ignore this command
                      if the busy flag is set in the status field.
        8 (0x0008):  stop
                      Stop the current motion.
        16 (0x0010): grip_w_offset
                      Same as grip, but width is calculated
                      with the set fingertip offset.
        """
        result = self.client.write_register(
            address=2, value=command, unit=65)

    def set_target_force(self, force_val):
        """Writes the target force to be reached
        when gripping and holding a workpiece.
        It must be provided in 1/10th Newtons.
        The valid range is 0 to 400 for the RG2 and 0 to 1200 for the RG6.
        """
        result = self.client.write_register(
            address=0, value=force_val, unit=65)

    def set_target_width(self, width_val):
        """Writes the target width between
        the finger to be moved to and maintained.
        It must be provided in 1/10th millimeters.
        The valid range is 0 to 1100 for the RG2 and 0 to 1600 for the RG6.
        Please note that the target width should be provided
        corrected for any fingertip offset,
        as it is measured between the insides of the aluminum fingers.
        """
        result = self.client.write_register(
            address=1, value=width_val, unit=65)

    def close_gripper(self, force_val=400):
        """Closes gripper.

        write_registers 1번으로 force / width / control 3개 레지스터를 한꺼번에 쓴다.
        width=0 (완전 닫힘), control=16 (grip_w_offset).
        force_val 단위: 1/10 N (default 400 = 40 N).
        """
        params = [force_val, 0, 16]
        print("Start closing gripper.")
        result = self.client.write_registers(
            address=0, values=params, unit=65)

    def open_gripper(self, force_val=400):
        """Opens gripper.

        width=max_width (RG2: 400 = 40 mm), control=16 (grip_w_offset).
        force_val 단위: 1/10 N (default 400 = 40 N).
        """
        params = [force_val, self.max_width, 16]
        print("Start opening gripper.")
        result = self.client.write_registers(
            address=0, values=params, unit=65)

    def move_gripper(self, width_val, force_val=400):
        """Moves gripper to the specified width.

        width_val 단위: 1/10 mm.
        force_val 단위: 1/10 N (default 400 = 40 N).
        """
        params = [force_val, width_val, 16]
        print("Start moving gripper.")
        result = self.client.write_registers(
            address=0, values=params, unit=65)
