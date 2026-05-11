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
        """그리퍼와 Modbus TCP 연결을 연다."""
        self.client.connect()

    def close_connection(self):
        """그리퍼와 Modbus TCP 연결을 닫는다."""
        self.client.close()

    def get_fingertip_offset(self):
        """현재 fingertip offset 값을 mm 단위로 반환한다.

        레지스터(address=258)에 저장된 값은 signed two's complement 16-bit 정수이며,
        1/10 mm 단위 → mm 단위로 변환 후 반환한다.
        """
        result = self.client.read_holding_registers(
            address=258, count=1, unit=65)
        offset_mm = result.registers[0] / 10.0
        return offset_mm

    def get_width(self):
        """현재 그리퍼 손가락 간 간격(width)을 mm 단위로 반환한다.

        주의: fingertip offset이 적용되지 않은 알루미늄 손가락 내측 간 거리.
        offset 포함 값이 필요하면 ``get_width_with_offset()`` 사용.
        """
        result = self.client.read_holding_registers(
            address=267, count=1, unit=65)
        width_mm = result.registers[0] / 10.0
        return width_mm

    def get_status(self):
        """현재 그리퍼 상태(7개 플래그)를 읽어서 리스트로 반환한다.

        상태 레지스터(address=268)는 16-bit 비트 필드이며, 하위 7비트가 의미를 갖는다.
        아래 표는 OnRobot RG 매뉴얼 원문 그대로 (vendor reference):

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
        """현재 그리퍼 손가락 간 간격(width)을 fingertip offset 포함 mm 단위로 반환한다.

        ``get_width()`` 와 달리 fingertip offset이 적용된 값 (address=275).
        """
        result = self.client.read_holding_registers(
            address=275, count=1, unit=65)
        width_mm = result.registers[0] / 10.0
        return width_mm

    def set_control_mode(self, command):
        """control 레지스터(address=2)에 명령을 써서 그리퍼 동작을 제어한다.

        한 번에 하나의 옵션만 설정해야 하며, 이전 동작이 끝나지 않았다면
        (status의 busy=1) 새 명령은 무시된다. 유효 flag:

        - ``1`` (0x0001) ``grip``         : 현재 target force/width로 동작 시작.
                                            width는 fingertip offset 미포함으로 계산.
                                            busy=1이면 명령 무시됨.
        - ``8`` (0x0008) ``stop``         : 현재 동작 중지.
        - ``16`` (0x0010) ``grip_w_offset``: grip과 동일하나 width 계산에
                                            fingertip offset 반영.
        """
        result = self.client.write_register(
            address=2, value=command, unit=65)

    def set_target_force(self, force_val):
        """그리퍼가 물체를 잡을 때 도달/유지할 target force를 설정한다.

        단위: 1/10 N. 유효 범위: RG2는 0~400, RG6는 0~1200.
        """
        result = self.client.write_register(
            address=0, value=force_val, unit=65)

    def set_target_width(self, width_val):
        """그리퍼 손가락이 이동/유지할 target width를 설정한다.

        단위: 1/10 mm. 유효 범위: RG2는 0~1100, RG6는 0~1600.
        주의: 측정값은 알루미늄 손가락 내측 거리이므로,
        fingertip offset이 적용된 값을 직접 넘겨야 한다.
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
