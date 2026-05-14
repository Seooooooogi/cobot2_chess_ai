#!/usr/bin/env python3
"""OnRobot RG2/RG6 그리퍼 Modbus TCP 드라이버.

pymodbus로 OnRobot RG 시리즈 그리퍼와 직접 통신한다. ROS2 노드/서비스를 거치지
않는 순수 Modbus TCP 호출이므로 동기 호출 워크플로에 적합하다.

Modbus 레지스터 (slave unit ID = 65):
    0: target force (1/10 N)
    1: target width (1/10 mm)
    2: control command (1=grip, 8=stop, 16=grip with fingertip offset)
    258: fingertip offset (signed 1/10 mm)
    267: 현재 width (fingertip offset 미포함, 1/10 mm)
    268: status bit field (bit0=busy, bit1=grip detected, bit2~6=safety)
    275: 현재 width (fingertip offset 포함, 1/10 mm)

Note:
    force·width 레지스터는 모두 1/10 단위 정수다. 호출 측에서 mm·N으로 환산해
    전달할 책임을 진다.
"""

from pymodbus.client.sync import ModbusTcpClient as ModbusClient


class RG():
    """OnRobot RG2/RG6 그리퍼 Modbus TCP 클라이언트.

    생성과 동시에 ``open_connection()``으로 TCP 연결을 시도한다. 그리퍼 모델에
    따라 ``max_width``·``max_force`` 상한이 다르게 설정된다.

    Args:
        gripper (str): 'rg2' 또는 'rg6'. 다른 값이면 connection 시도 없이 즉시 종료.
        ip (str): 그리퍼 컨트롤러 IP.
        port (int): Modbus TCP 포트. OnRobot 기본값 502.

    Note:
        pymodbus 2.x는 connect 실패를 silent하게 삼킨다. 호출 측에서
        ``client.is_socket_open()``으로 재검증할 것.
    """

    def __init__(self, gripper, ip, port):
        self.client = ModbusClient(
            ip,
            port=port,
            stopbits=1,
            bytesize=8,
            parity='E',        # OnRobot 통신 규격: 짝수 parity 고정
            baudrate=115200,   # TCP이므로 무시되나 ModbusClient 시그니처 요구
            timeout=1)
        if gripper not in ['rg2', 'rg6']:
            print("Please specify either rg2 or rg6.")
            return
        self.gripper = gripper
        if self.gripper == 'rg2':
            self.max_width = 400   # 40.0 mm
            self.max_force = 400   # 40.0 N
        elif self.gripper == 'rg6':
            self.max_width = 1600  # 160.0 mm
            self.max_force = 1200  # 120.0 N
        self.open_connection()

    def open_connection(self):
        """Modbus TCP 연결을 연다."""
        self.client.connect()

    def close_connection(self):
        """Modbus TCP 연결을 닫는다."""
        self.client.close()

    def get_fingertip_offset(self):
        """Fingertip offset 레지스터(258) 값을 mm 단위로 반환한다.

        Returns:
            float: signed fingertip offset (mm).
        """
        result = self.client.read_holding_registers(
            address=258, count=1, unit=65)
        offset_mm = result.registers[0] / 10.0
        return offset_mm

    def get_width(self):
        """현재 손가락 간 width를 mm 단위로 반환한다.

        레지스터 267 (fingertip offset **미포함**) 값을 1/10 mm → mm로 환산한다.

        Returns:
            float: 알루미늄 손가락 내측 거리 (mm).

        Note:
            fingertip 두께를 반영한 값이 필요하면 ``get_width_with_offset()``을 쓴다.
        """
        result = self.client.read_holding_registers(
            address=267, count=1, unit=65)
        width_mm = result.registers[0] / 10.0
        return width_mm

    def get_status(self):
        """Status 비트 필드(268)를 7개 플래그 리스트로 풀어서 반환한다.

        하위 7비트의 각 비트가 set이면 매뉴얼 메시지를 stdout으로 출력한다
        (vendor 원형 보존). Bit 의미:

        - 0 (LSB) busy           : motion 진행 중. busy=1이면 새 명령 거부.
        - 1       grip detected  : internal/external grip 감지.
        - 2       S1 pushed      : safety switch 1 눌림.
        - 3       S1 triggered   : safety circuit 1 활성. 전원 재투입 전까지 lock.
        - 4       S2 pushed      : safety switch 2 눌림.
        - 5       S2 triggered   : safety circuit 2 활성. 전원 재투입 전까지 lock.
        - 6       safety error   : 전원 인가 시 safety switch 중 하나가 눌려 있음.

        Returns:
            list[int]: 길이 7, 각 슬롯은 0 또는 1.

        Warning:
            매 호출마다 ``print()``를 수행한다. tight polling loop에서 사용 시
            로그가 폭증할 수 있다.
        """
        result = self.client.read_holding_registers(
            address=268, count=1, unit=65)
        # 16-bit register → zero-padded binary string, LSB부터 비트 매핑
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
        """현재 width를 fingertip offset 포함하여 mm 단위로 반환한다.

        Returns:
            float: 레지스터 275 값을 환산한 width (mm).
        """
        result = self.client.read_holding_registers(
            address=275, count=1, unit=65)
        width_mm = result.registers[0] / 10.0
        return width_mm

    def set_control_mode(self, command):
        """Control 레지스터(2)에 명령을 써서 동작을 트리거한다.

        이전 동작이 끝나지 않은 상태(``status.busy=1``)에서 보낸 명령은 무시된다.

        Args:
            command (int): 한 번에 하나의 control flag만 지정한다.

                - ``1``  ``grip``         : 현재 target force/width로 동작 시작.
                  width 계산에 fingertip offset 미적용.
                - ``8``  ``stop``         : 현재 동작 정지.
                - ``16`` ``grip_w_offset``: ``grip``과 동일하되 width 계산에
                  fingertip offset 적용.
        """
        result = self.client.write_register(
            address=2, value=command, unit=65)

    def set_target_force(self, force_val):
        """Target force 레지스터(0)를 설정한다.

        Args:
            force_val (int): 1/10 N 단위. 유효 범위는 RG2 ``[0, 400]`` (≤40 N),
                RG6 ``[0, 1200]`` (≤120 N).
        """
        result = self.client.write_register(
            address=0, value=force_val, unit=65)

    def set_target_width(self, width_val):
        """Target width 레지스터(1)를 설정한다.

        Args:
            width_val (int): 1/10 mm 단위. 유효 범위는 RG2 ``[0, 400]`` (≤40 mm),
                RG6 ``[0, 1600]`` (≤160 mm).

        Note:
            본 값은 알루미늄 손가락 내측 거리 기준이다. fingertip 두께를 반영해야
            하면 호출 측에서 미리 보정해 넘긴다.
        """
        result = self.client.write_register(
            address=1, value=width_val, unit=65)

    def close_gripper(self, force_val=400):
        """Width=0으로 닫는다 (fingertip offset 적용 모드).

        레지스터 0/1/2를 한 번의 ``write_registers`` 호출로 갱신한다:
        force=force_val, width=0, control=16.

        Args:
            force_val (int): target force, 1/10 N 단위. 기본 400 (=40 N).
        """
        params = [force_val, 0, 16]
        print("Start closing gripper.")
        result = self.client.write_registers(
            address=0, values=params, unit=65)

    def open_gripper(self, force_val=400):
        """Width=``max_width``로 연다 (fingertip offset 적용 모드).

        Args:
            force_val (int): target force, 1/10 N 단위. 기본 400 (=40 N).
        """
        params = [force_val, self.max_width, 16]
        print("Start opening gripper.")
        result = self.client.write_registers(
            address=0, values=params, unit=65)

    def move_gripper(self, width_val, force_val=400):
        """지정한 width로 이동한다 (fingertip offset 적용 모드).

        Args:
            width_val (int): target width, 1/10 mm 단위.
            force_val (int): target force, 1/10 N 단위. 기본 400 (=40 N).
        """
        params = [force_val, width_val, 16]
        print("Start moving gripper.")
        result = self.client.write_registers(
            address=0, values=params, unit=65)
