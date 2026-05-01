import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor

import DR_init
import time
from .onrobot import RG

import json
import os

from datetime import datetime

from cobot2_interfaces.action import MoveChessPiece # 커스텀 액션 임포트

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1_1"

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
JSON_PATH = os.path.join(BASE_DIR, "data.json")

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

class MovingChessPiece:
    def __init__(self, logger_node: Node):
        self.logger_node = logger_node

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

        self.load_initial_config()
        self.calculate()    

    def log(self, msg: str):
        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        full = f"[{now}] {msg}"
        self.logger_node.get_logger().info(full)
    
    def load_initial_config(self):
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

            self.log("JSON sync done")
        except Exception as e:
            self.log(f"JSON load error: {e}")

    def grip(self): # 10mm
        gripper.close_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.25)
    
    # (0,1):50mm

    def release(self): # 35mm
        gripper.open_gripper()
        while gripper.get_status()[0]:
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
    def __init__(self):
        super().__init__('robot_action_server')
        
        # 로직 클래스 초기화
        self.chess_mover = MovingChessPiece(self)
        
        # 액션 서버 설정
        self._action_server = ActionServer(
            self,
            MoveChessPiece,
            'move_chess_piece',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info(f"Robot Action Server started for {ROBOT_ID} ({ROBOT_MODEL})")

    def goal_callback(self, goal_request):
        """액션 목표 수락 여부 결정"""
        self.get_logger().info(f"Received goal request: {goal_request.command}")
        # 간단한 유효성 검사 (예: 문자열 길이 등)를 추가할 수 있습니다.
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """액션 취소 요청 처리"""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """실제 로봇 동작 수행"""
        self.get_logger().info("Executing goal...")
        
        result = MoveChessPiece.Result()
        feedback_msg = MoveChessPiece.Feedback()
        
        try:
            # goal_handle을 통째로 넘겨줍니다.
            self.chess_mover.perform_task(goal_handle)
            
            goal_handle.succeed()
            result.success = True
        except Exception as e:
            self.get_logger().error(f"Task failed: {e}")
            goal_handle.abort()
            result.success = False
        
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