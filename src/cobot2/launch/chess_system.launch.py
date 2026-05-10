from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    """
    COBOT2 Chess System - 전체 시스템 Launch 파일

    실행되는 노드:
    1. Stockfish AI 노드
    2. CV 체스판 인식 노드 (ROS2 publisher: /vision/board_state)
    3. 로봇 제어 노드
    4. 통합 조정 노드 (board_state subscriber + Firebase ui_control listener)
    5. rosbridge websocket — Web UI ↔ ROS2 (Phase 5 sub-phase C)

    사용 예시:
    ros2 launch cobot2 chess_system.launch.py
    """

    return LaunchDescription([
        # 1. Stockfish AI Node
        Node(
            package='cobot2',
            executable='stockfish',
            name='stockfish_node',
            output='screen',
            respawn=True,
        ),

        # 2. CV Chess Recognition Node
        Node(
            package='cobot2',
            executable='object',
            name='cv_chess_recognition_node',
            output='screen',
            respawn=True,
        ),

        # 3. Robot Control Action Server
        Node(
            package='cobot2',
            executable='robotaction',
            name='moving_chess_piece_node',
            output='screen',
            respawn=True,
        ),

        # 4. Chess Integration Node
        Node(
            package='cobot2',
            executable='main',
            name='chess_integration_node',
            output='screen',
            respawn=True,
        ),

        # 5. rosbridge WebSocket bridge (Phase 5 sub-phase C)
        # Default bind 0.0.0.0:9090. ADR-002 LAN-only 가정 — 외부 노출 시 nginx + WSS + auth 별도.
        # Rule 9: 화이트리스트로 노출 표면 제한. motion control action server (move_chess_piece) 등
        # 안전 관련 인터페이스는 LAN 클라이언트로부터 차단. sub-phase D에서 ui_control 서비스를
        # ROS2로 마이그레이션할 때 services_glob에 /main_controller/* 등 명시적으로 추가.
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            respawn=True,
            parameters=[{
                'port': 9090,
                # Phase 5 sub-phase D1: UI가 /main_controller/ui_status 토픽 구독
                'topics_glob': '[/vision/*, /main_controller/ui_status]',
                # Phase 5 sub-phase D2/D3: UI가 user_decision Service + stockfish parameter
                # 표준 service 5종 (get/set/list/describe + get_parameter_types) 호출.
                # Rule 9 정밀 노출 — 와일드카드 대신 명시.
                # 의도적 미포함:
                #   - /chess_ai_node/reset_chess_state — 웹 UI 게임 리셋 노출 차단.
                #     향후 D4/E에서 reset 버튼 추가 시 이 list에 명시 필요.
                #   - /move_chess_piece, /dsr01/* — motion 인터페이스, Rule 9.
                'services_glob': (
                    '[/rosapi/*, /main_controller/user_decision, '
                    '/chess_ai_node/get_parameters, /chess_ai_node/set_parameters, '
                    '/chess_ai_node/list_parameters, /chess_ai_node/describe_parameters, '
                    '/chess_ai_node/get_parameter_types]'
                ),
                'actions_glob': '[]',
            }],
        ),
    ])
