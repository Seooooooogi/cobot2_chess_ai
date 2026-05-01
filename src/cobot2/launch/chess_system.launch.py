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
    2. CV 체스판 인식 노드
    3. 로봇 제어 노드
    4. 음성 인식 노드
    5. 통합 조정 노드 (Firebase 리스너)
    
    사용 예시:
    ros2 launch cobot2 chess_system.launch.py
    """
    
    return LaunchDescription([
        # 1. Stockfish AI Node
        Node(
            package='cobot2',
            executable='stockfish_node',
            name='stockfish_node',
            output='screen',
            respawn=True,
        ),
        
        # 2. CV Chess Recognition Node
        Node(
            package='cobot2',
            executable='cv_chess_recognition_node',
            name='cv_chess_recognition_node',
            output='screen',
            respawn=True,
            parameters=[{
                'camera_source': 2,
                'num_samples': 5,
            }]
        ),
        
        # 3. Robot Control Action Server
        Node(
            package='cobot2',
            executable='moving_chess_piece_node',
            name='moving_chess_piece_node',
            output='screen',
            respawn=True,
        ),
        
        # 4. Voice Control Node
        Node(
            package='cobot2',
            executable='voice_control_node',
            name='voice_control_node',
            output='screen',
            respawn=True,
            parameters=[{
                'confidence_threshold': 0.4,
                'mic_device_index': 2,
            }]
        ),
        
        # 5. Chess Integration Node (Firebase Listener & Coordinator)
        Node(
            package='cobot2',
            executable='chess_integration_node',
            name='chess_integration_node',
            output='screen',
            respawn=True,
        ),
    ])
