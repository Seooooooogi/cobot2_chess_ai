from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    """
    CV Chess Recognition Node Launch 파일
    
    사용 예시:
    ros2 launch cobot2 cv_chess_recognition.launch.py
    
    파라미터 오버라이드:
    ros2 launch cobot2 cv_chess_recognition.launch.py camera_source:=0
    """
    
    # 기본 경로 설정 (필요시 수정)
    home_dir = os.path.expanduser('~')
    default_yolo_path = os.path.join(home_dir, 'assembly_yolo11/runs/detect/chess_10k_result/weights/best.pt')
    default_resnet_path = os.path.join(home_dir, 'classifier.pt')
    default_grid_path = os.path.join(home_dir, 'chess_grid.json')
    
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'yolo_path',
            default_value=default_yolo_path,
            description='Path to YOLO model weights'
        ),
        
        DeclareLaunchArgument(
            'resnet_path',
            default_value=default_resnet_path,
            description='Path to ResNet classifier weights'
        ),
        
        DeclareLaunchArgument(
            'grid_path',
            default_value=default_grid_path,
            description='Path to chess grid JSON file'
        ),
        
        DeclareLaunchArgument(
            'camera_source',
            default_value='2',
            description='Camera device index or RTSP URL'
        ),
        
        DeclareLaunchArgument(
            'save_dir',
            default_value='./captured_boards',
            description='Directory to save captured images'
        ),
        
        DeclareLaunchArgument(
            'num_samples',
            default_value='5',
            description='Number of samples for majority voting'
        ),
        
        # CV Chess Recognition Node
        Node(
            package='cobot2',
            executable='cv_chess_recognition_node',
            name='cv_chess_recognition_node',
            output='screen',
            parameters=[{
                'yolo_path': LaunchConfiguration('yolo_path'),
                'resnet_path': LaunchConfiguration('resnet_path'),
                'grid_path': LaunchConfiguration('grid_path'),
                'camera_source': LaunchConfiguration('camera_source'),
                'save_dir': LaunchConfiguration('save_dir'),
                'num_samples': LaunchConfiguration('num_samples'),
            }]
        ),
    ])
