from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    """
    Voice Control Node Launch 파일
    
    사용 예시:
    ros2 launch cobot2 voice_control.launch.py
    
    파라미터 오버라이드:
    ros2 launch cobot2 voice_control.launch.py mic_device_index:=10
    """
    
    # 기본 경로 설정
    home_dir = os.path.expanduser('~')
    default_model_path = os.path.join(
        home_dir, 
        'ros2_ws/src/corobot2_project/corobot2_project/hello_rokey_8332_32.tflite'
    )
    
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'wakeword_model_path',
            default_value=default_model_path,
            description='Path to wakeword detection model (.tflite)'
        ),
        
        DeclareLaunchArgument(
            'wakeword_model_name',
            default_value='hello_rokey_8332_32',
            description='Name of the wakeword model'
        ),
        
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.4',
            description='Confidence threshold for wakeword detection (0.0-1.0)'
        ),
        
        DeclareLaunchArgument(
            'record_seconds',
            default_value='3',
            description='Duration to record voice command after wakeword (seconds)'
        ),
        
        DeclareLaunchArgument(
            'mic_device_index',
            default_value='2',
            description='Microphone device index (use arecord -l to find)'
        ),
        
        # Voice Control Node
        Node(
            package='cobot2',
            executable='voice_control_node',
            name='voice_control_node',
            output='screen',
            parameters=[{
                'wakeword_model_path': LaunchConfiguration('wakeword_model_path'),
                'wakeword_model_name': LaunchConfiguration('wakeword_model_name'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'record_seconds': LaunchConfiguration('record_seconds'),
                'mic_device_index': LaunchConfiguration('mic_device_index'),
            }]
        ),
    ])
