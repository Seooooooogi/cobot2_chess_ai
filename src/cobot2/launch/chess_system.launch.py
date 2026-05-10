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
    1. Stockfish AI 노드 (ROS2 parameter로 depth/skill_level/default_turn 관리)
    2. CV 체스판 인식 노드 (ROS2 publisher: /vision/board_state)
    3. 로봇 제어 노드 (action server: /move_chess_piece)
    4. 통합 조정 노드 (FSM, GameEvent publisher, UIStatus publisher, UserDecision Service)
    5. 게임 로거 노드 (SQLite append-only audit log — Phase 5 sub-phase E)
    6. rosbridge websocket — Web UI ↔ ROS2 (Phase 5 sub-phase C)

    사용 예시:
    ros2 launch cobot2 chess_system.launch.py
    """

    # 노드명은 각 노드의 ``super().__init__(...)`` 코드 값을 single source of truth로
    # 사용한다. launch ``name=`` 오버라이드는 의도적으로 제거 (Phase 6-0 baseline 발견
    # PB-1~3 RESOLVED, 2026-05-10):
    #   - chess_ai_node (stockfish)
    #   - vision_db
    #   - robot_action_server
    #   - main_controller
    #   - game_logger
    # ``~/topic`` 사설 네임스페이스 + 토픽/서비스 화이트리스트가 모두 코드 노드명에
    # 정렬되도록 유지. 변경 시 docstring + UI.html + game_logger 구독 경로도 동기화.
    return LaunchDescription([
        # 1. Stockfish AI Node → /chess_ai_node
        Node(
            package='cobot2',
            executable='stockfish',
            output='screen',
            respawn=True,
        ),

        # 2. CV Chess Recognition Node → /vision_db
        #    publisher: /vision/board_state (절대 경로 — namespace-relative).
        Node(
            package='cobot2',
            executable='object',
            output='screen',
            respawn=True,
        ),

        # 3. Robot Control Action Server → /robot_action_server
        #    ActionServer: /move_chess_piece (절대 경로). reset Service: /robot_action_server/reset.
        Node(
            package='cobot2',
            executable='robotaction',
            output='screen',
            respawn=True,
        ),

        # 4. Chess Integration Node → /main_controller
        #    Publishers: ~/ui_status, ~/game_event (사설 네임스페이스).
        #    Services:   ~/start_sampling, ~/user_decision.
        Node(
            package='cobot2',
            executable='main',
            output='screen',
            respawn=True,
        ),

        # 5. Game Logger → /game_logger
        # 구독: /main_controller/game_event + /main_controller/ui_status + /vision/board_state.
        # DB: env CHESS_AI_LOG_DB_PATH > 기본 ~/.local/share/cobot2_chess_ai/game_log.db.
        # respawn=True — DB write 실패 시 노드는 ERROR 로그만 남기고 계속 동작하지만
        # startup 실패 (권한 등) 시 launch가 자동 재시작.
        Node(
            package='cobot2',
            executable='gamelogger',
            output='screen',
            respawn=True,
        ),

        # 6. rosbridge WebSocket bridge (Phase 5 sub-phase C)
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
