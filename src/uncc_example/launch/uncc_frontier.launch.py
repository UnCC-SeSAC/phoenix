import os

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)

from launch.conditions import IfCondition

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch.substitutions import (
    LaunchConfiguration,
)

from launch_ros.actions import Node


def include_launch(
    path,
    condition=None,
):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        condition=condition,
    )


def generate_launch_description():

    # -----------------------------------------
    # Package paths
    # -----------------------------------------

    uncc_share = get_package_share_directory(
        'uncc_example'
    )

    frontier_share = get_package_share_directory(
        'frontier_exploration_ros2'
    )

    peripherals_share = get_package_share_directory(
        'peripherals'
    )

    launch_dir = os.path.join(
        uncc_share,
        'launch',
    )

    frontier_params = os.path.join(
        frontier_share,
        'config',
        'params.yaml',
    )

    # -----------------------------------------
    # Arguments
    # -----------------------------------------

    start_hardware = LaunchConfiguration(
        'start_hardware'
    )

    start_lidar_app = LaunchConfiguration(
        'start_lidar_app'
    )

    start_slam = LaunchConfiguration(
        'start_slam'
    )

    start_nav2 = LaunchConfiguration(
        'start_nav2'
    )

    start_frontier = LaunchConfiguration(
        'start_frontier'
    )

    start_avoidance = LaunchConfiguration(
        'start_avoidance'
    )

    start_fire_suppression = LaunchConfiguration(
        'start_fire_suppression'
    )

    start_mission = LaunchConfiguration(
        'start_mission'
    )

    start_vision = LaunchConfiguration(
        'start_vision'
    )

    # =========================================
    # 1. Hardware
    # =========================================

    hardware = include_launch(
        os.path.join(
            launch_dir,
            'hardware.launch.py',
        ),
        IfCondition(start_hardware),
    )

    # =========================================
    # 2. Hiwonder lidar_controller
    # =========================================

    lidar_app = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='app',
                executable='lidar_controller',
                output='screen',
                condition=IfCondition(
                    start_lidar_app
                ),
            )
        ],
    )

    # =========================================
    # 3. SLAM
    # =========================================

    slam = TimerAction(
        period=3.0,
        actions=[
            include_launch(
                os.path.join(
                    launch_dir,
                    'slam_mapping.launch.py',
                ),
                IfCondition(start_slam),
            )
        ],
    )

    # =========================================
    # 4. Nav2
    # =========================================

    nav2 = TimerAction(
        period=7.0,
        actions=[
            include_launch(
                os.path.join(
                    launch_dir,
                    'nav2_online.launch.py',
                ),
                IfCondition(start_nav2),
            )
        ],
    )

    # =========================================
    # 5. Frontier Explorer
    # =========================================

    frontier = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='frontier_exploration_ros2',
                executable='frontier_explorer',

                name='frontier_explorer',

                output='screen',

                condition=IfCondition(
                    start_frontier
                ),

                parameters=[
                    frontier_params,

                    {
                        # avoidance_manager / fire_suppression_node에서
                        # STOP / START를 호출하기 위해 필수
                        'control_service_enabled': True,

                        'autostart': True,

                        # Raspberry Pi 5에서는
                        # 먼저 가볍게 시작
                        'mrtsp_solver': 'greedy',

                        'map_processing_rate_hz': 0.5,

                        # 처음에는 기능을 단순하게
                        'goal_preemption_enabled': False,

                        'return_to_start_on_complete': False,
                    },
                ],
            )
        ],
    )

    # =========================================
    # 6. Avoidance Manager
    # =========================================

    avoidance = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='uncc_example',

                executable='avoidance_manager',

                name='avoidance_manager',

                output='screen',

                condition=IfCondition(
                    start_avoidance
                ),

                parameters=[
                    {
                        'trigger_distance': 0.50,

                        'clear_distance': 0.75,

                        'front_angle_deg': 90.0,

                        'clear_hold_sec': 0.60,

                        'avoidance_timeout_sec': 5.0,
                    }
                ],
            )
        ],
    )

    # =========================================
    # 6.5. Fire Suppression
    #    (fire_status_service_node + fire_suppression_node +
    #    fire_extinguisher 세 노드를 같이 띄운다. mission_manager가
    #    /fire_extinguisher/extinguish 를 호출하므로 mission_manager
    #    (t=13)보다 반드시 먼저 떠 있어야 한다. frontier(t=11) /
    #    avoidance(t=12) 다음, control_exploration 서비스가 이미
    #    준비된 뒤에 뜨도록 배치했다.)
    # =========================================

    fire_suppression = TimerAction(
        period=12.5,
        actions=[
            Node(
                package='uncc_example',

                executable='fire_status_service_node',

                name='fire_status_service_node',

                output='screen',

                condition=IfCondition(
                    start_fire_suppression
                ),
            ),

            Node(
                package='uncc_example',

                executable='fire_suppression_node',

                name='fire_suppression_node',

                output='screen',

                condition=IfCondition(
                    start_fire_suppression
                ),
            ),

            Node(
                package='uncc_example',

                executable='fire_extinguisher',

                name='fire_extinguisher',

                output='screen',

                condition=IfCondition(
                    start_fire_suppression
                ),
            ),
        ],
    )

    # =========================================
    # 7. Depth Camera + yolo_detector
    #    (yolo_detector 는 실제 YOLO 모델 없이 파이프라인 테스트용
    #    더미 감지만 발행하는 임시 노드 — 나중에 다른 사람이 작성한
    #    코드로 교체될 예정이라 mission_manager 와 별도 프로세스로
    #    유지한다)
    # =========================================

    vision = TimerAction(
        period=4.0,
        actions=[
            include_launch(
                os.path.join(
                    peripherals_share,
                    'launch',
                    'depth_camera.launch.py',
                ),
                IfCondition(start_vision),
            ),

            Node(
                package='uncc_example',

                executable='yolo_detector',

                name='yolo_detector',

                output='screen',

                condition=IfCondition(
                    start_vision
                ),
            ),
        ],
    )

    # =========================================
    # 8. Mission Manager
    #    (vision_detector + state_manager + mission_executor 를
    #    한 프로세스에서 같이 돌린다 — RAM 절약 목적. start_vision
    #    이 false 여도 vision_detector 는 이 안에서 같이 뜨지만,
    #    depth 카메라 하드웨어 자체가 안 켜져있으면 그냥 데이터
    #    없이 대기만 한다)
    # =========================================

    mission = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='uncc_example',

                executable='mission_manager',

                name='mission_manager',

                output='screen',

                condition=IfCondition(
                    start_mission
                ),
            ),
        ],
    )

    # =========================================
    # LaunchDescription
    # =========================================

    return LaunchDescription([

        DeclareLaunchArgument(
            'start_hardware',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'start_lidar_app',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'start_slam',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'start_nav2',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'start_frontier',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'start_avoidance',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'start_fire_suppression',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'start_mission',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'start_vision',
            default_value='false',
        ),

        hardware,
        lidar_app,
        slam,
        nav2,
        frontier,
        avoidance,
        fire_suppression,
        vision,
        mission,
    ])
