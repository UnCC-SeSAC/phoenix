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
from launch_ros.parameter_descriptions import ParameterValue


def include_launch(path, condition=None, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        condition=condition,
        launch_arguments=(launch_arguments or {}).items(),
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

    image_pipeline_share = get_package_share_directory(
        'image_pipeline'
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

    start_mission = LaunchConfiguration(
        'start_mission'
    )

    start_vision = LaunchConfiguration(
        'start_vision'
    )

    vision_model_path = LaunchConfiguration('vision_model_path')
    vision_class_names = LaunchConfiguration('vision_class_names')
    vision_layout = LaunchConfiguration('vision_layout')

    return_to_start_on_complete = LaunchConfiguration(
        'return_to_start_on_complete'
    )

    goal_preemption_enabled = LaunchConfiguration(
        'goal_preemption_enabled'
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
                        # avoidance_manager에서
                        # STOP / START를 호출하기 위해 필수
                        'control_service_enabled': True,

                        'autostart': True,

                        # Raspberry Pi 5에서는
                        # 먼저 가볍게 시작
                        'mrtsp_solver': 'greedy',

                        'map_processing_rate_hz': 0.5,

                        # 처음에는 기능을 단순하게
                        'goal_preemption_enabled': ParameterValue(
                            goal_preemption_enabled,
                            value_type=bool,
                        ),

                        'return_to_start_on_complete': ParameterValue(
                            return_to_start_on_complete,
                            value_type=bool,
                        ),
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
                        'trigger_distance': 0.30,

                        'clear_distance': 0.35,

                        'front_angle_deg': 210.0,

                        'clear_hold_sec': 0.60,

                        'avoidance_timeout_sec': 5.0,
                    }
                ],
            )
        ],
    )

    # =========================================
    # 7. Production Camera + image_pipeline
    #    Test-only full_chain_check.launch.py is intentionally excluded:
    #    it starts fake_detection_node and the stub YOLO backend.
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
            include_launch(
                os.path.join(
                    image_pipeline_share,
                    'launch',
                    'preprocess.launch.py',
                ),
                IfCondition(start_vision),
            ),
            include_launch(
                os.path.join(
                    image_pipeline_share,
                    'launch',
                    'yolo.launch.py',
                ),
                IfCondition(start_vision),
                {
                    'model_path': vision_model_path,
                    'class_names': vision_class_names,
                    'layout': vision_layout,
                },
            ),
            include_launch(
                os.path.join(
                    image_pipeline_share,
                    'launch',
                    'detection_3d.launch.py',
                ),
                IfCondition(start_vision),
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
            default_value='false',
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
            default_value='false',
        ),

        DeclareLaunchArgument(
            'start_mission',
            # Frontier is the default deterministic goal owner. MissionExecutor
            # must be enabled explicitly and never together with Frontier/VLA.
            default_value='false',
        ),

        DeclareLaunchArgument(
            'start_vision',
            default_value='false',
        ),

        DeclareLaunchArgument(
            'vision_model_path',
            default_value='',
            description='Production YOLO model path; required when start_vision=true',
        ),

        DeclareLaunchArgument(
            'vision_class_names',
            default_value="['fire', 'person']",
            description='Phoenix production model order: fire=0, person=1',
        ),

        DeclareLaunchArgument(
            'vision_layout',
            default_value='auto',
            description='Production model output layout; pin after backend verification',
        ),

        DeclareLaunchArgument(
            'return_to_start_on_complete',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'goal_preemption_enabled',
            default_value='false',
        ),

        hardware,
        lidar_app,
        slam,
        nav2,
        frontier,
        avoidance,
        vision,
        mission,
    ])
