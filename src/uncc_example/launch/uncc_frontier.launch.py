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

    uncc_share = get_package_share_directory("uncc_example")

    frontier_share = get_package_share_directory("frontier_exploration_ros2")

    launch_dir = os.path.join(
        uncc_share,
        "launch",
    )

    frontier_params = os.path.join(
        frontier_share,
        "config",
        "params.yaml",
    )

    # -----------------------------------------
    # Arguments
    # -----------------------------------------

    start_hardware = LaunchConfiguration("start_hardware")

    start_lidar_app = LaunchConfiguration("start_lidar_app")

    start_slam = LaunchConfiguration("start_slam")

    start_nav2 = LaunchConfiguration("start_nav2")

    start_frontier = LaunchConfiguration("start_frontier")

    start_avoidance = LaunchConfiguration("start_avoidance")

    return_to_start_on_complete = LaunchConfiguration("return_to_start_on_complete")
    goal_preemption_enabled = LaunchConfiguration("goal_preemption_enabled")
    # =========================================
    # 1. Hardware
    # =========================================

    hardware = include_launch(
        os.path.join(
            launch_dir,
            "hardware.launch.py",
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
                package="app",
                executable="lidar_controller",
                output="screen",
                condition=IfCondition(start_lidar_app),
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
                    "slam_mapping.launch.py",
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
                    "nav2_online.launch.py",
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
                package="frontier_exploration_ros2",
                executable="frontier_explorer",
                name="frontier_explorer",
                output="screen",
                condition=IfCondition(start_frontier),
                parameters=[
                    frontier_params,
                    {
                        # avoidance_manager에서
                        # STOP / START를 호출하기 위해 필수
                        "control_service_enabled": True,
                        "autostart": True,
                        # Raspberry Pi 5에서는
                        # 먼저 가볍게 시작
                        "mrtsp_solver": "greedy",
                        "map_processing_rate_hz": 0.5,
                        # 처음에는 기능을 단순하게
                        "goal_preemption_enabled": ParameterValue(
                            goal_preemption_enabled, value_type=bool
                        ),
                        "return_to_start_on_complete": ParameterValue(
                            return_to_start_on_complete, value_type=bool
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
                package="uncc_example",
                executable="avoidance_manager",
                name="avoidance_manager",
                output="screen",
                condition=IfCondition(start_avoidance),
                parameters=[
                    {
                        "trigger_distance": 0.30,
                        "clear_distance": 0.35,
                        "front_angle_deg": 210.0,
                        "clear_hold_sec": 0.60,
                        "avoidance_timeout_sec": 5.0,
                    }
                ],
            )
        ],
    )

    # =========================================
    # LaunchDescription
    # =========================================

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_hardware",
                default_value="true",
            ),
            # lidar 기반 회피 비활성화 false
            DeclareLaunchArgument(
                "start_lidar_app",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "start_slam",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "start_nav2",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "start_frontier",
                default_value="true",
            ),
            # avoidance manager 비활성화 false
            DeclareLaunchArgument(
                "start_avoidance",
                default_value="false",
            ),
            # 모든 탐사가 종료 되었을 때 시작지점 복귀
            DeclareLaunchArgument("return_to_start_on_complete", default_value="true"),
            # frontier 선점
            DeclareLaunchArgument("goal_preemption_enabled", default_value="false"),
            hardware,
            lidar_app,
            slam,
            nav2,
            frontier,
            avoidance,
        ]
    )
