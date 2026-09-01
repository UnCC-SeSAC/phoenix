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
    PythonExpression,
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

    uncc_share = get_package_share_directory("uncc_example")

    frontier_share = get_package_share_directory("frontier_exploration_ros2")

    image_pipeline_share = get_package_share_directory("image_pipeline")

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

    start_fire_suppression = LaunchConfiguration("start_fire_suppression")

    # true 로 주면 fire_suppression_node 대신 로컬 테스트용 더미
    # (fire_suppression_node_dummy_stub) 를 띄운다. GPIO/YOLO/frontier
    # 의존성 없이 mission_executor 와의 액션 통신만 확인하고 싶을 때 사용.
    use_dummy_fire_suppression = LaunchConfiguration("use_dummy_fire_suppression")

    start_mission = LaunchConfiguration("start_mission")

    start_vision = LaunchConfiguration("start_vision")

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
                        # avoidance_manager / fire_suppression_node에서
                        # STOP / START를 호출하기 위해 필수
                        "control_service_enabled": True,
                        # Frontier Controller 로 진행하기 때문에 False 로
                        "autostart": False,
                        # Raspberry Pi 5에서는
                        # 먼저 가볍게 시작
                        "mrtsp_solver": "greedy",
                        "map_processing_rate_hz": 0.5,
                        # 처음에는 기능을 단순하게
                        "goal_preemption_enabled": False,
                        # 탐사 완료 후, 시작지점 복귀 True
                        "return_to_start_on_complete": True,
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
                        "trigger_distance": 0.50,
                        "clear_distance": 0.75,
                        "front_angle_deg": 90.0,
                        "clear_hold_sec": 0.60,
                        "avoidance_timeout_sec": 5.0,
                    }
                ],
            )
        ],
    )

    # =========================================
    # 5.5. Frontier State Controller
    # =========================================

    frontier_state_controller = TimerAction(
        period=12.0,
        actions=[
            Node(
                package="uncc_example",
                executable="frontier_state_controller",
                name="frontier_state_controller",
                output="screen",
                condition=IfCondition(start_mission),
                parameters=[
                    {
                        "frontier_control_service": "/control_exploration",
                        "stop_timeout_sec": 5.0,
                    }
                ],
            )
        ],
    )

    # =========================================
    # 6.5. Fire Suppression
    #    (fire_status_service_node + fire_suppression_node
    #    두 노드를 같이 띄운다. mission_manager가 SuppressFire
    #    Action을 호출하므로 mission_manager(t=13)보다
    #    반드시 먼저 떠 있어야 한다. frontier(t=11) /
    #    avoidance(t=12) 다음, control_exploration 서비스가 이미
    #    준비된 뒤에 뜨도록 배치했다.)
    # =========================================

    fire_suppression = TimerAction(
        period=12.5,
        actions=[
            Node(
                package="uncc_example",
                executable="fire_status_service_node",
                name="fire_status_service_node",
                output="screen",
                condition=IfCondition(start_fire_suppression),
            ),
            Node(
                package="uncc_example",
                executable="fire_suppression_node",
                name="fire_suppression_node",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            '"',
                            start_fire_suppression,
                            '" == "true" and "',
                            use_dummy_fire_suppression,
                            '" == "false"',
                        ]
                    )
                ),
            ),
            Node(
                package="uncc_example",
                executable="fire_suppression_node_dummy_stub",
                name="fire_suppression_node",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            '"',
                            start_fire_suppression,
                            '" == "true" and "',
                            use_dummy_fire_suppression,
                            '" == "true"',
                        ]
                    )
                ),
            ),
        ],
    )

    # =========================================
    # 7. Vision (image_pipeline)
    #    실제 모델(.onnx) 준비 전까지 full_chain_check.launch.py(더미
    #    체인)를 씀. depth_camera.launch.py 와 같이 켜면 토픽이
    #    겹치니 지금은 빼둠 — 모델 준비되면 진짜 카메라+체인으로 교체.
    # =========================================

    vision = TimerAction(
        period=4.0,
        actions=[
            include_launch(
                image_pipeline_share + "/launch/full_chain_check.launch.py",
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
    #
    #    state_manager 는 STANDBY 로 시작해서 launch 가 끝나도 자동으로
    #    탐사를 시작하지 않는다. 카메라/YOLO/서보모터 등 모든 노드가
    #    bringup 된 걸 확인한 뒤 아래 명령으로 직접 신호를 줘야 한다:
    #    ros2 service call /state_manager/start_mission std_srvs/srv/Trigger "{}"
    # =========================================

    mission = TimerAction(
        period=13.0,
        actions=[
            Node(
                package="uncc_example",
                executable="mission_manager",
                name="mission_manager",
                output="screen",
                condition=IfCondition(start_mission),
            ),
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
            DeclareLaunchArgument(
                "start_lidar_app",
                default_value="false",  # 라이다 회피 기능 사용 안함 Local Planner 로 동작
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
            DeclareLaunchArgument(
                "start_avoidance",
                default_value="false",  # 사용 안함 Local Planner 로만 회피
            ),
            DeclareLaunchArgument(
                "start_fire_suppression",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "use_dummy_fire_suppression",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "start_mission",
                default_value="true",
            ),
            # 기본 false. 켜지면 (지금은 image_pipeline 더미 체인이라)
            # 가짜 감지로도 실제 Nav2 이동 + 실제 fire_suppression 펌프
            # 분사까지 이어질 수 있어서, 의도적으로 켤 때만
            # start_vision:=true 로 명시하게 한다.
            DeclareLaunchArgument(
                "start_vision",
                default_value="false",
            ),
            hardware,
            lidar_app,
            slam,
            nav2,
            frontier,
            avoidance,
            frontier_state_controller,
            fire_suppression,
            vision,
            mission,
        ]
    )
