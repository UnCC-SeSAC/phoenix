from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mock_vision = LaunchConfiguration("mock_vision")
    mock_slam = LaunchConfiguration("mock_slam")
    mock_status_demo = LaunchConfiguration("mock_status_demo")

    return LaunchDescription([
        # frontier_fire_suppression_hw_test.launch.py와 동일한 기본값 —
        # 런치하자마자 다른 PC에서 http://<이 기기 IP>:8080으로 바로
        # 붙는다. 이 기기에서만 열려면 ui_host:=127.0.0.1,
        # ui_allow_remote:=false로 덮어쓸 것 (LAN에 START/STOP 미션
        # 제어까지 열리는 게 싫으면 반드시 그렇게 할 것).
        DeclareLaunchArgument("ui_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("ui_port", default_value="8080"),
        DeclareLaunchArgument("ui_allow_remote", default_value="true"),
        DeclareLaunchArgument("mock_vision", default_value="true"),
        DeclareLaunchArgument("mock_slam", default_value="true"),
        # true면 vla_orchestrator 대신 person/fire 7가지 상태를 한 번에 박아
        # 넣는 정적 목업(vla_status_demo)이 /vla/status를 낸다 — 시맨틱 맵
        # 마커(모양/상태별 색)만 확인할 때 쓴다. 같은 토픽에 둘이 동시에
        # 발행하면 화면이 깜빡이므로 서로 배타적으로 뜬다.
        DeclareLaunchArgument("mock_status_demo", default_value="false"),

        Node(
            package="fire_vla_core",
            executable="vla_orchestrator",
            name="vla_orchestrator",
            output="screen",
            condition=UnlessCondition(mock_status_demo),
            parameters=[{
                "llm_backend": "mock",
                "decision_period_sec": 1.0,
                "navigation_mode": "MOCK",
                "report_mode": "MOCK",
                "spray_mode": "MOCK",
                "status_topic": "/vla/status",
            }],
        ),
        Node(
            package="fire_vla_core",
            executable="vla_status_demo",
            name="vla_status_demo",
            output="screen",
            condition=IfCondition(mock_status_demo),
            parameters=[{"status_topic": "/vla/status"}],
        ),

        # --- 목업 영상 체인 ---------------------------------------------
        # fake_detection_node가 합성 컬러 프레임과 /yolo_result를 **같은 stamp**로
        # 냅니다. 새 목업 카메라를 만들지 않고 이걸 그대로 재료로 씁니다.
        Node(
            package="image_pipeline",
            executable="fake_detection_node",
            name="fake_detection_node",
            output="screen",
            condition=IfCondition(mock_vision),
            parameters=[{"publish_color": True, "class_id": "fire", "fps": 15.0}],
        ),
        Node(
            package="image_pipeline",
            executable="preprocess_node",
            name="preprocess_node",
            output="screen",
            condition=IfCondition(mock_vision),
            # camera_info는 fake_detection_node가 이미 냅니다. 여기서도 내면
            # 같은 토픽에 발행자가 둘이 됩니다.
            parameters=[{"mode": "passthrough", "publish_camera_info": False}],
        ),
        Node(
            package="image_pipeline",
            executable="ui_stream_node",
            name="ui_stream_node",
            output="screen",
            condition=IfCondition(mock_vision),
            parameters=[{
                "class_names": ["fire", "person"],
                "stream_fps": 8.0,
                "stream_max_width": 640,
                "jpeg_quality": 70,
            }],
        ),

        # --- 목업 SLAM ---------------------------------------------------
        Node(
            package="fire_vla_core",
            executable="vla_mock_slam",
            name="vla_mock_slam",
            output="screen",
            condition=IfCondition(mock_slam),
        ),

        Node(
            package="fire_vla_core",
            executable="firefighter_ui",
            name="firefighter_ui",
            output="screen",
            parameters=[{
                "ui_host": LaunchConfiguration("ui_host"),
                "ui_port": ParameterValue(
                    LaunchConfiguration("ui_port"), value_type=int
                ),
                "ui_allow_remote": ParameterValue(
                    LaunchConfiguration("ui_allow_remote"), value_type=bool
                ),
                "status_topic": "/vla/status",
                "mission_topic": "/vla/mission",
            }],
        ),
    ])