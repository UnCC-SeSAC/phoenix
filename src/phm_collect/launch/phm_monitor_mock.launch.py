#!/usr/bin/env python3
"""센서 없는 기기에서 PHM 전체 경로를 확인합니다.

수집한 JSONL 을 `phm_mock_source` 가 진짜 ROS 토픽으로 틀고, `phm_monitor` 가 그걸
구독해 `/phm/status` 를 냅니다. `firefighter_ui` 를 따로 띄우면 브라우저까지 이어집니다.

    ros2 launch phm_collect phm_monitor_mock.launch.py jsonl:=/shared/live25_lift.jsonl
    ros2 launch fire_vla_bringup firefighter_ui_mock.launch.py      # 다른 터미널

★ rf2o 는 띄우지 않습니다
    목업이 `/odom_rf2o` 를 직접 내기 때문입니다. rf2o 를 같이 띄우면 발행자가 둘이
    되어 값이 섞입니다. (라이다가 없으면 rf2o 는 어차피 아무것도 못 냅니다.)

무엇을 확인할 수 있나 / 없나
    확인됨   colcon 빌드(aarch64/Humble), QoS 협상, DDS 왕복, /api/phm, 브라우저 화면,
             재생 결과가 오프라인 스윕과 맞는지
    확인 안 됨  실제 센서 주기·CPU 부하·진짜 주행. 이건 4단계(로봇)에서만 됩니다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "jsonl", description="재생할 수집 파일 (out/*.jsonl 을 기기로 복사해 두세요)"),
        DeclareLaunchArgument("speed", default_value="1.0"),
        DeclareLaunchArgument(
            "loop", default_value="true",
            description="끝나면 처음부터 다시. false 면 재생 후 토픽이 끊겨 "
                        "phm_monitor 가 stale 로 바뀌는 것을 볼 수 있습니다."),
        DeclareLaunchArgument("status_topic", default_value="/phm/status"),
        Node(
            package="phm_collect",
            executable="phm_mock_source",
            name="phm_mock_source",
            output="screen",
            parameters=[{
                "jsonl": LaunchConfiguration("jsonl"),
                "speed": ParameterValue(LaunchConfiguration("speed"), value_type=float),
                "loop": ParameterValue(LaunchConfiguration("loop"), value_type=bool),
            }],
        ),
        Node(
            package="phm_collect",
            executable="phm_monitor",
            name="phm_monitor",
            output="screen",
            parameters=[{
                "status_topic": LaunchConfiguration("status_topic"),
                "publish_period_sec": 1.0,
            }],
        ),
    ])
