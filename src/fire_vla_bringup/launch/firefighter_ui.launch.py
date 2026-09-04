from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ui_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("ui_port", default_value="8080"),
        # ★ 기본은 loopback. LAN 관제 PC에서 열 때만 켜세요 —
        #   Mission/START/STOP 제어 경계도 같이 열립니다.
        DeclareLaunchArgument("ui_allow_remote", default_value="false"),
        DeclareLaunchArgument("ui_vision_enabled", default_value="true"),
        DeclareLaunchArgument("ui_map_enabled", default_value="true"),
        # PHM: 로봇의 phm_monitor(phm_collect)가 내는 건전성 상태를 /api/phm 으로
        # 실어 나릅니다. 노드가 안 떠 있어도 available:false 를 돌려주므로
        # 켜 두어도 무해합니다.
        DeclareLaunchArgument("ui_phm_enabled", default_value="true"),
        DeclareLaunchArgument("phm_status_topic", default_value="/phm/status"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint"),
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
                "ui_vision_enabled": ParameterValue(
                    LaunchConfiguration("ui_vision_enabled"), value_type=bool
                ),
                "ui_map_enabled": ParameterValue(
                    LaunchConfiguration("ui_map_enabled"), value_type=bool
                ),
                "ui_phm_enabled": ParameterValue(
                    LaunchConfiguration("ui_phm_enabled"), value_type=bool
                ),
                "phm_status_topic": LaunchConfiguration("phm_status_topic"),
                "map_frame": LaunchConfiguration("map_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "status_topic": "/vla/status",
                "mission_topic": "/vla/mission",
                "rule_based_status_topic": "/rule_based/status",
                "rule_based_mission_topic": "/rule_based/mission",
            }],
        ),
    ])