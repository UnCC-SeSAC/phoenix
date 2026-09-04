from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ui_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("ui_port", default_value="8080"),
        DeclareLaunchArgument("map_topic", default_value="/map"),
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
                "status_topic": "/vla/status",
                "mission_topic": "/vla/mission",
                "rule_based_status_topic": "/rule_based/status",
                "rule_based_mission_topic": "/rule_based/mission",
                "map_topic": LaunchConfiguration("map_topic"),
                "map_frame": LaunchConfiguration("map_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
            }],
        ),
    ])
