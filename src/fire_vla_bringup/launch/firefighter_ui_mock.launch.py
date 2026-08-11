from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ui_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("ui_port", default_value="8080"),
        Node(
            package="fire_vla_core",
            executable="vla_orchestrator",
            name="vla_orchestrator",
            output="screen",
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
            }],
        ),
    ])
