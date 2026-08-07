from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="fire_vla_core",
            executable="vla_orchestrator",
            name="vla_orchestrator",
            output="screen",
            parameters=[{
                "llm_backend": "mock",
                "decision_period_sec": 1.0,
            }],
        )
    ])
