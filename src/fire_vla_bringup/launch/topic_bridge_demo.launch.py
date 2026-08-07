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
                "navigation_mode": "TOPIC_BRIDGE",
            }],
        ),
        Node(
            package="fire_vla_core",
            executable="vla_demo_input",
            name="vla_demo_input",
            output="screen",
            parameters=[{
                "person_x": 2.0,
                "person_y": 0.0,
                "fire_x": 1.0,
                "fire_y": 0.0,
                "publish_period_sec": 0.5,
                "repeat_observation": True,
            }],
        ),
    ])
