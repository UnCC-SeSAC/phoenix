from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="uncc_example",
            executable="vla_navigation_bridge",
            name="vla_navigation_bridge",
            output="screen",
            parameters=[{
                "goal_topic": "/vla/navigation_goal",
                "result_topic": "/vla/navigation_result",
                "cancel_topic": "/vla/navigation_cancel",
                "robot_pose_topic": "/vla/robot_pose_json",
                "map_frame": "map",
                "base_frame": "base_footprint",
                "pose_publish_period_sec": 0.2,
            }],
        ),
    ])
