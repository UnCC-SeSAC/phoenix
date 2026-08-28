import os
import shlex
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _create_vla_node(context):
    python_value = LaunchConfiguration(
        "vla_python_executable"
    ).perform(context).strip()
    prefix = None
    if python_value:
        python_path = Path(python_value).expanduser()
        if not python_path.is_file():
            raise RuntimeError(
                f"vla_python_executable 파일이 없습니다: {python_path}"
            )
        if not os.access(python_path, os.X_OK):
            raise RuntimeError(
                f"vla_python_executable 실행 권한이 없습니다: {python_path}"
            )
        prefix = shlex.quote(str(python_path))

    return [Node(
        package="fire_vla_core",
        executable="vla_orchestrator",
        name="vla_orchestrator",
        output="screen",
        prefix=prefix,
        parameters=[{
            "llm_backend": LaunchConfiguration("llm_backend"),
            "transformers_model_id": LaunchConfiguration(
                "transformers_model_id"
            ),
            "transformers_device": LaunchConfiguration(
                "transformers_device"
            ),
            "transformers_max_new_tokens": ParameterValue(
                LaunchConfiguration("transformers_max_new_tokens"),
                value_type=int,
            ),
            "remote_qwen_endpoint": LaunchConfiguration(
                "remote_qwen_endpoint"
            ),
            "remote_qwen_timeout_sec": ParameterValue(
                LaunchConfiguration("remote_qwen_timeout_sec"),
                value_type=float,
            ),
            "person_fire_risk_distance_m": ParameterValue(
                LaunchConfiguration("person_fire_risk_distance_m"),
                value_type=float,
            ),
            "decision_period_sec": 1.0,
            "navigation_mode": "TOPIC_BRIDGE",
            "report_mode": "TOPIC_BRIDGE",
            "spray_mode": "TOPIC_BRIDGE",
            "mission_topic": "/vla/mission",
            "perception_topic": "/vla/perception_observation",
            "robot_pose_topic": "/vla/robot_pose_json",
            "navigation_goal_topic": "/vla/navigation_goal",
            "navigation_result_topic": "/vla/navigation_result",
            "navigation_cancel_topic": "/vla/navigation_cancel",
            "spray_command_topic": "/vla/spray_command",
            "spray_result_topic": "/vla/spray_result",
            "spray_cancel_topic": "/vla/spray_cancel",
            "person_report_topic": "/vla/person_report",
            "person_report_result_topic": "/vla/person_report_result",
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("llm_backend", default_value="mock"),
        DeclareLaunchArgument(
            "vla_python_executable",
            default_value="",
        ),
        DeclareLaunchArgument(
            "transformers_model_id",
            default_value="Qwen/Qwen2.5-1.5B-Instruct",
        ),
        DeclareLaunchArgument("transformers_device", default_value="xpu:0"),
        DeclareLaunchArgument(
            "transformers_max_new_tokens",
            default_value="64",
        ),
        DeclareLaunchArgument(
            "remote_qwen_endpoint",
            default_value="http://127.0.0.1:8088/infer",
        ),
        DeclareLaunchArgument(
            "remote_qwen_timeout_sec",
            default_value="3.0",
        ),
        DeclareLaunchArgument(
            "person_fire_risk_distance_m",
            default_value="0.10",
        ),
        DeclareLaunchArgument(
            "start_perception_bridge",
            default_value="false",
            description="Bridge /fire/detections into the canonical VLA topic",
        ),
        DeclareLaunchArgument(
            "perception_input_topic",
            default_value="/fire/detections",
        ),
        DeclareLaunchArgument(
            "perception_status_topic",
            default_value="/fire/detections/status",
        ),
        DeclareLaunchArgument(
            "perception_camera_info_topic",
            default_value="/ascamera/camera_publisher/rgb0/camera_info",
        ),
        Node(
            package="fire_vla_core",
            executable="vla_perception_bridge",
            name="vla_perception_bridge",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_perception_bridge")),
            parameters=[{
                "input_topic": LaunchConfiguration("perception_input_topic"),
                "status_topic": LaunchConfiguration("perception_status_topic"),
                "camera_info_topic": LaunchConfiguration(
                    "perception_camera_info_topic"
                ),
            }],
        ),
        OpaqueFunction(function=_create_vla_node),
    ])
