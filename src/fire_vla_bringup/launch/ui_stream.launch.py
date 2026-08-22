from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("input_topic", default_value="/image_enhanced"),
        DeclareLaunchArgument("detections_topic", default_value="/yolo_result"),
        # ★ 학습 때 클래스 순서 그대로. 틀리면 불을 사람으로 표시합니다.
        DeclareLaunchArgument("class_names", default_value="['fire','person']"),
        # LAN 대역폭 knob 3개
        DeclareLaunchArgument("stream_fps", default_value="8.0"),
        DeclareLaunchArgument("stream_max_width", default_value="640"),
        DeclareLaunchArgument("jpeg_quality", default_value="70"),
        Node(
            package="image_pipeline",
            executable="ui_stream_node",
            name="ui_stream_node",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "detections_topic": LaunchConfiguration("detections_topic"),
                "class_names": ParameterValue(
                    LaunchConfiguration("class_names"),
                    value_type=list,
                ),
                "stream_fps": ParameterValue(
                    LaunchConfiguration("stream_fps"), value_type=float
                ),
                "stream_max_width": ParameterValue(
                    LaunchConfiguration("stream_max_width"), value_type=int
                ),
                "jpeg_quality": ParameterValue(
                    LaunchConfiguration("jpeg_quality"), value_type=int
                ),
            }],
        ),
    ])