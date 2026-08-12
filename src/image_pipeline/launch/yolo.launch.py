"""YOLO26 검출 노드 런치 — 태스크① 출력 -> 태스크② 입력.

  ros2 launch image_pipeline yolo.launch.py model_path:=models/fire_yolo26s.onnx

전체 사슬(①→YOLO→②)을 한 번에 띄우려면 셋을 각각 launch 하세요:

  ros2 launch image_pipeline preprocess.launch.py
  ros2 launch image_pipeline yolo.launch.py model_path:=...
  ros2 launch image_pipeline detection_3d.launch.py

★ `class_names`는 **학습 때 순서 그대로**여야 합니다. 순서가 틀리면 불을
  사람으로 발행하고, 메인은 물을 사람에게 쏩니다. 순서 근거는 학습 시
  `data.yaml`의 `names`입니다 (`tools/make_dataset.py`가 만든 것).

★ `layout`은 실제 모델을 처음 붙일 때 `auto` 그대로 두고 시작 로그의
  「레이아웃=」을 확인한 뒤 **그 값으로 못박으세요**. 자동 판별은 휴리스틱이고,
  Hailo 출력이 PC ONNX와 다를 수 있습니다 (HANDOVER 7-2).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='.onnx (로봇/개발) | .pt (개발 PC) | .hef (Hailo, 미구현)'),
        DeclareLaunchArgument(
            'input_topic', default_value='/image_enhanced',
            description='태스크①의 출력. 원본 rgb0 가 아닙니다'),
        DeclareLaunchArgument('detections_topic', default_value='/yolo_result'),
        DeclareLaunchArgument(
            'class_names', default_value="['fire']",
            description='★ 학습 때 순서 그대로'),
        DeclareLaunchArgument(
            'imgsz', default_value='640',
            description='★ 학습 때 값과 같아야 합니다'),
        DeclareLaunchArgument('conf', default_value='0.25'),
        DeclareLaunchArgument('iou', default_value='0.45'),
        DeclareLaunchArgument(
            'layout', default_value='auto',
            description='auto | v8 | end2end — 실측 후 못박을 것'),
        DeclareLaunchArgument(
            'threads', default_value='0',
            description='Pi 5에서는 3 권장 (ROS·Hailo 드라이버와 코어 분배)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    node = Node(
        package='image_pipeline',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'input_topic': LaunchConfiguration('input_topic'),
            'detections_topic': LaunchConfiguration('detections_topic'),
            'class_names': LaunchConfiguration('class_names'),
            'imgsz': LaunchConfiguration('imgsz'),
            'conf': LaunchConfiguration('conf'),
            'iou': LaunchConfiguration('iou'),
            'layout': LaunchConfiguration('layout'),
            'threads': LaunchConfiguration('threads'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription(args + [node])
