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

★ 박스를 눈으로 보려면 (2026-08-26 추가)
------------------------------------------
`detection_overlay_node`를 **같이 띄웁니다**(`overlay:=true`, 기본). 박스와
`fire 0.87` 같은 라벨이 구워진 영상이 `/yolo/overlay` 와
`/yolo/overlay/compressed` 로 나갑니다. 원격 PC에서:

  ros2 run rqt_image_view rqt_image_view      # /yolo/overlay/compressed 선택

`yolo_node`는 `Detection2DArray`만 내므로 rqt_image_view로는 박스를 볼 수
없습니다. 그래서 별도 노드가 필요합니다. 끄려면 `overlay:=false`.

★ 오버레이의 `image_topic`은 `input_topic`과 **같은 값으로 묶여 있습니다.**
  박스 좌표는 YOLO가 받은 프레임 기준이라, 다른 토픽에 그리면 어긋납니다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='.onnx (로봇/개발) | .pt (개발 PC) | .hef (Hailo — 후처리 '
                        'onnx/config json은 같은 폴더에서 자동으로 찾습니다)'),
        DeclareLaunchArgument(
            'input_topic', default_value='/image_enhanced',
            description='태스크①의 출력. 원본 rgb0 가 아닙니다'),
        DeclareLaunchArgument('detections_topic', default_value='/yolo_result'),
        DeclareLaunchArgument(
            'class_names', default_value="['fire','person']",
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

        # --- 디버그 오버레이 (rqt로 박스를 보기 위한 것) ---
        DeclareLaunchArgument(
            'overlay', default_value='true',
            description='박스가 구워진 영상 토픽을 낼지. 끄려면 false'),
        DeclareLaunchArgument('overlay_topic', default_value='/yolo/overlay'),
        DeclareLaunchArgument(
            'overlay_min_score', default_value='0.0',
            description='화면에 그릴 최소 confidence. 0이면 전부 그립니다 — '
                        '오탐이 어느 대에서 갈리는지 보려면 0으로 두세요'),
        DeclareLaunchArgument(
            'display_width', default_value='640',
            description='오버레이 전송 폭. 0이면 원본. 무선이 느리면 480/320'),
        DeclareLaunchArgument(
            'max_fps', default_value='10.0',
            description='오버레이 발행 상한. 추론 fps 와는 무관합니다'),
        DeclareLaunchArgument('jpeg_quality', default_value='75'),
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

    # ★ image_topic / detections_topic 을 위 노드와 **같은 LaunchConfiguration**
    #   으로 넘깁니다. 문자열을 따로 박으면 언젠가 한쪽만 바뀌고, 그러면 박스가
    #   조용히 어긋난 자리에 그려집니다.
    overlay = Node(
        package='image_pipeline',
        executable='detection_overlay_node',
        name='detection_overlay_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('overlay')),
        parameters=[{
            'image_topic': LaunchConfiguration('input_topic'),
            'detections_topic': LaunchConfiguration('detections_topic'),
            'output_topic': LaunchConfiguration('overlay_topic'),
            'min_score': LaunchConfiguration('overlay_min_score'),
            'display_width': LaunchConfiguration('display_width'),
            'max_fps': LaunchConfiguration('max_fps'),
            'jpeg_quality': LaunchConfiguration('jpeg_quality'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription(args + [node, overlay])