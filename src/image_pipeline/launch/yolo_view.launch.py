"""YOLO만 띄워서 **박스와 confidence를 rqt로 눈으로 보는** 경량 런치.

  ros2 launch image_pipeline yolo_view.launch.py \\
      model_path:=/home/lemma/Hailo/models/baseline_yolo26_neural_norm.hef \\
      class_names:="['fire','person']" conf:=0.10

띄우는 것은 **카메라 + yolo_node + detection_overlay_node** 뿐입니다.
`frontier_fire_suppression_hw_test.launch.py`는 SLAM·Nav2·frontier·미션 스택·GPIO까지
전부 띄우기 때문에 파이 5(4코어)에서 CPU가 다 먹히고, 정작 "불꽃과 사람을 제대로
보는가"를 판단할 수 없습니다. 이 런치는 그 판단만 하기 위한 것입니다.

원격 PC에서 보기 (같은 ROS_DOMAIN_ID, 같은 RMW):

  ros2 run rqt_image_view rqt_image_view
  # 드롭다운에서 /yolo/overlay/compressed  (무선이면 compressed 권장)

★ class_names 는 **학습 때 순서 그대로**입니다
------------------------------------------------
순서가 틀리면 불을 사람으로 발행합니다. 현재 데이터셋은 `['fire','person']`
입니다 (`fire_status_service_node_v2.py` 주석 참조 — '인형'이라는 클래스는 없고
사람 클래스가 `person` 입니다). 대소문자도 그대로여야 합니다.

⚠ 기본은 전처리(태스크①)를 **뺀** 배선입니다
----------------------------------------------
기본값 `use_preprocess:=false`는 카메라 원본(`rgb0/image`)을 YOLO에 직결합니다.
가장 가볍지만 실제 배포에서 YOLO가 받는 영상(`/image_enhanced` — 디헤이즈+CLAHE)과
**다른 영상**입니다. 연기 없는 환경의 1차 탐지 확인에는 충분하나, 최종 성능
판단은 `use_preprocess:=true`로 한 번 더 보세요. 이 인자 하나로 두 조건을
같은 자리에서 비교할 수 있습니다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# detection_3d.launch.py / full_chain_check.launch.py 와 같은 실카메라 접두어.
ASCAMERA = '/ascamera/camera_publisher'
DEFAULT_CAMERA_TOPIC = f'{ASCAMERA}/rgb0/image'


def launch_setup(context, *args, **kwargs):
    """`use_preprocess`에 따라 YOLO 입력 토픽이 갈리므로 여기서 분기합니다.

    오버레이의 `image_topic`은 **YOLO가 실제로 받은 그 토픽**과 같아야 합니다.
    다르면 박스 좌표가 어긋나고, 그 화면을 보고 모델을 의심하게 됩니다.
    """
    use_preprocess = LaunchConfiguration('use_preprocess').perform(context)
    use_preprocess = use_preprocess.strip().lower() in ('true', '1', 'yes')
    camera_topic = LaunchConfiguration('camera_topic').perform(context)

    nodes = []

    if use_preprocess:
        yolo_input = '/image_enhanced'
        image_pipeline_share = get_package_share_directory('image_pipeline')
        nodes.append(Node(
            package='image_pipeline',
            executable='preprocess_node',
            name='rgb_preprocess_node',
            output='screen',
            parameters=[
                os.path.join(image_pipeline_share, 'config', 'preprocess.yaml'),
                {
                    # YAML(구형 realsense 이름)보다 나중에 와서 실카메라로 덮어씀.
                    'input_topic': camera_topic,
                    'camera_info_topic': f'{ASCAMERA}/rgb0/camera_info',
                    'output_topic': yolo_input,
                    'output_camera_info_topic': '/image_enhanced/camera_info',
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
        ))
    else:
        yolo_input = camera_topic

    nodes.append(Node(
        package='image_pipeline',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'class_names': LaunchConfiguration('class_names'),
            'imgsz': LaunchConfiguration('imgsz'),
            'conf': LaunchConfiguration('conf'),
            'iou': LaunchConfiguration('iou'),
            'layout': LaunchConfiguration('layout'),
            'threads': LaunchConfiguration('threads'),
            'input_topic': yolo_input,
            'detections_topic': '/yolo_result',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    ))

    nodes.append(Node(
        package='image_pipeline',
        executable='detection_overlay_node',
        name='detection_overlay_node',
        output='screen',
        parameters=[{
            'image_topic': yolo_input,          # ★ YOLO 입력과 반드시 동일
            'detections_topic': '/yolo_result',
            'output_topic': LaunchConfiguration('overlay_topic'),
            'min_score': LaunchConfiguration('overlay_min_score'),
            'display_width': LaunchConfiguration('display_width'),
            'max_fps': LaunchConfiguration('max_fps'),
            'jpeg_quality': LaunchConfiguration('jpeg_quality'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    ))

    # 카메라(t=0)가 자리를 잡은 뒤에 붙입니다. 모델 적재도 여기서 일어납니다.
    return [TimerAction(period=3.0, actions=nodes)]


def generate_launch_description():
    peripherals_share = get_package_share_directory('peripherals')

    args = [
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='실제 YOLO 가중치 절대경로 (.onnx | .pt | .hef). '
                        '비우면 yolo_node 가 즉시 에러로 알립니다'),
        DeclareLaunchArgument(
            'class_names', default_value="['fire','person']",
            description='★ 학습 때 순서 그대로. 틀리면 불을 사람으로 발행합니다'),
        DeclareLaunchArgument(
            'conf', default_value='0.25',
            description='낮춰서 보면 아슬아슬하게 잡히는 것까지 보입니다 (0.10 등). '
                        '런타임 변경도 됩니다: ros2 param set /yolo_node conf 0.35'),
        DeclareLaunchArgument('iou', default_value='0.45'),
        DeclareLaunchArgument(
            'imgsz', default_value='640',
            description='★ 학습 때 값과 같아야 합니다. 추론이 밀리면 480'),
        DeclareLaunchArgument(
            'layout', default_value='auto',
            description='auto | v8 | end2end — 시작 로그의 「레이아웃=」 확인 후 못박을 것'),
        DeclareLaunchArgument(
            'threads', default_value='3',
            description='파이 5(4코어) 권장값'),

        DeclareLaunchArgument(
            'start_camera', default_value='true',
            description='false면 카메라가 이미 떠 있다고 가정합니다'),
        DeclareLaunchArgument(
            'camera_topic', default_value=DEFAULT_CAMERA_TOPIC,
            description='★ RGB는 /image 입니다 (/image_raw 는 뎁스)'),
        DeclareLaunchArgument(
            'use_preprocess', default_value='false',
            description='true면 태스크①(디헤이즈+CLAHE)을 끼워 실제 배포와 같은 '
                        '영상으로 봅니다. 파이에서 그만큼 무거워집니다'),

        DeclareLaunchArgument('overlay_topic', default_value='/yolo/overlay'),
        DeclareLaunchArgument(
            'overlay_min_score', default_value='0.0',
            description='화면에 그릴 최소 confidence. 0이면 전부 그립니다'),
        DeclareLaunchArgument(
            'display_width', default_value='640',
            description='오버레이 전송 폭. 0이면 원본. 무선이 느리면 480/320'),
        DeclareLaunchArgument(
            'max_fps', default_value='10.0',
            description='오버레이 발행 상한. 추론 fps 와는 무관합니다'),
        DeclareLaunchArgument('jpeg_quality', default_value='75'),
        DeclareLaunchArgument(
            'start_rqt', default_value='false',
            description='로봇 화면/VNC에서 볼 때만 true. 원격 PC에서는 거기서 '
                        'rqt_image_view 를 따로 띄우세요'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    # ★ depth_camera.launch.py 는 두 환경변수를 `os.environ[...]` 으로 **직접**,
    #   그것도 `generate_launch_description()` 안에서 읽습니다. 없으면 KeyError 로
    #   런치가 통째로 죽습니다.
    #   hw_test 런치에서는 hardware.launch.py 가 need_compile 을 세팅해 주지만
    #   여기서는 hardware 를 안 띄우므로 이 런치가 직접 채워야 합니다.
    #   `SetEnvironmentVariable` 액션이 아니라 여기서 채우는 이유는, 저 액션은
    #   실행 시점에 적용되는데 include 대상이 언제 평가되는지에 기대고 싶지
    #   않아서입니다. `setdefault` 라 이미 내보낸 값이 있으면 그쪽을 존중합니다
    #   (예: DEPTH_CAMERA_TYPE 을 usb_cam 으로 쓰는 환경).
    os.environ.setdefault('need_compile', 'True')
    os.environ.setdefault('DEPTH_CAMERA_TYPE', 'ascamera')

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_share, 'launch', 'depth_camera.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('start_camera')),
    )

    rqt = TimerAction(
        period=6.0,
        actions=[ExecuteProcess(
            cmd=['rqt_image_view', LaunchConfiguration('overlay_topic')],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_rqt')),
        )],
    )

    return LaunchDescription(
        args + [camera, OpaqueFunction(function=launch_setup), rqt])
