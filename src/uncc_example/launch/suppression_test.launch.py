import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    진압 거리 테스트 — 탐사/SLAM/Nav2 없이 **10 cm 전진 후 진압**만.

        controller (모터 + /odom)
              │
        nudge_and_suppress ──/cmd_vel──▶ 오도메트리 기준 10 cm 전진
              │
              └──suppress_fire 액션──▶ fire_suppression_node (펌프/서보 실구동)
                                              │
                                       check_fire_status
                                              ▼
                                    fire_status_service_node

    SLAM·Nav2·frontier·state_manager·mission_executor 는 **하나도 안 뜬다.**
    거리만 바꿔가며 "몇 cm 앞에서 실제로 꺼지는가"를 재기 위한 벤치다.

    ★ 이 런치는 장애물을 전혀 보지 않는다 (costmap 없음). 로봇 앞이 비어
      있는지 눈으로 확인하고 돌릴 것. 펌프·서보가 실제로 구동되므로 물통과
      배선도 미리 확인할 것.

    사용 예
    -------
        # 기본: 10 cm 전진 → 1회 진압 (카메라 없음 → 판정은 항상 '안꺼짐')
        ros2 launch uncc_example suppression_test.launch.py

        # 거리만 바꿔가며
        ros2 launch uncc_example suppression_test.launch.py distance:=0.15

        # 뒤로 10 cm (음수)
        ros2 launch uncc_example suppression_test.launch.py distance:=-0.10

        # 이동만 하고 진압은 건너뛰기 — 거리 정확도 캘리브레이션용
        ros2 launch uncc_example suppression_test.launch.py skip_suppression:=true

        # controller 가 이미 다른 터미널에서 돌고 있으면
        ros2 launch uncc_example suppression_test.launch.py start_hardware:=false

        # 소화 판정까지 진짜로 보려면 카메라+YOLO 를 같이 띄운다
        ros2 launch uncc_example suppression_test.launch.py \\
            start_vision:=true \\
            model_path:=/home/lemma/Hailo/models/baseline_yolo26_neural_norm.hef

    반복 실행
    ---------
        런치를 끄지 않고 다시 돌릴 수 있다 (매번 그 자리에서 또 10 cm 간다):
        ros2 service call /nudge_and_suppress/run std_srvs/srv/Trigger "{}"

    확인할 것
    ---------
        nudge_and_suppress 터미널:
          "전진 시작 → 이동 중 x.x / 10.0 cm → 정지 — 최종 이동 x.x cm
           (목표 대비 ±x.x cm) → 진압 goal 전송 → 진압 1차 ... → 진압 성공/실패"
        fire_suppression_node 터미널: 실제 펌프/서보 구동 로그
        start_vision:=false 이면 fire_status_service_node 가 "표본 0건 →
        안꺼짐"으로 응답하는 것이 정상이다 (카메라가 없으므로).
    """

    uncc_share = get_package_share_directory('uncc_example')
    controller_share = get_package_share_directory('controller')
    peripherals_share = get_package_share_directory('peripherals')
    image_pipeline_share = get_package_share_directory('image_pipeline')

    ASCAMERA = '/ascamera/camera_publisher'

    start_hardware = LaunchConfiguration('start_hardware')
    start_vision = LaunchConfiguration('start_vision')

    # =========================================
    # 모터 + 오도메트리
    # hardware.launch.py 를 안 쓰고 controller 만 띄운다 — 이 테스트에는
    # LiDAR 가 필요 없고, 라이다까지 켜면 RPi5 CPU만 먹는다.
    # =========================================

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_share, 'launch', 'controller.launch.py')
        ),
        condition=IfCondition(start_hardware),
    )

    # =========================================
    # 진압 유닛 (fire_extinguisher.launch.py 와 같은 조합)
    # controller 가 뜬 뒤에 올려서 GPIO/시리얼 초기화가 겹치지 않게 한다.
    # =========================================

    fire_nodes = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='uncc_example',
                executable='fire_status_service_node',
                name='fire_status_service_node',
                output='screen',
                parameters=[{
                    'detections_topic': '/yolo_result',
                    'fire_class_name': 'fire',
                    'min_score': ParameterValue(
                        LaunchConfiguration('fire_min_score'), value_type=float),
                    'extinguished_ratio': ParameterValue(
                        LaunchConfiguration('fire_extinguished_ratio'),
                        value_type=float),
                }],
            ),
            Node(
                package='uncc_example',
                executable='fire_suppression_node',
                name='fire_suppression_node',
                output='screen',
            ),
        ],
    )

    # =========================================
    # (선택) 카메라 + YOLO — 소화 판정을 실제 영상으로 하고 싶을 때만.
    # frontier_fire_suppression_hw_test.launch.py 의 비전 섹션과 같은 배선에서
    # detection_3d_node/vision_detector 만 뺐다 (map 좌표가 필요 없으므로).
    # =========================================

    vision = TimerAction(
        period=4.0,
        actions=[
            SetEnvironmentVariable(name='need_compile', value='True'),
            SetEnvironmentVariable(name='DEPTH_CAMERA_TYPE', value='ascamera'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        peripherals_share, 'launch', 'depth_camera.launch.py')
                ),
            ),
            Node(
                package='image_pipeline',
                executable='preprocess_node',
                name='rgb_preprocess_node',
                output='screen',
                parameters=[
                    os.path.join(
                        image_pipeline_share, 'config', 'preprocess.yaml'),
                    {
                        'input_topic': f'{ASCAMERA}/rgb0/image',
                        'camera_info_topic': f'{ASCAMERA}/rgb0/camera_info',
                        'output_topic': '/image_enhanced',
                        'output_camera_info_topic': '/image_enhanced/camera_info',
                    },
                ],
            ),
            Node(
                package='image_pipeline',
                executable='yolo_node',
                name='yolo_node',
                output='screen',
                parameters=[{
                    'model_path': LaunchConfiguration('model_path'),
                    'class_names': LaunchConfiguration('class_names'),
                    'layout': LaunchConfiguration('layout'),
                    'threads': LaunchConfiguration('threads'),
                    'conf': ParameterValue(
                        LaunchConfiguration('conf'), value_type=float),
                    'input_topic': '/image_enhanced',
                    'detections_topic': '/yolo_result',
                }],
            ),
        ],
        condition=IfCondition(start_vision),
    )

    # =========================================
    # 전진 + 진압 트리거
    # fire_suppression_node(t=3)보다 뒤에 시작하고, 노드 자체도
    # start_delay_sec 만큼 더 기다렸다가 움직인다 (EKF /odom 안정화).
    # =========================================

    nudge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='uncc_example',
                executable='nudge_and_suppress',
                name='nudge_and_suppress',
                output='screen',
                parameters=[{
                    'distance': ParameterValue(
                        LaunchConfiguration('distance'), value_type=float),
                    'speed': ParameterValue(
                        LaunchConfiguration('speed'), value_type=float),
                    'settle_sec': ParameterValue(
                        LaunchConfiguration('settle_sec'), value_type=float),
                    'max_attempts': ParameterValue(
                        LaunchConfiguration('max_attempts'), value_type=int),
                    'skip_suppression': ParameterValue(
                        LaunchConfiguration('skip_suppression'), value_type=bool),
                    'auto_start': ParameterValue(
                        LaunchConfiguration('auto_start'), value_type=bool),
                    'start_delay_sec': ParameterValue(
                        LaunchConfiguration('start_delay_sec'), value_type=float),
                    'shutdown_after': ParameterValue(
                        LaunchConfiguration('shutdown_after'), value_type=bool),
                }],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'distance', default_value='0.10',
            description='전진 거리(m). 음수면 후진'),
        DeclareLaunchArgument(
            'speed', default_value='0.08',
            description='이동 속도(m/s). 안 움직이면 0.10~0.12 로 올릴 것'),
        DeclareLaunchArgument(
            'settle_sec', default_value='1.0',
            description='정지 후 진압 시작까지 대기(s)'),
        DeclareLaunchArgument(
            'max_attempts', default_value='1',
            description='진압 시도 횟수. 0이면 서버 기본값(3회)'),
        DeclareLaunchArgument(
            'skip_suppression', default_value='false',
            description='true면 이동만 하고 진압을 안 부름 (거리 캘리브레이션용)'),
        DeclareLaunchArgument(
            'auto_start', default_value='true',
            description='false면 ~/run 서비스를 부를 때까지 안 움직임'),
        DeclareLaunchArgument(
            'start_delay_sec', default_value='3.0',
            description='노드 기동 후 출발까지 대기(s). /odom 안정화용'),
        DeclareLaunchArgument(
            'shutdown_after', default_value='false',
            description='true면 진압 결과를 받고 런치를 종료'),
        DeclareLaunchArgument(
            'start_hardware', default_value='true',
            description='controller(모터+오도메트리)를 여기서 띄울지'),
        DeclareLaunchArgument(
            'start_vision', default_value='false',
            description='true면 카메라+YOLO 도 띄워 실제 영상으로 소화 판정'),
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='start_vision:=true 일 때 필수. .onnx | .hef 절대경로'),
        DeclareLaunchArgument(
            'class_names', default_value="['fire','person']",
            description='★ 학습 때 순서 그대로'),
        DeclareLaunchArgument('layout', default_value='auto'),
        DeclareLaunchArgument('threads', default_value='3'),
        DeclareLaunchArgument('conf', default_value='0.75'),
        DeclareLaunchArgument(
            'fire_min_score', default_value='0.0',
            description='이 점수 미만 화재 검출은 무시. 0.0=끔'),
        DeclareLaunchArgument(
            'fire_extinguished_ratio', default_value='0.3',
            description='관찰 구간 내 화재 프레임 비율이 이 값 미만이면 꺼짐'),
        SetEnvironmentVariable(name='need_compile', value='True'),
        controller,
        fire_nodes,
        vision,
        nudge,
    ])
