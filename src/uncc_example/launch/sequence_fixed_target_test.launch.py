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
    고정 좌표 3개(불1 -> 사람 -> 불2 -> base)를 실제 Nav2/GPIO로 검증하는
    테스트 전용 launch. 실제 불을 좌표에 놓고 테스트하므로 카메라/YOLO는
    그대로 띄운다 — 다만 그 결과는 "이동할 좌표"를 정하는 데는 안 쓰고
    (좌표는 launch 인자 fire1_xy/person_xy/fire2_xy 로 고정 주입),
    fire_suppression_node 가 분사 후 "꺼졌는지"를 확인하는 용도로만 쓴다.

    frontier_fire_suppression_hw_test.launch.py 와 같은 하드웨어 조합
    (실제 모터/라이다 + slam_toolbox + Nav2, 실제 fire_suppression_node
    GPIO)을 쓰되, detection_3d_node/vision_detector(카메라 검출을 map
    좌표로 바꿔 state_manager 에 먹이는 부분)는 안 띄운다 — 그 경로는
    거리 기반 클러스터링/타겟 선택에만 쓰이는데, state_manager 를
    sequence_test_state_manager 로 교체해 /vision/detections 자체를
    무시하고 고정 좌표 순서를 강제하기 때문이다. yolo_node 의
    /yolo_result 만 fire_status_service_node 가 직접 구독해서 실제
    진화 여부를 판별한다.

    사용 예:
        ros2 launch uncc_example sequence_fixed_target_test.launch.py \\
            model_path:=/home/lemma/Hailo/models/baseline_yolo26_neural_norm.hef \\
            class_names:="['fire','person']" \\
            fire1_xy:="[1.0, 2.0]" person_xy:="[1.5, 1.5]" \\
            fire2_xy:="[3.0, 1.0]"

    확인할 것:
        ros2 topic echo /mission/state
        # sequence_test_state_manager 로그: INITIAL_SWEEP -> FIRE_DETECTED
        #   (불1) -> PERSON_DETECTED -> FIRE_DETECTED(불2) ->
        #   RETURNING_TO_CHARGE -> MISSION_COMPLETE
        # fire_suppression_node 터미널: 실제 펌프/서보 구동 로그 +
        #   1차 판별 결과: 안꺼짐 -> 2차 판별 결과: 꺼짐
        ros2 service call /state_manager/start_mission std_srvs/srv/Trigger "{}"
    """

    uncc_share = get_package_share_directory('uncc_example')
    frontier_share = get_package_share_directory('frontier_exploration_ros2')
    peripherals_share = get_package_share_directory('peripherals')
    image_pipeline_share = get_package_share_directory('image_pipeline')
    launch_dir = os.path.join(uncc_share, 'launch')
    frontier_params = os.path.join(frontier_share, 'config', 'params.yaml')

    # detection_3d.launch.py / full_chain_check.launch.py 와 동일한 실카메라
    # 토픽 접두어 (ascamera 드라이버 기준).
    ASCAMERA = '/ascamera/camera_publisher'

    def include_launch(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, name)
            ),
        )

    # =========================================
    # 실제 하드웨어 + SLAM + Nav2
    # =========================================

    hardware = include_launch('hardware.launch.py')

    camera = TimerAction(
        period=1.0,
        actions=[
            SetEnvironmentVariable(name='DEPTH_CAMERA_TYPE', value='ascamera'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(peripherals_share, 'launch', 'depth_camera.launch.py')
                ),
            ),
        ],
    )

    slam = TimerAction(
        period=3.0,
        actions=[include_launch('slam_mapping.launch.py')],
    )

    nav2 = TimerAction(
        period=7.0,
        actions=[include_launch('nav2_online.launch.py')],
    )

    # =========================================
    # 비전 — 실제 불을 카메라로 확인하기 위해 preprocess_node/yolo_node
    # 는 그대로 띄운다. detection_3d_node/vision_detector 는 뺐다 —
    # 좌표는 launch 인자로 고정 주입되므로 카메라 검출을 map 좌표로
    # 바꿔 타겟을 고르는 경로 자체가 필요 없다.
    # =========================================

    vision = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='image_pipeline',
                executable='preprocess_node',
                name='rgb_preprocess_node',
                output='screen',
                parameters=[
                    os.path.join(image_pipeline_share, 'config', 'preprocess.yaml'),
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
            # 디버그용 — 박스와 confidence 가 구워진 영상을 /yolo/overlay 로.
            # 실제 불이 잘 검출되는지 눈으로 확인하는 용도.
            Node(
                package='image_pipeline',
                executable='detection_overlay_node',
                name='detection_overlay_node',
                output='screen',
                condition=IfCondition(LaunchConfiguration('overlay')),
                parameters=[{
                    'image_topic': '/image_enhanced',
                    'detections_topic': '/yolo_result',
                    'output_topic': '/yolo/overlay',
                    'display_width': LaunchConfiguration('overlay_display_width'),
                    'max_fps': LaunchConfiguration('overlay_max_fps'),
                }],
            ),
        ],
    )

    # =========================================
    # Frontier Explorer + Frontier State Controller
    # sequence_test_state_manager 는 EXPLORING 상태로 절대 안 가지만,
    # mission_executor 가 FIRE/PERSON/RETURNING 상태에서 nav goal을 보내기
    # 전에 frontier 를 STATE_IDLE 로 정지시키는 걸 기다리므로 반드시 떠
    # 있어야 한다 (hw_test 와 동일 파라미터).
    # =========================================

    frontier = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='frontier_exploration_ros2',
                executable='frontier_explorer',
                name='frontier_explorer',
                output='both',
                parameters=[
                    frontier_params,
                    {
                        'control_service_enabled': True,
                        'autostart': False,
                        'mrtsp_solver': 'greedy',
                        'map_processing_rate_hz': 0.5,
                        'goal_preemption_enabled': False,
                        'return_to_start_on_complete': False,
                    },
                ],
            ),
        ],
    )

    frontier_state_controller = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='uncc_example',
                executable='frontier_state_controller',
                name='frontier_state_controller',
                output='both',
                parameters=[{
                    'frontier_control_service': '/control_exploration',
                    'stop_timeout_sec': 5.0,
                }],
            ),
        ],
    )

    # =========================================
    # 미션 스택 — state_manager 만 sequence_test_state_manager 로 교체
    # =========================================

    state_manager = Node(
        package='uncc_example',
        executable='sequence_test_state_manager',
        name='state_manager',
        output='both',
        parameters=[{
            # class_names 와 같은 방식 — 기본값이 "[1.0, 2.0]" 같은 YAML
            # 리스트 문자열이면 launch_ros 가 evaluate 해서 double array 로
            # 넘겨준다.
            'fire1_xy': LaunchConfiguration('fire1_xy'),
            'person_xy': LaunchConfiguration('person_xy'),
            'fire2_xy': LaunchConfiguration('fire2_xy'),
            'sweep_angle_deg': ParameterValue(
                LaunchConfiguration('sweep_angle_deg'), value_type=float),
            'sweep_dwell_sec': ParameterValue(
                LaunchConfiguration('sweep_dwell_sec'), value_type=float),
        }],
    )

    mission_executor = Node(
        package='uncc_example',
        executable='mission_executor',
        name='mission_executor',
        output='both',
    )

    # 실제 노드 — yolo_node 의 /yolo_result 를 직접 구독해 진화 여부를
    # 판정한다 (fire/person 위치 자체는 vision_detector 를 거치지 않으므로
    # 몰라도 된다).
    fire_status_real = Node(
        package='uncc_example',
        executable='fire_status_service_node',
        name='fire_status_service_node',
        output='both',
        parameters=[{
            'detections_topic': '/yolo_result',
            'fire_class_name': 'fire',
            'min_score': ParameterValue(
                LaunchConfiguration('fire_min_score'), value_type=float),
            'extinguished_ratio': ParameterValue(
                LaunchConfiguration('fire_extinguished_ratio'), value_type=float),
        }],
    )

    # 실제 GPIO13(펌프)/GPIO18(서보) 구동 노드.
    fire_suppression_real = Node(
        package='uncc_example',
        executable='fire_suppression_node',
        name='fire_suppression_node',
        output='both',
    )

    fire_keepout = Node(
        package='uncc_example',
        executable='fire_keepout_node',
        name='fire_keepout_node',
        output='both',
    )

    mission_stack = TimerAction(
        period=11.0,
        actions=[
            state_manager,
            mission_executor,
            fire_status_real,
            fire_suppression_real,
            fire_keepout,
        ],
    )

    # =========================================
    # UI — rule_based_ui_adapter 가 /mission/state, /mission/found_targets
    # (불1/사람/불2 고정 좌표) 를 /rule_based/status 로 변환해 발행하고,
    # firefighter_ui 가 그걸 구독해 지도에 표시한다. START/STOP 버튼도
    # state_manager 의 start_mission/stop_mission 서비스에 연결돼 있다.
    # 기본 http://<Pi IP>:8080.
    # =========================================

    ui = TimerAction(
        period=14.0,
        actions=[
            Node(
                package='uncc_example',
                executable='rule_based_ui_adapter',
                name='rule_based_ui_adapter',
                output='screen',
            ),
            Node(
                package='image_pipeline',
                executable='ui_stream_node',
                name='ui_stream_node',
                output='screen',
                condition=IfCondition(LaunchConfiguration('start_ui_vision')),
                parameters=[{
                    'input_topic': '/image_enhanced',
                    'detections_topic': '/yolo_result',
                    'class_names': LaunchConfiguration('class_names'),
                }],
            ),
            Node(
                package='fire_vla_core',
                executable='firefighter_ui',
                name='firefighter_ui',
                output='screen',
                parameters=[{
                    'ui_host': LaunchConfiguration('ui_host'),
                    'ui_port': ParameterValue(
                        LaunchConfiguration('ui_port'), value_type=int),
                    'ui_allow_remote': ParameterValue(
                        LaunchConfiguration('ui_allow_remote'),
                        value_type=bool),
                    'ui_vision_enabled': ParameterValue(
                        LaunchConfiguration('start_ui_vision'),
                        value_type=bool),
                    'rule_based_status_topic': '/rule_based/status',
                    'rule_based_mission_topic': '/rule_based/mission',
                    'ui_default_mode': 'RULE_BASED',
                }],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'fire1_xy', default_value='[1.0, 2.0]',
            description='불1 map 좌표 [x, y]'),
        DeclareLaunchArgument(
            'person_xy', default_value='[1.5, 1.5]',
            description='사람 map 좌표 [x, y]'),
        DeclareLaunchArgument(
            'fire2_xy', default_value='[3.0, 1.0]',
            description='불2 map 좌표 [x, y]'),
        DeclareLaunchArgument(
            'sweep_angle_deg', default_value='15.0',
            description='초기 좌우 스캔 각도(도)'),
        DeclareLaunchArgument(
            'sweep_dwell_sec', default_value='1.0',
            description='스캔 각 스텝 사이 대기 시간(초)'),
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='실제 YOLO 가중치 절대경로 (.onnx | .hef). '
                        '비우면 yolo_node 가 즉시 에러로 알림'),
        DeclareLaunchArgument(
            'class_names', default_value="['fire','person']",
            description='★ 학습 때 순서 그대로. 순서가 틀리면 불을 사람으로 발행'),
        DeclareLaunchArgument(
            'layout', default_value='auto',
            description='auto | v8 | end2end — 첫 실행 로그의 "레이아웃=" 확인 후 못박을 것'),
        DeclareLaunchArgument(
            'threads', default_value='3',
            description='Pi 5(4코어) 기준 권장값 — ROS·다른 프로세스와 코어 분배'),
        DeclareLaunchArgument(
            'conf', default_value='0.75',
            description='이 confidence 미만 검출은 버림 (0.0~1.0)'),
        DeclareLaunchArgument(
            'fire_min_score', default_value='0.0',
            description='이 점수 미만 화재 검출은 무시. 0.0=끔'),
        DeclareLaunchArgument(
            'fire_extinguished_ratio', default_value='0.1',
            description='관찰 구간 내 화재 프레임 비율이 이 값 미만이면 꺼짐 판정'),
        DeclareLaunchArgument(
            'overlay', default_value='true',
            description='박스가 구워진 /yolo/overlay 영상을 낼지 (rqt 확인용)'),
        DeclareLaunchArgument(
            'overlay_display_width', default_value='480',
            description='오버레이 전송 폭. 0이면 원본'),
        DeclareLaunchArgument(
            'overlay_max_fps', default_value='5.0',
            description='오버레이 발행 상한'),
        DeclareLaunchArgument(
            'start_ui_vision', default_value='true',
            description='false면 ui_stream_node(JPEG 인코딩)를 끄고 '
                        'firefighter_ui 영상 구독도 끈다'),
        DeclareLaunchArgument(
            'ui_host', default_value='0.0.0.0',
            description='0.0.0.0=LAN 어디서든 접속 가능. 이 Pi에서만 보려면 '
                        '127.0.0.1로 바꾸고 ui_allow_remote도 false로'),
        DeclareLaunchArgument('ui_port', default_value='8080'),
        DeclareLaunchArgument(
            'ui_allow_remote', default_value='true',
            description='true면 LAN 어디서든 웹 UI로 START/STOP 및 지도 열람 가능'),
        hardware,
        camera,
        slam,
        nav2,
        vision,
        frontier,
        frontier_state_controller,
        mission_stack,
        ui,
    ])
