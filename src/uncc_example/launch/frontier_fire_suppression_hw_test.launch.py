import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    카메라 -> YOLO26 -> 탐사/이동 -> 진압(GPIO) -> YOLO26로 꺼짐 재확인까지
    더미 없이 실제 노드로 도는 하드웨어-인-더-루프 테스트용 launch.
    탐사용 감지와 진압 후 상태확인 둘 다 같은 YOLO26 결과를 쓴다
    (fire_detections_bridge 가 /fire/detections 를 fire_status_service_node
    용 포맷으로 재발행 — 모델을 두 번 안 띄움).

    Nav2 이동도 더미가 아니라 실제 스택을 띄운다 (uncc_frontier.launch.py
    와 동일한 조합):
      - hardware.launch.py: 실제 모터/오도메트리(controller) + 라이다
      - slam_mapping.launch.py: slam_toolbox 가 실시간으로 map->odom 발행
      - nav2_online.launch.py: 실제 controller_server/planner_server/
        bt_navigator (AMCL/map_server 는 안 씀 — slam_toolbox 와
        map->odom 이 겹치면 안 되므로)
    mission_executor 는 기존 그대로 "navigate_to_pose" 액션을 호출하며,
    더미 스텁과 이름/타입이 같아 코드 수정 없이 실제 bt_navigator 로
    대체된다. 로봇이 실제로 주행하니 테스트 전 충분한 공간을 확보할 것.

    frontier_explorer(frontier_exploration_ros2) + frontier_state_controller
    도 uncc_frontier.launch.py 와 동일한 파라미터/순서(nav2 -> frontier ->
    frontier_state_controller -> mission)로 함께 띄운다. mission_executor
    는 EXPLORING 상태에서 frontier_state_controller 를 통해 탐사 시작을,
    FIRE/PERSON/RETURNING 상태에서는 정지를 요청하고, fire_suppression_node
    는 진압 중 raw control_exploration 서비스를 직접 호출해 탐사를
    정지시킨다 (avoidance_manager 는 프로덕션 기본값과 동일하게 뺐다 —
    회피는 Nav2 local planner 가 담당).

    비전도 실제 체인이다 (uncc_frontier.launch.py 의 start_vision:=true 와
    같은 조합, full_chain_check.launch.py 의 더미 대신):
      - peripherals/depth_camera.launch.py(ascamera) 가 rgb0/depth0 를
        실제로 발행
      - image_pipeline: preprocess_node -> yolo_node(실제 model_path) ->
        detection_3d_node 가 /fire/detections 를 픽셀+depth 로 발행
      - uncc_example/vision_detector 가 camera_info+TF 로 map 좌표로
        변환해 /vision/detections 로 재발행 (state_manager 가 구독)
      model_path 는 launch 인자로 반드시 넘겨야 한다 (.onnx 권장 — 로봇에서
      torch/ultralytics 불필요).

    사용 예 (라즈베리파이에서, Hailo):
        ros2 launch uncc_example frontier_fire_suppression_hw_test.launch.py \\
            model_path:=/home/lemma/Hailo/models/baseline_yolo26_neural_norm.hef \\
            class_names:="['fire','person']"
    (.hef 후처리용 onnx/config json은 같은 폴더에서 자동으로 찾습니다 —
     config_onnx_best_sim.json / best_sim_postprocess.onnx 라는 이름이어야 함)

    확인할 것:
        ros2 topic echo /mission/state
        ros2 service list | grep control_exploration
        # mission_executor 터미널 로그:
        #   [EXPLORING] -> frontier 로 이동 (탐사 시작 로그)
        #   [FIRE_DETECTED] target=... -> Nav2 목적지 도착
        #   -> fire_suppression 1차 ... 2차 ... -> 성공 로그
        # fire_suppression_node 터미널: 실제 펌프/서보 구동 로그 +
        #   1차 판별 결과: 안꺼짐 -> 2차 판별 결과: 꺼짐
        # FIRE_DETECTED 진입 시 frontier 탐사가 멈추고, 진압 종료 후
        #   다시 EXPLORING 으로 돌아가면 탐사가 재개되는지 확인
        # rviz2 등으로 map/odom/base_footprint TF, /scan, 실제 이동 확인
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

    # depth_camera.launch.py 는 os.environ['need_compile']/['DEPTH_CAMERA_TYPE']
    # 를 읽는다 — need_compile 은 hardware.launch.py 가 이미 True 로 설정.
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
    # 비전 (image_pipeline 실제 체인 + vision_detector)
    # 카메라(t=1)와 image_enhanced/camera_info 를 쓰는 preprocess 이후
    # 단계라 nav2(t=7) 다음에 띄운다. mission_stack(t=13) 전에는 뜬다.
    # =========================================

    vision = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='image_pipeline',
                executable='preprocess_node',
                name='rgb_preprocess_node',
                output='screen',
                parameters=[
                    os.path.join(image_pipeline_share, 'config', 'preprocess.yaml'),
                    {
                        # YAML(더미 카메라 토픽)보다 나중에 와서 실카메라로 덮어씀.
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
                    'input_topic': '/image_enhanced',
                    'detections_topic': '/yolo_result',
                }],
            ),
            Node(
                package='image_pipeline',
                executable='detection_3d_node',
                name='detection_3d_node',
                output='screen',
                parameters=[{
                    'detections_topic': '/yolo_result',
                    'depth_topic': f'{ASCAMERA}/depth0/image_raw',
                    'depth_info_topic': f'{ASCAMERA}/depth0/camera_info',
                    'color_info_topic': '/image_enhanced/camera_info',
                    'rgb0_info_topic': f'{ASCAMERA}/rgb0/camera_info',
                    'output_topic': '/fire/detections',
                }],
            ),
            Node(
                package='uncc_example',
                executable='vision_detector',
                name='vision_detector',
                output='screen',
                parameters=[{
                    'camera_info_topic': '/image_enhanced/camera_info',
                    'depth_frame_id': 'ascamera_color_0',
                }],
            ),
        ],
    )

    # =========================================
    # Frontier Explorer + Frontier State Controller
    # (uncc_frontier.launch.py 와 동일한 파라미터/타이밍)
    # =========================================

    frontier = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='frontier_exploration_ros2',
                executable='frontier_explorer',
                name='frontier_explorer',
                output='both',
                parameters=[
                    frontier_params,
                    {
                        # avoidance_manager / fire_suppression_node 에서
                        # STOP / START 를 호출하기 위해 필수
                        'control_service_enabled': True,
                        # Frontier Controller 로 진행하기 때문에 False 로
                        'autostart': False,
                        # Raspberry Pi 5 에서는 먼저 가볍게 시작
                        'mrtsp_solver': 'greedy',
                        'map_processing_rate_hz': 0.5,
                        # 처음에는 기능을 단순하게
                        'goal_preemption_enabled': False,
                        # 탐사 완료 후, 시작지점 복귀 True
                        'return_to_start_on_complete': True,
                    },
                ],
            ),
        ],
    )

    frontier_state_controller = TimerAction(
        period=12.0,
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
    # 미션 스택
    # frontier_state_controller(t=12) 뒤에 시작해야 mission_executor 가
    # EXPLORING 진입 즉시 frontier 서비스를 찾을 수 있다 (nav2 의
    # bt_navigator 도 t=7 이후라 이 시점엔 이미 떠 있다).
    # =========================================

    state_manager = Node(
        package='uncc_example',
        executable='state_manager',
        name='state_manager',
        output='both',
    )

    mission_executor = Node(
        package='uncc_example',
        executable='mission_executor',
        name='mission_executor',
        output='both',
    )

    # detection_3d_node(/fire/detections, 실제 YOLO26 인식)를
    # fire_status_service_node가 기대하는 형식(/yolo_result_fire,
    # interfaces/msg/ObjectsInfo)으로 재발행 — 모델을 두 번 안 띄우고
    # 같은 실제 인식 결과를 재사용한다.
    fire_detections_bridge = Node(
        package='uncc_example',
        executable='fire_detections_bridge',
        name='fire_detections_bridge',
        output='both',
    )

    # 더미 아닌 진짜 노드 — 위 브리지가 주는 실제 YOLO 인식 이력으로
    # check_fire_status 를 판정한다.
    fire_status_real = Node(
        package='uncc_example',
        executable='fire_status_service_node',
        name='fire_status_service_node',
        output='both',
    )

    # 실제 GPIO13(펌프)/GPIO18(서보) 구동 노드.
    fire_suppression_real = Node(
        package='uncc_example',
        executable='fire_suppression_node',
        name='fire_suppression_node',
        output='both',
    )

    mission_stack = TimerAction(
        period=13.0,
        actions=[
            state_manager,
            mission_executor,
            fire_detections_bridge,
            fire_status_real,
            fire_suppression_real,
        ],
    )

    return LaunchDescription([
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
        hardware,
        camera,
        slam,
        nav2,
        vision,
        frontier,
        frontier_state_controller,
        mission_stack,
    ])
