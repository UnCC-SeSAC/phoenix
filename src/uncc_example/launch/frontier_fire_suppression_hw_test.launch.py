import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """
    fire_suppression_node 의 실제 GPIO 모션(펌프/서보)과, frontier_explorer
    기반 자율 탐사까지 함께 검증하는 하드웨어-인-더-루프 테스트용 launch.
    full_chain_dummy_test.launch.py 와 달리 fire_suppression 은 더미로
    대체하지 않고 실제 노드를 그대로 띄운다 — 라즈베리파이(GPIO13 펌프,
    GPIO18 서보 배선)에서 실행해야 한다.

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

    나머지 입력/판정은 여전히 더미다:
      - 비전(불 감지): image_pipeline 의 더미 체인(full_chain_check.launch.py)
      - 불 꺼짐 판정(check_fire_status): fire_status_service_node_dummy_stub
        이 1차 호출은 안꺼짐, 2차 호출부터는 꺼짐으로 응답 (파라미터
        succeed_on_call 로 조절 가능)

    사용 예 (라즈베리파이에서):
        ros2 launch uncc_example frontier_fire_suppression_hw_test.launch.py

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
    image_pipeline_share = get_package_share_directory('image_pipeline')
    frontier_share = get_package_share_directory('frontier_exploration_ros2')
    launch_dir = os.path.join(uncc_share, 'launch')
    frontier_params = os.path.join(frontier_share, 'config', 'params.yaml')

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

    slam = TimerAction(
        period=3.0,
        actions=[include_launch('slam_mapping.launch.py')],
    )

    nav2 = TimerAction(
        period=7.0,
        actions=[include_launch('nav2_online.launch.py')],
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
    # 비전(더미) + 미션 스택
    # frontier_state_controller(t=12) 뒤에 시작해야 mission_executor 가
    # EXPLORING 진입 즉시 frontier 서비스를 찾을 수 있다 (nav2 의
    # bt_navigator 도 t=7 이후라 이 시점엔 이미 떠 있다).
    # =========================================

    # distance_m=0.29 : 목표거리(카메라 오프셋 0.0614 + 0.29 = 0.35m) 에서
    # xy_goal_tolerance(0.25m) 만큼 앞에서 멈추면 실제 전진거리 = 0.10m
    dummy_vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            image_pipeline_share + '/launch/full_chain_check.launch.py'
        ),
        launch_arguments={
            'distance_m': '0.29',
        }.items(),
    )

    # mecanum.xacro 실측 장착값(base_footprint->depth_cam) + REP-103
    # 카메라->광학 프레임 표준 회전
    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_camera_dummy',
        arguments=[
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'camera_depth_optical_frame',
            '--x', '0.061376',
            '--y', '-0.00013463',
            '--z', '0.121154',
            '--roll', '-1.5707963267948966',
            '--pitch', '0',
            '--yaw', '-1.5707963267948966',
        ],
    )

    vision_detector = Node(
        package='uncc_example',
        executable='vision_detector',
        name='vision_detector',
        output='both',
        parameters=[{
            'camera_info_topic': '/image_enhanced/camera_info',
        }],
    )

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

    fire_status_dummy = Node(
        package='uncc_example',
        executable='fire_status_service_node_dummy_stub',
        name='fire_status_service_node',
        output='both',
    )

    # 더미로 대체하지 않는 실제 노드 — GPIO13(펌프)/GPIO18(서보) 실물 구동
    fire_suppression_real = Node(
        package='uncc_example',
        executable='fire_suppression_node',
        name='fire_suppression_node',
        output='both',
    )

    mission_stack = TimerAction(
        period=13.0,
        actions=[
            dummy_vision,
            tf_base_to_camera,
            vision_detector,
            state_manager,
            mission_executor,
            fire_status_dummy,
            fire_suppression_real,
        ],
    )

    return LaunchDescription([
        hardware,
        slam,
        nav2,
        frontier,
        frontier_state_controller,
        mission_stack,
    ])
