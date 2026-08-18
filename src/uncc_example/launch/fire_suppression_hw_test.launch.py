import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """
    fire_suppression_node 의 실제 GPIO 모션(펌프/서보)까지 검증하는
    하드웨어-인-더-루프 테스트용 launch. full_chain_dummy_test.launch.py
    와 달리 fire_suppression 은 더미로 대체하지 않고 실제 노드를 그대로
    띄운다 — 라즈베리파이(GPIO13 펌프, GPIO18 서보 배선)에서 실행해야
    한다.

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

    나머지 입력/판정은 여전히 더미다:
      - 비전(불 감지): image_pipeline 의 더미 체인(full_chain_check.launch.py)
      - 불 꺼짐 판정(check_fire_status): fire_status_service_node_dummy_stub
        이 1차 호출은 안꺼짐, 2차 호출부터는 꺼짐으로 응답 (파라미터
        succeed_on_call 로 조절 가능)

    control_exploration 서비스(frontier_exploration_ros2)는 띄우지 않는다
    — fire_suppression_node 는 이 서비스가 없으면 2초 대기 후 경고만
    찍고 탐사 제어 없이 계속 진행하도록 이미 되어 있다.

    사용 예 (라즈베리파이에서):
        ros2 launch uncc_example fire_suppression_hw_test.launch.py

    확인할 것:
        ros2 topic echo /mission/state
        # mission_executor 터미널 로그:
        #   [FIRE_DETECTED] target=... -> Nav2 목적지 도착
        #   -> fire_suppression 1차 ... 2차 ... -> 성공 로그
        # fire_suppression_node 터미널: 실제 펌프/서보 구동 로그 +
        #   1차 판별 결과: 안꺼짐 -> 2차 판별 결과: 꺼짐
        # rviz2 등으로 map/odom/base_footprint TF, /scan, 실제 이동 확인
    """

    uncc_share = get_package_share_directory('uncc_example')
    image_pipeline_share = get_package_share_directory('image_pipeline')
    launch_dir = os.path.join(uncc_share, 'launch')

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
    # 비전(더미) + 미션 스택
    # nav2 가 뜬 뒤(t=7) 에 시작해야 mission_executor 가 목표를 보낼 때
    # bt_navigator 가 이미 떠 있다.
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
        period=8.0,
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
        mission_stack,
    ])
