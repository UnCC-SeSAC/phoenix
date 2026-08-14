from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """
    fire_suppression_node 의 실제 GPIO 모션(펌프/서보)까지 검증하는
    하드웨어-인-더-루프 테스트용 launch. full_chain_dummy_test.launch.py
    와 달리 fire_suppression 은 더미로 대체하지 않고 실제 노드를 그대로
    띄운다 — 라즈베리파이(GPIO13 펌프, GPIO18 서보 배선)에서 실행해야
    한다.

    나머지 입력/판정은 여전히 더미다:
      - 비전(불 감지): image_pipeline 의 더미 체인(full_chain_check.launch.py)
      - Nav2 이동: navigate_to_pose_dummy_stub 이 goal 을 받으면 실제
        이동 없이 즉시 도착 처리
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
    """

    image_pipeline_share = get_package_share_directory('image_pipeline')

    dummy_vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            image_pipeline_share + '/launch/full_chain_check.launch.py'
        ),
    )

    # vision_detector/state_manager 둘 다 TF(카메라->map, base_footprint)가
    # 없으면 조용히 경고만 찍고 넘어가므로, 값은 아무 값이나 상관없이
    # 존재하기만 하면 된다.
    tf_map_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_base_footprint_dummy',
        arguments=['--frame-id', 'map', '--child-frame-id', 'base_footprint'],
    )

    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_camera_dummy',
        arguments=[
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'camera_depth_optical_frame',
        ],
    )

    vision_detector = Node(
        package='uncc_example',
        executable='vision_detector',
        name='vision_detector',
        output='screen',
        parameters=[{
            'camera_info_topic': '/image_enhanced/camera_info',
        }],
    )

    state_manager = Node(
        package='uncc_example',
        executable='state_manager',
        name='state_manager',
        output='screen',
    )

    mission_executor = Node(
        package='uncc_example',
        executable='mission_executor',
        name='mission_executor',
        output='screen',
    )

    navigate_to_pose_dummy = Node(
        package='uncc_example',
        executable='navigate_to_pose_dummy_stub',
        name='navigate_to_pose_dummy_stub',
        output='screen',
    )

    fire_status_dummy = Node(
        package='uncc_example',
        executable='fire_status_service_node_dummy_stub',
        name='fire_status_service_node',
        output='screen',
    )

    # 더미로 대체하지 않는 실제 노드 — GPIO13(펌프)/GPIO18(서보) 실물 구동
    fire_suppression_real = Node(
        package='uncc_example',
        executable='fire_suppression_node',
        name='fire_suppression_node',
        output='screen',
    )

    return LaunchDescription([
        dummy_vision,
        tf_map_to_base,
        tf_base_to_camera,
        vision_detector,
        state_manager,
        mission_executor,
        navigate_to_pose_dummy,
        fire_status_dummy,
        fire_suppression_real,
    ])
