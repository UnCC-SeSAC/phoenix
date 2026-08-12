from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """
    비전 → state_manager → mission_executor 결정/실행 로직 전체 사슬을
    실제 이동/진압 없이 로그로만 확인하는 테스트 전용 launch.

    image_pipeline 의 더미 비전 체인(full_chain_check.launch.py)을
    입력으로 쓰고, Nav2/fire_suppression 은 goal 을 받으면 즉시
    성공 처리하는 더미로 대체한다. SLAM/Nav2/하드웨어 스택은 전혀
    안 뜬다 — fire_extinguisher.launch.py 와 같은 원칙.

    사용 예:
        ros2 launch uncc_example full_chain_dummy_test.launch.py

    확인할 것:
        ros2 topic echo /mission/state
        ros2 topic echo /vision/detections
        # mission_executor 터미널 로그에서
        #   [FIRE_DETECTED] target=... -> Nav2 목적지 도착
        #   -> fire_suppression N차 -> 성공/실패 로그까지 이어지는지
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

    fire_suppression_dummy = Node(
        package='uncc_example',
        executable='fire_suppression_node_dummy_stub',
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
        fire_suppression_dummy,
    ])
