from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    lidar_frame = LaunchConfiguration('lidar_frame', default='base_laser')
    scan_raw = LaunchConfiguration('scan_raw', default='scan_raw')
    smoke_filter_intensity_threshold = LaunchConfiguration(
        'smoke_filter_intensity_threshold', default='200.0')
    lidar_frame_arg = DeclareLaunchArgument('lidar_frame', default_value=lidar_frame)
    scan_raw_arg = DeclareLaunchArgument('scan_raw', default_value=scan_raw)
    smoke_filter_intensity_threshold_arg = DeclareLaunchArgument(
        'smoke_filter_intensity_threshold',
        default_value=smoke_filter_intensity_threshold,
        description='LD19 intensity가 이 값 미만인 점은 연기로 보고 버린다 (0~255 추정 범위).',
    )

    # 드라이버는 필터를 거치기 전 이름으로 내보내고, 다른 모든 노드가
    # 지금까지 구독하던 scan_raw는 필터를 거친 결과로 채운다 — 이렇게
    # 하면 slam_toolbox/rf2o_laser_odometry 등 하류 쪽은 하나도 안
    # 건드려도 된다.
    scan_raw_unfiltered = [scan_raw, '_unfiltered']

    ld19_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD19',
        output='screen',
        # RESET 버튼이 이 프로세스를 pkill로 죽여서 재시작을 유도한다
        # (state_manager._restart_lidar 참고) — respawn이 없으면 그냥
        # 죽은 채로 끝나버린다.
        respawn=True,
        parameters=[
            {
                'topic_name': 'scan',
                'product_name': 'LDLiDAR_LD19',
                'port_baudrate': 230400,
                'port_name': '/dev/ldlidar',
                'frame_id': lidar_frame,
                'laser_scan_dir': True,
                'enable_angle_crop_func': False,
                'angle_crop_min': 135.0,
                'angle_crop_max': 225.0
            }
        ],
        remappings=[('scan', scan_raw_unfiltered)]
    )

    smoke_filter_node = Node(
        package='peripherals',
        executable='lidar_smoke_filter',
        name='lidar_smoke_filter',
        output='screen',
        parameters=[{
            'input_topic': scan_raw_unfiltered,
            'output_topic': scan_raw,
            'intensity_threshold': ParameterValue(
                smoke_filter_intensity_threshold, value_type=float),
        }],
    )

    return LaunchDescription([
        lidar_frame_arg,
        scan_raw_arg,
        smoke_filter_intensity_threshold_arg,
        ld19_node,
        smoke_filter_node,
    ])

if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
