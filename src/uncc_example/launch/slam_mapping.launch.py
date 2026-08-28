import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('uncc_example')
    params = os.path.join(share, 'config', 'slam.yaml')

    slam_cpu_cores = LaunchConfiguration('slam_cpu_cores')

    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        prefix=['taskset -c ', slam_cpu_cores],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_cpu_cores',
            default_value='1,2',
        ),
        slam,
    ])
