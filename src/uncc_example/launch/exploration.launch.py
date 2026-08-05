import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('uncc_example')
    params = os.path.join(
        share,
        'config',
        'exploration.yaml',
    )

    hazard_map = Node(
        package='uncc_example',
        executable='hazard_map_node',
        name='hazard_map_node',
        output='screen',
        parameters=[params],
    )

    explorer = Node(
        package='uncc_example',
        executable='explorer_node',
        name='explorer_node',
        output='screen',
        parameters=[params],
    )

    return LaunchDescription([
        hazard_map,
        explorer,
    ])
