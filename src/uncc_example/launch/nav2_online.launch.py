"""
Start the Hiwonder Nav2 navigation stack WITHOUT AMCL/map_server.

This is important during online SLAM:
  slam_toolbox publishes map -> odom
  AMCL must not publish another map -> odom at the same time.

The launch reuses the current Hiwonder:
  navigation/launch/include/navigation_base.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('uncc_example')
    navigation_share = get_package_share_directory('navigation')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_teb = LaunchConfiguration('use_teb')

    container = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[
            params_file,
            {'autostart': True},
        ],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        output='screen',
    )

    navigation_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_share,
                'launch',
                'include',
                'navigation_base.launch.py',
            )
        ),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'false',
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'params_file': params_file,
            'container_name': 'nav2_container',
            'use_teb': use_teb,
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='need_compile',
            value='True',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                package_share,
                'config',
                'nav2_params.yaml',
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'use_teb',
            default_value='false',
        ),
        container,
        navigation_base,
    ])
