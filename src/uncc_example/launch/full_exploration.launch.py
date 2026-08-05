"""
Full real-robot exploration for:
  ROS 2 Humble
  Raspberry Pi 5
  Hiwonder + LD19

Order:
  1. Hiwonder controller + LD19
  2. slam_toolbox
  3. Nav2 navigation-only stack
  4. hazard map + frontier explorer
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(path, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        condition=condition,
    )


def generate_launch_description():
    share = get_package_share_directory('uncc_example')
    launch_dir = os.path.join(share, 'launch')

    start_hardware = LaunchConfiguration('start_hardware')
    start_slam = LaunchConfiguration('start_slam')
    start_nav2 = LaunchConfiguration('start_nav2')
    start_explorer = LaunchConfiguration('start_explorer')

    hardware = _include(
        os.path.join(launch_dir, 'hardware.launch.py'),
        IfCondition(start_hardware),
    )

    slam = TimerAction(
        period=3.0,
        actions=[
            _include(
                os.path.join(
                    launch_dir,
                    'slam_mapping.launch.py',
                ),
                IfCondition(start_slam),
            )
        ],
    )

    nav2 = TimerAction(
        period=7.0,
        actions=[
            _include(
                os.path.join(
                    launch_dir,
                    'nav2_online.launch.py',
                ),
                IfCondition(start_nav2),
            )
        ],
    )

    explorer = TimerAction(
        period=11.0,
        actions=[
            _include(
                os.path.join(
                    launch_dir,
                    'exploration.launch.py',
                ),
                IfCondition(start_explorer),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_hardware',
            default_value='true',
            description=(
                'Start Hiwonder controller and LD19. '
                'Set false if they are already running.'
            ),
        ),
        DeclareLaunchArgument(
            'start_slam',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'start_nav2',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'start_explorer',
            default_value='true',
        ),
        hardware,
        slam,
        nav2,
        explorer,
    ])
