from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os


def generate_launch_description():
    controller_share = get_package_share_directory('controller')
    peripherals_share = get_package_share_directory('peripherals')

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                controller_share,
                'launch',
                'controller.launch.py',
            )
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                peripherals_share,
                'launch',
                'lidar.launch.py',
            )
        )
    )

    return LaunchDescription([
        # Hiwonder launch files use this environment switch.
        SetEnvironmentVariable(
            name='need_compile',
            value='True',
        ),
        controller_launch,
        lidar_launch,
    ])
