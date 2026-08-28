from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    controller_share = get_package_share_directory('controller')
    peripherals_share = get_package_share_directory('peripherals')

    hardware_cpu_cores = LaunchConfiguration('hardware_cpu_cores', default='0')

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                controller_share,
                'launch',
                'controller.launch.py',
            )
        ),
        launch_arguments={
            'hardware_cpu_cores': hardware_cpu_cores,
        }.items(),
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
        DeclareLaunchArgument(
            'hardware_cpu_cores',
            default_value='0',
        ),
        controller_launch,
        lidar_launch,
    ])
