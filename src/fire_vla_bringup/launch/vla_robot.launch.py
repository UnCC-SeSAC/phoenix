from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def _launch(package, filename, arguments=None):
    path = os.path.join(get_package_share_directory(package), 'launch', filename)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    return LaunchDescription([
        _launch(
            'uncc_example',
            'uncc_frontier.launch.py',
            {
                'start_frontier': 'false',
                'start_mission': 'false',
                'start_vision': 'false',
            },
        ),
        _launch(
            'fire_vla_bringup',
            'topic_bridge_vla.launch.py',
            {'start_perception_bridge': 'true'},
        ),
        _launch('uncc_example', 'vla_navigation_bridge.launch.py'),
    ])
