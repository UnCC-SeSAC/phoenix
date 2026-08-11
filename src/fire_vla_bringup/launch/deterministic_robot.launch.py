from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os


def _validate_goal_owner(context):
    owner = LaunchConfiguration('goal_owner').perform(context)
    if owner not in {'frontier', 'mission'}:
        raise RuntimeError("goal_owner must be 'frontier' or 'mission'")
    return []


def generate_launch_description():
    uncc_launch = os.path.join(
        get_package_share_directory('uncc_example'),
        'launch',
        'uncc_frontier.launch.py',
    )
    owner = LaunchConfiguration('goal_owner')
    return LaunchDescription([
        DeclareLaunchArgument(
            'goal_owner',
            default_value='frontier',
            description='The only deterministic NavigateToPose goal owner',
        ),
        OpaqueFunction(function=_validate_goal_owner),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(uncc_launch),
            launch_arguments={
                'start_frontier': PythonExpression(["'", owner, "' == 'frontier'"]),
                'start_mission': PythonExpression(["'", owner, "' == 'mission'"]),
                'start_vision': 'false',
            }.items(),
        ),
    ])
