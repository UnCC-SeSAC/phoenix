"""태스크① 전처리 노드 런치.

  ros2 launch image_pipeline preprocess.launch.py mode:=passthrough
  ros2 launch image_pipeline preprocess.launch.py mode:=full use_sim_time:=true

bag 재생과 함께 쓸 때는 반드시 use_sim_time:=true
(`ros2 bag play --clock`과 짝. 안 맞추면 TF extrapolation 에러가 쏟아집니다.)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('image_pipeline')
    default_params = os.path.join(pkg_share, 'config', 'preprocess.yaml')

    args = [
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'mode', default_value='full',
            description='passthrough | clahe | dehaze | full'),
        DeclareLaunchArgument('output_topic', default_value='/image_enhanced'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    node = Node(
        package='image_pipeline',
        executable='preprocess_node',
        name='rgb_preprocess_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                # YAML보다 나중에 오므로 런치 인자가 우선합니다.
                'mode': LaunchConfiguration('mode'),
                'output_topic': LaunchConfiguration('output_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    return LaunchDescription(args + [node])
