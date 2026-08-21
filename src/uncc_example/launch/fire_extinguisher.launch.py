from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    화재진압 action server 격리 launch.

    SLAM/Nav2/탐사 전체 스택을 켜지 않고 화재진압 관련 노드만 띄운다.
    frontier_exploration_ros2(control_exploration)나 YOLO 화재 노드가
    안 떠 있어도 fire_suppression_node가 알아서 경고만 찍고 넘어가도록
    이미 만들어뒀기 때문에, 이 launch 만으로 격리된 상태에서 테스트 가능.

    사용 예:
        ros2 launch uncc_example fire_extinguisher.launch.py

        하드웨어 작동 승인 없이 suppress_fire goal을 보내지 말 것.
    """
    return LaunchDescription([
        Node(
            package='uncc_example',
            executable='fire_status_service_node',
            name='fire_status_service_node',
            output='screen',
        ),
        Node(
            package='uncc_example',
            executable='fire_suppression_node',
            name='fire_suppression_node',
            output='screen',
        ),
        Node(
            package='uncc_example',
            executable='vla_spray_bridge',
            name='vla_spray_bridge',
            output='screen',
            parameters=[{'max_attempts_per_command': 1}],
        ),
    ])
