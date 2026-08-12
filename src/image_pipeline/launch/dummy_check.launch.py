"""로봇·YOLO 없이 태스크②를 끝까지 돌려보는 런치.

  ros2 launch image_pipeline dummy_check.launch.py
  ros2 launch image_pipeline dummy_check.launch.py flame_hole:=true    # 발행 없어야 정상
  ros2 launch image_pipeline dummy_check.launch.py break_stamp_sec:=0.02  # 감시 확인

더미 노드가 시작 로그에 **[정답]** 값을 찍습니다. `/fire/detections`와
비교하세요. 기본 설정이면 `base_link=(+3.300, +0.000, +0.350)` 입니다.

역투영·TF는 메인이 하므로 이 노드에는 TF가 필요 없습니다. 더미가 내는
실기에서는 URDF/tf2가 정답입니다.

알아둘 것: 파이썬 rclpy는 0.61MB짜리 뎁스 이미지를 15Hz로 못 밀어냅니다(약 11Hz).
그래서 동기화율이 검출보다 낮게 나오는데 **태스크② 노드의 문제가 아닙니다.**
실기 드라이버는 C++입니다. 노드 통계의 "검출/뎁스/동기화" 세 수치로 구분됩니다.
"""


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('distance_m', default_value='3.2',
                              description='정답 거리 (HP60C 스펙 상한 4m의 80%)'),
        DeclareLaunchArgument('flame_hole', default_value='false',
                              description='true면 박스 영역 뎁스가 0 — 발행이 없어야 정상'),
        DeclareLaunchArgument('floor_height_m', default_value='0.0',
                              description='>0이면 바닥면 장면 (5-1 폴백 비교용)'),
        DeclareLaunchArgument('break_stamp_sec', default_value='0.0',
                              description='>0이면 검출 stamp를 밀어 계약 위반 재현'),
        DeclareLaunchArgument('fallback_regions', default_value=''),
    ]

    dummy = Node(
        package='image_pipeline',
        executable='fake_detection_node',
        name='fake_detection_node',
        output='screen',
        parameters=[{
            'distance_m': LaunchConfiguration('distance_m'),
            'flame_hole': LaunchConfiguration('flame_hole'),
            'floor_height_m': LaunchConfiguration('floor_height_m'),
            'break_stamp_sec': LaunchConfiguration('break_stamp_sec'),
        }],
    )

    task2 = Node(
        package='image_pipeline',
        executable='detection_3d_node',
        name='detection_3d_node',
        output='screen',
        parameters=[{
            # 2026-08-10 계약 변경으로 TF가 필요 없어졌습니다 — 역투영은 메인 몫.
            'fallback_regions': LaunchConfiguration('fallback_regions'),
        }],
    )

    return LaunchDescription(args + [dummy, task2])
