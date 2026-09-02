"""태스크② 노드 런치 — 2D 검출 + 뎁스 → base_link 3D 좌표.

  ros2 launch image_pipeline detection_3d.launch.py
  ros2 launch image_pipeline detection_3d.launch.py use_sim_time:=true   # bag 재생

bag 재생과 함께 쓸 때는 반드시 use_sim_time:=true
(`ros2 bag play --clock`과 짝. 안 맞추면 TF extrapolation 에러가 쏟아집니다.)

★ 거리를 재는 위치가 **클래스마다 다릅니다** (2026-08-26 실기 실측):

    fire   -> below   박스 대부분이 불꽃이라 박스 안에 대상 표면이 없습니다.
                      bottom으로 재니 불꽃 사이의 **배경(벽)**이 잡혀 멀게 나갔습니다.
    person -> bottom  below로 재니 사람 앞쪽 바닥을 재서 **가깝게** 나왔습니다.

  `region_by_class:="fire:below,person:bottom"` 이 기본값이고, 매핑에 없는
  클래스는 `region`(기본 `bottom`)을 씁니다. method는 자동으로 따라갑니다
  (below→max). **클래스 이름은 학습 라벨과 정확히 같아야** 하며, 틀리면
  예외 없이 기본값으로 떨어집니다 — 시작 로그에서 매핑을 확인하세요.

★ `fallback_regions`는 기본으로 비어 있습니다. `below`/`ring`은 대상이 아니라
  **주변**을 재고 서로 반대 방향으로 편향됩니다(below 가깝게 / ring 멀게).
  실측 전에 켜면 "확신에 찬 틀린 좌표"가 나갑니다 — `HANDOVER.md` 8장 참조.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ASCAMERA = '/ascamera/camera_publisher'


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'detections_topic', default_value='/yolo_result',
            description='YOLO 2D 박스 (vision_msgs/Detection2DArray)'),
        DeclareLaunchArgument(
            'depth_topic', default_value=f'{ASCAMERA}/depth0/image_raw'),
        DeclareLaunchArgument(
            'depth_info_topic', default_value=f'{ASCAMERA}/depth0/camera_info'),
        DeclareLaunchArgument(
            'color_info_topic', default_value='/image_enhanced/camera_info',
            description='박스가 있는 이미지의 K. 원본 K를 주면 거리가 배율만큼 틀립니다'),
        DeclareLaunchArgument(
            'rgb0_info_topic', default_value=f'{ASCAMERA}/rgb0/camera_info',
            description='★ 발행할 x,y의 기준 K. 메인이 역투영에 쓸 것과 같아야 합니다'),
        DeclareLaunchArgument('output_topic', default_value='/fire/detections'),
        DeclareLaunchArgument('status_topic', default_value='/fire/detections/status'),
        DeclareLaunchArgument(
            'region', default_value='bottom',
            description='매핑에 없는 클래스의 기본 영역. 화염 때문에 center 아님'),
        DeclareLaunchArgument(
            'region_by_class', default_value='fire:below,person:bottom',
            description='클래스별 영역. fire는 박스가 불로 차서 below, person은 bottom'),
        DeclareLaunchArgument(
            'band_offset', default_value='3.5',
            description='below 띠 시작 위치(박스높이 배수). 0=접지점, 3.5=촛대 받침'),
        DeclareLaunchArgument(
            'band_ratio', default_value='3.0',
            description='below 띠 두께(박스높이 배수)'),
        DeclareLaunchArgument(
            'fallback_regions', default_value='',
            description='예: "below,ring". 기본 꺼짐 — 켜기 전 HANDOVER 8장'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    node = Node(
        package='image_pipeline',
        executable='detection_3d_node',
        name='detection_3d_node',
        output='screen',
        parameters=[{
            'detections_topic': LaunchConfiguration('detections_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'depth_info_topic': LaunchConfiguration('depth_info_topic'),
            'color_info_topic': LaunchConfiguration('color_info_topic'),
            'rgb0_info_topic': LaunchConfiguration('rgb0_info_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'status_topic': LaunchConfiguration('status_topic'),
            'region': LaunchConfiguration('region'),
            'region_by_class': LaunchConfiguration('region_by_class'),
            'band_offset': LaunchConfiguration('band_offset'),
            'band_ratio': LaunchConfiguration('band_ratio'),
            'fallback_regions': LaunchConfiguration('fallback_regions'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription(args + [node])
