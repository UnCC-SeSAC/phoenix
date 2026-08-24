"""태스크② 노드 런치 — 2D 검출 + 뎁스 → base_link 3D 좌표.

  ros2 launch image_pipeline detection_3d.launch.py
  ros2 launch image_pipeline detection_3d.launch.py use_sim_time:=true   # bag 재생

bag 재생과 함께 쓸 때는 반드시 use_sim_time:=true
(`ros2 bag play --clock`과 짝. 안 맞추면 TF extrapolation 에러가 쏟아집니다.)

★ `region` 기본은 **`bottom`**입니다 (2026-08-24 실기 실측 반영). 성냥불 위에서
  뎁스가 안 나오는 것을 확인해서, 박스 중앙(=불꽃)을 재던 `center`를 내렸습니다.
  `bottom`은 박스 **안**이라 여전히 대상 자체를 읽습니다.

★ `fallback_regions`는 기본으로 비어 있습니다. `below`/`ring`은 대상이 아니라
  **주변**을 재고 서로 반대 방향으로 편향됩니다(below 가깝게 / ring 멀게).
  실측 전에 켜면 "확신에 찬 틀린 좌표"가 나갑니다 — `HANDOVER.md` 8장 참조.
  단 화염이 박스를 가득 채우면 `bottom`도 막히므로, 그런 장면이 잦으면
  `fallback_regions:=below,ring`을 검토하세요.
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
            description='거리를 뽑을 영역. 화염 위 뎁스가 비어서 center 아님'),
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
            'fallback_regions': LaunchConfiguration('fallback_regions'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription(args + [node])
