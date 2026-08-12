"""전 구간 사슬 점검 — **카메라 프레임 입력부터 최종 JSON 발행까지** 더미로.

  ros2 launch image_pipeline full_chain_check.launch.py
  ros2 launch image_pipeline full_chain_check.launch.py haze:=0.35   # 디헤이즈가 일하는지
  ros2 launch image_pipeline full_chain_check.launch.py enhanced_width:=320
  ros2 launch image_pipeline full_chain_check.launch.py flame_hole:=true

```
fake_detection_node ─rgb0/image──▶ preprocess_node ─/image_enhanced─▶ yolo_node
        │                              (태스크①)                    (backend=stub)
        │                                                                 │
        └─depth0/image_raw + camera_info 3종───▶ detection_3d_node ◀──/yolo_result
                                                    (태스크②)
                                                        │
                                                        ▼
                                                 /fire/detections
```

`dummy_check.launch.py`와의 차이: 저건 **②만** 돌립니다(더미가 박스를 직접 냄).
이건 ①과 YOLO를 사이에 끼워 **네 노드가 실제로 이어지는지**를 봅니다.

★ 확인하는 값
--------------
더미가 시작 로그에 `[정답]`을 찍습니다. 기본 설정이면 `/fire/detections`에

    {"x": 320, "y": 240, "depth": 3.200, "depth_status": "ok"}

가 나와야 합니다. 스텁이 **화면 정중앙**에 박스를 놓고 더미 장면의 불도
정중앙에 있어서 둘이 만나야 정상입니다. `enhanced_width:=320`을 주면 축소본
좌표를 원본으로 되돌리는 경로까지 걸리는데, 그때도 **x는 320**이어야 합니다
(160이 나오면 되돌리기가 빠진 것).

⚠ 이건 검출 성능 시험이 아닙니다
--------------------------------
`backend:=stub`은 사진을 **보지 않고** 항상 정중앙에 박스를 놓습니다.
여기서 확인되는 것은 토픽 배선 · QoS · stamp 전파 · 좌표계 왕복이고,
"불을 불로 보는가"는 가중치가 있어야 합니다 (`HANDOVER.md` 7-6).

⚠ 컬러도 더미가 냅니다
----------------------
`fake_camera_node`를 따로 띄우지 않는 이유는 **stamp** 때문입니다. 컬러와
뎁스를 다른 노드가 각자 `now()`로 찍으면 stamp가 갈라지고, 그러면
`StampMonitor`가 실제 버그가 아닌 **가짜 경보**를 냅니다. 한 노드가 한 틱에
같은 stamp로 내야 감시가 의미를 가집니다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ASCAMERA = '/ascamera/camera_publisher'


def generate_launch_description():
    args = [
        DeclareLaunchArgument('distance_m', default_value='3.2',
                              description='정답 거리 (HP60C 스펙 상한 4m의 80%)'),
        DeclareLaunchArgument('haze', default_value='0.0',
                              description='합성 컬러의 연기 농도 0~1. 태스크①이 걷어냄'),
        DeclareLaunchArgument('mode', default_value='full',
                              description='태스크①: passthrough | clahe | dehaze | full'),
        DeclareLaunchArgument('enhanced_width', default_value='0',
                              description='>0이면 축소본 좌표 되돌리기까지 검증'),
        DeclareLaunchArgument('flame_hole', default_value='false',
                              description='true면 depth:null / unknown 이 나와야 정상'),
        DeclareLaunchArgument('fallback_regions', default_value=''),
        DeclareLaunchArgument('imgsz', default_value='640'),
        # ★ 더미 불꽃 크기를 **스텁 박스와 같게** 맞춥니다 (yolo.StubBackend 기본 40px).
        #   더미 쪽이 크면 `ring`(박스의 +25%) 이 뎁스 구멍 안에 갇혀서
        #   폴백이 영원히 실패합니다 — 코드가 아니라 장치 때문에 실패하는 것이라
        #   `fallback_regions` 를 시험할 수 없게 됩니다.
        DeclareLaunchArgument('box_size', default_value='40.0',
                              description='더미 불꽃 = 스텁 박스 크기 (px)'),
    ]

    # ① 의 입력(rgb0/image) + 뎁스 + camera_info 3종을 **한 틱 같은 stamp**로.
    #    검출과 /image_enhanced/camera_info 는 아래 노드들이 내므로 끕니다.
    dummy = Node(
        package='image_pipeline',
        executable='fake_detection_node',
        name='fake_detection_node',
        output='screen',
        parameters=[{
            'distance_m': LaunchConfiguration('distance_m'),
            'flame_hole': LaunchConfiguration('flame_hole'),
            'haze': LaunchConfiguration('haze'),
            'box_width': LaunchConfiguration('box_size'),
            'box_height': LaunchConfiguration('box_size'),
            'publish_color': True,
            'publish_detections': False,      # yolo_node 가 냅니다
            'publish_color_info': False,      # preprocess_node 가 냅니다
        }],
    )

    task1 = Node(
        package='image_pipeline',
        executable='preprocess_node',
        name='rgb_preprocess_node',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('mode'),
            'input_topic': f'{ASCAMERA}/rgb0/image',
            'camera_info_topic': f'{ASCAMERA}/rgb0/camera_info',
            'output_topic': '/image_enhanced',
            'output_camera_info_topic': '/image_enhanced/camera_info',
            # 0이면 축소 없음. 런치 인자로 320을 주면 좌표 되돌리기가 걸립니다.
            'process_width': LaunchConfiguration('enhanced_width'),
        }],
    )

    yolo = Node(
        package='image_pipeline',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        parameters=[{
            'backend': 'stub',                # ★ 가중치 없이 배선만
            'model_path': '',
            'imgsz': LaunchConfiguration('imgsz'),
            'class_names': ['fire'],
            'input_topic': '/image_enhanced',
            'detections_topic': '/yolo_result',
        }],
    )

    task2 = Node(
        package='image_pipeline',
        executable='detection_3d_node',
        name='detection_3d_node',
        output='screen',
        parameters=[{
            'fallback_regions': LaunchConfiguration('fallback_regions'),
        }],
    )

    return LaunchDescription(args + [dummy, task1, yolo, task2])
