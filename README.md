# Fire VLA Brain + uncc_example ROS2 Integration Overlay

이 산출물은 다음 두 코드베이스를 연결한 **통합 오버레이**입니다.

- `uncc_example`: Raspberry Pi 5 / ROS2 Humble / SLAM Toolbox / Nav2
- `fire_vla_core`: Ubuntu PC / ROS2 Jazzy / VLA Brain Subsystem

전체 `src_0805`를 복제하지 않고, 변경되거나 추가된 관련 패키지만 포함합니다.

## 통합 방식

ROS2 Jazzy와 Humble 사이에서 Nav2 Action을 직접 호출하지 않고, 표준 `std_msgs/String` JSON 토픽을 사용하는 Topic Bridge를 추가했습니다.

```text
Ubuntu PC / ROS2 Jazzy

Mission + Semantic Observation
        ↓
fire_vla_core / VLAOrchestratorNode
        ↓
/vla/navigation_goal   (JSON String)
        │
        │ DDS / 동일 네트워크
        ▼
Raspberry Pi / ROS2 Humble

uncc_example / VLANavigationBridgeNode
        ↓
Nav2Navigator
        ↓
/navigate_to_pose
        ↓
Nav2 → /cmd_vel → Robot
        ↓
/vla/navigation_result (JSON String)
        │
        ▼
Ubuntu PC WorldModel 갱신
```

Pi Bridge는 TF `map → base_footprint`를 읽어 다음 토픽도 발행합니다.

```text
/vla/robot_pose_json
```

## 추가된 주요 파일

```text
src/fire_vla_core/fire_vla_core/ros/
├── orchestrator_node.py
├── topic_bridge_navigation_adapter.py
└── demo_input_node.py

src/uncc_example/uncc_example/
└── vla_navigation_bridge_node.py

src/uncc_example/launch/
└── vla_navigation_bridge.launch.py

src/fire_vla_bringup/launch/
├── local_mock_vla.launch.py
├── topic_bridge_vla.launch.py
└── topic_bridge_demo.launch.py
```

## 기존 워크스페이스에 적용

현재 ROS2 워크스페이스가 `~/ros2_ws/src`라면:

```bash
cd ~/ros2_ws/src

# 기존 uncc_example은 백업 후 교체하거나 diff를 확인하세요.
cp -a uncc_example uncc_example.backup.$(date +%Y%m%d_%H%M%S)

cp -a <이_폴더>/src/uncc_example ./
cp -a <이_폴더>/src/fire_vla_core ./
cp -a <이_폴더>/src/fire_vla_interfaces ./
cp -a <이_폴더>/src/fire_vla_bringup ./
```

## Raspberry Pi / ROS2 Humble 빌드

Pi에서는 Nav2 Bridge를 포함한 `uncc_example`만 필수입니다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install \
  --packages-select uncc_example

source install/setup.bash
```

## Ubuntu PC / ROS2 Jazzy 빌드

```bash
mkdir -p ~/fire_vla_ws/src
cp -a <이_폴더>/src/fire_vla_core ~/fire_vla_ws/src/
cp -a <이_폴더>/src/fire_vla_interfaces ~/fire_vla_ws/src/
cp -a <이_폴더>/src/fire_vla_bringup ~/fire_vla_ws/src/

cd ~/fire_vla_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Custom interface 패키지는 현재 통합 경로에서 필수로 사용하지 않지만 향후 팀 계약용으로 유지했습니다.

## 실행 순서

### 1. Pi: 기존 Hardware + SLAM + Nav2

`uncc_example/README.md`에 따라 각각 확인합니다.

```bash
ros2 launch uncc_example slam_mapping.launch.py
ros2 launch uncc_example nav2_online.launch.py
```

이미 controller/LiDAR/SLAM/Nav2가 실행 중이면 중복 실행하지 마세요.

### 2. Pi: VLA Navigation Bridge

```bash
ros2 launch uncc_example vla_navigation_bridge.launch.py
```

확인:

```bash
ros2 topic echo /vla/robot_pose_json --once
ros2 topic echo /vla/navigation_goal
ros2 topic echo /vla/navigation_result
```

### 3. Ubuntu PC: VLA Brain

```bash
source /opt/ros/jazzy/setup.bash
source ~/fire_vla_ws/install/setup.bash

ros2 launch fire_vla_bringup topic_bridge_vla.launch.py
```

현재 기본값은:

```text
use_mock_llm = true
navigation_mode = TOPIC_BRIDGE
```

따라서 Qwen 없이도 ROS2/Nav2 연결부터 검증할 수 있습니다.

## 입력 토픽

### Mission

```text
/vla/mission
std_msgs/msg/String
```

JSON:

```json
{
  "mission_id": "mission_001",
  "text": "인명을 우선 확인해."
}
```

### Semantic Observation

```text
/vla/perception_observation
std_msgs/msg/String
```

JSON:

```json
{
  "timestamp": "2026-08-06T01:00:00+00:00",
  "frame_valid": true,
  "detector_healthy": true,
  "detections": [
    {
      "entity_id": "person_01",
      "class_name": "person",
      "confidence": 0.95,
      "map_position": {"x": 2.0, "y": 0.0, "yaw": 0.0}
    }
  ]
}
```

중요: `map_position`은 이미 `map` 좌표로 변환된 값이어야 합니다. bbox만 있는 YOLO 결과는 아직 연결되지 않습니다.

## 안전한 Demo Input

`topic_bridge_demo.launch.py`는 실제 Nav2 goal을 발행할 수 있습니다.

```bash
ros2 launch fire_vla_bringup topic_bridge_demo.launch.py
```

기본 목표:

```text
fire:  (1.0, 0.0)
person:(2.0, 0.0)
```

**실물 로봇에서는 좌표를 현장 지도에 맞게 바꾸고, 매우 짧은 거리·저속·비상정지 준비 상태에서만 실행하세요.**

초기에는 `local_mock_vla.launch.py`로 물리 이동 없이 확인하는 것을 권장합니다.

```bash
ros2 launch fire_vla_bringup local_mock_vla.launch.py
```

## 확인 토픽

```bash
ros2 topic echo /vla/action_validated
ros2 topic echo /vla/world_model
ros2 topic echo /vla/navigation_goal
ros2 topic echo /vla/navigation_result
```

## 현재 Mock인 기능

- LLM: `MockVLABrain`
- Spray: `MockSprayAdapter`
- Report: `MockReportAdapter`
- YOLO → map 좌표 Adapter: 미구현

실제로 연결된 기능:

- VLA Action → Topic Bridge
- Humble Bridge → Nav2 `NavigateToPose`
- Nav2 Result → Jazzy VLA WorldModel
- TF `map → base_footprint` → VLA robot pose

## 검증 결과

이 환경에서는 ROS2 런타임이 없어 실제 `colcon build`와 Nav2 실행은 수행하지 못했습니다.

검증 완료:

```text
17 Python tests passed
전체 Python compileall 성공
ROS 비설치 환경에서 fire_vla_core.ros.orchestrator_node import 성공
```

실제 장비에서 확인해야 할 것:

- Jazzy ↔ Humble DDS discovery
- `/vla/*` String 토픽 양방향 통신
- Nav2 Action Server readiness
- TF `map → base_footprint`
- 실제 로봇 주행
