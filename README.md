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
ros2 topic echo /vla/person_report
ros2 topic echo /vla/person_report_result
```

### 3. Ubuntu PC: VLA Brain

```bash
source /opt/ros/jazzy/setup.bash
source ~/fire_vla_ws/install/setup.bash

ros2 launch fire_vla_bringup topic_bridge_vla.launch.py
```

현재 기본값은:

```text
llm_backend = mock
navigation_mode = TOPIC_BRIDGE
```

따라서 Qwen 없이도 ROS2/Nav2 연결부터 검증할 수 있습니다.

`llm_backend`의 허용값은 `mock`, `ollama`, `transformers`이며 기본값은 `mock`입니다. 현재 Transformers backend의 기본 Runtime 설정은 `Qwen/Qwen2.5-1.5B-Instruct`, `xpu:0`, `float32`입니다. CPU 자동 fallback은 사용하지 않습니다.

ROS2 Jazzy system Python과 XPU Python 환경은 분리되어 있으므로 Transformers를 사용할 때는 `vla_python_executable`에 XPU venv의 Python을 지정합니다. 이 prefix는 `VLAOrchestratorNode`에만 적용되며 다른 ROS Node의 interpreter는 바꾸지 않습니다.

```bash
ros2 launch fire_vla_bringup topic_bridge_vla.launch.py \
  llm_backend:=transformers \
  transformers_model_id:=Qwen/Qwen2.5-1.5B-Instruct \
  transformers_device:=xpu:0 \
  vla_python_executable:=<workspace>/.venv-xpu/bin/python
```

## Navigation goal ownership mode

SLAM, TF, Localization은 두 mode의 공통 기반이며, goal decision owner는 launch composition으로 하나만 활성화합니다. 별도 arbitration manager는 두지 않습니다.

- **DETERMINISTIC mode**: Frontier/StateManager/MissionExecutor가 `/navigate_to_pose` goal을 소유하며 VLA navigation goal sender는 끕니다.
- **VLA mode**: VLA Brain → `VLANavigationBridgeNode`가 `/navigate_to_pose` goal을 소유하며 Frontier와 MissionExecutor goal sender는 끕니다.

두 owner가 동시에 활성화되는 구성은 지원하지 않습니다. `VLAOrchestratorNode`의 `navigation_mode=MOCK|TOPIC_BRIDGE`는 Adapter 선택 parameter이며 위 system ownership mode와 같은 개념이 아닙니다. 현재 `topic_bridge_vla.launch.py`는 VLA-side sender만 구성하고 팀 Frontier/MissionExecutor를 함께 시작하지 않습니다.

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
  "frame_id": "map",
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

중요: `frame_id="map"`과 유한한 2D `map_position`이 필수입니다. `entity_id`는 optional이며, non-empty upstream ID는 보존하고 ID가 없으면 VLA boundary가 같은 class의 최근 2D 위치를 기준으로 MVP stable ID를 연결합니다. bbox/depth/camera-frame 3D/TF는 Perception 책임이며 bbox만 있는 YOLO 결과는 아직 연결되지 않습니다. 실제 팀 topic/message Adapter는 VLA-03B 범위입니다.

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

- LLM: 기본 backend는 `MockVLABrain`; `ollama`와 `transformers`도 선택 가능
- Spray: `MockSprayAdapter`
- Report: `report_mode=MOCK`은 `MockReportAdapter`; `TOPIC_BRIDGE`는 `TopicBridgePersonReportAdapter`
- YOLO → map 좌표 Adapter: 미구현

실제로 연결된 기능:

- VLA Action → Topic Bridge
- Humble Bridge → Nav2 `NavigateToPose`
- Nav2 Result → Jazzy VLA WorldModel
- TF `map → base_footprint` → VLA robot pose

## 검증 결과

현재 checkout에서 재검증한 결과:

```text
python3 -m pytest -q: 115 passed
colcon build --packages-select fire_vla_core fire_vla_bringup: PASS
코드 및 launch wiring 확인
Mock ActionDecision → `/vla/navigation_goal` ROS2 topic boundary publish: PASS
Deterministic REPORT_PERSON → `/vla/person_report` → result → WorldModel: PASS
```

이번 검증에서 재실행하지 않은 항목:

- 실제 Qwen XPU inference 및 ROS2 + Qwen runtime
- Jazzy ↔ Humble DDS discovery
- `/vla/*` String 토픽 양방향 통신
- Nav2 Action Server readiness
- TF `map → base_footprint`
- 실제 로봇 주행

VLA-01에서 실제 Nav2/Robot 없이 person/fire pose resolve와 `/vla/navigation_goal` 발행을 검증했습니다.

VLA-02에서는 deterministic `/vla/navigation_result`를 ROS2 topic boundary에 publish하여 `ActionResult → VLAOrchestrator → WorldModel` lifecycle, terminal status, cancel correlation, stale/duplicate 방어 및 다음 판단 가능 여부를 검증했습니다. 실제 `/navigate_to_pose` Action Server와 Robot은 사용하지 않았습니다.

VLA-03A에서 canonical `/vla/perception_observation`의 `frame_id=map`, timestamp, class, confidence, finite 2D position을 검증하고, upstream ID 우선 및 ID 없는 detection의 map-distance stable ID fallback을 구현했습니다. association radius는 0.5 m, recent candidate TTL은 2.0초이며 빠른 이동/교차/긴 occlusion/process restart에서는 ID switch가 가능한 MVP association입니다. 실제 YOLO/Depth/TF topic Adapter는 VLA-03B로 남아 있습니다.

VLA-04에서 `report_mode=TOPIC_BRIDGE` composition과 `/vla/person_report`, `/vla/person_report_result` JSON boundary를 추가했습니다. report payload의 person ID, map `(x,y)`, confidence는 WorldModel에서 가져오며 ACCEPTED 단계에서는 상태를 바꾸지 않습니다. correlated `SUCCEEDED` 결과에서만 `reported=true`가 되고, 실패·취소·timeout은 미보고 상태를 유지합니다. 실제 UI/외부 보고 backend는 사용하지 않았습니다.

Qwen2.5 XPU 및 ROS2 runtime smoke 이력은 `docs/INTEGRATION_REPORT.md`와 `docs/private/handoffs/HANDOFF_2026-08-07_VLA_BRAIN.md`에 현재 재검증 결과와 구분하여 기록합니다.
