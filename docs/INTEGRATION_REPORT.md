# 통합 변경 보고서

## 기준으로 사용한 코드

- `src_0805/src/uncc_example/README.md`
- `uncc_example.Nav2Navigator`
- Sprint A 리팩토링 VLA Core

## 연결 지점

`uncc_example`이 제공하는 실제 Nav2 Action 이름:

```text
/compute_path_to_pose
/navigate_to_pose
```

이번 통합에서는 VLA가 `/navigate_to_pose`를 직접 호출하지 않고 Pi의 Bridge가 호출하도록 구성했습니다.

## 이 방식을 선택한 이유

1. Ubuntu PC는 ROS2 Jazzy, Pi는 ROS2 Humble입니다.
2. Core는 Nav2와 ROS2 배포판을 몰라야 합니다.
3. `std_msgs/String` 계약은 임시 통합 테스트에 단순합니다.
4. 기존 `Nav2Navigator`는 이미 비동기 Action Client와 cancel을 구현합니다.
5. 실제 팀 인터페이스가 확정되면 JSON String을 custom msg로 교체할 수 있습니다.

## 수정한 기존 파일

- `uncc_example/setup.py`
  - `vla_navigation_bridge` entry point 추가
- `fire_vla_core/setup.py`
  - `vla_demo_input` entry point 추가
- `fire_vla_core/ros/orchestrator_node.py`
  - `MOCK` / `TOPIC_BRIDGE` Navigation composition 지원
  - robot pose를 JSON String으로 수신

## 새 파일

- `fire_vla_core/ros/topic_bridge_navigation_adapter.py`
- `fire_vla_core/ros/demo_input_node.py`
- `uncc_example/uncc_example/vla_navigation_bridge_node.py`
- `uncc_example/launch/vla_navigation_bridge.launch.py`
- `fire_vla_bringup/launch/topic_bridge_vla.launch.py`
- `fire_vla_bringup/launch/topic_bridge_demo.launch.py`
- `fire_vla_bringup/launch/local_mock_vla.launch.py`

## 의도적으로 하지 않은 것

- `ExplorerNode` 내부 로직 수정
- Frontier score에 VLA MissionValue 직접 결합
- YOLO bbox → map 좌표 변환
- Pump 실제 제어
- Nav2와 SLAM launch 자동 중복 실행

이 범위는 현재 요구사항보다 크거나 외부 계약이 미확정입니다.

## Navigation ownership 및 VLA-02 result lifecycle

Navigation goal owner는 system mode로 분리합니다. DETERMINISTIC mode에서는 Frontier/StateManager/MissionExecutor만 goal을 보내고 VLA sender를 끕니다. VLA mode에서는 VLA Brain과 `VLANavigationBridgeNode`만 goal을 보내고 Frontier/MissionExecutor sender를 끕니다. 동시 owner 구성과 별도 arbitration manager는 지원하지 않습니다. Core node의 `MOCK|TOPIC_BRIDGE` parameter는 Adapter composition 선택이며 system ownership mode와 구분합니다.

VLA-02에서 Humble bridge의 Nav2 `GoalStatus` 정규화, action ID cancel correlation, Jazzy `TopicBridgeNavigationAdapter` result parsing, WorldModel terminal lifecycle을 unit/integration test로 확인했습니다. 미등록 action ID 결과는 `UNRELATED_RESULT_IGNORED`로 차단하고, 동일 terminal result는 한 번만 적용합니다. ROS2 Jazzy smoke에서 deterministic `/vla/navigation_result`의 SUCCEEDED가 `current_action`을 해제하고 `last_action`을 갱신하는 것을 확인했습니다. 실제 Nav2 Action Server와 Robot은 사용하지 않았습니다.

## VLA-03A canonical perception 및 stable ID

Canonical boundary는 `/vla/perception_observation` `std_msgs/String` JSON을 유지합니다. `frame_id="map"`, timezone-aware timestamp, person/fire class, `[0,1]` confidence와 finite 2D map `(x,y)`를 요구합니다. non-empty upstream `entity_id`는 그대로 보존하고, ID가 없을 때만 같은 class의 최근 WorldModel entity를 0.5 m radius/2.0초 TTL로 nearest association합니다. 한 batch에서 ID는 한 번만 사용하며 거리 동률은 entity ID 순서로 결정합니다.

이는 tracker가 아닌 MVP fallback입니다. 빠른 이동, 근접 교차, 긴 occlusion, process restart에서는 ID switch가 가능하며 전역 영구 ID를 보장하지 않습니다. Depth 기반 camera-frame 3D point는 최종 2D map 위치를 얻기 위한 Perception 내부 중간 계산일 뿐 3D map이 아닙니다. 실제 팀 Perception topic/message wiring은 VLA-03B로 남아 있습니다. obstacle은 VLA semantic entity가 아니며 Nav2 costmap/local planner 책임입니다.

## Qwen/XPU 통합 상태

현재 코드에는 `TransformersQwenAdapter`와 `mock`/`ollama`/`transformers` backend 선택, 그리고 VLA Node에만 XPU Python을 적용하는 `vla_python_executable` launch wiring이 구현되어 있습니다. 현재 선택 모델은 `Qwen/Qwen2.5-1.5B-Instruct`이며 기본 device는 Intel XPU `xpu:0`입니다.

이전 integration verification에서 Intel Arc B580 XPU smoke와 다음 ROS2 runtime 경로가 확인되었습니다.

```text
Mission "대기해."
→ VLAOrchestratorNode
→ TransformersQwenAdapter / Qwen2.5 / XPU
→ ActionDecision(WAIT, target=null)
→ strict parser → TargetResolver → ActionValidator
→ ActionDispatcher → MockWaitAdapter ACCEPTED
```

이는 handoff 기준 integration history이며 현재 문서 최신화 세션에서 실제 Qwen inference를 재실행한 결과는 아닙니다. 현재 세션에서는 Python unit test, ROS2 package build, 코드 및 launch 구조를 별도로 재검증했습니다.

## VLA-04 Person Report Topic Adapter

`report_mode=MOCK|TOPIC_BRIDGE`로 ReportPort composition을 선택합니다. Topic Bridge mode는 `/vla/person_report`에 `action_id`, `mission_id`, stable `person_id`, authoritative WorldModel `map_position`, `confidence`, `timestamp`, `frame_id=map` JSON을 발행하고 `/vla/person_report_result`의 correlated terminal result를 `ActionResult(source=REPORT)`로 정규화합니다.

Submission `ACCEPTED`만으로 person 상태를 변경하지 않으며 `SUCCEEDED` terminal result가 WorldModel에 적용될 때만 `reported=true`, `state=REPORTED`가 됩니다. FAILED/ABORTED/CANCELED/TIMED_OUT은 미보고 상태를 유지합니다. Validator는 unknown/already-reported person을 차단하고 Dispatcher와 Adapter가 동일 `action_id` 중복 발행을 방어합니다. 실제 UI나 외부 reporting backend 없이 ROS2 topic round-trip smoke를 확인했습니다.
