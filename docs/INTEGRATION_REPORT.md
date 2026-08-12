# 통합 변경 보고서

## VLA-07C image_pipeline integration — 2026-08-12

- Upstream: `origin/albitro/image_pipeline` @
  `ef592b5f756d87bff5dac0db1aeb0fbda05819ad`
- Branch 전체에는 image_pipeline 외 과거 workflow 변경이 포함되어 전체 merge하지
  않고 `src/image_pipeline`만 file-level import했다.
- VLA bridge 입력을 구형 `/vision/detections` flat map JSON에서 최신
  `/fire/detections` multi-detection pixel/depth envelope로 변경했다.
- ROS Adapter가 rgb0 CameraInfo로 역투영하고 원본 sec/nanosec 기준 TF를 조회해
  canonical 2D map 위치를 만든다.
- `score → confidence`는 1:1 rename이며 scaling하지 않는다.
- `unknown` depth는 fail-closed, `fallback_bottom/below/ring`은 status를 보존한다.
- `/fire/detections/status`의 `ok/stalled/waiting_camera_info/no_input`을 detection
  silence와 독립된 health signal로 mapping한다.
- VLA Core, SemanticObservation, WorldModel, stable ID fallback은 변경하지 않았다.

이 아래 VLA-03B `/vision/detections` 설명은 과거 integration 이력이다. 현재
runtime contract는 `CURRENT_VLA_DATA_ARCHITECTURE.md`의 VLA-07C 절을 따른다.

## VLA-07D latest YOLO pipeline sync — 2026-08-12

- Previous upstream: `ef592b5f756d87bff5dac0db1aeb0fbda05819ad`
- Latest upstream: `31845b563fdec42d5d061853746365543e0dc8d2`
- Branch 전체 merge 없이 변경된 `src/image_pipeline` 14개 파일만 갱신했다.
- YOLO node/backend, letterbox inverse, NMS/layout parsing, explicit test stub,
  offline/wiring tools와 `full_chain_check.launch.py`가 추가됐다.
- `/fire/detections`와 `/fire/detections/status` 계약은 변경되지 않아 VLA Adapter
  production code 수정은 하지 않았다.
- Production model 부재는 startup failure이며 stub으로 자동 fallback하지 않는다.

## VLA-07E flattened ROS package layout sync — 2026-08-12

- Previous imported upstream: `31845b563fdec42d5d061853746365543e0dc8d2`
- Latest upstream: `a8caf2c9b45f35e66b0e3660ecad0ce8e422d719`
- `src/image_pipeline/ros/image_pipeline`의 ROS package를
  `src/image_pipeline` root로 semantic rename하고 upstream에서 제거한 training
  subtree도 함께 제거했다.
- Python import package명 `image_pipeline`, launch/config/resource 내용과 perception
  topic/payload contract는 변경되지 않았다. 따라서 VLA production code는 수정하지
  않았다.
- colcon discovery에는 `image_pipeline` package가 정확히 한 번만 나타난다.

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

이는 tracker가 아닌 MVP fallback입니다. 빠른 이동, 근접 교차, 긴 occlusion, process restart에서는 ID switch가 가능하며 전역 영구 ID를 보장하지 않습니다. Depth 기반 camera-frame 3D point는 최종 2D map 위치를 얻기 위한 Perception 내부 중간 계산일 뿐 3D map이 아닙니다. VLA-03B는 `origin/state_manage`의 `/vision/detections` String 계약을 기준으로 person/fire map `(x,y)`, confidence, 원본 timestamp를 canonical boundary로 전달하는 thin bridge를 구현했습니다. smoke는 MVP에서 무시합니다. 실제 YOLO/camera/depth/TF hardware smoke는 수행하지 않았고 obstacle은 Nav2 costmap/local planner 책임입니다.

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

## VLA-05 Spray Topic Adapter

팀 branch 조사에서 확인된 `state_manage`의 `fire_extinguisher`는 `std_srvs/Trigger` 기반 즉시 성공 더미이며 action ID, cancel/timeout, correlated result 계약이 없습니다. 실제 Pump/MCU production contract는 확인되지 않아 이를 직접 결합하지 않고 VLA-side canonical boundary를 구현했습니다.

`spray_mode=MOCK|TOPIC_BRIDGE`로 composition을 선택합니다. Topic Bridge mode는 `/vla/spray_command`에 `action_id`, `mission_id`, authoritative `fire_id`, `command=SPRAY`, timestamp JSON을 발행합니다. `/vla/spray_result`는 correlated terminal result를 `ActionResult(source=SPRAY)`로 정규화하고 `/vla/spray_cancel`은 현재 active action ID만 전달합니다.

Validator와 Adapter는 fire 존재, ACTIVE, `robot_within_spray_range=true`, 최대 시도 미만 조건을 방어합니다. ACCEPTED만으로 fire 상태를 바꾸지 않습니다. SUCCEEDED는 `spray_count`를 증가시키고 `PENDING_VERIFICATION`으로 전이할 뿐이며 후속 semantic observation 검증 전에는 EXTINGUISHED로 처리하지 않습니다. VLA Spray ROS boundary는 구현/검증 완료이고 실제 Pump/MCU bridge는 pending/upstream입니다.

## VLA-06 Firefighter Mission / Semantic Status UI

기존 `/vla/world_model`은 semantic snapshot을 제공하지만 decision/reason/blocked cycle을 모두 표현하지 않아, 이를 유지하면서 읽기 전용 `/vla/status` `std_msgs/String` JSON boundary를 추가했습니다. Payload는 `WorldModel.create_snapshot()`을 그대로 `world_model`에 사용하고 최신 meaningful DecisionCycle의 `decision`, `validation`, `submission`, `blocked_reason`과 timestamp만 덧붙입니다. LLM reason, validation reason, blocked reason은 별도 필드로 유지합니다.

`firefighter_ui` ROS node는 `/vla/status`만 구독하고 `/vla/mission`만 발행합니다. Python stdlib `ThreadingHTTPServer`와 package static HTML을 사용하며 기본 `127.0.0.1:8080`, `GET /`, `GET /api/status`, `POST /api/mission`만 제공합니다. UI는 Mission, robot/person/fire, decision/safety/execution 상태와 auto-fit 2D semantic SVG overlay를 표시하고 직접 Action/Nav2/Pump command를 제공하지 않습니다.

Mock launch smoke에서 HTTP Mission POST, canonical person/fire/robot 입력, report SUCCEEDED, spray PENDING_VERIFICATION, blocked reason을 status API로 확인했습니다. VLA-03B bridge의 deterministic live-topic smoke는 완료했으며 UI는 동일 canonical status를 소비하므로 코드 변경이 필요하지 않았습니다. 실제 YOLO/camera/depth/TF hardware feed는 미검증입니다. 실제 Browser GUI automation, Robot, Pump/MCU, Qwen은 사용하지 않았습니다.

## VLA-07 Robot integration checkpoint

- Branch: `integration/vla-robot-e2e`
- Assembly commit: `a1a1b20bdcf34edd36f65bfd51475f427b5f650f`
- Robot runtime: Pi `IntelPi` Humble Docker
- PC runtime: Jazzy VLA packages

실제 stationary에서 DDS, odom, scan, map, TF chain, Nav2 lifecycle/action server, VLA Bridge actual pose, PC WorldModel, motor board와 explicit zero halt가 PASS했다.

Autonomous Frontier, MissionExecutor, VLA goal sender는 stationary gate에서 모두 OFF였다. 속도 경로는 `/cmd_vel_nav → velocity_smoother → /cmd_vel → /ros_robot_controller/set_motor`로 확인했다.

Board driver는 빠진 것이 아니다. `hardware.launch.py → controller.launch.py → odom_publisher.launch.py → ros_robot_controller.launch.py`로 포함된다. Root Docker shell에서는 user-local pyserial을 찾지 못하지만 기존 `exec_shell.sh`의 `ubuntu` runtime에서는 정상 연결된다.

첫 actual short navigation은 **SAFE ABORT**다. 예상 NAVIGATE_TO 대신 RETURN_HOME이 생성되고 robot pose freshness validation이 실패했다. 실제 goal 0, non-zero cmd_vel 0, 이동 0 m다.

Production YOLO와 camera stream은 pending이다. Parent VLA-07은 actual short navigation PASS 전까지 완료로 보지 않는다.

### 2026-08-12 Hardware E2E checkpoint

Software regression과 production `TopicBridgeNavigationAdapter` publish path는 PASS다.
실제 production Orchestrator/XPU/Qwen 구성에서도 actual Robot pose와 fresh Mock
`person_0001`이 WorldModel에 반영됐다. 그러나 이후 Robot/Pi pose stream이 소실되어
Mission 입력 전에 중단했으므로 ActionDecision 이후 경로와 Nav2/motor는 실행되지
않았다.

현재 blocker는 production navigation publisher가 아니라 **production decision 이전
Robot runtime/DDS input continuity 소실**이다. actual NavigateToPose goal, non-zero
`cmd_vel`, Robot 이동은 역사상 모두 0이며 live Camera/YOLO perception E2E도 pending이다.
