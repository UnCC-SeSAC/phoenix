# Verification Report

## 현재 checkout 재검증 결과

기준 branch와 commit:

```text
feature/vla-brain
base: e2140a526cd3b007b37e3070ed3aa90abd27bf09
```

현재 세션에서 다음 항목을 재검증했습니다.

```text
python3 -m pytest -q
189 passed

colcon build --packages-select fire_vla_core fire_vla_bringup uncc_example
fire_vla_core PASS
fire_vla_bringup PASS
uncc_example PASS
```

추가로 VLA Core, Qwen Adapter, decision dedup, ROS backend 및 launch wiring,
Topic Bridge 코드 구조를 현재 checkout에서 확인했습니다.

## 현재 세션에서 재실행하지 않은 항목

- 실제 Qwen2.5 Intel XPU inference
- 실제 ROS2 + Qwen runtime
- Jazzy ↔ Humble DDS 통신
- 실제 Nav2 goal 실행과 TF `map → base_footprint`
- Pump, MCU 및 실제 로봇 하드웨어

Qwen2.5 XPU smoke와 ROS2 `Mission → WAIT → Resolver → Validator → MockWaitAdapter`
통합 PASS는 이전 integration verification 이력입니다. 해당 이력을 현재 checkout에서 다시
실행한 결과로 해석하지 않습니다.

## VLA-01 Topic Bridge 검증

```text
Deterministic ActionDecision
→ TargetResolver
→ ActionValidator
→ TopicBridgeNavigationAdapter
→ /vla/navigation_goal JSON publish
```

직접 unit/integration test와 ROS2 Jazzy topic smoke를 통과했습니다. person/fire의 WorldModel pose resolve, invalid target 및 stale pose validation reject의 미발행, 동일 action ID 중복 방어를 확인했습니다. 실제 Nav2/Robot과 Qwen XPU는 이 검증에서 사용하지 않았습니다.

## VLA-02 Navigation Result Lifecycle 검증

```text
/vla/navigation_result
→ TopicBridgeNavigationAdapter
→ ActionResult
→ VLAOrchestrator.process_results()
→ WorldModel current_action / last_action
```

SUCCEEDED, ABORTED, FAILED, CANCELED 반영과 current action 해제, last action 갱신, physical result 이후 decision signature invalidation을 unit/integration test로 확인했습니다. stale/unrelated action ID는 WorldModel 변경 없이 차단하며 동일 terminal result는 한 번만 적용합니다. Humble bridge의 Nav2 GoalStatus mapping과 action ID가 일치하는 cancel만 전달하는 계약도 직접 검증했습니다.

ROS2 Jazzy topic smoke에서 deterministic `/vla/navigation_result` SUCCEEDED를 publish한 뒤 `/vla/world_model`의 `current_action=null`, `last_action.status=SUCCEEDED`를 관측했습니다. 실제 Nav2 server, `/navigate_to_pose` goal, SLAM, Robot은 실행하지 않았습니다.

Navigation ownership은 DETERMINISTIC mode와 VLA mode에서 goal sender를 하나만 켜는 launch composition 계약입니다. 동시에 여러 sender를 활성화하는 구성과 별도 arbitration manager는 지원하지 않습니다.

## VLA-03A Canonical Perception / Stable ID 검증

```text
/vla/perception_observation
→ canonical map-frame validation
→ upstream ID preservation or fallback association
→ SemanticObservation
→ WorldModel snapshot
```

`frame_id=map`, timezone-aware timestamp, person/fire class, finite `[0,1]` confidence와 finite 2D map position을 검증합니다. ID 없는 detection은 같은 class, 0.5 m 이내, last_seen 2.0초 이내의 nearest entity와 연결하며 batch one-to-one과 entity ID tie-break를 적용합니다. upstream non-empty ID는 그대로 보존합니다.

25개 추가 test로 최초/근접/원거리/class 분리/upstream ID/one-to-one/TTL/tie-break/non-map/NaN·Inf/timestamp/confidence/snapshot 시나리오를 확인했습니다. ROS2 Jazzy smoke에서 ID 없는 person `(2.0,1.0)`과 `(2.05,1.03)` 연속 입력 후 `people=1`, `person_0001` 유지, 최신 위치 갱신을 관측했습니다.

실제 YOLO, camera/depth hardware, object TF, SLAM, Nav2, Robot, Qwen은 VLA-03A 검증에 사용하지 않았습니다.

## VLA-03B Live Perception Bridge 검증

```text
/vision/detections (std_msgs/String)
→ vla_perception_bridge
→ /vla/perception_observation (canonical JSON)
→ CanonicalPerceptionNormalizer
→ WorldModel /vla/status
```

upstream `vision_detector.py`가 confidence와 원본 sec/nanosec timestamp를 map output까지 보존하도록 additive 보강했습니다. person/fire mapping, source timestamp의 UTC ISO 변환, finite/map-frame 검증, smoke/malformed/missing/invalid 입력 drop, VLA-03A stable ID E2E를 테스트했습니다. ROS2 Jazzy manual payload smoke에서 연속 person이 `person_0001`로 유지·갱신되고 `fire_0001`이 생성되며 smoke가 canonical topic과 WorldModel을 변경하지 않음을 확인했습니다. 실제 YOLO node와 camera/depth/TF hardware는 실행하지 않았습니다.

## VLA-04 Person Report Lifecycle 검증

```text
REPORT_PERSON
→ Resolver → Validator → Dispatcher
→ /vla/person_report
→ /vla/person_report_result
→ ActionResult(REPORT)
→ VLAOrchestrator → WorldModel
```

15개 test로 authoritative person ID/map position/confidence payload, ACCEPTED와 terminal result 분리, SUCCEEDED에서만 `reported=true`, FAILED/ABORTED/CANCELED/TIMED_OUT 미보고 유지, unknown/already-reported 차단, Dispatcher/Adapter duplicate submission, stale/duplicate terminal result 방어, decision signature 변화와 VLA-03A `person_0001` 연계를 확인했습니다.

ROS2 Jazzy topic smoke에서 `/vla/person_report` payload를 실제 구독하고 `/vla/person_report_result` SUCCEEDED를 회신하여 WorldModel `reported=true`를 확인했습니다. 실제 UI, 외부 보고 backend, Robot, Nav2, Qwen, VLA-03B는 사용하지 않았습니다.

## VLA-05 Spray Lifecycle 검증

```text
EXTINGUISH
→ Resolver → Validator → Dispatcher
→ /vla/spray_command
→ /vla/spray_result
→ ActionResult(SPRAY)
→ VLAOrchestrator → WorldModel
```

20개 test로 authoritative fire ID command, submission/terminal 분리, SUCCEEDED의 `PENDING_VERIFICATION` 및 `spray_count+1`, FAILED/ABORTED/CANCELED/TIMED_OUT의 ACTIVE 유지, out-of-range/inactive/max-attempt/unknown 차단, command/result/cancel correlation, duplicate/stale/mismatched result 방어, physical result decision signature invalidation과 VLA-03A `fire_0001` 연계를 확인했습니다.

ROS2 Jazzy topic smoke에서 `/vla/spray_command`를 실제 구독하고 `/vla/spray_result` SUCCEEDED를 회신하여 `current_action=null`, `fire.state=PENDING_VERIFICATION`, `spray_count=1`을 확인했습니다. 실제 Pump/MCU, 물 분사, Robot, Qwen, VLA-03B는 사용하지 않았습니다.

## VLA-06 Firefighter UI 검증

```text
Browser POST /api/mission
→ /vla/mission
→ VLAOrchestrator / WorldModel / DecisionCycle
→ /vla/status
→ Firefighter UI GET /api/status
```

21개 test로 status serialization, robot/person/fire, decision/LLM reason/validation reason/blocked reason 분리, immutable status store, Mission ID 및 빈·비문자열·malformed request 차단, loopback host/port validation, HTML/API 응답, navigation/report/spray lifecycle 표시와 기존 launch의 UI 비강제 구성을 확인했습니다.

ROS2 Jazzy mock launch와 실제 HTTP smoke에서 Mission 1회 입력, canonical `person_0001`/`fire_0001` 표시, report `reported=true`, spray `PENDING_VERIFICATION`/`spray_count=1`, blocked reason과 2D semantic map HTML을 확인했습니다. GUI browser visual automation은 실행하지 않았으며 HTTP/HTML/API/ROS boundary까지 검증했습니다. 실제 Robot, Pump/MCU, Qwen, live Perception은 사용하지 않았습니다.

## VLA-07 Hardware stationary verification — 2026-08-11

실제 환경은 Ubuntu/Jazzy PC, Debian 13 Pi host, `IntelPi` Ubuntu 22.04/Humble Docker다. Container는 host network, privileged, `/dev` bind이며 nav2_msgs/navigation2/nav2_bringup 1.1.15와 slam_toolbox 2.6.8을 확인했다.

PASS:

- test-only PC domain 205 + Fast DDS의 양방향 String과 `/vla/*` JSON
- odom, scan, map, `map → odom → base_footprint`
- Nav2 lifecycle active와 NavigateToPose server
- actual Pi→PC robot pose와 WorldModel 반영
- board driver subscriber와 explicit four-motor zero
- autonomous goal sender OFF

Persistent PC `124 + CycloneDDS`와 Pi `205 + Fast DDS`는 변경하지 않았다. Pi team workspace도 수정하지 않았다.

First short-nav preflight는 Mock person map (0.5,0.0), 인명을 우선 확인해 Mission을 사용했다. Expected는 `NAVIGATE_TO person_0001`, actual은 `RETURN_HOME`과 stale robot pose validation reject였다.

결과는 **SAFE ABORT**이며 NavigateToPose goal 0, non-zero cmd_vel 0, Robot movement 0 m다.

## VLA-07 Software-only short-nav preflight — 2026-08-11

SAFE ABORT 증거를 분리 분석했다.

- stale pose: actual pose는 PC WorldModel에 한 번 들어왔지만 hotspot/DDS pose
  stream 중단 뒤 `pose_updated_at`이 0.5초 freshness 한계를 넘었다.
  Validator reject는 정상이며 threshold는 변경하지 않았다.
- `RETURN_HOME`: 현장 perception fixture가 고정 timestamp를 사용해
  observation이 1.0초 freshness 한계를 넘고 `person_0001`이 생성되지 않았다.
  또한 일반 MockVLABrain은 0.8 m 이내 person을 `REPORT_PERSON`으로 고르는
  production mock semantics를 갖기 때문에 first-motion transport gate에 사용하지 않는다.

`vla_short_nav_preflight`는 gate 전용 최소 decision stub과
`MockNavigationAdapter`를 사용한다. fresh robot pose stream, 현재 timestamp의
person map `(0.5,0.0)`, Mission `인명을 우선 확인해.`를 입력한 결과:

- stable ID: `person_0001`
- decision: `NAVIGATE_TO person_0001`
- Resolver: map `(0.5,0.0, yaw=0.0)`
- Validator: approved
- Dispatcher: `ACCEPTED` by Mock Navigation
- actual Nav2 goal: 0
- cmd_vel: 0

연속 pose update의 freshness 유지, stale pose reject, stable-ID/decision/
Resolver/Validator/Mock dispatch E2E를 regression test로 고정했다.
