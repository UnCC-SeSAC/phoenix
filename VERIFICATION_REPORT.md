# Verification Report

## 현재 checkout 재검증 결과

기준 branch와 commit:

```text
feature/vla-brain
2da88d58f710d110860a8f397278f161cd641946
```

현재 세션에서 다음 항목을 재검증했습니다.

```text
python3 -m pytest -q
75 passed

colcon build --packages-select fire_vla_core fire_vla_bringup
fire_vla_core PASS
fire_vla_bringup PASS
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
