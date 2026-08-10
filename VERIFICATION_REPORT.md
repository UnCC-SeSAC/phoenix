# Verification Report

## 현재 checkout 재검증 결과

기준 branch와 commit:

```text
feature/vla-brain
19afe915d6a7eaf5cc437ce8d71edf792c0c5068
```

현재 세션에서 다음 항목을 재검증했습니다.

```text
python3 -m pytest -q
64 passed

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

직접 unit/integration test와 ROS2 Jazzy topic smoke를 통과했습니다. person/fire의 WorldModel pose resolve, invalid target 및 stale pose validation reject의 미발행, 동일 action ID 중복 방어를 확인했습니다. 실제 Nav2/Robot과 Qwen XPU는 이 검증에서 사용하지 않았으며 Nav2 result E2E는 VLA-02 범위입니다.
