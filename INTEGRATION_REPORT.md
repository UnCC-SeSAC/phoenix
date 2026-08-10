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
