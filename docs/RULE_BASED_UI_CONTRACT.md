# Rule-based Firefighter UI 계약

Firefighter UI는 `VLA Brain`과 `Rule-based` 두 mode를 같은 Browser shell에서
제공한다. Browser는 backend 내부 topic을 직접 구독하지 않고 local HTTP API만
사용한다.

## 공개 boundary

| 방향 | Boundary | 역할 |
|---|---|---|
| Browser read | `GET /api/status?mode=RULE_BASED` | Rule-based snapshot 조회 |
| Browser write | `POST /api/mission` | `{"text":"START|STOP","mode":"RULE_BASED"}` |
| ROS read | `/rule_based/status` | versioned JSON snapshot |
| ROS write | `/rule_based/mission` | mission JSON envelope |

`rule_based_ui_adapter`가 StateManager, Nav2, Frontier, detection, battery,
suppression 상태를 `schema_version=1` snapshot으로 변환한다. 모든 위치는
authoritative 2D `map` 좌표다.

## Status 구성

```json
{
  "schema_version": 1,
  "mode": "RULE_BASED",
  "timestamp": "UTC ISO-8601",
  "mission": {
    "state": "EXPLORING|PERSON_DETECTED|FIRE_DETECTED|RETURNING_TO_BASE|UNKNOWN",
    "target_type": "idle|frontier|person|fire|base",
    "current_target": {"frame_id": "map", "x": 1.2, "y": -0.3},
    "last_command": {"mission_id": "mission_ui_...", "command": "START|STOP", "status": "ACCEPTED"}
  },
  "robot": {
    "battery_raw": 7200,
    "navigation_status": "IDLE|ACCEPTED|RUNNING|CANCELING|SUCCEEDED|CANCELED|ABORTED|UNKNOWN"
  },
  "exploration": {"status": "IDLE|RUNNING|START_SCHEDULED|STOP_SCHEDULED|STOPPING|UNKNOWN"},
  "detections": {
    "targets": [{"type": "person_unconfirmed", "x": 0.5, "y": 0.0}],
    "counts": {"person": 1, "fire": 0}
  },
  "suppression": {"status": "IDLE|ACCEPTED|RUNNING|CANCELING|SUCCEEDED|CANCELED|ABORTED|UNKNOWN"},
  "blocked_reason": ""
}
```

`current_target`은 `null`일 수 있다. detection category는 기존 StateManager
contract의 `person_unconfirmed`, `person_confirmed`, `fire_unvisited`,
`fire_failed`, `fire_extinguished`를 사용한다.

## Mission control

Rule-based mode는 `START`와 `STOP`만 허용한다.

- `START`: MissionExecutor를 활성화하고 기존 FSM에 따라 탐색 또는 semantic target을 처리한다.
- `STOP`: MissionExecutor의 Nav2/SuppressFire goal을 cancel하고 Frontier stop을 직렬화한다.

잘못된 JSON, unknown command, 중복 `mission_id`는 control command를 발행하지 않는다.
이 boundary는 자연어 planner가 아니며 StateManager 우선순위, Nav2, Motor,
Pump/Servo를 우회하지 않는다.

## 내부 source mapping

| UI field | Production source |
|---|---|
| mission state/target | `/mission/state`, `/mission/target_type`, `/mission/current_target` |
| detections | `/mission/found_targets` |
| battery | `/ros_robot_controller/battery` |
| navigation | `/navigate_to_pose/_action/status` |
| exploration | `/rule_based/exploration_state` |
| suppression | `/suppress_fire/_action/status` |
| START/STOP output | `/mission/enabled` |

VLA mode의 `/vla/status`, `/vla/mission` contract와 WorldModel은 변경하지 않는다.
Frontend에는 FSM, Nav2, Frontier 또는 actuator logic을 넣지 않는다.

## Local software mock 실행

이 절차는 Ubuntu ROS 2 Jazzy PC에서만 실행하며 Pi/Robot은 필요하지 않다.
`integration/vla-robot-e2e`의 commit `626626b` 이상을 사용한다.

UI server:

```bash
source /opt/ros/jazzy/setup.bash
PYTHONPATH="$PWD/src/fire_vla_core:$PYTHONPATH" \
python3 -c 'from fire_vla_core.ros.firefighter_ui_node import main; main()'
```

다른 terminal에서 최소 VLA snapshot을 발행한다:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /vla/status std_msgs/msg/String \
'data: "{\"timestamp\":\"2026-08-21T15:30:00+09:00\",\"world_model\":{\"mission\":{\"id\":\"mission_mock_vla\",\"text\":\"인명을 우선 확인해\"},\"robot\":{\"pose\":{\"x\":0.2,\"y\":-0.1},\"navigation_status\":\"SUCCEEDED\"},\"people\":[{\"id\":\"person_0001\",\"position\":{\"x\":1.4,\"y\":0.5},\"confidence\":0.93}],\"fires\":[]},\"decision\":{\"action\":\"REPORT_PERSON\",\"target\":\"person_0001\",\"reason\":\"mock decision\"},\"validation\":{\"approved\":true},\"submission\":{\"status\":\"ACCEPTED\"}}"'
```

최소 Rule-based snapshot을 발행한다:

```bash
ros2 topic pub --once /rule_based/status std_msgs/msg/String \
'data: "{\"schema_version\":1,\"mode\":\"RULE_BASED\",\"timestamp\":\"2026-08-21T15:30:01+09:00\",\"mission\":{\"state\":\"FIRE_DETECTED\",\"target_type\":\"fire\",\"current_target\":{\"frame_id\":\"map\",\"x\":2.3,\"y\":-0.7},\"last_command\":{\"mission_id\":\"mission_mock_rule\",\"command\":\"START\",\"status\":\"ACCEPTED\"}},\"robot\":{\"battery_raw\":7210,\"navigation_status\":\"RUNNING\"},\"exploration\":{\"status\":\"RUNNING\"},\"detections\":{\"targets\":[{\"type\":\"fire_unvisited\",\"x\":2.3,\"y\":-0.7}],\"counts\":{\"person\":0,\"fire\":1}},\"suppression\":{\"status\":\"IDLE\"},\"blocked_reason\":\"\"}"'
```

Browser는 `http://127.0.0.1:8080/`을 연다. Snapshot은 UI process의
`StatusStore`에 유지되며 UI server를 재시작하면 다시 발행해야 한다.
