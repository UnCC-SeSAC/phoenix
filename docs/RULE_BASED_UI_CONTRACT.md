# Rule-based Firefighter UI Contract

## Boundary

The Rule-based UI uses one read topic and one write topic:

- read: `/rule_based/status` (`std_msgs/msg/String`, JSON)
- write: `/rule_based/mission` (`std_msgs/msg/String`, JSON)

`rule_based_ui_adapter` owns this translation boundary. The UI must not subscribe
directly to StateManager, Nav2, Frontier, detection, or suppression internals.
The VLA boundary remains unchanged at `/vla/status` and `/vla/mission`.

## Status schema

`schema_version=1` snapshots are published at 2 Hz by default:

```json
{
  "schema_version": 1,
  "mode": "RULE_BASED",
  "timestamp": "UTC ISO-8601",
  "mission": {
    "state": "EXPLORING|PERSON_DETECTED|FIRE_DETECTED|RETURNING_TO_BASE|UNKNOWN",
    "target_type": "idle|frontier|person|fire|base",
    "current_target": {"frame_id": "map", "x": 1.2, "y": -0.3},
    "last_command": {
      "mission_id": "mission_ui_...",
      "command": "START|STOP",
      "status": "ACCEPTED"
    }
  },
  "robot": {
    "battery_raw": 7200,
    "navigation_status": "IDLE|ACCEPTED|RUNNING|CANCELING|SUCCEEDED|CANCELED|ABORTED|UNKNOWN"
  },
  "exploration": {
    "status": "IDLE|RUNNING|START_SCHEDULED|STOP_SCHEDULED|STOPPING|SHUTDOWN_PENDING|UNKNOWN"
  },
  "detections": {
    "targets": [
      {"type": "person_unconfirmed", "x": 0.5, "y": 0.0},
      {"type": "fire_unvisited", "x": 1.2, "y": -0.3}
    ],
    "counts": {"person": 1, "fire": 1}
  },
  "suppression": {
    "status": "IDLE|ACCEPTED|RUNNING|CANCELING|SUCCEEDED|CANCELED|ABORTED|UNKNOWN"
  },
  "blocked_reason": ""
}
```

`current_target` can be `null`. Positions are authoritative 2D `map`
coordinates. Detection target categories are the existing StateManager contract:
`person_unconfirmed`, `person_confirmed`, `fire_unvisited`, `fire_failed`,
and `fire_extinguished`.

## Mission/control input

The envelope intentionally matches the existing UI mission boundary:

```json
{"mission_id":"mission_ui_001","text":"START"}
```

Rule-based mode accepts only:

- `START`: enable MissionExecutor; it resumes Frontier exploration or handles the
  current semantic target according to the existing FSM.
- `STOP`: disable MissionExecutor, cancel its active Nav2 and SuppressFire goals,
  and request a serialized Frontier stop.

Unknown commands, missing/extra fields, empty mission IDs, and malformed JSON are
rejected without publishing a control command. Duplicate `mission_id` values are
idempotently ignored.

This is not a free-text planner. It does not translate natural language into goals,
alter StateManager priority, bypass Nav2, or issue direct motor/Pump/Servo commands.

## Internal adapter mapping

| UI field | Existing production source |
|---|---|
| `mission.state` | `/mission/state` |
| `mission.target_type` | `/mission/target_type` |
| `mission.current_target` | `/mission/current_target` |
| `detections.targets` | `/mission/found_targets` |
| `robot.battery_raw` | `/ros_robot_controller/battery` |
| `robot.navigation_status` | `/navigate_to_pose/_action/status` |
| `exploration.status` | `/rule_based/exploration_state` |
| `suppression.status` | `/suppress_fire/_action/status` |
| START/STOP output | `/mission/enabled` |

The Adapter is integration-owned. No Rule-based fields are added to the VLA
WorldModel or VLA status schema.
