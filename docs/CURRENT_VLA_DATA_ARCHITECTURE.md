# 현재 VLA 데이터 아키텍처

Phoenix의 authoritative navigation·semantic 공간은 2D `map` frame이다.
Depth로 계산한 camera-frame `(X,Y,Z)`는 객체의 `map(x,y)`를 얻기 위한
Perception 내부 중간값이며 3D WorldModel을 뜻하지 않는다.

## Runtime 역할 분리

```text
Raspberry Pi
Camera / HEF YOLO / Depth / CameraInfo / TF / SLAM / Nav2
        ↓
VLA Orchestrator / Semantic WorldModel
        ↓ compact WorldModel
HTTP/JSON
        ↓
Ubuntu Local PC
Qwen3 inference
        ↓ action / target / reason
HTTP/JSON
        ↓
Raspberry Pi
TargetResolver → ActionValidator → ActionDispatcher
        ↓
ROS 2 Navigation / Report / Spray boundary
```

Pi는 Robot state와 control plane을 소유한다. PC는
`Qwen/Qwen3-1.7B` model loading과 inference만 수행한다. raw image, Depth,
LaserScan, OccupancyGrid, TF tree, ROS graph, Nav2와 `cmd_vel`은 PC로 보내지 않는다.
HTTP failure나 잘못된 model 응답은 blocked cycle로 처리하며 자동 motion fallback은
없다.

## Perception

```text
Camera
→ Hailo split HEF YOLO
→ bbox / score
→ Depth
→ CameraInfo backprojection
→ source-time TF
→ map(x,y)
→ SemanticObservation
→ WorldModel
```

현재 production HEF는 final NMS 단일 output이 아니라 6개 raw neural head를
출력한다. Hailo backend는 companion postprocess model로 이를
`[x1,y1,x2,y2,score,class_id]` detection으로 변환한다. ONNX-only runtime이나
test Stub을 production fallback으로 사용하지 않는다.

YOLO node는 `/yolo_result` `vision_msgs/Detection2DArray`를 발행한다.
`image_pipeline`은 Depth를 결합해 다음 경계를 만든다.

| Boundary | Type | 의미 |
|---|---|---|
| `/yolo_result` | `vision_msgs/Detection2DArray` | bbox와 score |
| `/fire/detections` | `std_msgs/String` JSON | 원본 pixel, depth, score, source stamp |
| `/fire/detections/status` | `std_msgs/String` JSON | 독립 health heartbeat |
| `/vla/perception_observation` | `std_msgs/String` JSON | canonical person/fire `map(x,y)` |

VLA ROS Adapter가 rgb0 CameraInfo로 역투영하고 원본 source timestamp의 TF를 조회한다.
`score`는 scaling 없이 `confidence`로 이름만 바꾼다. `depth_status=unknown`,
invalid CameraInfo, non-finite depth·좌표, TF failure는 fail-closed한다.
`fallback_bottom`, `fallback_below`, `fallback_ring`은 유효한 depth일 때
provenance를 보존한다. 일반 obstacle avoidance는 Nav2 costmap 책임이다.

## Semantic WorldModel

WorldModel은 Mission, Robot pose/home/navigation status, people, fires,
unexplored zones, current/last Action과 최근 결과를 저장한다. person/fire의 위치는
Perception과 TF가 만든 authoritative `map(x,y)`다.

upstream ID가 있으면 그대로 보존한다. 없으면 같은 class, 0.5 m radius,
2.0초 TTL, batch one-to-one association으로 process-local ID를 만든다. 이는
full tracker가 아니므로 긴 occlusion이나 process restart 뒤 영구 ID를 보장하지 않는다.

## Decision

```text
Mission + compact WorldModel
→ HTTP POST /infer
→ Qwen
→ strict ActionDecision
→ TargetResolver
→ ActionValidator
→ ActionDispatcher
```

HTTP request는 `mission`, `world_model`, `allowed_actions`만 포함한다.
response는 정확히 `action`, `target`, `reason` 세 필드다. Qwen은 좌표를
생성하지 않고 WorldModel에 존재하는 target ID만 선택한다.

지원 Action은 다음과 같다.

| Action | target | 실행 경계 |
|---|---|---|
| `NAVIGATE_TO` | person/fire ID | NavigationPort |
| `REPORT_PERSON` | person ID | ReportPort |
| `EXTINGUISH` | fire ID | SprayPort |
| `SEARCH` | unexplored zone ID | NavigationPort |
| `WAIT` | `null` | WaitPort |
| `RETURN_HOME` | `null` | NavigationPort |

Resolver가 target ID를 WorldModel의 pose로 변환한다. Validator는 target 존재,
fresh Robot pose, finite/map bounds, 중복 physical Action, report 상태, fire 상태와
spray range를 검사한다. LLM output은 물리 상태의 source of truth가 아니다.

## Control과 result lifecycle

```text
Navigation action
→ /vla/navigation_goal
→ VLA Navigation Bridge
→ Nav2 NavigateToPose
→ cmd_vel
→ Motor
```

| 용도 | ROS 2 boundary |
|---|---|
| Mission / status | `/vla/mission`, `/vla/status` |
| Robot pose | `/vla/robot_pose_json` |
| Navigation | `/vla/navigation_goal`, `/vla/navigation_result`, `/vla/navigation_cancel` |
| Person report | `/vla/person_report`, `/vla/person_report_result` |
| Suppression | `/vla/spray_command`, `/vla/spray_result`, `/vla/spray_cancel` |

terminal result는 `SUCCEEDED`, `FAILED`, `ABORTED`, `CANCELED`,
`TIMED_OUT`으로 correlation한다. report는 correlated `SUCCEEDED`에서만
reported 상태가 된다. spray `SUCCEEDED`는 `PENDING_VERIFICATION` 전이이며
그 자체로 `EXTINGUISHED`를 의미하지 않는다.

Navigation goal owner는 하나만 활성화한다. DETERMINISTIC mode는
Frontier/StateManager/MissionExecutor가, VLA mode는 VLA Brain과 Navigation Bridge가
goal을 소유한다.

## Firefighter UI

Browser는 local HTTP boundary만 사용한다.

- `GET /api/status?mode=VLA|RULE_BASED`
- `POST /api/mission` with `{"text":"...","mode":"VLA|RULE_BASED"}`

VLA mode는 `/vla/status`, `/vla/mission`을 사용한다. UI는 Mission, Robot,
people/fire 위치, WorldModel, Qwen decision/reason, Current Action, result,
report/suppression, blocked/error 상태를 표시한다. Browser가 Nav2, Motor, Pump/Servo를
직접 제어하지 않는다.

## 현재 구현 범위

구현된 범위는 canonical Perception Adapter, Semantic WorldModel, remote Qwen
HTTP contract, Resolver/Validator/Dispatcher, Navigation·Report·Spray ROS boundary,
result lifecycle과 Firefighter UI다.

실제 Hardware E2E는 단계적으로 검증 중이다. software-only PASS를 Camera/Robot/Pump
Hardware PASS로 간주하지 않는다.
