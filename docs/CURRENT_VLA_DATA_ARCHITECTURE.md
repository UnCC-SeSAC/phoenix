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

RUNNING Mission에서는 각 ACTIVE fire와 가장 가까운 person의 `map(x,y)` 거리를
계산한다. Demo 기본값 `person_fire_risk_distance_m=0.30` 이내이면 fire에
`threatens_person=true`와 `threatened_person_id`를 기록해 Qwen context에 제공한다.
이 값은 demo-scale 관계 parameter이며 실제 배포 안전거리 기준이 아니다.
`blocks_route_to`의 수동/observation route 의미와는 독립적이다.

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
spray range를 검사한다. 직접 선택된 `EXTINGUISH`에는 fire `last_seen`의 나이로
거절하는 추가 Validator gate를 두지 않는다. 관측 입력 수용은 최대 5초를 사용하며,
Nav2 후 자동 continuation은 선택된 Mission·target·제출 시 정렬 좌표와 현재 Robot
pose·분사거리를 확인한다. fire last_seen의 나이만으로 이를 취소하지 않는다.
소화 후 검증 조건은 그대로 유지한다.
따라서 저장된 ACTIVE target은 마지막 관측이 오래돼도 분사할 수 있으며,
그동안 표적이 이동했다면 이전 위치에 분사할 수 있다.
LLM output은 물리 상태의 source of truth가 아니다.

동일 Mission의 동일 target에 대한 중복 physical Action은 차단한다. 다만 분사 후
같은 fire가 `ACTIVE`로 재관측되고 분사거리 밖이면, 해당 `spray_count` 회차에서
갱신 좌표로 `NAVIGATE_TO`를 한 번 다시 허용한다. target lock과 동일 회차의
중복 이동·분사는 계속 유지된다.

`FULL_EXPLORATION`에서 사람을 위협하는 `ACTIVE` fire가 남아 있으면 Core가
다른 fire, `SEARCH`, `RETURN_HOME`보다 해당 fire를 우선한다. Qwen이 다른
target을 제안해도 현재 action을 중간 취소하지 않고 다음 action 선택 경계에서
교정한다. 우선 fire가 `EXTINGUISHED` 또는 `SUPPRESSION_FAILED`가 되면 다른
fire 처리를 계속한다. 사람 위협 fire 우선 후 나머지 fire도 모두 처리하는
Mission의 scope는 `PERSON_FIRE`가 아니라 `FULL_EXPLORATION`이다.

다중 화점 처리에서 첫 화점과 다른 fire ID를 분사하려면 해당 Mission에서 그 ID에
대한 Navigation 성공을 먼저 확인한다. 정렬 완료 식별자는 `(mission_id, fire_id)`이며,
정렬 기준 좌표는 Navigation 제출 시 저장하며 완료 시 최신 좌표로 덮어쓰지 않는다.
동일 target의 0.10m 이내 흔들림은 재이동을 만들지 않고, 그보다 큰 변화는 기존
회차별 재정렬 제한 안에서 새 좌표를 향한 Navigation을 요구한다.
거리 및 Robot pose freshness Validator는 이 정렬 기록과 별개로 계속 적용한다.
선택한 화점의 Navigation이 성공하면 후속 `EXTINGUISH`는 같은 Mission과 target ID를
유지한다. 다른 사람/화점의 추가·갱신이나 선택 화점의 관측 공백만으로 후속 동작을
취소하고 Qwen에 다시 묻지 않는다. 전체 장면 signature 대신 Mission ID와 해당
화점의 정렬 기록을 사용한다. 대상 자체가 ACTIVE가 아니거나 좌표 변화로 기존
정렬이 유효하지 않거나 Robot pose/분사거리가 유효하지 않으면 후속 분사를 하지 않는다.
STOP 및 새 Mission에는 이전 후속 행동을 적용하지 않는다. 분사 후 대상별
verification이 진행 중이면 결과가 정해지기 전 다른 물리 행동을 선택하지 않는다.
이에 따라 새 우선 대상 발견 시 선택 전환은 현재 대상 실행 결과 이후로 늦어질 수 있다.

Mission boundary 직전 관측한 ACTIVE fire의 `map` 좌표는 `FULL_EXPLORATION`의 제한적
재확인 후보로 보존한다. Mission 이전 사람 위협 관계도 후보에 보존하여,
그 후보가 남아 있으면 일반 화점 후보나 일반 ACTIVE 화점보다 먼저 재확인한다.
이는 저장 좌표로 분사를 허용하는 것이 아니라 기존 SEARCH의 순서만 정한다.
Mission 뒤 같은 위치의 fresh fire가 재관측되면 후보를
제거하고, 화면에서 사라진 후보만 한 번 `SEARCH`한다. 일반 Frontier 좌표나
`FIRE_ONLY`에는 이 후보를 사용하지 않는다. 다른 fire 처리로 정렬 소유가 바뀐
fresh/in-range target은 동일 Mission에서 한 번만 재정렬할 수 있다.
화점 후보 `SEARCH` 성공 시 정렬 필요 처리는 후보 제거 성공 여부와 독립적이다.
이동 중 fresh 관측으로 후보가 먼저 제거돼도, 다음 화점의 ID 기준 Navigation
성공을 확인한 뒤 분사한다. 기존 fresh 관측이 있는 경우 후보가 이미 제거됐다는
이유만으로 perception readiness를 다시 초기화하지 않는다.

A/B 고정 시연에서만 wrapper 환경변수 `VLA_AB_DEMO_MODE=true`를 사용하면
launch의 `ab_demo_mode`가 Orchestrator의 WorldModelConfig에 전달된다. Core/launch
기본값은 `false`이며 일반 모드의 다중 화재 Mission은 제한하지 않는다. 시연 모드에서는 같은
종류의 기존 0.15m 병합 기준을 유지하고, 서로 분리된 화재 ID 두 개가 등록된 뒤에는
새 화재 ID를 WorldModel 실행 대상에 추가하지 않는다. 정확히 0.15m는 같은 위치다.
이미 등록된 두 화재의 관측 갱신·소화 verification은 계속 수행하고, terminal 화재를
새 후보로 교체하지 않는다. 새 Mission의 기존 목록 초기화로 등록 슬롯도 다시 열린다.
YOLO 추론·원본 bbox 출력이나 person ID/좌표 정책은 변경하지 않는다.
Core/launch의 `ab_demo_mode` 기본값은 OFF지만, 현재 canonical Hardware wrapper는
운영자 시연 결정에 따라 `VLA_AB_DEMO_MODE` 기본값을 ON으로 사용한다. UI 토글은 없다.
현재 시연 wrapper 기본값은 stand-off `0.25 m`, spray range `0.45 m`, Qwen endpoint
`http://192.168.1.198:8088/infer`, 로그 root `/ros2_ws/phoenix_vla/log/hardware_e2e`다.
이는 승인된 시연 실행값이며 Core/launch의 일반 거리 기본값을 확정 변경한 것은 아니다.
Pi host `/home/uncc/ros2_ws/phoenix_vla`에서 별도 export 없이 wrapper의
`start`, `status`, `stop`을 사용한다. 기존 환경변수를 지정하면 override가 우선한다.
`status` 조회 실패는 UNKNOWN을 유지하되 stderr와 현재 Base/VLA 로그 끝부분을
표시하고, container run directory의 `status_observer.log`에 진단 출력을 남긴다.
조회 subprocess의 기존 제한시간은 유지하고 종료 유예만 적용하며 자동 재시작은 없다.
Humble의 진단 parameter 조회는 `rcl_interfaces/srv/GetParameters` service를 사용한다.
해당 환경에 없는 `rclpy.parameter_client` import 실패를 runtime 장애와 혼동하지 않는다.
시연 모드에서는 Qwen snapshot의 탐색 후보를 저장된 화재 hint로 한정하며,
일반 Frontier `SEARCH`는 target resolution에서 실행 대상으로 만들지 않는다.
`NAVIGATE_TO`는 등록된 화재만 허용하고 사람 보고·화재 hint 재확인·복귀는 유지한다.
Mission 초기화 후 사람 재관측 전에 잘못된 Frontier 판단이 나오면 이동하지 않으며,
다음 관측 변화 후 기존 판단 경로를 다시 사용한다. 새 timeout/retry는 추가하지 않는다.
이 제한은 오검출 판별기가 아니다. 잘못된 후보가 먼저 등록되면 실제 B의 등록을
막을 수 있고, 실제 세 번째 화재나 기존 표적에서 0.15m를 넘게 벗어나 새 ID로 잡힌
관측도 제외될 수 있다. 일반 화재 탐사에서는 켜지 않는다. A/B 처리·인계·복귀·terminal
경로는 실제 실행 2회에서 확인했으나, 모든 물리 화염 제거 및 반복 기동 안정성을
검증 완료한 것은 아니다. 검증 범위는 아래 기록을 따른다.

### A/B 실행 계약 검증 범위 — 2026-10-19

- 실제 실행 2회에서 사람 보고, A/B 접근·각 최대 3회 분사, 대상별 verification,
  소방관 인계 및 `RETURN_HOME` 성공 뒤 `COMPLETED_WITH_ESCALATION`을 확인했다.
  첫 실행은 A 실패·B 소화 판정, 두 번째는 A 소화·B 실패 판정이었다. 이는 대상별
  관측 결과이며 모든 물리 화염의 제거가 직접 검증됐다는 의미는 아니다.
- 완료 후 STOP의 terminal 보존과 A/B 외 Frontier 실행 제한을 확인했다.
- 변경 관련 Core/adapter/launch/bridge 테스트 202개 PASS. PC Jazzy 환경이며
  생성된 SuppressFire interface와 controller message 일부는 stub을 사용했다.
  전체 저장소 회귀나 실제 Humble 전체 회귀 PASS를 의미하지 않는다.
- wrapper syntax, 기본값·override dry-run, status 실패 로그 노출 및 synthetic
  owned-process stop 검증 PASS. SW 검증 중 실제 로봇 runtime은 실행하지 않았다.
- 마지막 clean start는 Base의 SLAM/IMU 이후 Nav2 미기동·map TF 부재로 Mission 전에
  중단했다. 내부 원인은 미확정이다. stop의 종료 미완료 출력과 후속 active 0 판정도
  불일치해 미해결로 남긴다. 이전 행동 경로 성공과 기동·종료 안정성은 별도로 판정한다.

Mission의 현장 목표가 모두 해결돼도 즉시 terminal로 전환하지 않는다. Core가
Qwen을 다시 호출하지 않고 `RETURN_HOME`을 선택하며, 저장된 home pose에 대한
Navigation result가 `SUCCEEDED`일 때만 `COMPLETED` 또는
`COMPLETED_WITH_ESCALATION`으로 전환한다. 복귀 실패 시 Mission은 `RUNNING`을
유지하고 기존 Navigation 실패·재시도 계약을 그대로 따른다.

운영자 STOP은 수행 중 Mission을 `ABORTED`로 기록하지만, 이미 `COMPLETED` 또는
`COMPLETED_WITH_ESCALATION`인 Mission의 결과와 완료 이벤트는 보존한다. 이 보존은
물리 정지 경로와 별개이며, 완료 조건이나 `RETURN_HOME` 판정을 변경하지 않는다.

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
| Emergency stop | `POST /api/emergency-stop`, `/vla/emergency_stop` |

Pi의 `vla_spray_bridge`는 validated `/vla/spray_command`를 기존
`SuppressFire` action으로 변환하고 terminal result를 동일한 `action_id`와
`fire_id`로 `/vla/spray_result`에 반환한다. 이 bridge는
`fire_extinguisher.launch.py`에 포함되며, 기본 한 command당 물리 분사 시도는
최대 3회다. command의 `max_attempts`는 해당 fire의 남은 시도 예산이며,
bridge는 이 값과 설정 상한을 서버의 기존 `max_attempts`에 전달한다.

terminal result는 `SUCCEEDED`, `FAILED`, `ABORTED`, `CANCELED`,
`TIMED_OUT`으로 correlation한다. report는 correlated `SUCCEEDED`에서만
reported 상태가 된다. spray `SUCCEEDED`는 `PENDING_VERIFICATION` 전이이며
그 자체로 `EXTINGUISHED`를 의미하지 않는다. `SuppressFire` action의 ROS terminal
status가 `SUCCEEDED`여도 action result의 `success=false`이면 Bridge는 `ABORTED`로
전달한다. 결과에는 실제 `attempts`와 `verification_required`를 추가한다.
ROS action이 정상 종료되고 분사가 수행됐으면 `success=false`여도 대상별 WorldModel
verification을 시작한다. 실패 결과를 성공으로 바꾸지 않으며, hardware abort·취소는
이 경로에 포함하지 않는다. 실제 소화 여부의 authoritative owner는 계속 WorldModel이다.
WorldModel은 실제 횟수로 `spray_count`를 누적하며 같은 fire의 총 3회 제한을 유지한다.
`FULL_EXPLORATION`은 기존 `PENDING_VERIFICATION`이 해소된 뒤 다음 대상을 선택한다.
서버의 전역 화재 판정 때문에 A가 꺼져도 B가 보이면 A에 남은 횟수를 분사할 수 있으나,
최종 소화 상태는 A/B를 구분해 검증한다. 저장된 B hint 도착 뒤 메시지가 없으면 기다리고,
fresh 유효 빈 관측이면 후보를 끝내고 복귀한다. 빈 관측만으로 B의 소화를 기록하지 않는다.
Verification은 0.5초 delay 후 유효한 fire 미검출을 3회 이상 요구하고,
5초 관찰 구간이 끝나기 전에는 `EXTINGUISHED`를 확정하지 않는다. 관찰 중
같은 fire가 한 번이라도 재탐지되면 미검출 count와 관찰 시작 시각을
초기화하고 `ACTIVE`로 복귀한다. 이로써 간헐적 detector miss 3 frame이
실제 소화로 오판되는 경계를 차단한다.
3회 이내 완전 소화가 검증되면 `EXTINGUISHED`/Mission `COMPLETED`, 3회 후 잔불이면
`SUPPRESSION_FAILED`/Mission `COMPLETED_WITH_ESCALATION`으로 구분해 소방관 대응이
필요함을 보존한다.

Navigation goal owner는 하나만 활성화한다. DETERMINISTIC mode는
Frontier/StateManager/MissionExecutor가, VLA mode는 VLA Brain과 Navigation Bridge가
goal을 소유한다.

UI emergency stop은 active-action mode gate를 우회해 control mode를 `NONE`으로
전환하고 Navigation/Suppression cancel 및 four-motor zero 경로를 요청한다. Wrapper
stop도 이 HTTP boundary를 먼저 호출한 뒤 기존 bounded cancel·motor-zero·소유 process
종료 순서를 유지한다. 물리 정지는 운영자가 별도로 확인한다.

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

현재 software-only 검증은 다음 계약까지 포함한다.

- `FULL_EXPLORATION`에서 사람 위협 fire를 일반 fire보다 우선한다.
- 우선 fire 해결 뒤 남은 일반 fire 처리를 계속한다.
- 현장 목표 해결 뒤 `RETURN_HOME` 성공 전까지 Mission은 `RUNNING`이다.
- 복귀 성공 뒤 완전 해결은 `COMPLETED`, 잔존 fire는
  `COMPLETED_WITH_ESCALATION`으로 종료한다.

2026-10-14 Hardware에서는 사람+위협 fire A와 일반 fire B를 실제 센서로 관측하고
A→B→home 순서와 최종 `COMPLETED_WITH_ESCALATION`까지 연결했다. A/B 모두 물 부족으로
실제 불이 남아 각 3회 뒤 `SUPPRESSION_FAILED`였으므로, 이는 다중 대상 처리·인계·복귀
계약의 Hardware PASS이며 물리 완전 소화/`COMPLETED` PASS와는 구분한다.
