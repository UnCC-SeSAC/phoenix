# VLA Robot E2E Current Status

## Current HEAD

- Branch: `integration/vla-robot-e2e`
- Session start HEAD: `5efa4a3120753d3cb0976c6264fc3b1ad3def445`
- `5efa4a3`: Firefighter UI V2 후속 변경
- `6cbfff3`: 최신 fire suppression spray/cancel 동작 선별 통합

## Completed

- Firefighter UI V2는 기존 `/vla/status` 조회와 `/vla/mission` 제출 경계를 유지한
  software/replay 관제 화면으로 완료했다. Live Vision placeholder, Semantic Map,
  Situation Timeline, detected objects, VLA decision reason과 recent result를 제공하며
  직접 Robot 제어 기능은 없다.
- Fire suppression은 `MissionExecutor → SuppressFire Action → fire_suppression_node →
  CheckFireStatus` lifecycle, retry/cancel, Pump OFF와 servo center 복귀를 software로
  검증했다. 실제 Pump/Servo Hardware E2E는 수행하지 않았다.
- Hardware/SLAM/Nav2 integration에는 LD19 `ObstacleLayer`, 최신 footprint/DWB,
  velocity smoother와 `ObstacleFootprint.scale: 0.02`, VLA Navigation Bridge가
  반영돼 있다. 기존 isolated deployment와 vendor underlay는 수정하지 않았다.

## Software E2E Status

```text
VLA software pipeline                  PASS
Firefighter UI V2 mock/replay          PASS
Fire Suppression action lifecycle      PASS
Nav2/VLA topic bridge wiring           PASS
Actual Hardware navigation             NOT CLAIMED
```

Mock/replay의 `NAVIGATE_TO`, result lifecycle, UI 표시는 실제 Nav2 goal이나 Robot
이동을 의미하지 않는다.

## Hardware E2E Status

- 실제 Hardware/LiDAR/SLAM/Nav2 runtime 기동을 확인했다.
- `/scan_raw`, `/odom`, `/map`, `/vla/robot_pose_json` application data를
  PC에서 실제 수신했다.
- `/vla/robot_pose_json`은 안정 구간에서 약 5 Hz로 5분 연속 수신됐다. 이후 별도
  actual Mission 시도에서는 pose publisher/TF가 다시 소실돼 이 결과를 영구
  continuity PASS로 확대 해석하지 않는다.
- 실제 Mission `인명을 우선 확인해`를 1회 제출했고 production MockVLABrain은
  `NAVIGATE_TO person_0001`을 결정했으며 Resolver는 target을 정상 해결했다.
- Validator는 약 100초 stale인 Robot pose 때문에
  `로봇 위치가 없거나 너무 오래되었습니다.`로 REJECT했다.
- Actual `/vla/navigation_goal`: **0건**
- Actual `NavigateToPose` goal: **0건**
- Actual non-zero `cmd_vel`: **0건**
- Actual Robot movement: **0 m**
- Actual Pump/Servo command 및 Hardware E2E: **0건 / PENDING**
- 따라서 `ACTUAL_SHORT_NAV_PASS = NO`다.

## DDS / Network Findings

- Fast DDS SIMPLE multicast: initial discovery 이후 3~8초 freeze/burst와
  `ParticipantEntitiesInfo` Fast CDR error가 반복돼 continuity FAIL.
- known-peer SIMPLE unicast: 같은 freeze/burst가 재현돼 FAIL. Multicast 단독
  root cause 가설은 지지되지 않았다.
- runtime-only Discovery Server: continuity가 회복되지 않아 FAIL.
- 동일-host Humble/Humble 및 Jazzy/Jazzy control은 정상 범주였지만 cross-machine
  continuity 문제를 해소하지 못했다.
- 기존 Wi-Fi NIC에서는 association/SSH 소실이 실제 관찰됐다.
- Wi-Fi Power Save OFF A/B에서는 약 220초 동안 220건 연속 수신, loss 0,
  3초 이상 gap 0, largest gap 약 2.01초로 application-data continuity가 크게
  개선됐다. 영구 설정은 변경하지 않았다.
- 새 NIC에서도 초기 pose 수신 후 source/runtime 소실이 재현됐다. 당시 PC에서
  pose가 끊겼고 Pi에서도 `/vla/robot_pose_json`이 unknown,
  `map → base_footprint` TF unavailable, `vla_navigation_bridge` process
  미확인 상태였다. WorldModel은 마지막 정상 수신 timestamp에서 멈췄다.

## Current Runtime Configuration

```text
PC Wi-Fi NIC:          wlx588694f8e434
PC Robot-network IP:  10.42.0.210
Pi:                    10.42.0.1
SSH:                   lemma@10.42.0.1
Container:             IntelPi / ROS 2 Humble
PC ROS:                ROS 2 Jazzy
ROS_DOMAIN_ID:         42
ROS_LOCALHOST_ONLY:    0
RMW_IMPLEMENTATION:    rmw_fastrtps_cpp
Hardware test runtime: Wi-Fi Power Save OFF (runtime-only)
```

## Actual Short Navigation Attempts

1. 약 0.40 m person 입력은 production policy의 `distance <= 0.8 m` 조건에 따라
   정상적으로 `REPORT_PERSON`을 선택해 navigation goal이 생성되지 않았다.
2. 약 0.95 m 전방 fresh `person_0001`과 실제 Mission에서는
   `NAVIGATE_TO person_0001` 및 정상 Resolver까지 도달했지만 stale Robot pose로
   Validator가 REJECT했다.
3. pose 경로 동시 비교의 정상 구간에서는 `/vla/robot_pose_json` source timestamp가
   WorldModel `robot.pose_updated_at`으로 UTC wall-clock 기준 정상 반영됐다. 소실
   구간에서는 topic source/TF 자체가 사라지고 WorldModel이 마지막 값에서 멈췄다.

## Current Blocker

가장 먼저 확인할 경로는 다음이다.

```text
/vla/robot_pose_json
→ VLA Orchestrator pose ingestion
→ WorldModel.robot.pose / pose_updated_at
→ Validator freshness
```

현재 코드의 JSON schema와 UTC timestamp 변환은 정상 구간에서 일치했다. 직접 blocker는
Mission/Validator 시점까지 pose source와 필수 TF가 지속되지 않아 WorldModel timestamp가
stale해지는 것이다. Validator threshold를 늘리거나 우회하지 않는다.

## Next Session First Task

검증된 runtime을 재사용하고 pose source, `map → base_footprint`, Bridge process와
WorldModel `pose_updated_at`을 같은 시점에 최소 감시한다. fresh pose가 Validator까지
유지되면 약 0.95 m 전방 fresh `person_0001`과 Mission을 1회 제출하고 production
path의 Actual Short Navigation을 정확히 1회 수행한다.

## Do Not Repeat

- SIMPLE multicast/unicast A/B를 처음부터 반복하지 않는다.
- Discovery Server A/B를 반복하지 않는다.
- 동일-host DDS control test를 반복하지 않는다.
- 장시간 generic talker/listener 진단을 반복하지 않는다.
- 기존 Wi-Fi 동글의 association 원인 분석을 반복하지 않는다.
- 동일 isolated deployment clean rebuild나 이미 확인한 dependency 탐색을 반복하지
  않는다.

## Hardware Safety Status

- 실제 NavigateToPose goal, non-zero `cmd_vel`, Robot 이동은 아직 모두 0이다.
- Pump/Servo/SuppressFire Hardware command는 수행하지 않았다.
- 다음 motion은 필수 TF/LiDAR-local-costmap, 단일 goal owner와 operator 안전 확인 후
  production VLA path로 goal 정확히 1건만 허용한다.
## Pi-local VLA Control Plane (2026-08-19)

반복된 PC↔Pi Fast DDS user-data continuity 실패로 Robot control plane을 Pi-local로
변경했다. Hardware, SLAM, Nav2, TF, VLA Navigation Bridge, VLA Orchestrator,
WorldModel, Resolver, Validator, Dispatcher는 Pi에서 함께 실행한다. 핵심
`/vla/robot_pose_json`, `/vla/navigation_goal`, `/vla/navigation_result`는
cross-machine DDS를 사용하지 않는다.

PC는 compute-heavy Qwen inference만 담당한다. Pi의 `RemoteQwenBackend`가
configuration으로 받은 HTTP `/infer` endpoint에 compact semantic WorldModel과
Mission을 전송하고, PC server는 기존 `ActionDecision` JSON
(`action`, `target`, `reason`)을 반환한다. raw image, LaserScan,
OccupancyGrid, TF tree는 전송하지 않는다.

Remote timeout, connection refusal, malformed JSON, schema violation은 기존
`LLM_INFERENCE_FAILED` 또는 `LLM_OUTPUT_INVALID` blocked cycle로 처리하며
Resolver/Validator/Dispatcher를 우회하거나 navigation action을 만들지 않는다.
Hardware mock-remote 및 actual Qwen navigation E2E는 아직 수행하지 않았다.
Software verification:
- Remote success/timeout/malformed 및 기존 backend regression: 194 tests PASS.
- PC mock HTTP server: health PASS, synthetic `NAVIGATE_TO person_0001` PASS.
- Pi → PC HTTP: health PASS, synthetic inference PASS. 첫 연결 timeout 후 재시도에서
  ICMP와 HTTP가 정상 확인됐다.
- 실제 XPU Qwen server health는 PASS했지만 synthetic response가 기존 strict schema의
  `SEARCH target required` 규칙을 위반해 안전하게 reject됐다.
- 변경 source의 Pi `/tmp` 전달은 SSH/SCP가 전송 단계에서 지속 정지해 완료하지 못했다.
  따라서 Hardware mock-remote E2E와 actual short navigation은 수행하지 않았다.
## Remote Qwen Software Completion (2026-08-19)

이 절이 위의 초기 strict-schema reject 기록보다 최신이다. 기본 PC inference model은
`Qwen/Qwen3-1.7B` non-thinking deterministic generation으로 갱신했다. prompt는 기존
production action 우선순위와 action별 target 계약을 명시하며 parser strictness는
완화하지 않았다. compact payload에는 raw data 대신 기존 map pose에서 계산한
`distance_from_robot_m`과 `within_report_range` semantic field만 추가했다.

Software-only 결과:

- 실제 Qwen 대표 시나리오: far person `NAVIGATE_TO`, near person
  `REPORT_PERSON`, blocking in-range fire `EXTINGUISH`, empty targets
  `RETURN_HOME`; 모두 strict parser PASS.
- 실제 local HTTP server → `RemoteQwenBackend` → Resolver → Validator → mock
  navigation submission 1건 → mock `SUCCEEDED` → WorldModel 반영 PASS.
- software fixture는 실제 Pi의 5 Hz pose callback을 모사했다. 정적 pose fixture는
  추론 중 정상적으로 stale REJECT되며 production freshness threshold는 변경하지 않았다.
- timeout, server unavailable/HTTP 500, invalid JSON, invalid schema와 unsupported
  action은 모두 non-dispatch blocked/failure로 종료한다.
- Pi 배포는 Git을 우선하고, 불가능할 때
  `scripts/create_pi_vla_bundle.sh` bundle을 사용한다. 재개 절차는
  `docs/VLA_HARDWARE_RESUME_RUNBOOK.md`에 기록했다.

SOFTWARE PASS: RemoteQwenBackend, PC Qwen server, actual Qwen structured decision,
Resolver/Validator software flow, mock navigation result/WorldModel flow.

HARDWARE PENDING: Pi deployment, Pi-local Orchestrator runtime, actual NavigateToPose,
non-zero cmd_vel, Robot movement, actual navigation result. Hardware PASS와 Actual Short
Navigation PASS는 주장하지 않는다.
## First Remote-Qwen Hardware Attempt (2026-08-19)

Pi에 `10e1186`을 배포하고 Humble isolated overlay에서 `fire_vla_core`,
`fire_vla_bringup`, `uncc_example`을 build했다. Hardware stack은 container의
`ubuntu` 사용자로 실행해야 user-local `pyserial`과 motor controller가 연결된다.
PC `Qwen/Qwen3-1.7B` health와 Pi→PC synthetic inference는 PASS했다.

실제 Robot pose 기준 약 0.95 m 전방에 `person_0001`을 반영하고 Mission을 정확히
1회 제출했다. Qwen은 `NAVIGATE_TO person_0001`을 반환했고 Resolver는 PASS했지만,
동기 HTTP 추론 중 single-threaded ROS executor가 pose callback을 처리하지 못해
Validator가 `로봇 위치가 없거나 너무 오래되었습니다.`로 REJECT했다.

- `/vla/navigation_goal`: 0
- NavigateToPose: 0
- non-zero cmd_vel: 0
- Robot movement: 0 m
- `/vla/navigation_result`: 0
- 최초 실패 단계: Validator

production freshness threshold는 변경하지 않았다. Orchestrator의 pose subscription만
별도 callback group에 두고 2-thread executor를 사용하도록 최소 수정했다. 이 수정은
software regression 후 Hardware 재검증 전 상태이므로
`REMOTE_QWEN_HARDWARE_E2E_PASS = NO`, `ACTUAL_SHORT_NAV_PASS = NO`를 유지한다.

## Remote-Qwen Hardware Navigation and Duplicate Guard (2026-08-19)

`ac7eb68` Hardware 재시험에서 HTTP 추론 중 Pi-local robot pose가 약 5 Hz로
계속 갱신되어 executor concurrency fix와 Validator freshness PASS를 확인했다.
실제 `NAVIGATE_TO person_0001`은 ACCEPTED, non-zero cmd_vel, Robot 이동,
Nav2 `SUCCEEDED`, `/vla/navigation_result`까지 완료됐다. 그러나 Mission 1회에서
성공한 동일 navigation이 새 action ID로 한 번 더 dispatch되어 전체 E2E PASS는
보류했다.

직접 원인은 성공 결과가 `current_action`을 비우고 decision input signature를
초기화한 뒤, 계속 RUNNING인 Mission의 다음 Qwen decision이 새 action ID를 받아
기존 action-ID idempotency cache를 통과한 것이다. Mission은 navigation 성공만으로
즉시 완료하지 않는다. 동일 target의 후속 `REPORT_PERSON` 등은 계속 허용한다.

Orchestrator에 `(mission_id, action type, target_id)` semantic key guard를 추가했다.
ACCEPTED/RUNNING 및 SUCCEEDED key는 같은 Mission에서 재-dispatch하지 않는다.
FAILED, ABORTED, CANCELED, TIMED_OUT은 기존 retry semantics를 유지하며, 새 Mission과
다른 action type은 허용한다. Validator, Nav2 Bridge, Qwen contract는 변경하지 않았다.
Software regression 후 `51cf2ec`을 Pi Humble isolated overlay에 배포해 같은
Hardware scenario를 Mission 1회로 재검증했다. Qwen `NAVIGATE_TO person_0001`,
Resolver/Validator PASS, `/vla/navigation_goal` 1건, NavigateToPose 1건, non-zero
cmd_vel, 실제 Robot 이동, Nav2 `SUCCEEDED`, `/vla/navigation_result` 1건을 확인했다.
result 이후 decision loop는 허용된 `REPORT_PERSON person_0001`을 제출했으며,
동일 navigation 재결정은 `DUPLICATE_ACTION_BLOCKED`로 막혀 추가 물리 goal은 없었다.
최종 WorldModel은 `last_action=action_0001/NAVIGATE_TO/SUCCEEDED`, Mission은 후속
보고가 남아 `RUNNING`이었다.

- `DUPLICATE_NAV_GOAL_GUARD_PASS = YES`
- `REMOTE_QWEN_HARDWARE_E2E_PASS = YES`
- `ACTUAL_SHORT_NAV_PASS = YES`

## Hailo HEF Perception Integration Status (2026-08-19)

Hugging Face `song59/yolo26_base`의 두 HEF를 HailoRT 5.3으로 직접 열어 계약을
측정했다. `base_yolov26s_hailo10h.hef`는 11,644,928 bytes,
SHA-256 `cff8e534f3aa050845a1483271010ac2a780242da02a5139b55fdc8369d82b5c`,
`best_filtered_hailo10h.hef`는 11,616,256 bytes,
SHA-256 `3f141f4604e4eec9c45c49fa17455fda29b78b3e5df2c550e9ee89d64d29063f`이다.

두 모델의 runtime contract는 같다. input은 `yolov26s/input_layer1`,
`(640,640,3)` NHWC UINT8, quantization scale 1.0/zero-point 0.0이다. 기존 BGR
letterbox와 RGB NCHW float 0..1 preprocessing을 유지하고 Hailo adapter 경계에서
NHWC RGB UINT8 0..255로 변환한다. output은
`yolov26s/yolov8_nms_postprocess`, metadata shape `(2,5,100)`, FLOAT32,
`HAILO_NMS_BY_CLASS`이며 embedded NMS(score 0.20, IoU 0.70)가 포함된다.
HailoRT TensorFlow NMS output `(1,2,5,100)`의 normalized yxyx를 기존 end-to-end
`(1,N,6)` xyxy/score/class contract로 변환한다. output dict iteration order는
사용하지 않고 HEF sorted stream name으로 추출한다.

`HailoBackend`와 NMS adapter focused tests는 PASS했지만 실제 perception E2E는
HARDWARE PENDING이다. 현재 Pi에는 Hailo device(`/dev/hailo*`)와 설치된 runtime/
driver가 없고 `hailortcli scan`도 device를 찾지 못했다. 또한 모델 저장소에는
`data.yaml`, label metadata, model card가 없어 2개 class의 실제 순서를 증명할 수
없다. 따라서 기본값 `fire=0, person=1`을 추측 적용하지 않았으며, class order가
학습 artifact로 확인되기 전에는 live `/yolo_result`와 `/fire/detections`를
production label로 발행하지 않는다. Camera, depth fusion, WorldModel live 검증은
이 두 hardware/contract blocker가 해소된 다음 단계다. HEF binary는 Git에 넣지 않는다.
