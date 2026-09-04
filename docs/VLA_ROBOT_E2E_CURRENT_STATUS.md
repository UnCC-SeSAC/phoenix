# VLA Robot E2E Current Status

## Current HEAD

- Branch: `integration/vla-robot-e2e`
- Current software checkpoint before this documentation update:
  `3f01e7554177f3f4c5100d8a240f36ac1a6d5b70`
- Software regression: focused `120 PASS`, full `273 PASS`, new failures `0`
- Firefighter UI 포함 SW-only E2E: `PASS`
- Issue A integration baseline: `98e1f65ac8b39fd43d3d5f204eaa751f8ec21e77`
- This document is the integration checkpoint after the verified VLA, Hardware,
  Remote Qwen, duplicate-goal, Hailo-backend, and perception-downstream work.
- Model binaries and local runtime artifacts are not tracked.

### 2026-09-04 current presentation checkpoint

- Wrapper는 `VLA_NAVIGATION_STANDOFF_M`과 `VLA_SPRAY_RANGE_M`을 공식 launch
  argument로 Orchestrator의 `WorldModelConfig`까지 전달하고 start/status에 실제
  적용값을 표시한다. 미지정 기본값은 `0.15/0.30 m`다.
- 다음 새 노즐 시험값 `0.35/0.40 m`는 **Hardware 미검증 후보**다. 실측은 앞바퀴
  축–불꽃 `40 cm`, 렌즈–불꽃 `38 cm`, 노즐 끝–불꽃 `36 cm`이고 `36 cm`에서
  중앙 착탄했다. 성공 전 production 기본값으로 동결하지 않는다. Nav2 tolerance는
  `0.05 m`를 유지한다.
- 새 SD카드는 `interfaces` 선행 build, workspace-local HEF/ONNX/JSON과 root의
  `gpiozero`/`lgpio` import가 필요하다.
- Mission active-action gate는 current/pending action이 있을 때 새 Mission을
  `MISSION_REJECTED_ACTIVE_ACTION`으로 거절한다. FIRE_ONLY target lock은 같은 Mission의
  다른 fallback fire ID에 추가 Nav2/suppression을 실행하지 않는다.
- 같은 Mission에서 association radius 안의 새 fire ID는 최초 canonical fire ID로
  연계한다. 소화된 fire도 Mission 동안 `EXTINGUISHED`로 유지해 재실행을 막고, 새
  Mission boundary에서 entity/association을 초기화한다.
- suppression 실패 또는 소화 검증 후 재탐지는 fire별 최대 3회까지 재시도한다.
  세 번째 실패 뒤 해당 fire는 `INACCESSIBLE`로 전환하고 다른 fire 처리를 계속한다.
- Firefighter UI는 기존 semantic marker에 더해 `/image_enhanced`와 `/yolo_result`
  Camera overlay, `/map` OccupancyGrid, map→base Robot pose 및 실제 지도 위
  person/fire/selected-target 표시를 지원한다. `/map`이 없으면 기존 SVG를 유지한다.
- Wrapper stop은 기록된 start time과 command가 일치하는 owned process만 bounded
  SIGINT→SIGTERM→최종 SIGKILL 대상으로 삼는다. Orphan Nav2를 남기지 않고 zombie는
  active에서 제외하며 마지막에 Pump와 바퀴 정지를 물리 확인한다.
- 발표용 단일 명령과 순서는 `docs/VLA_HARDWARE_E2E_HAPPY_PATH.md`만 authoritative로
  사용한다. 다음 Hardware 검증은 실제 Camera/Map UI, override `0.35/0.40 m`의
  fire-only terminal SUCCESS이며 PASS 후에만 기본값 동결을 검토한다.

## Authoritative Production Hardware Test Runtime

Issue #88/#89에서 실제 장비로 확인된 조합을 이 Robot의 단일 production test
runtime으로 사용한다. `/tmp` 아래의 과거 isolated/test overlay, source-tree 직접
import, ONNX validation workspace, Stub backend는 production 후보가 아니다.

VLA production source/build/install은 팀 workspace와 분리한다.

- VLA workspace: `/ros2_ws/phoenix_vla`
- VLA branch: `integration/vla-robot-e2e`
- VLA production overlay: `/ros2_ws/phoenix_vla/install`
- team/rule-based workspace: `/ros2_ws/phoenix` (VLA production overlay로 사용 금지)
- 두 workspace 사이에서 branch를 전환하거나 `build/`, `install/`, `log/`를 공유하지 않는다.
- `colcon build`는 normal build user로 실행하며 `sudo colcon build`를 사용하지 않는다.

- Robot: `MentorPi_Mecanum`
- container: `IntelPi` (production run user `root`, working directory `/`; ROS 2 Humble)
- environment: `MACHINE_TYPE=MentorPi_Mecanum`, `need_compile=True`,
  `DEPTH_CAMERA_TYPE=ascamera`, `ROS_DOMAIN_ID=42`,
  `ROS_LOCALHOST_ONLY=0`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- Python runtime order after sourcing: prepend `/usr/local/lib/python3.10/dist-packages` to the existing `PYTHONPATH`, then append `/home/ubuntu/.local/lib/python3.10/site-packages`; this preserves ROS `rclpy`, selects NumPy 1.26.4, and keeps HailoRT available
- underlay/source order:
  `/opt/ros/humble/setup.bash` →
  `/home/ubuntu/third_party_ros2/third_party_ws/install/setup.bash` →
  `/home/ubuntu/ros2_ws/install/setup.bash` →
  `/ros2_ws/phoenix_vla/install/setup.bash`
- authoritative overlay: `/ros2_ws/phoenix_vla/install`
- inference module:
  `/ros2_ws/phoenix_vla/install/image_pipeline/lib/python3.10/site-packages/image_pipeline/yolo.py`
- HEF: `/ros2_ws/phoenix_vla/Hailo/models/baseline_yolo26_neural_norm.hef`
  - size: `11288576` bytes
  - SHA-256: `67496fe3eefb710bef56ce9fd30af0102520c234f697f715ed0935a881e75aad`
- postprocess: `/ros2_ws/phoenix_vla/Hailo/models/best_sim_postprocess.onnx`
  - size: `106676` bytes
  - SHA-256: `b05022e4741258840e48143e7dc0f88cc676d11a842e6950623c59cf189f60b4`
- output mapping: `/ros2_ws/phoenix_vla/Hailo/models/config_onnx_best_sim.json`
  - size: `1073` bytes
  - SHA-256: `33afa6da43c84ceaa9cae992c8372acf8008bc19e56b9b86b8425997172d2278`
- model binaries remain Git-untracked. The VLA copies were provisioned from the
  previously verified production artifacts and verified byte-identical; do not
  resolve or load them from the team workspace at runtime.
- isolated VLA deployment verification (2026-08-22): focused builds for
  `image_pipeline`, `fire_vla_core`, `uncc_example`, and `fire_vla_bringup` PASS;
  `vla_spray_bridge` PRESENT; all four package prefixes resolve from
  `/ros2_ws/phoenix_vla/install`; cross-workspace install NONE; root-owned
  `build/`/`install/`/`log/` artifacts 0.
- LiDAR: `LDLiDAR_LD19`, `/dev/ldlidar → /dev/ttyUSB0`, `230400`
- Robot/LiDAR/SLAM/Nav2 base entrypoint:
  `ros2 launch uncc_example uncc_frontier.launch.py start_frontier:=false start_mission:=false start_vision:=false`
- LiDAR authoritative launch:
  `peripherals/launch/lidar.launch.py` → `include/ldlidar_LD19.launch.py`
- Camera: `ros2 launch peripherals depth_camera.launch.py`
- preprocessing: `ros2 run image_pipeline preprocess_node --ros-args -r __node:=rgb_preprocess_node -p input_topic:=/ascamera/camera_publisher/rgb0/image -p camera_info_topic:=/ascamera/camera_publisher/rgb0/camera_info -p output_topic:=/image_enhanced -p output_camera_info_topic:=/image_enhanced/camera_info -p mode:=passthrough`
- inference: `ros2 launch image_pipeline yolo.launch.py` with the HEF path above,
  `backend:=hailo`, `layout:=end2end`, and `class_names:="['fire','person']"`
- depth fusion: `ros2 launch image_pipeline detection_3d.launch.py`
- SLAM: `uncc_example/launch/slam_mapping.launch.py` (included by the entrypoint)
- Nav2: `uncc_example/launch/nav2_online.launch.py` (included by the entrypoint)
- VLA: `ros2 launch fire_vla_bringup topic_bridge_vla.launch.py start_perception_bridge:=true llm_backend:=remote_qwen remote_qwen_endpoint:=http://<CURRENT_PC_IP>:8088/infer remote_qwen_timeout_sec:=10.0` plus `ros2 launch uncc_example vla_navigation_bridge.launch.py`. The last successful stationary suppression test used `192.168.100.124:8088`; confirm the current PC address instead of treating it as a fixed endpoint.
- suppression bridge/action server: `ros2 launch uncc_example fire_extinguisher.launch.py`; starting it does not actuate Hardware, but an actual suppress goal still requires explicit operator approval
- current production defaults: fire confidence `>= 0.40`, spray range
  `<= 0.30 m`, navigation stand-off `0.15 m`; 다음 새 노즐 시험만 wrapper override
  stand-off `0.35 m`, spray range `0.40 m`를 사용한다. `0.80 m`는 historical 값이다.

### Suppression Hardware runtime contract

The stationary Hardware PASS and the subsequent minimal graph reproduction used the
container default user `root` and default working directory `/`. The working directory
is part of the `lgpio` runtime contract because its notification FIFO is created as
`/.lgd-nfy0`. Run the suppression layer with:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/third_party_ros2/third_party_ws/install/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
source /ros2_ws/phoenix_vla/install/setup.bash
export MACHINE_TYPE=MentorPi_Mecanum need_compile=True DEPTH_CAMERA_TYPE=ascamera
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH=/usr/local/lib/python3.10/dist-packages:${PYTHONPATH}:/home/ubuntu/.local/lib/python3.10/site-packages
cd /
ros2 launch uncc_example fire_extinguisher.launch.py
```

Expected readiness is one `/fire_suppression_node`, one `/vla_spray_bridge`, and
exactly one `/suppress_fire` action server. The Servo starts detached
(`AngularServo(initial_angle=None)`), so launch readiness must not move or continuously
energize it. An authorized suppression goal re-enables Servo PWM through the first angle
command; completion returns it to center and detaches it again. Never send a suppression
goal without explicit Hardware authorization. Persistent buzzing during an authorized
movement can indicate a mechanical stop, nozzle/hose load, or center/PWM calibration
mismatch; it is not by itself evidence of a Pump command.

### Ethernet to Wi-Fi transition contract

Use `lemma@192.168.100.128` for fixed preparation. Before Robot motion, remove
Ethernet and verify Wi-Fi connectivity. In the final Issue #89 attempt, the processes
remained alive after cable removal, but Fast DDS participants retained the old
interface and the VLA navigation client and suppression server disappeared from the
ROS graph. Refreshing only the ROS daemon was insufficient.

The verified transition recovery is:

```text
remove Ethernet
→ verify Wi-Fi and current Qwen endpoint connectivity
→ clean-stop only topic_bridge_vla.launch.py,
  vla_navigation_bridge.launch.py, and fire_extinguisher.launch.py
→ restart those three launch trees exactly once with Wi-Fi active
→ refresh the ROS daemon
→ require NavigateToPose server/client = 1 and /suppress_fire server/client = 1
```

Do not restart Base, Camera, YOLO, SLAM, or Nav2 for this interface transition. The
remote-Qwen launch contract is `llm_backend:=remote_qwen`,
`remote_qwen_endpoint:=http://<CURRENT_PC_IP>:8088/infer`, and
`remote_qwen_timeout_sec:=10.0`. Resolve the endpoint from the current PC network;
do not embed a historical address. During the verified transition, disabling power
save on the active PC Wi-Fi interface restored three consecutive Pi-to-Qwen HTTP 200
health checks.

The default test flow is:

```text
known production launch tree clean stop
→ verify no duplicate production process once
→ source the authoritative environment
→ start the authoritative launch sequence exactly once
→ minimum readiness for the pending test
→ test
→ result
```

An existing runtime is reused only when its launch owner, production environment,
lifecycle state, and absence of duplicate/stale processes are all clear. Otherwise
restart only the affected production launch tree. OS reboot and Docker restart are
not default recovery steps. A side-effect command that times out is never resent
until process existence has been checked once.

### Remote Qwen / Intel XPU recovery

The verified PC uses Intel Arc B580 with the `xe` driver and `torch 2.7.1+xpu`. If
Qwen returns HTTP 503 with `UR_RESULT_ERROR_DEVICE_LOST`, and a Qwen-only clean
restart then reports `UR_RESULT_ERROR_UNKNOWN`, do not repeat server restarts. Run the
XPU probe first; if device state remains unavailable, clean reboot the PC, require the
XPU probe to pass, then start the authoritative Qwen server exactly once. This recovery
restored HTTP 200 during the stationary suppression test.

## Architecture

Robot state and control remain Pi-local:

```text
Pi: Hardware → SLAM → Nav2 → VLA Navigation Bridge
    → VLA Orchestrator → WorldModel → Resolver → Validator → Dispatcher

Pi RemoteQwenBackend ↔ HTTP JSON ↔ PC Qwen/Qwen3-1.7B inference only
```

The critical topics `/vla/robot_pose_json`, `/vla/navigation_goal`, and
`/vla/navigation_result` are Pi-local. Camera, depth, TF, WorldModel, Resolver,
Validator, Dispatcher, Nav2, `cmd_vel`, and motor control also remain Pi-local.
PC↔Pi ROS 2/Fast DDS user-data continuity repeatedly failed while Pi-local pose, TF,
and Bridge paths were valid, so cross-machine DDS was removed from the Robot control
critical path. The Pi sends only compact semantic WorldModel/Mission data to the PC
over HTTP/JSON. The PC runs Qwen inference only and returns the existing strict
`action`/`target`/`reason` decision contract; raw camera/depth and Robot control data
are never part of that HTTP boundary.

## Completed

- Software VLA E2E and strict Resolver/Validator/Dispatcher result lifecycle.
- Remote Qwen software E2E with `Qwen/Qwen3-1.7B`, non-thinking deterministic mode.
- Pi-local Remote Qwen Hardware short-navigation E2E with actual Robot movement.
- Navigation result propagation into WorldModel.
- Per-Mission semantic duplicate physical-action guard.
- Production `/fire/detections` downstream software E2E through WorldModel and
  actual Remote Qwen plus mock navigation result.
- Hailo HEF backend implementation and measured HEF tensor contract adapter.
- Firefighter UI software/replay and fire-suppression action lifecycle software tests.

## Hardware E2E Status

Actual Hardware PASS path:

```text
Pi-local WorldModel
→ HTTP
→ PC Qwen3-1.7B
→ NAVIGATE_TO person_0001
→ Resolver PASS
→ Validator PASS
→ /vla/navigation_goal (1)
→ NavigateToPose (1, ACCEPTED)
→ non-zero cmd_vel
→ actual Robot movement (~0.79 m)
→ automatic stop
→ Nav2 SUCCEEDED
→ /vla/navigation_result (1)
→ WorldModel result
```

- `REMOTE_QWEN_HARDWARE_E2E_PASS = YES`
- `ACTUAL_SHORT_NAV_PASS = YES`

The earlier pose-starvation failure was fixed with a separate pose callback group and
a two-thread executor. Pose freshness continued during synchronous HTTP inference;
Validator freshness thresholds were not changed or bypassed.

### Duplicate navigation guard

A successful `NAVIGATE_TO person_0001` previously caused the still-running Mission to
request the same physical action again. Orchestration now keys idempotency by:

```text
(mission_id, action_type, target_id)
```

The same semantic action is blocked while ACCEPTED/RUNNING and after SUCCEEDED.
`ABORTED` also retains the semantic key so a terminal Nav2 abort cannot immediately
redispatch the same Mission/action/target. `FAILED`, `CANCELED`, and `TIMED_OUT` retain
the existing retry semantics. A new Mission may use the same target, and a different
action such as `REPORT_PERSON` remains allowed. Hardware revalidation of the successful
path produced one navigation goal, one NavigateToPose, zero
duplicate NAVIGATE_TO actions, and an allowed follow-up REPORT_PERSON.

- `DUPLICATE_NAV_GOAL_GUARD_PASS = YES`

## Qwen Software Status

The default PC model is `Qwen/Qwen3-1.7B` in non-thinking deterministic mode. Strict
schema scenarios PASS:

- far person → `NAVIGATE_TO`
- near person → `REPORT_PERSON`
- blocking/in-range fire → `EXTINGUISH`
- no target → `RETURN_HOME`

Timeout, unavailable server, HTTP 500, invalid JSON, invalid schema, and unsupported
action terminate without physical dispatch. Parser strictness and production Validator
freshness remain unchanged.

- `REMOTE_QWEN_SOFTWARE_E2E_PASS = YES`

## Perception Status

Production downstream software path PASS:

```text
/fire/detections production JSON
→ pixel + depth
→ CameraInfo backprojection
→ source-time camera optical frame → map TF
→ map (x,y)
→ stable entity ID
→ WorldModel
→ Remote Qwen
→ Resolver / Validator
→ mock navigation SUCCEEDED
→ WorldModel result
```

Deterministic verified fixtures:

- person: pixel `(320,240)`, depth `0.95 m`, CameraInfo
  `fx=fy=400, cx=320, cy=240` → map `(0.95,0.0)` → `person_0001`
- fire: pixel `(360,240)`, depth `2.0 m` → map `(2.0,-0.2)` → `fire_0001`

Null/unknown depth is ignored. Non-positive depth, out-of-frame pixel, unsupported
class, malformed JSON, stale source timestamp, and confidence below the existing
WorldModel threshold do not create an unsafe entity/action.

- `PERCEPTION_DOWNSTREAM_SW_E2E_PASS = YES`
- `REMOTE_QWEN_PERCEPTION_SW_E2E_PASS = YES`

Live Hardware perception checkpoint for Issue #88:

```text
actual ASCAMERA RGB → preprocess → split HEF neural inference
→ companion ONNX postprocess → person bbox/confidence
```

- Camera stream and passthrough preprocessing: PASS
- Offline split-HEF detection: PASS
- Live HEF person detection from actual ASCAMERA frame: PASS
- Actual person confidence (latest downstream checkpoint): `0.500`
- Actual person bbox `(x1,y1,x2,y2)`:
  `(257.41,279.07,393.14,467.47)`
- `OFFLINE_HEF_DETECTION_PASS = YES`
- `LIVE_HEF_PERSON_DETECTION_PASS = YES`

The earlier live `detections: []` failure had two confirmed causes. First, the
repository assumed a single `HAILO_NMS_BY_CLASS` output, while the deployed HEF emits
six raw neural heads that require the hardware-team companion ONNX postprocess.
Second, the ROS shell resolved a stale installed backend ahead of the isolated fixed
backend. Commit `e28491dacd8a591f0aaee6df4ac1640460eaa82f` adds the measured split-HEF contract
while retaining the existing single-output NMS HEF path.

Live depth fusion also passed using that actual person detection:

- `LIVE_PERSON_DEPTH_PASS = YES`
- actual depth: `0.371 m`
- camera XYZ: approximately `(-0.0047,0.0842,0.3710) m`

The VLA perception Adapter continues to own CameraInfo backprojection and source-time
TF conversion. Commit `8a6fece04210ac197423169ae68e8a0a4927570f` contains the
Adapter compatibility correction used by this Hardware checkpoint.

### LD19 and localization runtime checkpoint

The first map-localization attempt had valid `base_link → ascamera_color_0` and
`odom → base_link`, but no continuous `map → odom`: `/scan_raw` briefly appeared and
then stopped, so SLAM could not maintain its transform.

The authoritative runtime was confirmed as:

- driver source:
  `/home/ubuntu/third_party_ros2/third_party_ws/src/ldlidar_stl_ros2`
- launch: `peripherals/launch/lidar.launch.py`
  → `include/ldlidar_LD19.launch.py`
- model / device / baud: `LDLiDAR_LD19`,
  `/dev/ldlidar` → `/dev/ttyUSB0`, `230400`

Serial input and the LD19 packet contract remained valid. After confirming exclusive
serial ownership, fully stopping the driver, and cleanly restarting the authoritative
launch, `/scan_raw` produced actual LaserScan messages for 35 seconds at an average
of approximately `9.91 Hz`; continuous `map → odom` was then observed. No driver,
configuration, SLAM, or TF source was changed.

This is recorded as a **runtime transient issue**: an intermittent parser/scan-assembly
stall. The detailed trigger was not reproduced and therefore is **not a confirmed
root cause**. Model/protocol/baud mismatch and persistent USB/CRC failure were excluded
by the captured stream; they must not be reported as the cause.

Issue #88 is **CLOSED**. A later Hardware session acquired a fresh person observation
and completed the remaining source-time TF, map localization, WorldModel, and UI/report
path. The earlier unavailable-observation checkpoint remains historical and is not the
current result.

Current Issue #88 status:

- `LIVE_HEF_PERSON_DETECTION_PASS = YES`
- `LIVE_PERSON_DEPTH_PASS = YES`
- `LIVE_PERSON_MAP_LOCALIZATION_PASS = YES`
- `LIVE_PERSON_WORLDMODEL_PASS = YES`
- `LIVE_PERSON_UI_REPORT_PASS = YES`

No Nav2 goal, exploration, `cmd_vel`, Motor, Pump, Servo, or fire suppression was run
for Issue #88.

## Hailo Status

The current Raspberry Pi reference is read-only hardware-team content under
`/home/lemma/Hailo`:

- HEF: `/home/lemma/Hailo/models/baseline_yolo26_neural_norm.hef`
- reference inference: `/home/lemma/Hailo/yolo26_split_test.py`
- input: `baseline_yolo26_neural/input_layer1`, `(640,640,3)`, UINT8 RGB NHWC
- outputs: six FLOAT32-dequantized raw neural heads
  (`conv61`, `conv77`, `conv91`, `conv64`, `conv80`, `conv94`)
- postprocess: output name/shape mapping from `config_onnx_best_sim.json`
  → `best_sim_postprocess.onnx` → `output0 [1,300,6]`
- class contract: `0=fire`, `1=person`

This HEF does **not** contain the final Hailo NMS layer. The reference path requests
FLOAT32 outputs from HailoRT, maps NHWC heads to the exact ONNX NCHW input names, and
parses `[x1,y1,x2,y2,score,class_id]`. Known-image Hardware inference produced
`fire=0.7890` and `person=0.6020`; the captured live frame also produced person
detections `0.7759` and `0.5000` through the unmodified reference script.

`HailoBackend` now detects single-output NMS versus multi-output split HEFs. For the
split artifact it uses the measured HailoRT 5.3 asynchronous binding lifecycle and
companion postprocess inside the backend Adapter, leaving the downstream detection
contract unchanged. HEF/ONNX binaries remain untracked.

## ONNX Runtime Model Validation

User-provided untracked `best_base.onnx` was validated without committing the binary.
SHA-256 is `2e04e029ca825b021990bbeb44ecd1ba5f8c2fe41739b222b7d54fdb9ed10f9d`.
ONNX checker and ONNX Runtime 1.29 PASS with opset 17, input
`images [1,3,640,640] float32`, output `output0 [1,300,6] float32`, metadata
`end2end=True`, and class names `0=fire, 1=person`. The existing Phoenix letterbox,
BGR→RGB, float 0..1 preprocessing and end-to-end decoder completed dummy inference in
about 101 ms on PC CPU; a blank image correctly produced zero detections.

OpenCV DNN 4.6.0 still cannot import the model's attention `Split` node. An explicit
`OnnxRuntimeBackend` now implements the existing `infer(blob) -> list[np.ndarray]`
port and is selected with `backend:=onnxruntime`. The selected runtime must provide the
optional `onnxruntime` Python package; absence is reported with an explicit installation
error. Existing OpenCV, Ultralytics, Hailo, and Stub paths are unchanged; there is no
silent fallback. The ONNX live attempt remains a historical `NOT_VERIFIED` result
because Pi SSH access was lost after startup. It is not the current production
inference path; the verified production candidate is the Hailo split HEF above.

## Person Hardware E2E — Issue #88

Issue #88 is CLOSED. Actual ASCAMERA, split-HEF person detection, depth, source-time
TF, map localization, `person_0001` WorldModel ingestion, and Firefighter UI/report
were verified on Hardware. No Nav2 goal, Motor, Pump, or Servo command was used for
that perception/localization validation.

## Fire navigation and suppression — Issue #89

Hardware PASS before navigation:

```text
actual fire → HEF bbox/confidence → depth → map (x,y)
→ WorldModel → VLA decision NAVIGATE_TO fire_0001
```

The first actual Nav2 goal was accepted but terminated `ABORTED` after path planning
failed. The terminal lifecycle cleared the bridge pending goal, but Orchestrator then
redispatched the same target four times because `ABORTED` released the semantic key.
The Robot was stopped, autonomous goal senders were terminated, and Pump/Servo commands
remained zero. A later single-goal Hardware retry was NOT RUN because Pi SSH/runtime
continuity failed before goal submission.

Software fixes and validation:

- `1c8c64238a91d6c120eddac9faaa95e100bfee14`: keep the same-Mission semantic key
  after `ABORTED`; focused navigation lifecycle/topic bridge tests PASS.
- `31fec79580a81093a59858da3d9339128cd89b51`: count every terminal suppression
  attempt so the existing `max_spray_attempts=2` bounds FAILED retries.
- Mock lifecycle PASS: `NAVIGATE_TO fire_0001 → SUCCEEDED → EXTINGUISH fire_0001
  → SUCCEEDED → WorldModel/status/UI contract`.
- Same fire navigation after SUCCEEDED: zero additional dispatches.
- Suppression success: one command and zero additional commands.
- Suppression FAILED: one permitted retry, then Validator blocks further commands at
  the existing two-attempt limit.

Stationary suppression Hardware E2E subsequently passed with no Robot motion:

```text
Camera → split HEF / YOLO → fire observation → WorldModel → Qwen
→ VLA EXTINGUISH fire_0001 → /vla/spray_command → vla_spray_bridge
→ /suppress_fire → Servo → Pump
```

- fire confidence: `0.659`
- fire depth: `0.843 m`
- WorldModel accepted: YES
- VLA decision: `EXTINGUISH fire_0001`
- suppression request: 1
- Servo: EXECUTED
- Pump: EXECUTED
- Pump OFF: CONFIRMED
- Robot motion: 0
- `STATIONARY_SUPPRESSION_HW_E2E_PASS = YES`

The terminal result was `ABORTED` because no extinguishing water was loaded during
this stationary test. The Pump and Servo executed correctly, but the flame remained,
so continued fire detection correctly prevented terminal success. This is not a
suppression software/Hardware path failure:

- Stationary suppression Hardware path: PASS
- Actual extinguish verification: NOT YET VALIDATED
- `ABORTED` cause: no extinguishing water was loaded during the test

The stationary test temporarily used fire confidence `>= 0.20` and spray range
`<= 1.00 m` for one fixed-position E2E observation. That temporary parameter wiring
was removed after the test. Both current production source and the Pi production
install use the authoritative `>= 0.60` and `<= 0.80 m` values.

### Final Full E2E attempt checkpoint (2026-08-22)

The production stack reached readiness, transitioned from Ethernet to Wi-Fi using the
procedure above, and reached the live fire/Qwen boundary:

```text
production readiness
→ Ethernet removal and Wi-Fi runtime recovery
→ fresh fire detection
→ confidence 0.659 at about 2.14 m
→ WorldModel accepted (robot_within_spray_range=false)
→ mission delivery
→ Qwen inference
```

The first fire observation was `0.582`, below the production confidence threshold;
the threshold was not changed. Subsequent fresh observations reached `0.659`. A first
one-shot mission CLI command printed locally but left WorldModel mission null and
dispatched zero goals. After semantic non-delivery was confirmed, using
`ros2 topic pub --once -w 1 /vla/mission ...` explicitly waited for its subscriber.

The first inference attempt timed out because the Wi-Fi VLA launch omitted the verified
`remote_qwen_timeout_sec:=10.0` argument. After restoring it, the next inference returned
HTTP 503. The XPU probe still passed on Intel Arc B580, but Qwen shutdown reported
`invalid device pointer` in `XPUCachingAllocator`. Treat this as the existing Intel XPU
runtime-corruption family: do not repeatedly restart Qwen; clean reboot the PC, rerun
the XPU probe, then start authoritative Qwen once and require HTTP 200.

Physical dispatch never started in this attempt:

- `NAV2_GOAL_COUNT = 0`
- `ROBOT_MOTION = 0`
- `SUPPRESSION_REQUEST = 0`
- `PUMP = 0`
- `FULL_E2E_RESULT = BLOCKED_AT_QWEN_XPU_RUNTIME`

This is not a Nav2 or suppression failure. Resume after the PC clean reboot at XPU
probe → authoritative Qwen clean start → existing Pi runtime inspection → the verified
Ethernet/Wi-Fi transition and final Full E2E.

The Pi checkpoint was inspected read-only after the test. Production source was on
`add-map-camera` at `1d8a471`, with only the fire-status sensor-data QoS diff present
as a tracked modification. The suppression action, `vla_spray_bridge`, entry point,
launch wiring, QoS implementation, and production VLA config were byte-identical to
PC integration `eaa26d4`; no missing production source change needed importing.

Issue #89 remains OPEN because the complete moving and actual-extinguish path has not
yet been validated.

## Current Pending

- Issue #89: actual Nav2 approach and goal arrival
- Issue #89: safe stop followed by suppression with water
- Issue #89: actual flame removal and terminal `SUCCESS`
- full fire perception → Nav2 → suppression → extinguished result/UI confirmation

## Rule-based driving integration — Issue A

The latest driving branches were compared against the integration baseline:

- `state_manage` @ `3f2db36bced34287c48e8f9a8d193f46e015e209`
- `nav_local_plan` @ `620e7a6268d3bcf802157a126ee5882683a4ecc5`
- `caron2002/local_plan_avoidance` @
  `daafb4b24a13c308251b49c47a9ba17ff180ee25`
- `frontier_basic` @ `33140b6bb7f71035bd7ff952361e1dbdf8cb5841`

The branches were not merged wholesale. Their histories share an older baseline and
contain overlapping or obsolete Nav2, image-pipeline, dummy-test, and VLA bridge
changes. The integration selectively imports the current Rule-based lineage from
`state_manage`:

- a `FrontierStateController` serializes START/STOP ownership and confirms the
  Frontier-owned Nav2 goal has stopped before MissionExecutor dispatches a semantic
  target;
- `uncc_frontier.launch.py` preserves standalone Frontier autostart, but transfers
  lifecycle ownership to MissionExecutor only when mission mode is explicitly enabled;
- StateManager battery threshold transitions use a three-second debounce;
- fire suppression uses the latest team-owned duplicate-goal rejection, cancel-safe
  Pump/Servo shutdown, servo detach, SIGTERM cleanup, and GPIO release behavior.

The current integration Nav2 settings remain authoritative. In particular, the
verified 2D-LiDAR `ObstacleLayer`, conservative velocity smoother, footprint, and
`ObstacleFootprint.scale` configuration were retained. The alternative branch
versions would restore a VoxelLayer, relax verified velocity limits, or use an invalid
critic parameter. The older ONNX/dummy Hardware launch and changes that delete the VLA
Navigation Bridge or replace the production perception contract were also excluded.

Software validation covers Python/launch syntax, the StateManager debounce regression,
existing `uncc_example` tests, affected package builds, VLA regressions, and
`git diff --check`. Actual Rule-based driving and suppression Hardware validation is
`HARDWARE_PENDING`; no motion or actuator command was issued for Issue A.

Issue B formalizes the Rule-based status, mission, navigation, detection, and
suppression interface consumed by the Firefighter UI without changing the existing
VLA contract.

### Latest team-branch audit (2026-08-21)

The remote branches were fetched with pruning and compared by merge-base, unique
commits, and file-level semantics against integration baseline
`66033723cc049da7800a507ce8f450b448581ffe`. No new team commit was imported:

- `state_manage` @ `12d7befafb525f261adf0376ca32249fd27d8d38`: **SKIP**. Its useful
  Frontier/StateManager/suppression lineage is already selectively present. The new
  vision variant removes current confidence, source timestamp, and malformed-input
  validation, while its Hailo implementation predates the verified split-HEF fixes.
- `edit_nav2_params` @ `5cfedff691079bcc7ba1ef61ec752f4d3283b276`: **SKIP**. It is already
  merged into `state_manage`; importing it would revert verified costmap publication
  and velocity-smoother values and replace `ObstacleFootprint.scale` with a non-matching
  parameter key.
- `img_grpc_protocol` @ `a1c105a00167702aa76f50784ddff8abc5f6a0d8`: **SKIP**. The added raw
  image gRPC path is outside the Pi-local perception architecture, and the final servo
  edit uses an undefined `NONE` token. Neither change is production-ready here.
- `nav_local_plan` @ `620e7a6268d3bcf802157a126ee5882683a4ecc5` and
  `caron2002/local_plan_avoidance` @ `daafb4b24a13c308251b49c47a9ba17ff180ee25`:
  **SKIP** as older overlapping navigation lineages. Their compatible 2D obstacle and
  Frontier ownership changes are already represented by the current integration.
- `frontier_basic` @ `33140b6bb7f71035bd7ff952361e1dbdf8cb5841`:
  **ALREADY_INCLUDED** by ancestry.
- `albitro/image_processing` @ `0bf507e`: **SKIP** as an older video-capture prototype;
  current `image_pipeline` remains the upstream perception contract.
- `feature/vla-brain` @ `c75c65b`: **ALREADY_INCLUDED** semantically through the newer
  integration VLA/UI lineage; its separate standalone history is not merged wholesale.

Focused VLA #88/#89, Rule-based state/contract, perception, and UI regression produced
88 PASS. Temporary Jazzy builds of `interfaces`, `image_pipeline`, `fire_vla_core`, and
`fire_vla_bringup` PASS. `uncc_example` build is `NOT_RUNNABLE` in isolation on this PC
because the Robot/vendor packages `controller`, `frontier_exploration_ros2`,
`navigation`, and `peripherals` are not installed in the local underlay; production
code was not changed to bypass that environment limitation. Hardware validation was
`NOT_RUN`, and the existing Rule-based UI contract did not change. The next task may
proceed directly with two-mode UI software verification/completion.

### Rule-based UI contract — Issue B

Issue B provides an integration-only Port–Adapter boundary:

- `/rule_based/status`: versioned Rule-based status snapshot
- `/rule_based/mission`: the shared UI mission envelope, restricted to strict
  `START` and `STOP` commands
- `/mission/enabled`: internal MissionExecutor enable boundary

The snapshot covers FSM state/target, Nav2 action status, Frontier lifecycle,
detection targets, battery, and SuppressFire action status. `STOP` cancels
MissionExecutor-owned Nav2/SuppressFire work and serializes Frontier shutdown through
the existing controller. It does not send direct actuator commands. The VLA
`/vla/status` and `/vla/mission` contracts are unchanged. See
`docs/RULE_BASED_UI_CONTRACT.md`.

Hardware validation is `NOT_RUN`. Issue C can now connect the Firefighter UI mode
selector to these two Rule-based topics without embedding backend/FSM logic in the
frontend.

### Firefighter UI two-mode integration — Issue C

The existing Firefighter UI now provides a shared-shell mode selector:

- `VLA Brain` preserves the existing decision, reason, current action, semantic
  map, WorldModel entities, validation, submission, and recent-result view.
- `Rule-based` consumes the Issue B `/rule_based/status` contract and presents
  FSM/target, Nav2, Frontier exploration, detections, battery, suppression, and
  last-command state. Its mission boundary routes to `/rule_based/mission`.

The browser selects a mode through the local HTTP API; ROS topic routing remains
inside `firefighter_ui_node`. Therefore backend/FSM ownership is not duplicated in
frontend code, and the original `/vla/status` and `/vla/mission` behavior remains
the default. Software API, rendering-contract, launch syntax, and VLA regression
tests cover this change. Actual Rule-based Robot driving and suppression remain
`HARDWARE_PENDING` and were not run for Issue C.

Final software-only verification at commit
`626626b850ff046912ef6f8c17f74f69b12f803b` confirmed:

- `VLA_MODE_PASS = YES` and `RULE_BASED_MODE_PASS = YES`;
- `VLA → Rule-based → VLA` selects isolated snapshots and clears the previous mode's
  object, map, timeline, and current-state display before fetching the next snapshot;
- an in-flight response for the previous mode is ignored after a selector change;
- one VLA Mission routes only to `/vla/mission`;
- one Rule-based `START` and one `STOP` route only to `/rule_based/mission`;
- Rule-based free text is rejected at the HTTP boundary instead of being acknowledged
  and rejected later by the ROS consumer;
- the Rule-based UI uses explicit `START` and `STOP` controls while VLA retains its
  natural-language Mission form.

Focused HTTP, StatusStore, frontend-contract, mission-routing, and Rule-based producer
tests: 41 PASS. Python compile, JavaScript syntax, `image_pipeline`/`fire_vla_core`
build, and `git diff --check`: PASS. Local mock snapshots were also published to both
status topics and read back through their separate HTTP mode endpoints. Hardware UI
validation remains `NOT_RUN`.

## Next Milestone

At the 2026-08-22 end-of-day checkpoint, all identified production launch/process
groups, including Base/Nav2/Camera, VLA/Navigation Bridge, and Suppression, were
stopped. No Nav2 goal, suppression request, Pump command, or Servo suppression command
was sent during shutdown. Start the authoritative production stack cleanly from the
isolated VLA overlay next session; do not reuse historical defunct processes.

For the next Issue #89 Hardware test, reuse the verified Camera/HEF/depth/map and
stationary suppression results without benchmarking them again:

```text
authoritative production stack clean stop
→ verify the untracked production HEF/postprocess in /ros2_ws/phoenix_vla
→ load the authoritative environment and /ros2_ws/phoenix_vla/install
→ start the authoritative production stack exactly once
→ minimum readiness
→ fire detection → Nav2 approach → safe stop
→ suppression with water → actual extinguish → terminal SUCCESS
```

If Nav2 returns `ABORTED`, stop after confirming terminal delivery, zero redispatch,
and Robot stop. Treat path-planning failure separately from the fixed lifecycle bug.

## Do Not Repeat

- Do not restart cross-machine DDS multicast/unicast diagnosis.
- Do not restart Discovery Server diagnosis.
- Do not relax or bypass Validator freshness.
- Do not reselect the Qwen model without a new requirement.
- Do not redesign the duplicate-goal lifecycle; verify the committed ABORT guard with
  one Hardware goal when runtime access is restored.
- Do not reimplement the already-passed short-navigation path.
- Do not directly inject final map coordinates when validating live perception.
- Do not commit HEF/PT/ONNX binaries or user runtime artifacts.

## Checkpoint Validation

- image_pipeline pytest: 351 PASS
- focused fire_vla_core/perception/Remote-Qwen regression: 84 PASS
- Issue #89 focused navigation/suppression/UI lifecycle regression: 68 PASS
- Issue #89 direct WorldModel/orchestrator/spray regression: 49 PASS
- Python `py_compile`: PASS
- colcon build: `image_pipeline`, `fire_vla_core`, `fire_vla_bringup` PASS
- `git diff --check`: PASS
- full fire_vla_core pytest on this PC Jazzy environment: NOT RUNNABLE because the
  installed Python environment lacks Robot-specific ROS `action_msgs` (and related ROS
  Python message/launch dependencies). This environment limitation was not bypassed by
  changing production code.

## Checkpoint Summary

PASS:

- Software VLA E2E
- Remote Qwen software E2E
- Remote Qwen Hardware short-navigation E2E
- actual Robot movement and automatic stop
- navigation result propagation
- duplicate navigation guard
- perception downstream software E2E
- HailoBackend split-HEF implementation
- offline HEF fire/person detection
- Issue #88 live person Camera/HEF/depth/map/WorldModel/UI Hardware E2E
- LD19 `/scan_raw` recovery and continuous `map → odom`
- Issue #89 actual fire HEF/depth/map/WorldModel and VLA decision
- Issue #89 stationary Camera-to-Qwen-to-Servo/Pump Hardware E2E
- Issue #89 Pump OFF confirmation with Robot motion 0
- Issue #89 software navigation-success → suppression-success → status/UI lifecycle
- Issue #89 bounded suppression failure retry at the existing two-attempt limit

PENDING:

- Issue #89 actual Nav2 approach, goal arrival, and safe stop
- Issue #89 suppression with water and actual flame extinguish verification
- full fire perception-to-navigation-to-suppression terminal `SUCCESS`

## Issue #89 Actual Full E2E Update (2026-08-24)

Fresh fire-only Hardware run에서 다음 경로를 실제로 통과했다.

```text
Fresh fire/WorldModel/Qwen: PASS
NAVIGATE_TO fire / Nav2 result: PASS / SUCCEEDED
Robot stop: PASS
EXTINGUISH / Servo / Pump / Pump OFF: PASS
Physical flame extinction: FAIL
Terminal result: ABORTED
```

Nav2는 fire target 약 `(0.79, 1.30)`으로 실제 주행해 robot pose 약
`(0.58, 1.22)`에서 정지했다. 그러나 물줄기가 실제 화염을 빗나가 사용자가
수동 소화했다. 현재 `FIRST_REAL_FAILURE`는 일반 SLAM/Nav2 실패가 아니라 fire
자체 좌표를 goal로 사용하는 접근 pose와 nozzle spray yaw/alignment의 물리적
불일치다. 검증된 stand-off/yaw recovery는 아직 없다.

운영 중 확인한 사항:

- 점화 중 operator가 person으로 수용되지 않도록 VLA를 끈 채 점화하고 10초 후
  화각/주행 경로 이탈을 확인한 뒤 VLA와 Navigation Bridge를 함께 fresh-start한다.
- VLA만 restart하면 `action_0001`이 기존 Navigation Bridge cache와 충돌한다.
  두 launch tree 동시 restart 후 실제 fire Nav2 goal과 motion을 확인했다.
- Production Qwen의 실제 mission decision은 수초~수십 초가 걸렸으며 테스트
  전용 지연으로 간주하지 않는다.

Final run rosbag:
`/ros2_ws/phoenix_vla/rosbags/issue89_final_fireonly3_20260824_2045`
(29.2 MiB, 9,388 messages; navigation goal/result 1/1, spray command/result 1/1).

## Hardware-free Qwen latency stabilization (2026-08-26)

실제 #89 log의 16.691초 timeout 뒤 동일 WorldModel retry가 즉시 실행됐다.
HTTP timeout은 server-side generation을 취소하지 않고, 기존 `ThreadingHTTPServer`는
동일 XPU model의 concurrent inference를 막지 않았다.

Software 변경:

- server inference single-flight: 동시 backend 실행 최대 1
- inference failure signature cache: 의미 있는 WorldModel 변화 전 동일 retry 0
- Qwen lifecycle timestamp logging과 HTTP error body 보존
- ACTIVE/PENDING_VERIFICATION fire는 기존 0.5 m association radius 안에서 stable ID 유지
- resolved fire와 0.5 m 밖 fire는 별도 ID
- Qwen context에서 중복 mission, timestamp, last action history 제외
- `max_new_tokens`: 128 → 64

Intel XPU HTTP fixture 50회 결과:

```text
scenarios: fire far / fire in range / person near fire / no target / stale target
repeats: 10 each
latency p50 / p95 / max: 1.125 / 1.510 / 2.332 sec
timeout / HTTP error / schema failure: 0 / 0 / 0
32 / 48 tokens: truncated JSON
64 tokens: strict schema PASS
HW commands: 0
```

## Issue #101 human-safety policy audit (2026-08-26)

Production Qwen policy는 유효한 map 위치의 미보고 person을 거리와 관계없이 먼저
`REPORT_PERSON`하고, 보고만을 위한 person 접근을 하지 않는다.

- report terminal success 후 `reported=true`를 반영하고 fire risk를 재판단
- 보고 완료 person은 Qwen target/context에서 제외하되 fire의 route-blocking 관계는 유지
- fire suppression range와 physical action validation contract 유지
- physical action 중 deterministic `WAIT`와 single-flight 유지
- Resolver/Validator/Dispatcher가 invalid/stale robot pose와 invalid target dispatch 차단
- suppression failure budget 2회 유지

Software 검증:

```text
human-safety Qwen fixture: person-only / person+fire / blocking-fire pre-report 30/30 PASS
post-report blocking fire 단독: NAVIGATE_TO 10/10 PASS
multiple-person fixture: unreported person REPORT 10/10 PASS
fire outside/inside range: NAVIGATE_TO / EXTINGUISH 각 10/10 PASS
fire_vla_core regression: 247 PASS
latency before p50/p95/max: 1.564/1.658/2.413 sec
latency after  p50/p95/max: 0.913/0.951/0.978 sec
timeout / HTTP 503 / schema failure: 0 / 0 / 0
representative HTTP context: 739 -> 746 bytes
prompt template tokens: 385 -> 451
HW commands: 0
```

남은 GAP:

- `blocks_route_to`는 수동/observation relation이며 VLA가 route geometry로 계산하지 않음
- multiple-fire human-risk ranking은 일관되지 않음; first fire 선택 10/10
- no-target Qwen은 존재하지 않는 zone 이름을 선택함; Resolver가 physical dispatch 차단

## Issue #101 stored entity freshness 정정 (2026-08-26)

- `observation_max_age_sec=1.0`은 inbound observation stale gate로만 사용
- 정상 저장된 person/fire에는 별도 decision TTL을 적용하지 않음
- robot pose freshness, fire state/range/spray count, target 존재 검증 유지
- `PENDING_VERIFICATION`과 physical action single-flight 계약 유지
- entity freshness 정책은 실제 HW E2E 데이터 확보 후 재검토

## Depth sampling selective integration (2026-08-26)

- upstream source: `albitro/image_fire@ed3abbc301b0d49b52300cbc0e01fcb06eeef2e6`
- fire: `below`, band offset/ratio `3.5/3.0`, `p25`
- person: `bottom`, `median`
- production topic/schema: `/yolo_result → /fire/detections` 유지
- Hailo backend/model과 VLA/Qwen policy 변경 없음
- software: image_pipeline 351 PASS, fire_vla_core 247 PASS
- actual Robot HW: PENDING

## Fire-only Hardware E2E checkpoint (2026-08-26)

`ab0d02e` production stack에서 Qwen, Camera, production HEF YOLO, Depth,
LiDAR, TF, Nav2, suppression, VLA 개별 readiness를 확인했다. Fresh fire를
연속 검출했고 class별 Depth sampling이 적용된 3D detection 발행도 확인했다.

첫 Mission은 일반 문자열 `화재를 찾아 진압해줘`를 발행해 JSON Mission 계약을
위반했다. 올바른 payload는 다음과 같다.

```json
{"mission_id":"mission_fire_001","text":"화재를 찾아 진압해줘"}
```

결과:

- WorldModel: `mission: null`
- Nav2 goal: 0
- suppression: 0
- 실제 이동/분사: 0/0
- FIRST_FAILURE_STAGE: Mission transport

다음 fire-only E2E는 rosbag 없이 정상 JSON Mission을 정확히 1회 발행한다.
전체 E2E와 실제 소화는 아직 `PENDING`이며 Issue #89/#91을 유지한다.

## 2026-08-27 fire-only Hardware E2E 실행환경과 중단 상태

기준은 `integration/vla-robot-e2e@0af75f9627170f2eee3218110225c78762cb0f29`다.
Camera package는 `/home/ubuntu/ros2_ws/install/peripherals`, ASCamera SDK는
`v1.2.22.20240516`이며 `IntelPi` container의 `ros:humble-export` image와
`MentorPi_Mecanum`을 사용했다. Qwen endpoint는 당시 PC Wi-Fi 주소인
`http://192.168.1.37:8088/infer`였다. 다음 세션에는 실제 PC 주소를 다시 확인한다.

검증된 source와 environment 순서:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/third_party_ros2/third_party_ws/install/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
source /ros2_ws/phoenix_vla/install/setup.bash
export MACHINE_TYPE=MentorPi_Mecanum
export need_compile=True
export DEPTH_CAMERA_TYPE=ascamera
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH=/usr/local/lib/python3.10/dist-packages:${PYTHONPATH}:/home/ubuntu/.local/lib/python3.10/site-packages
cd /
```

실제 RGB/Depth/CameraInfo PASS를 만든 startup transcript는 Camera를 먼저 시작하고
8초 기다린 뒤 나머지 tree를 각각 정확히 1회 시작했다. 각 명령은 위 environment를 source한 별도 `docker exec -u root -w / IntelPi bash -lc` process에서 실행했다.

```bash
ros2 launch peripherals depth_camera.launch.py
# Camera initialization wait: 8 sec
ros2 launch uncc_example uncc_frontier.launch.py start_frontier:=false start_mission:=false start_vision:=false
ros2 run image_pipeline preprocess_node --ros-args -r __node:=rgb_preprocess_node -p input_topic:=/ascamera/camera_publisher/rgb0/image -p camera_info_topic:=/ascamera/camera_publisher/rgb0/camera_info -p output_topic:=/image_enhanced -p output_camera_info_topic:=/image_enhanced/camera_info -p mode:=passthrough
ros2 launch image_pipeline yolo.launch.py model_path:=/ros2_ws/phoenix_vla/Hailo/models/baseline_yolo26_neural_norm.hef postprocess_path:=/ros2_ws/phoenix_vla/Hailo/models/best_sim_postprocess.onnx backend:=hailo layout:=end2end class_names:="[fire,person]"
ros2 launch image_pipeline detection_3d.launch.py
ros2 launch fire_vla_bringup topic_bridge_vla.launch.py start_perception_bridge:=true llm_backend:=remote_qwen remote_qwen_endpoint:=http://192.168.1.37:8088/infer remote_qwen_timeout_sec:=10.0
ros2 launch uncc_example vla_navigation_bridge.launch.py
ros2 launch uncc_example fire_extinguisher.launch.py
```

오늘 확인한 PASS:

- compatible-QoS direct subscriber: RGB / Depth / CameraInfo PASS
- Qwen HTTP health: 200
- Nav2 `/navigate_to_pose` action server: PASS
- `/suppress_fire` action server: PASS
- YOLO fire detection: 지속 발생
- Detection3D depth: PASS, `depth_status=fallback_below`

오전 표적 배치 성공값은 confidence `0.729`, depth `0.634 m`, map position 약
`(0.470, -0.514)`, WorldModel `fire_0023 ACTIVE`였다.
`FRESH_FIRE_OBSERVATION: PASS`, `READY_FOR_MISSION: YES`까지 확인했다.

오후 최종 시도에서는 fire detection과 유효 depth가 지속됐지만 최고 confidence가
`0.5817`로 production threshold `0.60` 미만이었다. Depth는 약 `1.53 m`, 상태는
`fallback_below`였고 WorldModel ACTIVE fire는 등록되지 않았다. Mission, Nav2,
Robot motion, Servo, Pump command는 모두 0이다. 첫 실제 실패 단계는
`YOLO confidence threshold`다.

다음 표적은 threshold를 바꾸기 전에 화면 중앙 약 `0.9 m`에 둔다. 불꽃 아래
불연성 고체 받침과 거리 측정 가능한 비반사·불투명 배경을 확보하고 불 ON 시간은
최소화한다. 이 시점에는 `0.60 → 0.40` threshold 변경이
`PROPOSED / NOT_APPLIED`였고, 이후 사용자 승인으로 `5717087`에서 적용했다.
지속 검출 횟수+유효 Depth 결합 정책은 계속 `NOT_APPLIED`다.

## 2026-08-27 fire geometry / Nav2 분리 시험

승인된 최소 변경으로 production fire confidence threshold `0.60 → 0.40`을
`WorldModelConfig`와 `fire_vla_bringup/config/vla.yaml`에 적용했고, 관련 VLA package만
build한 뒤 VLA bridge/Orchestrator만 재시작했다. Camera, YOLO, Detection3D, Nav2,
Suppression runtime은 유지했다.

직전 Mission/WorldModel을 폐기한 뒤 Test 1에서 fresh `fire_0003 ACTIVE`를 만들었다.
Robot map pose는 `(0.141, -0.268, yaw 110.6°)`, fire map position은
`(-0.511, 1.153)`, robot 기준 bearing은 `+4.0°`였다. 실제 불도 카메라 정면에 있어
Detection3D/source-time TF의 fire 방향은 `PASS`다. Confidence는 `0.418`이었다.

불 OFF 후 같은 좌표로 Test 2를 시작했으나 이동 직전 robot yaw가 `64.0°`로 바뀌었다.
Robot은 수동·자율 이동하지 않았으므로 Test 1 대비 약 `46.6°`의 localization/TF
heading 변화다. Local costmap 정면 0.12~0.60 m 구간의 lethal cell은 `0/25`였지만
`ComputePathToPose`가 path를 생성하지 못했다. `NavigateToPose` goal은 발행하지
않았고 Robot motion, Mission, Servo, Pump command는 0이다.

첫 실패 계층은 `localization/TF heading stability`다. Fire map 좌표 방향 오류는
이번 Test 1로 배제됐다. Nav2 planner/costmap root cause는 아직 확정하지 않는다.
Test 3 Full E2E는 실행하지 않았고 terminal SUCCESS는 `NO`다.

## 2026-08-27 charging checkpoint

최종 source checkpoint는 `integration/vla-robot-e2e`의 threshold commit
`5717087`과 이어지는 문서 commit이다. 실제 Hardware runtime은
`/ros2_ws/phoenix_vla/install` overlay를 사용했다. Pi는 `IntelPi` container,
`ros:humble-export`, `MentorPi_Mecanum`이며 Camera package prefix는
`/home/ubuntu/ros2_ws/install/peripherals`다. ASCamera SDK는
`v1.2.22.20240516`이다.

Production 환경은 `MACHINE_TYPE=MentorPi_Mecanum`, `need_compile=True`,
`DEPTH_CAMERA_TYPE=ascamera`, `ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=0`,
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`다. `PYTHONPATH`는
`/usr/local/lib/python3.10/dist-packages:${PYTHONPATH}:/home/ubuntu/.local/lib/python3.10/site-packages`다.
Qwen endpoint는 고정 주소가 아니라 현재 PC Wi-Fi IP의 `http://<PC_IP>:8088/infer`로
설정하고 Pi에서 HTTP 200을 확인한다. Overlay와 canonical launch 원문은 바로 위
실행환경 section이 authoritative하다.

확인된 Happy Path는 RGB/Depth/CameraInfo, 지속 YOLO fire, Detection3D
`fallback_below` depth, fresh ACTIVE fire, Qwen HTTP 200, Mission 1회 발행,
VLA physical action 시작, Nav2/Suppression action server까지다. 미완료 항목은 Nav2
terminal result, system-level Robot stop, Servo/Pump suppression, 실제 화염 제거,
terminal SUCCESS다.

현재 blocker는 전원과 localization이다. PC–Pi 단절 당시 Pi 전원이 실제로 꺼져
있었다. 배터리 부족 또는 Motor 부하 중 voltage drop이 직접 원인 후보이며 미확정이다.
Phoenix low-battery contract는 Pi poweroff가 아니라 `RETURNING_TO_BASE`다. 또한
정지 상태에서 localization yaw가 `110.6° → 64.0°`로 변했다. 수동 이동 뒤 기존
localization, fire, Mission, goal은 모두 폐기한다. 현재 Robot과 Pi는 충전을 위해
종료됐고 Hardware E2E는 중단 상태다.

## 2026-08-27 final fire-only checkpoint

기준은 `integration/vla-robot-e2e@2b5ee9a12147754004d293e0d9f7d8799e69f951`다.
Qwen `REPORT_PERSON` 판단은 약 `2.36 s`에 성공했다. Production launch에는
`remote_qwen_timeout_sec:=10.0`을 명시해야 한다. `REPORT_PERSON` terminal result가
WorldModel에 반영되지 않은 현상은 별도 known issue이며 이번 fire-only 실행에서
수정하지 않았다.

점화 중 사람 관측을 피하기 위해 VLA Orchestrator와 Navigation Bridge를 끄고,
작업자가 화각에서 빠진 뒤 두 component를 시작했다. 새 WorldModel의 `people=0`을
확인한 이 방식으로 fresh `fire_0001 ACTIVE`를 생성했다. Confidence는 `0.582`, map
position은 약 `(1.066, -0.854)`이며 Depth와 map position은 유효했다.

Qwen이 분사거리 밖 ACTIVE fire에 `EXTINGUISH`를 반환해 Validator가 거절하던 경로는
`2b5ee9a`에서 같은 target의 `NAVIGATE_TO`로 제출 전 정확히 1회 교정한다. 역방향
교정은 없고 stale/invalid/inactive target 차단, spray range `0.8 m`, 나머지 safety
threshold는 유지한다. 검증은 focused `4 PASS`, Orchestrator `22 PASS`다.

마지막 canonical runtime은 Camera 선기동과 8초 대기 후 나머지 stack을 정확히 1회
시작했다. RGB/Depth/CameraInfo/YOLO는 PASS했고 Detection3D counter는
`frames=43`, `detections_in=94`, `depth_in=349`였다. Nav2와 Suppression action
server는 각각 1개였고 Mission/Nav2/Servo/Pump command는 0이었다. 이후 PC–Pi
경로가 끊겨 WorldModel empty와 Pi→Qwen HTTP 200 최종 확인은 `UNKNOWN`이다.
`READY_FOR_FIRE=NO`로 종료했다.

SSH는 정상 접속 뒤 reset, timeout, `No route to host`가 간헐적으로 발생했다. Ping은
약 `85–125 ms`, packet loss `33%`였으나 이 정보만으로 production failure를
확정하지 않는다. 세션 종료 시 SSH가 복구되지 않아 runtime clean stop, Robot stop,
Pump OFF의 원격 확인 상태도 `UNKNOWN`이다.

## Latest VLA software checkpoint (2026-08-29)

`integration/vla-robot-e2e@0a3af10882d29bfcc51aac34905fe1d84703b6ee`에서
#104~#108, #109, #111 범위가 완료됐다.

- Mission 시작 시 이전 person/fire observation과 association을 초기화한다. Robot
  pose, map, home pose는 유지한다.
- Mission 중 새 person은 기존 report topic/schema로 자동 보고되고 Mission당 한 번만
  `reported=true`가 된다. UI ACK 지연은 VLA 판단을 차단하지 않는다.
- `person_fire_risk_distance_m=0.10`은 demo-scale 관계값이다. 범위 안 ACTIVE fire는
  가장 가까운 person의 `threatens_person=true`와 `threatened_person_id`를 노출한다.
- 유효한 분사거리 밖 ACTIVE fire에 대한 Qwen `EXTINGUISH`는 같은 target의
  `NAVIGATE_TO`로 한 번 교정된다. #109 test expectation은 navigation 1건, spray
  0건으로 현재 정책에 맞춰졌다.
- Qwen이 선택한 fire의 Nav2가 성공하고 fire/pose가 유효·fresh하며 설정된
  `spray_range_m` 이내이고
  의미 있는 WorldModel 변화가 없을 때 같은 fire의 `EXTINGUISH`를 deterministic하게
  한 번 연결한다. 조건이 바뀌면 자동 분사하지 않고 Qwen이 재판단한다.
- `scripts/vla_hardware_e2e.sh`가 canonical runtime의 `start`, `status`, `mission`,
  `stop`을 제공한다. 여러 component terminal 대신 이 wrapper를 사용한다.

Mission scope는 첫 유효 Qwen 구조화 응답에서 action/target과 함께 한 번 확정한다.
정상 경로의 Qwen 호출은 1회이며 Mission 도중 scope 변경은 거절한다.

- `FIRE_ONLY`: 선택한 fire가 `EXTINGUISHED` 또는 `INACCESSIBLE`, action 없음,
  perception ready이면 완료한다. 관계없는 entity와 exploration은 blocker가 아니다.
- `PERSON_FIRE`: 대상 person reported, 해당 person을 위협하는 대상 fire 처리 완료,
  action 없음, perception ready이면 완료한다.
- `FULL_EXPLORATION`: exploration completed, 모든 person reported, 모든 fire 처리 완료,
  action 없음, perception ready인 기존 전역 계약을 유지한다.

소화 성공은 즉시 `EXTINGUISHED`로 바꾸지 않는다. `PENDING_VERIFICATION`에서 delay
`0.5 s` 이후 유효한 fire 미검출 3회를 요구하며, `5 s` timeout이면 `ACTIVE`로
복귀한다. Invalid/future/stale frame은 소화 증거에서 제외한다. Validator, freshness,
spray range, Robot stop, spray-count와 중복 action 차단도 유지된다.

Firefighter UI 포함 SW-only E2E 결과:

```text
Mission input/boundary: PASS
person auto report + UI reported display: PASS
person-threat fire + threatened person ID UI display: PASS
Qwen calls: 1
Mock Nav2: SUCCESS (dispatch 1)
deterministic EXTINGUISH: 1
Mock suppression: SUCCESS
fire verification: EXTINGUISHED
Mission/UI terminal: COMPLETED
duplicate Mission/Nav2/report/suppression: 0
focused: 120 PASS
full fire_vla_core regression: 273 PASS
new failures: 0
Hardware commands: 0
```

### Latest team-branch comparison (2026-08-29)

현재 merge 대상은 없다. `albitro/image_opt@bfafdad`와 `6ccf370`의 Hailo 입력 및
thread 최적화는 현재 integration에 없지만 production HW E2E 증거가 없어 Pi에서
동일 출력과 latency/CPU를 측정하기 전까지 보류한다. Nav2/SLAM parameter 변경은
실제 관련 오류가 재현될 때만 검토한다. `fire_service_v2@fc060a5`는 Frontier 기반
별도 fire-status/suppression owner를 도입해 현재 VLA suppression lifecycle과 이중
owner 위험이 있으므로 통합하지 않는다. 팀 branch 전체 merge는 금지하고 실제 증거가
있는 commit만 계약 단위로 선별한다.

## 2026-08-29 latest Hardware checkpoint

Fire floor ROI를 실제 Hardware에서 다시 측정했다. 카메라 렌즈와 촛불 받침의 실측
직선거리는 `0.45~0.46 m`였고, 세 관측의 선택 Depth는 `0.456~0.457 m`였다. 최대
절대 오차는 약 `0.007 m`이며 세 건 모두 `fallback_below`였다. Production 설정
`band_offset=3.5`, `band_ratio=3.0`, `p25`를 그대로 사용했다. 과거 `1.45 m`
과대 측정은 현재 배치에서 재현되지 않았으므로 floor ROI production 변경은 필요하지
않다.

Person+fire pre-spray 실행은 Mission을 한 번 발행한 뒤 같은 정지 person이
`person_0001`, `person_0002`처럼 반복 생성·보고되어 fire/Qwen/Nav2/spray 전에
중단했다. ID 없는 관측 간격 `2~4 s`가 association TTL `2.0 s`를 넘었지만 map 위치
변화는 최대 약 `0.0038 m`로 merge radius `0.50 m` 안이었다. 같은 Mission에서 이미
reported인 person은 TTL 이후에도 동일 class와 radius 조건으로 기존 ID에 연결한다.
Mission boundary 초기화, unreported person TTL, fire association lifecycle과 radius는
변경하지 않는다.

Suppression은 현재 `DISABLED`다. 현장 관찰은 Servo가 physical pin 7 (`BCM4`),
Pump가 physical pin 8 (`BCM14`)에 연결된 것으로 보였지만 핀과 극성은 Hardware
team이 아직 확정하지 않았다. Suppression node startup만으로 Pump가 작동한 incident가
있어 `fire_suppression_node`, `fire_extinguisher`, `vla_spray_bridge`, 전체
Hardware wrapper start와 GPIO 접근을 금지한다. 실패한 `active_high=True` 실험
변경은 production commit에 포함하지 않았다.

## 2026-09-02 Hardware E2E checkpoint

위 `DISABLED` 기록은 Hardware team의 pin 확정 전 historical 상태다. 현재 production
계약은 Pump `BCM14`, Servo `BCM13`, Pump `active_high=False`,
`initial_value=0.0`이며 불 OFF suppression startup 5초 동안 실제 Pump/Servo 무동작을
확인했다.

최신 기준은 `integration/vla-robot-e2e@414abc6c57f61f6bdf44b5212f512d3937afc811`다.
관련 반영은 실행별 고유 log directory `03a8edb`, wrapper control owner `4a2bb4c`,
Nav2 goal당 최소 결과 로그 `4f2e2f1`, Nav2 readiness의 `bt_navigator` ACTIVE 조건
`0fe496a`, `spray_range_m=0.25 m`와 fire stand-off `0.20 m`의 `9801afa`, 실제
Nav2 XY goal tolerance `0.05 m`의 `414abc6`이다. Goal tolerance가 과거 `0.25 m`일
때 최악 fire 거리는 `0.45 m`였지만 현재 최악값은 `0.25 m`다.

Floor ROI는 넓은 무광 비가연성 받침을 사용한 40~45 cm 배치에서 valid Depth 약
`0.471 m`를 냈다. Valid pixel 0의 `depth_status=unknown`에는 Mission을 발행하지
않는다. 실제 E2E에서 stand-off goal geometry 자체는 정상이고 Nav2 goal도 최종
`SUCCEEDED`했으나, 시작 약 1.2초 사이 map pose가 약 `0.434 m`, yaw가 약 `19.4°`
급변했다. Controller의 반복 `Failed to make progress`와 recovery spin의
`Collision Ahead`가 뒤따랐다. 이는 바닥 배치 뒤 runtime/localization을 다시 만들지
않고 책상-start 상태를 이어 사용한 절차 오류와 일치한다.

수정된 바닥-start 절차로 실제 화염 제거와 Mission `COMPLETED`는 아직 확인하지
못했다. VLA Mission 단독 STOP API는 없고 통합 status observer timeout 원인도
미확정이다. 현재 authoritative 실행 순서는
`docs/VLA_HARDWARE_E2E_HAPPY_PATH.md`만 따른다.

## 2026-09-03 최초 물리 소화 성공 checkpoint (historical)

Authoritative 기준은
`integration/vla-robot-e2e@76251b3f16ceffc6f680b628a6a6f6ce399e2d8f`다.
`navigation_standoff_m=0.15 m`, `spray_range_m=0.30 m`, Nav2 XY goal tolerance
`0.05 m`로 실행했다. Nav2는 약 `4.28 s`에 `SUCCEEDED`했고 도착 후 WorldModel
base→fire 거리는 약 `0.198 m`였다. Robot stop은 `PASS`, suppression은 정확히 1회,
Servo/Pump 실제 작동은 `PASS`였다. 물줄기는 불꽃보다 살짝 뒤에 착탄했지만 실제
화염 제거는 `PASS`다.

당시 Physical suppression과 software terminal을 분리해 판정했다. Fire verification은
`FAIL`, fire final state는 `ACTIVE`, Mission terminal은 `RUNNING`이었다. 당시 blocker는
`fire_status_service`가 `관찰 구간 내 YOLO 감지 기록 없음`을 성공으로 처리하지 못한
verification 경계였다. 이는 후속 `a8859f4`에서 유효 empty detection frame을
WorldModel의 기존 3-frame verification owner로 전달하도록 해결했으며 현재 blocker가
아니다.
