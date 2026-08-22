# VLA Robot E2E Current Status

## Current HEAD

- Branch: `integration/vla-robot-e2e`
- Source checkpoint before this documentation update:
  `eaa26d4156f70e3b48d7759bab1c838ca27c89ac`
- Issue A integration baseline: `98e1f65ac8b39fd43d3d5f204eaa751f8ec21e77`
- This document is the integration checkpoint after the verified VLA, Hardware,
  Remote Qwen, duplicate-goal, Hailo-backend, and perception-downstream work.
- Model binaries and local runtime artifacts are not tracked.

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
- container: `IntelPi` (`ubuntu` user, ROS 2 Humble)
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
- VLA: `ros2 launch fire_vla_bringup topic_bridge_vla.launch.py start_perception_bridge:=true llm_backend:=remote_qwen remote_qwen_endpoint:=http://<CURRENT_PC_IP>:8088/infer` plus `ros2 launch uncc_example vla_navigation_bridge.launch.py`. The last successful stationary suppression test used `192.168.100.124:8088`; confirm the current PC address instead of treating it as a fixed endpoint.
- suppression bridge/action server: `ros2 launch uncc_example fire_extinguisher.launch.py`; starting it does not actuate Hardware, but an actual suppress goal still requires explicit operator approval
- production thresholds: fire confidence `>= 0.60`, spray range `<= 0.80 m`

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
exactly one `/suppress_fire` action server. Launching centers the Servo at startup;
never send a suppression goal without explicit Hardware authorization.

`AngularServo(initial_angle=90)` can cause a brief startup center alignment. Persistent
Servo buzzing is different: possible causes include failure to reach the target angle,
a mechanical stop, nozzle/hose load, or center/PWM calibration mismatch. If stopping
the suppression node immediately stops the sound, do not classify it as a Pump command.

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
