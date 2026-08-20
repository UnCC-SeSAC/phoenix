# VLA Robot E2E Current Status

## Current HEAD

- Branch: `integration/vla-robot-e2e`
- Issue A integration baseline: `98e1f65ac8b39fd43d3d5f204eaa751f8ec21e77`
- This document is the integration checkpoint after the verified VLA, Hardware,
  Remote Qwen, duplicate-goal, Hailo-backend, and perception-downstream work.
- Model binaries and local runtime artifacts are not tracked.

## Architecture

Robot state and control remain Pi-local:

```text
Pi: Hardware → SLAM → Nav2 → VLA Navigation Bridge
    → VLA Orchestrator → WorldModel → Resolver → Validator → Dispatcher

Pi RemoteQwenBackend ↔ HTTP JSON ↔ PC Qwen/Qwen3-1.7B inference only
```

The critical topics `/vla/robot_pose_json`, `/vla/navigation_goal`, and
`/vla/navigation_result` are Pi-local. PC↔Pi ROS 2/Fast DDS user-data continuity
repeatedly failed while Pi-local pose, TF, and Bridge paths were valid, so
cross-machine DDS was removed from the Robot control critical path. The PC receives
only compact semantic WorldModel/Mission data and returns the existing strict
`ActionDecision` JSON contract.

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
FAILED, ABORTED, CANCELED, and TIMED_OUT retain the existing retry semantics. A new
Mission may use the same target, and a different action such as `REPORT_PERSON` remains
allowed. Hardware revalidation produced one navigation goal, one NavigateToPose, zero
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

Live Hardware perception has reached Camera and preprocessing; ONNX YOLO and downstream live stages remain unverified:

```text
actual person → ASCAMERA RGB → YOLO → actual depth → /fire/detections
→ actual CameraInfo/source-time TF → map (x,y) → person_0001 → WorldModel
```

### Live perception Hardware validation — 2026-08-19

After moving NOVATEK 3482:6723 to another Pi USB port, both Video interfaces enumerated with uvcvideo at 480 Mbps. A clean launch with exactly one ascamera_node produced actual Hardware data.

- Camera stream start: PASS
- RGB Image and CameraInfo: PASS, frame ascamera_color_0
- Depth Image and CameraInfo: PASS, frame ascamera_color_0
- preprocess passthrough input: 14.7–14.9 Hz
- /image_enhanced actual Image: PASS
- LIVE_CAMERA_PASS = YES

IntelPi used system NumPy 1.26.4, OpenCV 4.5.4, ONNX Runtime 1.23.2, and a working CvBridge. UID 1000 user-site NumPy 2.2.6 was incompatible with the Humble CvBridge binary. No package was installed or removed; a process-local PYTHONPATH selected system NumPy/OpenCV first while retaining user-site ONNX Runtime.

The installed /ros2_ws/phoenix/install/image_pipeline was older and routed explicit backend:=onnxruntime into OpenCV DNN. The verified source at /ros2_ws/phoenix_perception/src/image_pipeline contained OnnxRuntimeBackend; only image_pipeline was built in that isolated workspace. The team workspace was unchanged.

The live attempt used /ros2_ws/models/best_base.onnx, backend onnxruntime, layout end2end, and class order fire=0/person=1. Immediately after startup, Pi SSH port 22 returned connection refused while ping still responded. The process state, inference output, and final log could not be recovered safely.

- LIVE_ONNX_YOLO_PASS = NO
- LIVE_DEPTH_FUSION_PASS = NOT_RUN
- LIVE_MAP_LOCALIZATION_PASS = NOT_RUN
- LIVE_PERCEPTION_WORLDMODEL_PASS = NOT_RUN
- FIRST_FAILURE_STAGE = G. ONNX YOLO runtime

LIVE_ONNX_YOLO_PASS = NO means the live inference result was not verifiable because runtime access was lost. It does not establish a model, decoder, or ONNX Runtime inference failure. OOM/resource exhaustion, sshd failure, and CPU or memory pressure remain unconfirmed hypotheses only. Mission, Qwen, Nav2, cmd_vel, Motor, Pump, and Servo were not run.

Next session:

1. Check kernel/OOM logs around the failure time.
2. Check ssh.service and its journal.
3. Record CPU and RAM before and during inference.
4. Record docker stats for IntelPi.
5. Measure ONNX Runtime thread/resource usage.
6. Revalidate incrementally: YOLO alone, Camera plus YOLO, then the full perception chain.

## Hailo Status

`HailoBackend` implements HailoRT `VDevice`/`InferVStreams` lifecycle, explicit sorted
stream-name extraction, NCHW RGB float-to-NHWC RGB UINT8 conversion, and conversion of
`HAILO_NMS_BY_CLASS` output into the existing end-to-end detection contract.
`backend:=hailo` is available through `yolo.launch.py`.

Production candidate:

- file: `best_filtered_hailo10h.hef`
- SHA-256: `3f141f4604e4eec9c45c49fa17455fda29b78b3e5df2c550e9ee89d64d29063f`
- class contract: `0=fire`, `1=person`
- input: `yolov26s/input_layer1`, `(640,640,3)`, UINT8 NHWC
- output: `yolov26s/yolov8_nms_postprocess`, `HAILO_NMS_BY_CLASS`, embedded NMS

Live HEF inference is `NOT_RUN` because Hailo device/runtime installation is not yet
complete. The HEF binary is intentionally not committed.

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
silent fallback. Live Camera and preprocess testing now pass; live ONNX YOLO output remains unverified because Pi SSH access was lost after startup.

## Current Pending

- live ONNX YOLO detection (Camera and preprocess already PASS)
- live RGB/depth synchronization and depth fusion
- live `/fire/detections`
- actual CameraInfo/source-time TF map localization
- actual person/fire entity creation in WorldModel
- Hailo HEF live inference
- full perception → Qwen → Nav2 Hardware E2E
- full fire-suppression Pump/Servo Hardware E2E

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

## Next Milestone

Before Hailo installation completes, use the team-owned
`albitro/phoenix_detection` PT or ONNX model to verify the same production live path:

```text
actual Camera → YOLO → Depth → /fire/detections
→ camera-to-map conversion → WorldModel person/fire
```

No Mission or Robot motion is required until live perception localization is confirmed.

## Do Not Repeat

- Do not restart cross-machine DDS multicast/unicast diagnosis.
- Do not restart Discovery Server diagnosis.
- Do not relax or bypass Validator freshness.
- Do not reselect the Qwen model without a new requirement.
- Do not reinvestigate the duplicate-goal bug already guarded and Hardware-verified.
- Do not reimplement the already-passed short-navigation path.
- Do not directly inject final map coordinates when validating live perception.
- Do not commit HEF/PT/ONNX binaries or user runtime artifacts.

## Checkpoint Validation

- image_pipeline pytest: 347 PASS
- focused fire_vla_core/perception/Remote-Qwen regression: 84 PASS
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
- HailoBackend implementation

PENDING:

- live ONNX YOLO/depth/map/WorldModel perception
- Hailo HEF live inference
- full perception-to-navigation Hardware E2E
- fire suppression full Hardware E2E
