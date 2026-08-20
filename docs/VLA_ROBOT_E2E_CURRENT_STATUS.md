# VLA Robot E2E Current Status

## Current HEAD

- Branch: `integration/vla-robot-e2e`
- Current verified code checkpoint: `e28491dacd8a591f0aaee6df4ac1640460eaa82f`
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

Live Hardware perception checkpoint for Issue #88:

```text
actual ASCAMERA RGB → preprocess → split HEF neural inference
→ companion ONNX postprocess → person bbox/confidence
```

- Camera stream and passthrough preprocessing: PASS
- Offline split-HEF detection: PASS
- Live HEF person detection from actual ASCAMERA frame: PASS
- Actual person confidence: `0.3979718685`
- Actual person bbox `(x1,y1,x2,y2)`: `(347.33,157.15,472.93,315.16)`
- `OFFLINE_HEF_DETECTION_PASS = YES`
- `LIVE_HEF_PERSON_DETECTION_PASS = YES`

The earlier live `detections: []` failure had two confirmed causes. First, the
repository assumed a single `HAILO_NMS_BY_CLASS` output, while the deployed HEF emits
six raw neural heads that require the hardware-team companion ONNX postprocess.
Second, the ROS shell resolved a stale installed backend ahead of the isolated fixed
backend. Commit `e28491dacd8a591f0aaee6df4ac1640460eaa82f` adds the measured split-HEF contract
while retaining the existing single-output NMS HEF path.

Issue #88 remains **OPEN**. Detection itself is no longer the blocker. The remaining
stationary Hardware validation is:

```text
person bbox → actual depth fusion → camera/Robot coordinates
→ source-time TF → map (x,y) → SemanticObservation
→ WorldModel → Firefighter UI/report
```

These remaining stages are `NOT_RUN`. They do not require Robot motion, but valid
localization and camera-to-map TF must be available. Nav2 goals, `cmd_vel`, Motor,
Pump, and Servo were not run during the live HEF person-detection validation.

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

## Current Pending

- Issue #88: live person depth fusion and `/fire/detections`
- Issue #88: actual CameraInfo/source-time TF map localization
- Issue #88: actual `person_0001` creation in WorldModel and UI/report
- actual fire detection/depth/map localization
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

## Next Milestone

Continue Issue #88 from the first unverified stage, without repeating Camera or HEF
person detection:

```text
verified live person bbox → actual depth → /fire/detections
→ CameraInfo/source-time TF → map (x,y)
→ person_0001 → WorldModel → Firefighter UI/report
```

Keep the Robot stationary. No Mission, Qwen action dispatch, Nav2 goal, `cmd_vel`,
Motor, Pump, or Servo command is needed for this checkpoint.

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

- image_pipeline pytest: 351 PASS
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
- HailoBackend split-HEF implementation
- offline HEF fire/person detection
- live ASCAMERA HEF person bbox/confidence detection

PENDING:

- Issue #88 live person depth/map/WorldModel/UI report
- full perception-to-navigation Hardware E2E
- fire suppression full Hardware E2E
