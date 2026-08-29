# Issue #89 Next Session Handoff

Date: 2026-08-22 KST
Latest software update: 2026-08-29 KST

이 문서는 새 Codex session이 workspace 복원이나 기존 PASS 항목 재진단 없이 Final
Full E2E를 바로 이어가기 위한 authoritative handoff다. 먼저
`docs/VLA_ROBOT_E2E_CURRENT_STATUS.md`와
`docs/VLA_ROBOT_RUNTIME_TROUBLESHOOTING.md`를 읽고 이 절차를 따른다.

## Git and isolated deployment checkpoint

- branch: `integration/vla-robot-e2e`
- current software checkpoint before this documentation update:
  `0a3af10882d29bfcc51aac34905fe1d84703b6ee`
- historical runtime/model checkpoint (2026-08-22):
  `fc44b2b4cdabce3db4f4675616f479b6b1e068d8`
- session 시작 시 local HEAD와 `origin/integration/vla-robot-e2e`가 일치하는지만
  확인한다. SHA가 docs-only 후속 commit이면 rebuild하지 않는다.
- team/legacy workspace: `/ros2_ws/phoenix`
- VLA production workspace: `/ros2_ws/phoenix_vla`
- VLA production overlay: `/ros2_ws/phoenix_vla/install`
- `/ros2_ws/phoenix`는 팀원 환경이며 VLA production overlay로 사용하지 않는다.
- `/ros2_ws/phoenix_vla`는 `integration/vla-robot-e2e` 전용이다. 두 workspace 사이에서
  branch를 전환하거나 `build/`, `install/`, `log/`를 공유하지 않는다.
- build user: normal user `ubuntu`; `sudo colcon build` 금지.
- suppression runtime의 root + `/` 실행 경계는 build user 계약과 별개다.

Fresh isolated workspace verification:

```text
image_pipeline: PASS
fire_vla_core: PASS
uncc_example: PASS
fire_vla_bringup: PASS
vla_spray_bridge: PRESENT
production prefix: /ros2_ws/phoenix_vla/install
cross-workspace install: NONE
root-owned build/install/log artifacts: 0
```

새 clone이나 rebuild를 반복하지 않는다. 현재 isolated workspace와 build/install/log를
그대로 사용한다.

## Production model artifacts

두 파일은 Git-untracked production artifact이며 `ubuntu:ubuntu` 소유다. 기존 검증
artifact와 byte-identical PASS했다. Git에 추가하지 않는다.

```text
HEF:
/ros2_ws/phoenix_vla/Hailo/models/baseline_yolo26_neural_norm.hef
size: 11288576 bytes
SHA-256: 67496fe3eefb710bef56ce9fd30af0102520c234f697f715ed0935a881e75aad

Postprocess:
/ros2_ws/phoenix_vla/Hailo/models/best_sim_postprocess.onnx
size: 106676 bytes
SHA-256: b05022e4741258840e48143e7dc0f88cc676d11a842e6950623c59cf189f60b4
```

## Authoritative production and suppression contract

```text
Container: IntelPi
Suppression run user: root
Suppression working directory: /
MACHINE_TYPE=MentorPi_Mecanum
need_compile=True
DEPTH_CAMERA_TYPE=ascamera
ROS_DOMAIN_ID=42
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Setup order:

```text
/opt/ros/humble/setup.bash
→ /home/ubuntu/third_party_ros2/third_party_ws/install/setup.bash
→ /home/ubuntu/ros2_ws/install/setup.bash
→ /ros2_ws/phoenix_vla/install/setup.bash
```

Suppression Happy Path:

```bash
cd /
ros2 launch uncc_example fire_extinguisher.launch.py
```

Expected readiness:

```text
fire_suppression_node: PRESENT
vla_spray_bridge: PRESENT
/suppress_fire action server: 1
Pump active goal: 0
```

`AngularServo(initial_angle=90)`의 짧은 startup center alignment는 정상이다. 지속적인
buzzing은 mechanical/PWM load 문제일 수 있으며 Pump activation으로 해석하지 않는다.

## Ethernet to Wi-Fi transition

고정 준비 단계는 Ethernet `lemma@192.168.100.128`을 사용한다. 실제 motion 직전에
Ethernet을 제거한다. Fast DDS participant가 이전 Ethernet interface를 유지하면
process는 살아 있어도 VLA navigation client와 suppression server가 graph에서 이탈할
수 있고 ROS daemon refresh만으로는 부족하다.

Verified transition:

```text
Ethernet 제거
→ Wi-Fi connectivity와 현재 Qwen endpoint 확인
→ topic_bridge_vla.launch.py, vla_navigation_bridge.launch.py,
  fire_extinguisher.launch.py만 clean stop
→ Wi-Fi가 활성화된 상태에서 세 launch를 정확히 1회 재시작
→ ROS daemon refresh
→ NavigateToPose server/client = 1
→ /suppress_fire server/client = 1
```

이 transition 때문에 Base, Camera, YOLO, SLAM, Nav2를 재시작하지 않는다. Timeout 후
side-effect launch를 blind retry하지 않고 process 존재 여부를 한 번 확인한다.

## Qwen contract and recovery

Authoritative VLA arguments:

```text
llm_backend:=remote_qwen
remote_qwen_endpoint:=http://<CURRENT_PC_IP>:8088/infer
remote_qwen_timeout_sec:=10.0
```

Endpoint는 고정 IP로 재사용하지 않고 현재 PC network에서 확인한다. Known XPU failure:

```text
Qwen HTTP 503
UR_RESULT_ERROR_DEVICE_LOST
UR_RESULT_ERROR_UNKNOWN
invalid device pointer
XPUCachingAllocator
```

Verified recovery:

```text
Qwen 반복 restart 금지
→ PC clean reboot
→ XPU probe PASS
→ authoritative Qwen clean start exactly once
→ Qwen HTTP 200
```

## Already verified PASS

- LD19 `/scan_raw` and `map→odom` continuity
- Camera, HEF/YOLO, depth observation, and WorldModel acceptance
- Qwen/VLA `EXTINGUISH fire_0001`
- `/vla/spray_command → vla_spray_bridge → /suppress_fire`
- Servo execution, Pump execution, and Pump OFF
- stationary suppression Hardware E2E with Robot motion 0
- isolated VLA clone, focused build, package prefix, ownership, and model fingerprints

Stationary terminal `ABORTED` was caused by no water being loaded, not by a suppression
pipeline failure. Actual extinguish verification remains pending.

## Last actual Full E2E position

```text
Production runtime readiness: PASS
Ethernet → Wi-Fi runtime rebinding: PASS
Qwen HTTP: 200
NavigateToPose server: 1
VLA navigation client: 1
/suppress_fire server: 1
vla_spray_bridge client: 1
VLA/Perception: ACTIVE
Fire detection: YES
fresh confidence: 0.659
WorldModel: ACCEPTED
distance: about 2.14 m
robot_within_spray_range: false
```

Mission delivery reached Qwen inference, then Intel XPU/Qwen runtime failed before
physical dispatch:

```text
NAV2_GOAL_COUNT: 0
ROBOT_MOTION: 0
SUPPRESSION_REQUEST: 0
PUMP: 0
FIRST_REAL_FAILURE: Intel XPU / Qwen runtime stability
```

This is not a Nav2 or suppression failure. At end of day, all identified production
launch/process groups were stopped; no new goal or actuator command was sent during
shutdown. Physical fire-target OFF must still be confirmed by the operator.

## Exact next-session start

```text
1. Read this handoff, current-status, and troubleshooting docs.
2. Verify /ros2_ws/phoenix_vla branch/HEAD and the two artifact paths only.
3. PC XPU probe.
4. Authoritative Qwen clean start exactly once.
5. Require Qwen HTTP 200.
6. Authoritative production clean start using phoenix_vla overlay.
7. Minimum readiness only.
8. At PRE_MOTION_READY, remove Ethernet and place/clear the Robot safely.
9. Apply the verified Wi-Fi participant rebinding procedure.
10. Require minimum readiness, then request READY_FOR_FIRE_TARGET.
11. Continue the Final Full E2E once.
```

Do not start with source analysis, workspace recreation, rebuild, or completed preflight
diagnostics unless the SHA/artifacts changed or a direct failure points there.

## Final Full E2E success criteria

```text
FIRE_DETECTED: YES
WORLDMODEL_ACCEPTED: YES
VLA_DECISION: EXTINGUISH

NAV2_GOAL_COUNT: 1
NAV2_RESULT: SUCCEEDED
ROBOT_STOPPED: YES

SUPPRESSION_REQUEST: 1
SERVO: EXECUTED
PUMP: EXECUTED
PUMP_OFF: YES

FIRE_AFTER_SUPPRESSION: NOT_DETECTED
TERMINAL_RESULT: SUCCESS
```

Issue #89 remains OPEN until the actual Nav2 approach, safe stop, water suppression,
flame disappearance, and terminal SUCCESS are all observed.

## 2026-08-24 latest Hardware checkpoint

PC clean reboot 후 XPU probe, authoritative Qwen clean start, health/inference HTTP
200을 확인했다. Fresh fire-only run에서
Qwen→`NAVIGATE_TO fire_0006`→Nav2 실제 주행→`SUCCEEDED`→robot stop→
`EXTINGUISH`→Servo/Pump→Pump OFF까지 실행됐다. 물줄기가 화염을 빗나가 실제
소화는 실패했고 terminal result는 `ABORTED`다.

```text
Latest valid bag:
/ros2_ws/phoenix_vla/rosbags/issue89_final_fireonly3_20260824_2045
Bag size: 29.2 MiB
Messages: 9,388
navigation goal/result: 1 / 1
spray command/result: 1 / 1
cmd_vel: 182
```

다음 작업은 Hardware blind retry가 아니라 fire stand-off approach pose와 spray
yaw/nozzle alignment의 불일치를 해결하는 것이다. Nav2는 주어진 fire goal을 실제로
수행했으므로 일반 SLAM/Nav2 실패로 분류하지 않는다.

Operational recovery:

```text
VLA + Navigation Bridge clean-start
→ VLA OFF 상태에서 점화 후 operator exit 10초 대기
→ people=[] / fresh fire only 확인
→ new rosbag
→ new mission exactly once
```

VLA만 restart하면 `action_0001`이 기존 Navigation Bridge cache와 충돌해 과거
result가 새 fire action에 재사용될 수 있다. 두 launch tree를 함께 restart한 후
실제 fire Nav2 goal과 motion으로 recovery를 검증했다.

## 2026-08-26 Hardware-free latency checkpoint

`13f2282` 기준 실제 #89 latency 원인을 software fixture로 재현했다. Client timeout
후 server generation이 계속될 수 있고 timer-driven 동일-state retry가 중첩될 수 있었다.
Server single-flight와 meaningful-state-change retry gate를 적용했다.

Qwen3-1.7B XPU HTTP fixture 50회에서 64 tokens 기준 p50 1.125초, p95 1.510초,
max 2.332초, timeout/HTTP/schema failure 0을 확인했다. 32/48 tokens는 reason
문자열이 잘려 strict JSON contract에 실패했다.

Fire fallback ID는 ACTIVE/PENDING_VERIFICATION lifecycle 동안 기존 0.5 m radius를
유지한다. Observation max age 1.0초와 resolved-fire 분리는 변경하지 않았다.
Hardware command는 실행하지 않았다.

## 2026-08-26 fire-only Mission transport checkpoint

`ab0d02e` production runtime에서 Qwen, Camera, HEF YOLO, Depth, LiDAR, TF,
Nav2, suppression, VLA readiness를 확인했다. 실제 fire 연속 검출과 Depth 3D
발행까지 PASS했다.

첫 Mission 실패 원인은 일반 문자열 발행에 따른 JSON 계약 위반이다.

```text
wrong: 화재를 찾아 진압해줘
correct: {"mission_id":"mission_fire_001","text":"화재를 찾아 진압해줘"}
WorldModel mission: null
Nav2 goal / suppression / 실제 이동 / 분사: 0 / 0 / 0 / 0
```

다음 단계는 충전 완료, 전원선 분리, 안전한 바닥 배치 후 production runtime을
정확히 1회 clean start하고 정상 JSON Mission 1회로 fire-only 전체 E2E를 재시도하는
것이다. 다음 실행은 rosbag을 사용하지 않는다. #89/#91은 OPEN 유지한다.

## 2026-08-27 next fire-only execution

현재 production runtime은 안전 종료됐고 active production process는 0개다. 다음
세션은 `integration/vla-robot-e2e@0af75f9627170f2eee3218110225c78762cb0f29`와
`/ros2_ws/phoenix_vla/install`을 그대로 사용한다. Canonical source/environment와
실제 launch 명령은 `docs/VLA_ROBOT_E2E_CURRENT_STATUS.md`의
`2026-08-27 fire-only Hardware E2E 실행환경과 중단 상태`를 그대로 재사용한다.

```text
불 OFF에서 production start exactly once
→ compatible-QoS direct subscriber 통합 preflight 1회
→ Robot 바닥 배치와 전원선 분리
→ 표적을 화면 중앙 약 0.9 m에 배치
→ 불연성 고체 받침과 거리 측정 가능한 배경 확인
→ BEST_EFFORT passive observer를 불보다 먼저 연결
→ 불 ON
→ confidence >= 0.60 + valid depth/map + fresh ACTIVE fire
→ {"mission_id":"mission_fire_001","text":"화재를 찾아 진압해줘"} 정확히 1회
→ Qwen → Nav2 → Robot stop → Servo/Pump → Pump OFF
→ flame disappearance → terminal SUCCESS
```

Observer는 `/yolo_result`, `/fire/detections`, `/fire/detections/status`,
`/vla/world_model`, VLA navigation/spray command/result와 action status를 한 process로
관찰한다. `/fire/detections*`에는 BEST_EFFORT `qos_profile_sensor_data`를 사용한다.
매 실행마다 새 QoS나 timeout을 조립하지 않는다. Fresh ACTIVE fire가 생성되면 중간
readiness 없이 Mission부터 terminal까지 연속 실행한다. 명백히 threshold 미만으로
안정되면 불을 60초 유지하지 않고 중단한다.

오전 성공값은 confidence `0.729`, depth `0.634 m`, map 약
`(0.470, -0.514)`, `fire_0023 ACTIVE`다. 오후 실패값은 최고 confidence `0.5817`,
depth 약 `1.53 m`, `fallback_below`, WorldModel ACTIVE fire 미등록이다. Mission과
모든 HW command는 0이었다. 이 시점에는 threshold 변경을 적용하지 않았고, 이후
사용자 승인으로 `5717087`에서 `0.40`을 적용했다.

## 2026-08-27 geometry test checkpoint

Production threshold `0.40` 적용 후 VLA만 재시작해 이전 Mission/fire를 폐기했다.
Fresh fire geometry는 robot pose `(0.141, -0.268, 110.6°)`, fire
`(-0.511, 1.153)`, relative bearing `+4.0°`로 실제 카메라 정면과 일치했다.

불 OFF 후 Nav2-only 시험 직전, 정지 상태의 robot yaw가 `64.0°`로 바뀌었다.
Local costmap 정면 lethal cell은 `0/25`였지만 plan은 생성되지 않았다. 실제
`NavigateToPose` goal, Mission, Servo/Pump command는 0이며 Full E2E는 NOT RUN이다.

다음 실행은 불 OFF에서 localization/TF heading이 정지 상태에서 안정적인지 먼저
확인한다. 이전 fire, Mission, goal은 재사용하지 않는다. Heading이 안정된 뒤 fresh
fire bearing이 정면과 일치하고 plan이 생성될 때만 Nav2 goal 1회와 Full E2E를
진행한다. 현재 blocker는 `localization/TF heading stability`다.

## 2026-08-27 charging handoff: final fire-only retry

Robot과 Pi는 배터리 충전을 위해 종료됐다. 사용자가 `충전 완료, 바닥 배치 완료`를
알리기 전에는 Pi, runtime, Hardware command를 실행하지 않는다. 다음 실행에서는
기존 출발 위치를 억지로 재현하지 않고 새 안전 위치를 기준으로 localization을 정확히
1회 설정한다.

Canonical source 순서:

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

Active production process 0개를 확인한 뒤 Camera를 정확히 1회 시작하고 8초 기다린다.
그다음 아래 순서로 각각 정확히 1회 시작한다.
각 launch/run은 위 environment를 source한 별도
`docker exec IntelPi bash -lc` process에서 `nohup setsid ... </dev/null &`로 실행하고
각각 별도 `/tmp/e2e_*.log`에 기록한다. 한 shell에서 foreground로 순차 실행하지 않는다.

```bash
ros2 launch peripherals depth_camera.launch.py
sleep 8
ros2 launch uncc_example uncc_frontier.launch.py start_frontier:=false start_mission:=false start_vision:=false
ros2 run image_pipeline preprocess_node --ros-args -r __node:=rgb_preprocess_node -p input_topic:=/ascamera/camera_publisher/rgb0/image -p camera_info_topic:=/ascamera/camera_publisher/rgb0/camera_info -p output_topic:=/image_enhanced -p output_camera_info_topic:=/image_enhanced/camera_info -p mode:=passthrough
ros2 launch image_pipeline yolo.launch.py model_path:=/ros2_ws/phoenix_vla/Hailo/models/baseline_yolo26_neural_norm.hef postprocess_path:=/ros2_ws/phoenix_vla/Hailo/models/best_sim_postprocess.onnx backend:=hailo layout:=end2end class_names:="[fire,person]"
ros2 launch image_pipeline detection_3d.launch.py
ros2 launch fire_vla_bringup topic_bridge_vla.launch.py start_perception_bridge:=true llm_backend:=remote_qwen remote_qwen_endpoint:=http://<PC_IP>:8088/infer
ros2 launch uncc_example vla_navigation_bridge.launch.py
ros2 launch uncc_example fire_extinguisher.launch.py
```

Observer는 하나의 `rclpy` process에서 RGB/Depth/CameraInfo, `/yolo_result`,
`/fire/detections`, `/fire/detections/status`, `/vla/world_model`, navigation/spray
result를 함께 구독한다. Sensor와 `/fire/detections*`는
`qos_profile_sensor_data`(BEST_EFFORT, volatile)를 사용하고 WorldModel/result는
RELIABLE, volatile을 사용한다. 불보다 먼저 observer를 시작하고 최대 60초 동안 실제
event를 연속 관찰하되, confidence가 `0.40` 아래로 안정되면 즉시 불 OFF를 요청한다.
ROS CLI daemon의 echo 결과만으로 FAIL을 판정하지 않는다.

검증된 direct-subscriber observer 원문:

```bash
python3 - <<'PY'
import json, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String

rclpy.init()
n = Node("fire_e2e_observer")
counts = {k: 0 for k in ("rgb", "depth", "info", "yolo", "fire", "status", "world", "nav", "spray")}
latest = {}
sensor_topics = (
    (Image, "/ascamera/camera_publisher/rgb0/image", "rgb"),
    (Image, "/ascamera/camera_publisher/depth0/image_raw", "depth"),
    (CameraInfo, "/ascamera/camera_publisher/rgb0/camera_info", "info"),
    (Detection2DArray, "/yolo_result", "yolo"),
    (String, "/fire/detections", "fire"),
    (String, "/fire/detections/status", "status"),
)
for msg_type, topic, key in sensor_topics:
    n.create_subscription(msg_type, topic, lambda m, k=key: (counts.__setitem__(k, counts[k] + 1), latest.__setitem__(k, getattr(m, "data", None))), qos_profile_sensor_data)
reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
for topic, key in (("/vla/world_model", "world"), ("/vla/navigation_result", "nav"), ("/vla/spray_result", "spray")):
    n.create_subscription(String, topic, lambda m, k=key: (counts.__setitem__(k, counts[k] + 1), latest.__setitem__(k, m.data)), reliable)
end = time.time() + 60
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
    print(json.dumps({"counts": counts, "latest": latest}, ensure_ascii=False), flush=True)
n.destroy_node()
rclpy.shutdown()
PY
```

이 observer는 읽기 전용이다. Mission은 fresh ACTIVE fire를 확인한 뒤 별도 canonical
JSON publisher로 정확히 1회만 발행한다.

최소 실행 순서:

```text
배터리 완충 → 전원선 분리 → 새 안전 바닥 위치 배치
→ PC→Pi SSH + Pi→Qwen HTTP 200
→ canonical production start exactly once
→ 현재 위치 localization exactly once
→ 정지 yaw 5초 동안 ±5° 이내
→ observer → READY_FOR_FIRE → 불 ON → fresh ACTIVE fire
→ {"mission_id":"mission_fire_001","text":"화재를 찾아 진압해줘"} exactly once
→ Qwen → Nav2 → Robot stop → Suppression → flame removed → terminal SUCCESS
```

기존 fire, localization, Mission, goal은 재사용하지 않는다. Active Mission 중 Robot을
손으로 옮기지 않는다. Threshold는 `0.40`, spray range는 `0.8 m`다. 현재 미완료는
Nav2 terminal, system-level stop, Servo/Pump, 실제 소화, terminal SUCCESS다.

## 2026-08-27 final resume checkpoint

Authoritative checkpoint는
`integration/vla-robot-e2e@2b5ee9a12147754004d293e0d9f7d8799e69f951`다.
Out-of-range ACTIVE fire에 대한 잘못된 Qwen `EXTINGUISH`는 같은 target의
`NAVIGATE_TO`로 한 번 교정된다. `remote_qwen_timeout_sec:=10.0`, fire confidence
threshold `0.40`, spray range `0.8 m`를 유지한다.

마지막 readiness는 RGB/Depth/CameraInfo/YOLO, Detection3D frame 진행, Nav2 server
1개, Suppression server 1개까지 PASS했다. Command는 Mission/Nav2/Servo/Pump 모두
0이었다. 네트워크 단절로 WorldModel empty, Pi→Qwen HTTP 200, runtime clean stop,
Robot stop, Pump OFF의 최종 확인은 `UNKNOWN`이며 `READY_FOR_FIRE=NO`다. 이전
fire/Mission/goal은 재사용하지 않는다.

다음 세션 최소 순서:

1. 배터리 충분, 불 OFF, 충전선 분리, 안전한 바닥 배치를 확인한다.
2. Pi IP와 SSH 실제 명령 응답을 확인한다.
3. Active production process를 한 번 확인한다. 살아 있으면 재사용하고, 없을 때만
   위 canonical Camera-first 순서로 clean start를 정확히 1회 수행한다.
4. WorldModel empty와 Pi→현재 PC Qwen endpoint HTTP 200만 마무리한다.
5. 점화 중 VLA/Navigation Bridge를 OFF로 유지하고 사람이 화각에서 빠진 뒤 시작해
   새 WorldModel `people=0`과 fresh ACTIVE fire를 확인한다.
6. `READY_FOR_FIRE` 이후 이전과 다른 새 Mission ID로 Mission을 정확히 1회 발행해
   Qwen → Nav2 → Robot stop → Suppression → terminal SUCCESS를 연속 관찰한다.

SSH 조회 실패만으로 runtime을 다시 시작하지 않는다. Mission, Nav2 goal, 전체
runtime start를 중복 실행하지 않는다. `REPORT_PERSON` terminal result 미반영은 별도
known issue이며 fire-only 완료와 분리해 추적한다.


## Production 운영 wrapper

`/ros2_ws/phoenix_vla`에서 `scripts/vla_hardware_e2e.sh`를 사용한다.

```bash
export VLA_QWEN_ENDPOINT=http://<CURRENT_PC_IP>:8088/infer
scripts/vla_hardware_e2e.sh start
scripts/vla_hardware_e2e.sh status
scripts/vla_hardware_e2e.sh mission
scripts/vla_hardware_e2e.sh stop
```

`start`는 기존 process가 있으면 중복 시작하지 않는다. 새 runtime은 Camera 선기동,
8초 대기, 나머지 canonical stack 순서를 유지하며 component 로그를
`/tmp/e2e_<component>.log`에 저장한다. `mission`은 새 ID로 정확히 한 번 발행한다.
SSH 실패 뒤 `start`를 재전송하지 말고 `status`로 실제 process 상태를 먼저 확인한다.
명령·경로 확인에는 `VLA_E2E_DRY_RUN=1`을 사용한다.

## 2026-08-29 next Hardware execution

Software checkpoint는 `0a3af10882d29bfcc51aac34905fe1d84703b6ee`이며 focused
`120 PASS`, full `273 PASS`, 신규 실패 0이다. UI 포함 SW-only E2E에서 Mission
boundary, person 자동 보고와 위험 표시, Qwen 1회, Mock Nav2 성공, deterministic
EXTINGUISH, suppression verification, Mission/UI `COMPLETED`, 중복 action 0을 확인했다.

다음 실제 작업은 #89 fire-only actual Hardware E2E다. 과거 Hardware checkpoint와
명령 원문은 historical evidence로 보존하되 새 실행은 wrapper를 사용한다.

```text
배터리/바닥/불 OFF 확인
→ 현재 PC Qwen endpoint 설정
→ scripts/vla_hardware_e2e.sh start
→ scripts/vla_hardware_e2e.sh status
→ READY_FOR_FIRE
→ fresh ACTIVE fire
→ scripts/vla_hardware_e2e.sh mission
→ actual Qwen → Nav2 → Robot stop → Servo/Pump
→ 실제 화염 제거 → terminal SUCCESS
→ scripts/vla_hardware_e2e.sh stop
```

Mission scope는 첫 Qwen 구조화 응답에서 action과 함께 `FIRE_ONLY`, `PERSON_FIRE`,
`FULL_EXPLORATION` 중 하나로 고정된다. Fire-only 완료에는 관계없는 person/fire와
exploration 상태를 요구하지 않는다. Suppression 후에는 기존 `PENDING_VERIFICATION`,
0.5초 delay, 유효한 fire 미검출 3회, 5초 timeout 시 ACTIVE 복귀 계약을 그대로
적용한다. 실제 Camera/Qwen/Nav2/Servo/Pump, 화염 제거, terminal SUCCESS는 여전히
`HARDWARE_PENDING`이다.
