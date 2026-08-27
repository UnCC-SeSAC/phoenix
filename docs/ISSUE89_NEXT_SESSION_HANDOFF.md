# Issue #89 Next Session Handoff

Date: 2026-08-22 KST

이 문서는 새 Codex session이 workspace 복원이나 기존 PASS 항목 재진단 없이 Final
Full E2E를 바로 이어가기 위한 authoritative handoff다. 먼저
`docs/VLA_ROBOT_E2E_CURRENT_STATUS.md`와
`docs/VLA_ROBOT_RUNTIME_TROUBLESHOOTING.md`를 읽고 이 절차를 따른다.

## Git and isolated deployment checkpoint

- branch: `integration/vla-robot-e2e`
- runtime/model checkpoint before this handoff commit:
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
모든 HW command는 0이었다. Threshold 변경은 적용하지 않았다.
