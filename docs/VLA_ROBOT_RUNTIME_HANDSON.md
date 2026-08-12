# VLA Robot Runtime Hands-on

이 문서는 VLA-07에서 실제 측정한 PC/Jazzy와 Robot/Pi/Humble 분리 배치를 기준으로 한다. 비밀번호와 개인 SSH secret은 기록하지 않는다.

## Runtime architecture

```text
Ubuntu PC / ROS 2 Jazzy
├─ VLA Core / Orchestrator
├─ Mock 또는 Qwen
├─ VLA Perception Bridge
└─ Firefighter UI

Raspberry Pi host / Debian 13 (host ROS 없음)
└─ IntelPi Docker / Ubuntu 22.04 / ROS 2 Humble
   ├─ Hardware driver
   ├─ SLAM Toolbox
   ├─ Nav2
   ├─ uncc_example
   └─ VLA Navigation Bridge
```

`uncc_example`, Hardware, SLAM, Nav2는 Pi Humble Docker에서 build/run한다. PC/Jazzy에 Nav2를 추가 설치하지 않는다.

## 접속과 환경 확인

PC를 대상 Robot hotspot에 연결하고 gateway와 SSH fingerprint를 확인한 뒤 Pi host에 SSH로 접속한다.

```bash
cd ~/docker
./exec_shell.sh
id
printenv ROS_DISTRO
printenv ROS_DOMAIN_ID
printenv RMW_IMPLEMENTATION
ros2 pkg prefix nav2_msgs
ros2 pkg prefix nav2_bringup
ros2 pkg prefix navigation2
ros2 pkg prefix slam_toolbox
```

`exec_shell.sh`는 `IntelPi`를 `ubuntu` 사용자로 연다. `pyserial` 같은 user-local dependency 때문에 Robot launch를 root로 실행하지 않는다. 실측 container는 host network, privileged, `/dev` bind를 사용한다.

## DDS test-only 환경

Persistent 기준은 PC `124 + CycloneDDS`, Pi `205 + Fast DDS`이다. persistent 설정을 바꾸지 않고 PC test process에만 적용한다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=205
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

이 조합에서 PC↔Pi String 양방향과 `/vla/*` JSON transport가 PASS했다.

## Stationary runtime

Frontier, MissionExecutor, avoidance motion owner, VLA goal publish를 OFF로 유지한다.

```text
hardware.launch.py → controller.launch.py → odom_publisher.launch.py
→ ros_robot_controller.launch.py → ros_robot_controller
```

```bash
ros2 topic info /odom -v
ros2 topic info /scan_raw -v
ros2 topic info /map -v
ros2 run tf2_ros tf2_echo map base_footprint
ros2 lifecycle get /bt_navigator
ros2 action info /navigate_to_pose
ros2 topic echo /vla/robot_pose_json --once --full-length
```

Frame contract는 `map → odom → base_footprint`, odom topic은 `/odom`이다. 속도 경로는 `controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel → odom_publisher → /ros_robot_controller/set_motor`이다.

## Explicit software halt

`PHYSICAL_ESTOP_NOT_IMPLEMENTED`. 정상 중지는 Nav2/VLA goal cancel → cmd_vel 정지 확인 → explicit four-motor zero 순서다.

```bash
ros2 topic info /ros_robot_controller/set_motor -v
ros2 topic pub --once /ros_robot_controller/set_motor \
  ros_robot_controller_msgs/msg/MotorsState \
  "{data: [{id: 1, rps: 0.0}, {id: 2, rps: 0.0}, {id: 3, rps: 0.0}, {id: 4, rps: 0.0}]}"
```

## 실제 short-nav 전 checklist

1. hotspot/route/SSH identity와 별도 zero-halt terminal을 확인한다.
2. odom, scan, map, TF, Nav2 lifecycle/action server를 확인한다.
3. Frontier와 MissionExecutor를 OFF로 두고 goal owner가 하나인지 확인한다.
4. 실제 `/vla/robot_pose_json`이 연속 갱신되는지 확인한다.
5. WorldModel의 robot pose freshness와 `person_0001`을 확인한다.
6. expected decision이 `NAVIGATE_TO person_0001`인지 확인한다.
7. Resolver target이 map `(0.5, 0.0)`, Validator가 PASS인지 확인한다.
8. 그 다음에만 actual navigation backend를 enable한다.

Production perception을 함께 사용할 때는 그 전에 다음도 확인한다.

1. `/yolo_result`와 `/fire/detections/status`가 실제 publish되는지 확인한다.
2. status가 `ok`인지 확인하고 detection silence만으로 failure로 판정하지 않는다.
3. rgb0 CameraInfo의 해상도와 `/fire/detections.frame_size`가 같은지 확인한다.
4. `/fire/detections`의 timestamp가 계속 원본 source stamp인지 확인한다.
5. camera optical frame에서 source timestamp의 `map` TF가 가능한지 확인한다.
6. `/vla/perception_observation`에 projected `map_position`과 scaling하지 않은
   `confidence=score`가 나오는지 확인한다.
7. `unknown`은 WorldModel 위치를 갱신하지 않고 `fallback_*` status는 보존되는지
   확인한다.

Hardware 없이 4~7번 계약을 먼저 확인할 수 있다.

```bash
ros2 run fire_vla_core vla_short_nav_preflight
```

PASS 출력에는 `robot_pose_fresh=true`, `person_0001`,
`decision=NAVIGATE_TO`, target map `(0.5,0.0)`,
`validator_approved=true`, `submission=ACCEPTED`,
`navigation_adapter=MockNavigationAdapter`,
`actual_nav2_goals=0`, `cmd_vel_messages=0`이 포함되어야 한다.

## 금지 사항

- persistent PC/Pi domain 또는 RMW 설정 변경
- Pi team workspace 직접 수정
- goal owner가 둘 이상인 상태에서 navigation 시작
- pose stream이 끊겼거나 Validator가 stale을 반환한 상태에서 goal 전송
- motor board subscriber와 explicit zero halt를 확인하지 않은 상태에서 이동
- production YOLO가 준비됐다고 가정

Production YOLO raw `score`는 실제 연결 boundary에서 scaling 없이 VLA
`confidence`로 이름만 mapping한다.
