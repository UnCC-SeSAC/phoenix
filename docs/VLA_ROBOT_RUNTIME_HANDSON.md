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
