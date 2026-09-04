# phm_collect

PHM 잔차 수집 전용 최소 기동 패키지.

## 왜 만들었나

`bringup.launch.py` 는 48노드를 띄웁니다. live08 수집에서 CPU 중앙 **90.9%**, 전송 오프셋
**670ms**, `/diagnostics` **22% 유실**이 났습니다. 그런데 잔차 계산에 실제로 쓰는 토픽은
다섯 개뿐입니다. 나머지는 CPU 만 먹고 수집 품질을 깎습니다.

## 무엇이 올라가나

| 노드 | 나오는 토픽 | 역할 |
|---|---|---|
| `ros_robot_controller` | `/ros_robot_controller/imu_raw`, `/battery` | 시리얼 하드웨어. 자이로 = 잔차의 '실측' |
| `odom_publisher` | `/odom_raw`, `/ros_robot_controller/set_motor` | `cmd_vel` → 모터. 차동 지령 확인용 |
| `robot_state_publisher` + `joint_state_publisher` | TF | rf2o 가 스캔→`base_footprint` 변환에 필요 |
| `imu_calib` + `imu_filter` | `/imu` | 보정·필터된 IMU |
| `LD19` | `/scan_raw` | 라이다 |
| `rf2o_laser_odometry` | `/odom_rf2o` | 스캔매칭. 전진속도 잔차의 '실측' |
| `rf2o_covariance_relay` | `/odom_rf2o_fixed` | rf2o 의 0 covariance 를 실측 기반 값으로 채움 |
| `ekf_filter_node` | `/odom` | 3-way 융합 |
| `joy_node` + `joystick_control` | `/controller/cmd_vel` | 수동 주행. 잔차의 '지령' |

**안 올라가는 것**: `ascamera`(depth camera), `start_app` 의 앱 노드 5개, `rosbridge_websocket`,
`web_video_server`, `startup_check`, `init_pose`, 그리고 nav2 / slam / yolo / mission 계층 전부.

## 쓰는 법

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
source /ros2_ws/phoenix/install/setup.bash
source /home/ubuntu/third_party_ros2/third_party_ws/install/setup.bash   # rf2o, ldlidar, imu_calib

ros2 launch phm_collect collect.launch.py
```

| 인자 | 기본 | 설명 |
|---|---|---|
| `use_ekf` | `true` | 끄면 EKF + 릴레이 2노드가 빠집니다. `/odom` 도 안 나옵니다 |
| `use_joy` | `true` | nav2 등 외부에서 `/cmd_vel` 을 쏠 때는 `false` |
| `use_rf2o` | `true` | 전진속도 잔차를 안 볼 때만 `false` |
| `use_lidar` | `true` | `use_rf2o:=true` 면 반드시 켜야 합니다 |
| `use_imu_filter` | `true` | `/imu` 가 필요 없으면 `false`. 잔차는 `imu_raw` 를 쓰므로 영향 없음 |
| `max_linear` | `0.2` | 조이스틱 전진 상한 (m/s) |
| `max_angular` | `0.5` | 조이스틱 회전 상한 (rad/s) |
| `rf2o_freq` | `10.0` | rf2o 처리 주기 (Hz) |
| `machine_type` | `MentorPi_Mecanum` | 바꾸지 마세요 (아래 참고) |

## 반드시 알아야 할 세 가지

### 1. 왼쪽 스틱 좌우(lx)는 건드리지 마세요

이 로봇은 좌우쌍 차동(skid-steer)인데 `MACHINE_TYPE` 은 `MentorPi_Mecanum` 입니다.
mecanum 기구학에 `linear.y=0` 을 넣으면 차동이 나오기 때문입니다. `lx` 축은
`twist.linear.y` 로 매핑되는데(`joystick_control.py:67`) 하드웨어가 못 내는 지령입니다.

- 전진/후진 = 왼쪽 스틱 **상하** (`ly`)
- 회전 = 오른쪽 스틱 **좌우** (`rx`)

### 2. 속도 상한을 함부로 올리지 마세요

`/cmd_vel` 경로는 `odom_publisher_node.py:181-192` 에서 ±0.2 m/s, ±0.5 rad/s 로 클램프됩니다.
그런데 조이스틱이 쓰는 `/controller/cmd_vel` 경로는 **클램프가 없습니다**(`:132` → `cmd_vel_callback` 직결).
기본값 0.2 / 0.5 는 그 클램프와 일부러 맞춘 값입니다. 올려서 수집하면 nav 이 절대 못 내는
속도의 데이터가 섞여서, 거기서 잡은 임계값이 실제 주행에 안 맞습니다.

그리고 **스틱을 천천히 움직이세요.** 검출기는 `e(t) = wz_gyro(t+lag) − gain·wz_cmd(t)` 이고
`lag`/`gain` 을 정상 데이터에서 적합합니다. 계단 입력을 주면 로봇이 못 따라가는 게 정상인데도
잔차로 크게 잡혀서 오경보 기준선이 부풀고 임계가 과도하게 느슨해집니다.

### 3. 수집 쪽 토픽 이름을 맞춰야 합니다

조이스틱은 `/cmd_vel` 이 **아니라** `/controller/cmd_vel` 에 씁니다. 수집 측에서 두 가지를
고쳐야 잔차가 계산됩니다.

- `08_residual_detect.py` — 지령 토픽에 `/controller/cmd_vel` 추가
- `ensure_bridge.sh` — `--rate-limit` 에 `/controller/cmd_vel=0` 추가.
  안 하면 `*=5` 규칙에 걸려 5Hz 로 깎이는데, 원본은 20Hz(`autorepeat_rate`)이고
  검출기의 지속조건 N 은 20Hz 기준이라 시간 의미가 4배로 어긋납니다.

## 설계 메모

**기존 런치를 include 하지 않습니다.** 로봇 런타임이 `need_compile=False` 라서
`controller.launch.py:38` 과 `odom_publisher.launch.py:31-33` 이 패키지 경로를
`/home/ubuntu/ros2_ws/src/...`(벤더 워크스페이스)로 잡습니다. phoenix 의 런치가 실행돼도
그 안에서 include 하는 하위 런치와 `ekf.yaml` 은 벤더 사본을 읽어 phoenix 쪽 수정이
조용히 무시됩니다. 그래서 필요한 노드를 직접 선언하고 경로는 전부
`get_package_share_directory` 로 풉니다.

**`config/ekf_collect.yaml`** 은 `controller/config/ekf.yaml` 에서 `namespace/` 치환자만
확정한 사본입니다. 단일 로봇 전용입니다.

**rf2o 는 pose 만 융합하고 twist 는 안 씁니다.** live08/09 정지 실측에서 rf2o 의 `|wz|` 는
자이로의 15~23배였습니다(p95 0.13~0.20 대 0.006~0.009 rad/s). 요레이트 채널에 넣으면
자기 잡음만으로 검출 임계 0.25 rad/s 를 넘깁니다. 반면 pose 드리프트는 60초에 yaw 2.2°,
위치 1cm 로 작습니다. **요레이트는 자이로, 전진속도는 rf2o** 가 결론입니다.

`rf2o_covariance_relay` 의 `POSE_COV`(yaw 0.05, xy 0.02)는 위 실측 대비 6~13배 보수적입니다.
초기 브링업엔 안전하지만 rf2o pose 가 EKF 에 거의 기여를 못 합니다. **주행 런에서 드리프트를
다시 잰 뒤** 조이세요 — 정지 수치로 조이면 안 됩니다.

---

## 센서 없는 기기에서 목업으로 검증하기

로봇이 없거나 센서가 없어도 **수집한 JSONL 을 진짜 ROS 토픽으로 다시 틀어** PHM 전체
경로를 확인할 수 있습니다. `phm_mock_source` 노드가 그 일을 합니다.

검증된 환경: `lemma@192.168.1.174` 의 `IntelPi` 컨테이너 (ROS Humble, aarch64,
Python 3.10, host 네트워크). 2026-09-04 실측.

### 0) 준비 — 수집 파일을 기기로

```bash
# 호스트에서 (컨테이너의 /shared 는 파이의 ~/docker/shared 입니다)
scp data_analysis/out/live24_normal3.jsonl \
    data_analysis/out/live25_lift.jsonl   lemma@192.168.1.174:~/docker/shared/
```

정상 런과 고장 런을 **둘 다** 가져가세요. 고장이 잡히는 것만 보고 끝내면 오경보가
없다는 것을 확인 못 합니다.

### 1) 빌드

```bash
ssh lemma@192.168.1.174
docker exec -it IntelPi bash
source /opt/ros/humble/setup.bash
cd /ros2_ws/phoenix
colcon build --packages-select phm_collect fire_vla_core --symlink-install
source install/setup.bash          # ★ 안 하면 'Package not found' 가 납니다
```

### 2) 띄우기 — 터미널 두 개

```bash
# [터미널 1] 목업 재생 + PHM 감시
export ROS_DOMAIN_ID=205
ros2 launch phm_collect phm_monitor_mock.launch.py \
    jsonl:=/shared/live25_lift.jsonl loop:=true

# [터미널 2] UI. 컨테이너가 host 네트워크라 이대로 브라우저에서 붙습니다.
export ROS_DOMAIN_ID=205
ros2 run fire_vla_core firefighter_ui --ros-args \
    -p ui_host:=0.0.0.0 -p ui_allow_remote:=true \
    -p ui_vision_enabled:=false -p ui_map_enabled:=false
```

그리고 브라우저에서 **http://192.168.1.174:8080** — `Robot Health (PHM)` 패널.

> `ui_allow_remote:=true` 는 **Mission/START/STOP 제어 경계까지 LAN 에 엽니다.**
> PHM 만 볼 거면 `ui_host:=127.0.0.1` 로 두고 컨테이너 안에서 curl 하세요.

`rf2o` 는 띄우지 않습니다 — 목업이 `/odom_rf2o` 를 직접 냅니다. 같이 띄우면 발행자가
둘이 됩니다. (라이다가 없으면 rf2o 는 어차피 아무것도 못 냅니다.)

### 3) 무엇을 봐야 하나

```bash
ros2 topic hz /phm/status          # 1.000 Hz 여야 합니다
curl -s http://127.0.0.1:8080/api/phm | python3 -m json.tool | head -30
```

| 재생 파일 | 기대 |
|---|---|
| `live25_lift.jsonl` | `health: ALARM`, `alarms` 에 `LIFT_SUSPECTED` |
| `live24_normal3.jsonl` | `health: OK`, `alarms: []` — **여러 번 확인하세요** |

`live24` 를 한 바퀴 돌리는 동안 경보가 **한 번도** 안 떠야 합니다. 이게 임계가
살아 있다는 유일한 증거입니다.

`잔차 > 임계` 이고 `창비율 = 1.000` 인데 경보가 아닌 순간이 정상 런에 나옵니다.
**버그가 아닙니다** — 창이 표본으로 안 찬 것입니다(`MIN_FILL`, 진행상황 19.3).
정지 후 재출발 구간이 경보가 되던 문제를 막는 조건입니다.

### 4) 끊김도 확인하세요

경보가 뜨는 것만큼 **끊긴 값을 정상으로 안 보여주는지**가 중요합니다.
두 실패는 다르게 보여야 합니다.

```bash
pkill -f phm_mock_source     # 센서만 끊김  -> stale:false, 축 fresh:false, health UNKNOWN
pkill -f phm_monitor         # 노드가 사라짐 -> stale:true (age 증가), health UNKNOWN
```

둘 다 `health` 는 `UNKNOWN` 이고 **잔차 값은 남아** 있어야 합니다
('마지막으로 본 값' 을 화면에 표시하기 위해서입니다).

### 5) 정리

```bash
pkill -f phm_mock_source; pkill -f phm_monitor
# ros2 run 은 부모/자식 두 개라 PID 로 끄는 게 확실합니다
ps -eo pid,args | grep firefighter_ui | grep -v grep
kill <PID들>
```

### 확인되는 것 / 안 되는 것

| | |
|---|---|
| **확인됨** | colcon 빌드, QoS 협상, DDS 왕복, `/phm/status` 주기, `/api/phm`, 브라우저 화면, 정상=무경보 / 고장=경보, 두 층의 끊김 처리 |
| **확인 안 됨** | 실제 센서 주기·지터, rf2o 실기동, 진짜 주행, CPU 실부하 |

**목업은 실기 검증을 대신하지 못합니다.** 배선이 맞는지를 볼 뿐입니다.
