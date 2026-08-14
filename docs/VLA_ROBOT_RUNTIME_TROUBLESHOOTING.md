# VLA Robot Runtime Troubleshooting

이 문서는 VLA-07 Hardware Gate에서 반복 관측된 PC–Robot 연결 및 Pi runtime
장애를 계층별로 구분하기 위한 이력과 최소 점검 순서를 정리한다. 확정되지 않은
원인은 추정으로 단정하지 않는다. 정상 접속·기동 절차는
`VLA_ROBOT_RUNTIME_HANDSON.md`를 따른다.

## 장애 이력

### 1. 다른 Robot hotspot으로 자동 전환

- **증상:** PC 연결이 `uncc`에서 `robot_of_intel`로 바뀐 뒤 SSH와 DDS Robot
  pose stream이 소실됐다.
- **관측 증거:** 활성 SSID 변경 시점과 SSH/DDS 단절 시점이 일치했다.
- **원인 판정:** NetworkManager의 다른 hotspot 자동 연결이 원인으로 확인됐다.
- **수행한 조치:** `robot_of_intel`은 `autoconnect=no`, `uncc`는
  `autoconnect=yes`, priority `100`으로 설정했다.
- **결과:** 이후 동일 구성에서 network continuity가 장시간 유지됨을 실측했다.
- **재발 시 확인:** 활성 SSID, `uncc` connection profile 및 autoconnect 우선순위.

### 2. Jazzy/Humble `ParticipantEntitiesInfo` 오류

- **증상:** ROS graph 조회 중 Fast CDR 역직렬화 또는 `Bad alloc` 오류가
  `rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_`에 대해 출력됐다.
- **관측 증거:** 같은 환경에서 별도 `std_msgs/msg/String` 양방향 payload는 실제
  수신됐다.
- **원인 판정:** 현재 증거상 Jazzy/Humble 사이의 ros2cli/DDS graph metadata
  interoperability 문제다. user payload 전송 실패와 동일한 현상이 아니다.
- **수행한 조치:** graph 오류와 별도로 격리 String topic의 실제 수신을 확인했다.
- **결과:** Fast DDS 양방향 String transport는 PASS했다.
- **재발 시 확인:** graph count만 보지 말고 안전한 격리 topic의 실제 payload와
  production topic callback을 각각 확인한다.

### 3. stale `/vla/robot_pose_json`

- **증상:** WorldModel에 pose가 한 번 들어왔지만 navigation 판단 시 Validator가
  stale pose로 거부했다.
- **관측 증거:** network/DDS 단절 뒤 pose timestamp 갱신이 멈췄다.
- **원인 판정:** 당시 원인은 pose producer의 좌표 계산이 아니라 continuous
  network/DDS stream 중단이었다.
- **수행한 조치:** pose를 one-shot 값이 아닌 연속 stream으로 검증하고 freshness
  회귀 테스트를 추가했다.
- **결과:** Validator는 stale physical state에서 navigation을 정상 차단했다.
- **재발 시 확인:** Pi publish, PC receive, WorldModel `pose_updated_at`을 순서대로
  비교한다.

### 4. 수동 Robot 재배치 후 TF/costmap context 변경

- **증상:** Robot을 손으로 옮긴 뒤 TF yaw와 costmap 상태가 달라져 이전 target이
  더 이상 같은 의미를 갖지 않았다.
- **관측 증거:** 수동 이동 전후 wheel odom, SLAM pose correction, map target
  context가 일치하지 않았다.
- **원인 판정:** wheel odom이 수동 이동을 직접 관측하지 못하는 동안 LiDAR 기반
  SLAM/map alignment가 재조정된 결과로 판단한다.
- **수행한 조치:** 재배치 후 fresh mapping/TF와 live target을 다시 확인했다.
- **결과:** 고정 위치 관측에서는 TF가 안정됐고 안전 후보를 다시 산출할 수 있었다.
- **재발 시 확인:** Robot 배치 후 테스트가 끝날 때까지 손으로 이동하거나 돌리지
  않는다.

### 5. LiDAR/TF timestamp warning

- **증상:** slam_toolbox 및 costmap message filter에서 queue full 또는 TF cache보다
  이른 timestamp 경고가 일부 발생했다.
- **관측 증거:** `/scan_raw` 약 10 Hz, TF lookup 연속 성공, local/global costmap
  지속 갱신을 동시에 확인했다.
- **원인 판정:** timestamp 차이의 세부 발생원은 완전히 확정하지 않았으나, 관측
  구간에서 short-nav를 막는 outage는 아니었다.
- **수행한 조치:** warning count가 아니라 scan/TF/costmap의 실제 continuity와
  freshness를 측정했다.
- **결과:** `SHORT_NAV_NON_BLOCKING`으로 판정했다.
- **재발 시 확인:** scan 대부분이 drop되는지, TF outage 또는 costmap stale이
  실제 발생하는지를 확인한다.

### 6. IMU/EKF stationary yaw variation

- **증상:** 정지 상태 yaw가 이전 실측 범위와 다르다는 이유로 주행 전 중단한 적이
  있다.
- **관측 증거:** 초기 관측 IMU yaw range는 약 `0.00854 rad`, 후속 관측은 약
  `0.02 rad` 수준이었다.
- **원인 판정:** 단일 과거 측정값은 안전 임계값이 아니며 경미한 stationary yaw
  jitter는 IMU/EKF runtime warning 범주다.
- **수행한 조치:** 과거 실측값을 Hard Gate로 사용하는 정책을 폐기했다.
- **결과:** 필수 TF availability가 유지되는 한 경미한 yaw variation만으로 motion을
  차단하지 않는다.
- **재발 시 확인:** 고정 숫자 비교 대신 TF 소실, localization jump, 제어 불능 같은
  실제 장애 여부를 본다.

### 7. 첫 navigation publish 확인 실패

- **증상:** Decision, Resolver, Validator는 PASS했지만 Pi Bridge 전달 기록과 실제
  NavigateToPose goal이 없었다.
- **관측 증거:** 임시 Hardware Gate Python process가 publish 직후 진단 출력에서
  `Pose2D.to_dict()`를 호출해 `AttributeError`로 종료됐다.
- **원인 판정:** production `TopicBridgeNavigationAdapter` 결함이 아니라 임시 진단
  process의 잘못된 출력 코드와 즉시 종료가 원인이었다.
- **수행한 조치:** 격리 ROS domain에서 production Adapter와 deterministic subscriber를
  연결해 실제 `/vla/navigation_goal` payload를 검증했다.
- **결과:** production Adapter publish path는 PASS했다.
- **재발 시 확인:** production executor가 계속 spin하는지와 Pi Bridge callback이 같은
  `action_id`를 받았는지 확인한다.

### 8. 최신 Hardware Gate의 Pi runtime/stack 소실

- **증상:** pre-dispatch 도중 `/vla/robot_pose_json` publisher와 Nav2 action server가
  사라지고 SSH session도 단절됐다.
- **관측 증거:** 최종 graph에서 `/navigate_to_pose` server `0`, pose payload 없음,
  SSH banner timeout을 확인했다. actual goal, non-zero `cmd_vel`, Robot 이동은 모두
  `0`이었다.
- **원인 판정:** **미확정**. network, Pi host, Docker, ROS process 중 최초 실패
  계층이 아직 분리되지 않았다.
- **수행한 조치:** goal 전송을 중단하고 production Orchestrator를 종료했다.
- **결과:** pre-dispatch safe abort로 종료했으며 production hot-fix는 하지 않았다.
- **재발 시 확인:** 아래 최소 진단표를 위에서부터 적용해 최초 소실 계층과 시간을
  기록한다.

### 9. 고부하로 보이는 시점의 소음과 연결 단절

- **증상:** 사용자는 Codex가 비교적 무거운 작업을 수행할 때 Robot/Pi 쪽에서 팬으로
  추정되는 큰 “웽” 소리가 나고 비슷한 시점에 hotspot/runtime이 끊기는 현상을
  반복 관찰했다.
- **관측 증거:** 현장 체감 소음과 연결 단절의 시간적 근접성만 있다. 온도, 전력,
  OOM 또는 process exit를 같은 시점에 계측한 증거는 아직 없다.
- **원인 판정:** **OBSERVATION이며 원인 미확정**이다.
- **수행한 조치:** 아직 이 관찰만을 근거로 설정이나 production code를 변경하지
  않았다.
- **결과:** 조사 후보를 CPU load/thermal, RAM/OOM, power/undervoltage,
  Docker/resource contention, Wi-Fi AP failure, USB/device instability로 한정해
  기록했다.
- **재발 시 확인:** 소음 발생 시각과 아래 host/container 자원 지표 및 kernel log를
  같은 타임라인으로 수집한다.

### 10. Pi runtime 자원 및 DDS locator 진단

- **증상:** Hardware → SLAM → Nav2 → VLA Bridge 순차 기동에서 Nav2 이후 network
  latency가 증가했고 Pi ROS log에 `192.168.100.124` 방향 CycloneDDS UDP write
  failure가 반복됐다.
- **관측 증거:** Pi 최고 온도 약 `50.5°C`, `get_throttled=0x0`, swap 사용·OOM·
  undervoltage·Docker restart는 모두 없었고 RAM도 충분했다. VLA Bridge 추가 시
  Docker CPU는 증가했지만 thermal/power/RAM failure는 재현되지 않았다.
- **원인 판정:** `192.168.100.124`는 PC 유선 NIC `enp130s0`이다. PC의 다중 NIC
  locator 광고와 DDS 재시도가 latency에 영향을 줬을 가능성이 있으나, 반복되는
  Robot runtime 소실의 전체 root cause는 아직 미확정이다.
- **수행한 조치:** 이번 Hardware test에서 생성된 Jazzy/domain 205/Fast DDS
  `ros2-daemon` 하나를 소유자·환경·시작 시각으로 식별해 종료했다. persistent PC
  `124 + CycloneDDS` 설정은 변경하지 않았다.
- **결과:** stale test ROS/DDS process 0, Robot Wi-Fi route 정상, ping 5/5와 SSH
  응답을 확인했다. 이후 Hardware 재시도에서 pose stream 소실이 재발했으므로 stale
  daemon 하나만을 전체 원인으로 보지 않는다.
- **재발 시 확인:** test process에 Robot Wi-Fi NIC만 사용하도록 한정하고, Pi가
  접근할 수 없는 locator 광고 여부와 actual payload continuity를 함께 확인한다.

### 11. Production short-nav 재시도의 pre-dispatch continuity 소실

- **증상:** production Orchestrator와 XPU/Qwen을 기동하고 actual Robot pose 및 fresh
  `person_0001`을 WorldModel에 반영한 뒤 Pi ping·SSH·pose stream이 소실됐다.
- **관측 증거:** 시작 pose는 약 `(0.000, 0.000, yaw 0.059)`, 승인된 목표는 약
  `(0.400, 0.010, yaw 0.024)`였다. Mission 입력 전 중단했으며 navigation goal,
  non-zero `cmd_vel`, Robot 이동은 모두 0이었다.
- **원인 판정:** 실패 boundary는 production decision 이전 Robot runtime/DDS input
  continuity다. Nav2, motor 또는 production navigation publisher 실행 실패가 아니다.
- **수행한 조치:** production Orchestrator와 test monitor를 종료하고 Pi 응답 복구 후
  test stack을 종료했다. production code와 Pi team workspace는 수정하지 않았다.
- **결과:** `FIRST_SHORT_NAV_E2E_FAIL`. actual NavigateToPose는 아직 실행되지 않았다.
- **재발 시 확인:** 이미 완료한 software/yaw/timestamp/resource 진단을 반복하지 말고
  Pi host, container, ROS stack, DDS pose 중 최초 소실 계층만 최소 분리한다.

### 12. 동일 SHA isolated deployment 재사용

- integration `cc20d29ca2214a6b93f7d3e944d65f0c5cc976c8`은
  `/tmp/vla_integration_deploy_cc20d29_clean`에서 필요한 11개 package build와 isolated
  prefix 우선순위, `ObstacleLayer`, 최신 footprint/DWB/velocity smoother 및
  `ObstacleFootprint.scale: 0.02`를 확인했다.
- 같은 SHA의 Hardware retry에서는 해당 경로의 `build/`, `install/`, package prefix와
  config가 유효하면 그대로 재사용한다. retry 자체를 이유로 새 workspace나 전체 clean
  build를 만들지 않는다.
- rebuild는 SHA 변경, deployment 삭제, artifact 손상 증거, source/config 불일치 또는
  코드 변경에 따른 dependency 변화가 있을 때만 수행한다.
- `/opt/ros/humble → 검증된 Robot/vendor runtime install(read-only) → isolated
  integration overlay` 순서를 사용한다. 팀 source/config/build/install은 수정하지 않고,
  integration에서 변경된 package와 config는 isolated overlay가 먼저 해석돼야 한다.
- 기존 이력에서 calibration, `mentorpi_description`, `ros_robot_controller`,
  `ros_robot_controller_msgs`, `MACHINE_TYPE=MentorPi_Mecanum`, ubuntu user-local
  `pyserial` 조건은 이미 확인됐다. 같은 SHA에서 dependency를 처음부터 재탐색하지 않는다.
- 최근 `ISOLATED_DEPLOY_SHORT_NAV_FAIL`은 deployment/build 실패가 아니라 boundary K,
  Robot/DDS continuity 소실이다. actual goal, non-zero `cmd_vel`, 이동, Pump 명령은 모두
  0이었으므로 다음 retry는 identity와 deployment 유효성을 짧게 확인한 뒤 runtime
  continuity와 actual short-nav 1회에 집중한다.

## 재발 시 최소 진단표

“연결 끊김”을 하나의 원인으로 처리하지 말고 첫 FAIL 단계에서 실패 계층과 시각을
기록한다. Robot motion 중이면 진단보다 검증된 정지 절차를 우선한다.

| 순서 | 확인 대상 | 최소 확인 | 구분되는 실패 계층 |
|---:|---|---|---|
| 1 | PC Wi-Fi | 활성 SSID가 `uncc`인지 | PC connection profile/AP 전환 |
| 2 | Pi network | `10.42.0.1` ping 응답 | hotspot link 또는 Pi network |
| 3 | SSH | TCP/22 및 SSH login 가능 여부 | network와 Pi user space 분리 |
| 4 | Pi host | hostname, uptime, host process 응답 | Pi host reboot/hang |
| 5 | Docker | `IntelPi` running 상태 | container stop/restart |
| 6 | ROS process | hardware, SLAM, Nav2, VLA Bridge process/node | ROS stack만 종료됐는지 |
| 7 | Nav2 | `/navigate_to_pose` action server 존재 | Nav2 lifecycle/runtime |
| 8 | CPU/온도 | load, throttling, thermal 상태 | CPU contention/thermal 후보 |
| 9 | Memory/OOM | free memory, swap, kernel OOM 기록 | RAM pressure/OOM 후보 |
| 10 | 전원 | undervoltage/throttled 또는 power warning | 전원 불안정 후보 |
| 11 | Container 이력 | exit code, restart count, start/finish 시각 | Docker exit/restart 후보 |

모든 계층이 살아 있는데 DDS topic만 끊겼다면 publisher 존재, subscriber 존재,
RMW/domain/interface, 실제 user payload 순으로 확인한다. `ParticipantEntitiesInfo` 오류만
보고 user payload 전체가 실패했다고 결론 내리지 않는다.
