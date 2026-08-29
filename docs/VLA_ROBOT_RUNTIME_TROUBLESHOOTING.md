# VLA Robot Runtime Troubleshooting

HW validation에서는 iteration 속도를 engineering constraint로 취급하고 다음 순서로
진행한다.

```text
이 문서 검색
→ bringup
→ minimum readiness
→ test
→ result
```

이미 PASS한 Camera, HEF, Depth, TF 단계를 매번 장시간 재검증하지 않는다.
명확한 direct error부터 처리하고, known problem에는 documented recovery를 정확히 1회
적용한다. side-effect command가 SSH timeout을 냈다고 blind retry하지 않고 process
존재 여부를 한 번 확인한다. 정상 iteration은 수십 초~수분 안에 끝내며, recovery가
실패할 때만 새 root-cause 진단을 시작한다.
일시적 복구와 root cause 해결을 구분하며, 원인이 확정되지 않았으면 `미확정`으로
기록한다.

## Wi-Fi/Hotspot SSH command 실행 지연

### 증상

`lemma@10.42.0.1`에서 SSH 인증과 login banner는 성공하지만 command execution이
5~45초 지연되거나 멈춘 것처럼 보인다.

### 직접 오류

명시적인 SSH protocol 오류 없이 shell prompt 또는 command 결과가 늦게 반환된다.

### 원인

미확정. Wi-Fi/Hotspot transport 또는 중첩·잔존 SSH session과의 관련 가능성은 있으나
인과관계가 확정되지 않았다.

### 검증된 해결법

책상 위 development/diagnostics에서는 Ethernet `lemma@192.168.100.128`을 사용한다.

### 정상 기준

Ethernet ping 0% loss(평균 약 0.383 ms), SSH 명령 3회 연속 PASS, Pi host command와
`docker exec IntelPi` PASS.

### 예방

불필요한 병렬 SSH client를 만들지 않고 확보한 단일 session을 재사용한다.

### 미확정 사항

Wi-Fi/Hotspot 지연의 상세 trigger와 root cause는 확인되지 않았다.

## SSH timeout 후 duplicate bringup

### 증상

응답이 늦은 `ros2 launch`를 재전송한 뒤 VLA Orchestrator, Navigation Bridge, SLAM,
LD19 runtime이 각각 2개씩 실행된다.

### 직접 오류

SSH client timeout 또는 무응답처럼 보이는 상태이며, 첫 command의 실제 실행 여부는
확인되지 않는다.

### 원인

SSH timeout/slow response는 remote side-effect command가 실행되지 않았다는 보장이
아니다. 첫 launch가 늦게 실행된 상태에서 같은 launch를 재전송해 중복 runtime이
생성됐다.

### 검증된 해결법

재전송 전에 해당 process/runtime 존재 여부를 한 번 확인한다. 이미 실행 중이면
재전송하지 않고, 중복이 확인된 경우 이번 session에서 생성한 runtime만 식별해
정리한다.

### 정상 기준

각 authoritative runtime과 motion/goal owner가 정확히 1개다.

### 예방

launch, restart, goal 전송 같은 side-effect command는 timeout 후 blind retry하지
않는다.

### 미확정 사항

SSH 지연 자체의 root cause는 이 항목에서 다루지 않는다.

## `MACHINE_TYPE` runtime contract 누락

### 증상

production bringup이 시작 직후 종료된다.

### 직접 오류

```text
MACHINE_TYPE environment variable missing
```

### 원인

vendor launch가 요구하는 hardware identity 환경변수가 runtime shell에 없었다.

### 검증된 해결법

현재 검증된 Robot에서는 authoritative value `MentorPi_Mecanum`을 사용한다.

### 정상 기준

production bringup이 hardware identity 오류 없이 진행된다.

### 예방

동일 Robot에서는 확정값을 재추론하거나 `MentorPi_Acker`와 반복 비교하지 않는다.

### 미확정 사항

이 값은 현재 Robot 전용이며 다른 hardware에 그대로 적용하지 않는다.

## HEF는 실행되지만 `detections=[]`

### 증상

HEF inference는 실행되지만 person/fire detection이 0건이다.

### 기존 해결법

- `/home/lemma/Hailo/yolo26_split_test.py` 기준 output/postprocess를 사용한다.
- 최신 ROS overlay가 수정 source의 backend를 실제로 로드하는지 확인한다.

### 원인

실제 HEF는 단일 NMS output이 아니라 6개 raw neural head를 출력하므로
`best_sim_postprocess.onnx` 후처리가 필요하다. stale ROS installed backend가 수정
source 대신 로드된 경우도 있었다.

### 정상 기준

live person bbox/confidence 출력.

## LD19 `/scan_raw` 0 Hz와 TF timeout

### 증상

```text
/scan_raw 0 Hz
→ SLAM 갱신 중단
→ map→odom TF 중단
→ TF timeout/extrapolation
```

환경은 LD19, `/dev/ldlidar → /dev/ttyUSB0`, baudrate `230400`이다.
Issue #89에서는 LD19 serial timeout과 `/scan_raw` readiness failure도 관찰됐다.

### 직접 오류

`/scan_raw`가 0 Hz이거나 LD19 serial timeout이 발생한다.

### 검증된 해결법

serial 중복 점유 확인 후 LD19 driver를 완전히 종료하고 authoritative launch를 clean
restart한다. 이후 `/scan_raw`와 `map→odom` continuity를 확인한다.

authoritative launch는 `peripherals/launch/lidar.launch.py`가 포함하는
`ldlidar_LD19.launch.py`이다.

### 원인

intermittent parser/scan-assembly stall로 확인했으나 세부 trigger는 미확정이다.

### 정상 기준

`/scan_raw` 약 9.91 Hz로 35초 연속 LaserScan PASS, `map→odom` continuity PASS.

### 예방

restart command timeout 시 blind retry하지 않고 LD19 process 존재 여부를 한 번
확인한다. recovery가 PASS하면 추가 parser/serial 진단 없이 원래 test로 복귀한다.

### 미확정 사항

parser/scan-assembly stall의 상세 trigger는 확인되지 않았다. Issue #89의 serial
timeout이 동일 root cause였는지도 입증되지 않았다.

## Pi Docker process/exec resource exhaustion

### 증상

새 process/exec 실행이 불안정하거나 실패해 HW downstream test를 시작하지 못한다.

### 기존 해결법

1. 다른 팀원의 Pi/Docker 사용 여부를 확인한다.
2. 전체 runtime 종료 허가를 확인한다.
3. 실행 중 ROS2/test process를 정리한다.
4. 필요할 때 container만 clean restart한다.
5. 최소 stack으로 happy-path를 다시 실행한다.

container, image, volume, Hailo 환경을 삭제하지 않는다. clean restart 후에도 재발할
때만 resource 원인을 상세 진단한다.

### 원인

미확정.

### 정상 기준

새 process/exec가 안정적으로 실행되고 최소 HW stack이 정상 기동한다.

## Nav2 `ABORTED` 후 동일 goal 반복 dispatch

### 증상

Nav2가 terminal `ABORTED`를 반환한 뒤 같은 Mission의 동일 action/target navigation
goal이 다시 발행된다.

### 기존 해결법

Orchestrator가 `ABORTED` 결과를 WorldModel terminal lifecycle에 반영한 뒤 해당
`(mission_id, action_type, target_id)` semantic key를 유지하도록 한다. 새로운
Mission에서는 같은 target을 다시 실행할 수 있으며, 기존 `FAILED` retry semantics는
유지한다.

### 원인

terminal result 처리 시 `SUCCEEDED` 이외의 모든 상태에서 semantic duplicate key를
해제해 `ABORTED` 직후 같은 Qwen decision이 다시 dispatch될 수 있었다.

### 정상 기준

`ABORTED` terminal result 1회 반영, pending action 정리, 동일 Mission/target 추가
navigation dispatch 0건.

## Intel XPU device loss로 Remote Qwen HTTP 503

### 증상

Intel Arc B580, `xe` driver, `torch 2.7.1+xpu` 환경에서 Qwen endpoint가 HTTP 503을
반환하고 `UR_RESULT_ERROR_DEVICE_LOST`를 기록한다. Qwen만 clean restart해도
`UR_RESULT_ERROR_UNKNOWN`이 이어질 수 있다.

### 검증된 해결법

Qwen을 반복 restart하지 않는다. XPU probe를 먼저 실행하고 device runtime이
복구되지 않았으면 PC를 clean reboot한다. reboot 후 XPU probe PASS를 확인하고
authoritative Qwen server를 한 번 clean start한다.

### 원인

교육장 PC Intel XPU runtime/driver state loss. 세부 trigger는 미확정이다.

### 정상 기준

XPU probe PASS 후 authoritative Qwen inference endpoint HTTP 200.

## `/suppress_fire` action server가 보이지 않음

### 증상

`fire_extinguisher.launch.py` 실행 중 `lgpio`의 `.lgd-nfy*` 생성 오류가 나거나,
launch가 준비 메시지를 출력했는데 `/fire_suppression_node`와 `/suppress_fire` action
server가 ROS graph에 보이지 않는다.

### 검증된 해결법

새 GPIO workaround를 만들지 않는다. `IntelPi` container의 기본 사용자 `root`, 작업
디렉터리 `/`, production source 순서와 `ROS_DOMAIN_ID=42`,
`ROS_LOCALHOST_ONLY=0`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`를 사용해
`ros2 launch uncc_example fire_extinguisher.launch.py`를 정확히 한 번 시작한다.
`lgpio` notification FIFO는 `/.lgd-nfy0`에 생성된다.

### 원인

성공한 Hardware runtime과 다른 container 사용자 또는 작업 디렉터리에서 실행해
`lgpio` notification FIFO 경계가 달라진 것. 별도 코드 결함은 확인되지 않았다.

### 정상 기준

`/fire_suppression_node`와 `/vla_spray_bridge`가 각각 1개이고,
`/suppress_fire` action server가 정확히 1개다. readiness 확인에는 Pump goal을 보내지
않는다.

## Suppression launch 직후 Servo 움직임 또는 지속 소음

### 증상

Pump goal이 0인데 `fire_extinguisher.launch.py` 시작 직후 Servo가 움직이거나 계속
웅- 소리를 낸다.

### 원인 및 검증된 해결법

`AngularServo(initial_angle=90)`은 노드 생성 시 즉시 중앙각 PWM을 출력한다. 교체한
Servo의 혼/노즐 하중 또는 실제 중심과 PWM 보정 차이가 있으면 시작 동작이나 지속
소음이 발생할 수 있다. Production node는 `initial_angle=None`으로 시작해 readiness
중 PWM을 detach 상태로 유지한다. 실제 `/suppress_fire` goal의 첫 angle 명령에서만
PWM을 활성화하고, 완료 시 중앙 복귀 후 다시 detach한다.

### 정상 기준

Suppression launch와 action server readiness만으로 Servo가 움직이거나 계속 energized
되지 않는다. Pump goal은 보내지 않는다. 실제 goal 중 지속 소음이 있으면 mechanical
stop, nozzle/hose load, center/pulse calibration을 확인한다.

## Ethernet 제거 후 VLA/Nav/Suppression graph 이탈

### 증상

process는 살아 있지만 Ethernet 제거 후 VLA navigation client와 `/suppress_fire`
server가 ROS graph에서 사라진다. ROS daemon refresh만으로 복구되지 않을 수 있다.

### 검증된 해결법

Wi-Fi/Qwen 연결을 확인하고 `topic_bridge_vla.launch.py`,
`vla_navigation_bridge.launch.py`, `fire_extinguisher.launch.py`만 clean-stop한다.
Wi-Fi가 활성화된 상태에서 동일 root + `/` production 계약으로 정확히 한 번 재시작한
뒤 ROS daemon을 refresh한다. Base/Camera/YOLO/SLAM/Nav2는 재시작하지 않는다.

### 원인

Fast DDS participants가 시작 당시 Ethernet interface를 유지해 network transition 뒤
graph에서 이탈했다.

### 정상 기준

NavigateToPose server/client 각각 1, `/suppress_fire` server/client 각각 1,
Pi→현재 Qwen endpoint HTTP 200, Pump goal 0.

## Remote Qwen timeout 또는 HTTP 503과 XPU allocator 오류

### 증상

Wi-Fi VLA에서 `Remote Qwen timed out`이 발생하거나 HTTP 503 후 Qwen 종료 시
`invalid device pointer`와 `XPUCachingAllocator`가 출력된다. 단순 XPU probe는 PASS할
수 있다.

### 검증된 해결법

VLA launch에 `remote_qwen_timeout_sec:=10.0`을 포함한다. HTTP 503과 allocator 오류가
발생하면 Qwen을 반복 restart하지 않고 PC를 clean reboot한다. reboot 후 XPU probe,
authoritative Qwen clean start, HTTP 200 순서로 확인한다.

### 원인

Intel Arc B580 XPU runtime 손상 계열. 세부 trigger는 미확정이다.

### 정상 기준

XPU probe PASS, Qwen inference HTTP 200. Endpoint IP는 현재 PC network에서 확인한다.

## `/vla/mission` one-shot이 subscriber에 전달되지 않음

### 증상

CLI가 `publishing #1`을 출력했지만 WorldModel mission은 null이고 navigation goal은
0건이다.

또는 `/vla/mission`에 일반 문자열을 넣으면 Orchestrator의 JSON parsing contract를
통과하지 못해 동일 증상이 발생한다.

### 검증된 해결법

payload가 `{"mission_id":"mission_fire_001","text":"화재를 찾아 진압해줘"}` 형식인지
먼저 확인한다. WorldModel과 goal 0건으로 semantic non-delivery를 확인한 뒤
`ros2 topic pub --once -w 1 /vla/mission ...`으로 전달한다. rosbag recorder도
subscriber 수에 포함될 수 있으므로 수신 성공은 subscriber 수가 아니라 WorldModel의
`mission.id/text` 갱신으로 확인한다. 확인 없이 blind retry하지 않는다.

### 정상 기준

WorldModel mission이 갱신되거나 후속 VLA decision이 시작된다.

## branch/build/install 충돌 방지: VLA workspace 격리

### 증상

한 workspace에서 `integration/vla-robot-e2e`와 `state_manage`를 번갈아 사용해
source와 stale `build/`/`install/` metadata가 서로 섞인다.

### 검증된 예방

VLA는 `/ros2_ws/phoenix_vla`의 `integration/vla-robot-e2e`만 사용하고, 팀/rule-based
작업은 기존 `/ros2_ws/phoenix`에 둔다. 두 workspace 사이에서 branch를 전환하거나
`build/`, `install/`, `log/`를 복사/source하지 않는다. Build는 normal user로 실행하고
`sudo colcon build`를 사용하지 않는다. Suppression runtime의 root + `/` 계약은 build
user 계약과 별개다.

### 정상 기준

VLA package prefix가 모두 `/ros2_ws/phoenix_vla/install`을 가리키고 기존
`/ros2_ws/phoenix/install`이 VLA shell의 prefix path에 없다.

## 새 항목 작성 형식

새 HW 오류가 실제로 발생하고 해결됐을 때만 아래 형식으로 추가한다.

```text
### 증상
무엇이 안 되는지

### 기존 해결법
가장 빠른 복구 방법

### 원인
확정된 원인 또는 미확정

### 정상 기준
재사용 가능한 최소 정상 지표
```

긴 실행 로그, 디버깅 일지, 실패한 명령어 목록은 남기지 않는다.
## VLA만 restart 후 이전 navigation result가 새 fire action에 재사용됨

### 증상

VLA Orchestrator만 restart한 뒤 새 `NAVIGATE_TO fire_*`가 결정됐지만 Nav2에
새 goal이 제출되지 않고 이전 run의 `action_0001` terminal result가 반영된다.

### 원인

Orchestrator restart로 action ID는 `action_0001`부터 다시 시작하지만 기존
`vla_navigation_bridge`의 `completed_results` cache는 이전 result를 유지한다.
Bridge는 동일 ID의 새 goal 대신 cached result를 재발행한다.

### 검증된 해결법

Stale mission을 폐기하며 VLA Orchestrator를 clean restart할 때
`vla_navigation_bridge.launch.py`도 함께 clean-stop하고 두 launch tree를 각각
정확히 1회 restart한다. Base/Nav2는 restart하지 않는다.

### 정상 기준

새 `action_0001` fire goal 1회, 실제 Nav2 path와 `/cmd_vel`, terminal result가
생성된다.

## Qwen timeout 뒤 동일 inference가 중첩됨

### 증상

Remote client가 timeout된 직후 같은 WorldModel로 재호출한다. 이전 HTTP request는
client 연결 종료 뒤에도 XPU generation을 계속할 수 있다.

### 원인

`ThreadingHTTPServer`가 같은 model의 concurrent request를 허용했고 inference
failure가 decision signature를 소비하지 않아 timer가 동일 입력을 재호출했다.

### 검증된 해결법

Server inference를 single-flight로 제한하고 inference failure signature를 기록한다.
의미 있는 WorldModel 변화가 있을 때만 다음 decision을 실행한다. HTTP 503 body를
확인해 XPU 오류와 truncated JSON을 구분한다. 32/48 tokens의 truncated JSON에는
64 tokens를 사용한다.

### 정상 기준

동시 backend inference 최대 1, 동일-state retry 0, 64-token fixture timeout/HTTP/
schema failure 0.

## ASCamera process는 있지만 image/CameraInfo가 발행되지 않음

### 증상

- Camera parent/child process는 살아 있고 SDK 초기화 뒤 stream 시작 로그가 없다.
- RGB/Depth/CameraInfo publisher는 0이고 Detection3D는 `waiting_camera_info`다.

### 확인된 원인과 최소 복구

2026-08-27 HW E2E에서는 Pi OS와 IntelPi 양쪽에서 ASCamera USB 장치가 보이지
않았다. ROS topic/remap, Docker device 노출, 코드가 아니라 물리 USB·전원 연결
문제였다.

Production runtime을 안전하게 종료하고 카메라 전원·USB 연결을 한 번 확인·재연결한
뒤 Pi OS, IntelPi 순서로 장치 인식을 확인한다. Authoritative environment와 install
overlay로 production runtime을 정확히 1회 clean start한다. 정상 복구 시 양쪽에서
`3482:6723 NOVATEK ASJ ZNX_NVT`가 보이고 RGB/Depth/CameraInfo가 발행된다.

## 중복 production runtime과 간헐적 perception stall

### 증상과 판정

`ascamera_node` 3개와 Base/Camera/VLA process 여러 세대가 남아 있었고, 입력
counter가 증가하는 중 Detection3D가 간헐적으로 `stalled`가 됐다. 반복 launch와
불완전한 종료에 따른 runtime 오염이며 이 상태만으로 코드 회귀를 확정하지 않는다.

### 최소 복구

관련 production process만 안전하게 clean stop하고 active production process 0개를
확인한다. Unrelated user process는 종료하지 않는다. Authoritative environment와
`/ros2_ws/phoenix_vla/install` overlay로 정확히 1회 clean start한다.

## Detection3D가 입력 처리 중 간헐적으로 stalled

### 확인된 값과 설정

- RGB 3.05 Hz, Depth 8.92 Hz, CameraInfo 12.03 Hz, YOLO 1.62 Hz
- WorldModel 1.00 Hz, `frames` 107 → 143, `published` 22 → 41
- 상태 `ok` → 일시적 `stalled` → `ok`, `last_frame_age` 2.499초
- `ApproximateTimeSynchronizer`, `queue_size=30`, `slop_sec=0.05` (50 ms)
- `stall_after_sec=1.0`
- YOLO는 원본 RGB header timestamp를 유지하며 Depth와 같은 ROS clock epoch를 쓴다.

### 판정

입력과 callback의 영구 정지가 아니다. 느리고 불규칙한 YOLO와 1초 stall 판정으로
상태가 일시적으로 바뀔 수 있다. 임의로 측정한 YOLO–Depth 차이 397.3 ms는 50 ms
tolerance 밖이지만 synchronizer가 선택하려던 대응 pair라는 근거가 없으므로 이 값
하나로 sync 실패를 확정하지 않는다. `frames`와 `published`가 증가하고 fresh
observation이 생성되면 일시적 `stalled`만으로 E2E를 중단하지 않는다. 현재
코드·threshold 변경 근거는 없다.

## Fire detection은 있지만 Depth가 unknown

### 첫 시험

- fire confidence 최대 0.582로 production threshold 0.60 미달
- `depth: null`, `depth_status: unknown`, map position 없음
- WorldModel `fires=[]`

### 확인된 원인과 최소 복구

불꽃 자체의 Depth가 불안정했고 fire sampling 영역에 거리 측정 가능한 고체 표면이
없었다. 불 OFF 상태에서 불꽃 아래 불연성 고체 받침과 거리 측정 가능한 배경을
확보한다. 반사·투명 표면을 피하고 표적을 화면 중앙, 카메라에서 약 0.5~0.7 m에
둔다. 코드나 threshold는 바꾸지 않는다.

### 확인된 정상 결과

- YOLO confidence 0.729
- Detection3D depth 0.634 m, `depth_status: fallback_below`
- map position 약 `(0.470, -0.514)`
- WorldModel `fire_0023 ACTIVE`, confidence 0.659
- `FRESH_FIRE_OBSERVATION: PASS`, `READY_FOR_MISSION: YES`

## rosbag이 live perception에 영향을 주는지 구분

`ros2 bag record/play` process, recorder/player node, production launch 자동
실행 여부, live topic의 playback publisher를 확인한다. 2026-08-27에는 record/play
process가 없고 rosbag node는 0개였으며 자동 실행도 없었다. Live RGB/Depth
publisher는 ASCamera가 각각 1개였다. 당시 stalled/Detection3D 문제와 rosbag은
무관했다. Rosbag은 재현·분석 도구이며 live HW E2E 필수 구성요소가 아니다.

## 원격 ROS 조회 무응답과 PC–Pi 경로 단절

### 증상과 판정

원격 ROS 조회가 50초 이상 무응답이었고 이후 SSH `No route to host`가 발생했다.
원격 조회 실패와 Pi 내부 runtime 실패를 구분한다. ICMP ping만으로 단절을 확정하지
않고 SSH 경로도 확인한다. PC–Pi 경로가 없으면 PC Qwen HTTP도 불가능하므로 E2E
전에 복구한다. Pi 내부 runtime 상태는 별도 확인 전까지 `unknown`이다.

### Power-save 확인

NetworkManager `wifi.powersave=2`는 기존부터 OFF였다. wlan0/wlan1 runtime power
control은 `auto`에서 `on`으로 임시 변경했으며 재부팅 시 초기화될 수 있다. Pi
자체 전원 종료는 Wi-Fi power-save로 단정하지 않고 배터리·전원 공급·케이블을 먼저
확인한다.

## Fire-only Hardware E2E 운영 절차

### Hardware E2E 판정 규칙

1. 모든 점검 결과를 `PASS`, `FAIL`, `UNKNOWN`으로 구분한다.
2. 조회 timeout, SSH 실패, ROS CLI 오류는 production `FAIL`이 아니라
   `UNKNOWN`이다.
3. ERROR 한 줄만 보지 않고 바로 뒤 fallback과 정상 진행 여부를 확인한다.
4. 기능 미동작과 독립적인 직접 증거가 있을 때만 실제 `FAIL`로 판정한다.
5. 센서 topic은 호환 QoS의 직접 subscriber로 확인한다.
6. 원인 미확정 상태에서 runtime restart, USB·port·power 변경을 반복하지 않는다.
7. 복구는 확인된 원인에 대응하는 절차를 정확히 1회 적용한다.
8. 실제 데이터가 지속되는 순간 `stalled`, 단일 timeout, ping 실패만으로 E2E를
   중단하지 않는다.
9. 실제 safety, Mission, Nav2, Robot stop, actuator 실패에서는 즉시 중단한다.
10. 사전 점검이 PASS하면 terminal SUCCESS까지 불필요한 중간 진단을 하지 않는다.

### ASCamera `getMjpegSize` 판정

`get mjpeg from xu command failed`는 단독 fatal error가 아니다. 2026-08-27 오전
정상 runtime과 오후 runtime 모두에서 다음 순서가 같았다.

```text
getMjpegSize ERROR
→ mjpeg size:640x480 fallback
→ camera opened
→ depth/rgb 640x480 @ 15fps
→ start streaming
```

이 로그만으로 Camera FAIL, vendor SDK blocker, launch resolution mismatch, Camera
hardware failure를 판정하지 않는다.

과거 12 Mbps USB negotiation 문제와 현재 incident도 구분한다. 12 Mbps이면 물리
연결을 복구해 480 Mbps를 확인한다. 현재 incident는 480 Mbps이고 vendor runtime
종료 상태의 표준 UVC MJPEG `640x642 @ 30fps` 5-frame capture가 PASS했다.
따라서 현재 문제를 USB bandwidth 문제로 분류하지 않는다.

현재 incident에서 확인된 범위는 다음과 같다.

- `ascamera_node`: active 1
- 예상 RGB/Depth/CameraInfo/PointCloud publisher endpoint: 각 1
- 다른 RGB topic: 없음
- publisher QoS: reliable, volatile
- sensor-data QoS 직접 RGB subscriber: 10초간 0건
- Camera log: `start streaming`과 단일 `streamCallback set gain ret 0`
- 반복 frame callback/publish counter: 확인 불가
- process CPU ticks: 3초 동안 55 증가. Frame 처리 성공의 증거로 사용하지 않음

Publisher endpoint는 존재하지만 실제 frame callback 진행을 확인할 counter가 없어
현재 상태는 `UNKNOWN`이다. 정확한 실패 경계도 vendor frame acquisition과 ROS
publish 사이에서 미확정이다.

### 불 OFF 준비

1. 물리 연결과 배터리를 확인한다. 책상 위나 충전선 연결 상태에서는 motion 금지다.
2. PC–Pi–IntelPi–Qwen 경로를 확인한다.
3. Production runtime은 없거나 오염된 경우에만 정확히 1회 clean start한다.
4. RGB/Depth/YOLO/Detection3D/WorldModel/Nav2/Suppression/Qwen을 최대 15초의
   통합 preflight 한 번으로 확인한다.

### `READY_FOR_E2E_FIRE` 이후

사용자에게 불 ON을 요청하고 fresh fire observation을 생성한다. 중간 readiness
조회로 흐름을 끊지 않고 정상 Mission JSON → Qwen → Nav2 → Robot stop →
Servo/Pump → 화염 제거 → terminal SUCCESS를 연속 실행한다.

### 즉시 중단 조건

- 제한시간 안에 fresh fire observation이 생성되지 않음
- Qwen Mission 호출 실제 실패
- Nav2 실패 또는 위험 상태
- Robot stop 확인 실패
- Servo/Pump action 실패
- 물리적 안전 위험

실제 데이터가 지속되는 상황의 일시적 Detection3D `stalled`, 단일 상태 조회
timeout, SSH 조회 한 번 실패, ICMP ping만 실패한 경우는 단독 중단 조건이 아니다.

### 안전 원칙

- 불을 켜기 전에 readiness를 완료하고 실제 주행 전 충전선·전원선을 분리한다.
- Robot 위치가 바뀌면 이전 fire observation과 map 좌표를 폐기한다.
- 불 OFF 상태에서는 Mission/HW 명령을 0으로 유지한다.
- 첫 실제 실패 뒤 actuator 동작을 중단하되 직접 원인, 미확인 부분, 다음 최소 조치를
  설명한다.

## Sensor observation과 fire confidence 판정

- ROS CLI daemon의 `topic echo` 결과만 센서 PASS/FAIL 근거로 사용하지 않는다.
  Sensor topic은 publisher와 호환되는 직접 subscriber로 확인한다.
- `/fire/detections`와 `/fire/detections/status`는 BEST_EFFORT
  `qos_profile_sensor_data`로 관찰한다. RELIABLE subscriber의 QoS incompatibility는
  perception failure가 아니라 observer 설정 오류다.
- 조회 timeout이나 SSH/ROS CLI 오류는 `UNKNOWN`이다. 실제 메시지 0건이 독립적으로
  확인되기 전에는 production `FAIL`로 바꾸지 않는다.
- Camera 초기화와 메시지 관찰에 충분한 시간을 둔다. 10~15초 단일 timeout만으로
  Camera hardware failure를 확정하지 않는다.
- `getMjpegSize` ERROR 뒤 `640x480` fallback, `camera opened`, `start streaming`이
  이어지면 그 ERROR는 비치명적이다.

2026-08-27 오후 fire 관찰에서는 YOLO fire와 Detection3D depth가 계속 생성됐고
Detection3D는 일시적 `stalled` 뒤 다시 `ok`로 회복했다. 최고 confidence `0.5817`,
depth 약 `1.53 m`, `depth_status=fallback_below`였다. Production threshold `0.60`을
넘지 않아 WorldModel ACTIVE fire가 등록되지 않았다. 이는 timeout이 아니라 실제
confidence gate 실패다.

다음 시험에서는 오전 성공 조건인 화면 중앙 `0.5~0.7 m`,
불연성 고체 받침, 거리 측정 가능한 배경을 복원한다. Confidence가 충분한 연속
detection에서 명백히 threshold 아래로 안정되면 60초를 채우기 위해 불을 유지하지
않는다. Threshold `0.40`은 사용자 승인으로 `5717087`에서 적용했다. 지속 검출+유효
Depth 정책은 `NOT_APPLIED`다. Spray range `0.8 m`와 다른 safety gate는 유지한다.

## Fire 방향은 정면이지만 Nav2 plan 전 localization yaw가 변함

증상: 정지 상태 Test 1에서 robot pose yaw `110.6°`, fresh fire relative bearing
`+4.0°`로 실제 정면과 일치했다. 불 OFF 후 Test 2 직전 yaw가 `64.0°`로 바뀌었고,
동일 fire 좌표의 `ComputePathToPose`는 path를 생성하지 못했다.

판정: Fire Detection3D/map 방향은 PASS다. 정지 상태에서 약 `46.6°` 바뀐
localization/TF heading이 첫 실패 계층이다. 정면 local costmap lethal cell은
`0/25`였지만 이 값만으로 planner 원인을 확정하지 않는다.

복구: 이전 fire/Mission/goal을 폐기하고 불 OFF에서 localization heading 안정성을
먼저 복구·확인한다. Heading과 fresh fire bearing이 일치하고 plan이 생성된 뒤에만
실제 Nav2 goal을 1회 보낸다. `NavigateToPose`, Mission, Servo/Pump는 이 incident에서
발행하지 않았다.

## Pi 전원 종료를 network/runtime failure와 구분

증상: PC는 `uncc_router`와 Qwen HTTP 200을 유지했지만 Pi SSH가 `No route to host`를
반환했다. 확인 결과 Pi 전원이 실제로 꺼져 있었다.

판정: 이 incident는 NetworkManager나 Qwen failure가 아니다. 배터리 부족 또는 Motor
부하 중 voltage drop은 직접 원인 후보지만 아직 미확정이다. Phoenix low-battery
동작은 Pi poweroff가 아니라 `RETURNING_TO_BASE`이므로 서로 구분한다.

복구: 불 OFF, Mission/HW command 0에서 충분히 충전한다. 충전 완료 후 전원선을
분리하고 새 안전 위치에 배치한다. PC→Pi SSH와 Pi→Qwen HTTP를 한 번 확인한 뒤에만
canonical production stack을 시작한다.

## Hardware E2E 운영 판정

- 조회 실패, 단일 timeout, 순간 `stalled`를 production FAIL로 바꾸지 않는다.
- ERROR 한 줄 뒤 fallback과 정상 진행 여부를 확인한다.
- 원인 미확정 상태에서 USB, 전원, runtime restart를 반복하지 않는다.
- 불 ON 후 confidence가 threshold 아래로 안정되면 즉시 불을 끈다.
- E2E를 별도 geometry/planner dry-run으로 나누지 않는다.
- 시작 전 network와 localization을 한 번 확인하고 terminal까지 연속 실행한다.
- Active Mission 중 Robot을 손으로 옮기지 않는다.
- 책상 위 또는 전원선 연결 상태에서는 motion을 실행하지 않는다.

## Remote Qwen timeout과 out-of-range action 처리

- `REPORT_PERSON` 판단은 약 `2.36 s`에 성공했다. Production VLA launch에는
  `remote_qwen_timeout_sec:=10.0`을 명시한다.
- `REPORT_PERSON` terminal result 미반영은 별도 known lifecycle issue다.
- Qwen이 유효한 분사거리 밖 ACTIVE fire에 `EXTINGUISH`를 반환하면 기존 Validator가
  거절하고 Mission이 진행되지 않았다.
- `2b5ee9a`부터 Validator 제출 전에 같은 target의 `NAVIGATE_TO`로 한 번만 교정한다.
  Stale/invalid/inactive target과 다른 safety gate는 기존대로 차단한다.

## 점화 작업자의 person observation 회피

Fire-only 검증에서 점화 작업자가 WorldModel person으로 남으면 Qwen이
`REPORT_PERSON`을 우선할 수 있다. 불 OFF에서 VLA Orchestrator와 Navigation Bridge만
정지하고, 점화 후 작업자가 화각에서 빠진 다음 두 component를 한 번 시작한다. 새
WorldModel에서 `people=0`, fresh ACTIVE fire, `current_action=null`을 확인한 뒤 새
Mission ID를 한 번만 사용한다. Camera, YOLO, Detection3D, Nav2, Suppression server는
이 절차 때문에 재시작하지 않는다.

## 간헐적 PC–Pi SSH 단절

증상: SSH가 정상 접속된 뒤 reset, timeout 또는 `No route to host`로 바뀔 수 있다.
2026-08-27 관측값은 ping `85–125 ms`, packet loss `33%`였다.

판정: SSH 조회 실패만으로 Pi production runtime failure나 종료 성공을 확정하지
않는다. Side-effect 명령 timeout 뒤 runtime을 중복 시작하지 않는다.

복구: 불 OFF에서 Pi 전원·배터리와 SSH 실제 명령 경로를 먼저 확인한다. 접속되면
active production process와 Mission/Nav2/Suppression 상태를 한 번 확인해 살아 있는
runtime은 재사용한다. Process가 없을 때만 canonical clean start를 한 번 수행한다.
접속되지 않으면 remote stop 상태는 `UNKNOWN`으로 남긴다.

## 팀 branch 변경 선별 통합

2026-08-29 remote branch read-only audit에서는 즉시 merge할 변경을 확정하지 않았다.

- `albitro/image_opt@bfafdad`, `6ccf370`: Hailo 입력 변환과 thread 최적화 후보지만
  production Pi의 동일 출력과 latency/CPU 측정 전까지 보류한다.
- Nav2/SLAM parameter 변경: 실제 관련 오류가 재현되고 HW 증거가 있을 때만 검토한다.
- `fire_service_v2@fc060a5`: Frontier 기반 fire-status/suppression owner를 별도로
  도입하므로 현재 VLA suppression owner와 함께 사용하지 않는다.

Branch 전체 merge나 commit message만으로 복구를 시도하지 않는다. 현재 schema,
Mission scope, 정상 Qwen 1회, Validator와 suppression verification 계약을 기준으로
실제 증거가 있는 최소 commit만 선별한다.
