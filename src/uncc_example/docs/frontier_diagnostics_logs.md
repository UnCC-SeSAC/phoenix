# Frontier Diagnostics 로그 가이드

이 문서는 `frontier_diagnostics` 노드가 **언제**, **어떤 로그를**, **어떤 심각도**로 출력하는지 설명한다. 이 노드는 진단 전용이며 navigation goal이나 velocity 명령을 보내지 않는다.

## 실행

```bash
colcon build --packages-select uncc_example --symlink-install
source install/setup.bash
ros2 launch uncc_example frontier_diagnostics.launch.py
```

기본 설정은 `config/frontier_diagnostics.yaml`에 있다. 기본값인 `anomaly_only_logging: true`에서는 시작, 상태 변경, 이상 발생, 이상 반복, 복구만 출력한다.

## 로그 출력 방식

| 형식 | 의미 | 반복 방식 |
|---|---|---|
| 일반 시작 로그 | 노드 시작과 주요 설정 확인 | 시작할 때 한 번 |
| `[STATE]` | frontier 또는 NavigateToPose 상태 변경 | 상태가 바뀔 때마다 |
| `[ANOMALY]` | 새로운 이상 발생 | 처음 감지할 때 즉시 |
| `[ANOMALY_REPEAT]` | 동일한 지속형 이상이 계속됨 | 기본 10초마다 |
| `[RECOVERED]` | 지속형 이상 조건이 사라짐 | 복구 시 한 번 |
| `[HEARTBEAT]` | 현재 goal과 활성 이상 목록 | `anomaly_only_logging: false`일 때 진단 주기마다 |

일회성 이벤트형 이상은 `[ANOMALY]`만 출력하며 같은 `type`은 `event_cooldown_s` 동안 다시 출력하지 않는다. 이벤트형에는 별도의 `[RECOVERED]`가 없다.

지속형 이상은 처음에 `[ANOMALY]`, 계속되면 `event_cooldown_s`마다 `[ANOMALY_REPEAT]`, 조건이 사라지면 `[RECOVERED]`를 출력한다.

기본 진단 주기는 1초이고, 기본 cooldown은 10초다.

## 항상 출력되는 정상 상태 로그

### 노드 시작

노드 생성 직후 INFO로 한 번 출력된다.

```text
Frontier diagnostics started: anomaly_only=True history=10.0s cooldown=10.0s (read-only; no navigation or velocity goal is sent)
```

### 새로운 frontier 수신

`/explore/selected_frontier`에서 새 좌표를 받거나 `frontier_explorer` 로그의 `dispatch=(x,y)`를 인식하면 INFO로 출력된다. 이전 frontier와 거리가 0.01 m 미만이면 같은 목표로 간주해 다시 출력하지 않는다.

```text
[STATE] new frontier source=selected_frontier_topic frame=map xy=(1.230, 0.450)
```

`source`는 보통 다음 중 하나다.

- `selected_frontier_topic`: frontier topic에서 수신
- `frontier_explorer_rosout`: `frontier_explorer`의 전송 로그에서 좌표 추출

### NavigateToPose 상태 변경

`/navigate_to_pose/_action/status`의 최신 상태가 바뀌면 INFO로 출력된다.

```text
[STATE] navigate_to_pose EXECUTING -> SUCCEEDED active=False
```

표시 가능한 상태는 `UNKNOWN`, `ACCEPTED`, `EXECUTING`, `CANCELING`, `SUCCEEDED`, `CANCELED`, `ABORTED`다.

## ROS 로그 감시로 발생하는 이상

다음 logger의 `/rosout`만 감시한다.

- `frontier_explorer`
- `planner_server`
- `controller_server`
- `bt_navigator`
- `behavior_server`
- `global_costmap`
- `local_costmap`
- `slam_toolbox`
- `velocity_smoother`

logger 이름은 위 이름으로 끝나면 감시 대상에 포함된다. 진단 노드 자신이 출력한 로그는 재감지하지 않는다.

| `type` | 심각도 | 발생 조건 |
|---|---:|---|
| `EMPTY_PATH_LOG` | ERROR | 메시지에 `path is empty` 또는 `empty path` 포함 |
| `INVALID_PATH_LOG` | ERROR | 메시지에 `invalid path` 또는 `no valid path` 포함 |
| `GOAL_FAILED_LOG` | ERROR | 메시지에 `goal failed` 또는 `goal aborted` 포함 |
| `UNEXPECTED_NODE_HALT` | WARN | 메시지에 `halt` 포함, 예상된 cancel 보호 시간 밖 |
| `ROS_ERROR` | ERROR | 위의 구체적인 문자열에 해당하지 않는 ROS ERROR 이상 로그 |
| `MATCHED_WARN` | WARN | ROS WARN 이상이며 아래 키워드 중 하나를 포함 |

`MATCHED_WARN` 키워드는 다음과 같다.

```text
failed, failure, invalid, empty, no valid, no path, blocked, stuck,
timeout, aborted, transform, extrapolation, collision, halt
```

구체적인 문자열 판정이 일반 `ROS_ERROR`보다 먼저 적용된다. 예를 들어 ERROR 메시지에 `Invalid path`가 있으면 `ROS_ERROR`가 아니라 `INVALID_PATH_LOG`로 출력된다.

`frontier_explorer` 로그에 다음 문자열이 나타난 후 5초 동안의 halt/cancel은 목표 교체 등에 따른 정상 동작으로 보고 억제한다.

```text
preempt, goal replacement, control stop, skipping blocked frontier
```

## Navigation goal 및 path 이상

| `type` | 종류 | 심각도 | 발생 조건 |
|---|---|---:|---|
| `GOAL_ABORTED` | 이벤트 | ERROR | NavigateToPose action status가 `ABORTED`로 변경 |
| `UNEXPECTED_CANCEL` | 이벤트 | WARN | action status가 `CANCELED`이며 예상 cancel 보호 시간 밖 |
| `NAV2_PATH_SERVER_UNAVAILABLE` | 지속 | WARN | `/compute_path_to_pose` action server를 사용할 수 없음 |
| `NAV2_PATH_REQUEST_FAILED` | 이벤트 | ERROR | ComputePathToPose goal 요청 중 예외 발생 |
| `NAV2_PATH_REJECTED` | 이벤트 | ERROR | ComputePathToPose server가 요청을 거부 |
| `NAV2_PATH_RESULT_FAILED` | 이벤트 | ERROR | ComputePathToPose 결과 수신 중 예외 발생 |
| `NAV2_PATH_INVALID` | 지속 | ERROR | 결과 status 실패, error code 비정상 또는 path pose가 0개 |
| `LOCAL_PATH_EMPTY` | 지속 | ERROR | goal 활성 중 `/local_plan`의 pose가 0개 |
| `TRANSFORMED_GLOBAL_PATH_EMPTY` | 지속 | ERROR | goal 활성 중 `/transformed_global_plan`의 pose가 0개 |

ComputePathToPose 검사는 `compute_path_enabled: true`일 때 새 frontier마다 한 번 요청한다. 이전 frontier에 대한 늦은 결과는 `stale=True`로 기록하고 `NAV2_PATH_INVALID` 판정에서는 제외한다.

## 이동 및 controller 이상

### `NO_PROGRESS` — ERROR, 지속형

다음 조건을 모두 만족할 때 발생한다.

- goal이 활성 상태
- 목표까지 남은 거리가 0.20 m보다 큼, 또는 feedback 거리 정보가 없음
- 기본 8초 동안의 이력이 확보됨
- 해당 시간 동안 목표 거리 감소량이 기본 0.05 m 미만

기본 detail 예시:

```text
goal progress 0.012m over 8.0s; minimum 0.050m
```

### `ROBOT_STUCK` — ERROR, 지속형

다음 조건을 모두 만족할 때 발생한다.

- goal이 활성 상태
- 기본 3초 동안 controller 명령이 한 번 이상 `linear >= 0.03 m/s` 또는 `angular >= 0.10 rad/s`
- 같은 시간 동안 로봇 위치 변화가 0.02 m 미만
- yaw 변화가 0.05 rad 미만

즉, controller는 움직이라고 명령하지만 TF로 확인한 로봇 pose가 거의 변하지 않는 상태다.

### `CONTROLLER_ZERO_COMMAND` — WARN, 지속형

다음 조건을 모두 만족할 때 발생한다.

- goal이 활성 상태
- 목표까지 남은 거리가 0.20 m보다 큼, 또는 feedback 거리 정보가 없음
- 기본 3초 동안 모든 controller 명령이 `linear < 0.03 m/s` 및 `angular < 0.10 rad/s`

`/cmd_vel_nav`를 controller 출력으로 사용한다. velocity smoother를 지난 최종 값 `/cmd_vel`도 스냅샷에 함께 표시하지만 zero-command 판정에는 controller 출력을 사용한다.

## 데이터 및 TF 이상

| `type` | 심각도 | 발생 조건 |
|---|---:|---|
| `DATA_STALE` | WARN | goal 활성 중 필수 데이터가 없거나 기본 3초보다 오래됨 |
| `TF_UNAVAILABLE` | ERROR | 필요한 point 또는 robot pose TF 변환 실패 |

`DATA_STALE`은 노드 시작 후 기본 15초의 grace period가 지난 뒤부터 검사한다. 검사 대상은 다음과 같다.

- `/map`
- `/global_costmap/costmap`
- `/local_costmap/costmap`
- `/odom`
- `/scan_raw`

goal이 비활성 상태이거나 startup grace period 중이면 두 조건을 해제한다.

## map/costmap 크기 변경

| `type` | 종류 | 심각도 | 발생 조건 |
|---|---|---:|---|
| `MAP_RESIZE` | 이벤트 | WARN | map의 frame, width, height, resolution 또는 origin 변경 |
| `GLOBAL_RESIZE` | 이벤트 | WARN | global costmap의 frame, width, height, resolution 또는 origin 변경 |
| `LOCAL_RESIZE` | 이벤트 | WARN | local costmap의 frame, width, height 또는 resolution 변경 |

local costmap은 rolling window가 로봇을 따라 움직이면서 origin이 계속 변하는 것이 정상이므로, **local costmap의 origin 변화만으로는 로그를 출력하지 않는다**. resize 판정은 goal 활성 여부와 관계없이 grid geometry가 이전 메시지와 달라지는 순간 발생한다.

detail에는 변경 전후 geometry tuple이 표시된다.

```text
trigger=('map', 400, 400, 0.05, -10.0, -10.0) -> ('map', 500, 500, 0.05, -12.5, -12.5)
```

## frontier 유효성 이상

### `FRONTIER_INVALID` — ERROR, 지속형

새 frontier뿐 아니라 현재 frontier를 진단 주기마다 다시 검사한다. 다음 이유가 하나라도 있으면 발생한다.

| 검사 대상 | 무효 조건 |
|---|---|
| raw map | frontier가 map 바깥 |
| raw map | occupancy가 `map_occupied_threshold` 이상인 벽 cell |
| raw map | unknown cell |
| raw map | 가장 가까운 벽이 기본 0.10 m 이내인 wall-adjacent 상태 |
| raw map | 벽 clearance가 로봇 inscribed radius보다 작음 |
| global costmap | costmap 바깥 |
| global costmap | unknown이며 `allow_unknown_path: false` |
| global costmap | cost가 `costmap_blocked_threshold` 이상 |
| local costmap | unknown이며 `allow_unknown_path: false` |
| local costmap | cost가 `costmap_blocked_threshold` 이상 |
| global grid path | 로봇 cell에서 frontier cell까지 연결 경로 없음 |

local costmap은 작은 rolling window이므로 frontier가 local costmap 바깥이라는 이유만으로는 무효 처리하지 않는다.

기본 cost 기준은 다음과 같다.

- `0`: free
- `1..64`: `inflated-allowed`
- `65..98`: `inflated-blocked`
- `99 이상`: `lethal/inscribed`
- `-1`: unknown

따라서 `inside_inflation=True` 자체가 항상 오류는 아니다. 기본 설정에서는 inflation cost가 65 이상일 때 `FRONTIER_INVALID`가 된다. 실제 threshold는 `costmap_blocked_threshold`로 조정한다.

global grid 연결성 검사는 기본적으로 cost 99 미만의 cell만 통과 가능하다고 보고 8방향 A*를 수행한다. unknown 통과 여부는 `allow_unknown_path`, 최대 탐색량은 `max_grid_path_expansions`로 결정한다.

## 로봇 주변 통과 가능성

| `type` | 심각도 | 발생 조건 |
|---|---:|---|
| `ROBOT_GLOBAL_BLOCKED` | ERROR | goal 활성 중 global costmap에서 로봇 중심으로부터 기본 0.60 m까지 연결된 통과 cell을 찾지 못함 |
| `ROBOT_LOCAL_BLOCKED` | ERROR | goal 활성 중 local costmap에서 같은 조건을 만족하지 못함 |

다음 경우에도 blocked로 본다.

- 로봇 위치가 costmap 바깥
- 로봇 중심 cell의 cost가 `path_blocked_threshold` 이상
- `allow_unknown_path: false`이고 중심 또는 주변 경로가 unknown으로 막힘

이 검사는 costmap에 이미 반영된 inflation을 사용해 로봇 중심 주변의 cell 연결성을 검사한다. footprint polygon을 각 cell에서 직접 회전·충돌 검사하는 방식은 아니다. 실제 수신한 global/local published footprint는 크기를 계산해 스냅샷에 표시하며, footprint가 없으면 설정된 길이와 폭을 사용한다.

## DWB local planner 이상

`/evaluation`은 DWB controller에서 `debug_trajectory_details: true`일 때 발행된다.

| `type` | 심각도 | 발생 조건 |
|---|---:|---|
| `DWB_NO_VALID_TRAJECTORY` | ERROR | goal 활성 중 evaluation에 trajectory가 없거나 유효 trajectory 수가 0 |
| `DWB_INVALID_SELECTION` | ERROR | `best_index`가 trajectory 배열 범위 밖 |
| `DWB_EVALUATION_MISSING` | WARN | goal 활성 후 startup grace period가 지났지만 노드 시작 이후 `/evaluation`을 한 번도 받지 못함 |

정상 evaluation을 받으면 스냅샷의 `dwb=`에 다음이 기록된다.

- 선택된 trajectory index와 전체 개수
- 선택된 `vx`, `vy`, `wz`
- total score
- valid/invalid trajectory 개수
- invalid trajectory를 처음 탈락시킨 critic별 개수와 비율
- worst index
- critic별 raw score, scale, contribution

`first_reject_critic`은 다음 형식으로 출력된다.

```text
first_reject_critic=[
  BaseObstacle=1430(65.5% invalid,64.9% all),
  PathDist=402(18.4% invalid,18.2% all),
  Oscillation=210(9.6% invalid,9.5% all)
]
```

첫 번째 비율은 모든 invalid trajectory 중 해당 critic이 처음 탈락시킨
비율이고, 두 번째 비율은 valid를 포함한 전체 trajectory 대비 비율이다.
DWB는 critic을 설정 순서대로 평가하고 처음 발생한
`IllegalTrajectoryException`에서 평가를 중단하므로, 이 값은 해당
trajectory를 탈락시킬 수 있는 모든 critic이 아니라 **첫 탈락 critic**을
의미한다. `/evaluation` 메시지에는 상세 예외 문구가 없으므로 critic 이름은
알 수 있지만 같은 critic 내부의 구체적인 탈락 사유까지는 구분하지 못한다.

DWB가 아닌 local planner를 사용하면 `/evaluation`이 없으므로 `DWB_EVALUATION_MISSING`이 발생할 수 있다. 이 경우 topic 설정을 맞추거나 해당 로그를 DWB 미사용 확인용으로 해석해야 한다.

## 이상 스냅샷 읽는 방법

ERROR 이상은 ROS ERROR로, WARN 이상은 ROS WARN으로 아래 형식의 여러 줄 스냅샷을 출력한다.

```text
[ANOMALY] type=NO_PROGRESS severity=ERROR
  trigger=goal progress 0.012m over 8.0s; minimum 0.050m
  goal_status=EXECUTING active=True goal=(1.230,0.450) frame=map source=selected_frontier_topic
  robot=(0.800,0.200,15.0deg) window=8.00s samples=9 pose_delta=0.010m yaw_delta=0.002rad goal_progress=0.012m
  feedback_distance=0.48 recoveries=0 feedback_age=0.20s
  cmd_controller=(linear=0.100,angular=0.000) cmd_smoothed=(linear=0.090,angular=0.000)
  odom=(linear=0.005,angular=0.000) odom_age=0.05s scan_age=0.03s
  data_age=(map=0.10s global=0.05s local=0.05s)
  footprint=(inscribed=0.095 circumscribed=0.153 span=0.306)
  frontier_checks=...
  robot_passage=...
  grid_path=... nav2_path=...
  local_paths=...
  dwb=...
```

각 줄의 의미는 다음과 같다.

| 필드 | 의미 |
|---|---|
| `type` | 이상 원인을 구분하는 고정 식별자 |
| `trigger` | 해당 판정을 발생시킨 실제 값 또는 원본 ROS 로그 |
| `goal_status`, `active`, `goal` | 현재 action 상태와 frontier 좌표 |
| `robot`, `window` | 현재 pose와 이상 직전 이력 구간의 이동량·목표 접근량 |
| `feedback_distance`, `recoveries` | NavigateToPose feedback 값과 수신 age |
| `cmd_controller` | local controller가 선택한 원본 속도 명령 |
| `cmd_smoothed` | velocity smoother 등을 지난 최종 속도 명령 |
| `odom` | 실제 측정 속도와 데이터 age |
| `data_age` | map과 costmap 데이터가 마지막으로 수신된 뒤 지난 시간 |
| `footprint` | inscribed radius, circumscribed radius, 최대 폭 |
| `frontier_checks` | map/costmap별 frontier cell, cost, 벽 거리, inflation 상태 |
| `robot_passage` | 로봇 주변 cost, 이웃 free cell, 연결 범위와 PASS/BLOCKED |
| `grid_path` | 내부 grid 연결성 결과 |
| `nav2_path` | ComputePathToPose 결과, error code, pose 수와 길이 |
| `local_paths` | local/transformed global plan pose 수, 길이, age |
| `dwb` | DWB 선택 속도와 critic 점수 |

## 주요 설정값

| parameter | 기본값 | 용도 |
|---|---:|---|
| `diagnostic_period_s` | 1.0 s | 상태 검사 주기 |
| `anomaly_only_logging` | true | true이면 heartbeat 비활성화 |
| `history_window_s` | 10.0 s | 스냅샷 및 동작 판정용 최대 이력 |
| `event_cooldown_s` | 10.0 s | 같은 이상 재출력 간격 |
| `startup_grace_period_s` | 15.0 s | 시작 직후 stale/DWB missing 유예 |
| `no_progress_timeout_s` | 8.0 s | 목표 접근 정체 판정 구간 |
| `min_goal_progress_m` | 0.05 m | 정체가 아니라고 볼 최소 접근 거리 |
| `stuck_timeout_s` | 3.0 s | 명령 대비 로봇 정지 판정 구간 |
| `min_pose_progress_m` | 0.02 m | 정지 판정 위치 변화 기준 |
| `min_yaw_progress_rad` | 0.05 rad | 정지 판정 회전 변화 기준 |
| `linear_cmd_threshold` | 0.03 m/s | 선속도 명령 활성 기준 |
| `angular_cmd_threshold` | 0.10 rad/s | 각속도 명령 활성 기준 |
| `zero_cmd_timeout_s` | 3.0 s | zero command 지속 판정 구간 |
| `data_stale_timeout_s` | 3.0 s | topic stale 기준 |
| `costmap_blocked_threshold` | 65 | frontier costmap 무효 기준 |
| `path_blocked_threshold` | 99 | grid 경로 및 주변 통과 불가 기준 |
| `frontier_wall_proximity_m` | 0.10 m | wall-adjacent 판정 거리 |
| `passage_probe_distance_m` | 0.60 m | 로봇 주변 연결 공간 검사 거리 |

## CLI에서 로그 확인

전체 진단 로그:

```bash
ros2 launch uncc_example frontier_diagnostics.launch.py
```

별도 터미널에서 이상 및 복구 로그만 필터링:

```bash
ros2 topic echo /rosout | grep -E "frontier_diagnostics|ANOMALY|RECOVERED"
```

ROS 로그 파일에서 type별 검색:

```bash
grep -R "type=FRONTIER_INVALID" ~/.ros/log/latest
grep -R "type=ROBOT_STUCK" ~/.ros/log/latest
grep -R "type=NO_PROGRESS" ~/.ros/log/latest
```

`ROS_LOG_DIR`를 별도로 지정했다면 `~/.ros/log/latest` 대신 해당 디렉터리를 사용한다.
