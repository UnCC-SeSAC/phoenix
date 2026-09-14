# Phoenix

**ROS 2 기반 화재 대응 자율 탐사 로봇**

Phoenix는 소방 대원 진입에 앞서 실내 화재 현장을 자율 탐사하는 로봇입니다. 구조대상자와 화점을 탐지하고, 접근·보고·진압까지 수행합니다. 하나의 하드웨어 스택 위에서 **규칙 기반 FSM** 과 **VLA(Vision-Language-Action) 기반** 두 가지 판단 계층을 선택적으로 운용할 수 있도록 설계되었습니다.

---

## 목차

1. [개요](#개요)
2. [두 가지 운용 모드](#두-가지-운용-모드)
3. [시스템 아키텍처](#시스템-아키텍처)
4. [기술 스택](#기술-스택)
5. [저장소 구성](#저장소-구성)
6. [빌드 및 실행](#빌드-및-실행)
7. [ROS 2 인터페이스](#ros-2-인터페이스)
8. [안전 경계 및 알려진 한계](#안전-경계-및-알려진-한계)
9. [문서](#문서)

---

## 개요

Phoenix는 Raspberry Pi 5 기반 MentorPi 로봇에서 동작하며, 다음 과제를 하나의 파이프라인으로 처리합니다.

| 단계 | 내용 |
|---|---|
| **탐사(Exploration)** | Frontier 기반 미탐사 영역 탐색, SLAM으로 실시간 지도 작성 |
| **인지(Perception)** | Hailo NPU 가속 YOLO로 구조대상자·화점 탐지, Depth·CameraInfo·TF로 2D `map(x, y)` 좌표 산출 |
| **판단(Decision)** | Rule-based FSM 또는 VLA Brain이 다음 action을 결정 |
| **실행(Execution)** | Nav2 navigation, 구조대상자·화점 보고, 화재 진압 |
| **감독(Monitoring)** | 웹 기반 소방 관제 UI로 상태·지도·영상 실시간 제공 |

시스템 전체가 기준으로 삼는 좌표계는 **2D `map` frame**입니다. Depth로 계산한 camera frame `(X, Y, Z)`를 이용하여 객체의 `map(x, y)`를 계산합니다.

---

## 두 가지 운용 모드

Phoenix의 핵심 설계 목표는 **판단 계층을 교체 가능하게 만드는 것**입니다. 인지·주행·구동 계층은 두 모드가 완전히 공유하며, 어떤 행동을 할지 결정하는 layer만 달라집니다.

### 모드 A — Rule-based FSM

State transition과 우선순위 규칙으로 동작하는 자율 시스템입니다. 동작이 재현 가능하고 모든 판단 근거를 코드에서 추적할 수 있어, **현장 시연과 안전 기준선 역할**을 합니다.

**State transition**

| State | 진입 조건 |
|---|---|
| `EXPLORING` | 처리할 화점·구조대상자가 없음 → Frontier 미탐사 영역으로 이동 |
| `PERSON_DETECTED` | 대기 goal 중 구조대상자가 최우선 → 구조대상자 접근 |
| `FIRE_DETECTED` | 대기 goal 중 화점이 최우선 → 화점 접근 |
| `RETURNING_TO_BASE` | 배터리 임계치 이하 → 다른 모든 goal보다 우선하여 복귀 |

**판단 규칙**

- 동일 class 객체는 `target_merge_radius`(기본 0.5 m) 이내에서 하나의 goal로 merge
- detection 누적 횟수가 임계치에 도달해야 goal로 확정 (false positive 억제)
- 화점과 구조대상자가 `fire_person_proximity_threshold` 이내로 근접하면 **인명 위험으로 판단해 진압을 우선**, 그보다 멀면 인명 확인을 우선
- 배터리 저전압은 `low_battery_confirm_sec` 동안 연속 유지되어야 상태 전이

**핵심 노드**

| 노드 | 역할 |
|---|---|
| `state_manager` | FSM state 판정, goal 우선순위 결정, detection 확정 및 기록 |
| `mission_executor` | Nav2 goal 전달, 객체 기준 정밀 접근, 진압 action 호출 |
| `frontier_state_controller` | state에 따른 Frontier exploration start·stop 직렬화 |
| `fire_suppression_node` | 펌프·서보 구동 및 진압 재시도 루프 |
| `fire_status_service_node` | YOLO 기반 소화 여부 판정 서비스 |
| `fire_keepout_node` | 화점·구조대상자 주변 진입 금지 영역을 Nav2 KeepoutFilter 마스크로 발행 |
| `rule_based_ui_adapter` | 내부 상태를 UI용 버전 관리 JSON 스냅샷으로 변환 |


### 모드 B — VLA Brain (구현 완료, 실전 검증 필요)

자연어 명령과 Semantic WorldModel을 입력으로 받아 LLM이 다음 행동을선택하는 구조입니다. 규칙을 코드로 열거하지 않고도 상황에 따른 유연한 판단이 가능하다는 것을 검증하기 위한 계층입니다.

> **현재 상태**: 소프트웨어 통합과 유닛·계약 테스트는 완료되었으나, **실제 화재 시나리오에서의 검증은 진행 전입니다.**

**판단 파이프라인**

```
Mission(자연어) + Compact WorldModel
        ↓ HTTP POST /infer
    Qwen3 inference (Ubuntu PC)
        ↓ ActionDecision(action, target, reason)
    TargetResolver   — target ID를 WorldModel의 실제 pose로 변환
        ↓
    ActionValidator  — 안전 조건 검사
        ↓
    ActionDispatcher — ROS 2 action boundary로 전달
```

**지원 행동**

| Action | Target | Execution Boundary |
|---|---|---|
| `NAVIGATE_TO` | person / fire ID | NavigationPort |
| `REPORT_PERSON` | person ID | ReportPort |
| `EXTINGUISH` | fire ID | SprayPort |
| `SEARCH` | unexplored zone ID | NavigationPort |
| `WAIT` | `null` | WaitPort |
| `RETURN_HOME` | `null` | NavigationPort |

**안전 설계 원칙**

- LLM은 WorldModel에 이미 존재하는 target ID만 선택하며, 실제 좌표 변환은 `TargetResolver`가 담당합니다.
- `ActionValidator`가 target 존재 여부, 좌표가 finite한지와 map 범위 내인지, 중복 physical action, 보고 상태, 화점 상태와 분사 사거리를 독립적으로 검증합니다.
- HTTP 실패나 스키마 위반 답변은 `blocked cycle`로 처리되어 로봇이 정지합니다.
- 답변 형식은 `action`, `target`, `reason` 세 영역으로 고정되어 있습니다.

**Pi / PC 역할 분리**

| 실행 주체 | 담당 |
|---|---|
| Raspberry Pi | Camera, YOLO, Depth, TF, SLAM, Nav2, 모터 구동, WorldModel, 안전 검증, Actuator |
| Ubuntu PC | Qwen3 모델 로딩 및 추론 **only** |

원본 image, Depth, LiDAR scan, map, TF, 구동 명령은 모두 로봇에서 처리됩니다.
PC로 나가는 것은 compact WorldModel(JSON)뿐입니다.

### 모드 비교

| 항목 | Rule-based FSM | VLA Brain |
|---|---|---|
| 판단 주체 | FSM | Qwen3 LLM |
| 결정성 | fully deterministic | 확률적 (검증 계층 제약) |
| 임무 지시 | `START` / `STOP` | 자연어 |
| 외부 의존성 | 없음 (fully onboard) | Ubuntu PC inference server |
| Latency | 밀리초 | 수백 밀리초 ~ 초 (`llm_timeout_sec` 기본 10 s) |
| 확장 방식 | 코드에 규칙 추가 | Prompt·WorldModel 확장 |
| 검증 수준 | **현장 검증 완료** | **소프트웨어 검증 완료 / 현장 검증 필요** |
| 권장 용도 | 시연, 안전 기준선 | 연구, 유연한 판단 실험 |

---

## 시스템 아키텍처

```
┌─────────────────────────── Raspberry Pi 5 ───────────────────────────┐
│                                                                       │
│  Camera ──▶ Hailo HEF YOLO ──▶ bbox / score                          │
│                                     │                                 │
│  Depth ──▶ CameraInfo backprojection ──▶ source-time TF ──▶ map(x,y) │
│                                     │                                 │
│                          ┌──────────┴──────────┐                     │
│                          ▼                     ▼                     │
│              ┌───────────────────┐   ┌───────────────────┐           │
│  [모드 A]    │  StateManager     │   │  Semantic         │  [모드 B] │
│              │  (Rule-based FSM) │   │  WorldModel       │           │
│              └─────────┬─────────┘   └─────────┬─────────┘           │
│                        │                       │ compact JSON        │
│                        │                       ▼    HTTP/JSON        │
│                        │             ┌──────────────────────┐        │
│                        │             │  Ubuntu PC : Qwen3   │        │
│                        │             └──────────┬───────────┘        │
│                        │                        ▼ action/target      │
│                        │          Resolver ▶ Validator ▶ Dispatcher  │
│                        │                        │                    │
│                        └───────────┬────────────┘                    │
│                                    ▼                                  │
│           Nav2 (NavigateToPose) / SuppressFire / PersonReport         │
│                                    │                                  │
│                    LD19 LiDAR ▶ slam_toolbox ▶ map→odom              │
│                    Wheel Odom + IMU ▶ EKF ▶ odom→base_footprint      │
│                                    │                                  │
│                            cmd_vel ▶ Motor / Pump / Servo             │
│                                                                       │
│                        Firefighter UI (HTTP :8080)                    │
└───────────────────────────────────────────────────────────────────────┘
```

**좌표 및 TF chain**

```
map ──(slam_toolbox)──▶ odom ──(robot_localization EKF)──▶ base_footprint
```

EKF는 wheel encoder(`odom_raw`)와 IMU를 fusion합니다. 기준이 되는 객체 좌표는 `map` frame 기준입니다.

---

## 기술 스택

| 영역 | 구성 |
|---|---|
| Middleware | ROS 2 Humble |
| Compute | Raspberry Pi 5, Hailo-10H NPU |
| Perception | YOLO26 (HEF), Depth Camera, CameraInfo, TF |
| SLAM | `slam_toolbox` (sync, mapping mode) |
| Navigation | Nav2 (NavFn / DWB / RotationShimController), KeepoutFilter |
| State estimation | `robot_localization` EKF (wheel odometry + IMU) |
| Sensor | LDROBOT LD19 2D LiDAR, RGB-D Camera, IMU |
| Inference | Qwen3, PyTorch, HTTP/JSON |
| UI | Python stdlib HTTP server + 단일 HTML |
| Build·Test | `colcon`, `pytest` |

---

## 저장소 구성

```
src/
├── uncc_example/          Rule-based FSM, mission 실행, suppression, Frontier 연동
├── fire_vla_core/         VLA 도메인 로직 (WorldModel, Resolver, Validator,
│                          Dispatcher, LLM client, Firefighter UI)
├── fire_vla_interfaces/   VLA 전용 message · service 정의
├── fire_vla_bringup/      VLA 실행 launch 및 config
├── frontier_exploration_ros2/  Frontier 기반 exploration planner
├── image_pipeline/        Preprocessing · YOLO inference · Depth 결합
├── peripherals/           LiDAR, Camera, IMU filter driver 연동
├── driver/                Motor controller, Odometry, EKF
├── navigation/            Nav2 parameter 및 launch
├── slam/                  SLAM config 및 launch
└── interfaces/            공용 message · action 정의
```

---

## 빌드 및 실행

### 1. 소프트웨어 Mock (로봇 불필요)

ROS 2가 준비된 개발 PC에서 UI와 판단 계층만 확인합니다.

```bash
colcon build --packages-select \
    fire_vla_interfaces fire_vla_core fire_vla_bringup
source install/setup.bash

ros2 launch fire_vla_bringup firefighter_ui_mock.launch.py
```

브라우저에서 `http://<Robot_IP>:8080` 접속.

### 2. Rule-based 모드 (실제 하드웨어)

```bash
colcon build --symlink-install
source install/setup.bash

ros2 launch uncc_example uncc_frontier.launch.py \
    model_path:=/path/to/model.hef \
    class_names:="['fire','person']"
```

Mission 제어:

```bash
ros2 service call /state_manager/start_mission std_srvs/srv/Trigger "{}"
ros2 service call /state_manager/stop_mission  std_srvs/srv/Trigger "{}"
```

또는 Firefighter UI에서 `START` / `STOP` 버튼 사용.

### 3. VLA 모드 (실제 하드웨어 + Inference server)

**Ubuntu PC — Inference server**

```bash
ros2 run fire_vla_core qwen_inference_server
```

**Raspberry Pi — Robot runtime**

```bash
ros2 launch fire_vla_bringup vla_robot.launch.py
```

`vla_robot.launch.py`에는 다음이 포함되어 있습니다.

- `uncc_frontier.launch.py` (`start_frontier:=false`, `start_mission:=false`, `start_vision:=false`)
  — 하드웨어·SLAM·Nav2만 기동하고 Rule-based 모드는 비활성화
- `topic_bridge_vla.launch.py` (`start_perception_bridge:=true`)
- `vla_navigation_bridge.launch.py`

Mission 지시:

```bash
ros2 topic pub --once /vla/mission std_msgs/String \
    "{data: '{\"text\": \"건물을 수색해서 사람을 찾고 화재를 진압하라\"}'}"
```

---

## ROS 2 인터페이스

### 공통 (Perception · Actuation)

| Topic / Action | Type | 용도 |
|---|---|---|
| `/scan_raw` | `sensor_msgs/LaserScan` | LD19 LiDAR |
| `/map` | `nav_msgs/OccupancyGrid` | slam_toolbox map |
| `/yolo_result` | `vision_msgs/Detection2DArray` | YOLO bbox · score |
| `/fire/detections` | `std_msgs/String` (JSON) | 원본 pixel, depth, score, source stamp |
| `/fire/detections/status` | `std_msgs/String` (JSON) | Perception pipeline heartbeat |
| `navigate_to_pose` | `nav2_msgs/NavigateToPose` | Nav2 navigation |
| `suppress_fire` | `interfaces/SuppressFire` | Suppression action |

### Rule-based 모드

| Boundary | 용도 |
|---|---|
| `/mission/state` | 현재 FSM state |
| `/mission/current_target` | 현재 goal (`map` 좌표) |
| `/mission/found_targets` | 발견된 객체 전체 snapshot (JSON) |
| `/mission/approach_status` | Precision approach 진행 상태 |
| `/fire_keepout_mask`, `/fire_keepout_circles` | Keepout 영역 |
| `/rule_based/status` | UI용 `schema_version=1` snapshot |
| `/rule_based/mission` | UI → 로봇 mission 명령 |

**객체 상태 분류** — `person_unconfirmed`, `person_confirmed`, `person_unreachable`, `fire_unvisited`, `fire_failed`, `fire_extinguished`, `fire_unreachable`

### VLA 모드

| Boundary | 용도 |
|---|---|
| `/vla/mission`, `/vla/status` | Mission 지시 및 상태 |
| `/vla/perception_observation` | 정규화된 person/fire `map(x, y)` |
| `/vla/robot_pose_json` | Robot pose |
| `/vla/navigation_goal` / `_result` / `_cancel` | Navigation |
| `/vla/person_report` / `_result` | 구조대상자 보고 |
| `/vla/spray_command` / `_result` / `_cancel` | 화재 진압 |

---

## 안전 경계 및 알려진 한계

### 안전 경계

- **물리 구동은 검증된 명령으로만 실행됩니다.** LLM이나 브라우저가 모터·펌프·서보를 직접 제어하지 않습니다.
- **Fail-closed 설계.** `depth_status=unknown`, 유효하지 않은 CameraInfo, non-finite depth·좌표, TF lookup 실패는 모두 해당 관측을 폐기합니다.
- **자동 Fallback 없음.** 추론 서버 장애 시 로봇은 정지하며 임의 동작으로 대체하지 않습니다.
- **Navigation goal 단일 소유.** 두 판단 계층이 동시에 goal을 소유하지 않습니다.
- **진압 결과 해석.** Spray `SUCCEEDED`는 `PENDING_VERIFICATION` 전이이며, 그 자체로 소화 완료(`EXTINGUISHED`)를 의미하지 않습니다. 소화 판정은 별도 vision service가 수행합니다.

### 한계 및 보완할 점

- **VLA 모드는 현장 검증 전입니다.** 소프트웨어 통합과 테스트는 완료되었으나 실제 시나리오 검증이 필요합니다.
- **2D LiDAR 기반 시스템입니다.** 화점이 LiDAR 스캔 평면보다 낮은 경우 costmap의 `obstacle_layer`로는 탐지되지 않으며, 이를 보완하기 위해 Keepout masking을 사용합니다. 향후 3D LiDAR를 도입해 높이 정보를 확보하는 것을 고려하고 있습니다.
- **연기 환경에서 2D LiDAR 성능이 저하됩니다.** 고밀도 에어로졸은 강한 산란체로 작용하여 허상 장애물을 생성할 수 있습니다. 운용 전 costmap 파라미터 조정이 필요합니다.

---

## 문서

| 문서 | 내용 |
|---|---|
| [`docs/CURRENT_VLA_DATA_ARCHITECTURE.md`](docs/CURRENT_VLA_DATA_ARCHITECTURE.md) | Pi/PC 역할 분리, Perception, WorldModel, Qwen, Action, ROS 2/HTTP boundary |
| [`docs/RULE_BASED_UI_CONTRACT.md`](docs/RULE_BASED_UI_CONTRACT.md) | Firefighter UI의 Rule-based 모드 상태·제어 boundary |
| [`docs/VLA_ROBOT_RUNTIME_TROUBLESHOOTING.md`](docs/VLA_ROBOT_RUNTIME_TROUBLESHOOTING.md) | Runtime troubleshooting |
| [`src/uncc_example/README.md`](src/uncc_example/README.md) | Frontier exploration 패키지 상세 |

---

## 프로젝트 정보

본 저장소는 **SeSAC Physical AI 최종 프로젝트**로 수행한 팀 연구·개발 결과물입니다. 화재 대응 자율주행 로봇의 인지-판단-제어 파이프라인을 E2E로 구현하고 검증하는 것을 목표로 했습니다.

교육 및 연구 목적으로 공개하며, 실제 재난 현장 투입을 전제하지 않습니다.