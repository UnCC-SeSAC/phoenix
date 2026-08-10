# VLA System Architecture Guide

## 1. 전체 시스템 아키텍처

현재 시스템은 크게 **센싱 → 객체 위치화 → 의미 월드모델 → 판단 → 실행 → 결과 반영**의 폐루프 구조입니다.

```text
┌──────────────────────────────────────────────────────────────┐
│                        SENSOR LAYER                          │
│                                                              │
│   RGB Camera                 Depth Camera         LiDAR       │
│       │                           │                 │          │
└───────┼───────────────────────────┼─────────────────┼──────────┘
        │                           │                 │
        ▼                           ▼                 │
     YOLO                     Depth 처리             │
 class / bbox /               거리 계산              │
 confidence                       │                  │
        │                          │                  │
        └─────────────┬────────────┘                  │
                      ▼                               │
             camera-frame 3D point                   │
                  (X,Y,Z)                            │
             ※ 중간 계산일 뿐                        │
                      │                               │
                      ▼                               │
                     TF ◀──────────── SLAM / Localization
                      │
                      ▼
             map-frame 2D object
                   (x,y)
                      │
                      ▼
           SemanticObservation
                      │
                      ▼
           ┌────────────────────┐
           │ Semantic WorldModel│
           └─────────┬──────────┘
                     │ snapshot
                     ▼
                  Qwen2.5
                  VLA Brain
                     │
                     ▼
               ActionDecision
                     │
                     ▼
               TargetResolver
                     │
                     ▼
               ActionValidator
                     │
                     ▼
              ActionDispatcher
          ┌──────────┼───────────┐
          ▼          ▼           ▼
        Nav2       Pump        Report
          │
          ▼
      Robot Controller
```

중요한 점은 **3D 지도를 쓰는 것이 아닙니다.**

`camera (X,Y,Z)`는 Depth로 객체의 실제 공간 위치를 구하기 위한 Perception 내부 중간값이고, 최종적으로 VLA와 Nav2가 사용하는 authoritative 위치는 `map` frame의 **2D `(x,y)`**입니다.

---

## 2. 전체 시스템 책임 분리

```text
Perception
→ 무엇이 보이는가?

SLAM / TF
→ 그 객체가 지도에서 어디에 있는가?

VLA
→ 현재 상황에서 무엇을 해야 하는가?

Nav2
→ 목표 위치까지 어떻게 갈 것인가?

Robot Controller
→ 모터를 실제로 어떻게 움직일 것인가?
```

### Perception

```text
YOLO
→ class
→ bbox
→ confidence
```

### Depth + TF + SLAM

```text
bbox
→ Depth
→ camera-frame 실제 위치
→ TF
→ map-frame 2D (x,y)
```

### VLA

```text
person_0001 = map (2.4, 1.7)
fire_0001   = map (3.2, 0.8)

Mission:
"인명을 우선 확인해"

→ ActionDecision
```

### Nav2

```text
target map pose
→ path planning
→ local avoidance
→ robot driving
```

---

## 3. 전체 시스템 주요 Input / Output

| Producer | Consumer | ROS Boundary | 핵심 데이터 |
|---|---|---|---|
| RGB Camera | YOLO | `/ascamera/camera_publisher/rgb0/image` | RGB Image |
| Perception + TF | VLA Bridge | `/vision/detections` | person/fire + map `(x,y)` + confidence/source stamp |
| VLA Bridge | VLA | `/vla/perception_observation` | canonical person/fire batch |
| Operator / UI | VLA | `/vla/mission` | 자연어 Mission |
| TF / Localization | VLA | `/vla/robot_pose_json` | robot map `(x,y,yaw)` |
| VLA | Navigation Bridge | `/vla/navigation_goal` | target map pose |
| Navigation Bridge | VLA | `/vla/navigation_result` | success/fail/cancel |
| VLA | Report Consumer | `/vla/person_report` | 발견 인명 정보 |
| Report Consumer | VLA | `/vla/person_report_result` | 보고 성공/실패 |
| VLA | Future Pump bridge | `/vla/spray_command`, `/vla/spray_result`, `/vla/spray_cancel` | correlated 소화 명령/결과/취소 |

Navigation, Person Report, Spray의 VLA-side ROS boundary와 `/vision/detections` → canonical Perception thin bridge는 구현/검증되어 있습니다. 실제 YOLO/camera/depth/TF hardware smoke와 Pump/MCU hardware bridge는 남아 있습니다.

---

# 4. VLA Brain 내부 아키텍처

```text
                   ┌──────────────────────┐
Mission ──────────▶│                      │
                   │                      │
Perception ───────▶│    WORLD MODEL       │
                   │                      │
Robot Pose ───────▶│                      │
                   │                      │
Action Result ────▶│                      │
                   └──────────┬───────────┘
                              │
                              ▼
                       Snapshot 생성
                              │
                              ▼
                         LLMPort
                 ┌────────────┼────────────┐
                 ▼            ▼            ▼
               Mock         Ollama      Qwen2.5
                                           │
                                           ▼
                                    ActionDecision
                                           │
                                           ▼
                                    TargetResolver
                                           │
                                           ▼
                                    ActionValidator
                                           │
                                           ▼
                                   ActionDispatcher
                     ┌────────────┬───────────┬────────┐
                     ▼            ▼           ▼        ▼
                Navigation     Spray       Report     Wait
```

---

## 5. VLA Input ① Mission

Topic:

```text
/vla/mission
```

Type:

```text
std_msgs/msg/String
```

예:

```json
{
  "mission_id": "mission_001",
  "text": "인명을 우선 확인해."
}
```

Mission은 **운영자가 원하는 목적**입니다.

---

## 6. VLA Input ② Semantic Perception

Topic:

```text
/vla/perception_observation
```

Canonical payload:

```json
{
  "timestamp": "2026-08-10T02:00:00+00:00",
  "frame_id": "map",
  "frame_valid": true,
  "detector_healthy": true,
  "detections": [
    {
      "entity_id": "person_0001",
      "class_name": "person",
      "confidence": 0.93,
      "map_position": {
        "x": 2.4,
        "y": 1.7,
        "yaw": 0.0
      }
    },
    {
      "entity_id": "fire_0001",
      "class_name": "fire",
      "confidence": 0.91,
      "map_position": {
        "x": 3.2,
        "y": 0.8
      },
      "size": "SMALL"
    }
  ]
}
```

핵심은 VLA가 이미 **map-frame 2D 좌표**를 받는다는 점입니다.

Stable ID가 upstream에서 없으면 현재 MVP fallback association을 사용합니다.

```text
같은 class
+ 최근 2초 이내
+ 0.5m 이내
→ 동일 entity ID 유지
```

예:

```text
frame 1: person @ (2.00,1.00) → person_0001
frame 2: person @ (2.08,1.04) → person_0001
```

---

## 7. VLA Input ③ Robot Pose

Topic:

```text
/vla/robot_pose_json
```

예:

```json
{
  "timestamp": 1786328403.0,
  "frame_id": "map",
  "pose": {
    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0
  }
}
```

의미:

> 로봇 자신이 2D SLAM 지도에서 현재 어디에 있는가?

---

## 8. VLA Input ④ Action Result

Navigation 예:

```json
{
  "action_id": "action_0001",
  "target_id": "person_0001",
  "status": "SUCCEEDED",
  "message": "Nav2 GoalStatus=4"
}
```

ActionResult는 이전 행동의 성공/실패/취소/timeout을 WorldModel로 되돌려 다음 판단의 입력으로 사용합니다.

---

# 9. Semantic WorldModel

WorldModel은 로봇의 현재 상황판입니다.

```text
WorldModel
├── Mission
├── Robot
│   ├── pose
│   ├── home_pose
│   └── navigation_status
├── People
├── Fires
├── current_action
├── last_action
├── pending_action_ids
└── recent_events
```

예:

```json
{
  "mission": {
    "id": "mission_001",
    "text": "인명을 우선 확인해.",
    "status": "RUNNING"
  },
  "robot": {
    "pose": {
      "x": 0.0,
      "y": 0.0,
      "yaw": 0.0
    },
    "navigation_status": "IDLE"
  },
  "people": [
    {
      "id": "person_0001",
      "position": {
        "x": 2.4,
        "y": 1.7
      },
      "confidence": 0.93,
      "state": "DETECTED",
      "reported": false
    }
  ],
  "fires": [
    {
      "id": "fire_0001",
      "position": {
        "x": 3.2,
        "y": 0.8
      },
      "confidence": 0.91,
      "state": "ACTIVE"
    }
  ]
}
```

---

# 10. Qwen의 책임

Qwen은 좌표를 생성하지 않습니다.

출력은 strict JSON의 세 필드뿐입니다.

```json
{
  "action": "NAVIGATE_TO",
  "target": "person_0001",
  "reason": "미보고 인명에게 접근해야 한다."
}
```

즉 Qwen은:

```text
WHAT
무엇을 할까?
```

를 판단합니다.

---

# 11. TargetResolver

Qwen:

```text
NAVIGATE_TO person_0001
```

Resolver:

```text
WorldModel에서 person_0001 검색
→ map (2.4,1.7)
→ target_pose 생성
→ robot→target 방향 yaw 계산
```

즉:

```text
LLM
→ 누구에게/무엇에게 갈지

Resolver
→ 그 대상의 실제 좌표는 어디인지
```

를 담당합니다.

---

# 12. ActionValidator

LLM의 제안을 바로 실행하지 않습니다.

Validator가 구조화된 WorldModel 사실로 안전성을 검사합니다.

예:

- target 존재 여부
- robot pose freshness
- 좌표 finite 여부
- map bounds
- 이미 보고된 person 여부
- fire ACTIVE 여부
- spray range 여부
- spray attempt limit

따라서:

```text
LLM = 행동 제안자
Validator = 실행 승인자
```

입니다.

---

# 13. ActionDispatcher

```text
NAVIGATE_TO → NavigationPort
SEARCH      → NavigationPort
RETURN_HOME → NavigationPort

EXTINGUISH  → SprayPort
REPORT_PERSON → ReportPort
WAIT        → WaitPort
```

현재 상태:

```text
NavigationPort  → Topic Bridge 구현 완료
ReportPort      → Topic Bridge 구현 완료
SprayPort       → Topic Bridge 구현 완료
WaitPort        → Mock
```

---

# 14. VLA Output ① Navigation

LLM:

```json
{
  "action": "NAVIGATE_TO",
  "target": "person_0001",
  "reason": "인명에게 접근한다."
}
```

Resolver/Validator 이후 `/vla/navigation_goal`:

```json
{
  "action_id": "action_0001",
  "action": "NAVIGATE_TO",
  "target_id": "person_0001",
  "target_pose": {
    "x": 2.4,
    "y": 1.7,
    "yaw": 0.615
  },
  "frame_id": "map"
}
```

Navigation Bridge가 이를 Nav2 `NavigateToPose`로 연결합니다.

---

# 15. VLA Output ② Person Report

```text
REPORT_PERSON person_0001
→ Validator
→ TopicBridgePersonReportAdapter
→ /vla/person_report
```

Payload:

```json
{
  "action_id": "action_0002",
  "mission_id": "mission_001",
  "person_id": "person_0001",
  "map_position": {
    "x": 2.4,
    "y": 1.7
  },
  "confidence": 0.93,
  "timestamp": "2026-08-10T05:40:53+00:00",
  "frame_id": "map"
}
```

`/vla/person_report_result`에서 correlated `SUCCEEDED`가 돌아올 때만:

```text
person_0001.reported = true
state = REPORTED
```

가 됩니다.

---

# 16. VLA Output ③ Pump

현재 VLA-side canonical boundary:

```text
Qwen
→ EXTINGUISH fire_0001
→ Resolver
→ Validator
→ TopicBridgeSprayAdapter
→ /vla/spray_command
→ future Pump/MCU bridge
→ /vla/spray_result
→ PENDING_VERIFICATION
```

실제 Pump/MCU hardware bridge는 pending이며, spray SUCCEEDED만으로 EXTINGUISHED 처리하지 않습니다.

---

# 17. 전체 데이터 폐루프

```text
Camera
→ YOLO
→ Depth
→ TF/SLAM
→ person/fire map(x,y)
→ SemanticObservation
→ WorldModel
→ Qwen
→ ActionDecision
→ Resolver
→ Validator
→ Dispatcher
→ Nav2 / Pump / Report
→ ActionResult
→ WorldModel 갱신
→ 다시 판단
```

개념적으로:

```text
Observe
→ Understand
→ Decide
→ Act
→ Observe again
```

입니다.

---

# 18. Deterministic Mode vs VLA Mode

공통 infrastructure:

```text
SLAM + TF + Localization + Nav2
```

그 위의 **goal decision owner만 모드별로 다릅니다.**

```text
                 공통 기반
        SLAM + TF + Localization + Nav2

              ┌─────────────┐
              │             │
     DETERMINISTIC         VLA
         MODE              MODE
              │             │
 Frontier / StateManager   WorldModel
 MissionExecutor             │
              │            Qwen
              │              │
              └──────→ Nav2 ←┘
```

### Deterministic Mode

```text
Frontier / StateManager / MissionExecutor
→ Nav2
```

### VLA Mode

```text
Semantic WorldModel
→ Qwen
→ VLA Brain
→ VLANavigationBridge
→ Nav2
```

실제 실행에서는 두 goal owner를 동시에 켜지 않습니다.

---

# 19. 현재 구현 상태

```text
Camera/RGB                   ✅
YOLO 자체                    팀 작업
Depth                        팀 작업
Object TF → map(x,y)         팀 upstream / bridge 계약 완료 (hardware smoke pending)

Canonical Perception         ✅
Stable ID fallback           ✅
Semantic WorldModel          ✅
Qwen2.5 Adapter              ✅
Resolver                     ✅
Validator                    ✅
Dispatcher                   ✅

Navigation Topic Bridge      ✅
Navigation Result lifecycle  ✅

Person Report Bridge         ✅
Person Report Result         ✅

VLA Spray Topic Bridge       ✅
Pump/MCU hardware bridge     ⏳ upstream
Firefighter Mission/Status UI ✅
Actual Robot VLA E2E         ⏳ 후반
```

---

# 20. 핵심 암기용 요약

VLA Input은 크게 4개입니다.

```text
1. Mission
2. 사람/불의 map 위치
3. Robot의 map 위치
4. 이전 Action의 결과
```

VLA 내부:

```text
Input
→ WorldModel
→ Qwen
→ ActionDecision
→ Resolver
→ Validator
→ Dispatcher
```

주요 Output:

```text
1. 어디로 가라 → Nav2
2. 불을 꺼라   → Pump
3. 사람을 보고 → Report
```

---

# 21. Firefighter Mission / Semantic Status UI

```text
Browser
├─ POST /api/mission → /vla/mission
└─ GET /api/status ← FirefighterUINode ← /vla/status
```

`/vla/status`는 WorldModel snapshot과 최신 decision, validation, submission, blocked reason을 전달한다. UI는 Mission과 robot/person/fire, navigation/report/spray lifecycle, safety 상태를 한 화면에 표시하며 browser SVG로 map-frame semantic point를 auto-fit한다. SLAM occupancy grid는 표시하지 않는다. 기본 서버는 `127.0.0.1:8080`이고 직접 Action/Nav2/Pump 제어 endpoint는 없다. 실제 Perception이 VLA-03B에서 연결되어도 UI는 동일 status boundary를 사용한다.
