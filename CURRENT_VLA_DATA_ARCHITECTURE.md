# Current VLA Data Architecture — 2D Map Contract

분석 기준:

- VLA branch: `feature/vla-brain` @ `102852e17365424dc9c6359df9d551b5c8312bdc`
- Perception branch: `origin/albitro/image_processing` @ `0bf507edc801086bb90ecbff7e5a4eabf5662363`
- 분석일: 2026-08-10
- 관련 Issue: #25, #28, #29, #30, #31, #33, #41

상태 표기:

- `[IMPLEMENTED]`: 현재 읽은 branch 코드에서 확인됨
- `[PLANNED/UPSTREAM]`: Issue 또는 계획에는 있으나 현재 branch 구현으로 확인되지 않음
- `[UNKNOWN/NEEDS CONTRACT]`: producer/consumer 사이에서 확정이 필요한 계약

## 1. 결론

현재 시스템의 최종 지도와 Navigation 공간은 **2D SLAM / 2D Nav2의 `map` frame**이다.

> Depth 기반 camera-frame 3D point는 객체의 2D map 위치를 얻기 위한 Perception 내부 중간 표현이다. 시스템이 3D map을 사용한다는 의미가 아니다.

VLA의 authoritative semantic target position은 `map` frame의 `(x, y)`이다. VLA-03A는 canonical map-frame 검증과 upstream ID 우선/fallback stable ID association까지 구현했다. Perception branch에는 실행 가능한 YOLO publisher, depth 역투영, object TF 변환, 최종 map `(x,y)` publisher가 확인되지 않아 실제 팀 topic/message 연결은 VLA-03B pending이다.

## 2. 전체 시스템 아키텍처

```text
                              2D SLAM / 2D Nav2 System

                 ┌──────── RGB Camera [IMPLEMENTED infra] ────────┐
                 │                                                 │
                 ▼                                                 │
       YOLO class/confidence/bbox                           Depth image
       [PARTIAL/implementation unavailable]                [IMPLEMENTED infra]
                 │                                                 │
                 └───────────────┬─────────────────────────────────┘
                                 │ + CameraInfo
                                 ▼
                    camera optical-frame 3D point
                     (X,Y,Z: Perception 중간 계산)
                         [PLANNED/UPSTREAM #28/#29]
                                 │
                                 ▼
                      TF(source frame → map)
                         [PLANNED/UPSTREAM #29]
                                 │
                                 ▼
                     map-frame object 2D (x,y)
                         [PLANNED/UPSTREAM]
                                 │
                                 ▼
             /vla/perception_observation / std_msgs/String JSON
              [IMPLEMENTED VLA consumer, upstream producer absent]
                                 │
                                 ▼
                       SemanticObservation
                                 │
                                 ▼
                       Semantic WorldModel
                                 │
                                 ▼
                     LLMPort → Qwen2.5 VLA
                                 │
                                 ▼
                         ActionDecision
                                 │
                    TargetResolver → Validator
                                 │
                                 ▼
                         ActionDispatcher
                    ┌────────────┼─────────────┐
                    ▼            ▼             ▼
              NavigationPort  SprayPort    ReportPort
                    │         [MOCK]        [MOCK]
                    ▼
             VLANavigationBridge
                    │
                    ▼
            Nav2 NavigateToPose (2D map)
                    │
                    ▼
              /cmd_vel / Robot controller
```

SLAM/TF/Localization/Nav2는 공통 2D navigation infrastructure다. 객체의 camera-frame 3D point는 map-frame 2D target을 산출하기 위한 Perception 내부 단계이며 VLA Domain에 전달하지 않는다.

## 3. Perception branch 현재 구현

### 3.1 확인된 구현

- `[IMPLEMENTED]` `src/video_frame.py`: 기본 `result_img` Image 구독 및 AVI 저장. detection data producer가 아니다.
- `[IMPLEMENTED]` `src/view_result.py`: Image 결과 viewer. detection data producer가 아니다.
- `[IMPLEMENTED]` depth camera launch 및 RGB/depth camera topic infrastructure.
- `[IMPLEMENTED]` `interfaces/ObjectInfo.msg`: `class_name`, `box`, `score`, `width`, `height`.
- `[IMPLEMENTED]` `interfaces/ObjectsInfo.msg`: `ObjectInfo[] objects`.
- `[IMPLEMENTED]` YOLO launch는 `image_topic=/ascamera/camera_publisher/rgb0/image`, `device=cpu`, `pub_result_img=true`를 설정한다.

### 3.2 확인 불가능하거나 미구현인 부분

`src/yolov5_ros2/yolov5_ros2`는 repository 내부 Python package가 아니라 `/ros2_ws/src/yolov5_ros2/yolov5_ros2`를 가리키는 절대 symlink다. 따라서 다음은 branch checkout을 source of truth로 삼아 구현됐다고 판정할 수 없다.

- `[UNKNOWN/NEEDS CONTRACT]` detection publisher의 실제 topic/type/schema
- `[UNKNOWN/NEEDS CONTRACT]` 실제 class set (`person`, `fire`, `obstacle`)과 label normalization
- `[PLANNED/UPSTREAM #28]` bbox 내부 depth median, invalid-depth 판정, hole filling, mm→m
- `[PLANNED/UPSTREAM #29]` CameraInfo K 기반 역투영
- `[PLANNED/UPSTREAM #29]` 원본 image timestamp 보존
- `[PLANNED/UPSTREAM #29]` camera optical frame → `map` TF
- `[PLANNED/UPSTREAM #29]` TF 실패 시 drop 및 옛 좌표 재사용 금지
- `[PLANNED/UPSTREAM #29]` `/fire/detections_3d`
- `[UNKNOWN/NEEDS CONTRACT]` stable tracking/entity ID
- `[UNKNOWN/NEEDS CONTRACT]` 최종 VLA용 map-frame 2D output

README는 `/yolo_result`와 raw/normalized image coordinates를 설명하지만 실행 publisher 코드를 branch에서 확인할 수 없으므로 실제 계약으로 확정하지 않는다. `ObjectInfo.box`는 pixel bbox이며 map 좌표가 아니다. `ObjectInfo/ObjectsInfo`에는 ROS Header, timestamp, frame ID, entity ID, depth 상태, map 좌표가 없다.

## 4. Perception 실제 output contract

현재 branch 코드만으로 확정 가능한 detection data contract는 제한적이다.

| 항목 | 현재 확인 결과 | 상태 |
|---|---|---|
| RGB input | `/ascamera/camera_publisher/rgb0/image` launch parameter | `[IMPLEMENTED]` |
| Result image | `result_img` 또는 README의 `/result_img`; namespace 확정 불가 | `[UNKNOWN/NEEDS CONTRACT]` |
| Detection topic | README는 `/yolo_result`, publisher 구현은 repository에 없음 | `[UNKNOWN/NEEDS CONTRACT]` |
| Detection ROS type | package는 `vision_msgs`에 의존하지만 실제 publisher type 확인 불가 | `[UNKNOWN/NEEDS CONTRACT]` |
| Candidate message | `interfaces/ObjectsInfo`: `objects[]` | `[IMPLEMENTED definition only]` |
| Class | `ObjectInfo.class_name: string` | `[IMPLEMENTED definition only]` |
| Confidence | `ObjectInfo.score: float32` | `[IMPLEMENTED definition only]` |
| Bbox | `ObjectInfo.box: int32[]`, width, height | `[IMPLEMENTED definition only]` |
| Entity/tracking ID | 필드 및 tracker producer 없음 | `[UNKNOWN/NEEDS CONTRACT]` |
| Timestamp/Header | `ObjectInfo/ObjectsInfo`에 없음 | `[UNKNOWN/NEEDS CONTRACT]` |
| Source frame | 없음 | `[UNKNOWN/NEEDS CONTRACT]` |
| Depth | 최종 detection message에 없음 | `[PLANNED/UPSTREAM #28]` |
| camera-frame 3D | 없음 | `[PLANNED/UPSTREAM #29]` |
| final map x/y | 없음 | `[PLANNED/UPSTREAM]` |
| invalid depth | 표현/동작 없음 | `[PLANNED/UPSTREAM #28/#29]` |
| TF failure | 구현 없음; Issue는 발행 생략+경고 요구 | `[PLANNED/UPSTREAM #29]` |

`/fire/detections_3d`라는 계획 이름은 camera-frame 3D 또는 TF 결과를 전달하는 Perception pipeline 단계일 수 있다. 이것을 3D map으로 해석하지 않는다. 최종 VLA boundary는 반드시 `frame_id=map`, `map_x`, `map_y` 또는 이에 동등한 2D 표현이어야 한다.

## 5. 전체 시스템 boundary Input / Output

| Producer → Consumer | ROS boundary | Type | Payload/coordinate/timestamp | Source of truth | 상태 |
|---|---|---|---|---|---|
| RGB camera → YOLO | `/ascamera/camera_publisher/rgb0/image` | `sensor_msgs/Image` | image header/source camera frame | camera driver | `[IMPLEMENTED infra]` |
| Depth camera → depth processor | camera-specific depth topic | `sensor_msgs/Image` | depth + source image stamp | camera driver | `[IMPLEMENTED infra, processing planned]` |
| Camera → projection | CameraInfo topic | `sensor_msgs/CameraInfo` | K/intrinsics + frame/stamp | camera calibration | `[IMPLEMENTED infra, use planned]` |
| YOLO → Perception pipeline | README `/yolo_result` | unknown; candidate `vision_msgs` or `interfaces/ObjectsInfo` | class/confidence/bbox; no verified stable ID | YOLO producer | `[UNKNOWN/NEEDS CONTRACT]` |
| Projection → TF | planned `/fire/detections_3d` | unknown | camera-frame `(X,Y,Z)`, original stamp | Perception | `[PLANNED/UPSTREAM]` |
| Perception+TF → VLA | final topic not implemented; VLA expects `/vla/perception_observation` | VLA expects `std_msgs/String` JSON | map-frame 2D entities, ISO timestamp | Perception+TF/SLAM | `[PLANNED/UPSTREAM producer]` |
| Mission UI/operator → VLA | `/vla/mission` | `std_msgs/String` JSON | mission ID/text | operator | `[IMPLEMENTED]` |
| TF bridge → VLA | `/vla/robot_pose_json` | `std_msgs/String` JSON | `map` Pose2D, Unix seconds | TF/localization | `[IMPLEMENTED]` |
| VLA → Navigation bridge | `/vla/navigation_goal` | `std_msgs/String` JSON | map-frame target Pose2D | Resolver/WorldModel | `[IMPLEMENTED/VERIFIED]` |
| Navigation bridge → VLA | `/vla/navigation_result` | `std_msgs/String` JSON | correlated terminal result | Nav2 bridge | `[IMPLEMENTED/VERIFIED without real Nav2]` |
| VLA → Pump | production ROS boundary 미정 | current `SprayPort`, mock adapter | Action/ActionResult | VLA + Pump team | `[PLANNED/UPSTREAM]` |
| VLA → Report | `/vla/person_report`, `/vla/person_report_result` | `std_msgs/String` JSON | authoritative person data + correlated ActionResult | VLA + future reporting consumer | `[IMPLEMENTED/VERIFIED without external backend]` |

## 6. VLA 현재 input contract

### 6.1 Mission `[IMPLEMENTED]`

Topic/type: `/vla/mission`, `std_msgs/msg/String`

```json
{
  "mission_id": "mission_001",
  "text": "인명을 우선 확인해."
}
```

`text`는 필수다. `mission_id`가 없으면 `mission_001`을 사용한다.

### 6.2 Semantic observation `[IMPLEMENTED VLA consumer]`

Topic/type: `/vla/perception_observation`, `std_msgs/msg/String`

```json
{
  "timestamp": "2026-08-10T02:00:00+00:00",
  "frame_id": "map",
  "frame_valid": true,
  "detector_healthy": true,
  "detections": [
    {
      "entity_id": "person_01",
      "class_name": "person",
      "confidence": 0.93,
      "map_position": {
        "x": 2.4,
        "y": 1.7,
        "yaw": 0.0
      }
    },
    {
      "entity_id": "fire_01",
      "class_name": "fire",
      "confidence": 0.91,
      "map_position": {
        "x": 3.2,
        "y": 0.8
      },
      "size": "SMALL",
      "blocks_route_to": "person_01"
    }
  ]
}
```

실제 parser 계약:

- batch `timestamp`: 필수 문자열이며 `datetime.fromisoformat()` 가능한 timezone-aware ISO-8601을 사실상 요구한다.
- `detections`: 없으면 빈 배열.
- 각 detection의 `class_name`, `confidence`, `map_position.x`, `map_position.y`: 필수. `entity_id`는 optional이며 non-empty 값은 보존한다.
- `map_position.yaw`: optional, 기본 0.0. 객체 고유 yaw가 아니라 Resolver가 현재 robot→target 방향 yaw를 다시 계산할 수 있다.
- `size`, `blocks_route_to`: optional.
- `frame_valid`, `detector_healthy`: optional, 기본 true.
- accepted semantic classes: `person`, `fire`만. 대소문자는 WorldModel에서 lower-case 처리한다.
- confidence threshold: person 0.50, fire 0.60.
- batch stale threshold: 기본 1.0초 초과 시 전체 batch drop.
- top-level `frame_id="map"`은 필수이며 canonical normalizer가 다른 frame 또는 누락을 거부한다.
- malformed JSON/missing key/type conversion error는 callback warning 후 batch를 적용하지 않는다.
- canonical boundary는 person/fire 이외 class를 거부한다. WorldModel을 직접 호출하는 내부 경로에서는 unknown class를 upsert하지 않는다.
- confidence `[0,1]`, timestamp timezone, person/fire class, finite position, batch ID 중복은 boundary에서 검증한다. map bounds는 ingest 시 새 정책을 만들지 않고 기존 ActionValidator를 유지한다.
- map bounds 및 finite check는 이후 navigation ActionValidator에서 target pose에 대해 `[-100,100]`과 finite를 검사한다. 이는 ingest 방어를 대체하지 않는다.

Domain 변환:

```text
JSON detection
→ SemanticObservation(entity_id, class_name, confidence,
                      Pose2D(map_x, map_y, yaw), observed_at,
                      size?, blocks_route_to?)
→ ObservationBatch(timestamp, observations, frame_valid, detector_healthy)
```

### 6.3 Robot pose `[IMPLEMENTED]`

Topic/type: `/vla/robot_pose_json`, `std_msgs/msg/String`

```json
{
  "timestamp": 1786328403.0,
  "frame_id": "map",
  "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}
}
```

Parser는 Unix seconds를 UTC ISO-8601로 변환한다. 현재 parser는 `frame_id`를 검증하지 않으며 bridge launch가 `map → base_footprint` pose를 제공하는 계약에 의존한다. 이동 Validator의 robot pose freshness 기본값은 0.5초다.

### 6.4 Navigation result `[IMPLEMENTED/VERIFIED]`

Topic/type: `/vla/navigation_result`, `std_msgs/msg/String`

```json
{
  "action_id": "action_0001",
  "target_id": "person_01",
  "status": "SUCCEEDED",
  "message": "Nav2 GoalStatus=4"
}
```

허용 status는 `SUCCEEDED`, `FAILED`, `ABORTED`, `CANCELED`, `TIMED_OUT`이다. action ID correlation, stale/unrelated result 차단, duplicate terminal 방어가 구현되어 있다.

## 7. Contract gap

| 항목 | Perception 실제 출력 | VLA 현재 입력 | 상태 | 필요한 조치 |
|---|---|---|---|---|
| topic | README `/yolo_result`; 실제 publisher 미확인 | `/vla/perception_observation` | Gap | upstream 최종 semantic topic 확정 또는 얇은 Adapter topic mapping |
| ROS type | 미확인; candidate `ObjectsInfo`/`vision_msgs` | `std_msgs/String` JSON | Gap | 실제 producer type 확정 후 한 개 Adapter에서 변환 |
| class | candidate `class_name` | `class_name`, person/fire | Partial | 실제 label set과 person/fire canonical mapping 확정 |
| confidence | candidate `score` | `confidence`, person≥0.50/fire≥0.60 | Small mapping | score scale `[0,1]` 확인 후 rename |
| bbox | `box` pixel array | 사용하지 않음 | Expected | Perception 내부 depth 계산에만 사용 |
| entity ID | 없음 | optional upstream ID + VLA fallback | VLA-03A complete | upstream ID 우선, 없으면 2D association |
| map_x | 없음 | `map_position.x` 필수 | Blocker | Perception+Depth+TF가 map x 제공 |
| map_y | 없음 | `map_position.y` 필수 | Blocker | Perception+Depth+TF가 map y 제공 |
| yaw | 없음 | optional, 기본 0; Resolver가 target 방향 재계산 | Compatible | upstream에서 억지로 생성할 필요 없음 |
| frame_id | detection contract에 없음 | explicit `map` 필수 | VLA-03A complete | VLA-03B Adapter가 map-frame contract 제공 |
| timestamp | Object messages에 Header 없음 | timezone-aware ISO batch timestamp 필수 | Blocker | 원본 image stamp 보존 후 boundary format 확정 |
| source frame | 없음 | VLA에는 불필요하나 traceability에 유용 | Contract needed | upstream 내부/diagnostic field로 보존 여부 결정 |
| invalid depth | 표현/동작 없음 | `frame_valid` batch flag만 있음 | Blocker | map position 불가능 detection은 upstream drop, 상태/metric 정의 |
| TF failure | 구현 없음 | map position을 신뢰 | Blocker | upstream drop+warning, old coordinate 재사용 금지 |
| stale observation | 원본 stamp 보존 미구현 | 1.0초 초과 batch drop | Partial | upstream original stamp와 clock/timezone 계약 |
| person | 실제 model label/출력 미확인 | 정식 WorldModel entity | VLA ready/03B dependency | upstream class mapping 확인 |
| fire | 실제 model label/출력 미확인 | 정식 WorldModel entity | VLA ready/03B dependency | upstream class mapping 확인 |
| obstacle | 실제 model label 미확인 | 정식 semantic entity 아님; 무시됨 | No VLA mapping | Nav2 costmap/local avoidance owner 유지 |
| NaN/Inf | 동작 미확인 | canonical boundary에서 차단 | VLA-03A complete | upstream도 finite map coordinate만 제공 |
| map bounds | 동작 미확인 | ingest 시 없음, ActionValidator만 ±100m | Contract gap | 실제 map bounds owner/정책 확인 |

## 8. Entity ID ownership

현재 Perception branch에는 tracking ID가 확인되지 않는다. VLA-03A canonical boundary는 non-empty upstream ID를 우선 보존하고, ID가 없을 때 같은 class의 최근 map position으로 fallback ID를 연결한다.

현재 MVP 계약:

1. upstream stable ID가 있으면 VLA가 opaque identifier로 보존한다.
2. ID가 없으면 same-class, nearest 2D distance, radius 0.5 m, TTL 2.0초로 association한다.
3. batch one-to-one이며 거리 동률은 entity ID로 결정한다.
4. 새 ID는 `person_0001`, `fire_0001` 형식의 process-local ID다.
5. 빠른 이동, 근접 교차, 긴 occlusion, 장시간 재탐지, process restart에서는 ID switch가 가능하다. ByteTrack/Kalman 등 tracker가 아니다.

## 9. Map coordinate ownership

| 단계 | Owner | 계약 |
|---|---|---|
| YOLO class/bbox/confidence | Perception | image-space detection |
| bbox depth sampling | Perception | valid depth 또는 distance unknown |
| camera-frame `(X,Y,Z)` | Perception 내부 | 중간 계산, 3D map 데이터 아님 |
| camera/source → map TF | Perception + TF infrastructure | original image timestamp 기준 변환 |
| final object map `(x,y)` | Perception + TF/2D SLAM | VLA boundary의 authoritative target location |
| semantic state 저장 | VLA WorldModel | stable entity ID별 Pose2D/state |
| target ID → pose | VLA TargetResolver | WorldModel pose 사용, LLM 좌표 생성 금지 |
| target yaw | VLA TargetResolver | robot pose에서 target `(x,y)`를 향하는 yaw 계산 |
| path planning/driving | 2D Nav2 | map-frame Pose2D goal |

VLA는 bbox, raw depth, camera intrinsics, camera-frame z, 3D TF projection을 소유하지 않는다.

## 10. Stale / invalid 책임 경계

### Perception에서 drop해야 하는 것 `[PLANNED/UPSTREAM]`

- invalid/0/insufficient depth로 유효한 object position을 계산할 수 없는 detection
- CameraInfo가 없거나 intrinsic이 유효하지 않은 detection
- source frame이 없거나 변환 불가능한 detection
- original image timestamp의 TF lookup 실패
- finite map `(x,y)`를 만들 수 없는 detection
- 옛 map coordinate 재사용

진단 metric/status와 warning은 남기되, 좌표 없는 detection을 정상 semantic entity로 발행하지 않는다.

### VLA boundary에서 구현된 방어 `[IMPLEMENTED VLA-03A]`

- explicit `frame_id == "map"`
- timezone-aware ISO timestamp
- upstream ID 우선 또는 fallback ID 생성, batch duplicate ID 거부
- person/fire class만 허용
- confidence finite 및 `[0,1]`
- map x/y/yaw finite
- malformed detection은 canonical batch 전체를 안전 거부

실제 map bounds는 새 ingest 정책을 만들지 않고 기존 navigation ActionValidator를 유지한다.

### VLA Core에 이미 있는 방어 `[IMPLEMENTED]`

- observation batch 1.0초 stale drop
- `frame_valid=false` 또는 `detector_healthy=false` batch invalid 처리
- person/fire confidence threshold
- canonical boundary는 unknown class 거부; direct WorldModel path는 무시
- navigation 직전 pose finite/map bounds 검증

## 11. Obstacle과 Nav2 costmap 책임

`obstacle`은 현재 VLA Domain/WorldModel의 정식 semantic entity가 아니다. `update_observation_batch()`는 person/fire만 upsert하므로 obstacle은 무시한다.

일반적인 충돌 회피 장애물은 Nav2 costmap, LiDAR/depth obstacle layer, local planner/local avoidance가 소유해야 한다. VLA WorldModel에 모든 obstacle을 중복 모델링하면 stale state와 ownership 충돌이 생긴다. Mission 의미상 특별한 위험물/통행불가 semantic zone이 별도 요구될 때만 별 Issue/계약으로 검토한다. VLA-03에서 obstacle entity를 추가하지 않는다.

## 12. VLA Brain 내부 아키텍처

```text
Mission (/vla/mission)
SemanticObservation (map x,y; /vla/perception_observation)
RobotPose (map x,y,yaw; /vla/robot_pose_json)
NavigationResult (/vla/navigation_result)
                    │
                    ▼
             Semantic WorldModel
                    │ snapshot
                    ▼
                  LLMPort
      ┌─────────────┼────────────────┐
      ▼             ▼                ▼
 MockVLABrain  OllamaLLMClient  TransformersQwenAdapter
                                   Qwen2.5/XPU
                    │
                    ▼
              ActionDecision
                    │
                    ▼
              TargetResolver
                    │ authoritative WorldModel Pose2D
                    ▼
              ActionValidator
                    │
                    ▼
             ActionDispatcher
      ┌─────────────┼─────────────┬────────────┐
      ▼             ▼             ▼            ▼
 NavigationPort  SprayPort    ReportPort     WaitPort
 NAVIGATE/       EXTINGUISH    REPORT_PERSON  WAIT
 SEARCH/
 RETURN_HOME
```

`SEARCH`와 `RETURN_HOME`은 별 Port가 아니라 `NavigationPort`로 dispatch된다. Production ROS boundary는 Topic Bridge Navigation과 Person Report에 구현되어 있다. Spray/Wait는 현재 mock adapter다.

## 13. WorldModel snapshot

실제 `create_snapshot()` 구조 예시:

```json
{
  "mission": {
    "id": "mission_001",
    "text": "인명을 우선 확인해.",
    "status": "RUNNING"
  },
  "exploration_status": "RUNNING",
  "perception_ready": true,
  "robot": {
    "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
    "pose_updated_at": "2026-08-10T02:00:00+00:00",
    "navigation_status": "IDLE",
    "home_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}
  },
  "people": [
    {
      "id": "person_01",
      "position": {"x": 2.4, "y": 1.7, "yaw": 0.0},
      "confidence": 0.93,
      "state": "DETECTED",
      "reported": false,
      "first_seen": "2026-08-10T02:00:00+00:00",
      "last_seen": "2026-08-10T02:00:00+00:00"
    }
  ],
  "fires": [
    {
      "id": "fire_01",
      "position": {"x": 3.2, "y": 0.8, "yaw": 0.0},
      "confidence": 0.91,
      "size": "SMALL",
      "state": "ACTIVE",
      "blocks_route_to": "person_01",
      "spray_count": 0,
      "robot_within_spray_range": false,
      "first_seen": "2026-08-10T02:00:00+00:00",
      "last_seen": "2026-08-10T02:00:00+00:00",
      "verification_started_at": null,
      "verification_valid_observations": 0
    }
  ],
  "current_action": null,
  "last_action": null,
  "pending_action_ids": [],
  "unexplored_zones": [],
  "recent_events": []
}
```

| Snapshot field | Origin/owner | Source |
|---|---|---|
| mission | external input, VLA lifecycle update | `/vla/mission` |
| exploration_status | VLA state | mission/exploration integration |
| perception_ready | WorldModel derived | valid observation batch |
| robot.pose/updated_at | external input | `/vla/robot_pose_json` |
| robot.home_pose | VLA derived from first mission pose | robot pose |
| robot.navigation_status | VLA lifecycle derived | `/vla/navigation_result` |
| people/fires ID, position, confidence, seen time | external semantic input stored by VLA | `/vla/perception_observation` |
| people/fires state flags | VLA generated/derived | observations + ActionResults |
| `position` | **2D map target location** | upstream Perception+TF/SLAM |
| current_action/last_action/pending IDs | VLA generated | Dispatcher submission/result |
| recent_events | VLA diagnostic bookkeeping | all VLA state transitions |

## 14. VLA output contract

LLM output은 항상 strict JSON의 `action`, `target`, `reason` 세 필드이며 좌표를 만들지 않는다.

| Action | LLM target | Resolver output | 주요 Validator 조건 | Dispatcher/actual boundary |
|---|---|---|---|---|
| NAVIGATE_TO | person/fire stable ID | WorldModel 2D pose + facing yaw | target exists, fresh robot pose, finite/bounds | NavigationPort → `/vla/navigation_goal` |
| REPORT_PERSON | person stable ID | target ID, no target pose | person exists and not reported | ReportPort → `/vla/person_report`; result on `/vla/person_report_result` |
| EXTINGUISH | fire ID | target ID, no target pose | ACTIVE, within spray range, attempts limit | SprayPort → current mock; production ROS TBD |
| SEARCH | unexplored zone ID | zone 2D pose | fresh robot pose, finite/bounds | NavigationPort → `/vla/navigation_goal` |
| WAIT | JSON null | no pose | common action validity | WaitPort → current mock |
| RETURN_HOME | JSON null | robot.home_pose | fresh robot pose, finite/bounds | NavigationPort → `/vla/navigation_goal` |

NAVIGATE_TO example:

```text
Qwen: NAVIGATE_TO person_01
        │
WorldModel: person_01 → map (2.4, 1.7)
        │
Resolver: target_pose = (2.4, 1.7, yaw=robot→target)
        │
Validator: target/freshness/finite/map-bounds
        │
Topic Bridge: /vla/navigation_goal
        │
Nav2: map-frame 2D NavigateToPose
```

Navigation goal payload `[IMPLEMENTED/VERIFIED]`:

```json
{
  "action_id": "action_0001",
  "action": "NAVIGATE_TO",
  "target_id": "person_01",
  "target_pose": {"x": 2.4, "y": 1.7, "yaw": 0.615},
  "frame_id": "map"
}
```

Navigation cancel payload:

```json
{"action_id": "action_0001"}
```

## 15. Ownership matrix

| 데이터/결정 | Owner |
|---|---|
| RGB/depth capture | Camera driver / Perception infrastructure |
| YOLO class/bbox/confidence | Perception |
| Depth validity/filtering | Perception |
| CameraInfo/calibration | Perception/camera calibration |
| camera-frame 3D point | Perception 내부 중간 계산 |
| TF transform | Perception + TF infrastructure |
| 최종 object map `(x,y)` | Perception + TF/2D SLAM |
| stable entity/tracking ID | upstream 우선; 없으면 VLA MVP fallback |
| Semantic entity state | VLA WorldModel |
| Mission interpretation | VLA |
| Action decision | VLA |
| target ID → authoritative 2D pose | VLA TargetResolver |
| path planning | 2D Nav2 |
| obstacle avoidance | Nav2 costmap / local planner / local avoidance |
| low-level movement | Robot controller |
| Pump low-level control | Pump/MCU |
| Spray safety approval/action lifecycle | VLA Validator/Adapter |
| Person report lifecycle | VLA Report Adapter + reporting consumer |

## 16. DETERMINISTIC mode vs VLA mode

```text
             Common 2D infrastructure
       SLAM + TF + Localization + Nav2
                      │
          ┌───────────┴───────────┐
          │                       │
 DETERMINISTIC MODE            VLA MODE
          │                       │
 Frontier / StateManager      Semantic WorldModel
 / MissionExecutor                │
          │                    Qwen VLA Brain
          │                       │
          │                VLANavigationBridge
          │                       │
          └───────────→ Nav2 ←────┘
```

- DETERMINISTIC mode: Frontier/StateManager/MissionExecutor goal sender ON, VLA navigation goal sender OFF.
- VLA mode: VLA goal sender ON, Frontier/MissionExecutor goal sender OFF.
- SLAM, TF, Localization, Nav2 자체는 두 mode에서 공통이다.
- 실제 실행에서 두 goal owner를 동시에 사용하지 않는다.
- 새 Navigation Manager/arbitration layer를 만들지 않는다.
- VLA node의 `navigation_mode=MOCK|TOPIC_BRIDGE`는 Adapter composition 선택이며 system ownership mode와 구분한다.

## 17. VLA-03A 완료 및 VLA-03B dependency

VLA-03A canonical contract, safety validation, stable ID fallback과 WorldModel snapshot 경로는 구현/검증 완료했다. 실제 팀 Perception final topic/message를 canonical boundary로 변환하는 Adapter는 VLA-03B pending이다. VLA-03B는 아래 질문이 확정된 후 시작한다.

1. 최종 map-frame object output의 topic과 ROS message type은 무엇인가?
2. 각 detection에 original image timestamp와 명시적인 `frame_id="map"`가 포함되는가?
3. upstream stable person/fire ID를 제공할 계획이 있는가? 제공 시 형식과 lifecycle은 무엇인가?
4. class canonical value와 confidence 범위는 정확히 무엇인가 (`person`, `fire`, obstacle 포함 여부)?
5. invalid depth와 TF lookup failure는 detection drop으로 처리하는가? 진단 상태는 어디에 노출하는가?
6. `/fire/detections_3d`는 camera-frame 중간 topic인가, map-frame 변환 결과인가? 최종 VLA용 2D projection topic은 별도로 무엇인가?
7. timestamp clock 기준(ROS time/system time), QoS, 최대 허용 latency/rate는 무엇인가?

이 질문이 확정되기 전에는 VLA-03B를 구현하지 않으며, VLA가 bbox/depth/TF 책임을 가져오지 않는다.

## 18. VLA-04 Person Report boundary

```text
WorldModel person (reported=false)
→ REPORT_PERSON target stable ID
→ Validator (exists and not reported)
→ TopicBridgePersonReportAdapter
→ /vla/person_report
→ external consumer result
→ /vla/person_report_result
→ ActionResult(REPORT)
→ WorldModel
```

발행 payload는 `action_id`, `mission_id`, `person_id`, `map_position{x,y}`, `confidence`, ISO `timestamp`, `frame_id=map`이다. 위치와 confidence는 LLM text가 아니라 WorldModel이 source of truth다. Submission과 terminal result는 분리하며 correlated `SUCCEEDED`에서만 `reported=true`, `state=REPORTED`로 전이한다. 외부 UI/reporting backend 연결은 VLA-06 범위이고 VLA-03B Perception producer wiring은 여전히 pending이다.
