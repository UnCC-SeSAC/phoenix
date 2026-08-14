# Current VLA Data Architecture — 2D Map Contract

> 2026-08-12 최신 계약: 아래의 과거 VLA-03B 기록보다 이 절을 우선한다.
> image_pipeline source는 `origin/albitro/image_pipeline` @
> `a8caf2c9b45f35e66b0e3660ecad0ce8e422d719`이다. ROS package root는
> `src/image_pipeline`이며 구 `src/image_pipeline/ros/image_pipeline` 경로는 폐기됐다.

## VLA-07C 최신 image_pipeline 계약

```text
/yolo_result (vision_msgs/Detection2DArray)
+ depth0/image_raw + rgb0 CameraInfo
→ image_pipeline
→ /fire/detections (pixel + depth + source stamp)
+ /fire/detections/status (independent heartbeat)
→ VLA ROS Adapter (CameraInfo backprojection + source-time TF)
→ /vla/perception_observation (canonical map x/y)
→ stable ID fallback → SemanticObservation → WorldModel
```

`/fire/detections`는 envelope 하나에 여러 detection을 담는다. `x/y`는 원본
rgb0 pixel이며 map 좌표가 아니다. `stamp_sec/nanosec`는 publish time이 아닌
원본 detection/source image stamp다. Adapter는 이 정확한 sec/nanosec로 TF를
조회하며 `now()` 또는 float seconds로 바꾸지 않는다. Upstream `score`는 scaling
없이 canonical `confidence`로 1:1 rename한다.

Depth status는 `ok`, `unknown`, `fallback_bottom`, `fallback_below`,
`fallback_ring`이다. `unknown`은 numeric depth가 잘못 포함돼도 fail-closed하고,
`fallback_*`은 positive finite depth만 투영하되 status provenance를 보존한다.
smoke는 MVP에서 ignore한다. VLA WorldModel은 계속 2D `map (x,y)`만 사용한다.

Heartbeat state는 `ok`, `stalled`, `waiting_camera_info`, `no_input`이다.
Detection event silence만으로 detector failure를 판단하지 않는다. Event가 없어도
heartbeat `ok`면 detector는 정상이고, 나머지 state만 health problem으로 mapping한다.

| 이전 가정 | 최신 upstream | 처리 |
|---|---|---|
| `/yolo/detections` String | `/yolo_result` Detection2DArray | image_pipeline이 소비 |
| `/vision/detections` flat JSON | `/fire/detections` envelope | VLA bridge 입력 변경 |
| upstream confidence/map x/y | score/pixel x/y | rename + CameraInfo/TF |
| one-message/one-detection | multi-detection envelope | batch one-to-one association |
| silence 기반 health 추론 | `/fire/detections/status` | 독립 health mapping |

이 절 아래의 `/vision/detections` 및 `origin/state_manage` 설명은 이전 구현 이력이며
현재 runtime contract가 아니다.

### YOLO producer update

최신 upstream은 `/image_enhanced`를 구독하고 `/yolo_result`
`vision_msgs/Detection2DArray`를 발행하는 `yolo_node`를 제공한다. 원본 image
header의 stamp/frame_id를 그대로 보존하고 sensor-data QoS를 사용한다. 모델 입력
letterbox를 검출 후 되돌린 좌표는 enhanced image 기준이며, detection fusion이
CameraInfo로 원본 rgb0 pixel을 복원한다.

Production 기본은 모델 확장자에 따른 ONNX 또는 명시적 backend 선택이다. 모델이
없거나 지원되지 않으면 startup이 실패하며 test stub으로 자동 fallback하지 않는다.
`backend=stub`은 `full_chain_check.launch.py` 등 software wiring 검증에서만
명시적으로 사용한다. 실제 model class order와 output layout은 hardware gate 전에
학습 metadata 및 실제 tensor shape로 확정해야 한다.

분석 기준:

- VLA branch: `feature/vla-brain` @ `07fc86d1b4de1d46aa897252ee625d214d49bda0`
- Live Perception source: `origin/state_manage` @ `e432bed33af50861aaf4a501212d014c6c0bc9d3`
- Legacy image-processing reference: `origin/albitro/image_processing` @ `0bf507edc801086bb90ecbff7e5a4eabf5662363`
- 분석일: 2026-08-10
- 관련 Issue: #25, #28, #29, #30, #31, #33, #41

상태 표기:

- `[IMPLEMENTED/VERIFIED]`: 현재 VLA branch 코드와 software test/smoke에서 확인됨
- `[AGREED UPSTREAM CONTRACT]`: 팀과 합의된 최신 producer 계약이며 live hardware 검증은 남음
- `[HARDWARE PENDING]`: 실제 camera/YOLO/TF/Robot 환경에서 확인이 필요함

## 1. 결론

현재 시스템의 최종 지도와 Navigation 공간은 **2D SLAM / 2D Nav2의 `map` frame**이다.

> Depth 기반 camera-frame 3D point는 객체의 2D map 위치를 얻기 위한 Perception 내부 중간 표현이다. 시스템이 3D map을 사용한다는 의미가 아니다.

VLA의 authoritative semantic target position은 `map` frame의 `(x, y)`이다. VLA-03A는 canonical map-frame 검증과 upstream ID 우선/fallback stable ID association을 구현했다. VLA-03B는 `origin/state_manage`의 추적 가능한 `vision_detector.py` 계약을 기준으로 confidence/source timestamp가 보존된 `/vision/detections`를 canonical boundary에 연결했다. 실제 YOLO/camera/depth/TF hardware feed는 아직 검증하지 않았다.

## 2. 전체 시스템 아키텍처

```text
RGB / Depth source frame
        ↓
YOLO raw detection — /yolo/detections (std_msgs/String)
class_name, score, representative pixel x/y, frame_size,
depth, depth_status, shared source stamp_sec/nanosec
        ↓
thin score→confidence mapping → VisionDetector — CameraInfo + camera optical-frame point + TF
        ↓
/vision/detections (std_msgs/String)
class, confidence, map x/y, frame_id=map, source stamp
        ↓
VLAPerceptionBridgeNode
        ↓
/vla/perception_observation (canonical std_msgs/String JSON)
        ↓
CanonicalPerceptionNormalizer — VLA-03A stable-ID fallback
        ↓
SemanticObservation → WorldModel → Qwen/Resolver/Validator/Dispatcher
```

SLAM/TF/Localization/Nav2는 공통 2D navigation infrastructure다. Depth로 계산한 camera-frame `(X,Y,Z)`는 map-frame 2D object position을 얻기 위한 Perception 내부 중간값이며 VLA Domain이나 3D map으로 전달하지 않는다.

## 3. Live Perception upstream contract

### 3.1 YOLO raw detection `[AGREED UPSTREAM CONTRACT]`

Topic/type: `/yolo/detections`, `std_msgs/msg/String` JSON

```json
{
  "class_name": "person",
  "score": 0.93,
  "x": 320,
  "y": 240,
  "frame_size": [640, 480],
  "depth": 2.1,
  "depth_status": "ok",
  "stamp_sec": 1786329608,
  "stamp_nanosec": 489463639
}
```

| Field | 의미 | VLA WorldModel 전달 |
|---|---|---|
| `class_name` | `person`, `fire`, `smoke` | person/fire만 전달; smoke ignore |
| `score` | YOLO score `[0,1]` | scaling 없이 `confidence`로 이름만 mapping |
| `x`, `y` | representative image pixel | No |
| `frame_size` | pixel 좌표 기준 `[width,height]` | No |
| `depth` | representative pixel의 meter depth 또는 null | No |
| `depth_status` | `ok`, `fallback_bottom`, `fallback_below`, `fallback_ring`, `unknown` | No |
| `stamp_sec/nanosec` | 원본 RGB/depth source-frame timestamp | UTC ISO로 변환 후 Yes |

`x/y`는 map 좌표가 아니다. `frame_size`, raw depth/status, camera-frame point는 Perception metadata이므로 WorldModel에 저장하지 않는다. 한 source frame에서 여러 객체가 검출되면 detection별 String message를 발행하더라도 모두 같은 `stamp_sec/nanosec`를 공유한다. 각 객체의 class/score/pixel/depth/status는 독립적이다. Production YOLO는 아직 미구현이며 VLA Core 계약은 `confidence`로 유지한다.

현재 upstream은 stable tracking ID를 제공하지 않는다. VLA-03A가 same-class nearest map position, radius 0.5 m, TTL 2.0초, batch one-to-one 방식으로 process-local ID를 연결한다. 이는 full MOT가 아니며 빠른 이동, 근접 교차, 긴 occlusion에서 ID switch가 가능하다.

### 3.2 Depth validity

| `depth_status` | 합의된 처리 |
|---|---|
| `ok` | 유효한 meter depth이면 transform |
| `fallback_bottom` / `fallback_below` / `fallback_ring` | 유효한 meter depth이면 transform |
| `unknown` | `depth=null`; detection drop |

현재 `vision_detector.py`는 envelope의 source timestamp를 프레임 전체에 적용하고 detection별 confidence를 보존한다. `unknown` 또는 미지원 `depth_status`, null/비수치/비양수 depth, invalid confidence/timestamp/intrinsics, TF failure는 모두 fail-closed로 drop한다. `/vision/detections`는 같은 source frame의 유효한 map detection들을 하나의 batch로 발행한다.

추가로 depth `<=0`/NaN/Inf, invalid CameraInfo/intrinsics, malformed detection, invalid confidence/timestamp, TF failure는 map detection을 발행하지 않는다. 과거 map coordinate를 재사용하지 않는다.

`origin/state_manage`의 현재 `yolo_detector.py`는 dummy detector 기반 이전 구현이므로 새 `frame_size`/`depth_status` 계약을 live publisher로 재검증한 결과는 아니다. 이 문서는 합의된 최신 upstream contract와 VLA consumer 구현 상태를 구분한다.

## 4. Map output과 VLA-03B bridge

### 4.1 VisionDetector → `/vision/detections` `[IMPLEMENTED]`

```json
{
  "class": "person",
  "confidence": 0.93,
  "x": 2.4,
  "y": 1.7,
  "frame_id": "map",
  "stamp_sec": 1786329608,
  "stamp_nanosec": 489463639
}
```

기존 deterministic consumer가 쓰는 `class/x/y/frame_id`를 유지하고 confidence와 원본 timestamp를 additive하게 보존한다. pixel, `frame_size`, depth/status, camera-frame Z는 이 boundary를 넘지 않는다.

### 4.2 VLA Perception Bridge `[IMPLEMENTED/VERIFIED]`

`src/fire_vla_core/fire_vla_core/ros/perception_bridge_node.py`는 `/vision/detections` 한 건을 `/vla/perception_observation`의 detections 길이 1 canonical batch로 변환한다. person/fire만 전달하고 smoke는 ignore한다. tracking, stable ID 생성, WorldModel 직접 수정, Qwen/navigation 판단은 하지 않는다.

```text
/vision/detections
→ VLAPerceptionBridgeNode
→ /vla/perception_observation
→ CanonicalPerceptionNormalizer
→ VLA-03A stable ID fallback
→ SemanticObservation → WorldModel
```

VLA-03B software integration은 구현 완료했다. 189 tests, `fire_vla_core`/`fire_vla_bringup`/`uncc_example` build와 deterministic ROS smoke에서 `/vision/detections → bridge → WorldModel → /vla/status`를 확인했다. 실제 YOLO model, RGB/depth camera, CameraInfo sync, live camera→map TF, QoS/rate/latency는 `[HARDWARE PENDING]`이다.

## 5. 전체 시스템 boundary Input / Output

| Producer → Consumer | ROS boundary | Type | Payload/coordinate/timestamp | 상태 |
|---|---|---|---|---|
| RGB/depth → YOLO | camera-specific image topics | `sensor_msgs/Image` | shared source frame/stamp | `[HARDWARE PENDING]` |
| YOLO → VisionDetector | `/yolo/detections` | `std_msgs/String` JSON | class/confidence/pixel/frame_size/depth/status/source stamp | `[AGREED UPSTREAM CONTRACT]` |
| VisionDetector → VLA bridge | `/vision/detections` | `std_msgs/String` JSON | person/fire, confidence, map x/y, source stamp | `[IMPLEMENTED; HARDWARE PENDING]` |
| VLA bridge → VLA | `/vla/perception_observation` | `std_msgs/String` JSON | canonical map-frame batch, UTC ISO timestamp | `[IMPLEMENTED/VERIFIED]` |
| Mission UI/operator → VLA | `/vla/mission` | `std_msgs/String` JSON | mission ID/text | `[IMPLEMENTED/VERIFIED]` |
| VLA → Firefighter UI | `/vla/status` | `std_msgs/String` JSON | WorldModel + DecisionCycle metadata | `[IMPLEMENTED/VERIFIED]` |
| TF bridge → VLA | `/vla/robot_pose_json` | `std_msgs/String` JSON | robot map Pose2D/Unix time | `[IMPLEMENTED]` |
| VLA ↔ Navigation bridge | `/vla/navigation_goal/result/cancel` | `std_msgs/String` JSON | correlated map goal/result/cancel | `[IMPLEMENTED/VERIFIED without real Nav2]` |
| VLA ↔ Pump bridge | `/vla/spray_command/result/cancel` | `std_msgs/String` JSON | correlated spray lifecycle | `[IMPLEMENTED VLA boundary; hardware pending]` |
| VLA ↔ Report consumer | `/vla/person_report/result` | `std_msgs/String` JSON | authoritative person/report result | `[IMPLEMENTED VLA boundary]` |

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
| topic | `/vision/detections` | `/vla/perception_observation` | VLA-03B complete | optional thin bridge가 mapping |
| ROS type | `std_msgs/String` JSON | `std_msgs/String` JSON | Compatible | bridge에서 schema만 변환 |
| class | `class`: person/fire/smoke | `class_name`: person/fire | VLA-03B complete | person/fire 전달, smoke ignore |
| confidence | `confidence` `[0,1]` | `confidence`, person≥0.50/fire≥0.60 | Compatible | 값 보존 |
| representative pixel/frame_size | raw `x/y`, `[width,height]` | 사용하지 않음 | Expected | Perception transform metadata로만 사용 |
| entity ID | 없음 | optional upstream ID + VLA fallback | VLA-03A complete | upstream ID 우선, 없으면 2D association |
| map_x | `/vision/detections.x` | `map_position.x` 필수 | VLA-03B complete | bridge mapping |
| map_y | `/vision/detections.y` | `map_position.y` 필수 | VLA-03B complete | bridge mapping |
| yaw | 없음 | optional, 기본 0; Resolver가 target 방향 재계산 | Compatible | upstream에서 억지로 생성할 필요 없음 |
| frame_id | explicit `map` | explicit `map` 필수 | VLA-03B complete | non-map drop |
| timestamp | `stamp_sec` + `stamp_nanosec` | timezone-aware ISO batch timestamp 필수 | VLA-03B complete | UTC ISO nanosecond 문자열 변환 |
| source frame | VisionDetector parameter로 고정 | canonical은 `map`만 허용 | Compatible | raw frame은 VLA Domain에 전달하지 않음 |
| invalid depth/status | nonnumeric/null/`<=0`/NaN/Inf drop | finite map position 요구 | VLA-03B complete | `unknown+null` drop 확인; status-string consistency는 hardware pending |
| TF failure | warning 후 해당 detection drop | map position을 신뢰 | Compatible | old coordinate 재사용 없음 |
| stale observation | 원본 sec/nanosec 보존 | 1.0초 초과 batch drop | VLA-03B complete | bridge가 UTC ISO로 변환 |
| person | 합의된 raw class 및 bridge mapping | 정식 WorldModel entity | VLA-03B complete | live model output은 hardware pending |
| fire | 합의된 raw class 및 bridge mapping | 정식 WorldModel entity | VLA-03B complete | live model output은 hardware pending |
| smoke | 합의된 raw class | MVP semantic entity 아님 | Explicit ignore | WorldModel/Action/Qwen/UI 확장 없음 |
| NaN/Inf | VisionDetector와 canonical boundary에서 drop | finite 좌표만 허용 | VLA-03B complete | fail-closed |
| map bounds | 동작 미확인 | ingest 시 없음, ActionValidator만 ±100m | Contract gap | 실제 map bounds owner/정책 확인 |

## 8. Entity ID ownership

현재 합의된 upstream contract에는 tracking ID가 없다. VLA-03A canonical boundary는 non-empty upstream ID를 우선 보존하고, ID가 없을 때 같은 class의 최근 map position으로 fallback ID를 연결한다.

현재 MVP 계약:

1. upstream stable ID가 있으면 VLA가 opaque identifier로 보존한다.
2. ID가 없으면 same-class, nearest 2D distance, radius 0.5 m, TTL 2.0초로 association한다.
3. batch one-to-one이며 거리 동률은 entity ID로 결정한다.
4. 새 ID는 `person_0001`, `fire_0001` 형식의 process-local ID다.
5. 빠른 이동, 근접 교차, 긴 occlusion, 장시간 재탐지, process restart에서는 ID switch가 가능하다. ByteTrack/Kalman 등 tracker가 아니다.

## 9. Map coordinate ownership

| 단계 | Owner | 계약 |
|---|---|---|
| YOLO class/confidence/representative pixel/frame_size | Perception | image-space detection |
| representative pixel depth/status | Perception | valid meter depth 또는 `unknown`/null |
| camera-frame `(X,Y,Z)` | Perception 내부 | 중간 계산, 3D map 데이터 아님 |
| camera/source → map TF | Perception + TF infrastructure | original image timestamp 기준 변환 |
| final object map `(x,y)` | Perception + TF/2D SLAM | VLA boundary의 authoritative target location |
| semantic state 저장 | VLA WorldModel | stable entity ID별 Pose2D/state |
| target ID → pose | VLA TargetResolver | WorldModel pose 사용, LLM 좌표 생성 금지 |
| target yaw | VLA TargetResolver | robot pose에서 target `(x,y)`를 향하는 yaw 계산 |
| path planning/driving | 2D Nav2 | map-frame Pose2D goal |

VLA는 bbox, raw depth, camera intrinsics, camera-frame z, 3D TF projection을 소유하지 않는다.

## 10. Stale / invalid 책임 경계

### VisionDetector에서 drop하는 것 `[IMPLEMENTED; LIVE HARDWARE PENDING]`

- `depth_status="unknown"` + `depth=null`, nonnumeric/0/negative/NaN/Inf depth로 유효한 object position을 계산할 수 없는 detection
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

`SEARCH`와 `RETURN_HOME`은 별 Port가 아니라 `NavigationPort`로 dispatch된다. Production ROS boundary는 Topic Bridge Navigation, Person Report, Spray에 구현되어 있다. Wait는 현재 mock adapter다.

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
| EXTINGUISH | fire stable ID | target ID, no target pose | ACTIVE, within spray range, attempts limit | SprayPort → `/vla/spray_command`; result on `/vla/spray_result` |
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

## 17. VLA-03A 및 VLA-03B 완료

VLA-03A canonical validation, stable ID fallback과 WorldModel snapshot 경로에 이어 VLA-03B thin bridge를 구현/검증했다. upstream raw contract는 person/fire/smoke, confidence, representative pixel x/y, frame_size, depth/status와 source stamp를 정의한다. `vision_detector.py`는 person/fire의 pixel/depth/source stamp를 CameraInfo와 TF로 map `(x,y)`로 변환하며 `/vision/detections` String을 발행한다. confidence와 source sec/nanosec를 backward-compatible field로 보존하고 invalid depth, invalid intrinsics, malformed input, TF failure는 발행하지 않는다. 동일 source frame의 multiple detections는 같은 timestamp를 유지한다.

`vla_perception_bridge`는 `/vision/detections` 한 건을 canonical detections 길이 1 batch로 변환하며 pixel/frame_size/depth/status는 WorldModel로 전달하지 않는다. person/fire만 전달하고 smoke는 무시하며 stable ID는 생성하지 않고 기존 VLA-03A fallback에 맡긴다. deterministic ROS smoke는 완료했지만 실제 YOLO model, camera/depth, TF hardware pipeline은 실행하지 않았다. VLA는 계속 bbox/depth/deprojection/TF 책임을 가져오지 않는다.

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

발행 payload는 `action_id`, `mission_id`, `person_id`, `map_position{x,y}`, `confidence`, ISO `timestamp`, `frame_id=map`이다. 위치와 confidence는 LLM text가 아니라 WorldModel이 source of truth다. Submission과 terminal result는 분리하며 correlated `SUCCEEDED`에서만 `reported=true`, `state=REPORTED`로 전이한다. VLA-06 UI의 report 상태 표시는 구현 완료했다. 실제 외부 reporting consumer는 pending이며 VLA-03B bridge wiring은 완료했다.

## 19. VLA-05 Spray boundary

```text
WorldModel ACTIVE/in-range fire
→ EXTINGUISH stable fire ID
→ Validator
→ TopicBridgeSprayAdapter
→ /vla/spray_command
→ future Pump/MCU bridge
→ /vla/spray_result
→ ActionResult(SPRAY)
→ WorldModel PENDING_VERIFICATION
```

Command payload는 `action_id`, `mission_id`, authoritative `fire_id`, `command=SPRAY`, ISO timestamp다. `/vla/spray_cancel`은 active action ID를 전달하고 terminal CANCELED 결과 전에는 lifecycle을 완료하지 않는다. SUCCEEDED도 실제 진압 완료가 아니라 suppression verification 시작을 뜻한다. VLA-side ROS boundary는 구현/검증 완료이며 실제 Pump driver, MCU firmware, duration/intensity와 hardware ack 계약은 pending/upstream이다.

## 20. VLA-06 Firefighter UI boundary

```text
Browser POST /api/mission
→ FirefighterUINode → /vla/mission
→ VLAOrchestrator
→ WorldModel snapshot + latest DecisionCycle
→ /vla/status
→ FirefighterUINode → GET /api/status → Browser
```

`/vla/status`는 `timestamp`, existing `world_model` snapshot, `decision`, `validation`, `submission`, `blocked_reason`을 제공한다. UI backend는 thread-safe copy를 HTTP thread에 전달하며 loopback에만 bind한다. Frontend는 Mission, robot/person/fire, execution/safety 상태와 robot/home/person/fire/current target의 auto-fit 2D semantic overlay를 표시한다. UI는 Action을 직접 생성하거나 Validator, Navigation, Report, Spray Port를 우회하지 않는다. occupancy grid, 인증, DB, 외부 공개는 범위 밖이다. VLA-03B bridge는 완료했으며 실제 YOLO/camera/depth/TF hardware feed 미검증은 UI의 canonical status contract에 영향이 없다.

## 21. VLA-07 live boundary verification

실제 PC/Jazzy ↔ Pi/Humble DDS에서 `/vla/navigation_goal`, `/vla/navigation_cancel`, `/vla/navigation_result`, `/vla/robot_pose_json` neutral boundary를 확인했다.

`/vla/robot_pose_json`의 TF-derived map pose가 PC Orchestrator WorldModel과 `/vla/status`에 반영됐다. 이 pose는 one-shot 값이 아니라 short-nav preflight 동안 연속 fresh stream이어야 한다.

Production YOLO는 pending이다. 실제 producer의 `score`는 가장 얇은 연결 boundary에서 scaling 없이 VLA의 `confidence`로 이름만 mapping한다.

첫 motion preflight에서는 Mock person (0.5,0.0)과 인명을 우선 확인해 Mission을 사용했지만 stale robot pose와 unexpected Mock `RETURN_HOME`으로 SAFE ABORT했다. 이는 software-only preflight 문제다.
