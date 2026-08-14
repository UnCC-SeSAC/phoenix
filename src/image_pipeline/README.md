# image_pipeline

화재 탐사 로봇의 **RGB 전처리(태스크①) + YOLO 검출 + 거리 측정(태스크②)** 패키지.
팀원4 담당.

```
rgb0/image ──▶ [감마 → 디헤이즈 → CLAHE] ──▶ /image_enhanced ──▶ [YOLO26] ─┐
                                                                           ├─▶ /fire/detections
depth0/image_raw ──────────────────────────────────────────────────────────┘   (JSON: 픽셀 x,y + depth[m])
```

**역투영과 `base_link`/`map` 변환은 메인이 합니다.** 이 패키지가 내는 것은
원본 `rgb0` 픽셀 좌표와 거리 스칼라까지입니다 (2026-08-10 계약).

## 노드

| 실행 파일 | 역할 |
|---|---|
| `preprocess_node` | 태스크① — 감마·디헤이즈(DCP/AOD-Net)·CLAHE |
| `yolo_node` | YOLO26 추론 (`.pt`/Ultralytics 개발 PC, ONNX/OpenCV 백엔드) |
| `detection_3d_node` | 태스크② — 박스 + 뎁스 → 거리 → JSON 발행 |
| `fake_detection_node` | 로봇·YOLO 없이 돌려보기 위한 더미 (정답을 로그에 찍음) |
| `fake_camera_node` | 로봇 없이 쓰는 가짜 카메라 |

## 토픽

| | 토픽 | 타입 |
|---|---|---|
| ① 구독 | `/ascamera/camera_publisher/rgb0/image` | `Image` bgr8 640×480 @15fps |
| ① 구독 | `/ascamera/camera_publisher/rgb0/camera_info` | `CameraInfo` |
| ① 발행 | `/image_enhanced` (+ `/camera_info`) | `Image` bgr8 |
| YOLO | `/image_enhanced` → `/yolo_result` | `vision_msgs/Detection2DArray` |
| ② 구독 | `/yolo_result`, `depth0/image_raw`(16UC1 mm), `camera_info` 3종 | |
| ② 발행 | `/fire/detections` | `std_msgs/String` (JSON, **검출이 있을 때만**) |
| ② 발행 | `/fire/detections/status` | `std_msgs/String` (하트비트, 1초 주기) |

★ **RGB는 `/image`이고 뎁스만 `/image_raw`입니다.** 여기서 틀리면 콜백이 안 불립니다.
★ 구독·발행 모두 `qos_profile_sensor_data`입니다. 정수 큐를 쓰면 **조용히 실패**합니다.

발행 JSON:

```json
{
  "stamp_sec": 1786329608, "stamp_nanosec": 489463639,
  "frame_size": [640, 480],
  "detections": [
    {"class_name": "fire", "score": 0.87, "x": 320, "y": 240,
     "depth": 2.0, "depth_status": "ok"}
  ]
}
```

`depth_status`: `ok` / `fallback_bottom` / `fallback_below` / `fallback_ring` / `unknown`.
**거리 불명은 `depth: null`입니다** — `0.0`이 나가면 받는 쪽이 로봇 발밑을 화재
지점으로 계산합니다. `stamp`는 **원본 이미지의 것**이지 발행 시각이 아닙니다.

## 로봇 없이 확인하기

```bash
colcon build --packages-select image_pipeline && source install/setup.bash

# 카메라 입력부터 최종 JSON 발행까지 전 구간 (더미 + YOLO 스텁)
ros2 launch image_pipeline full_chain_check.launch.py
ros2 topic echo /fire/detections --field data --once
#   -> {"x":320,"y":240,"depth":3.2,"depth_status":"ok"}  (더미 로그의 [정답]과 일치)

# 태스크②만
ros2 launch image_pipeline dummy_check.launch.py
ros2 launch image_pipeline dummy_check.launch.py flame_hole:=true   # depth:null 이 정상
```

ROS 없이 도는 테스트 341개:

```bash
python3 -m pytest tests/ -q
```

## Phoenix production YOLO 가중치

Hugging Face `albitro/phoenix_detection` revision
`eed14671dfc1d32a26710594d865da03dbca664f`에는 YOLO26s 가중치 두 종류가 있다.
Production 전처리 체인(`full`: gamma → DCP → CLAHE)에는 `filtered/best.pt`를
사용하며, weight는 Git이 아닌 외부 artifact cache에 둔다. SHA-256은
`a09bc762875f61b0a822231ae12f15aeedb0408d62fcb03d9672c97a800ee989`,
입력 크기는 640, class 순서는 `fire=0, person=1`이다(`smoke` class 없음).

```bash
python3 tools/detect_offline.py --model /artifact/filtered/best.pt \
  --preprocess full --names fire person 사진.jpg -o out/
ros2 launch image_pipeline yolo.launch.py \
  model_path:=/artifact/filtered/best.pt class_names:="['fire', 'person']" imgsz:=640
```

`.pt` backend는 Ultralytics가 postprocess한 `Results`(`xyxy`, score, class id)를
사용하므로 ONNX용 `layout` 인자는 적용되지 않는다. 추후 ONNX export를 사용할
때는 실제 출력을 확인한 뒤 `layout:=v8` 또는 `end2end`로 고정한다.

`.hef`(Hailo) 백엔드는 **일부러 비워 뒀습니다** — 스트림 이름·양자화 스케일이
컴파일마다 달라 추측으로 채우면 "도는데 좌표가 틀린" 코드가 됩니다.
받아야 할 항목은 `image_pipeline/yolo.py`의 `HailoBackend` docstring에 있습니다.

## 원본 저장소

이 패키지는 개발 저장소에서 복사해 옵니다. **설계 근거·함정 기록·AOD-Net 학습
코드는 그쪽에 있습니다** (로봇에 올라갈 필요가 없어 여기엔 넣지 않습니다).

| 문서 | 내용 |
|---|---|
| `CLAUDE.md` | 규칙·금지사항 |
| `HANDOVER.md` | 배경, 설계 결정, 이미 밟은 지뢰, 다음 작업 |
| `docs/코드_워크스루.md` | 전 구간 동작 순서 (파일·함수·줄 번호) |
| `docs/메인_인계_주의사항.md` | 메인이 알아야 할 함정 5건 |
