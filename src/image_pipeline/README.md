# image_pipeline

화재 탐사 로봇의 영상 파이프라인 — **태스크① RGB 전처리 + 태스크② 검출 3D 좌표**
(SeSAC Intel Physical AI 최종 프로젝트, 팀원4 담당).

```
① rgb0/image ──▶ [감마 → 디헤이즈 → CLAHE] ──▶ /image_enhanced ──▶ [YOLO26] ─┐
                                                                              ├─▶ ② ──▶ /fire/detections
   depth0/image_raw ──────────────────────────────────────────────────────────┘      (JSON: 픽셀 x,y + depth[m])
```

역투영과 `base_link`/`map` 변환은 **메인(팀원1)이 합니다** (2026-08-10 계약 개정).
우리가 내는 것은 원본 `rgb0` 픽셀 좌표와 거리 스칼라까지입니다.

**뎁스는 ①과 YOLO를 거치지 않습니다.** 두 갈래를 다시 잇는 것은 이미지가 아니라
`stamp`입니다. 드라이버가 rgb·depth를 같은 stamp로 발행하므로, 헤더만 그대로
승계하면 ②에서 `message_filters`로 맞출 수 있습니다.

## 폴더

```
image_pipeline/
├── ros/image_pipeline/     ROS2 패키지. 로봇에 올라감
│   ├── image_pipeline/     계산 모듈 + 노드
│   ├── tests/              341개 (rclpy 없이 돌아감)
│   ├── tools/              오프라인 튜닝·비교·미리보기
│   └── launch/  config/  models/
├── training/               로봇에 안 올라감
│   ├── aodnet/             AOD-Net 학습·평가
│   └── frame_cut/          영상 → 프레임 추출
└── docs/                   배경 문서
```

`ros/image_pipeline/`이 주 작업 위치입니다. **아래 명령은 전부 거기서** 실행하세요.

## 빨리 확인하기

```bash
cd ros/image_pipeline
python3 -m pytest tests/ -q      # 341개
bash run_local_check.sh          # 합성데이터 → 테스트 → 비교이미지 → 벤치 → ② 장면
```

## 가상 데이터로 돌려보기 (로봇·YOLO 없이)

이 PC에 ROS2 Jazzy가 있어 **실제로 노드가 돕니다.** 저장소 최상위에서 한 번 빌드:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros --symlink-install
source install/setup.bash
```

`--symlink-install`이라 이후 파이썬 코드를 고쳐도 **다시 빌드할 필요가 없습니다.**

```bash
ros2 launch image_pipeline dummy_check.launch.py
```

```bash
# 다른 터미널
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 topic echo /fire/detections --once
ros2 topic hz /fire/detections/status
```

더미가 시작 로그에 **`[정답]`** 좌표를 찍습니다. 발행된 좌표와 비교하세요 —
기본 설정이면 `"depth": 3.200, "depth_status": "ok"` 이고, 참고로 찍히는
메인 계산분이 `(+3.300, +0.000, +0.350)` 입니다.

| 명령 | 무엇을 보는가 |
|---|---|
| `dummy_check.launch.py` | 기본. `[정답]`과 발행 좌표가 일치해야 정상 |
| `... flame_hole:=true` | 화염으로 뎁스가 빔 → **발행 0건**이 정상. 좌표를 지어내면 안 됨 |
| `... flame_hole:=true fallback_regions:="bottom,below,ring"` | 폴백. 값이 나오되 `score`가 절반으로 표시됨 |
| `... floor_height_m:=0.35 flame_hole:=true` | 바닥면 장면. 5-1 폴백 3종 비교 |
| `... break_stamp_sec:=0.02` | stamp를 일부러 깨뜨림 → `StampMonitor`가 ERROR를 냄 |
| `... distance_m:=2.0` | 카메라에 유리한 거리 (스펙 0.2~4m) |

노드 통계는 5초마다 `검출 N Hz | 뎁스 N Hz | 동기화 N Hz | 발행 N개 | 제외: ...` 로
나옵니다. 셋을 비교하면 문제가 어디인지 좁혀집니다.

> ⚠ 더미에서 동기화율이 검출보다 낮게 나오는 건 **파이썬 rclpy가 0.61MB 뎁스
> 이미지를 15Hz로 못 밀어내기 때문**입니다(약 11Hz). 같은 노드에서 작은
> `camera_info`는 15.0Hz가 나옵니다. **실기 드라이버는 C++이라 해당 없습니다.**

ROS 없이 그림으로 보려면:

```bash
python3 tools/preview_depth_scene.py --box-y 60 -o out/
python3 tools/preview_depth_scene.py --floor 0.35 --flame-hole    # 5-1 폴백 비교
```

## 전 구간 한 번에 (더미, 모델 없이) ★

**카메라 프레임 입력부터 최종 JSON 발행까지** 네 노드가 실제로 이어져 돕니다.

```bash
ros2 launch image_pipeline full_chain_check.launch.py
ros2 topic echo /fire/detections --field data --once
```

```
fake_detection_node ─rgb0/image─▶ preprocess_node ─/image_enhanced─▶ yolo_node
        │                            (태스크①)                   (backend=stub)
        └─depth0 + camera_info 3종──▶ detection_3d_node ◀──/yolo_result
                                          (태스크②)  └─▶ /fire/detections
```

| 인자 | 나와야 하는 값 |
|---|---|
| (없음) | `{"x":320,"y":240,"depth":3.2,"depth_status":"ok"}` |
| `enhanced_width:=320` | **같은 값.** x가 160이면 축소본 좌표를 안 되돌린 것 |
| `flame_hole:=true` | `"depth":null,"depth_status":"unknown"` |
| `flame_hole:=true fallback_regions:=ring` | `"depth":3.8,"depth_status":"fallback_ring"` |
| `haze:=0.35` | 같은 값 + 태스크① 로그에 디헤이즈 시간 |

`dummy_check.launch.py`와의 차이: 저건 더미가 박스를 직접 내므로 **②만** 돕니다.

⚠ `backend:=stub`은 사진을 **보지 않고** 항상 화면 정중앙에 박스를 놓습니다.
여기서 확인되는 건 배선·QoS·stamp·좌표계이고, **검출 성능은 아닙니다.**

## YOLO — 층별로 확인하기

추론 노드는 **코드가 이미 완성돼 있습니다.** 가중치가 없어 검출 정확도만
미검증이고, 나머지는 지금 직접 확인할 수 있습니다.

```bash
cd ros/image_pipeline

# 1) 계산 층 — 레터박스·출력 파싱·NMS (0.2초)
python3 -m pytest tests/test_yolo.py -q

# 2) 눈으로 — 스텁이 모델 입력 정중앙에 박스를 놓습니다.
#    레터박스를 제대로 되돌렸다면 결과가 **원본 이미지 정중앙**이어야 합니다.
python3 tools/detect_offline.py --stub smoke01.jpg -o out/
#    smoke01.jpg 는 420x315 -> center=(210, 158) 이 나오면 정상

# 3) ROS 층 — QoS·stamp·frame_id·박스 왕복 (pytest가 못 보는 곳)
source /opt/ros/jazzy/setup.bash && export PYTHONPATH=$PWD:$PYTHONPATH
python3 tools/check_yolo_wiring.py
```

`--stub`은 **검증 도구지 폴백이 아닙니다.** 모델이 없을 때 자동으로 스텁이
선택되지는 않습니다 — 그러면 "검출이 이상한 노드"가 되어 원인을 엉뚱한 데서
찾게 됩니다.

## YOLO 모델을 받으면 (2026-08-11 기준 대기 중)

붙이는 순서:

```bash
cd ros/image_pipeline

# 1) 출력 텐서 모양부터 눈으로. 자동 판별은 휴리스틱입니다
python3 tools/detect_offline.py --model models/fire_yolo26s.onnx --describe

# 2) 사진에 돌려보기 (ROS 불필요). --names 는 학습 때 순서 그대로
python3 tools/detect_offline.py --model models/fire_yolo26s.onnx \
    smoke01.jpg --names fire,person -o out/

# 3) 속도 (RPi5에서 imgsz 정할 때. 15fps = 프레임당 66.7ms를 ①과 나눠 씀)
python3 tools/detect_offline.py --model ... smoke01.jpg --bench 30

# 4) 노드로
ros2 launch image_pipeline yolo.launch.py \
    model_path:=models/fire_yolo26s.onnx class_names:="['fire','person']"
```

시작 로그의 `레이아웃=` 값을 확인한 뒤 `layout:=v8` 또는 `end2end`로 **못박으세요.**
남은 미검증 항목 6가지는 `HANDOVER.md` 7-6에 순서대로 있습니다.

`.hef`(Hailo) 백엔드는 **일부러 비워 뒀습니다** — 스트림 이름·양자화 스케일이
컴파일마다 달라 추측으로 채우면 "도는데 좌표가 틀린" 코드가 됩니다.
팀원5에게 받아야 할 것은 `image_pipeline/yolo.py`의 `HailoBackend` docstring에.

## 실기 배선

| | 토픽 |
|---|---|
| ① 구독 | `/ascamera/camera_publisher/rgb0/image` (bgr8 640×480 @15fps) |
| ① 구독 | `/ascamera/camera_publisher/rgb0/camera_info` |
| ① 발행 | `/image_enhanced`, `/image_enhanced/camera_info` |
| YOLO 구독 | `/image_enhanced` |
| YOLO 발행 | `/yolo_result` (`vision_msgs/Detection2DArray`, 축소본 좌표) |
| ② 구독 | `/yolo_result` (`vision_msgs/Detection2DArray`) |
| ② 구독 | `/ascamera/camera_publisher/depth0/image_raw` (16UC1, mm) |
| ② 구독 | `depth0/camera_info`, `/image_enhanced/camera_info`, `rgb0/camera_info` |
| ② 발행 | `/fire/detections` (`std_msgs/String` JSON — 검출이 있을 때만) |
| ② 발행 | `/fire/detections/status` (하트비트, 1초 주기) |

**RGB는 `/image`이고 뎁스만 `/image_raw`입니다.** 여기서 틀리면 콜백이 안 불립니다.

## 읽는 순서

1. `CLAUDE.md` — 규칙과 금지사항 (짧습니다)
2. `HANDOVER.md` — 배경·설계 근거·이미 밟은 지뢰·다음 작업
3. `docs/` — 프로젝트 전체 맥락 (필요할 때만)

`docs/태스크2_작업지시서.md`는 RealSense를 전제로 쓰였고
`docs/인수인계_문서_v2.md`의 카메라 항목도 실물과 다릅니다.
차이와 확인 근거는 `HANDOVER.md` 5-2c·5-2d에 있습니다.
