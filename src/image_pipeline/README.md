# image_pipeline

화재 탐사 로봇의 영상 파이프라인 — **태스크① RGB 전처리 + 태스크② 검출 3D 좌표**
(SeSAC Intel Physical AI 최종 프로젝트, 팀원4 담당).

```
① rgb0/image ──▶ [감마 → 디헤이즈 → CLAHE] ──▶ /image_enhanced ──▶ [YOLO] ─┐
                                                                            ├─▶ ② ──▶ /fire/detections_3d
   depth0/image_raw ────────────────────────────────────────────────────────┘         (base_link, 미터)
```

**뎁스는 ①과 YOLO를 거치지 않습니다.** 두 갈래를 다시 잇는 것은 이미지가 아니라
`stamp`입니다. 드라이버가 rgb·depth를 같은 stamp로 발행하므로, 헤더만 그대로
승계하면 ②에서 `message_filters`로 맞출 수 있습니다.

## 폴더

```
image_pipeline/
├── ros/image_pipeline/     ROS2 패키지. 로봇에 올라감
│   ├── image_pipeline/     계산 모듈 + 노드
│   ├── tests/              224개 (rclpy 없이 돌아감)
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
python3 -m pytest tests/ -q      # 224개
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
ros2 topic echo /fire/detections_3d --once
ros2 topic hz /fire/detections_3d
```

더미가 시작 로그에 **`[정답]`** 좌표를 찍습니다. 발행된 좌표와 비교하세요 —
기본 설정이면 둘 다 `(+3.300, +0.000, +0.350)` 입니다.

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

## 실기 배선

| | 토픽 |
|---|---|
| ① 구독 | `/ascamera/camera_publisher/rgb0/image` (bgr8 640×480 @15fps) |
| ① 구독 | `/ascamera/camera_publisher/rgb0/camera_info` |
| ① 발행 | `/image_enhanced`, `/image_enhanced/camera_info` |
| ② 구독 | `/yolo_result` (`vision_msgs/Detection2DArray`) |
| ② 구독 | `/ascamera/camera_publisher/depth0/image_raw` (16UC1, mm) |
| ② 구독 | `/ascamera/camera_publisher/depth0/camera_info`, `/image_enhanced/camera_info` |
| ② 발행 | `/fire/detections_3d` (`Detection3DArray`, `base_link`, 미터) |

**RGB는 `/image`이고 뎁스만 `/image_raw`입니다.** 여기서 틀리면 콜백이 안 불립니다.

## 읽는 순서

1. `CLAUDE.md` — 규칙과 금지사항 (짧습니다)
2. `HANDOVER.md` — 배경·설계 근거·이미 밟은 지뢰·다음 작업
3. `docs/` — 프로젝트 전체 맥락 (필요할 때만)

`docs/태스크2_작업지시서.md`는 RealSense를 전제로 쓰였고
`docs/인수인계_문서_v2.md`의 카메라 항목도 실물과 다릅니다.
차이와 확인 근거는 `HANDOVER.md` 5-2c·5-2d에 있습니다.
