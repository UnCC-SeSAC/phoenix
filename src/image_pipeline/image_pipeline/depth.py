#!/usr/bin/env python3
"""
태스크② 뎁스 → 3D 좌표 계산 — ROS 비의존 순수 함수.

  2D 박스 + 뎁스 ──▶ [대표 거리] ──▶ [역투영] ──▶ optical 3D ──▶ [고정 TF] ──▶ base_link

노드(`detection_3d_node.py`)는 배선만 하고 계산은 전부 여기 있습니다.
이유는 `CLAUDE.md` "계산은 모듈에, 노드는 배선만" — rclpy 없이 pytest로
검증할 수 있어야 하기 때문입니다. 그리고 이 태스크의 사고는 **전부 조용합니다**:

  - depth 0(측정 실패)을 0m로 집계   -> 메인이 로봇 발밑을 화재 지점으로 계산
  - 축소본 좌표에 원본 K            -> 거리가 배율만큼 틀림 (에러 없음)
  - 뎁스에 선형보간                 -> 물체 경계에 존재하지 않는 거리가 생김

**단위 변환은 이 파일 한 곳에서만 합니다** (`to_meters`). 노드나 도구에서
`* 0.001`을 직접 쓰면 두 곳이 갈라지는 순간 조용히 틀립니다.

좌표계 규약(REP-103):
  optical frame  x 오른쪽, y 아래,  z 앞
  base_link      x 앞,     y 왼쪽,  z 위
`backproject`는 **optical frame**을 반환합니다. 축 변환은 TF(`to_base_link`)가
하므로 여기서 손으로 돌리지 마세요 — 두 번 돌아갑니다.
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

import cv2
import numpy as np

# ★ Angstrong Nuwa-HP60C(양안 스테레오)의 스펙 범위 0.2~4m.
# 이 밖의 값은 "먼 물체"가 아니라 쓰레기입니다 — 스테레오는 범위를 벗어나면
# 오차가 커지는 게 아니라 **틀린 대응점**을 잡아 그럴듯한 값을 냅니다.
# 지시서는 RealSense(0.15~10m)를 전제로 쓰였지만 실제 하드웨어는 HP60C입니다
# (`docs/인수인계_문서_v2.md` 2장). 노드에서 파라미터로 덮어쓸 수 있습니다.
DEFAULT_Z_MIN = 0.2
DEFAULT_Z_MAX = 4.0

# 박스 안에서 대표 거리를 뽑는 방식.
#   median  기본. 구멍·이상치에 강함.
#   min/p25 앞면 쪽으로 당김 (배경이 섞였을 때).
#   p75/max 뒷면 쪽으로 당김. **"below" 접지점 샘플링 전용**입니다 — 그 띠는
#           아래로 갈수록 가까워지므로 위쪽 꼬리가 곧 접지점입니다.
#           다른 영역에 쓰면 배경 비침을 그대로 채택합니다.
METHODS = ("median", "min", "p25", "p75", "max")
# 어느 픽셀에서 뽑을지. "center" 외에는 전부 화염 대응 폴백입니다 (지시서 5-1).
#   center  기본. 박스 중앙.
#   bottom  박스 **안**의 아래쪽. 화염이 박스를 다 채우지 않을 때.
#   below   박스 **바로 아래 바깥** 띠 = 불이 놓인 바닥. 5-1 1번의 정확한 구현.
#   ring    박스 주변 테두리. 최후 수단이고 **뒤로 편향**됩니다.
REGIONS = ("center", "bottom", "below", "ring")

# 폴백 순서 — `(region, method)`를 앞에서부터 시도해 **처음 성공한 값**을 씁니다.
# 순서 근거는 HANDOVER 8 "5-1 폴백 후보 정량 비교" (합성 바닥 장면 실측):
#
#   center/median  대상 그 자체. 편향 없음
#   bottom/median  화염이 박스를 다 안 채울 때 대상을 읽습니다 (오차 0.000m)
#   below/max      접지점. **max여야 합니다** — 띠가 아래로 갈수록 가까워지므로
#                  가장 먼 값이 접지점입니다 (median은 -0.326m, max는 -0.046m)
#   ring/median    최후 수단. 주변이 대상보다 뒤에 있어 **멀게** 편향 (+0.600m)
#
# ★ 뒤로 갈수록 틀립니다. 그래서 어느 단계가 나왔는지를 `region`으로 알 수 있어야
#   하고, 발행할 때 `depth_status`로 옮겨 신뢰도를 낮춰 표시합니다
#   (`detection_json.depth_status`). 전부 사유가 `ok`로 나오므로 사유로는 구분이
#   안 됩니다.
#
# ⚠ 이 순서는 **실측 전 잠정값**입니다. 실제 뎁스 노이즈와 "화염이 박스를 얼마나
#   채우는지"를 보고 바꿔야 합니다 (HANDOVER 8 P0). 노드 파라미터로 뺐습니다.
DEFAULT_CASCADE = (
    ("center", "median"),
    ("bottom", "median"),
    ("below", "max"),
    ("ring", "median"),
)

Box = tuple  # (x1, y1, x2, y2), 픽셀 좌표. x2/y2는 열린 경계로 취급합니다.


# ============================================================ 단위 변환 (§3-4)

def to_meters(depth, encoding: str | None = None,
              depth_scale: float | None = None) -> np.ndarray:
    """뎁스 raw 배열을 **미터 float32**로. 무효 픽셀은 `NaN`이 됩니다.

    ★ depth 0은 "0미터"가 아니라 **측정 실패**입니다(지시서 3-4). 그대로 평균
    내면 거리가 0쪽으로 끌려가고, 그 좌표는 로봇 발밑을 가리킵니다.
    그래서 0을 여기서 곧바로 NaN으로 바꿉니다 — 이후 어떤 집계를 써도
    실패 픽셀이 값처럼 섞이지 않습니다.

    단위 판정 우선순위 (앞이 이깁니다):
      1. `depth_scale`  — 드라이버가 알려준 값이 있으면 무조건 이것
      2. `encoding`     — "16UC1"/"mono16" = mm, "32FC1" = m
      3. dtype 추측     — 정수면 mm, 실수면 m

    2번이 3번보다 앞인 이유: **float32에 mm를 담아 발행하는 드라이버가
    실재합니다.** dtype만 보면 3000m짜리 장면이 만들어집니다.
    노드는 시작 시 `depth_unit_sanity()`로 한 번 검산하세요.

    무효로 처리하는 값: 0, 음수, NaN/inf, 그리고 **정수 dtype의 최댓값**
    (16UC1의 65535는 포화 표시라 값이 아닙니다).
    """
    arr = np.asarray(depth)
    integer = np.issubdtype(arr.dtype, np.integer)

    if depth_scale is not None:
        scale = float(depth_scale)
    elif encoding is not None:
        enc = encoding.strip().lower()
        if enc in ("16uc1", "mono16", "16sc1"):
            scale = 0.001
        elif enc in ("32fc1", "64fc1"):
            scale = 1.0
        else:
            raise ValueError(f"뎁스 encoding을 모르겠습니다: {encoding!r}")
    else:
        scale = 0.001 if integer else 1.0

    invalid = None
    if integer:
        # 포화값(예: 65535)은 "가장 먼 것"이 아니라 "못 쟀다"는 뜻입니다.
        invalid = arr >= np.iinfo(arr.dtype).max

    out = arr.astype(np.float32) * np.float32(scale)
    bad = ~np.isfinite(out) | (out <= 0.0)
    if invalid is not None:
        bad |= invalid
    out[bad] = np.nan
    return out


def depth_unit_sanity(depth_m, lo: float = 0.1, hi: float = 20.0) -> bool:
    """미터 배열의 중앙값이 실내에서 말이 되는 범위인지.

    `principal_point_sanity`의 뎁스판입니다. mm를 m로 착각하면 3000m가 나오고,
    m를 mm로 착각하면 3mm가 나옵니다. 둘 다 예외 없이 조용히 틀리므로
    **노드 시작 시 첫 프레임에서 한 번** 검사해 경고를 띄우세요.
    """
    vals = np.asarray(depth_m, dtype=np.float64)
    vals = vals[np.isfinite(vals) & (vals > 0)]
    if vals.size == 0:
        return False
    return bool(lo <= float(np.median(vals)) <= hi)


# ================================================================ 박스 기하

def clip_box(box, width: int, height: int):
    """박스를 이미지 안으로 자르고 정수 슬라이스 경계로 변환. 밖이면 `None`.

    YOLO 박스는 이미지 경계를 자주 넘습니다(특히 화면 끝의 불). 자르지 않으면
    numpy가 조용히 빈 배열을 주고, 거리 계산이 NaN이 됩니다.

    경계는 floor/ceil로 **바깥쪽으로** 넓힙니다. 성냥불처럼 몇 픽셀짜리
    박스에서 반올림 때문에 픽셀이 0개가 되는 걸 막기 위해서입니다.
    """
    x1, y1, x2, y2 = (float(v) for v in box)
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1

    ix1 = max(0, int(np.floor(x1)))
    iy1 = max(0, int(np.floor(y1)))
    ix2 = min(int(width), int(np.ceil(x2)))
    iy2 = min(int(height), int(np.ceil(y2)))
    if ix2 <= ix1 or iy2 <= iy1:
        return None
    return (ix1, iy1, ix2, iy2)


def box_from_center(cx: float, cy: float, w: float, h: float):
    """center+size -> (x1,y1,x2,y2). `vision_msgs/Detection2D`가 이 형식입니다."""
    return (cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0)


def box_center(box):
    x1, y1, x2, y2 = (float(v) for v in box)
    return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)


def scale_box(box, sx: float, sy: float):
    """같은 카메라·같은 화각을 **해상도만 바꿔** 옮길 때 (예: 전처리 축소).

    ⚠ **컬러↔뎁스에는 쓰면 안 됩니다.** 축 배율만으로 옮기는 건 두 이미지가
    같은 화각을 담고 있을 때만 성립합니다. HP60C는 컬러 1920x1080(16:9)과
    뎁스 640x480(4:3)이라 **화각 자체가 다릅니다** — 배율로 옮기면 화면
    중앙만 맞고 위아래로 갈수록 어긋납니다(아래 계산 참조).
    센서가 다르면 `project_box()`를 쓰세요.

    쓸 수 있는 경우: `/camera/color/image_raw`(1920x1080) -> `/image_enhanced`
    (640x360)처럼 **같은 스트림을 줄인** 관계.
    """
    x1, y1, x2, y2 = (float(v) for v in box)
    return (x1 * sx, y1 * sy, x2 * sx, y2 * sy)


def project_point(u: float, v: float, k_from, k_to) -> tuple[float, float]:
    """한 카메라의 픽셀을 다른 카메라의 픽셀로. **화각이 달라도 성립합니다.**

        정규화 좌표  x = (u - cx_from) / fx_from
        되투영       u' = fx_to * x + cx_to

    두 이미지가 **같은 광심**을 볼 때 정확합니다 — 즉 뎁스가 컬러에 정합된
    (registered) 경우. 정합이 안 된 원본 뎁스라면 베이스라인만큼 어긋나며,
    오차는 대략 `베이스라인 / 거리`입니다(3.2m·5cm 베이스라인이면 약 5cm).
    그 경우는 외부 파라미터가 따로 필요하고 이 태스크의 범위를 넘습니다
    (지시서 3-2). **로봇 수령 후 정합 여부를 반드시 확인하세요.**
    """
    fx_f, cx_f, fy_f, cy_f = float(k_from[0]), float(k_from[2]), float(k_from[4]), float(k_from[5])
    fx_t, cx_t, fy_t, cy_t = float(k_to[0]), float(k_to[2]), float(k_to[4]), float(k_to[5])
    if fx_f == 0.0 or fy_f == 0.0:
        raise ValueError("k_from의 fx/fy가 0입니다 — camera_info를 못 받은 상태로 보입니다")
    return (fx_t * (float(u) - cx_f) / fx_f + cx_t,
            fy_t * (float(v) - cy_f) / fy_f + cy_t)


def project_box(box, k_from, k_to):
    """★ 컬러 박스를 뎁스 좌표계로 옮기는 **정석**. 두 `camera_info`만 있으면 됩니다.

    해상도 배율(`scale_box`)이 아니라 K를 거치므로, 컬러 16:9와 뎁스 4:3처럼
    **화각이 다른 조합에서도 맞습니다.** 두 K가 순수 배율 관계이면 `scale_box`와
    똑같은 값이 나오므로, 이걸 기본으로 써도 잃는 게 없습니다.

    해상도·화각을 하드코딩하지 마세요 — `process_width`는 미확정이고
    카메라도 바뀔 수 있습니다 (지시서 6장). K는 항상 `camera_info`에서.
    """
    x1, y1, x2, y2 = (float(v) for v in box)
    u1, v1 = project_point(x1, y1, k_from, k_to)
    u2, v2 = project_point(x2, y2, k_from, k_to)
    return (u1, v1, u2, v2)


def k_from_hfov(width: int, height: int, hfov_deg: float,
                vfov_deg: float | None = None) -> list[float]:
    """화각으로 그럴듯한 K를 만듭니다 — **더미/테스트 전용.**

    실기에서는 `camera_info`가 정답입니다. HP60C의 실제 화각은 미확인이라
    (`docs/인수인계_문서_v2.md`에 없음) 로드맵의 HFOV 60도 가정을 씁니다.
    `vfov_deg`를 안 주면 정사각 픽셀(fy = fx)로 둡니다.
    """
    fx = (width / 2.0) / np.tan(np.radians(float(hfov_deg)) / 2.0)
    fy = fx if vfov_deg is None else (height / 2.0) / np.tan(np.radians(float(vfov_deg)) / 2.0)
    return [float(fx), 0.0, width / 2.0, 0.0, float(fy), height / 2.0, 0.0, 0.0, 1.0]


def expand_box(box, margin: float):
    """박스를 각 방향으로 크기의 `margin` 비율만큼 넓힙니다 (ring 영역용)."""
    x1, y1, x2, y2 = (float(v) for v in box)
    dx, dy = (x2 - x1) * margin, (y2 - y1) * margin
    return (x1 - dx, y1 - dy, x2 + dx, y2 + dy)


def _shrink(box, fx: float, fy: float, anchor: str = "center"):
    x1, y1, x2, y2 = (float(v) for v in box)
    cx = (x1 + x2) / 2.0
    hw = (x2 - x1) * fx / 2.0
    hh = (y2 - y1) * fy / 2.0
    if anchor == "bottom":
        return (cx - hw, y2 - 2.0 * hh, cx + hw, y2)
    cy = (y1 + y2) / 2.0
    return (cx - hw, cy - hh, cx + hw, cy + hh)


# ============================================================ 거리 샘플링

@dataclass(frozen=True)
class DistanceSample:
    """거리 한 개 + **왜 그렇게 됐는지**.

    `distance is None`이면 거리 불명입니다. 계약상 이런 검출은 좌표를 지어내지
    않고 빼거나 명시적으로 표시해야 합니다(HANDOVER 7-3). `reason`은 노드가
    진단 로그에 남기라고 있는 값입니다 — 실기에서 "왜 거리가 안 나오는지"를
    현장에서 알 수 없으면 5-1(화염 위 뎁스 무효) 대응을 고를 수 없습니다.
    """
    distance: Optional[float]
    n_valid: int
    n_total: int
    valid_ratio: float
    spread: float
    region: str
    reason: str


def sample_distance(depth, box, method: str = "median", central: float = 0.5,
                    **kwargs) -> Optional[float]:
    """박스 안에서 대표 거리 하나(미터). 못 구하면 `None`.

    지시서 4-1의 계약 그대로입니다. 세부 진단이 필요하면
    `sample_distance_detail`을 쓰세요 (이 함수는 그 얇은 래퍼입니다).
    """
    return sample_distance_detail(depth, box, method=method,
                                  central=central, **kwargs).distance


def sample_distance_detail(
    depth, box,
    method: str = "median",
    central: float = 0.5,
    region: str = "center",
    min_valid_ratio: float = 0.2,
    min_valid_px: int = 1,
    z_min: float = DEFAULT_Z_MIN,
    z_max: float = DEFAULT_Z_MAX,
    max_spread_m: Optional[float] = None,
    ring_margin: float = 0.25,
    band_ratio: float = 0.15,
    encoding: str | None = None,
    depth_scale: float | None = None,
) -> DistanceSample:
    """박스 -> 대표 거리 + 진단. 설계 근거는 각 단계 주석에.

    `box`는 **뎁스 이미지 좌표계**여야 합니다. 컬러가 축소본이면 먼저
    `scale_box`로 옮기세요 (§3-1).
    """
    if method not in METHODS:
        raise ValueError(f"method는 {METHODS} 중 하나 (받은 값: {method!r})")
    if region not in REGIONS:
        raise ValueError(f"region은 {REGIONS} 중 하나 (받은 값: {region!r})")

    depth_m = to_meters(depth, encoding=encoding, depth_scale=depth_scale)
    height, width = depth_m.shape[:2]

    # ★ 자르기를 먼저, 좁히기를 나중에. 순서를 바꾸면 화면 끝에 걸친 박스에서
    #   중앙 영역이 통째로 화면 밖이 되어 픽셀이 0개가 됩니다.
    clipped = clip_box(box, width, height)
    if clipped is None:
        return DistanceSample(None, 0, 0, 0.0, 0.0, region, "box_outside_image")

    vals = _region_values(depth_m, clipped, region, central, ring_margin,
                          band_ratio, width, height)
    if vals is None or vals.size == 0:
        return DistanceSample(None, 0, 0, 0.0, 0.0, region, "box_outside_image")

    n_total = int(vals.size)
    # to_meters가 0/음수/포화를 이미 NaN으로 바꿔뒀습니다. 여기서는 거리 범위만.
    good = vals[np.isfinite(vals) & (vals >= z_min) & (vals <= z_max)]
    n_valid = int(good.size)
    ratio = n_valid / n_total if n_total else 0.0

    if n_valid == 0:
        return DistanceSample(None, 0, n_total, 0.0, 0.0, region, "no_valid_pixels")

    # ★ 유효 픽셀이 너무 적으면 값을 내지 않습니다. "거리 불명"이 잘못된
    #   거리보다 낫습니다 — 0m 하나가 섞이면 메인이 발밑을 화재 지점으로 씁니다.
    if ratio < min_valid_ratio or n_valid < min_valid_px:
        return DistanceSample(None, n_valid, n_total, ratio, 0.0, region,
                              "low_valid_ratio")

    q25, q75 = (float(v) for v in np.percentile(good, [25.0, 75.0]))
    spread = q75 - q25

    # 박스가 앞/뒤 두 면에 걸쳐 있으면 중앙값은 '대표'가 아니라 둘 중 하나를
    # 고른 것뿐입니다. 임계값은 실측 후 정할 값이라 기본은 꺼둡니다(None).
    if max_spread_m is not None and spread > max_spread_m:
        return DistanceSample(None, n_valid, n_total, ratio, spread, region,
                              "too_spread")

    if method == "median":
        # 평균이 아니라 중앙값. 구멍 주변의 잔여 이상치와 배경 비침에 강합니다.
        dist = float(np.median(good))
    elif method == "p25":
        # 앞쪽 25% — 배경이 조금 섞였을 때 앞면 쪽으로 당깁니다.
        dist = q25
    elif method == "p75":
        # 뒤쪽 25%. "below" 띠에서는 이쪽이 접지점에 가깝습니다.
        dist = q75
    elif method == "max":
        # 가장 먼 면. "below" 띠의 맨 윗줄 = 접지점. 다른 영역에서는
        # 배경 비침을 그대로 고르므로 쓰지 마세요.
        dist = float(np.max(good))
    else:  # "min"
        # 가장 가까운 면. 물총 조준처럼 '앞면'이 필요할 때만 쓰세요.
        dist = float(np.min(good))

    return DistanceSample(dist, n_valid, n_total, ratio, spread, region, "ok")


def sample_distance_cascade(depth, box, stages=DEFAULT_CASCADE,
                            **kwargs) -> DistanceSample:
    """폴백 순서대로 시도해 **처음 성공한** 샘플을 돌려줍니다.

    화염 위 뎁스가 비는 건 이 시나리오의 예외가 아니라 기본 상황이라
    (어두운 배경 위 텍스처 없는 고휘도 점 = 스테레오가 가장 취약), `center`
    하나만 쓰면 정작 불이 났을 때 거리가 없습니다.

    ★ **어느 단계가 성공했는지가 결과의 일부입니다.** 돌려주는 `DistanceSample`의
    `region`을 보고 신뢰도를 낮춰 표시하세요 — 폴백은 자기가 틀렸다는 걸 모르고
    전부 `reason="ok"`를 냅니다 (HANDOVER 8).

    전부 실패하면 **첫 단계의** 실패 샘플을 돌려줍니다. 마지막(`ring`)의 사유를
    주면 "박스 주변에 픽셀이 없다"가 되어, 정작 알고 싶은 "대상에 왜 없는가"가
    가려집니다.

    `region`/`method`는 `stages`가 정하므로 `kwargs`로 넘기면 안 됩니다.
    """
    if not stages:
        raise ValueError("stages가 비었습니다")
    for forbidden in ("region", "method"):
        if forbidden in kwargs:
            raise TypeError(f"{forbidden}는 stages가 정합니다 — kwargs로 넘기지 마세요")

    first = None
    for region, method in stages:
        sample = sample_distance_detail(depth, box, region=region,
                                        method=method, **kwargs)
        if sample.distance is not None:
            return sample
        if first is None:
            first = sample
    return first


def parse_cascade(text: str):
    """`"center:median,below:max"` -> `(("center","median"), ("below","max"))`.

    노드 파라미터로 폴백 순서를 바꿀 수 있게 하려고 있습니다 — 순서가 **실측 전
    잠정값**이라서입니다 (HANDOVER 8). 노드가 아니라 여기 있는 이유는 rclpy 없이
    테스트하기 위해서입니다.

    오타는 여기서 바로 터뜨립니다. `sample_distance_detail`까지 들고 가면 첫
    프레임의 콜백 안에서 나고, 그건 **검출이 조용히 사라지는** 형태로 보입니다.
    """
    stages = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        region, _, method = part.partition(":")
        region, method = region.strip(), (method.strip() or "median")
        if region not in REGIONS:
            raise ValueError(f"region은 {REGIONS} 중 하나 (받은 값: {region!r})")
        if method not in METHODS:
            raise ValueError(f"method는 {METHODS} 중 하나 (받은 값: {method!r})")
        stages.append((region, method))
    if not stages:
        raise ValueError(f"cascade가 비었습니다: {text!r}")
    return tuple(stages)


def cascade_to_str(stages) -> str:
    return ",".join(f"{r}:{m}" for r, m in stages)


def _region_values(depth_m, clipped_box, region: str, central: float,
                   ring_margin: float, band_ratio: float, width: int, height: int):
    """샘플링할 픽셀 값들. 영역 선택이 화염 대응의 조절 손잡이입니다 (§5-1)."""
    if region == "ring":
        # 박스 바깥 테두리. 화염이 박스를 가득 채워 안쪽 뎁스가 통째로 비는
        # 경우에 주변 벽/바닥의 거리를 대신 씁니다.
        outer = clip_box(expand_box(clipped_box, ring_margin), width, height)
        if outer is None:
            return None
        ox1, oy1, ox2, oy2 = outer
        ix1, iy1, ix2, iy2 = clipped_box
        mask = np.ones((oy2 - oy1, ox2 - ox1), dtype=bool)
        mask[max(0, iy1 - oy1):max(0, iy2 - oy1),
             max(0, ix1 - ox1):max(0, ix2 - ox1)] = False
        return depth_m[oy1:oy2, ox1:ox2][mask]

    if region == "below":
        # ★ 박스 **바로 아래**의 얇은 띠 = 불이 놓인 바닥과의 접지점.
        # 화염이 박스를 가득 채워 안쪽이 통째로 비어도 여기는 살아 있습니다.
        #
        # 띠를 두껍게 하면 안 됩니다. 바닥은 아래로 갈수록 **가까워지므로**
        # (`ground_plane_depth` 참조) 두꺼운 띠의 중앙값은 거리를 가깝게 잡습니다.
        # band_ratio는 "픽셀 수(=노이즈 강건성) vs 근거리 편향"의 교환입니다.
        x1, y1, x2, y2 = clipped_box
        band = max(2.0, (y2 - y1) * float(band_ratio))
        cx = (x1 + x2) / 2.0
        hw = (x2 - x1) * central / 2.0
        sub = (cx - hw, float(y2), cx + hw, y2 + band)
    elif region == "bottom":
        # 박스 **안**의 아래쪽. 화염이 박스를 다 채우지 않아 아래쪽에 바닥이
        # 보일 때 씁니다. 박스를 꽉 채우는 화염에는 못 씁니다 -> "below".
        sub = _shrink(clipped_box, central, central, anchor="bottom")
    else:
        # ★ 기본. 박스 가장자리는 배경이라 거리가 섞입니다. 중앙만 씁니다.
        sub = _shrink(clipped_box, central, central)

    box = clip_box(sub, width, height)
    if box is None:
        return None
    x1, y1, x2, y2 = box
    return depth_m[y1:y2, x1:x2].reshape(-1)


# =========================================================== 해상도 / 구멍

def resize_depth_nearest(depth, size) -> np.ndarray:
    """뎁스 리샘플. **반드시 INTER_NEAREST** (지시서 3-1, 8장).

    선형보간을 걸면 물체 경계에서 **존재하지 않는 중간 거리**가 만들어지고,
    구멍(0)이 옆 유효 픽셀에 번져 거리를 0쪽으로 오염시킵니다.
    `size`는 cv2 관례대로 (width, height)입니다.
    """
    w, h = int(size[0]), int(size[1])
    return cv2.resize(np.asarray(depth), (w, h), interpolation=cv2.INTER_NEAREST)


def fill_holes(depth_m, max_radius: int = 3, min_neighbors: int = 3) -> np.ndarray:
    """작은 구멍(NaN)만 이웃의 **중앙값**으로 메웁니다. 입력은 미터 배열.

    두 가지를 일부러 제한합니다.

    1. **중앙값**입니다. 평균으로 메우면 물체 경계에서 앞/뒤의 중간값이
       생기는데, 그건 `INTER_LINEAR`와 똑같은 종류의 '없는 거리'입니다.
    2. 한 번에 1픽셀씩 `max_radius`번만 자랍니다. 그래서 실제 측정값에서
       `max_radius`픽셀보다 먼 곳은 절대 안 채워집니다 — 화염처럼 큰 구멍은
       메워지지 않고 남고, 그건 `sample_distance`가 "거리 불명"으로
       처리해야 할 것이지 지어낼 것이 아닙니다.

    `sample_distance`는 이미 중앙값이라 구멍에 강하므로 **기본 경로에서는
    쓰지 않습니다.** 시각화나 뎁스 맵 자체가 필요한 곳에서만 쓰세요.
    """
    filled = np.array(depth_m, dtype=np.float32, copy=True)
    for _ in range(max(0, int(max_radius))):
        holes = np.isnan(filled)
        if not holes.any():
            break
        padded = np.pad(filled, 1, mode="constant", constant_values=np.nan)
        h, w = filled.shape
        stack = np.stack([padded[i:i + h, j:j + w]
                          for i in range(3) for j in range(3)])
        counts = np.count_nonzero(~np.isnan(stack), axis=0)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)  # all-NaN 슬라이스
            med = np.nanmedian(stack, axis=0)
        target = holes & (counts >= min_neighbors)
        if not target.any():
            break
        filled[target] = med[target]
    return filled


# ================================================================== 역투영

def backproject(u: float, v: float, z: float, k) -> tuple[float, float, float]:
    """픽셀 + 거리 -> **optical frame** 3D 좌표 (미터). `plumb_bob` 전제.

        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        Z = z

    ★ `k`는 **그 픽셀 좌표가 속한 이미지의** K여야 합니다. 다른 해상도의 K를
    쓰면 배율만큼 틀리는데 **에러는 안 납니다** (`intrinsics.scale_k` 주석 참조).

    2026-08-11 기준 우리는 `/fire/detections`에 픽셀을 **원본 rgb0 좌표로** 실어
    보내므로, 메인은 `rgb0/camera_info`의 K를 씁니다 (`frame_size`가 그 해상도).
    이 함수는 그 계산을 우리 쪽에서 검산할 때 씁니다 — 발행 경로에는 없습니다.

    왜곡이 `equidistant`(어안)면 이 식은 성립하지 않습니다. 그 경우
    `cv2.undistortPoints`로 정규화 좌표를 먼저 얻어야 합니다 (지시서 3-3).

    반환은 optical frame(x 오른쪽, y 아래, z 앞)입니다. base_link로 돌리는
    것은 `to_base_link`(=TF)의 일이며, 여기서 직접 돌리면 두 번 돌아갑니다.
    """
    if len(k) != 9:
        raise ValueError(f"K는 길이 9여야 합니다 (받은 길이: {len(k)})")
    fx, cx, fy, cy = float(k[0]), float(k[2]), float(k[4]), float(k[5])
    if fx == 0.0 or fy == 0.0:
        raise ValueError("K의 fx/fy가 0입니다 — camera_info를 못 받은 상태로 보입니다")

    if z is None:
        raise TypeError("거리가 None입니다 — 거리 불명 검출은 좌표로 만들지 마세요")
    z = float(z)
    if not np.isfinite(z) or z <= 0.0:
        raise ValueError(f"거리가 유효하지 않습니다: {z} (0은 '측정 실패'입니다)")

    return ((float(u) - cx) * z / fx,
            (float(v) - cy) * z / fy,
            z)


# ======================================================================= TF

def transform_matrix(translation: Sequence[float],
                     quaternion: Sequence[float]) -> np.ndarray:
    """(t, q) -> 4x4 동차 변환. `quaternion`은 ROS 관례대로 **(x, y, z, w)**.

    (w, x, y, z) 순서로 넣으면 회전이 조용히 엉뚱해집니다 — 그래서 정규화
    검사를 겸해 길이도 확인합니다. 전부 0인 쿼터니언(=`w`를 안 채운 메시지)은
    가장 흔한 실수라 예외로 잡습니다.
    """
    q = np.asarray(quaternion, dtype=np.float64)
    if q.shape != (4,):
        raise ValueError("쿼터니언은 (x, y, z, w) 길이 4여야 합니다")
    norm = float(np.linalg.norm(q))
    if abs(norm - 1.0) > 1e-3:
        raise ValueError(
            f"쿼터니언이 정규화돼 있지 않습니다 (norm={norm:.6f}). "
            "w를 채우지 않은 메시지일 가능성이 큽니다."
        )
    x, y, z, w = (q / norm).tolist()

    m = np.eye(4, dtype=np.float64)
    m[0, 0] = 1.0 - 2.0 * (y * y + z * z)
    m[0, 1] = 2.0 * (x * y - z * w)
    m[0, 2] = 2.0 * (x * z + y * w)
    m[1, 0] = 2.0 * (x * y + z * w)
    m[1, 1] = 1.0 - 2.0 * (x * x + z * z)
    m[1, 2] = 2.0 * (y * z - x * w)
    m[2, 0] = 2.0 * (x * z - y * w)
    m[2, 1] = 2.0 * (y * z + x * w)
    m[2, 2] = 1.0 - 2.0 * (x * x + y * y)
    m[:3, 3] = np.asarray(translation, dtype=np.float64)
    return m


def matrix_from_rpy(roll: float, pitch: float, yaw: float,
                    translation: Sequence[float] = (0.0, 0.0, 0.0)) -> np.ndarray:
    """URDF와 같은 규약(고정축 Z-Y-X, 즉 R = Rz·Ry·Rx)으로 4x4를 만듭니다.

    런치 파라미터로 정적 변환을 넣을 때 쓰라고 있는 것입니다.
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)

    m = np.eye(4, dtype=np.float64)
    m[:3, :3] = rz @ ry @ rx
    m[:3, 3] = np.asarray(translation, dtype=np.float64)
    return m


def optical_to_base_link_matrix(
        translation: Sequence[float] = (0.0, 0.0, 0.0)) -> np.ndarray:
    """optical(x 우, y 하, z 전) -> ROS(x 전, y 좌, z 상) 표준 회전 + 평행이동.

    ⚠ **실기용이 아닙니다.** 로봇에서는 카메라 장착 위치·기울기가 URDF에 있고
    tf2가 정답을 줍니다. 이건 로봇·URDF가 없는 지금 더미 박스로 파이프라인을
    끝까지 돌려보기 위한 폴백이며, `translation`도 줄자로 잰 값을 넣는
    임시값입니다. 실기에서 이 함수를 쓰면 카메라 기울기가 통째로 무시됩니다.
    """
    return matrix_from_rpy(-np.pi / 2.0, 0.0, -np.pi / 2.0, translation)


def as_matrix(transform) -> np.ndarray:
    """여러 모양의 변환을 4x4로 통일. **rclpy/tf2를 import하지 않습니다.**

    받는 것:
      - 4x4 배열
      - `geometry_msgs/TransformStamped` 또는 `Transform` (덕 타이핑)
      - `((tx,ty,tz), (qx,qy,qz,qw))` 튜플
    """
    inner = getattr(transform, "transform", None)
    if inner is not None:
        transform = inner

    t = getattr(transform, "translation", None)
    r = getattr(transform, "rotation", None)
    if t is not None and r is not None:
        return transform_matrix((t.x, t.y, t.z), (r.x, r.y, r.z, r.w))

    if isinstance(transform, (tuple, list)) and len(transform) == 2:
        return transform_matrix(transform[0], transform[1])

    m = np.asarray(transform, dtype=np.float64)
    if m.shape != (4, 4):
        raise ValueError(f"변환을 해석할 수 없습니다: shape={m.shape}")
    return m


def to_base_link(xyz_optical, transform) -> tuple[float, float, float]:
    """고정 TF 적용. tf2로 조회한 변환을 받아 **적용만** 합니다.

    `camera_depth_optical_frame -> base_link`는 카메라가 볼트로 고정돼 있어
    URDF 고정 변환입니다. 그래서 이 계산은 SLAM에서 완전히 독립합니다
    (HANDOVER 4-8). 축 변환을 여기서 직접 짜지 마세요 — TF에 이미 들어 있어
    두 번 돌아갑니다.
    """
    m = as_matrix(transform)
    v = np.asarray([*(float(c) for c in xyz_optical), 1.0], dtype=np.float64)
    out = m @ v
    return (float(out[0]), float(out[1]), float(out[2]))


# ================================================= 테스트/더미용 합성 뎁스

def synthetic_depth(width: int, height: int, background_m=4.0,
                    targets: Iterable = (), hole_boxes: Iterable = (),
                    noise_m: float = 0.0, dtype=np.uint16,
                    seed: int | None = None) -> np.ndarray:
    """정답을 아는 가짜 뎁스. 로봇 없이 태스크②를 끝까지 검증하기 위한 것.

    `fake_detection_node`와 `tests/test_depth.py`가 **같은 함수**를 씁니다.
    (둘이 따로 만들면 테스트가 통과해도 노드는 다른 걸 발행합니다.)

      background_m  스칼라(평면 벽) 또는 2D 배열(예: `ground_plane_depth`의 바닥)
      targets       [(box, 거리_m), ...]  박스 영역을 그 거리로 채웁니다
      hole_boxes    [box, ...]            그 영역을 0(측정 실패)으로 — 화염 재현
      dtype         기본 uint16(mm). 드라이버의 `16UC1`과 같은 형식입니다.
    """
    bg = np.asarray(background_m, dtype=np.float64)
    if bg.ndim == 0:
        img = np.full((int(height), int(width)), float(bg), dtype=np.float64)
    elif bg.shape == (int(height), int(width)):
        img = bg.copy()
    else:
        raise ValueError(f"background 배열 크기가 안 맞습니다: {bg.shape}")

    for box, dist in targets:
        clipped = clip_box(box, width, height)
        if clipped is None:
            continue
        x1, y1, x2, y2 = clipped
        img[y1:y2, x1:x2] = float(dist)

    if noise_m:
        rng = np.random.default_rng(seed)
        img += rng.normal(0.0, float(noise_m), img.shape)

    holes = np.zeros_like(img, dtype=bool)
    for box in hole_boxes:
        clipped = clip_box(box, width, height)
        if clipped is None:
            continue
        x1, y1, x2, y2 = clipped
        holes[y1:y2, x1:x2] = True

    if np.issubdtype(np.dtype(dtype), np.integer):
        info = np.iinfo(dtype)
        out = np.clip(np.round(img * 1000.0), 0, info.max - 1).astype(dtype)
    else:
        out = img.astype(dtype)
    out[holes] = 0
    return out


def ground_plane_depth(width: int, height: int, k, camera_height_m: float,
                       max_m: float) -> np.ndarray:
    """카메라가 수평을 볼 때의 **바닥면 + 그 뒤 벽** 뎁스 맵 (미터).

    광축이 수평이면 바닥 위 한 점의 광선각이 곧 거리를 정합니다.

        v = cy + fy * h / Z     ->     Z = fy * h / (v - cy)

    즉 **아래로 갈수록 가까워집니다.** 이 성질 때문에 "불 밑 바닥"을 재는
    폴백(지시서 5-1 1번)은 띠가 두꺼울수록 거리를 **가깝게** 잡습니다.
    그 편향의 크기가 폴백 선택의 판단 근거라 더미가 이 기하를 재현해야 합니다.

    지평선 위(v <= cy)와 바닥이 `max_m`보다 멀어지는 구간은 벽이 가리므로
    `max_m`으로 둡니다.
    """
    fy, cy = float(k[4]), float(k[5])
    if fy <= 0:
        raise ValueError("K의 fy가 0 이하입니다")
    # 픽셀 중심은 인덱스 + 0.5. 반 픽셀 어긋나면 3.2m에서 수 cm가 틀립니다.
    dv = (np.arange(int(height), dtype=np.float64) + 0.5) - cy
    z = np.full(int(height), float(max_m), dtype=np.float64)
    below = dv > 0.0
    z[below] = np.minimum(fy * float(camera_height_m) / dv[below], float(max_m))
    return np.repeat(z.reshape(-1, 1), int(width), axis=1).astype(np.float32)


@dataclass(frozen=True)
class DummyScene:
    """더미 개발용 장면 하나 — **정답을 아는** 컬러 박스 + 뎁스 + K 묶음.

    `fake_detection_node`가 이걸 만들어 발행하고, `tests/test_depth.py`가 같은
    것을 만들어 검산합니다. 노드 안에서 장면을 조립하면 rclpy 없이 검증할 수
    없어서 "더미가 틀렸는지 태스크②가 틀렸는지" 구분이 안 됩니다.

    ★ 기본값은 실제 드라이버 설정입니다 (`ascamera.launch.py`).

        rgb0   640x480 (4:3)  --전처리(현재 무축소)--> 640x480   <- 박스 좌표계
        depth0 640x480 (4:3)                                     <- 거리는 여기서

    지금은 둘의 가로세로비가 같아서 `project_box`가 사실상 항등입니다.
    그래도 **`scale_box`가 아니라 `project_box`를 써야 합니다** — 성냥불 실측
    결과 RGB를 1080p(16:9)로 올리면 뎁스 4:3과 어긋나고, 그때 배율 방식은
    화면 중앙만 맞고 위아래로 갈수록 틀립니다(3.2m에서 세로 17cm).
    그 상황은 `color_size=(640, 360)`으로 재현할 수 있고 테스트가 잠급니다.
    """
    color_size: tuple
    depth_size: tuple
    k_color: list
    k_depth: list
    box_color: tuple
    box_depth: tuple
    distance_m: float
    background_m: float
    flame_hole: bool
    noise_m: float
    floor_height_m: Optional[float] = None

    def depth_image(self, seed: int | None = None) -> np.ndarray:
        """16UC1(mm) 합성 뎁스. `flame_hole`이면 박스 영역이 통째로 0입니다.

        `floor_height_m`이 있으면 배경이 평면 벽이 아니라 **바닥 + 그 뒤 벽**이
        됩니다. 그래야 5-1의 폴백 후보(`bottom`/`below`/`ring`)를 비교할 수
        있습니다 — 평면 벽만 있는 장면에서는 셋 다 같은 벽을 잽니다.
        """
        background = self.background_m
        if self.floor_height_m is not None:
            background = ground_plane_depth(
                self.depth_size[0], self.depth_size[1], self.k_depth,
                self.floor_height_m, self.background_m)
        return synthetic_depth(
            self.depth_size[0], self.depth_size[1],
            background_m=background,
            targets=[(self.box_depth, self.distance_m)],
            hole_boxes=[self.box_depth] if self.flame_hole else [],
            noise_m=self.noise_m, seed=seed,
        )

    def color_image(self, haze: float = 0.0) -> np.ndarray:
        """bgr8 합성 컬러. **뎁스와 같은 장면**의 박스 자리에 불꽃을 그립니다.

        `fake_detection_node`가 태스크①의 입력으로 발행합니다. 전처리 →
        YOLO → 태스크②를 **한 사슬로** 돌려보려면 컬러도 있어야 하고, 그
        컬러의 불꽃 위치가 뎁스의 대상 위치와 **같아야** 결과를 검산할 수
        있습니다. 그래서 노드가 아니라 여기서 만듭니다.

        `haze`(0~1)는 태스크①이 걷어낼 연기입니다. 0이면 걸지 않습니다 —
        디헤이즈가 실제로 뭔가를 하는지 보려면 0.3 근처를 주세요.

        ⚠ 이건 **검출 성능 평가용이 아닙니다.** 합성 원반은 실제 성냥불의
        분광·크기 분포와 다릅니다. 배선과 좌표를 보는 그림입니다.
        """
        w, h = self.color_size
        img = np.full((h, w, 3), 28, np.uint8)          # 어두운 실내
        # 바닥 쪽을 조금 밝게 — 완전 균일하면 CLAHE가 할 일이 없어 보입니다.
        grad = np.linspace(0, 26, h, dtype=np.float32)[:, None]
        img = np.clip(img.astype(np.float32) + grad[..., None], 0, 255).astype(np.uint8)

        x1, y1, x2, y2 = self.box_color
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        radius = max(2.0, min(x2 - x1, y2 - y1) / 2.0)

        yy, xx = np.mgrid[0:h, 0:w]
        r = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2) / radius
        core = np.clip(1.0 - r, 0.0, 1.0) ** 1.5        # 가운데가 흰 불꽃
        halo = np.exp(-(r ** 2) * 1.2) * 0.55           # 주변 발광

        out = img.astype(np.float32)
        for ch, (core_v, halo_v) in enumerate(((215, 90), (245, 170), (255, 255))):
            out[..., ch] += core * core_v + halo * halo_v   # BGR
        out = np.clip(out, 0, 255)

        if haze > 0.0:
            # I = J·t + A·(1−t) — 태스크①의 디헤이즈가 되돌리려는 바로 그 모델
            t = float(np.clip(1.0 - haze, 0.05, 1.0))
            out = out * t + 235.0 * (1.0 - t)

        return np.clip(out, 0, 255).astype(np.uint8)

    def expected_optical(self) -> Optional[tuple]:
        """태스크②가 내야 할 optical frame 좌표. 거리 불명이면 None."""
        if self.flame_hole:
            return None
        u, v = box_center(self.box_color)
        return backproject(u, v, self.distance_m, self.k_color)

    def expected_base_link(self, camera_offset=(0.0, 0.0, 0.0)) -> Optional[tuple]:
        """⚠ `optical_to_base_link_matrix`와 같은 한계 — 실기에서는 tf2가 정답."""
        optical = self.expected_optical()
        if optical is None:
            return None
        return to_base_link(optical, optical_to_base_link_matrix(camera_offset))


def dummy_scene(color_size=(640, 480), depth_size=(640, 480),
                box_size=(80.0, 80.0), box_center_xy=None,
                distance_m: float = 3.2, background_m: float = 3.8,
                color_hfov_deg: float = 60.0, depth_hfov_deg: float = 60.0,
                flame_hole: bool = False, noise_m: float = 0.0,
                floor_height_m: float | None = None,
                box_on_floor: bool = False) -> DummyScene:
    """`DummyScene` 생성. 크기는 (width, height), 박스는 **컬러 좌표계**입니다.

    기본값은 HP60C 형상입니다 — 컬러는 전처리 축소본 640x360(16:9),
    뎁스는 640x480(4:3). 거리 기본값 3.2m는 시나리오 값이고, 배경 3.8m는
    **카메라 스펙 상한 4m 안쪽**으로 잡았습니다(그 밖은 측정이 안 됩니다).
    화각은 미확인이라 로드맵의 60도 가정 — 수령 후 `camera_info`로 교체하세요.

    `floor_height_m`(카메라의 바닥 위 높이)을 주면 배경이 **바닥 + 벽**이 됩니다.
    `box_on_floor=True`면 박스 아래변이 `distance_m` 지점의 **접지점**에 놓이도록
    세로 위치를 자동 계산합니다 — 불은 공중에 뜨지 않으므로, 5-1 폴백을
    시험하려면 이 배치여야 의미가 있습니다.
    """
    cw, ch = int(color_size[0]), int(color_size[1])
    dw, dh = int(depth_size[0]), int(depth_size[1])

    k_color = k_from_hfov(cw, ch, color_hfov_deg)
    k_depth = k_from_hfov(dw, dh, depth_hfov_deg)

    if box_center_xy is not None:
        cx, cy = float(box_center_xy[0]), float(box_center_xy[1])
    else:
        cx, cy = cw / 2.0, ch / 2.0

    if box_on_floor:
        if floor_height_m is None:
            raise ValueError("box_on_floor 를 쓰려면 floor_height_m 이 필요합니다")
        # 뎁스 영상에서 distance_m 지점의 바닥이 찍히는 행 (ground_plane_depth 역산)
        v_contact_depth = k_depth[5] + k_depth[4] * float(floor_height_m) / float(distance_m)
        # 컬러 좌표계로 되돌린 뒤, 박스 아래변이 거기 닿도록 중심을 잡습니다.
        _, v_contact_color = project_point(cx, v_contact_depth, k_depth, k_color)
        cy = v_contact_color - float(box_size[1]) / 2.0

    box_color = box_from_center(cx, cy, float(box_size[0]), float(box_size[1]))

    return DummyScene(
        color_size=(cw, ch), depth_size=(dw, dh),
        k_color=k_color, k_depth=k_depth,
        box_color=box_color,
        # ★ 배율이 아니라 K를 거칩니다. 태스크②는 정확히 이걸 반대로 해야 합니다.
        box_depth=project_box(box_color, k_color, k_depth),
        distance_m=float(distance_m), background_m=float(background_m),
        flame_hole=bool(flame_hole), noise_m=float(noise_m),
        floor_height_m=None if floor_height_m is None else float(floor_height_m),
    )
