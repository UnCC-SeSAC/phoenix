#!/usr/bin/env python3
"""
태스크② 프레임 단위 변환 — 2D 박스 + 뎁스 → base_link 3D. **ROS 비의존.**

노드(`detection_3d_node.py`)는 구독·동기화·TF조회·발행만 하고, **"무엇을
발행하고 무엇을 버릴지"의 판단은 전부 여기** 있습니다. 그 판단이 이 태스크의
핵심이자 가장 조용히 틀리는 부분이라, rclpy 없이 pytest로 검증돼야 합니다.

한 프레임의 흐름:

    박스(컬러 좌표계)
      └─ project_box(k_color, k_depth) ─ 뎁스 좌표계로            (§3-1)
           └─ sample_distance_detail ─ 대표 거리 + 실패 사유      (§3-4, §4-1)
                └─ backproject(k_color) ─ optical frame 3D
                     └─ to_base_link(고정 TF) ─ base_link          (4-8)

계약 (HANDOVER 7-3):
  - 거리를 못 구한 검출은 **좌표를 지어내지 않고 뺍니다.** 0m가 섞이면 메인이
    로봇 발밑을 화재 지점으로 계산합니다.
  - 뺀 것은 `FrameResult.dropped`에 사유와 함께 남습니다. 실기에서 "왜 좌표가
    안 나오는지"를 모르면 5-1(화염 위 뎁스 무효) 대응을 고를 수 없습니다.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from image_pipeline.depth import (
    DEFAULT_Z_MAX,
    DEFAULT_Z_MIN,
    backproject,
    box_center,
    project_box,
    project_point,
    sample_distance_detail,
    to_base_link,
)
from image_pipeline.detection_json import detection_entry, is_surrogate


# 폴백 영역별 **권장 통계**. HANDOVER 8 "5-1 폴백 후보 정량 비교" 실측 기반.
#
#   below   접지점 띠. 아래로 갈수록 가까워지므로 **가장 먼 값이 접지점**입니다
#           (max −0.046m / median −0.326m). max만 띠 두께와 거의 무관합니다.
#   bottom  박스 **안**이라 대상 자체를 읽습니다. max를 쓰면 배경 비침을 고릅니다.
#   ring    주변 테두리. 이미 뒤로 편향(+0.600m)인데 max를 쓰면 **가장 먼 배경**을
#           골라 편향이 더 커집니다.
#
# ★ 즉 `fallback_method` 하나를 모든 영역에 똑같이 적용하면 안 됩니다.
#   `SamplingParams.fallback_method=None`(기본)이면 이 표를 씁니다.
REGION_METHOD = {"bottom": "median", "below": "max", "ring": "median"}


@dataclass(frozen=True)
class Detected3D:
    """발행할 검출 하나. 좌표는 **base_link, 미터**."""
    xyz: tuple
    class_id: str
    score: float
    distance_m: float
    region: str          # 거리를 얻은 영역 ("center"/"bottom"이면 대상 자체)
    valid_ratio: float
    is_fallback: bool    # ★ True면 대상이 아니라 **주변**을 잰 값입니다


@dataclass(frozen=True)
class Dropped:
    """발행하지 않은 검출 + 이유. 진단용."""
    class_id: str
    score: float
    reason: str


@dataclass(frozen=True)
class FrameResult:
    detections: list
    dropped: list

    def reason_counts(self) -> dict:
        """사유별 개수. 노드가 throttle 로그에 씁니다."""
        out: dict = {}
        for d in self.dropped:
            out[d.reason] = out.get(d.reason, 0) + 1
        return out


@dataclass
class SamplingParams:
    """거리 샘플링 설정. 전부 노드 파라미터로 노출됩니다.

    ★ `region`의 기본은 "center"가 아니라 **"bottom"**입니다 (2026-08-24).
    성냥불 위에서 뎁스가 안 나오는 것을 실기에서 확인했습니다 — 화염은 스스로
    적외선을 내는 광원이라 박스 중앙이 구조적으로 무효입니다. `bottom`은 박스
    **안**의 아래쪽이라 폴백이 아니라 여전히 대상 자체를 읽습니다.

    ★ `fallback_regions`는 **기본으로 비어 있습니다.**
    `below`/`ring`은 대상이 아니라 주변을 재고, 서로 **반대 방향으로** 편향되며
    (below 가깝게 / ring 멀게), 둘 다 `reason="ok"`에 유효비율 100%로 나옵니다.
    실측 없이 켜면 "확신에 찬 틀린 좌표"가 발행됩니다.
    근거 수치는 `HANDOVER.md` 8장 "5-1 폴백 후보 정량 비교".

    ⚠ 화염이 박스를 **가득** 채우면 `bottom`도 막힙니다(`no_valid_pixels`).
      그 장면에서 값이 나오는 건 `below`(max)뿐이므로, 화염이 박스를 얼마나
      채우는지를 실기에서 보고 `fallback_regions=("below", "ring")`을 켤지
      정하세요.
    """
    region: str = "bottom"
    method: str = "median"
    central: float = 0.5
    band_ratio: float = 0.15
    ring_margin: float = 0.25
    min_valid_ratio: float = 0.2
    min_valid_px: int = 1
    z_min: float = DEFAULT_Z_MIN
    z_max: float = DEFAULT_Z_MAX
    max_spread_m: Optional[float] = None
    fallback_regions: tuple = ()
    # None = 영역별 권장값(REGION_METHOD). 문자열을 주면 **모든** 폴백 영역에
    # 그 통계를 강제합니다 — ring 에 "max"를 강제하면 편향이 더 커집니다.
    fallback_method: Optional[str] = None
    min_score: float = 0.0

    def as_kwargs(self, region: str, method: str | None = None) -> dict:
        return dict(
            region=region, method=method or self.method, central=self.central,
            band_ratio=self.band_ratio, ring_margin=self.ring_margin,
            min_valid_ratio=self.min_valid_ratio, min_valid_px=self.min_valid_px,
            z_min=self.z_min, z_max=self.z_max, max_spread_m=self.max_spread_m,
        )


def convert_frame(boxes, depth, k_color, k_depth, transform,
                  params: SamplingParams | None = None,
                  encoding: str | None = None,
                  depth_scale: float | None = None) -> FrameResult:
    """한 프레임의 박스들을 base_link 좌표로. 못 구한 것은 버리고 사유를 남깁니다.

      boxes      [(box, class_id, score), ...]  box는 **컬러 좌표계** (x1,y1,x2,y2)
      k_color    박스가 있는 이미지의 K (= `/image_enhanced/camera_info`)
      k_depth    뎁스 이미지의 K
      transform  뎁스 optical frame -> base_link 고정 변환 (tf2 조회 결과)

    ★ `k_color`로 역투영하는 이유: 박스가 그 좌표계에 있기 때문입니다.
      뎁스 K로 역투영하면 광선 방향이 틀립니다.
    ★ 뎁스 샘플링은 `k_depth` 좌표계에서 합니다. 그래서 박스를 먼저 옮깁니다.
    """
    p = params or SamplingParams()
    detections: list = []
    dropped: list = []

    for box, class_id, score in boxes:
        if score < p.min_score:
            dropped.append(Dropped(class_id, score, "low_score"))
            continue

        box_d = project_box(box, k_color, k_depth)
        sample, region_used, is_fallback = _sample_with_fallback(
            depth, box_d, p, encoding, depth_scale)

        if sample.distance is None:
            # ★ 좌표를 지어내지 않습니다. 0m 하나가 발밑 좌표를 만듭니다.
            dropped.append(Dropped(class_id, score, sample.reason))
            continue

        u, v = box_center(box)
        xyz = to_base_link(backproject(u, v, sample.distance, k_color), transform)
        detections.append(Detected3D(
            xyz=xyz, class_id=class_id, score=score,
            distance_m=sample.distance, region=region_used,
            valid_ratio=sample.valid_ratio, is_fallback=is_fallback,
        ))

    return FrameResult(detections=detections, dropped=dropped)


@dataclass(frozen=True)
class PixelFrameResult:
    """JSON 계약(2026-08-10)용 프레임 결과.

    `convert_frame`(base_link 3D)과의 차이가 계약 변경 그 자체입니다:

    - 거리 불명을 **버리지 않고** `depth: null` + `depth_status: "unknown"`으로
      실어 보냅니다. "불은 보이는데 거리를 못 쟀다"는 메인에게 유용한 정보이고,
      빼버리면 메인은 그 검출의 존재조차 모릅니다.
    - 그래서 `dropped`에는 `low_score`만 남습니다.
    """
    entries: list        # detection_json 항목 dict
    dropped: list        # Dropped (진단용)

    def reason_counts(self) -> dict:
        out: dict = {}
        for d in self.dropped:
            out[d.reason] = out.get(d.reason, 0) + 1
        return out

    def unknown_count(self) -> int:
        return sum(1 for e in self.entries if e["depth"] is None)

    def fallback_count(self) -> int:
        """대상이 아니라 **주변**을 재서 얻은 거리의 개수 (`below`/`ring`).

        ★ 접두사 `fallback`으로 세지 않습니다. `fallback_bottom`은 이름과 달리
        박스 **안**이라 대상 자체를 읽고, 2026-08-24부터 기본 region이므로
        접두사로 세면 매 프레임 전건이 잡힙니다.
        """
        return sum(1 for e in self.entries if is_surrogate(e["depth_status"]))


def convert_frame_pixels(boxes, depth, k_color, k_depth, k_out=None,
                         params: SamplingParams | None = None,
                         encoding: str | None = None,
                         depth_scale: float | None = None) -> PixelFrameResult:
    """한 프레임의 박스들을 **픽셀 + 거리**로. 역투영·TF는 메인이 합니다.

      boxes    [(box, class_id, score), ...]  box는 **컬러 좌표계**
      k_color  박스가 있는 이미지의 K (= `/image_enhanced/camera_info`)
      k_depth  뎁스 이미지의 K — 거리 샘플링용
      k_out    **발행할 픽셀 좌표계**의 K (= 드라이버 `rgb0/camera_info`).
               None이면 `k_color` 좌표를 그대로 냅니다.

    ★ `k_out`이 핵심입니다. 전처리가 축소해 발행하므로 박스는 축소본 좌표인데,
      메인은 드라이버의 `rgb0/camera_info`로 역투영합니다. 되돌리지 않고 보내면
      **배율만큼 틀리고 에러는 안 납니다** (축소율 0.5, 3.2m에서 +1.10 → +4.15m).

    ★ 좌표 변환이 둘인데 **목적지가 다릅니다.** `project_box`는 거리를 재려고
      뎁스로, `project_point`는 발행하려고 원본으로 갑니다. 헷갈리기 쉬운 지점.
    """
    p = params or SamplingParams()
    entries: list = []
    dropped: list = []

    for box, class_id, score in boxes:
        if score < p.min_score:
            dropped.append(Dropped(class_id, score, "low_score"))
            continue

        box_d = project_box(box, k_color, k_depth)
        sample, _region, _is_fallback = _sample_with_fallback(
            depth, box_d, p, encoding, depth_scale)

        u, v = box_center(box)
        if k_out is not None:
            u, v = project_point(u, v, k_color, k_out)

        # depth_status가 폴백 여부를 담습니다 — `_is_fallback`을 따로 안 쓰는 이유는
        # `sample.region`이 이미 어느 단계였는지를 갖고 있어서입니다.
        entries.append(detection_entry(class_id, score, u, v, sample))

    return PixelFrameResult(entries=entries, dropped=dropped)


def _sample_with_fallback(depth, box_d, p: SamplingParams, encoding, depth_scale):
    """기본 영역 → (설정된 경우) 폴백 영역 순으로 시도.

    폴백으로 얻은 값은 `is_fallback=True`로 표시합니다. 노드가 이 표시를 보고
    신뢰도를 낮추거나 별도로 다뤄야 합니다 — **표시 없이 섞어 보내면 메인과
    물총이 대상 거리로 착각합니다.**
    """
    first = sample_distance_detail(
        depth, box_d, encoding=encoding, depth_scale=depth_scale,
        **p.as_kwargs(p.region))
    if first.distance is not None:
        return first, p.region, False

    for region in p.fallback_regions:
        # 폴백 영역은 대상이 아니라 주변이라 통계도 달라집니다. "below" 띠는
        # 아래로 갈수록 가까워지므로 중앙값이 아니라 위쪽 꼬리를 써야 접지점입니다.
        method = p.fallback_method or REGION_METHOD.get(region, "median")
        alt = sample_distance_detail(
            depth, box_d, encoding=encoding, depth_scale=depth_scale,
            **p.as_kwargs(region, method))
        if alt.distance is not None:
            return alt, region, True

    return first, p.region, False


class StampMonitor:
    """검출 stamp가 **카메라가 실제로 낸 stamp인지** 확인 — ROS 비의존.

    ★ 왜 "검출 stamp - 뎁스 stamp"를 비교하면 안 되는가 (실측으로 확인):
      `ApproximateTimeSynchronizer`는 검출과 **가장 가까운** 뎁스를 짝지어 줍니다.
      그래서 검출 stamp가 80ms 밀려 있으면 동기화기가 80ms 뒤의 뎁스 프레임을
      골라버리고, 짝지어진 둘의 차이는 거의 0이 됩니다. **드리프트가 상쇄되어
      원리적으로 안 보입니다.** (그리고 그 사이 로봇은 80ms만큼 움직였습니다)

    대신 여기서는 **정확한 일치**를 봅니다. 드라이버가 rgb와 depth를 같은
    콜백에서 같은 stamp로 발행하므로(`CameraPublisher.cpp:316-321`), 파이프라인이
    헤더를 제대로 승계했다면 검출 stamp는 **카메라가 낸 stamp 중 하나와
    nanosec까지 같아야** 합니다. `now()`로 새로 만든 값이 우연히 일치할 확률은
    사실상 0입니다.

    노드는 카메라 stamp를 `camera_info`(작은 메시지)에서 모으고, 검출은
    동기화기를 거치지 않은 **원본 구독**에서 확인합니다. 동기화기를 통과한
    것만 보면 이미 걸러진 뒤라 못 잡습니다.
    """

    def __init__(self, history: int = 90, window: int = 30,
                 min_match_ratio: float = 0.5):
        self.history = int(history)          # 15Hz 기준 90개 = 약 6초
        self.window = int(window)
        self.min_match_ratio = float(min_match_ratio)
        self._known: list = []               # 카메라가 낸 stamp들
        self._results: list = []             # 최근 검출들의 일치 여부
        self.warned = False

    def note_camera_stamp(self, key) -> None:
        """카메라(camera_info/이미지)에서 본 stamp를 기록. key = (sec, nanosec)."""
        k = (int(key[0]), int(key[1]))
        if k not in self._known:
            self._known.append(k)
            if len(self._known) > self.history:
                self._known.pop(0)

    def check_detection(self, key) -> bool:
        """검출 stamp 하나 확인. **새로** 경고할 상황이면 True."""
        if len(self._known) < 5:
            return False                     # 아직 기준이 없음 (시작 직후)
        self._results.append((int(key[0]), int(key[1])) in self._known)
        if len(self._results) > self.window:
            self._results.pop(0)
        if self.warned or len(self._results) < self.window:
            return False
        if self.match_ratio() < self.min_match_ratio:
            self.warned = True
            return True
        return False

    def match_ratio(self) -> float:
        if not self._results:
            return 1.0
        return sum(self._results) / len(self._results)

    def message(self) -> str:
        return (
            f"검출 stamp가 카메라 stamp와 일치하지 않습니다 "
            f"(최근 {len(self._results)}개 중 일치 {self.match_ratio():.0%}). "
            "파이프라인 어딘가가 stamp를 now()로 덮어쓰고 있습니다 "
            "(out.header = msg.header 누락). 동기화기가 '가장 가까운' 뎁스를 "
            "대신 골라주기 때문에 좌표는 그럴듯하게 나오지만, 메인의 map 변환이 "
            "그 지연만큼 틀어집니다 — 회전 중 3.2m에서 16cm."
        )
