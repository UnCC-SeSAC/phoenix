#!/usr/bin/env python3
"""
태스크② 뎁스 → 3D 좌표 로컬 테스트 — rclpy 없이 돕니다.

여기서 잡으려는 것은 "돌아가는가"가 아니라 **"에러 없이 틀리지 않는가"** 입니다.
이 태스크의 사고는 전부 조용합니다:

  - depth 0(측정 실패)을 0m로 집계     -> 로봇 발밑이 화재 지점이 됨
  - 축소본 좌표에 원본 K를 사용         -> 거리가 배율만큼 틀림
  - 뎁스에 선형보간                     -> 물체 경계에 없는 거리가 생김
  - 유효 픽셀 1개로 거리 확정           -> 노이즈가 좌표가 됨

전부 예외가 안 납니다. 그래서 테스트로 잠급니다.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from image_pipeline.depth import (  # noqa: E402
    DEFAULT_CASCADE,
    DistanceSample,
    cascade_to_str,
    parse_cascade,
    sample_distance_cascade,
    DummyScene,
    dummy_scene,
    ground_plane_depth,
    DEFAULT_Z_MAX,
    DEFAULT_Z_MIN,
    backproject,
    box_center,
    box_from_center,
    clip_box,
    depth_unit_sanity,
    fill_holes,
    matrix_from_rpy,
    optical_to_base_link_matrix,
    resize_depth_nearest,
    sample_distance,
    sample_distance_detail,
    k_from_hfov,
    project_box,
    project_point,
    scale_box,
    synthetic_depth,
    to_base_link,
    to_meters,
    transform_matrix,
)
from image_pipeline.detection_msgs import (  # noqa: E402
    best_class,
    best_score,
    bbox_center,
    box_from_bbox,
    hypothesis,
    set_bbox_center,
    set_hypothesis,
)
from image_pipeline.intrinsics import fit_size, principal_point_sanity, scale_k  # noqa: E402


# K는 640x480, fx=fy=600, 주점은 정중앙. 손계산이 편한 값으로 골랐습니다.
K_640 = [600.0, 0.0, 320.0,
         0.0, 600.0, 240.0,
         0.0, 0.0, 1.0]


# ---------------------------------------------------------------- 역투영 (§4-2)

class TestBackproject:
    """지시서 10장 4·5번 — 손으로 계산한 값과 맞는지가 이 태스크의 출발점."""

    def test_hand_computed(self):
        # (u-cx)*Z/fx = (420-320)*2.5/600 = 0.4166666...
        x, y, z = backproject(420.0, 340.0, 2.5, K_640)
        assert x == pytest.approx(100 * 2.5 / 600)
        assert y == pytest.approx(100 * 2.5 / 600)
        assert z == pytest.approx(2.5)

    def test_principal_point_maps_to_optical_axis(self):
        """주점을 역투영하면 X=Y=0. 부호 실수를 잡는 가장 싼 검사."""
        x, y, z = backproject(320.0, 240.0, 3.0, K_640)
        assert x == pytest.approx(0.0)
        assert y == pytest.approx(0.0)
        assert z == pytest.approx(3.0)

    def test_optical_frame_axis_directions(self):
        """REP-103 optical frame: x 오른쪽, y 아래, z 앞.

        주점보다 오른쪽/아래 픽셀은 X/Y가 양수여야 합니다. base_link(x 앞,
        y 왼쪽)와 헷갈려 여기서 축을 돌리면 TF가 두 번 돌립니다.
        """
        x, y, _ = backproject(400.0, 300.0, 2.0, K_640)
        assert x > 0  # 오른쪽 -> +x
        assert y > 0  # 아래   -> +y

    def test_distance_scales_linearly(self):
        x1, y1, _ = backproject(420.0, 340.0, 1.0, K_640)
        x2, y2, _ = backproject(420.0, 340.0, 2.0, K_640)
        assert x2 == pytest.approx(2 * x1)
        assert y2 == pytest.approx(2 * y1)

    def test_rejects_bad_k(self):
        with pytest.raises(ValueError):
            backproject(0.0, 0.0, 1.0, [1.0, 2.0, 3.0])
        with pytest.raises(ValueError):
            backproject(0.0, 0.0, 1.0, [0.0, 0, 320, 0, 600, 240, 0, 0, 1])  # fx=0

    def test_rejects_invalid_z(self):
        """거리 불명(None/NaN/0)을 좌표로 만들면 안 됩니다 — 계약 위반."""
        for bad in (None, float("nan"), 0.0, -1.0):
            with pytest.raises((ValueError, TypeError)):
                backproject(320.0, 240.0, bad, K_640)


class TestScaledIntrinsics:
    """지시서 4-2 마지막 항목 — 축소 K와 원본 K가 **다른** 결과를 내야 함."""

    SRC_W, SRC_H = 1280, 720
    K_SRC = [900.0, 0.0, 640.0,
             0.0, 900.0, 360.0,
             0.0, 0.0, 1.0]

    def test_scaled_and_original_k_disagree_on_scaled_pixels(self):
        _, _, sx, sy = fit_size(self.SRC_W, self.SRC_H, 640)
        k_small = scale_k(self.K_SRC, sx, sy)

        u_small, v_small, z = 420.0, 300.0, 3.0
        right = backproject(u_small, v_small, z, k_small)
        wrong = backproject(u_small, v_small, z, self.K_SRC)

        # 같게 나오면 스케일 보정이 안 걸린 것입니다.
        assert abs(right[0] - wrong[0]) > 0.5
        assert abs(right[1] - wrong[1]) > 0.5

    def test_scaled_k_agrees_with_original_k_on_original_pixels(self):
        """같은 실세계 점이면 어느 해상도에서 봐도 같은 3D 좌표가 나와야 합니다."""
        _, _, sx, sy = fit_size(self.SRC_W, self.SRC_H, 640)
        k_small = scale_k(self.K_SRC, sx, sy)

        u_small, v_small, z = 420.0, 300.0, 3.0
        small = backproject(u_small, v_small, z, k_small)
        full = backproject(u_small / sx, v_small / sy, z, self.K_SRC)

        assert small == pytest.approx(full, abs=1e-9)

    def test_principal_point_sanity_catches_wrong_camera_info(self):
        """원본 K를 축소 해상도에 쓰면 주점이 화면 밖 -> 노드 시작 시 걸러야 함."""
        assert principal_point_sanity(self.K_SRC, self.SRC_W, self.SRC_H)
        assert not principal_point_sanity(self.K_SRC, 640, 360)


# ------------------------------------------------------------ 단위 변환 (§3-4)

class TestToMeters:
    """단위 변환은 depth.py 한 곳에서만 (CLAUDE.md 구조 규칙)."""

    def test_uint16_is_millimeters(self):
        d = np.array([[1000, 2500]], dtype=np.uint16)
        m = to_meters(d)
        assert m[0, 0] == pytest.approx(1.0)
        assert m[0, 1] == pytest.approx(2.5)

    def test_float32_is_already_meters(self):
        d = np.array([[1.0, 2.5]], dtype=np.float32)
        m = to_meters(d)
        assert m[0, 0] == pytest.approx(1.0)
        assert m[0, 1] == pytest.approx(2.5)

    def test_encoding_overrides_dtype_guess(self):
        """float32에 mm가 담겨 오는 드라이버가 실재합니다. encoding이 우선."""
        d = np.array([[1000.0]], dtype=np.float32)
        assert to_meters(d, encoding="16UC1")[0, 0] == pytest.approx(1.0)

    def test_explicit_depth_scale_wins(self):
        d = np.array([[1000]], dtype=np.uint16)
        assert to_meters(d, depth_scale=0.0001)[0, 0] == pytest.approx(0.1)

    def test_zero_becomes_nan_not_zero_meters(self):
        """★ depth 0 = 측정 실패. 0m가 아닙니다 (지시서 3-4)."""
        m = to_meters(np.array([[0, 1500]], dtype=np.uint16))
        assert np.isnan(m[0, 0])
        assert m[0, 1] == pytest.approx(1.5)

    def test_out_of_range_becomes_nan(self):
        """65535mm(=65m) 같은 포화값은 실내에서 값이 아니라 쓰레기입니다."""
        m = to_meters(np.array([[65535, 3000]], dtype=np.uint16))
        assert np.isnan(m[0, 0])
        assert m[0, 1] == pytest.approx(3.0)

    def test_inf_and_nan_input_survive_as_nan(self):
        d = np.array([[np.nan, np.inf, 2.0]], dtype=np.float32)
        m = to_meters(d)
        assert np.isnan(m[0, 0]) and np.isnan(m[0, 1])
        assert m[0, 2] == pytest.approx(2.0)

    def test_does_not_mutate_input(self):
        d = np.array([[0, 1500]], dtype=np.uint16)
        to_meters(d)
        assert d[0, 0] == 0 and d[0, 1] == 1500

    def test_depth_unit_sanity(self):
        """mm를 m로 착각하면 4000m가 나옵니다. 노드 시작 시 한 번 검사."""
        assert depth_unit_sanity(np.full((8, 8), 3.0, dtype=np.float32))
        assert not depth_unit_sanity(np.full((8, 8), 3000.0, dtype=np.float32))
        assert not depth_unit_sanity(np.full((8, 8), np.nan, dtype=np.float32))


# ------------------------------------------------------------- 박스 유틸 (§4-2)

class TestBoxGeometry:
    def test_clip_box_inside(self):
        assert clip_box((10.0, 20.0, 30.0, 40.0), 640, 480) == (10, 20, 30, 40)

    def test_clip_box_partially_outside(self):
        """★ 지시서 4-2 4번 — 박스가 이미지 밖으로 나가도 죽지 않을 것."""
        assert clip_box((-50.0, -10.0, 100.0, 60.0), 640, 480) == (0, 0, 100, 60)
        assert clip_box((600.0, 460.0, 900.0, 700.0), 640, 480) == (600, 460, 640, 480)

    def test_clip_box_fully_outside_is_none(self):
        assert clip_box((700.0, 500.0, 800.0, 600.0), 640, 480) is None
        assert clip_box((-100.0, -100.0, -10.0, -10.0), 640, 480) is None

    def test_clip_box_zero_area_is_none(self):
        assert clip_box((10.0, 10.0, 10.0, 20.0), 640, 480) is None

    def test_box_from_center_roundtrip(self):
        """vision_msgs Detection2D는 center+size로 옵니다."""
        box = box_from_center(320.0, 240.0, 40.0, 20.0)
        assert box == pytest.approx((300.0, 230.0, 340.0, 250.0))
        assert box_center(box) == pytest.approx((320.0, 240.0))

    def test_scale_box(self):
        """컬러 640x360 박스를 뎁스 848x480 좌표로."""
        sx, sy = 848 / 640, 480 / 360
        out = scale_box((100.0, 50.0, 200.0, 150.0), sx, sy)
        assert out == pytest.approx((100 * sx, 50 * sy, 200 * sx, 150 * sy))


# --------------------------------------------------------- 거리 샘플링 (§4-1)

def _depth_mm(value_mm, shape=(120, 160)):
    return np.full(shape, value_mm, dtype=np.uint16)


class TestSampleDistance:
    def test_uniform_box(self):
        depth = _depth_mm(2500)
        assert sample_distance(depth, (40, 30, 120, 90)) == pytest.approx(2.5)

    def test_zero_is_excluded_from_aggregation(self):
        """★★ 지시서 4-2 1번 — 0을 섞어도 중앙값이 안 변해야 합니다.

        평균이었다면 2.5 -> 1.25로 끌려갑니다. 그게 '발밑 좌표' 사고입니다.
        """
        depth = _depth_mm(2500)
        depth[::2, :] = 0  # 절반을 측정 실패로
        got = sample_distance(depth, (40, 30, 120, 90))
        assert got == pytest.approx(2.5)

        naive_mean = float(np.mean(depth[30:90, 40:120])) * 0.001
        assert naive_mean < 1.5  # 0을 안 뺐다면 이 값이 나왔을 것

    def test_returns_none_when_all_invalid(self):
        depth = np.zeros((120, 160), dtype=np.uint16)
        assert sample_distance(depth, (40, 30, 120, 90)) is None

    def test_returns_none_when_valid_ratio_too_low(self):
        """★ 지시서 4-2 2번 — '거리 불명'이 잘못된 거리보다 낫습니다."""
        depth = np.zeros((120, 160), dtype=np.uint16)
        depth[46:48, 61:63] = 2500  # 중앙 영역(y 45..75, x 60..100)의 4픽셀만 유효
        assert sample_distance(depth, (40, 30, 120, 90), min_valid_ratio=0.2) is None
        # 임계치를 낮추면 같은 데이터로 값이 나옵니다 -> 비율 판정이 실제로 동작
        assert sample_distance(depth, (40, 30, 120, 90),
                               min_valid_ratio=0.0) == pytest.approx(2.5)

    def test_uses_central_region_not_whole_box(self):
        """★ 박스 가장자리는 배경입니다. 중앙 비율만 써야 배경이 안 섞입니다."""
        depth = _depth_mm(3800)          # 배경 3.8m (HP60C 상한 4m 안쪽)
        depth[50:70, 60:100] = 2000      # 박스 중앙의 물체 2m
        box = (40, 30, 120, 90)          # 중앙 50% -> x 60..100, y 45..75

        assert sample_distance(depth, box, central=0.5) == pytest.approx(2.0)
        # 박스 전체를 쓰면 배경이 다수라 3.8m로 끌려갑니다.
        assert sample_distance(depth, box, central=1.0) == pytest.approx(3.8)

    def test_median_beats_outliers(self):
        depth = _depth_mm(3000)
        depth[50:52, 60:100] = 3900  # 배경이 살짝 비침
        assert sample_distance(depth, (40, 30, 120, 90)) == pytest.approx(3.0)

    def test_out_of_image_box_does_not_crash(self):
        """★ 지시서 4-2 4번."""
        depth = _depth_mm(2500)
        assert sample_distance(depth, (-100, -100, 40, 40)) == pytest.approx(2.5)
        assert sample_distance(depth, (150, 110, 400, 300)) == pytest.approx(2.5)
        assert sample_distance(depth, (500, 500, 600, 600)) is None

    def test_float_meter_depth_also_works(self):
        depth = np.full((120, 160), 2.5, dtype=np.float32)
        assert sample_distance(depth, (40, 30, 120, 90)) == pytest.approx(2.5)

    def test_min_method_picks_nearest_surface(self):
        depth = _depth_mm(3800)
        depth[50:70, 60:100] = 2000
        assert sample_distance(depth, (40, 30, 120, 90),
                               method="min", central=1.0) == pytest.approx(2.0)

    def test_unknown_method_raises(self):
        with pytest.raises(ValueError):
            sample_distance(_depth_mm(2500), (40, 30, 120, 90), method="average")

    def test_default_z_range_matches_camera_spec(self):
        """★ 기본 유효범위는 실제 카메라(HP60C) 스펙 0.2~4m여야 합니다.

        스테레오는 범위 밖에서 오차가 커지는 게 아니라 **틀린 대응점**을 잡아
        그럴듯한 값을 냅니다. 넓혀두면 쓰레기가 좌표가 됩니다.
        """
        assert (DEFAULT_Z_MIN, DEFAULT_Z_MAX) == (0.2, 4.0)

        depth = _depth_mm(2500)
        depth[50:70, 60:100] = 5000   # 5m — 스펙 밖. 기본값이 걸러야 합니다.
        assert sample_distance(depth, (40, 30, 120, 90),
                               central=1.0) == pytest.approx(2.5)
        # 범위를 넓히면 같은 데이터로 값이 달라집니다 -> 필터가 실제로 동작
        assert sample_distance(depth, (40, 30, 120, 90), central=1.0,
                               z_max=10.0) == pytest.approx(2.5)
        # (min_valid_ratio=0: 5m 픽셀은 박스의 17%뿐이라 비율 검사에 먼저 걸립니다)
        assert sample_distance(depth, (40, 30, 120, 90), central=1.0,
                               z_min=4.5, z_max=10.0,
                               min_valid_ratio=0.0) == pytest.approx(5.0)


class TestSampleDistanceDetail:
    """노드가 '왜 거리를 못 구했는지'를 로그·진단 토픽에 남길 수 있어야 합니다."""

    def test_ok_result_carries_diagnostics(self):
        res = sample_distance_detail(_depth_mm(2500), (40, 30, 120, 90))
        assert isinstance(res, DistanceSample)
        assert res.distance == pytest.approx(2.5)
        assert res.reason == "ok"
        assert res.n_valid == res.n_total > 0
        assert res.valid_ratio == pytest.approx(1.0)
        assert res.spread == pytest.approx(0.0)

    def test_reason_distinguishes_failure_modes(self):
        depth = _depth_mm(2500)
        assert sample_distance_detail(depth, (500, 500, 600, 600)).reason == "box_outside_image"

        empty = np.zeros((120, 160), dtype=np.uint16)
        assert sample_distance_detail(empty, (40, 30, 120, 90)).reason == "no_valid_pixels"

        sparse = np.zeros((120, 160), dtype=np.uint16)
        sparse[46:48, 61:63] = 2500
        assert sample_distance_detail(sparse, (40, 30, 120, 90)).reason == "low_valid_ratio"

    def test_spread_rejects_boxes_straddling_two_surfaces(self):
        """앞/뒤가 반반이면 중앙값은 둘 중 하나를 고를 뿐 '대표'가 아닙니다."""
        depth = _depth_mm(2000)
        depth[:, 80:] = 3800
        res = sample_distance_detail(depth, (40, 30, 120, 90),
                                     central=1.0, max_spread_m=0.5)
        assert res.distance is None
        assert res.reason == "too_spread"
        assert res.spread > 0.5

    def test_failure_carries_no_distance(self):
        """거리 불명에 0이나 추정값이 절대 채워지면 안 됩니다 (지시서 8장)."""
        empty = np.zeros((120, 160), dtype=np.uint16)
        res = sample_distance_detail(empty, (40, 30, 120, 90))
        assert res.distance is None


class TestFlameFallbackRegions:
    """지시서 5-1 — 화염 위에서는 뎁스가 비어 있을 가능성이 큽니다.

    실측 전이라 '어느 영역을 쓸지'는 미정이지만, 영역 선택이 파라미터로
    빠져 있어야 실측 후 코드를 안 고치고 바꿀 수 있습니다.
    """

    def _flame_scene(self):
        # 배경 3.8m, 불이 놓인 바닥 3m, 불꽃 영역은 뎁스 무효(0)
        depth = _depth_mm(3800)
        depth[30:90, 40:120] = 3000
        depth[35:70, 50:110] = 0        # 화염 -> 측정 실패
        return depth

    def test_center_region_fails_on_flame(self):
        assert sample_distance(self._flame_scene(), (40, 30, 120, 90),
                               region="center", min_valid_ratio=0.2) is None

    def test_bottom_region_recovers_floor_distance(self):
        got = sample_distance(self._flame_scene(), (40, 30, 120, 90),
                              region="bottom", min_valid_ratio=0.2)
        assert got == pytest.approx(3.0)

    def test_ring_region_samples_surroundings(self):
        got = sample_distance(self._flame_scene(), (40, 30, 120, 90),
                              region="ring", min_valid_ratio=0.2)
        assert got is not None

    def test_unknown_region_raises(self):
        with pytest.raises(ValueError):
            sample_distance(_depth_mm(2500), (40, 30, 120, 90), region="corner")


# ------------------------------------------------------ 해상도 불일치 (§3-1)

class TestResolutionMismatch:
    """★ 컬러와 뎁스는 해상도도 **화각도** 다릅니다 (HP60C: 16:9 vs 4:3)."""

    # 실제 하드웨어 형상. 컬러는 전처리 축소본, 뎁스는 센서 원본.
    COLOR = (640, 360)
    DEPTH = (640, 480)

    def test_box_must_be_projected_into_the_depth_frame(self):
        k_color = k_from_hfov(*self.COLOR, 60.0)
        k_depth = k_from_hfov(*self.DEPTH, 60.0)
        dw, dh = self.DEPTH

        depth = np.full((dh, dw), 3800, dtype=np.uint16)

        box_color = (420.0, 40.0, 560.0, 140.0)   # 컬러 좌표계 (오른쪽 위)
        box_depth = project_box(box_color, k_color, k_depth)

        # 2m 물체를 "박스가 제대로 옮겨졌을 때 닿는 자리"에 정확히 놓습니다.
        x1, y1, x2, y2 = clip_box(box_depth, dw, dh)
        depth[y1:y2, x1:x2] = 2000

        assert sample_distance(depth, box_depth) == pytest.approx(2.0)
        # 옮기지 않고 그대로 쓰면 엉뚱한 픽셀 -> 배경
        assert sample_distance(depth, box_color) == pytest.approx(3.8)

    def test_scale_box_is_wrong_when_aspect_ratios_differ(self):
        """★★ 여기가 이 카메라의 핵심 함정.

        컬러 16:9와 뎁스 4:3은 세로 화각이 다릅니다. 해상도 배율로 옮기면
        화면 중앙만 맞고 위아래로 갈수록 어긋납니다 — 에러는 안 납니다.
        """
        k_color = k_from_hfov(*self.COLOR, 60.0)
        k_depth = k_from_hfov(*self.DEPTH, 60.0)
        sx = self.DEPTH[0] / self.COLOR[0]
        sy = self.DEPTH[1] / self.COLOR[1]

        # 화면 중앙은 두 방법이 일치합니다 (그래서 중앙만 보면 안 들킵니다)
        center = box_from_center(320.0, 180.0, 80.0, 80.0)
        assert box_center(project_box(center, k_color, k_depth)) == \
            pytest.approx(box_center(scale_box(center, sx, sy)))

        # 화면 위쪽에서는 30픽셀 어긋납니다
        high = box_from_center(320.0, 90.0, 80.0, 80.0)
        v_right = box_center(project_box(high, k_color, k_depth))[1]
        v_wrong = box_center(scale_box(high, sx, sy))[1]
        assert v_right == pytest.approx(150.0)
        assert v_wrong == pytest.approx(120.0)

        # 3.2m에서 세로 17cm — 물총 조준이 빗나가는 크기입니다
        err_m = abs(v_right - v_wrong) * 3.2 / k_depth[4]
        assert err_m > 0.15

    # 배율이 정확히 1/2이면 샘플점이 경계를 비껴가서 INTER_LINEAR로도 중간값이
    # 안 생깁니다(= 테스트가 아무것도 못 잡음). 일부러 정수배가 아닌 63으로 줄입니다.
    RESIZE_TO = (63, 63)

    def test_depth_resize_uses_nearest_not_linear(self):
        """★ 뎁스에 선형보간을 걸면 경계에 '존재하지 않는 중간 거리'가 생깁니다."""
        depth = np.zeros((100, 100), dtype=np.uint16)
        depth[:, :50] = 1000
        depth[:, 50:] = 5000

        small = resize_depth_nearest(depth, self.RESIZE_TO)
        present = set(np.unique(small).tolist())
        assert present <= {1000, 5000}, f"중간값이 생겼다면 보간이 걸린 것: {present}"

    def test_depth_resize_does_not_smear_holes(self):
        """0(측정 실패)이 보간에 섞이면 유효 픽셀이 0쪽으로 오염됩니다."""
        depth = np.full((100, 100), 3000, dtype=np.uint16)
        depth[:, 50:] = 0
        small = resize_depth_nearest(depth, self.RESIZE_TO)
        present = set(np.unique(small).tolist())
        assert present <= {0, 3000}, f"구멍이 번졌습니다: {present}"


# ------------------------------------------------------------- hole filling

class TestFillHoles:
    def test_fills_small_holes(self):
        depth = np.full((40, 40), 2.0, dtype=np.float32)
        depth[20, 20] = np.nan
        filled = fill_holes(depth, max_radius=3)
        assert filled[20, 20] == pytest.approx(2.0)

    def test_leaves_large_holes_alone(self):
        """큰 구멍을 메우면 '없는 거리'를 만드는 겁니다. 반경을 넘으면 포기."""
        depth = np.full((60, 60), 2.0, dtype=np.float32)
        depth[10:50, 10:50] = np.nan
        filled = fill_holes(depth, max_radius=2)
        assert np.isnan(filled[30, 30])

    def test_uses_median_so_boundaries_stay_sharp(self):
        """평균으로 메우면 경계에서 중간 거리가 생깁니다 (INTER_LINEAR와 같은 함정)."""
        depth = np.full((40, 40), 2.0, dtype=np.float32)
        depth[:, 20:] = 6.0
        depth[:, 19:21] = np.nan
        filled = fill_holes(depth, max_radius=3)
        vals = filled[~np.isnan(filled)]
        assert np.all((np.abs(vals - 2.0) < 1e-3) | (np.abs(vals - 6.0) < 1e-3))

    def test_does_not_mutate_input(self):
        depth = np.full((20, 20), 2.0, dtype=np.float32)
        depth[10, 10] = np.nan
        fill_holes(depth, max_radius=3)
        assert np.isnan(depth[10, 10])


# ------------------------------------------------------------------- TF (§4-1)

class TestToBaseLink:
    """고정 TF 적용만 합니다. 축 변환을 직접 짜서 두 번 돌리면 안 됩니다."""

    def test_identity(self):
        assert to_base_link((1.0, 2.0, 3.0), np.eye(4)) == pytest.approx((1.0, 2.0, 3.0))

    def test_translation_only(self):
        t = transform_matrix((0.1, 0.0, 0.5), (0.0, 0.0, 0.0, 1.0))
        assert to_base_link((1.0, 0.0, 0.0), t) == pytest.approx((1.1, 0.0, 0.5))

    def test_optical_to_ros_axis_convention(self):
        """optical(x 오른쪽, y 아래, z 앞) -> ROS(x 앞, y 왼쪽, z 위).

        정면 3m 앞의 점은 base_link에서 x=3이 되어야 합니다.
        y 부호가 뒤집히면 좌우가 반대인 곳에 물을 쏩니다.
        """
        q = (-0.5, 0.5, -0.5, 0.5)  # 표준 optical->link 회전 (xyzw)
        t = transform_matrix((0.0, 0.0, 0.0), q)
        assert to_base_link((0.0, 0.0, 3.0), t) == pytest.approx((3.0, 0.0, 0.0), abs=1e-9)
        assert to_base_link((1.0, 0.0, 0.0), t) == pytest.approx((0.0, -1.0, 0.0), abs=1e-9)
        assert to_base_link((0.0, 1.0, 0.0), t) == pytest.approx((0.0, 0.0, -1.0), abs=1e-9)

    def test_matrix_from_rpy_matches_quaternion(self):
        a = matrix_from_rpy(-np.pi / 2, 0.0, -np.pi / 2)
        b = transform_matrix((0.0, 0.0, 0.0), (-0.5, 0.5, -0.5, 0.5))
        assert a == pytest.approx(b, abs=1e-9)

    def test_accepts_geometry_msgs_like_object(self):
        """tf2가 주는 Transform을 그대로 받되, rclpy는 import하지 않습니다."""

        class _V:
            def __init__(self, x, y, z, w=None):
                self.x, self.y, self.z = x, y, z
                if w is not None:
                    self.w = w

        class _T:
            translation = _V(0.2, 0.0, 0.3)
            rotation = _V(0.0, 0.0, 0.0, 1.0)

        assert to_base_link((1.0, 0.0, 0.0), _T()) == pytest.approx((1.2, 0.0, 0.3))

    def test_accepts_translation_quaternion_tuple(self):
        tf = ((0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0))
        assert to_base_link((0.0, 0.0, 0.0), tf) == pytest.approx((0.0, 0.0, 1.0))

    def test_rejects_unnormalized_quaternion(self):
        with pytest.raises(ValueError):
            transform_matrix((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))

    def test_offline_helper_is_pure_convention(self):
        """로봇 없을 때만 쓰는 폴백. 실기에서는 tf2 조회가 정답입니다."""
        m = optical_to_base_link_matrix(translation=(0.15, 0.0, 0.4))
        assert to_base_link((0.0, 0.0, 3.0), m) == pytest.approx((3.15, 0.0, 0.4), abs=1e-9)


# --------------------------------------------------- 더미 파이프라인 (§7 검증2)

class TestSyntheticEndToEnd:
    """더미 박스 + 합성 뎁스로 '알려진 거리 -> 좌표'가 맞는지 (지시서 7장 2번).

    fake_detection_node가 발행하는 것과 **같은 함수**로 만든 데이터입니다.
    """

    def test_known_distance_gives_known_coordinates(self):
        w, h = 640, 480
        box = box_from_center(w / 2, h / 2, 80, 80)
        depth = synthetic_depth(w, h, background_m=3.8, targets=[(box, 3.2)])

        z = sample_distance(depth, box)
        assert z == pytest.approx(3.2, abs=1e-3)

        u, v = box_center(box)
        xyz = backproject(u, v, z, K_640)
        # 화면 정중앙 = 광축 위 -> 정면 3.2m
        assert xyz == pytest.approx((0.0, 0.0, 3.2), abs=1e-3)

        base = to_base_link(xyz, optical_to_base_link_matrix(translation=(0.1, 0.0, 0.35)))
        assert base == pytest.approx((3.3, 0.0, 0.35), abs=1e-3)

    def test_off_center_target_lands_left_of_robot(self):
        """화면 왼쪽의 불은 base_link에서 +y(왼쪽)여야 합니다."""
        w, h = 640, 480
        box = box_from_center(160.0, 240.0, 60, 60)
        depth = synthetic_depth(w, h, background_m=3.8, targets=[(box, 3.0)])

        z = sample_distance(depth, box)
        u, v = box_center(box)
        base = to_base_link(backproject(u, v, z, K_640), optical_to_base_link_matrix())
        assert base[0] == pytest.approx(3.0, abs=1e-3)
        assert base[1] > 0.5   # 왼쪽
        assert base[2] == pytest.approx(0.0, abs=1e-3)

    def test_flame_hole_yields_no_distance_not_a_wrong_one(self):
        """★ 계약: 거리 불명이면 좌표를 지어내지 않고 None."""
        w, h = 640, 480
        box = box_from_center(w / 2, h / 2, 80, 80)
        depth = synthetic_depth(w, h, background_m=3.8, targets=[(box, 3.2)],
                                hole_boxes=[box])
        assert sample_distance(depth, box) is None

    def test_synthetic_depth_is_uint16_millimeters(self):
        depth = synthetic_depth(64, 48, background_m=4.0)
        assert depth.dtype == np.uint16
        assert int(depth[0, 0]) == 4000

    def test_synthetic_depth_noise_is_reproducible(self):
        a = synthetic_depth(64, 48, background_m=4.0, noise_m=0.02, seed=7)
        b = synthetic_depth(64, 48, background_m=4.0, noise_m=0.02, seed=7)
        assert np.array_equal(a, b)


# ------------------------------------------------- vision_msgs 버전 차이 흡수

class _Pt:
    def __init__(self, x=0.0, y=0.0):
        self.x, self.y = x, y


class _Center4x:
    """vision_msgs 4.x (Humble 이후): center.position.x"""
    def __init__(self):
        self.position = _Pt()
        self.theta = 0.0


class _Center3x:
    """vision_msgs 3.x (Foxy/Galactic): center.x"""
    def __init__(self):
        self.x, self.y, self.theta = 0.0, 0.0, 0.0


class _BBox:
    def __init__(self, center, size_x=0.0, size_y=0.0):
        self.center, self.size_x, self.size_y = center, size_x, size_y


class _Hyp:
    def __init__(self, class_id="", score=0.0):
        self.class_id, self.score = class_id, score


class _Result4x:
    def __init__(self, class_id="", score=0.0):
        self.hypothesis = _Hyp(class_id, score)


class _Result3x:
    def __init__(self, id_=0, score=0.0):
        self.id, self.score = id_, score


class _Detection:
    def __init__(self, results):
        self.results = results


class TestDetectionMsgs:
    """★ vision_msgs는 버전마다 필드 모양이 다릅니다.

    로봇(팀원5 Hailo 배포)과 개발 PC의 배포판이 다를 수 있는데, 이 차이는
    "검출이 통째로 사라지는" 형태로 나타나서 원인을 찾기 어렵습니다.
    """

    def test_bbox_center_both_versions(self):
        b4 = _BBox(_Center4x())
        set_bbox_center(b4, 320.0, 240.0)
        assert bbox_center(b4) == pytest.approx((320.0, 240.0))
        assert b4.center.position.x == pytest.approx(320.0)

        b3 = _BBox(_Center3x())
        set_bbox_center(b3, 320.0, 240.0)
        assert bbox_center(b3) == pytest.approx((320.0, 240.0))
        assert b3.center.x == pytest.approx(320.0)

    def test_box_from_bbox_treats_size_as_size_not_corner(self):
        """★ size_x는 '폭'이지 우하단 x가 아닙니다. 헷갈리면 박스가 화면 밖으로."""
        bbox = _BBox(_Center4x(), size_x=40.0, size_y=20.0)
        set_bbox_center(bbox, 320.0, 240.0)
        assert box_from_bbox(bbox) == pytest.approx((300.0, 230.0, 340.0, 250.0))

    def test_box_from_bbox_feeds_sample_distance(self):
        """메시지 -> 박스 -> 거리까지 한 줄로 이어지는지 (실제 노드의 경로)."""
        bbox = _BBox(_Center4x(), size_x=80.0, size_y=80.0)
        set_bbox_center(bbox, 320.0, 240.0)
        depth = synthetic_depth(640, 480, background_m=6.0,
                                targets=[(box_from_bbox(bbox), 2.4)])
        assert sample_distance(depth, box_from_bbox(bbox)) == pytest.approx(2.4, abs=1e-3)

    def test_hypothesis_both_versions(self):
        assert hypothesis(_Result4x("fire", 0.9)) == ("fire", pytest.approx(0.9))
        assert hypothesis(_Result3x(3, 0.7)) == ("3", pytest.approx(0.7))

    def test_set_hypothesis_4x(self):
        r = _Result4x()
        set_hypothesis(r, "fire", 0.85)
        assert (r.hypothesis.class_id, r.hypothesis.score) == ("fire", pytest.approx(0.85))

    def test_set_hypothesis_3x_needs_numeric_class(self):
        r = _Result3x()
        set_hypothesis(r, "3", 0.85)
        assert r.id == 3
        with pytest.raises(ValueError):
            set_hypothesis(_Result3x(), "fire", 0.85)

    def test_best_score_and_class(self):
        det = _Detection([_Result4x("smoke", 0.3), _Result4x("fire", 0.8)])
        assert best_score(det) == pytest.approx(0.8)
        assert best_class(det) == "fire"

    def test_empty_results_do_not_crash(self):
        det = _Detection([])
        assert best_score(det) == 0.0
        assert best_class(det) == ""


# ------------------------------------------------ 더미 노드가 발행하는 장면 그대로

class TestDummyScene:
    """`fake_detection_node`가 발행하는 것과 **같은 함수**로 만든 장면.

    노드 안에서 장면을 조립하면 rclpy 없이 검증할 수 없고, 그러면 좌표가
    틀렸을 때 "더미가 틀렸는지 태스크②가 틀렸는지"를 구분할 수 없습니다.
    """

    def test_color_image_flame_is_where_the_depth_target_is(self):
        """★ 컬러의 불꽃과 뎁스의 대상이 **같은 자리**여야 합니다.

        사슬 점검(`full_chain_check.launch.py`)에서 YOLO가 컬러를 보고 낸
        박스로 뎁스를 샘플링합니다. 둘이 어긋나면 검증 장치 자체가
        "배선은 맞는데 거리가 이상한" 거짓 신호를 냅니다.
        """
        sc = dummy_scene(distance_m=3.2)
        img = sc.color_image()
        assert img.shape == (sc.color_size[1], sc.color_size[0], 3)
        assert img.dtype == np.uint8
        # 밝은 영역의 무게중심이 박스 중심과 맞는지.
        # (불꽃 중심은 포화돼 평평하므로 argmax는 동점 중 아무거나 고릅니다)
        gray = img.max(axis=2).astype(np.float64)
        hot = np.where(gray > 200, gray, 0.0)
        assert hot.sum() > 0, "불꽃이 그려지지 않았습니다"
        yy, xx = np.mgrid[0:gray.shape[0], 0:gray.shape[1]]
        u = float((xx * hot).sum() / hot.sum())
        v = float((yy * hot).sum() / hot.sum())
        cu, cv = box_center(sc.box_color)
        assert abs(u - cu) <= 2 and abs(v - cv) <= 2

    def test_color_image_is_darker_than_the_flame(self):
        """배경이 불꽃만큼 밝으면 태스크①·YOLO가 볼 게 없습니다."""
        sc = dummy_scene()
        img = sc.color_image()
        cu, cv = box_center(sc.box_color)
        flame = int(img[int(cv), int(cu)].max())
        corner = int(img[2, 2].max())
        assert flame > 200 and corner < 80

    def test_haze_raises_the_floor(self):
        """연기는 어두운 곳을 들어올립니다 — 디헤이즈가 되돌릴 대상."""
        sc = dummy_scene()
        clear = sc.color_image(haze=0.0)
        hazy = sc.color_image(haze=0.4)
        assert int(hazy[2, 2].max()) > int(clear[2, 2].max()) + 20

    def test_default_matches_driver_launch(self):
        """★ 기본값은 실제 드라이버 설정과 같아야 합니다.

        `ascamera.launch.py`가 rgb/depth 둘 다 640x480으로 엽니다.
        (인수인계 문서의 "RGB 1080p"는 launch와 어긋납니다 — launch가 실물입니다.)
        """
        sc = dummy_scene()
        assert isinstance(sc, DummyScene)
        assert sc.color_size == (640, 480)
        assert sc.depth_size == (640, 480)

    def test_projection_is_identity_at_current_settings(self):
        """지금은 화각·해상도가 같아 project_box가 사실상 항등입니다.

        그래도 `project_box`를 쓰는 이유는 아래 두 테스트 참조 — RGB 해상도를
        올리는 순간 항등이 아니게 되고, 그때 코드를 안 고쳐도 됩니다.
        """
        sc = dummy_scene()
        assert sc.k_color == pytest.approx(sc.k_depth)
        assert project_box(sc.box_color, sc.k_color, sc.k_depth) == \
            pytest.approx(sc.box_color)

    def test_background_stays_inside_camera_range(self):
        """0.2~4m 밖의 배경을 쓰면 실제로는 측정이 안 되는 장면입니다."""
        sc = dummy_scene()
        assert DEFAULT_Z_MIN < sc.distance_m < DEFAULT_Z_MAX
        assert DEFAULT_Z_MIN < sc.background_m < DEFAULT_Z_MAX

    def test_full_chain_recovers_ground_truth(self):
        """박스(컬러) -> 뎁스 좌표계 -> 거리 -> 역투영 -> base_link.

        태스크② 노드가 해야 할 일 전체입니다. 노드는 이 순서를 배선만 하면 됩니다.
        """
        sc = dummy_scene(distance_m=3.2, box_center_xy=(200.0, 90.0))
        depth = sc.depth_image()

        # ① 컬러 박스를 뎁스 좌표계로 — 배율이 아니라 **K를 거쳐서**
        box_depth = project_box(sc.box_color, sc.k_color, sc.k_depth)
        # ② 대표 거리
        z = sample_distance(depth, box_depth)
        assert z == pytest.approx(3.2, abs=1e-3)
        # ③ 역투영은 **컬러 K**로 (박스가 컬러 좌표계이므로)
        u, v = box_center(sc.box_color)
        optical = backproject(u, v, z, sc.k_color)
        assert optical == pytest.approx(sc.expected_optical(), abs=1e-3)
        # ④ 고정 TF
        base = to_base_link(optical, optical_to_base_link_matrix((0.1, 0.0, 0.35)))
        assert base == pytest.approx(sc.expected_base_link((0.1, 0.0, 0.35)), abs=1e-3)

    def test_skipping_the_projection_reads_the_background(self):
        """★ RGB를 1080p(16:9)로 올린 경우 — 박스를 안 옮기면 배경을 읽습니다.

        `color_size=(640, 360)` = 1920x1080을 전처리가 640폭으로 줄인 결과.
        성냥불 실측 결과 해상도를 올리면 이 형상이 됩니다 (지시서 6장).
        """
        sc = dummy_scene(color_size=(640, 360),
                         distance_m=3.2, background_m=3.8, box_center_xy=(200.0, 90.0))
        depth = sc.depth_image()
        assert sample_distance(depth, sc.box_color) == pytest.approx(3.8, abs=1e-3)

    def test_using_scale_box_instead_of_projection_misses(self):
        """★★ RGB 16:9 / 뎁스 4:3이면 배율 방식은 화면 위쪽에서 빗나갑니다."""
        sc = dummy_scene(color_size=(640, 360), distance_m=3.2, background_m=3.8,
                         box_size=(40.0, 40.0), box_center_xy=(200.0, 60.0))
        depth = sc.depth_image()

        # 제대로 옮기면 목표 거리
        assert sample_distance(depth, project_box(sc.box_color, sc.k_color, sc.k_depth)) \
            == pytest.approx(3.2, abs=1e-3)

        # 배율로 옮기면 목표를 완전히 빗나가 배경을 읽습니다
        sx = sc.depth_size[0] / sc.color_size[0]
        sy = sc.depth_size[1] / sc.color_size[1]
        assert sample_distance(depth, scale_box(sc.box_color, sx, sy)) \
            == pytest.approx(3.8, abs=1e-3)

    def test_wrong_k_gives_wrong_lateral_offset(self):
        """★ 뎁스 K로 역투영하면 상하 오차가 생깁니다 (컬러 K를 쓸 것).

        해상도가 갈리는 경우에만 드러나므로 16:9 설정으로 확인합니다.
        """
        sc = dummy_scene(color_size=(640, 360), distance_m=3.2,
                         box_center_xy=(200.0, 90.0))
        u, v = box_center(sc.box_color)
        right = backproject(u, v, 3.2, sc.k_color)
        wrong = backproject(u, v, 3.2, sc.k_depth)
        assert abs(right[1] - wrong[1]) > 0.15   # 세로로 17cm 이상

    def test_flame_hole_scene_has_no_ground_truth(self):
        sc = dummy_scene(flame_hole=True)
        assert sc.expected_optical() is None
        assert sc.expected_base_link() is None

        depth = sc.depth_image()
        box_depth = project_box(sc.box_color, sc.k_color, sc.k_depth)
        assert sample_distance(depth, box_depth) is None

    def test_flame_hole_ring_fallback_finds_the_wall(self):
        """지시서 5-1 폴백 후보가 이 장면에서 동작하는지 (판정은 실측 후)."""
        sc = dummy_scene(flame_hole=True, background_m=3.8)
        depth = sc.depth_image()
        box_depth = project_box(sc.box_color, sc.k_color, sc.k_depth)
        assert sample_distance(depth, box_depth, region="ring") \
            == pytest.approx(3.8, abs=1e-3)

    def test_depth_image_matches_driver_format(self):
        """`/camera/depth/image_raw` 는 16UC1(mm) 입니다 (인수인계 문서 2장)."""
        sc = dummy_scene()
        depth = sc.depth_image()
        assert depth.dtype == np.uint16
        assert depth.shape == (sc.depth_size[1], sc.depth_size[0])
        assert depth_unit_sanity(to_meters(depth))

    def test_noise_does_not_move_the_median_much(self):
        """뎁스 노이즈 2cm에서도 중앙값이 흔들리지 않아야 실측이 의미 있습니다."""
        sc = dummy_scene(distance_m=3.2, noise_m=0.02)
        depth = sc.depth_image(seed=1)
        z = sample_distance(depth, project_box(sc.box_color, sc.k_color, sc.k_depth))
        assert z == pytest.approx(3.2, abs=0.01)


class TestProjectBox:
    """K 기반 투영이 배율 방식의 상위 호환인지."""

    def test_identity_when_same_k(self):
        k = k_from_hfov(640, 480, 60.0)
        box = (100.0, 50.0, 200.0, 150.0)
        assert project_box(box, k, k) == pytest.approx(box)

    def test_matches_scale_box_when_k_is_pure_scale(self):
        """전처리 축소처럼 **같은 화각을 줄인** 관계면 두 방법이 같아야 합니다.

        같지 않으면 project_box를 기본으로 써도 잃는 게 있다는 뜻입니다.
        """
        k_src = k_from_hfov(1920, 1080, 60.0)
        sx, sy = 640 / 1920, 360 / 1080
        k_dst = scale_k(k_src, sx, sy)

        box = (500.0, 200.0, 900.0, 700.0)
        assert project_box(box, k_src, k_dst) == pytest.approx(scale_box(box, sx, sy))

    def test_principal_point_maps_to_principal_point(self):
        k_a = k_from_hfov(640, 360, 60.0)
        k_b = k_from_hfov(640, 480, 60.0)
        assert project_point(320.0, 180.0, k_a, k_b) == pytest.approx((320.0, 240.0))

    def test_roundtrip(self):
        k_a = k_from_hfov(640, 360, 60.0)
        k_b = k_from_hfov(640, 480, 60.0)
        u, v = project_point(200.0, 90.0, k_a, k_b)
        assert project_point(u, v, k_b, k_a) == pytest.approx((200.0, 90.0))

    def test_rejects_zero_focal_length(self):
        bad = [0.0, 0.0, 320.0, 0.0, 0.0, 240.0, 0.0, 0.0, 1.0]
        with pytest.raises(ValueError):
            project_point(1.0, 1.0, bad, k_from_hfov(640, 480, 60.0))

    def test_k_from_hfov_is_consistent(self):
        k = k_from_hfov(640, 480, 60.0)
        assert k[0] == pytest.approx((640 / 2) / np.tan(np.radians(30.0)))
        assert k[0] == pytest.approx(k[4])       # 정사각 픽셀
        assert (k[2], k[5]) == pytest.approx((320.0, 240.0))
        assert principal_point_sanity(k, 640, 480)



# ------------------------------------------------ 바닥면이 있는 장면 (지시서 5-1)

class TestGroundPlane:
    """카메라가 수평을 볼 때의 바닥면 뎁스.

    평면 벽만 있는 장면에서는 `bottom`/`below`/`ring`이 전부 같은 벽을 재서
    폴백 비교가 불가능합니다. 바닥이 있어야 셋이 갈립니다.
    """

    K = k_from_hfov(640, 480, 60.0)
    H = 0.35          # 카메라의 바닥 위 높이
    WALL = 3.8

    def test_recedes_downward(self):
        """★ 아래로 갈수록 **가까워집니다.** 이 부호가 폴백 편향의 원인입니다."""
        g = ground_plane_depth(640, 480, self.K, self.H, self.WALL)
        col = g[:, 320]
        below_horizon = col[300:]
        assert np.all(np.diff(below_horizon) < 0)

    def test_above_horizon_is_the_wall(self):
        g = ground_plane_depth(640, 480, self.K, self.H, self.WALL)
        assert g[100, 320] == pytest.approx(self.WALL)
        assert g[int(self.K[5]) - 1, 320] == pytest.approx(self.WALL)

    def test_contact_row_matches_formula(self):
        """v = cy + fy·h/Z. 이 식으로 '불이 바닥에 닿는 행'을 구합니다."""
        g = ground_plane_depth(640, 480, self.K, self.H, self.WALL)
        for z in (1.0, 2.0, 3.2):
            v = self.K[5] + self.K[4] * self.H / z
            assert g[int(v), 320] == pytest.approx(z, abs=0.02)

    def test_never_exceeds_wall(self):
        g = ground_plane_depth(640, 480, self.K, self.H, self.WALL)
        assert float(g.max()) == pytest.approx(self.WALL)

    def test_scene_places_box_on_the_floor(self):
        sc = dummy_scene(floor_height_m=self.H, box_on_floor=True, distance_m=3.2)
        box_d = project_box(sc.box_color, sc.k_color, sc.k_depth)
        v_contact = sc.k_depth[5] + sc.k_depth[4] * self.H / 3.2
        assert box_d[3] == pytest.approx(v_contact, abs=0.5)   # 아래변 = 접지점

    def test_box_on_floor_requires_floor(self):
        with pytest.raises(ValueError):
            dummy_scene(box_on_floor=True)


class TestFallbackBiasOnFloor:
    """★ 5-1 폴백 후보들의 **편향 방향과 크기**를 숫자로 못 박습니다.

    실측 전이라 어느 것을 쓸지는 못 정합니다. 하지만 "폴백이 값을 냈다 =
    맞다"가 아니라는 것, 그리고 **어느 쪽으로 얼마나 틀리는지**는 지금 정할
    수 있고, 그게 로봇 받았을 때 판단의 기준이 됩니다.
    """

    H, TRUE = 0.35, 3.2

    def _scene(self, flame_fills_box=True):
        sc = dummy_scene(floor_height_m=self.H, box_on_floor=True,
                         distance_m=self.TRUE, background_m=3.8)
        box = project_box(sc.box_color, sc.k_color, sc.k_depth)
        x1, y1, x2, y2 = box
        flame = box if flame_fills_box else (x1, y1, x2, y2 - (y2 - y1) * 0.2)
        depth = synthetic_depth(
            *sc.depth_size,
            background_m=ground_plane_depth(*sc.depth_size, sc.k_depth, self.H, 3.8),
            targets=[(box, self.TRUE)], hole_boxes=[flame])
        return depth, box

    def test_center_and_bottom_fail_when_flame_fills_the_box(self):
        """화염이 박스를 가득 채우면 박스 **안**에는 잴 게 없습니다."""
        depth, box = self._scene(flame_fills_box=True)
        assert sample_distance(depth, box, region="center") is None
        assert sample_distance(depth, box, region="bottom") is None

    def test_bottom_works_when_flame_leaves_the_lower_edge(self):
        """반대로 화염이 박스를 다 안 채우면 `bottom`이 대상 자체를 읽습니다."""
        depth, box = self._scene(flame_fills_box=False)
        assert sample_distance(depth, box, region="bottom") == pytest.approx(self.TRUE, abs=0.01)

    def test_below_biases_near_and_ring_biases_far(self):
        """★★ 두 폴백은 **반대 방향으로** 틀립니다. 둘 다 사유는 'ok'입니다."""
        depth, box = self._scene(flame_fills_box=True)
        below = sample_distance_detail(depth, box, region="below")
        ring = sample_distance_detail(depth, box, region="ring")

        assert below.reason == "ok" and ring.reason == "ok"
        # below: 바닥이 아래로 갈수록 가까워지므로 **가깝게**
        assert below.distance < self.TRUE
        # ring: 주변은 대상보다 뒤이므로 **멀게** (여기서는 벽)
        assert ring.distance > self.TRUE
        # 어느 쪽도 경고를 안 냅니다 — 그래서 표식이 따로 필요합니다
        assert below.valid_ratio == 1.0 and ring.valid_ratio == 1.0

    def test_below_bias_grows_with_band_thickness(self):
        """띠가 두꺼울수록 더 가깝게 잡힙니다 — band_ratio의 교환 관계."""
        depth, box = self._scene(flame_fills_box=True)
        errs = [sample_distance(depth, box, region="below", band_ratio=b) - self.TRUE
                for b in (0.05, 0.15, 0.30)]
        assert errs[0] > errs[1] > errs[2]      # 점점 더 음수로
        assert all(e < 0 for e in errs)

    def test_max_method_recovers_the_contact_point(self):
        """`below` 띠에서는 **가장 먼 값**이 접지점입니다 (중앙값이 아니라).

        띠 두께와 거의 무관해야 합니다 — 그게 접지점을 잡고 있다는 증거.
        """
        depth, box = self._scene(flame_fills_box=True)
        for band in (0.05, 0.15, 0.30):
            got = sample_distance(depth, box, region="below",
                                  band_ratio=band, method="max")
            assert got == pytest.approx(self.TRUE, abs=0.08)

    def test_max_is_worse_than_median_elsewhere(self):
        """`max`는 접지점 전용입니다. 일반 박스에 쓰면 배경 비침을 고릅니다."""
        depth = _depth_mm(2000)
        depth[50:52, 60:100] = 3800          # 배경이 살짝 비침
        box = (40, 30, 120, 90)
        assert sample_distance(depth, box, method="median") == pytest.approx(2.0)
        assert sample_distance(depth, box, method="max") == pytest.approx(3.8)

    def test_p75_sits_between_median_and_max(self):
        depth, box = self._scene(flame_fills_box=True)
        vals = {m: sample_distance(depth, box, region="below", method=m)
                for m in ("median", "p75", "max")}
        assert vals["median"] < vals["p75"] < vals["max"]


class TestCascade:
    """폴백 순서 — 어느 단계가 나왔는지가 결과의 일부입니다."""

    H = 0.35
    TRUE = 3.2

    def _scene(self, flame_fills_box=True):
        sc = dummy_scene(floor_height_m=self.H, box_on_floor=True,
                         distance_m=self.TRUE, background_m=3.8)
        box = project_box(sc.box_color, sc.k_color, sc.k_depth)
        x1, y1, x2, y2 = box
        flame = box if flame_fills_box else (x1, y1, x2, y2 - (y2 - y1) * 0.2)
        depth = synthetic_depth(
            *sc.depth_size,
            background_m=ground_plane_depth(*sc.depth_size, sc.k_depth, self.H, 3.8),
            targets=[(box, self.TRUE)], hole_boxes=[flame])
        return depth, box

    def test_center_wins_when_it_can(self):
        """폴백은 대상이 보일 때 끼어들면 안 됩니다."""
        depth = _depth_mm(2500)
        got = sample_distance_cascade(depth, (40, 30, 120, 90))
        assert got.region == "center"
        assert got.distance == pytest.approx(2.5)

    def test_falls_through_to_below_when_flame_fills_box(self):
        """★ 화염이 박스를 채우면 center·bottom이 죽고 below가 받습니다."""
        depth, box = self._scene(flame_fills_box=True)
        got = sample_distance_cascade(depth, box)
        assert got.region == "below"
        assert got.distance == pytest.approx(self.TRUE, abs=0.10)

    def test_bottom_catches_it_when_flame_leaves_a_gap(self):
        """하단에 대상이 조금이라도 보이면 below까지 안 갑니다 (편향이 없으므로)."""
        depth, box = self._scene(flame_fills_box=False)
        got = sample_distance_cascade(depth, box)
        assert got.region == "bottom"
        assert got.distance == pytest.approx(self.TRUE, abs=0.05)

    def test_all_stages_fail_returns_unknown_with_first_reason(self):
        """★ 전부 실패하면 **첫 단계**의 사유를 남깁니다.

        마지막(ring)의 사유를 주면 "주변에 픽셀이 없다"가 되어, 정작 알고 싶은
        "대상에 왜 없는가"가 가려집니다.
        """
        depth = _depth_mm(0)                     # 전부 측정 실패
        got = sample_distance_cascade(depth, (40, 30, 120, 90))
        assert got.distance is None
        assert got.region == "center"
        assert got.reason == "no_valid_pixels"

    def test_out_of_spec_distance_is_not_rescued_by_fallbacks(self):
        """4m 밖은 어느 단계도 값을 내면 안 됩니다 (카메라 스펙 0.2~4m)."""
        depth = _depth_mm(9000)
        assert sample_distance_cascade(depth, (40, 30, 120, 90)).distance is None

    def test_region_and_method_kwargs_are_rejected(self):
        """stages가 정하는 값을 kwargs로도 받으면 조용히 한쪽이 무시됩니다."""
        depth = _depth_mm(2500)
        with pytest.raises(TypeError):
            sample_distance_cascade(depth, (40, 30, 120, 90), region="ring")
        with pytest.raises(TypeError):
            sample_distance_cascade(depth, (40, 30, 120, 90), method="max")

    def test_default_cascade_pairs_below_with_max(self):
        """★ `below`에 median을 쓰면 -0.33m 편향됩니다 (max는 -0.05m).

        HANDOVER 8의 실측 결과를 기본값이 계속 반영하는지 잠급니다.
        """
        assert ("below", "max") in DEFAULT_CASCADE
        assert ("below", "median") not in DEFAULT_CASCADE

    def test_default_cascade_tries_target_before_surroundings(self):
        """대상을 재는 단계가 주변을 재는 단계보다 앞에 있어야 합니다."""
        order = [r for r, _ in DEFAULT_CASCADE]
        assert order.index("center") < order.index("below") < order.index("ring")


class TestParseCascade:
    def test_roundtrip(self):
        assert parse_cascade(cascade_to_str(DEFAULT_CASCADE)) == DEFAULT_CASCADE

    def test_method_defaults_to_median(self):
        assert parse_cascade("center,ring") == (("center", "median"),
                                                ("ring", "median"))

    def test_whitespace_is_tolerated(self):
        assert parse_cascade(" center:median , below:max ") == (
            ("center", "median"), ("below", "max"))

    @pytest.mark.parametrize("bad", ["", "  ", ",,"])
    def test_empty_raises(self, bad):
        with pytest.raises(ValueError):
            parse_cascade(bad)

    @pytest.mark.parametrize("bad", ["centre:median", "center:avg", "diagonal"])
    def test_typos_raise_at_startup_not_in_callback(self, bad):
        """★ 콜백 안에서 터지면 '검출이 조용히 사라지는' 형태로 보입니다."""
        with pytest.raises(ValueError):
            parse_cascade(bad)
