#!/usr/bin/env python3
"""
태스크② 프레임 변환 테스트 — rclpy 없이 돕니다.

여기서 잠그는 것은 "무엇을 발행하고 무엇을 버리는가"입니다. 이 판단이 틀리면
**예외 없이 화재 지점 좌표가 틀립니다.** 특히:

  - 거리 불명인데 좌표를 만들어 발행   -> 메인이 로봇 발밑을 화재로 계산
  - 폴백이 잰 주변 거리를 대상 거리로  -> 물총이 불 뒤 벽을 맞춤
  - 일부가 실패했다고 프레임 전체 폐기 -> 멀쩡한 검출까지 사라짐
"""

from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from image_pipeline.depth import (  # noqa: E402
    backproject,
    box_center,
    dummy_scene,
    ground_plane_depth,
    optical_to_base_link_matrix,
    project_box,
    synthetic_depth,
    to_base_link,
)
from image_pipeline.detection3d import (  # noqa: E402
    Detected3D,
    SamplingParams,
    StampMonitor,
    convert_frame,
)

CAM_OFFSET = (0.1, 0.0, 0.35)


def _tf():
    return optical_to_base_link_matrix(CAM_OFFSET)


class TestConvertFrame:
    def test_single_detection_matches_ground_truth(self):
        sc = dummy_scene(distance_m=3.2)
        res = convert_frame([(sc.box_color, "fire", 0.9)], sc.depth_image(),
                            sc.k_color, sc.k_depth, _tf())

        assert len(res.detections) == 1 and not res.dropped
        d = res.detections[0]
        assert isinstance(d, Detected3D)
        assert d.xyz == pytest.approx(sc.expected_base_link(CAM_OFFSET), abs=1e-3)
        # 기본 region은 "bottom" — 화염 위 뎁스가 비어서 center를 내렸습니다
        assert (d.class_id, d.region, d.is_fallback) == ("fire", "bottom", False)
        assert d.distance_m == pytest.approx(3.2, abs=1e-3)

    def test_uses_color_k_for_backprojection(self):
        """★ 박스가 컬러 좌표계이므로 역투영도 컬러 K로.

        해상도가 갈리는 설정(RGB 1080p 상당)에서만 드러납니다.
        """
        sc = dummy_scene(color_size=(640, 360), distance_m=3.2,
                         box_center_xy=(200.0, 90.0))
        res = convert_frame([(sc.box_color, "fire", 0.9)], sc.depth_image(),
                            sc.k_color, sc.k_depth, _tf())
        got = res.detections[0].xyz

        u, v = box_center(sc.box_color)
        right = to_base_link(backproject(u, v, 3.2, sc.k_color), _tf())
        wrong = to_base_link(backproject(u, v, 3.2, sc.k_depth), _tf())

        assert got == pytest.approx(right, abs=1e-3)
        assert abs(got[2] - wrong[2]) > 0.15

    def test_empty_input_is_empty_output(self):
        sc = dummy_scene()
        res = convert_frame([], sc.depth_image(), sc.k_color, sc.k_depth, _tf())
        assert res.detections == [] and res.dropped == []


class TestDropInsteadOfInvent:
    """★★ 계약: 거리를 못 구하면 좌표를 지어내지 않습니다 (HANDOVER 7-3)."""

    def test_unknown_distance_is_dropped_not_zeroed(self):
        sc = dummy_scene(flame_hole=True)
        res = convert_frame([(sc.box_color, "fire", 0.9)], sc.depth_image(),
                            sc.k_color, sc.k_depth, _tf())

        assert res.detections == []
        assert len(res.dropped) == 1
        assert res.dropped[0].reason == "no_valid_pixels"
        # 0m 좌표가 섞여 나가면 메인이 로봇 발밑을 화재 지점으로 씁니다
        assert all(d.xyz != (0.0, 0.0, 0.0) for d in res.detections)

    def test_partial_failure_keeps_the_good_ones(self):
        """★ 하나가 실패해도 나머지는 발행돼야 합니다."""
        sc = dummy_scene(distance_m=3.0, background_m=3.8)
        w, h = sc.depth_size
        good_box = sc.box_color
        # 화면 왼쪽에 뎁스가 통째로 빈 박스를 하나 더
        bad_box = (40.0, 40.0, 120.0, 120.0)
        depth = synthetic_depth(w, h, background_m=3.8,
                                targets=[(project_box(good_box, sc.k_color, sc.k_depth), 3.0)],
                                hole_boxes=[project_box(bad_box, sc.k_color, sc.k_depth)])

        res = convert_frame([(good_box, "fire", 0.9), (bad_box, "smoke", 0.8)],
                            depth, sc.k_color, sc.k_depth, _tf())
        assert [d.class_id for d in res.detections] == ["fire"]
        assert [d.class_id for d in res.dropped] == ["smoke"]

    def test_reason_counts(self):
        sc = dummy_scene(flame_hole=True)
        res = convert_frame([(sc.box_color, "fire", 0.9), (sc.box_color, "fire", 0.8)],
                            sc.depth_image(), sc.k_color, sc.k_depth, _tf())
        assert res.reason_counts() == {"no_valid_pixels": 2}

    def test_low_score_is_dropped_with_its_own_reason(self):
        sc = dummy_scene()
        p = SamplingParams(min_score=0.5)
        res = convert_frame([(sc.box_color, "fire", 0.2)], sc.depth_image(),
                            sc.k_color, sc.k_depth, _tf(), p)
        assert res.detections == []
        assert res.dropped[0].reason == "low_score"

    def test_box_outside_image_is_dropped(self):
        sc = dummy_scene()
        far_out = (5000.0, 5000.0, 5100.0, 5100.0)
        res = convert_frame([(far_out, "fire", 0.9)], sc.depth_image(),
                            sc.k_color, sc.k_depth, _tf())
        assert res.detections == []
        assert res.dropped[0].reason == "box_outside_image"


class TestFallbackPolicy:
    """지시서 5-1 폴백 — 켜면 값이 나오지만 **대상 거리가 아닙니다.**"""

    H, TRUE = 0.35, 3.2

    def _flame_scene(self):
        sc = dummy_scene(floor_height_m=self.H, box_on_floor=True,
                         distance_m=self.TRUE, background_m=3.8)
        box_d = project_box(sc.box_color, sc.k_color, sc.k_depth)
        depth = synthetic_depth(
            *sc.depth_size,
            background_m=ground_plane_depth(*sc.depth_size, sc.k_depth, self.H, 3.8),
            targets=[(box_d, self.TRUE)], hole_boxes=[box_d])
        return sc, depth

    def _partial_flame_scene(self, hole_fraction=0.75):
        """화염이 박스의 **위쪽만** 채운 장면 — 아래쪽에 대상이 남아 있습니다.

        기본 region이 `center`에서 `bottom`으로 바뀐 이유가 이 장면입니다.
        중앙은 불꽃이라 뎁스가 비고, 박스 아래쪽은 살아 있습니다.
        """
        sc = dummy_scene(floor_height_m=self.H, box_on_floor=True,
                         distance_m=self.TRUE, background_m=3.8)
        box_d = project_box(sc.box_color, sc.k_color, sc.k_depth)
        x1, y1, x2, y2 = box_d
        hole = (x1, y1, x2, y1 + (y2 - y1) * hole_fraction)
        depth = synthetic_depth(
            *sc.depth_size,
            background_m=ground_plane_depth(*sc.depth_size, sc.k_depth, self.H, 3.8),
            targets=[(box_d, self.TRUE)], hole_boxes=[hole])
        return sc, depth

    def test_기본_bottom은_화염_아래의_대상을_읽는다(self):
        """★ 2026-08-24 실기 확인: 성냥불 위에서 뎁스가 안 나옵니다.

        `center`였다면 불꽃 한가운데를 재서 검출이 통째로 버려집니다.
        `bottom`은 박스 **안**의 아래쪽이라 폴백이 아니라 대상 자체를 읽습니다.
        """
        sc, depth = self._partial_flame_scene()
        res = convert_frame([(sc.box_color, "fire", 0.9)], depth,
                            sc.k_color, sc.k_depth, _tf())

        assert len(res.detections) == 1 and not res.dropped
        d = res.detections[0]
        assert d.region == "bottom"
        assert d.is_fallback is False, "박스 안이므로 폴백이 아닙니다"
        assert d.distance_m == pytest.approx(self.TRUE, abs=1e-3)

    def test_center였다면_같은_장면에서_버려진다(self):
        """바뀐 기본값이 실제로 무엇을 구했는지 대조군으로 못 박습니다."""
        sc, depth = self._partial_flame_scene()
        p = SamplingParams(region="center")
        res = convert_frame([(sc.box_color, "fire", 0.9)], depth,
                            sc.k_color, sc.k_depth, _tf(), p)
        assert res.detections == []
        assert res.dropped[0].reason == "no_valid_pixels"

    def test_fallback_is_off_by_default(self):
        """★ 기본값으로는 폴백이 안 돕니다. 실측 전에 켜면 위험합니다.

        화염이 박스를 **가득** 채우면 `bottom`(박스 안)도 막힙니다. 이때
        값이 나오는 건 `below`뿐이고, 그건 명시적으로 켜야 합니다.
        """
        sc, depth = self._flame_scene()
        res = convert_frame([(sc.box_color, "fire", 0.9)], depth,
                            sc.k_color, sc.k_depth, _tf())
        assert res.detections == []
        assert res.dropped[0].reason == "no_valid_pixels"

    def test_enabling_fallback_produces_a_marked_result(self):
        sc, depth = self._flame_scene()
        p = SamplingParams(fallback_regions=("below",))
        res = convert_frame([(sc.box_color, "fire", 0.9)], depth,
                            sc.k_color, sc.k_depth, _tf(), p)

        assert len(res.detections) == 1
        d = res.detections[0]
        assert d.is_fallback is True, "폴백 결과에 표식이 없으면 대상 거리로 오해됩니다"
        assert d.region == "below"

    def test_fallback_cascade_order(self):
        """앞의 영역이 실패해야 다음으로 넘어갑니다."""
        sc, depth = self._flame_scene()
        p = SamplingParams(fallback_regions=("bottom", "below", "ring"))
        res = convert_frame([(sc.box_color, "fire", 0.9)], depth,
                            sc.k_color, sc.k_depth, _tf(), p)
        # bottom 은 박스 안이라 화염 구멍에 막히고, below 가 바닥을 잡습니다
        assert res.detections[0].region == "below"

    def test_primary_result_is_not_marked_as_fallback(self):
        sc = dummy_scene(distance_m=3.2)
        p = SamplingParams(fallback_regions=("below", "ring"))
        res = convert_frame([(sc.box_color, "fire", 0.9)], sc.depth_image(),
                            sc.k_color, sc.k_depth, _tf(), p)
        assert res.detections[0].is_fallback is False
        assert res.detections[0].region == "bottom"

    def test_fallback_distance_is_biased_not_exact(self):
        """★ 폴백은 값이 나와도 대상 거리가 아닙니다. 오차를 기록해 둡니다."""
        sc, depth = self._flame_scene()
        p = SamplingParams(fallback_regions=("below",), fallback_method="median")
        res = convert_frame([(sc.box_color, "fire", 0.9)], depth,
                            sc.k_color, sc.k_depth, _tf(), p)
        # 바닥은 아래로 갈수록 가까워지므로 중앙값은 **가깝게** 잡힙니다
        assert res.detections[0].distance_m < self.TRUE - 0.1


class TestStampMonitor:
    """★ 파이프라인이 stamp를 덮어썼는지 잡아내는 장치.

    "검출 stamp - 뎁스 stamp"를 비교하는 방식은 **원리적으로 못 잡습니다** —
    동기화기가 가장 가까운 뎁스를 대신 골라주기 때문에 차이가 상쇄됩니다
    (실측으로 확인). 그래서 **카메라가 낸 stamp와 정확히 일치하는지**를 봅니다.
    """

    def _feed(self, m, n, offset_ns=0):
        """카메라 stamp n개를 넣고, 검출을 offset만큼 밀어서 확인시킵니다."""
        fired = []
        for i in range(n):
            key = (100 + i, 0)
            m.note_camera_stamp(key)
            fired.append(m.check_detection((key[0], key[1] + offset_ns)))
        return fired

    def test_quiet_when_detection_reuses_camera_stamp(self):
        m = StampMonitor(window=10)
        assert not any(self._feed(m, 40))
        assert m.match_ratio() == pytest.approx(1.0)

    def test_warns_when_stamp_was_minted_locally(self):
        """now()로 새로 만든 값은 카메라 stamp와 nanosec까지 같을 수 없습니다."""
        m = StampMonitor(window=10)
        fired = self._feed(m, 40, offset_ns=12_345_678)   # 12ms 밀림
        assert sum(fired) == 1, "경고는 한 번만 나야 로그가 안 막힙니다"
        assert m.match_ratio() == pytest.approx(0.0)
        assert "stamp" in m.message()

    def test_needs_a_full_window_before_warning(self):
        m = StampMonitor(window=30)
        assert not any(self._feed(m, 20, offset_ns=12_345_678))

    def test_waits_for_camera_stamps_first(self):
        """카메라 stamp를 못 본 상태에서는 판단하지 않습니다 (시작 직후)."""
        m = StampMonitor(window=3)
        assert not any(m.check_detection((100 + i, 0)) for i in range(10))

    def test_history_is_long_enough_for_pipeline_latency(self):
        """★ 검출은 파이프라인 지연만큼 **늦게** 옵니다. 그동안 카메라 stamp가
        계속 쌓이므로, 이력이 짧으면 정상인데도 불일치로 오판합니다."""
        m = StampMonitor(history=90, window=10)
        stamps = [(100 + i, 0) for i in range(60)]
        for k in stamps:
            m.note_camera_stamp(k)
        # 30프레임(=2초) 전의 stamp를 단 검출이 지금 도착
        fired = [m.check_detection(stamps[i]) for i in range(30, 45)]
        assert not any(fired)
        assert m.match_ratio() == pytest.approx(1.0)

    def test_short_history_would_false_alarm(self):
        """이력을 짧게 잡으면 정상 파이프라인을 오판한다는 근거."""
        m = StampMonitor(history=5, window=10)
        stamps = [(100 + i, 0) for i in range(60)]
        for k in stamps:
            m.note_camera_stamp(k)
        assert any(m.check_detection(stamps[i]) for i in range(30, 45))

    def test_duplicate_camera_stamps_do_not_grow_history(self):
        m = StampMonitor(history=10, window=5)
        for _ in range(50):
            m.note_camera_stamp((100, 0))
        assert not any(m.check_detection((100, 0)) for _ in range(20))
