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
    convert_frame_pixels,
    method_for,
    parse_region_by_class,
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


class TestRegionByClass:
    """★ 클래스마다 재는 위치가 다릅니다 (2026-08-26 실기 실측).

    fire   : 박스 대부분이 불꽃이라 박스 **안**에 대상 표면이 없습니다.
             bottom으로 재면 불꽃 사이로 보이는 **배경(벽)**이 잡혀 멀게 나갑니다.
    person : below로 재면 사람 **앞쪽 바닥**을 재서 가깝게 나옵니다.

    이 두 방향의 오차가 서로 반대라, 한 영역으로 통일하면 어느 한쪽이 반드시
    틀립니다.
    """

    H, TRUE, WALL = 0.35, 3.2, 3.8

    def _scene(self, flame_fill=0.0):
        """flame_fill 비율만큼 박스 위쪽 뎁스가 비는 장면."""
        sc = dummy_scene(floor_height_m=self.H, box_on_floor=True,
                         distance_m=self.TRUE, background_m=self.WALL)
        box_d = project_box(sc.box_color, sc.k_color, sc.k_depth)
        x1, y1, x2, y2 = box_d
        holes = ([] if flame_fill <= 0
                 else [(x1, y1, x2, y1 + (y2 - y1) * flame_fill)])
        depth = synthetic_depth(
            *sc.depth_size,
            background_m=ground_plane_depth(*sc.depth_size, sc.k_depth,
                                            self.H, self.WALL),
            targets=[(box_d, self.TRUE)], hole_boxes=holes)
        return sc, depth

    def _flame_only_scene(self, flame_fill=0.7):
        """박스 안에 **대상 표면이 없는** 장면 — 실기의 화재가 이렇습니다.

        `_scene`과의 차이가 핵심입니다. `_scene`은 박스 안에 대상(3.2m)이
        있고 그 위를 불꽃이 가립니다. 여기는 박스가 통째로 불꽃이라 대상
        표면 자체가 없고, 뎁스가 잡히는 건 **불꽃 사이로 보이는 배경**뿐입니다.
        그래서 bottom이 벽을 읽습니다.
        """
        sc = dummy_scene(floor_height_m=self.H, box_on_floor=True,
                         distance_m=self.TRUE, background_m=self.WALL)
        box_d = project_box(sc.box_color, sc.k_color, sc.k_depth)
        x1, y1, x2, y2 = box_d
        depth = synthetic_depth(
            *sc.depth_size,
            background_m=ground_plane_depth(*sc.depth_size, sc.k_depth,
                                            self.H, self.WALL),
            targets=[],          # ★ 박스 안에 대상 표면이 없습니다
            hole_boxes=[(x1, y1, x2, y1 + (y2 - y1) * flame_fill)])
        return sc, depth

    def _params(self):
        return SamplingParams(
            region="bottom",
            region_by_class={"fire": "below", "person": "bottom"})

    # ---------------------------------------------------------------- 매핑

    def test_클래스마다_다른_영역을_쓴다(self):
        sc, depth = self._scene()
        res = convert_frame([(sc.box_color, "fire", 0.9),
                             (sc.box_color, "person", 0.8)],
                            depth, sc.k_color, sc.k_depth, _tf(), self._params())
        got = {d.class_id: d.region for d in res.detections}
        assert got == {"fire": "below", "person": "bottom"}

    def test_매핑에_없는_클래스는_기본값을_쓴다(self):
        sc, depth = self._scene()
        res = convert_frame([(sc.box_color, "smoke", 0.9)], depth,
                            sc.k_color, sc.k_depth, _tf(), self._params())
        assert res.detections[0].region == "bottom"

    def test_매핑이_비면_모두_기본값이다(self):
        sc, depth = self._scene()
        res = convert_frame([(sc.box_color, "fire", 0.9)], depth,
                            sc.k_color, sc.k_depth, _tf(), SamplingParams())
        assert res.detections[0].region == "bottom"

    def test_라벨_대소문자가_다르면_매핑이_안_걸린다(self):
        """★ 조용히 기본값으로 떨어집니다. 노드가 시작 로그로 알리는 이유."""
        sc, depth = self._scene()
        res = convert_frame([(sc.box_color, "Fire", 0.9)], depth,
                            sc.k_color, sc.k_depth, _tf(), self._params())
        assert res.detections[0].region == "bottom"   # below 가 아님

    # ------------------------------------------------------- method 동반 (★핵심)

    def test_below로_바꾸면_method도_max로_따라간다(self):
        """★★ region만 바꾸고 method를 안 바꾸면 -0.326m가 나갑니다.

        `as_kwargs`는 method를 안 주면 `self.method`(median)를 씁니다.
        `REGION_METHOD`는 폴백 경로에서만 적용되므로, 클래스 매핑이 직접
        method를 정해줘야 합니다.
        """
        p = self._params()
        assert p.region_for("fire") == ("below", "max")
        assert p.region_for("person") == ("bottom", "median")
        assert p.region_for("smoke") == ("bottom", "median")

    def test_max가_실제로_적용되어_접지점을_잡는다(self):
        """median이었다면 -0.3m대가 나옵니다. 값으로 잠급니다."""
        sc, depth = self._scene(flame_fill=1.0)      # 박스가 통째로 불꽃
        res = convert_frame([(sc.box_color, "fire", 0.9)], depth,
                            sc.k_color, sc.k_depth, _tf(), self._params())
        assert len(res.detections) == 1
        assert res.detections[0].distance_m == pytest.approx(self.TRUE, abs=0.1)

    def test_기본_region은_method_파라미터를_존중한다(self):
        """매핑에 안 걸린 클래스까지 REGION_METHOD로 덮으면 기존 사용법이 깨집니다."""
        p = SamplingParams(region="bottom", method="p25",
                           region_by_class={"fire": "below"})
        assert p.region_for("person") == ("bottom", "p25")

    # ---------------------------------------------------- 두 오차의 방향이 반대

    def test_fire를_bottom으로_재면_배경이_잡힌다(self):
        """실기 증상 재현 — 박스가 불이면 bottom은 벽을 읽습니다.

        ★ 사유가 `ok`이고 유효비율도 멀쩡합니다. 가드로는 못 막는 이유입니다 —
        픽셀이 부족한 게 아니라 **있는 픽셀 전부가 배경**이라서입니다.
        """
        sc, depth = self._flame_only_scene(flame_fill=0.7)
        p = SamplingParams(region="bottom")           # 매핑 없음 = 옛 동작
        res = convert_frame([(sc.box_color, "fire", 0.9)], depth,
                            sc.k_color, sc.k_depth, _tf(), p)
        d = res.detections[0]
        assert d.distance_m > self.TRUE + 0.05, \
            "박스 안에 대상이 없으면 bottom은 멀게 나갑니다"
        assert d.valid_ratio > 0.5, "픽셀이 부족해서가 아닙니다"

    def test_같은_장면에서_below는_접지점을_잡는다(self):
        """대조군 — below는 박스 바깥이라 불꽃이 박스를 채워도 무관합니다."""
        sc, depth = self._flame_only_scene(flame_fill=0.7)
        res = convert_frame([(sc.box_color, "fire", 0.9)], depth,
                            sc.k_color, sc.k_depth, _tf(), self._params())
        assert res.detections[0].distance_m == pytest.approx(self.TRUE, abs=0.1)

    def test_person을_below로_재면_가깝게_나온다(self):
        """실기 증상 재현 — below는 앞쪽 바닥이라 가깝습니다."""
        sc, depth = self._scene()                     # 뎁스 구멍 없음(사람)
        p = SamplingParams(region="below", method="max")
        res = convert_frame([(sc.box_color, "person", 0.8)], depth,
                            sc.k_color, sc.k_depth, _tf(), p)
        assert res.detections[0].distance_m < self.TRUE, \
            "below는 대상 앞쪽 바닥을 재므로 가깝게 나옵니다"

    def test_매핑을_쓰면_둘_다_맞는다(self):
        """위 두 실패가 매핑으로 동시에 해소되는지 — 이 변경의 목적입니다."""
        sc, depth = self._scene(flame_fill=0.7)
        res = convert_frame([(sc.box_color, "fire", 0.9),
                             (sc.box_color, "person", 0.8)],
                            depth, sc.k_color, sc.k_depth, _tf(), self._params())
        by = {d.class_id: d.distance_m for d in res.detections}
        assert by["fire"] == pytest.approx(self.TRUE, abs=0.1)
        assert by["person"] == pytest.approx(self.TRUE, abs=0.01)

    # ------------------------------------------------------------- 표식 정합성

    def test_below가_1차여도_주변측정으로_표시된다(self):
        """★ "폴백으로 넘어갔는가"가 아니라 "무엇을 쟀는가"로 판단해야 합니다.

        fire의 1차가 below라 1차 성공이 곧 대상 측정이 아닙니다. 표식이
        틀리면 메인과 물총이 바닥 거리를 대상 거리로 씁니다.
        """
        sc, depth = self._scene()
        res = convert_frame([(sc.box_color, "fire", 0.9),
                             (sc.box_color, "person", 0.8)],
                            depth, sc.k_color, sc.k_depth, _tf(), self._params())
        flag = {d.class_id: d.is_fallback for d in res.detections}
        assert flag == {"fire": True, "person": False}

    def test_json_상태도_클래스마다_갈린다(self):
        sc, depth = self._scene()
        res = convert_frame_pixels([(sc.box_color, "fire", 0.9),
                                    (sc.box_color, "person", 0.8)],
                                   depth, sc.k_color, sc.k_depth,
                                   params=self._params())
        got = {e["class_name"]: e["depth_status"] for e in res.entries}
        assert got == {"fire": "fallback_below", "person": "fallback_bottom"}
        assert res.fallback_count() == 1      # 주변을 잰 건 fire 하나뿐


class TestParseRegionByClass:
    """노드 파라미터 문자열 -> 매핑. 오타를 **시작할 때** 터뜨립니다."""

    def test_기본_형식(self):
        assert parse_region_by_class("fire:below,person:bottom") == {
            "fire": "below", "person": "bottom"}

    def test_공백을_허용한다(self):
        assert parse_region_by_class(" fire : below , person : bottom ") == {
            "fire": "below", "person": "bottom"}

    def test_빈_문자열은_빈_매핑(self):
        assert parse_region_by_class("") == {}
        assert parse_region_by_class("   ") == {}

    def test_모르는_영역은_거부한다(self):
        """오타를 들고 가면 첫 프레임 콜백에서 터져 '검출이 사라진' 것처럼 보입니다."""
        with pytest.raises(ValueError):
            parse_region_by_class("fire:bellow")

    def test_콜론이_없으면_거부한다(self):
        with pytest.raises(ValueError):
            parse_region_by_class("fire")

    def test_한쪽이_비면_거부한다(self):
        with pytest.raises(ValueError):
            parse_region_by_class("fire:")
        with pytest.raises(ValueError):
            parse_region_by_class(":below")


class TestBandOffset:
    """★ `below` 띠를 아래로 밀어 **불 아래 물체**를 겨냥합니다 (2026-08-26 실기).

    박스 바로 아래(band_offset=0)는 아직 화염 언저리라 배경(벽)이 잡혔습니다.
    실측: 화염 박스 19px, 실제 화염 1.5~2cm, 촛대 받침(종이컵)은 아래 약 9cm.
    9cm / 1.75cm ≈ 5.1배 — 눈대중 "5~6배"와 일치합니다.

    ★ 여기 장면은 **실제 촛불 크기로** 만듭니다. 기본 dummy_scene의 80px 박스로
      시험하면 3.5배가 화면 밖으로 나가서 아무것도 검증하지 못합니다.
    """

    FLAME_M, CUP_OFFSET_M, CUP_H_M = 0.0175, 0.09, 0.05
    WALL_GAP = 0.6

    def _scene(self, dist=0.5, cup_offset_m=None, place_cup=True):
        """거리 `dist`에 촛불, 그 아래 `cup_offset_m`에 컵이 있는 장면.

        박스 크기를 **거리에 맞춰** 계산합니다 — 그래야 배수가 거리와 무관하다는
        성질을 실제로 검증할 수 있습니다.
        """
        cup_offset_m = self.CUP_OFFSET_M if cup_offset_m is None else cup_offset_m
        probe = dummy_scene(distance_m=dist)
        fy = probe.k_color[4]
        h_px = fy * self.FLAME_M / dist              # 화염 박스 세로
        wall = dist + self.WALL_GAP

        sc = dummy_scene(box_size=(h_px, h_px), box_center_xy=(320.0, 140.0),
                         distance_m=dist, background_m=wall)
        bd = project_box(sc.box_color, sc.k_color, sc.k_depth)
        x1, y1, x2, y2 = bd
        h = y2 - y1
        # 컵을 "화염 아래 cup_offset_m" 위치에 픽셀로 환산해 놓습니다
        cup_mid = y2 + fy * cup_offset_m / dist
        cup_half = fy * self.CUP_H_M / dist / 2.0
        targets = ([((x1, cup_mid - cup_half, x2, cup_mid + cup_half), dist)]
                   if place_cup else [])
        depth = synthetic_depth(*sc.depth_size, background_m=wall,
                                targets=targets, hole_boxes=[bd])
        return sc, depth, dist, wall, h

    def _params(self, band_offset, band_ratio=3.0):
        return SamplingParams(region="bottom", region_by_class={"fire": "below"},
                              band_offset=band_offset, band_ratio=band_ratio)

    def _measure(self, params, **kw):
        sc, depth, dist, wall, _h = self._scene(**kw)
        res = convert_frame([(sc.box_color, "fire", 0.9)], depth,
                            sc.k_color, sc.k_depth, _tf(), params)
        got = res.detections[0].distance_m if res.detections else None
        return got, dist, wall

    # -------------------------------------------------------------- 기하

    def test_offset이_0이면_컵에_못_닿는다(self):
        """박스 바로 아래 얇은 띠는 컵까지 내려가지 않습니다 — 실기 증상."""
        got, dist, wall = self._measure(self._params(0.0, band_ratio=0.15))
        assert got == pytest.approx(wall, abs=0.1), \
            f"컵({dist})이 아니라 벽({wall})이 나와야 합니다: {got}"

    def test_offset을_주면_컵을_잡는다(self):
        """이 변경의 목적. 같은 장면·같은 영역인데 위치만 옮겨 값이 바뀝니다."""
        got, dist, _wall = self._measure(self._params(3.5))
        assert got == pytest.approx(dist, abs=0.05)

    def test_너무_많이_내리면_컵을_지나친다(self):
        """값이 클수록 좋은 게 아닙니다. 컵 아래는 다시 배경입니다."""
        got, _dist, wall = self._measure(self._params(12.0))
        assert got == pytest.approx(wall, abs=0.1)

    def test_권장_구간이_9cm를_가운데_둔다(self):
        """band_offset 3.5 + band_ratio 3.0 = 화염 아래 3.5~6.5배.

        화염 1.75cm 기준 6.1~11.4cm — 9cm가 구간 한가운데입니다.
        """
        p = self._params(3.5)
        lo = p.band_offset * self.FLAME_M
        hi = (p.band_offset + p.band_ratio) * self.FLAME_M
        assert lo < self.CUP_OFFSET_M < hi
        assert lo == pytest.approx(0.061, abs=0.002)
        assert hi == pytest.approx(0.114, abs=0.002)

    def test_촛불이_타들어가도_구간_안에_남는다(self):
        """★ 촛불이 짧아지면 화염-컵 거리가 줄어듭니다. 띠를 두껍게 잡은 이유."""
        for offset_cm in (6.5, 9.0, 11.0):
            got, dist, _wall = self._measure(
                self._params(3.5), cup_offset_m=offset_cm / 100.0)
            assert got == pytest.approx(dist, abs=0.05), \
                f"화염 아래 {offset_cm}cm 에서 놓쳤습니다"

    # ------------------------------------------- ★ 거리와 무관하다는 성질

    def test_배수는_거리와_무관하다(self):
        """★ 이 방식을 고른 이유입니다.

        9cm의 픽셀 크기도, 박스 높이도 똑같이 1/Z 이라 **비율이 상수**입니다.
        그래서 거리를 몰라도 되고 f_y도 필요 없습니다. 박스 크기가 거리에 따라
        달라지는 장면들에서 **같은 band_offset**이 계속 컵을 맞혀야 합니다.
        """
        for dist in (0.4, 0.5, 0.8, 1.2):
            got, d, _wall = self._measure(self._params(3.5), dist=dist)
            assert got == pytest.approx(d, abs=0.05), \
                f"거리 {dist}m 에서 빗나갔습니다 (얻은 값 {got})"

    # ----------------------------------------------------- method 자동 전환

    def test_offset이_있으면_median_없으면_max(self):
        """★ 띠가 무엇 위에 놓였는지가 통계를 정합니다.

        offset=0 이면 띠가 바닥이라 가장 먼 값이 접지점 -> max.
        offset>0 이면 컵이 배경보다 반드시 가까우므로 앞쪽으로 당김 -> p25.
        """
        assert method_for("below", 0.0) == "max"
        assert method_for("below", 3.5) == "p25"
        assert method_for("bottom", 3.5) == "median"   # 다른 영역은 무관
        assert method_for("ring", 3.5) == "median"

    def test_클래스_매핑이_전환된_method를_쓴다(self):
        assert self._params(3.5).region_for("fire") == ("below", "p25")
        assert self._params(0.0).region_for("fire") == ("below", "max")

    def test_max였다면_컵_뒤_배경에_끌린다(self):
        """자동 전환이 없었다면 무슨 일이 났는지 — 대조군으로 남깁니다."""
        p = SamplingParams(region="below", method="max",
                           band_offset=3.5, band_ratio=3.0)
        got, dist, _wall = self._measure(p)
        assert got > dist + 0.05, "max는 컵이 아니라 컵 뒤 배경 쪽을 고릅니다"

    # ------------------------------------------------------------ 호환성

    def test_기본값은_기존_접지점_동작이다(self):
        """라이브러리 기본은 0 — 기존 below 사용처가 안 깨집니다."""
        assert SamplingParams().band_offset == 0.0

    def test_median이었다면_없는_거리를_만들어낸다(self):
        """★★ 통계를 p25로 정한 이유 — 회귀 방지용으로 못 박습니다.

        띠가 컵과 배경에 절반씩 걸치면 `np.median`은 짝수 개일 때 가운데 두 값을
        **평균**냅니다. 컵 0.5m / 배경 1.1m 에서 0.8m가 나옵니다 — 장면에
        존재하지 않는 표면입니다. 촛불이 타들어가면 이 상태를 반드시 지나갑니다.
        """
        p_med = SamplingParams(region="below", method="median",
                               band_offset=3.5, band_ratio=3.0)
        got, dist, wall = self._measure(p_med, cup_offset_m=0.065)
        assert got not in (None,)
        assert abs(got - dist) > 0.1 and abs(got - wall) > 0.1, \
            f"median이 컵({dist})도 배경({wall})도 아닌 값을 냅니다: {got}"

        p25, _d, _w = self._measure(self._params(3.5), cup_offset_m=0.065)
        assert p25 == pytest.approx(dist, abs=0.05), "p25는 컵을 유지합니다"
