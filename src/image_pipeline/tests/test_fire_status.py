#!/usr/bin/env python3
"""
`fire_status.py` 판정 경계 잠금 — rclpy 없이 돕니다.

이 판정은 진압 성공/실패를 가르는데 **예외가 안 납니다.** 틀려도 로그에
"꺼짐"이라고만 찍히고 넘어가므로, 경계를 테스트로 못 박아 둡니다.
"""

import pytest

from image_pipeline.fire_status import (
    REASON_NO_DATA,
    REASON_OK,
    REASON_STALE,
    DetectionHistory,
    describe,
    labels_from_detections,
)


def fill(hist, start, count, *, fire, step=1.0 / 15.0, score=0.9):
    """프레임 `count`장을 `step` 간격으로 넣고 마지막 시각을 돌려줍니다."""
    t = start
    for _ in range(count):
        hist.add(t, [("fire", score)] if fire else [])
        t += step
    return t - step


class TestNoData:
    """★ "데이터 없음"을 "불 없음"으로 해석하면 카메라가 빠져도 진압 성공이 됩니다."""

    def test_빈_히스토리는_판정을_거부한다(self):
        hist = DetectionHistory(window_sec=10.0)
        v = hist.verdict(now=100.0, observation_sec=3.0)
        assert v.is_extinguished is False
        assert v.reason == REASON_NO_DATA
        assert v.samples == 0
        assert v.confidence == 0.0

    def test_관찰_구간_밖의_기록만_있으면_판정을_거부한다(self):
        hist = DetectionHistory(window_sec=10.0)
        fill(hist, 100.0, 15, fire=False)
        # 기록은 창 안에 있지만 관찰 구간(0.5초) 밖
        v = hist.verdict(now=105.0, observation_sec=0.5)
        assert v.reason == REASON_NO_DATA
        assert v.is_extinguished is False

    def test_기록이_전부_낡으면_stale로_판정을_거부한다(self):
        hist = DetectionHistory(window_sec=10.0)
        fill(hist, 100.0, 15, fire=False)
        v = hist.verdict(now=105.0, observation_sec=3.0, stale_after_sec=2.0)
        assert v.reason == REASON_STALE
        assert v.is_extinguished is False

    def test_stale_후에도_기본값_0이면_끈다(self):
        hist = DetectionHistory(window_sec=10.0)
        fill(hist, 100.0, 15, fire=False)
        v = hist.verdict(now=105.0, observation_sec=10.0)  # stale_after_sec 기본 0
        assert v.reason == REASON_OK


class TestVerdict:
    def test_불이_계속_보이면_안꺼짐(self):
        hist = DetectionHistory(window_sec=10.0)
        end = fill(hist, 100.0, 45, fire=True)
        v = hist.verdict(end, observation_sec=3.0)
        assert v.is_extinguished is False
        assert v.fire_ratio == pytest.approx(1.0)
        assert v.samples == 45

    def test_불이_안_보이면_꺼짐(self):
        hist = DetectionHistory(window_sec=10.0)
        end = fill(hist, 100.0, 45, fire=False)
        v = hist.verdict(end, observation_sec=3.0)
        assert v.is_extinguished is True
        assert v.fire_ratio == pytest.approx(0.0)

    def test_빈_검출_프레임이_불_없음의_증거다(self):
        """빈 배열을 안 넣으면 표본이 안 쌓여 영원히 판정 불가가 됩니다."""
        hist = DetectionHistory(window_sec=10.0)
        for i in range(10):
            hist.add(100.0 + i * 0.1, [])       # 검출 0개
        v = hist.verdict(100.9, observation_sec=3.0)
        assert v.samples == 10
        assert v.is_extinguished is True

    def test_임계값_경계_미만이면_꺼짐(self):
        hist = DetectionHistory(window_sec=10.0)
        t = 100.0
        for i in range(10):                     # 10장 중 2장만 화재 -> 0.2
            hist.add(t, [("fire", 0.9)] if i < 2 else [])
            t += 0.1
        v = hist.verdict(t, observation_sec=3.0, ratio_threshold=0.3)
        assert v.fire_ratio == pytest.approx(0.2)
        assert v.is_extinguished is True

    def test_임계값과_같으면_안꺼짐(self):
        """`ratio < threshold` 이지 `<=` 가 아닙니다. 경계를 잠급니다."""
        hist = DetectionHistory(window_sec=10.0)
        t = 100.0
        for i in range(10):                     # 3/10 = 0.3
            hist.add(t, [("fire", 0.9)] if i < 3 else [])
            t += 0.1
        v = hist.verdict(t, observation_sec=3.0, ratio_threshold=0.3)
        assert v.fire_ratio == pytest.approx(0.3)
        assert v.is_extinguished is False

    def test_다른_클래스는_화재로_안_센다(self):
        hist = DetectionHistory(window_sec=10.0, fire_class="fire")
        t = 100.0
        for _ in range(10):
            hist.add(t, [("person", 0.95)])
            t += 0.1
        v = hist.verdict(t, observation_sec=3.0)
        assert v.fire_ratio == pytest.approx(0.0)
        assert v.is_extinguished is True

    def test_관찰_구간이_0이면_창_전체를_쓴다(self):
        hist = DetectionHistory(window_sec=10.0)
        end = fill(hist, 100.0, 30, fire=True)
        v = hist.verdict(end, observation_sec=0.0)
        assert v.samples == 30


class TestConfidence:
    """★ confidence가 판정과 반대로 움직이면 로그가 자기모순처럼 보입니다."""

    def test_꺼짐일수록_확신도가_높다(self):
        hist = DetectionHistory(window_sec=10.0)
        end = fill(hist, 100.0, 45, fire=False)
        v = hist.verdict(end, observation_sec=3.0)
        assert v.is_extinguished is True
        assert v.confidence == pytest.approx(1.0)

    def test_안꺼짐일수록_확신도가_높다(self):
        hist = DetectionHistory(window_sec=10.0)
        end = fill(hist, 100.0, 45, fire=True)
        v = hist.verdict(end, observation_sec=3.0)
        assert v.is_extinguished is False
        assert v.confidence == pytest.approx(1.0)

    def test_확신도는_점수가_아니라_비율에서_온다(self):
        """점수가 낮아도 꾸준히 보이면 '안꺼짐'을 강하게 확신해야 합니다."""
        hist = DetectionHistory(window_sec=10.0)
        end = fill(hist, 100.0, 45, fire=True, score=0.26)
        v = hist.verdict(end, observation_sec=3.0)
        assert v.confidence == pytest.approx(1.0)
        assert v.mean_score == pytest.approx(0.26)

    def test_확신도는_항상_0과_1_사이다(self):
        for n_fire in range(0, 11):
            hist = DetectionHistory(window_sec=10.0)
            t = 100.0
            for i in range(10):
                hist.add(t, [("fire", 0.9)] if i < n_fire else [])
                t += 0.1
            v = hist.verdict(t, observation_sec=3.0)
            assert 0.0 <= v.confidence <= 1.0


class TestMinScore:
    def test_임계_미만_검출은_무시한다(self):
        hist = DetectionHistory(window_sec=10.0, min_score=0.5)
        end = fill(hist, 100.0, 20, fire=True, score=0.3)
        v = hist.verdict(end, observation_sec=3.0)
        assert v.fire_ratio == pytest.approx(0.0)
        assert v.is_extinguished is True

    def test_임계_이상은_센다(self):
        hist = DetectionHistory(window_sec=10.0, min_score=0.5)
        end = fill(hist, 100.0, 20, fire=True, score=0.5)
        v = hist.verdict(end, observation_sec=3.0)
        assert v.fire_ratio == pytest.approx(1.0)

    def test_한_프레임의_최고_점수를_남긴다(self):
        hist = DetectionHistory(window_sec=10.0)
        hist.add(100.0, [("fire", 0.3), ("fire", 0.8), ("person", 0.99)])
        v = hist.verdict(100.0, observation_sec=3.0)
        assert v.mean_score == pytest.approx(0.8)


class TestWindow:
    def test_창을_넘긴_기록은_버린다(self):
        hist = DetectionHistory(window_sec=1.0)
        fill(hist, 100.0, 30, fire=True, step=0.1)   # 3초치
        assert len(hist) <= 11                       # 1초 + 경계 1장

    def test_관찰_구간이_창보다_길면_truncated로_알린다(self):
        """★ 조용히 자르면 3초를 요청하고 1초 결과를 받게 됩니다."""
        hist = DetectionHistory(window_sec=1.0)
        end = fill(hist, 100.0, 15, fire=False, step=0.05)
        v = hist.verdict(end, observation_sec=3.0)
        assert v.truncated is True

    def test_구간이_창_안이면_truncated가_아니다(self):
        hist = DetectionHistory(window_sec=10.0)
        end = fill(hist, 100.0, 45, fire=False)
        v = hist.verdict(end, observation_sec=3.0)
        assert v.truncated is False

    def test_창은_양수여야_한다(self):
        with pytest.raises(ValueError):
            DetectionHistory(window_sec=0.0)


class TestNearMiss:
    """★ 라벨 대소문자가 어긋나면 불이 타는 중에도 '꺼짐'으로 응답합니다."""

    def test_대소문자만_다른_라벨을_잡아낸다(self):
        hist = DetectionHistory(window_sec=10.0, fire_class="fire")
        hist.add(100.0, [("Fire", 0.9)])
        assert hist.near_miss_labels() == ["Fire"]

    def test_정상이면_비어_있다(self):
        hist = DetectionHistory(window_sec=10.0, fire_class="fire")
        hist.add(100.0, [("fire", 0.9), ("person", 0.5)])
        assert hist.near_miss_labels() == []

    def test_대소문자_불일치는_화재로_안_센다(self):
        """잡아내기만 하고 자동 교정은 하지 않습니다 — 조용히 고치면 원인이 숨습니다."""
        hist = DetectionHistory(window_sec=10.0, fire_class="fire")
        end = fill(hist, 100.0, 10, fire=False)
        for i in range(10):
            hist.add(end + 0.1 * (i + 1), [("Fire", 0.9)])
        v = hist.verdict(end + 1.0, observation_sec=3.0)
        assert v.fire_ratio == pytest.approx(0.0)


class TestLabelsFromDetections:
    def test_vision_msgs_모양을_헬퍼로_흡수한다(self):
        class Res:
            def __init__(self, cid, score):
                self.cid, self.s = cid, score

        class Det:
            def __init__(self, results):
                self.results = results

        dets = [Det([Res("fire", 0.9), Res("person", 0.4)]), Det([])]
        out = labels_from_detections(dets, lambda r: (r.cid, r.s))
        assert out == [("fire", 0.9), ("person", 0.4)]

    def test_results가_없어도_죽지_않는다(self):
        class Bare:
            pass

        assert labels_from_detections([Bare()], lambda r: (r.cid, r.s)) == []


class TestDescribe:
    def test_판정_불가는_원인을_말한다(self):
        hist = DetectionHistory(window_sec=10.0)
        v = hist.verdict(100.0, observation_sec=3.0)
        assert "판정 불가" in describe(v, 3.0)

    def test_정상_판정은_비율과_확신도를_찍는다(self):
        hist = DetectionHistory(window_sec=10.0)
        end = fill(hist, 100.0, 45, fire=True)
        line = describe(hist.verdict(end, observation_sec=3.0), 3.0)
        assert "안꺼짐" in line and "확신도" in line

    def test_절삭은_로그에_표시된다(self):
        hist = DetectionHistory(window_sec=1.0)
        end = fill(hist, 100.0, 15, fire=False, step=0.05)
        assert "잘림" in describe(hist.verdict(end, observation_sec=3.0), 3.0)
