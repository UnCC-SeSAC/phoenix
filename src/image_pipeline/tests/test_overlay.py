#!/usr/bin/env python3
"""오버레이 그리기 테스트 — rclpy 없이 돕니다.

여기서 잠그는 것은 "화면에 보이는 게 실제 검출과 같은가"입니다. 이게 틀리면
예외가 아니라 **잘못된 화면**이 나오고, 그 화면을 보고 모델을 의심하게 됩니다.

  - size_x/size_y 를 우하단 좌표로 오해 -> 박스가 두 배로 커짐
  - 화면 밖 박스에서 라벨 좌표가 음수    -> confidence 숫자가 사라짐
  - 축소를 그리기 **전에** 하면          -> 박스가 화면 밖으로 나감
"""

from __future__ import annotations

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from image_pipeline.overlay import (  # noqa: E402
    class_color,
    draw_detections,
    draw_hud,
    hud_text,
    scale_frame,
)


# --------------------------------------------------------------------- 더미들
# vision_msgs 3.x / 4.x 의 필드 모양이 다릅니다 (detection_msgs.py 문서 참조).
# 오버레이가 **둘 다** 그릴 수 있어야 로봇과 개발 PC에서 같은 화면이 나옵니다.

class _Position:
    def __init__(self, x, y):
        self.x, self.y = float(x), float(y)


class _Center4x:
    def __init__(self, x, y):
        self.position = _Position(x, y)


class _Center3x:
    def __init__(self, x, y):
        self.x, self.y = float(x), float(y)


class _BBox:
    def __init__(self, cx, cy, w, h, *, legacy=False):
        self.center = _Center3x(cx, cy) if legacy else _Center4x(cx, cy)
        self.size_x, self.size_y = float(w), float(h)


class _Hypothesis4x:
    def __init__(self, class_id, score):
        self.class_id, self.score = str(class_id), float(score)


class _Result4x:
    def __init__(self, class_id, score):
        self.hypothesis = _Hypothesis4x(class_id, score)


class _Result3x:
    def __init__(self, class_id, score):
        self.id, self.score = int(class_id), float(score)


class _Detection:
    def __init__(self, cx, cy, w, h, results, *, legacy=False):
        self.bbox = _BBox(cx, cy, w, h, legacy=legacy)
        self.results = results


def _fire(cx, cy, w, h, score=0.9, legacy=False):
    return _Detection(cx, cy, w, h, [_Result4x("fire", score)], legacy=legacy)


def _frame(w=640, h=480):
    return np.zeros((h, w, 3), dtype=np.uint8)


def _bbox_of_drawn(frame):
    """그려진(0이 아닌) 픽셀의 경계 (x1, y1, x2, y2)."""
    ys, xs = np.nonzero(frame.any(axis=2))
    assert xs.size, "아무것도 그려지지 않았습니다"
    return int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())


# ------------------------------------------------------------------ 좌표 해석

def test_size_는_크기지_우하단_좌표가_아니다():
    """중심 (320,240) 크기 100x80 -> (270,200)-(370,280).

    우하단 좌표로 잘못 읽으면 (320,240)-(420,320)이 되어 박스가 엉뚱한 데
    그려집니다. 라벨까지 같이 밀려서 화면만 보면 알아채기 어렵습니다.
    """
    frame = _frame()
    assert draw_detections(frame, [_fire(320, 240, 100, 80)]) == 1

    # 라벨은 박스 위쪽에 붙으므로 아래/좌우 경계로만 판정합니다.
    x1, _, x2, y2 = _bbox_of_drawn(frame)
    assert abs(x1 - 270) <= 2
    assert abs(x2 - 370) <= 4          # 라벨 배경이 조금 더 넓을 수 있음
    assert abs(y2 - 280) <= 2


def test_vision_msgs_3x_필드도_그린다():
    """`bbox.center.x` / `result.id` 형태(구버전)에서도 같은 자리에 그립니다."""
    modern = _frame()
    draw_detections(modern, [_fire(320, 240, 100, 80)])

    legacy = _frame()
    det = _Detection(320, 240, 100, 80, [_Result3x(0, 0.9)], legacy=True)
    assert draw_detections(legacy, [det]) == 1

    # 라벨 문자열이 "fire"와 "0"으로 달라 픽셀이 완전히 같진 않지만,
    # 박스의 아래/좌 경계는 같아야 합니다.
    assert _bbox_of_drawn(modern)[0] == _bbox_of_drawn(legacy)[0]
    assert _bbox_of_drawn(modern)[3] == _bbox_of_drawn(legacy)[3]


# ------------------------------------------------------------ 화면 밖 / 이상값

@pytest.mark.parametrize("cx, cy", [(-200.0, 240.0), (900.0, 240.0),
                                    (320.0, -300.0), (320.0, 800.0)])
def test_완전히_화면_밖이면_그리지_않는다(cx, cy):
    frame = _frame()
    assert draw_detections(frame, [_fire(cx, cy, 40, 40)]) == 0
    assert not frame.any()


def test_걸친_박스는_잘라서_그린다():
    """왼쪽 위로 반쯤 나간 박스. 예외 없이, 프레임 안에만 그려져야 합니다."""
    frame = _frame()
    assert draw_detections(frame, [_fire(10, 10, 200, 200)]) == 1
    x1, y1, x2, y2 = _bbox_of_drawn(frame)
    assert x1 >= 0 and y1 >= 0
    assert x2 < frame.shape[1] and y2 < frame.shape[0]


def test_결과가_비면_건너뛴다():
    frame = _frame()
    assert draw_detections(frame, [_Detection(320, 240, 40, 40, [])]) == 0
    assert draw_detections(frame, []) == 0
    assert draw_detections(frame, None) == 0


# ---------------------------------------------------------------- min_score

def test_min_score_필터와_반환값():
    frame = _frame()
    dets = [_fire(160, 240, 40, 40, score=0.9),
            _fire(320, 240, 40, 40, score=0.4),
            _fire(480, 240, 40, 40, score=0.1)]
    assert draw_detections(_frame(), dets) == 3
    assert draw_detections(frame, dets, min_score=0.35) == 2


def test_최고_신뢰도_가설을_고른다():
    """`results`가 여러 개면 점수가 가장 높은 것이 라벨이 됩니다."""
    frame = _frame()
    det = _Detection(320, 240, 60, 60,
                     [_Result4x("person", 0.2), _Result4x("fire", 0.8)])
    assert draw_detections(frame, [det], min_score=0.5) == 1


# -------------------------------------------------------------------- 색/HUD

def test_클래스_색은_대소문자를_접는다():
    """학습 라벨이 'Fire'여도 불 색으로 그립니다."""
    assert class_color("Fire") == class_color("fire")
    assert class_color(" person ") == class_color("person")
    assert class_color("smoke") != class_color("fire")   # 모르는 클래스


def test_화면_위쪽_박스의_라벨은_HUD에_가리지_않는다():
    """상단에 걸친 검출의 confidence 가 HUD 띠 뒤로 숨으면 안 됩니다.

    하필 **가장 확인하고 싶은 숫자**가 사라지는 자리입니다.
    """
    from image_pipeline.overlay import HUD_BAND_PX

    frame = _frame()
    draw_hud(frame, n_drawn=1, n_total=1, fps=8.0)
    hud_only = frame.copy()

    assert draw_detections(frame, [_fire(60, 20, 120, 120)]) == 1
    # HUD 띠 아래쪽에 라벨(글자)이 실제로 새로 그려졌는지 본다.
    below = np.abs(frame[HUD_BAND_PX:HUD_BAND_PX + 30].astype(int)
                   - hud_only[HUD_BAND_PX:HUD_BAND_PX + 30].astype(int))
    assert below.sum() > 0


def test_hud_는_검출_0개여도_남는다():
    """빈 화면일 때 '검출이 0'인지 '영상이 안 옴'인지 갈라 주는 한 줄입니다."""
    assert "det 0/0" in hud_text(0, 0, 12.3)
    assert "yolo 12.3fps" in hud_text(0, 0, 12.3)
    frame = _frame()
    draw_hud(frame, n_drawn=0, n_total=0, fps=0.0)
    assert frame.any()


def test_hud_는_그린_수와_전체_수를_구분한다():
    """min_score로 걸러낸 것이 있으면 그 사실이 화면에 보여야 합니다."""
    assert "det 1/3" in hud_text(1, 3, 9.0)
    assert "conf>=0.35" in hud_text(1, 3, 9.0, note="conf>=0.35")


# --------------------------------------------------------------------- 축소

def test_scale_frame():
    frame = _frame(640, 480)
    assert scale_frame(frame, 0) is frame            # 0이면 원본 그대로
    assert scale_frame(frame, 1280) is frame         # 확대는 안 함
    small = scale_frame(frame, 320)
    assert small.shape[:2] == (240, 320)             # 가로세로비 유지


def test_먼저_그리고_나중에_축소한다():
    """축소를 먼저 하면 원본 좌표 박스가 화면 밖으로 나갑니다.

    노드가 그 순서를 지키는지 여기서 대신 못 박습니다 — 그린 뒤 축소하면
    박스가 살아남고, 축소한 뒤 그리면 아무것도 안 남습니다.
    """
    det = _fire(600, 440, 60, 60)

    drawn_then_scaled = _frame()
    assert draw_detections(drawn_then_scaled, [det]) == 1
    assert scale_frame(drawn_then_scaled, 320).any()

    scaled_then_drawn = scale_frame(_frame(), 320)
    assert draw_detections(scaled_then_drawn, [det]) == 0
