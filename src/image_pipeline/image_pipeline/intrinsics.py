#!/usr/bin/env python3
"""
카메라 내부 파라미터(K, P) 스케일링 — ROS 비의존 순수 함수.

노드 안에 두면 rclpy 없이는 테스트를 못 합니다. 그런데 여기가
**"에러 없이 거리가 틀리는"** 사고가 나는 자리라 검증이 가장 필요한 곳이라서,
계산만 떼어내 로컬에서 pytest로 검증할 수 있게 했습니다.

왜 스케일이 필요한가:
  전처리 노드가 1920x1080 -> 640x360으로 줄여서 발행하면, 그 이미지의
  픽셀 좌표는 더 이상 원본 K와 짝이 맞지 않습니다. 태스크②가 원본 K로
  역투영하면 거리가 약 3배 틀리는데 **에러는 나지 않습니다.**
"""

from __future__ import annotations


def scale_k(k, sx: float, sy: float) -> list[float]:
    """3x3 K를 담은 길이 9 배열을 스케일.

    K = [fx,  0, cx,
          0, fy, cy,
          0,  0,  1]

    fx, cx는 가로 배율 sx로, fy, cy는 세로 배율 sy로 곱합니다.
    (초점거리 단위가 '픽셀'이라 해상도가 바뀌면 함께 바뀝니다.)
    """
    if len(k) != 9:
        raise ValueError(f"K는 길이 9여야 합니다 (받은 길이: {len(k)})")
    out = [float(v) for v in k]
    out[0] *= sx   # fx
    out[2] *= sx   # cx
    out[4] *= sy   # fy
    out[5] *= sy   # cy
    return out


def scale_p(p, sx: float, sy: float) -> list[float]:
    """3x4 투영행렬 P를 담은 길이 12 배열을 스케일.

    P = [fx',  0, cx', Tx,
          0, fy', cy', Ty,
          0,   0,   1,  0]

    Tx, Ty는 스테레오 베이스라인 항(단안이면 0)이며 픽셀 단위라 함께 스케일합니다.
    """
    if len(p) != 12:
        raise ValueError(f"P는 길이 12여야 합니다 (받은 길이: {len(p)})")
    out = [float(v) for v in p]
    out[0] *= sx   # fx'
    out[2] *= sx   # cx'
    out[3] *= sx   # Tx
    out[5] *= sy   # fy'
    out[6] *= sy   # cy'
    out[7] *= sy   # Ty
    return out


def fit_size(src_w: int, src_h: int, target_w: int) -> tuple[int, int, float, float]:
    """가로를 target_w로 맞출 때의 (새 폭, 새 높이, sx, sy).

    target_w가 0이거나 원본보다 크면 원본을 그대로 둡니다(확대는 안 함).

    주의: 반올림 때문에 sx와 sy가 미세하게 다를 수 있습니다. 그래서 배율을
    "요청값"이 아니라 **실제 정수 크기에서 되계산**합니다. 이걸 안 하면
    cx가 0.5픽셀쯤 어긋납니다.
    """
    if src_w <= 0 or src_h <= 0:
        raise ValueError("원본 크기가 0 이하입니다")
    if not target_w or target_w >= src_w:
        return src_w, src_h, 1.0, 1.0

    new_w = int(target_w)
    new_h = max(1, int(round(src_h * new_w / src_w)))
    return new_w, new_h, new_w / src_w, new_h / src_h


def principal_point_sanity(k, width: int, height: int, tol: float = 0.15) -> bool:
    """cx ≈ width/2, cy ≈ height/2 인지 확인.

    로봇 수령 후 체크리스트의 "K값이 각각 해상도와 맞는가" 항목을 코드로 옮긴 것.
    False가 나오면 **컬러용 K를 뎁스에 쓰고 있을 가능성**이 큽니다.
    (1080p용 cx=960을 640 폭 이미지에 쓰면 화면 밖을 가리킵니다.)
    """
    cx, cy = float(k[2]), float(k[5])
    return (abs(cx - width / 2.0) <= width * tol
            and abs(cy - height / 2.0) <= height * tol)
