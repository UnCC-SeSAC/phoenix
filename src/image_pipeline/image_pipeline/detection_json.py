#!/usr/bin/env python3
"""
메인에 보낼 JSON 페이로드 만들기 — **ROS도 numpy도 import하지 않습니다.**

2026-08-10 계약 변경으로 태스크②의 출력이 `vision_msgs/Detection3DArray`(base_link
3D 좌표)에서 **픽셀 좌표 + 거리 스칼라의 JSON**으로 바뀌었습니다. 역투영과 TF는
메인이 수행합니다 (HANDOVER 7-3 재작성분).

    {"stamp_sec": ..., "stamp_nanosec": ..., "frame_size": [640, 480],
     "detections": [{"class_name": "fire", "score": 0.87,
                     "x": 320, "y": 240, "depth": 2.0, "depth_status": "ok"}, ...]}

계산이 줄어든 만큼 **조용히 틀릴 구석은 오히려 늘었습니다.** 이 모듈이 막는 것:

  1. 거리 불명을 `0.0`으로 발행    -> 메인이 로봇 발밑을 화재 지점으로 계산
  2. `float('nan')`을 그대로 직렬화 -> `NaN`은 **JSON이 아닙니다**(RFC 8259).
                                      엄격한 파서는 거부하고, 느슨한 파서는
                                      `depth`가 NaN인 채로 통과시킵니다
  3. numpy 스칼라를 그대로 넣기     -> `json.dumps`가 `TypeError`로 죽습니다.
                                      `depth.py`가 내는 값이 전부 그렇습니다
  4. 폴백 거리를 `ok`로 발행        -> `ring` 폴백은 +0.6m 틀린 값을 유효
                                      100%·사유 `ok`로 내놓습니다 (HANDOVER 8)

1·4는 예외가 안 나고, 2·3은 콜백 안에서 터져 **검출이 통째로 사라집니다.**
그래서 노드가 아니라 여기서 처리하고 테스트로 잠급니다.

`x`, `y`는 **원본 `rgb0` 좌표계**(`frame_size` 기준)입니다. 축소본
(`/image_enhanced`) 좌표를 그대로 넣으면 메인이 드라이버의 `camera_info`로
역투영할 때 배율만큼 틀립니다 — 에러 없이 거리만 어긋납니다. 노드에서
`scale_box`로 되돌린 뒤 넘기세요.
"""

from __future__ import annotations

import json
import math
from typing import Any, Iterable, Mapping, Optional, Sequence

# 거리의 출처. 메인은 무시해도 되지만, 무시하면 폴백 거리를 정확한 거리와
# 똑같이 신뢰하게 됩니다. 편향 실측치는 HANDOVER 8 "5-1 폴백 후보 정량 비교".
STATUS_OK = "ok"                       # 박스 중앙에서 직접 측정. 편향 없음
STATUS_UNKNOWN = "unknown"             # 거리 불명. depth 는 반드시 null
STATUS_BOTTOM = "fallback_bottom"      # 박스 하단 띠. 화염이 박스를 안 채울 때만 유효
STATUS_BELOW = "fallback_below"        # 박스 아래 접지점. 가깝게 편향 (max 기준 -0.05m)
STATUS_RING = "fallback_ring"          # 박스 주변. 멀게 편향 (+0.6m)

# `DistanceSample.region` -> 상태 문자열.
_REGION_STATUS = {
    "center": STATUS_OK,
    "bottom": STATUS_BOTTOM,
    "below": STATUS_BELOW,
    "ring": STATUS_RING,
}

# 거리는 mm 자리까지. 스테레오 오차가 cm 단위라 그 아래는 의미가 없고,
# 반올림하지 않으면 부동소수 잔재(2.0000000000000004)가 그대로 전송됩니다.
_DEPTH_DIGITS = 3
_SCORE_DIGITS = 3


def depth_status(sample) -> str:
    """`DistanceSample` -> 상태 문자열.

    ★ `reason == "ok"`는 **"맞다"가 아니라 "픽셀이 충분했다"**는 뜻입니다.
    폴백 영역(`below`/`ring`)은 대상이 아니라 주변을 재고도 `ok`를 냅니다.
    그래서 사유가 아니라 **`region`으로** 상태를 정합니다.
    """
    dist = getattr(sample, "distance", None)
    if dist is None or not math.isfinite(float(dist)):
        return STATUS_UNKNOWN
    if getattr(sample, "reason", None) != "ok":
        # 거리가 있는데 사유가 ok가 아닌 조합은 depth.py 계약상 없습니다.
        # 그래도 값이 왔다면 믿을 근거가 없으므로 불명으로 내립니다.
        return STATUS_UNKNOWN
    region = getattr(sample, "region", "center")
    if region not in _REGION_STATUS:
        raise ValueError(
            f"모르는 region={region!r} — 새 폴백을 추가했다면 이 표와 "
            "메인과의 계약을 함께 갱신하세요"
        )
    return _REGION_STATUS[region]


def detection_entry(class_name: str, score: float, x: float, y: float,
                    sample=None, *, depth: Optional[float] = None,
                    status: Optional[str] = None) -> dict[str, Any]:
    """검출 하나 -> 페이로드 항목.

    `sample`(`DistanceSample`)을 주면 `depth`와 `depth_status`를 거기서 뽑습니다.
    `sample` 없이 `depth`/`status`를 직접 주는 건 테스트·더미 노드용입니다.

    키 순서는 메인이 준 예시와 같게 유지합니다 — `ros2 topic echo`로 볼 때
    눈으로 대조할 수 있어야 합니다.
    """
    if sample is not None:
        status = depth_status(sample)
        depth = getattr(sample, "distance", None)
    elif status is None:
        raise ValueError("sample 없이 부르려면 status를 명시하세요")

    depth_value = _finite_or_none(depth)
    if status == STATUS_UNKNOWN:
        # ★ 계약의 핵심. 불명이면 값이 있어도 버립니다. 0.0이나 추정값이
        #   섞이면 메인이 그걸 실제 거리로 역투영합니다 (HANDOVER 7-3).
        depth_value = None
    elif depth_value is None:
        # 거꾸로, 값이 없는데 상태가 ok인 조합도 나가면 안 됩니다.
        status = STATUS_UNKNOWN

    return {
        "class_name": str(class_name),
        "score": _round(score, _SCORE_DIGITS, default=0.0),
        # 픽셀은 정수로. 640x480에서 1px은 3.2m 기준 5mm라 소수점이 무의미하고,
        # 메인 예시도 정수입니다.
        "x": _as_pixel(x),
        "y": _as_pixel(y),
        "depth": _round(depth_value, _DEPTH_DIGITS),
        "depth_status": status,
    }


def build_payload(stamp_sec: int, stamp_nanosec: int,
                  frame_size: Sequence[int],
                  detections: Iterable[Mapping[str, Any]]) -> dict[str, Any]:
    """봉투 + 검출 배열.

    ★ `stamp`는 **원본 이미지의 stamp**입니다. 발행 시각을 넣으면 메인이
    엉뚱한 시점의 TF로 변환합니다 — 회전 0.5rad/s·지연 100ms·3.2m에서 16cm가
    조용히 틀립니다 (HANDOVER 4-8).

    `frame_size`는 `x`,`y`가 어느 해상도 기준인지를 알려주는 값입니다.
    이게 없으면 메인은 축소본 좌표와 원본 좌표를 구분할 방법이 없습니다.
    """
    width, height = (int(v) for v in frame_size)
    if width <= 0 or height <= 0:
        raise ValueError(f"frame_size가 이상합니다: {frame_size!r}")
    sec, nanosec = int(stamp_sec), int(stamp_nanosec)
    if not 0 <= nanosec < 1_000_000_000:
        # sec에 실수로 float 초를 넣고 nanosec를 안 맞춘 경우가 여기서 걸립니다.
        raise ValueError(f"stamp_nanosec는 0~999999999: {stamp_nanosec!r}")

    return {
        "stamp_sec": sec,
        "stamp_nanosec": nanosec,
        "frame_size": [width, height],
        "detections": list(detections),
    }


def to_json(payload: Mapping[str, Any]) -> str:
    """페이로드 -> JSON 문자열. `std_msgs/String.data`에 그대로 넣으면 됩니다.

    `allow_nan=False`가 핵심입니다. 기본값(True)은 `NaN`/`Infinity`를 그대로
    쓰는데 **그건 JSON이 아닙니다.** 파이썬끼리는 왕복이 되니 로컬 테스트에서는
    안 걸리고, 메인이 다른 언어/엄격한 파서를 쓰는 순간 프레임이 통째로
    버려집니다. 여기서 막히면 값을 고치세요 — 이 인자를 풀지 말고.
    """
    return json.dumps(payload, ensure_ascii=False, allow_nan=False,
                      separators=(",", ":"))


def stamp_fields(header) -> tuple[int, int]:
    """ROS 헤더 -> `(sec, nanosec)`. 덕 타이핑이라 rclpy 없이 테스트됩니다."""
    stamp = header.stamp
    return (int(stamp.sec), int(stamp.nanosec))


# ==================================================== 하트비트 (2026-08-11 신설)
#
# 검출이 있을 때만 `/fire/detections`를 내면, 받는 쪽은 "불이 없다"와 "노드가
# 죽었다"를 구분할 수 없습니다. 그래서 상태를 따로 주기 발행합니다.
#
# ★★ 단, **살아있다고만 말하는 하트비트는 거짓말을 합니다.**
#    이 노드의 대표 실패는 크래시가 아니라 **동기화가 영원히 안 맞는 것**입니다
#    (YOLO가 stamp를 now()로 덮는 경우, HANDOVER 5-2d). 그때도 타이머는 정상
#    동작하므로 단순 펄스는 계속 "정상"을 주장합니다. 침묵은 의심이라도 사지만
#    거짓 정상은 안 삽니다.
#
#    그래서 하트비트에 **마지막으로 처리한 원본 프레임의 stamp와 경과 시간**을
#    싣습니다. 받는 쪽이 `state`를 안 믿어도 `age_sec`로 직접 판정할 수 있습니다.

STATE_OK = "ok"                       # 최근 프레임을 처리했음
STATE_STALLED = "stalled"             # 입력은 오는데 처리가 멈춤 ★ 동기화 의심
STATE_WAITING_INFO = "waiting_camera_info"   # camera_info 대기 중 (K 없이는 못 냄)
STATE_NO_INPUT = "no_input"           # 입력 자체가 없음 (토픽 이름·QoS 의심)


def stamp_age_sec(now: tuple[int, int], last: Optional[tuple[int, int]]) -> Optional[float]:
    """`(sec, nanosec)` 두 개의 차이를 초로.

    계약 밖의 **국소 계산**이라 float으로 합쳐도 됩니다. 발행하는 stamp를
    float으로 합치는 것과 헷갈리지 마세요 — 그쪽은 ns 정밀도가 날아갑니다.
    """
    if last is None:
        return None
    return (now[0] - last[0]) + (now[1] - last[1]) / 1e9


def heartbeat_state(age_sec: Optional[float], *, camera_info_ready: bool,
                    inputs_seen: bool, stall_after_sec: float) -> str:
    """상태 판정 — 순수 함수라 rclpy 없이 테스트됩니다.

    순서가 중요합니다. `camera_info` 대기를 먼저 보지 않으면, 시작 직후
    정상적인 대기 상태가 `stalled`로 보여 **매번 거짓 경보**가 납니다.
    """
    if not camera_info_ready:
        return STATE_WAITING_INFO
    if not inputs_seen:
        return STATE_NO_INPUT
    if age_sec is None or age_sec > stall_after_sec:
        # 입력은 오는데 한 프레임도 못 냈거나, 낸 지 오래됐습니다.
        return STATE_STALLED
    return STATE_OK


def build_heartbeat(stamp_sec: int, stamp_nanosec: int, state: str, *,
                    last_frame: Optional[tuple[int, int]] = None,
                    age_sec: Optional[float] = None,
                    counters: Optional[Mapping[str, int]] = None) -> dict[str, Any]:
    """상태 페이로드.

    ★ 여기 `stamp`는 **발행 시각**입니다 (`/fire/detections`와 반대).
    "지금 몇 시에 이 말을 하는가"가 하트비트의 요점이라서입니다. 원본 프레임
    시각은 `last_frame_*`에 따로 실립니다 — 둘을 섞으면 age를 못 구합니다.
    """
    payload: dict[str, Any] = {
        "stamp_sec": int(stamp_sec),
        "stamp_nanosec": int(stamp_nanosec),
        "state": str(state),
        "last_frame_sec": None if last_frame is None else int(last_frame[0]),
        "last_frame_nanosec": None if last_frame is None else int(last_frame[1]),
        # 받는 쪽이 state를 안 믿어도 이걸로 직접 판정할 수 있어야 합니다.
        "age_sec": _round(age_sec, 3),
    }
    if counters:
        payload["counters"] = {str(k): int(v) for k, v in counters.items()}
    return payload


def _finite_or_none(value) -> Optional[float]:
    """None·NaN·inf·numpy 스칼라를 전부 흡수해 순수 float 또는 None으로."""
    if value is None:
        return None
    number = float(value)          # numpy 스칼라도 여기서 파이썬 float이 됩니다
    return number if math.isfinite(number) else None


def _round(value, digits: int, default: Optional[float] = None) -> Optional[float]:
    number = _finite_or_none(value)
    if number is None:
        return default
    return round(number, digits)


def _as_pixel(value) -> int:
    number = _finite_or_none(value)
    if number is None:
        # 픽셀 위치가 없는 검출은 존재할 수 없습니다. 여기까지 왔다면 상위
        # 코드의 버그이므로 조용히 0(=화면 좌상단)을 넣지 않고 터뜨립니다.
        raise ValueError(f"픽셀 좌표가 유한하지 않습니다: {value!r}")
    return int(round(number))
