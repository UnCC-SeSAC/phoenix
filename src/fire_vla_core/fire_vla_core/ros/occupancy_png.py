"""OccupancyGrid를 UI가 그릴 수 있는 PNG로 — **표준 라이브러리만 씁니다.**

fire_vla_core는 전송 계층입니다. 여기에 Pillow나 OpenCV를 들이면 UI 노드가
이미지 처리 책임까지 지게 되고, rclpy 없이 도는 현재 테스트 구조도 깨집니다.
occupancy grid는 미지/빈공간/점유 3색이라 palette PNG로 충분합니다.

★ 좌표계 함정 2개
  1. OccupancyGrid의 row 0은 **origin(좌하단)** 이고 y가 커질수록 row가 증가합니다.
     PNG는 위에서 아래로 그립니다. 뒤집지 않으면 지도가 상하 반전되고,
     그 위에 얹는 로봇/화점 마커만 제자리라 **지도와 마커가 서로 어긋납니다.**
  2. 축소는 **max-pool**입니다. 최근접 표본으로 줄이면 한 칸짜리 벽이 통째로
     사라집니다. 관제 화면에서 벽이 없어지는 것은 벽이 두꺼워지는 것보다 훨씬
     나쁩니다.
"""

from __future__ import annotations

import math
import struct
import zlib

_SIGNATURE = b"\x89PNG\r\n\x1a\n"

# palette index
_UNKNOWN, _FREE, _OCCUPIED = 0, 1, 2
_PALETTE = bytes((0x00, 0x00, 0x00,    # unknown (tRNS로 투명 처리)
                  0xD8, 0xE2, 0xEC,    # free      — 밝은 청회색
                  0x24, 0x36, 0x49))   # occupied  — 진한 남색
_ALPHA = bytes((0, 255, 255))

# nav2의 occupied_thresh 기본값. slam_toolbox는 보통 -1/0/100만 냅니다.
_OCCUPIED_THRESHOLD = 65


def downsample_step(width, height, max_pixels):
    """출력 픽셀 수를 max_pixels 이하로 만드는 최소 정수 배수."""
    if width <= 0 or height <= 0 or max_pixels <= 0:
        return 1
    step = 1
    while math.ceil(width / step) * math.ceil(height / step) > max_pixels:
        step += 1
    return step


def render_occupancy_png(data, width, height, step=1,
                         occupied_threshold=_OCCUPIED_THRESHOLD):
    """`OccupancyGrid.data` -> palette PNG 바이트.

    빈 grid는 1x1 투명 PNG를 돌려줍니다. 0x0 PNG는 규격 위반이라 브라우저가
    깨진 이미지 아이콘을 띄우는데, "지도가 아직 없음"과 "인코더가 고장남"이
    화면에서 구분되지 않게 됩니다.
    """
    width, height, step = int(width), int(height), max(1, int(step))
    if width <= 0 or height <= 0:
        return _encode(bytes((0, _UNKNOWN)), 1, 1)

    out_w = math.ceil(width / step)
    out_h = math.ceil(height / step)

    raw = bytearray()
    for out_y in range(out_h):
        raw.append(0)                      # filter type 0 (None)
        y_end = height - out_y * step      # ★ 상하 반전
        y_start = max(0, y_end - step)
        for out_x in range(out_w):
            x_start = out_x * step
            x_end = min(width, x_start + step)
            raw.append(_pool(data, width, y_start, y_end,
                             x_start, x_end, occupied_threshold))
    return _encode(bytes(raw), out_w, out_h)


def grid_metadata(msg):
    """UI가 world(x,y) <-> 픽셀 변환에 쓸 값. 이게 틀리면 마커가 엉뚱한 방에 찍힙니다."""
    info = msg.info
    position = info.origin.position
    return {
        "width": int(info.width),
        "height": int(info.height),
        "resolution": float(info.resolution),
        "origin": {
            "x": float(position.x),
            "y": float(position.y),
            "yaw": yaw_from_quaternion(info.origin.orientation),
        },
        "frame_id": str(msg.header.frame_id),
        "stamp_sec": int(msg.header.stamp.sec),
        "stamp_nanosec": int(msg.header.stamp.nanosec),
    }


def yaw_from_quaternion(q):
    """쿼터니언 -> 2D yaw.

    지도 원점의 회전과 TF에서 읽은 로봇 heading이 **같은 식**을 써야 합니다.
    두 군데에 각자 적어 두면 한쪽만 고쳤을 때 지도와 로봇 방향이 조용히
    갈라지고, 화면에서는 "로봇이 벽을 향해 가는 것처럼" 보입니다.
    """
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _pool(data, width, y0, y1, x0, x1, threshold):
    """점유 > 빈공간 > 미지 우선순위의 max-pool."""
    best = _UNKNOWN
    for y in range(y0, y1):
        base = y * width
        for x in range(x0, x1):
            value = data[base + x]
            if value < 0:
                continue
            if value >= threshold:
                return _OCCUPIED          # 최상위라 더 볼 필요 없음
            best = _FREE
    return best


def _encode(raw, width, height):
    return b"".join((
        _SIGNATURE,
        # bit_depth=8, color_type=3(palette), compression=0, filter=0, interlace=0
        _chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 3, 0, 0, 0)),
        _chunk(b"PLTE", _PALETTE),
        _chunk(b"tRNS", _ALPHA),
        _chunk(b"IDAT", zlib.compress(raw, 9)),
        _chunk(b"IEND", b""),
    ))


def _chunk(tag, payload):
    return b"".join((
        struct.pack(">I", len(payload)),
        tag,
        payload,
        struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF),
    ))