from __future__ import annotations

import math
import struct
import zlib

_SIGNATURE = b"\x89PNG\r\n\x1a\n"
_UNKNOWN, _FREE, _OCCUPIED = 0, 1, 2
_PALETTE = bytes((0x00, 0x00, 0x00, 0xD8, 0xE2, 0xEC, 0x24, 0x36, 0x49))
_ALPHA = bytes((0, 255, 255))
_OCCUPIED_THRESHOLD = 65


def downsample_step(width, height, max_pixels):
    if width <= 0 or height <= 0 or max_pixels <= 0:
        return 1
    step = 1
    while math.ceil(width / step) * math.ceil(height / step) > max_pixels:
        step += 1
    return step


def render_occupancy_png(
    data, width, height, step=1, occupied_threshold=_OCCUPIED_THRESHOLD
):
    width, height, step = int(width), int(height), max(1, int(step))
    if width <= 0 or height <= 0:
        return _encode(bytes((0, _UNKNOWN)), 1, 1)
    output_width = math.ceil(width / step)
    output_height = math.ceil(height / step)
    raw = bytearray()
    for output_y in range(output_height):
        raw.append(0)
        y_end = height - output_y * step
        y_start = max(0, y_end - step)
        for output_x in range(output_width):
            x_start = output_x * step
            x_end = min(width, x_start + step)
            raw.append(_pool(
                data, width, y_start, y_end, x_start, x_end, occupied_threshold
            ))
    return _encode(bytes(raw), output_width, output_height)


def grid_metadata(message):
    info = message.info
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
        "frame_id": str(message.header.frame_id),
        "stamp_sec": int(message.header.stamp.sec),
        "stamp_nanosec": int(message.header.stamp.nanosec),
    }


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _pool(data, width, y_start, y_end, x_start, x_end, threshold):
    best = _UNKNOWN
    for y in range(y_start, y_end):
        offset = y * width
        for x in range(x_start, x_end):
            value = data[offset + x]
            if value < 0:
                continue
            if value >= threshold:
                return _OCCUPIED
            best = _FREE
    return best


def _encode(raw, width, height):
    return b"".join((
        _SIGNATURE,
        _chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 3, 0, 0, 0)),
        _chunk(b"PLTE", _PALETTE),
        _chunk(b"tRNS", _ALPHA),
        _chunk(b"IDAT", zlib.compress(raw, 9)),
        _chunk(b"IEND", b""),
    ))


def _chunk(tag, payload):
    return b"".join((
        struct.pack(">I", len(payload)), tag, payload,
        struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF),
    ))
