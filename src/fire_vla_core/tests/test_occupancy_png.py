import struct
import zlib

import pytest

from fire_vla_core.ros.occupancy_png import (
    downsample_step, grid_metadata, render_occupancy_png,
)


def _chunks(png):
    position, found = 8, {}
    while position < len(png):
        length = struct.unpack(">I", png[position:position + 4])[0]
        tag = png[position + 4:position + 8]
        found.setdefault(tag, b"")
        found[tag] += png[position + 8:position + 8 + length]
        position += 12 + length
    return found


def _size(png):
    return struct.unpack(">II", _chunks(png)[b"IHDR"][:8])


def _pixels(png):
    width, height = _size(png)
    raw = zlib.decompress(_chunks(png)[b"IDAT"])
    stride = width + 1
    return [list(raw[y * stride + 1:(y + 1) * stride]) for y in range(height)]


class _Value:
    def __init__(self, **values):
        self.__dict__.update(values)


def _grid(yaw_quaternion=None):
    orientation = yaw_quaternion or _Value(x=0.0, y=0.0, z=0.0, w=1.0)
    return _Value(
        header=_Value(frame_id="map", stamp=_Value(sec=12, nanosec=340_000_000)),
        info=_Value(
            width=4, height=3, resolution=0.05,
            origin=_Value(
                position=_Value(x=-1.0, y=-2.0, z=0.0), orientation=orientation,
            ),
        ),
    )


def test_png_distinguishes_unknown_free_and_occupied_and_flips_rows():
    png = render_occupancy_png([100, 0, -1, -1], 2, 2)
    assert png.startswith(b"\x89PNG\r\n\x1a\n")
    assert _pixels(png) == [[0, 0], [2, 1]]


def test_downsampling_keeps_obstacles_and_partial_blocks():
    png = render_occupancy_png([0, 0, 0, 100], 2, 2, step=2)
    assert _size(png) == (1, 1)
    assert _pixels(png) == [[2]]
    assert _size(render_occupancy_png([0] * 9, 3, 3, step=2)) == (2, 2)
    assert downsample_step(2000, 2000, 250_000) > 1


def test_empty_grid_returns_valid_transparent_pixel():
    png = render_occupancy_png([], 0, 0)
    assert _size(png) == (1, 1) and _pixels(png) == [[0]]


def test_metadata_preserves_origin_resolution_stamp_and_yaw():
    half = 2 ** -0.5
    metadata = grid_metadata(_grid(_Value(x=0.0, y=0.0, z=half, w=half)))
    assert metadata["resolution"] == pytest.approx(0.05)
    assert metadata["origin"] == pytest.approx({
        "x": -1.0, "y": -2.0, "yaw": 1.5707963,
    })
    assert metadata["frame_id"] == "map"
    assert metadata["stamp_sec"] == 12
