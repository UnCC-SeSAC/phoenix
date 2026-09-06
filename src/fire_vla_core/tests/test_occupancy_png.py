import struct
import zlib

import pytest

from fire_vla_core.ros.occupancy_png import (
    downsample_step,
    grid_metadata,
    render_occupancy_png,
)


def _chunks(png):
    pos, found = 8, {}
    while pos < len(png):
        (length,) = struct.unpack(">I", png[pos:pos + 4])
        tag = png[pos + 4:pos + 8]
        found.setdefault(tag, b"")
        found[tag] += png[pos + 8:pos + 8 + length]
        pos += 12 + length
    return found


def _size(png):
    return struct.unpack(">II", _chunks(png)[b"IHDR"][:8])


def _pixels(png):
    """테스트용 최소 디코더 — filter가 전부 0인 palette PNG 전용."""
    width, height = _size(png)
    raw = zlib.decompress(_chunks(png)[b"IDAT"])
    stride = width + 1
    return [list(raw[y * stride + 1:(y + 1) * stride]) for y in range(height)]


UNKNOWN, FREE, OCCUPIED = 0, 1, 2


class TestRender:
    def test_output_is_a_valid_png(self):
        png = render_occupancy_png([0] * 4, 2, 2)
        assert png.startswith(b"\x89PNG\r\n\x1a\n")
        assert _size(png) == (2, 2)
        assert b"PLTE" in _chunks(png) and b"tRNS" in _chunks(png)

    def test_empty_grid_is_a_transparent_one_pixel(self):
        """0x0 PNG는 규격 위반이라 '지도 없음'이 '인코더 고장'처럼 보입니다."""
        png = render_occupancy_png([], 0, 0)
        assert _size(png) == (1, 1)
        assert _pixels(png) == [[UNKNOWN]]

    def test_rows_are_flipped_for_png_top_down(self):
        """★ grid의 row 0은 아래쪽입니다. 뒤집지 않으면 지도와 마커가 어긋납니다."""
        data = [100, 100,   # row 0 = 지도의 아래쪽
                -1, -1]     # row 1 = 위쪽
        png = render_occupancy_png(data, 2, 2)
        assert _pixels(png) == [[UNKNOWN, UNKNOWN],   # PNG 첫 줄 = 위쪽
                               [OCCUPIED, OCCUPIED]]  # PNG 마지막 줄 = 아래쪽

    def test_values_map_to_three_palette_indices(self):
        png = render_occupancy_png([-1, 0, 100, 64], 4, 1)
        assert _pixels(png) == [[UNKNOWN, FREE, OCCUPIED, FREE]]

    def test_downsample_keeps_thin_walls(self):
        """최근접 표본으로 줄이면 한 칸 벽이 사라집니다 — max-pool이어야 합니다."""
        data = [0, 0, 0, 100]          # 2x2에서 한 칸만 점유
        png = render_occupancy_png(data, 2, 2, step=2)
        assert _size(png) == (1, 1)
        assert _pixels(png) == [[OCCUPIED]]

    def test_downsample_rounds_up_partial_blocks(self):
        png = render_occupancy_png([0] * 9, 3, 3, step=2)
        assert _size(png) == (2, 2)


class TestDownsampleStep:
    def test_small_map_needs_no_downsampling(self):
        assert downsample_step(100, 100, 250_000) == 1

    def test_large_map_is_reduced_under_the_cap(self):
        step = downsample_step(2000, 2000, 250_000)
        assert step > 1
        assert (2000 // step) * (2000 // step) <= 250_000

    def test_degenerate_input_returns_one(self):
        assert downsample_step(0, 0, 250_000) == 1
        assert downsample_step(100, 100, 0) == 1


class _V:
    def __init__(self, **kw): self.__dict__.update(kw)


def _grid(width=4, height=3, resolution=0.05, ox=-1.0, oy=-2.0):
    return _V(
        header=_V(frame_id="map", stamp=_V(sec=12, nanosec=340_000_000)),
        info=_V(width=width, height=height, resolution=resolution,
                origin=_V(position=_V(x=ox, y=oy, z=0.0),
                          orientation=_V(x=0.0, y=0.0, z=0.0, w=1.0))),
    )


class TestGridMetadata:
    def test_carries_world_to_pixel_terms(self):
        meta = grid_metadata(_grid())
        assert meta["width"] == 4 and meta["height"] == 3
        assert meta["resolution"] == pytest.approx(0.05)
        assert meta["origin"]["x"] == pytest.approx(-1.0)
        assert meta["origin"]["y"] == pytest.approx(-2.0)
        assert meta["origin"]["yaw"] == pytest.approx(0.0)
        assert meta["frame_id"] == "map"

    def test_yaw_comes_from_the_quaternion(self):
        grid = _grid()
        half = 2 ** -0.5                       # 90도
        grid.info.origin.orientation = _V(x=0.0, y=0.0, z=half, w=half)
        assert grid_metadata(grid)["origin"]["yaw"] == pytest.approx(1.5707963, abs=1e-6)