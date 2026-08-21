import math

import pytest

from fire_vla_core.ros.mock_slam_node import (
    build_room,
    circle_pose,
    quaternion_from_yaw,
    reveal_disc,
    world_to_cell,
)

WALL, FREE, UNKNOWN = 100, 0, -1


class TestBuildRoom:
    def test_outer_border_is_wall(self):
        width, height = 20, 12
        grid = build_room(width, height)
        assert all(grid[x] == WALL for x in range(width))
        assert all(grid[(height - 1) * width + x] == WALL for x in range(width))
        assert all(grid[y * width] == WALL for y in range(height))
        assert all(grid[y * width + width - 1] == WALL for y in range(height))

    def test_interior_has_both_free_and_wall(self):
        grid = build_room(60, 40)
        assert FREE in grid and WALL in grid
        assert UNKNOWN not in grid          # 정답 지도에 미지는 없습니다

    def test_grid_length_matches_dimensions(self):
        assert len(build_room(37, 21)) == 37 * 21


class TestRevealDisc:
    def test_nothing_is_known_before_revealing(self):
        truth = build_room(20, 20)
        known = [UNKNOWN] * 400
        assert known.count(UNKNOWN) == 400

    def test_reveal_copies_truth_within_radius(self):
        width = height = 20
        truth = build_room(width, height)
        known = [UNKNOWN] * (width * height)
        revealed = reveal_disc(truth, known, width, height, 10, 10, 3)
        assert revealed > 0
        assert known[10 * width + 10] == truth[10 * width + 10]
        assert known[0] == UNKNOWN          # 반경 밖은 그대로 미지

    def test_second_pass_at_the_same_spot_reveals_nothing(self):
        """0이 계속 나오면 탐색이 멈춘 것 — 화면에서는 지도가 정지합니다."""
        width = height = 20
        truth = build_room(width, height)
        known = [UNKNOWN] * (width * height)
        reveal_disc(truth, known, width, height, 10, 10, 3)
        assert reveal_disc(truth, known, width, height, 10, 10, 3) == 0

    def test_reveal_near_the_edge_does_not_wrap_or_crash(self):
        width = height = 20
        truth = build_room(width, height)
        known = [UNKNOWN] * (width * height)
        reveal_disc(truth, known, width, height, 0, 0, 5)
        # 왼쪽 끝에서 밝혔는데 오른쪽 끝이 밝혀지면 인덱스가 감긴 것입니다.
        assert known[19 * width + 19] == UNKNOWN


class TestCirclePose:
    def test_starts_on_the_positive_x_axis(self):
        x, y, _ = circle_pose(0.0, 2.0, 40.0)
        assert x == pytest.approx(2.0) and y == pytest.approx(0.0, abs=1e-9)

    def test_quarter_turn_reaches_positive_y(self):
        x, y, _ = circle_pose(10.0, 2.0, 40.0)
        assert x == pytest.approx(0.0, abs=1e-9) and y == pytest.approx(2.0)

    def test_yaw_is_tangent_to_the_path(self):
        _, _, yaw = circle_pose(0.0, 2.0, 40.0)
        assert yaw == pytest.approx(math.pi / 2.0)

    def test_zero_period_does_not_divide_by_zero(self):
        assert circle_pose(5.0, 2.0, 0.0) == (2.0, 0.0, math.pi / 2.0)


class TestWorldToCell:
    def test_origin_maps_to_cell_zero(self):
        assert world_to_cell(-2.0, -1.5, -2.0, -1.5, 0.05) == (0, 0)

    def test_center_of_a_centered_map_is_the_middle_cell(self):
        assert world_to_cell(0.0, 0.0, -2.0, -1.5, 0.05) == (40, 30)

    def test_negative_side_floors_instead_of_truncating(self):
        """int()는 -0.5를 0으로 만듭니다 — 원점 왼쪽 한 줄이 통째로 어긋납니다."""
        assert world_to_cell(-2.03, -1.5, -2.0, -1.5, 0.05) == (-1, 0)


def test_quaternion_round_trips_through_yaw():
    from fire_vla_core.ros.occupancy_png import yaw_from_quaternion

    class _Q:
        def __init__(self, x, y, z, w):
            self.x, self.y, self.z, self.w = x, y, z, w

    for yaw in (0.0, 0.5, -1.2, math.pi / 2):
        assert yaw_from_quaternion(_Q(*quaternion_from_yaw(yaw))) == pytest.approx(yaw)