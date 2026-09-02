import math

from uncc_example.mission_navigation_safety import (
    approach_candidates,
    forward_corridor_is_clear,
    front_scan_is_clear,
    occupancy_disk_is_clear,
)


def test_approach_candidate_prefers_robot_facing_side():
    candidates = approach_candidates(0.0, 0.0, 1.0, 0.0, 0.5)

    assert math.isclose(candidates[0][0], 0.5)
    assert math.isclose(candidates[0][1], 0.0, abs_tol=1e-12)
    assert all(math.isclose(math.hypot(x - 1.0, y), 0.5) for x, y in candidates)


def test_occupancy_disk_rejects_lethal_and_unknown_cells():
    clear_grid = [0] * 100
    assert occupancy_disk_is_clear(clear_grid, 10, 10, 0.1, 0.0, 0.0, 0.5, 0.5, 0.1)

    lethal_grid = clear_grid.copy()
    lethal_grid[5 * 10 + 5] = 100
    assert not occupancy_disk_is_clear(lethal_grid, 10, 10, 0.1, 0.0, 0.0, 0.5, 0.5, 0.1)

    unknown_grid = clear_grid.copy()
    unknown_grid[5 * 10 + 5] = -1
    assert not occupancy_disk_is_clear(unknown_grid, 10, 10, 0.1, 0.0, 0.0, 0.5, 0.5, 0.1)


def test_forward_corridor_checks_heading_and_width():
    grid = [0] * 400
    assert forward_corridor_is_clear(
        grid, 20, 20, 0.1, -1.0, -1.0, 0.0, 0.0, 0.0, 0.2, 0.12, 0.095, 0.05
    )

    blocked = grid.copy()
    blocked[10 * 20 + 12] = 100
    assert not forward_corridor_is_clear(
        blocked, 20, 20, 0.1, -1.0, -1.0, 0.0, 0.0, 0.0, 0.2, 0.12, 0.095, 0.05
    )


def test_front_scan_requires_valid_clear_rays():
    ranges = [float("inf")] * 181
    ranges[90] = 0.6
    assert front_scan_is_clear(
        ranges, -math.pi / 2.0, math.pi / 180.0, 0.05, 12.0, math.radians(25.0), 0.42
    )

    ranges[90] = 0.3
    assert not front_scan_is_clear(
        ranges, -math.pi / 2.0, math.pi / 180.0, 0.05, 12.0, math.radians(25.0), 0.42
    )
