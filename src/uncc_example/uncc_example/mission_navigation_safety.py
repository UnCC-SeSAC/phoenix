import math


def approach_candidates(robot_x, robot_y, target_x, target_y, distance):
    """Return approach points around a target, preferring the robot-facing side."""
    base_angle = math.atan2(robot_y - target_y, robot_x - target_x)
    offsets = (0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 180.0)

    return [
        (
            target_x + distance * math.cos(base_angle + math.radians(offset)),
            target_y + distance * math.sin(base_angle + math.radians(offset)),
        )
        for offset in offsets
    ]


def occupancy_disk_is_clear(
    data,
    width,
    height,
    resolution,
    origin_x,
    origin_y,
    center_x,
    center_y,
    radius,
    occupied_threshold=90,
    unknown_is_blocked=True,
):
    """Check every grid cell intersecting a circular footprint clearance area."""
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return False

    center_mx = int(math.floor((center_x - origin_x) / resolution))
    center_my = int(math.floor((center_y - origin_y) / resolution))
    cell_radius = int(math.ceil(max(0.0, radius) / resolution))

    for my in range(center_my - cell_radius, center_my + cell_radius + 1):
        for mx in range(center_mx - cell_radius, center_mx + cell_radius + 1):
            if mx < 0 or my < 0 or mx >= width or my >= height:
                return False

            cell_x = origin_x + (mx + 0.5) * resolution
            cell_y = origin_y + (my + 0.5) * resolution
            if math.hypot(cell_x - center_x, cell_y - center_y) > radius + resolution:
                continue

            value = data[my * width + mx]
            if value < 0:
                if unknown_is_blocked:
                    return False
            elif value >= occupied_threshold:
                return False

    return True


def forward_corridor_is_clear(
    data,
    width,
    height,
    resolution,
    origin_x,
    origin_y,
    robot_x,
    robot_y,
    robot_yaw,
    forward_distance,
    front_extent,
    half_width,
    safety_margin,
    occupied_threshold=90,
    unknown_is_blocked=True,
):
    """Check the swept rectangular corridor of a short forward recovery."""
    step = max(resolution * 0.5, 0.01)
    travel_end = max(0.0, forward_distance + front_extent + safety_margin)
    lateral_end = max(0.0, half_width + safety_margin)
    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)

    forward = 0.0
    while forward <= travel_end + 1e-9:
        lateral = -lateral_end
        while lateral <= lateral_end + 1e-9:
            world_x = robot_x + forward * cos_yaw - lateral * sin_yaw
            world_y = robot_y + forward * sin_yaw + lateral * cos_yaw
            mx = int(math.floor((world_x - origin_x) / resolution))
            my = int(math.floor((world_y - origin_y) / resolution))

            if mx < 0 or my < 0 or mx >= width or my >= height:
                return False

            value = data[my * width + mx]
            if value < 0:
                if unknown_is_blocked:
                    return False
            elif value >= occupied_threshold:
                return False

            lateral += step
        forward += step

    return True


def front_scan_is_clear(
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    half_angle,
    required_distance,
):
    """Require at least one valid front ray and no obstacle inside the limit."""
    valid_front_ray = False

    for index, measured_range in enumerate(ranges):
        angle = angle_min + index * angle_increment
        wrapped_angle = math.atan2(math.sin(angle), math.cos(angle))
        if abs(wrapped_angle) > half_angle:
            continue
        if math.isinf(measured_range) and measured_range > 0.0:
            valid_front_ray = True
            continue
        if not math.isfinite(measured_range):
            continue
        if measured_range < range_min or measured_range > range_max:
            continue

        valid_front_ray = True
        if measured_range < required_distance:
            return False

    return valid_front_ray
