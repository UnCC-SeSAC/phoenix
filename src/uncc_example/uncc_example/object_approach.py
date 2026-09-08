"""Planar approach geometry; distances are relative to the front wheel edge."""
import math


def approach_candidates(robot_x, robot_y, target_x, target_y, wheel_offset,
                        clearance, heading=None):
    """Five candidates using a saved sight line or the current robot pose."""
    if heading is None:
        _, _, heading = approach_pose(robot_x, robot_y, target_x, target_y,
                                      wheel_offset, clearance)
    radius = wheel_offset + clearance
    return [(target_x - radius * math.cos(heading + math.radians(angle)),
             target_y - radius * math.sin(heading + math.radians(angle)),
             heading + math.radians(angle)) for angle in (0, 15, -15, 30, -30)]


def footprint_is_free(pose, footprint, costmap, lethal_cost=254, allow_unknown=False):
    """Check every grid cell intersecting a convex footprint (including interior).

    nav2_msgs/Costmap raw costs: 253 inscribed, 254 lethal, 255 unknown.
    The costmap has already expanded 253 for the robot inscribed radius, so
    applying the full footprint to it would count the robot size twice. Accept
    253 here and reject lethal/keepout (254); unknown (255) is rejected unless
    allow_unknown is enabled. SAT includes
    cell boundaries, conservatively rejecting contact with those rejected cells.
    """
    meta = costmap.metadata
    if meta.resolution <= 0 or len(footprint) < 3:
        return False
    if len(costmap.data) != meta.size_x * meta.size_y:
        return False
    q = meta.origin.orientation
    origin_yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    c, s = math.cos(pose[2]), math.sin(pose[2])
    oc, os = math.cos(origin_yaw), math.sin(origin_yaw)
    polygon = []
    for x, y in footprint:
        dx = pose[0] + c*x - s*y - meta.origin.position.x
        dy = pose[1] + s*x + c*y - meta.origin.position.y
        polygon.append(((oc*dx + os*dy)/meta.resolution,
                        (-os*dx + oc*dy)/meta.resolution))
    min_x, max_x = min(x for x, _ in polygon), max(x for x, _ in polygon)
    min_y, max_y = min(y for _, y in polygon), max(y for _, y in polygon)
    if min_x < 0 or min_y < 0 or max_x >= meta.size_x or max_y >= meta.size_y:
        return False
    axes = [(1, 0), (0, 1)]
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        axes.append((a[1]-b[1], b[0]-a[0]))
    projections = [(ax, ay, min(ax*x+ay*y for x, y in polygon),
                    max(ax*x+ay*y for x, y in polygon)) for ax, ay in axes]
    for row in range(max(0, math.floor(min_y)-1), math.floor(max_y)+1):
        for col in range(max(0, math.floor(min_x)-1), math.floor(max_x)+1):
            cost = costmap.data[row*meta.size_x+col]
            if cost == 255 and allow_unknown:
                continue
            if cost < lethal_cost:
                continue
            square = [(col, row), (col+1, row), (col+1, row+1), (col, row+1)]
            separated = any(
                max(ax*x+ay*y for x, y in square) < low or
                min(ax*x+ay*y for x, y in square) > high
                for ax, ay, low, high in projections)
            if not separated:
                return False
    return True


def approach_pose(robot_x, robot_y, target_x, target_y, wheel_offset, clearance):
    distance = math.hypot(target_x - robot_x, target_y - robot_y)
    if distance < 1e-6:
        raise ValueError('Robot and object positions coincide; approach direction is undefined')
    yaw = math.atan2(target_y - robot_y, target_x - robot_x)
    radius = wheel_offset + clearance
    return (target_x - radius * math.cos(yaw),
            target_y - radius * math.sin(yaw), yaw)


def arrival_error(robot_pose, target_xy, wheel_offset, clearance):
    x, y, yaw = robot_pose
    dx, dy = target_xy[0] - x, target_xy[1] - y
    heading = math.atan2(dy, dx) - yaw
    heading = math.atan2(math.sin(heading), math.cos(heading))
    # Forward distance from the wheel-front plane, valid with heading check.
    gap = dx * math.cos(yaw) + dy * math.sin(yaw) - wheel_offset
    return gap, gap - clearance, heading
