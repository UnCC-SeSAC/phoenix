"""Planar approach geometry; distances are relative to the front wheel edge."""
import math


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
