#!/usr/bin/env bash
set -u

echo "=== ROS distro ==="
echo "${ROS_DISTRO:-not sourced}"

echo
echo "=== Required topics ==="
ros2 topic list | grep -E \
  '^/(map|scan_raw|odom|cmd_vel|tf|tf_static|global_costmap|local_costmap|hazard_map|exploration)' \
  || true

echo
echo "=== Rates ==="
timeout 4 ros2 topic hz /scan_raw || true
timeout 4 ros2 topic hz /odom || true
timeout 4 ros2 topic hz /map || true

echo
echo "=== Nav2 actions ==="
ros2 action list | grep -E \
  'navigate_to_pose|compute_path_to_pose' \
  || true

echo
echo "=== TF checks ==="
timeout 3 ros2 run tf2_ros tf2_echo odom base_footprint || true
timeout 3 ros2 run tf2_ros tf2_echo map odom || true
