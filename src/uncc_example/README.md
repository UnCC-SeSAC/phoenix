# uncc_example — Frontier Exploration

Target:

- Raspberry Pi 5
- ROS 2 Humble
- Hiwonder MentorPi-style workspace
- LD19 LiDAR
- `slam_toolbox`
- Nav2
- Python (`ament_python`)

This package is designed around the topic/frame structure found in the
current Hiwonder workspace:

- LiDAR: `/scan_raw`
- raw / filtered odometry stack ultimately provides `/odom`
- TF: `odom -> base_footprint`
- SLAM Toolbox: `map -> odom`
- Nav2 global costmap: `/global_costmap/costmap`
- Nav2 local costmap: `/local_costmap/costmap`
- robot velocity command: `/cmd_vel`

## Architecture

```text
LD19 /scan_raw
      |
      +--------------------+
      |                    |
      v                    v
slam_toolbox          Nav2 costmaps
      |                |        |
      | /map           |        |
      v                v        v
FrontierDetector   global     local
      |             costmap   costmap
      |                |        |
      +--------+-------+        |
               |                |
          ExplorerNode <--------+
               ^
               |
          /hazard_map
               ^
               |
         HazardMapNode
               ^
               |
         /hazard_points

ExplorerNode
   |
   +--> Nav2 ComputePathToPose
   |
   +--> score candidates
   |
   +--> Nav2 NavigateToPose
             |
             v
         /cmd_vel
```

## Important design rule

Frontiers are detected from the raw SLAM `/map`, not from
`global_costmap`.

- `/map`: free/unknown frontier detection
- `global_costmap`: reachability/path/environment cost
- `local_costmap`: immediate safety
- `/hazard_map`: semantic risk
- Nav2: path planning and motion

## Install

If this folder replaces the empty package skeleton you already created:

```bash
cd ~/ros2_ws/src
rm -rf uncc_example
# copy this uncc_example folder here
```

Install ROS dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-tf2-tools
```

Then:

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  -r -y

colcon build \
  --symlink-install \
  --packages-select uncc_example

source install/setup.bash
```

The existing Hiwonder workspace must already contain and build:

```text
controller
peripherals
navigation
slam
```

The Hiwonder controller also uses its own `MACHINE_TYPE` environment
setting. Keep the same machine configuration that currently makes your
robot drive and publish `/odom`.

## Recommended first test: do NOT enable autonomous exploration yet

Run the existing robot stack and confirm:

```bash
ros2 topic echo /odom --once
ros2 topic echo /scan_raw --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

Then run SLAM:

```bash
ros2 launch uncc_example slam_mapping.launch.py
```

Confirm:

```bash
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map odom
```

## Run Nav2 online with SLAM

This launch deliberately starts navigation-only Nav2 and does NOT start
AMCL/map_server.

```bash
ros2 launch uncc_example nav2_online.launch.py
```

Confirm:

```bash
ros2 action list | grep -E \
  'compute_path_to_pose|navigate_to_pose'

ros2 topic list | grep costmap
```

Expected topics include:

```text
/global_costmap/costmap
/local_costmap/costmap
```

## Run frontier visualization / exploration

```bash
ros2 launch uncc_example exploration.launch.py
```

Useful topics:

```text
/exploration/frontiers
/exploration/best_frontier
/exploration/state
/hazard_map
```

In RViz on a desktop PC:

- Fixed Frame: `map`
- Map: `/map`
- Map: `/global_costmap/costmap`
- Map: `/local_costmap/costmap`
- Map: `/hazard_map`
- MarkerArray: `/exploration/frontiers`
- Marker: `/exploration/best_frontier`
- LaserScan: `/scan_raw`
- TF

## Full launch

Once every component has been tested separately:

```bash
ros2 launch uncc_example full_exploration.launch.py
```

If controller/LiDAR are already running:

```bash
ros2 launch uncc_example full_exploration.launch.py \
  start_hardware:=false
```

If SLAM is already running too:

```bash
ros2 launch uncc_example full_exploration.launch.py \
  start_hardware:=false \
  start_slam:=false
```

Do not run two copies of controller, LD19, SLAM Toolbox, or Nav2.

## Hazard map test

Publish one temporary hazard point at map coordinate `(1.0, 0.0)`:

```bash
ros2 run uncc_example hazard_test --x 1.0 --y 0.0
```

Then:

```bash
ros2 topic echo /hazard_map --once
```

Later your thermal/YOLO/VLM node only needs to publish detected hazard
positions as:

```text
geometry_msgs/PoseArray
topic: /hazard_points
frame: map
```

The included `HazardMapNode` expands each point into a radial risk map.

## Exploration score

The current implementation uses:

```text
Score =
  + wI * InformationGain
  + wM * MissionValue
  - wP * PathCost
  - wH * HazardRisk
  - wG * GlobalCost
  - wN * Narrowness
  - wF * FailurePenalty
```

`MissionValue` is currently left at 0.0 as an extension point for fire,
person, victim, or mission semantics.

Tune weights in:

```text
config/exploration.yaml
```

## Raspberry Pi 5 notes

The package intentionally:

- evaluates only a small number of frontier candidates
- runs frontier cycles every few seconds
- uses Nav2 for planning instead of implementing another A*
- uses a 1 Hz semantic hazard map
- uses the existing Hiwonder costmaps

If CPU load is high, first reduce:

```yaml
max_candidates_to_plan: 5
exploration_period: 4.0
```

Do not reduce LiDAR or odometry rates before verifying TF timing.

## Diagnostics

From the source tree:

```bash
bash ~/ros2_ws/src/uncc_example/scripts/check_system.sh
```

## Diagrams

PlantUML sources:

```text
docs/frontier_exploration_class.puml
docs/frontier_exploration_sequence.puml
```

## VLA Brain integration

The integration overlay adds a Humble-side bridge:

```bash
ros2 launch uncc_example vla_navigation_bridge.launch.py
```

Topics:

```text
/vla/navigation_goal    std_msgs/String JSON input
/vla/navigation_result  std_msgs/String JSON output
/vla/navigation_cancel  std_msgs/String JSON input
/vla/robot_pose_json    std_msgs/String JSON output
```

The bridge reuses the package's asynchronous `Nav2Navigator` and calls
`/navigate_to_pose`. It does not replace `ExplorerNode`; only one component
should own robot navigation at a time. Do not run autonomous frontier
exploration and VLA-directed navigation simultaneously unless an explicit
arbitration policy is added.
