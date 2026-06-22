# sim_ws — Gazebo + Nav2 simulation workspace

A **separate colcon workspace** for the Dicerox mapping / path-planning
simulation. It is kept apart from the hardware workspace
(`software/ros2_ws/`) on purpose: it pulls in Gazebo Fortress and `ros_gz_*`,
which are **not** built or run on the robot (Jetson) or the operator
workstation. Build it only on a dev machine that has Gazebo installed.

This is the simulation counterpart to the real-robot `dicerox_mapping`
package in `software/ros2_ws/` (which does live SLAM on the Jetson with
slam_toolbox + ZED + RPLidar). Here, navigation runs against a known static
map inside Gazebo so the Nav2 planning stack can be developed and tuned
without hardware.

## Packages

Map-simulation stack (Stage A demo):

| Package | Purpose |
|---------|---------|
| `tracked_robot_bringup` | Single-command Stage A launch (Gazebo + TF + bridge + odom + Nav2 + RViz). |
| `tracked_robot_description` | URDF/xacro for the tracked base (imports the meshes below). |
| `tracked_robot_gz` | Gazebo worlds, models, and the `ros_gz_bridge` config. |
| `tracked_robot_nav2` | Nav2 params, static map (`generate_map.py`), and RViz config. |
| `tracked_robot_odom` | `tracked_odom_node` (cmd_vel → odom + TF) and `scan_sanitizer_node`. |

Robot model (URDF + STL meshes), imported by `tracked_robot_description`:

| Package | Purpose |
|---------|---------|
| `dicerox_urdf_v1` | Chassis + flipper meshes and URDF. |
| `dicerox_arm_urdf` | 6-DOF arm meshes and URDF (arm fixed at zero in Stage A). |

> These two description packages are copied from the standalone Dicerox arm
> workspace; only the URDF + meshes are used here, not the MoveIt / CAN
> control stack.

## Architecture (how it works)

`map → odom → base_footprint → base_link → {laser_link, flippers, arm}`

- **Mapping is static, not SLAM.** `tracked_robot_nav2/maps/generate_map.py`
  procedurally builds a 200×200 occupancy grid matching the Gazebo world;
  `map_server` serves it; **AMCL** localizes against it.
- **Sensing:** a front-facing GPU lidar publishes in Gazebo → `ros_gz_bridge`
  → `/scan_raw` → `scan_sanitizer_node` → `/scan`.
- **Odometry:** `tracked_odom_node` integrates `/cmd_vel` (open-loop
  dead-reckoning) into `/odom` + the `odom → base_footprint` TF.
- **Path planning (Nav2):** NavFn global planner over a static-map global
  costmap; DWB local planner over a rolling, lidar-fed local costmap;
  orchestrated by the BT navigator.

## Build & run

```bash
cd software/sim_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch tracked_robot_bringup stage_a_demo.launch.py
```

Requires Gazebo Fortress and `ros-humble-ros-gz*` plus the Nav2 stack
(`ros-humble-navigation2`, `ros-humble-nav2-bringup`).
