# rescue_nav

Autonomy proof-of-concept for RoboCorea: drive the tracked robot into a RoboCup
Rescue track, navigate to a waypoint (the end) and back, **autonomously**, with
progress visible in RViz.

> **Competition operators:** for the two production real-robot workflows
> (unknown-arena mapping and known-map navigation) and every copy-paste command,
> see **[docs/COMPETITION_WORKFLOWS.md](docs/COMPETITION_WORKFLOWS.md)**. The
> entry points are `real_mapping.launch.py`, `save_competition_map.launch.py`,
> and `real_navigation.launch.py` (AMCL by default, `localization:=slam_toolbox`
> optional). The launches below are the sim/PoC and lower-level building blocks.

This package is the **navigation layer**. It deliberately builds on
[`dicerox_mapping`](../dicerox_mapping/), which already does the 2D mapping,
localization, and TF. `rescue_nav` adds the pieces that were missing:

1. **Nav2** — global/local planning, the RegulatedPurePursuit controller, costmaps,
   the waypoint follower (`config/nav2_params.yaml`, `launch/nav2.launch.py`).
2. **A Gazebo Classic simulation** of the tracked base, so you can build/drive the
   whole stack with **no robot hardware** (`urdf/robot_sim.urdf.xacro`,
   `worlds/track.world`, `launch/sim.launch.py`, `launch/demo.launch.py`).
3. **The `/cmd_vel` → traction drive path** — the base is otherwise RC-only.
   A new firmware message (`MSG_TRACTION_CMD` 0x1A) + an `esp32_bridge` `/cmd_vel`
   subscriber turn Nav2's velocity commands into track motion (see "Hardware" below).
4. **Fused odometry (`robot_localization` EKF)** — a better `odom → base_footprint`
   than ZED-only, by fusing the VESC wheel odometry, ZED VIO, and the ZED IMU with
   **dynamic, condition-aware weighting**. See [Fused odometry](#fused-odometry-robot_localization-ekf).

## How the pieces fit (TF + topics)

```
            dicerox_mapping (mapping/localization/TF)        rescue_nav (navigation)
 ZED2 ─▶ zed_planar_odom ─▶ odom→base_footprint (TF)
 LiDAR ─▶ scan_frame_republisher ─▶ /scan_flat (base_laser)  ─┐
          static base_footprint→base_laser                    ├▶ Nav2 ─▶ /cmd_vel ─▶
          slam_toolbox ─▶ map→odom (TF) + /map  ──────────────┘            (sim: Gazebo;
                                                                       hardware: esp32_bridge
 TF tree:  map → odom → base_footprint → base_laser                     → MSG_TRACTION_CMD → VESC)
```

The **same** Nav2 config runs in sim and on hardware — only the thing producing
`odom→base_footprint` + `/scan` changes (Gazebo in sim, ZED+LiDAR on the robot).

## Fused odometry (`robot_localization` EKF)

`dicerox_mapping` ships with `zed_planar_odom` producing `odom → base_footprint`
from **ZED VIO only**. The wheel odometry the `esp32_bridge` computes from the two
traction VESC tachometers (`/odom/wheel`) goes unused. This package fuses them
(plus the ZED IMU) into a single, more robust estimate.

### Why it beats ZED-only

The two sensors fail in **opposite** regimes, so a complementary fusion wins:

| Quantity | Trusted source | Why |
|---|---|---|
| Forward velocity `vx` | **wheel** (+ ZED) | tachometer is metrically exact in a straight line; VIO scale drifts |
| Yaw rate `vyaw` | **ZED IMU gyro** (+ ZED VIO) | skid-steer tracks slip badly in turns → differential yaw is garbage |
| Lateral `vy` | **ZED VIO** | when visual tracking is healthy |
| Continuity in a dropout | **wheel + IMU** | ZED can lose tracking (smoke, dark, blank walls) and jump; wheels+gyro carry through |

Wheel **yaw is never fused** (slip). We fuse *velocities* (not two competing
absolute poses) so the `odom` frame stays smooth and drift-only.

### "Dynamic weights" = adaptive covariance

We do **not** hand-roll a filter. The stock EKF already weights each measurement
inversely to the covariance on that message. So `adaptive_odom_covariance` sits in
front of the EKF and **modulates the covariance** it stamps on each source from
live signals; the EKF rebalances itself update-to-update.

```
 /odom/wheel  (BEST_EFFORT)  ┐
 /filtered_odom (ZED VIO)    ├─▶ adaptive_odom_covariance ─▶ /odom/wheel_adaptive ┐
 /zed/.../imu/data           ┘        (dynamic covariance,    /odom/zed_adaptive   ├─▶ ekf_filter_node
                                       QoS + frame fixups)     /imu/adaptive       ┘     │
                                                               /diagnostics/odom_fusion  ▼
                                                                          /odometry/filtered + TF odom→base_footprint
```

Signals that detune a source (all in `config/ekf.yaml` defaults; gains are node params):

| Signal | Detects | Effect |
|---|---|---|
| `slip = |ω_wheel − ω_imu|` | track slip / skid turn | inflate wheel `vx` covariance |
| `tilt = max(|roll|,|pitch|)` | climbing a ramp/obstacle | inflate wheel `vx` covariance |
| `turn_rate = |ω_imu|` | motion blur | inflate ZED VIO covariance |
| ZED pose-cov trace | VIO low confidence / tracking loss | inflate ZED VIO covariance |

The node also fixes two plumbing gotchas for free: it **bridges QoS**
(`/odom/wheel` is `BEST_EFFORT`; the EKF subscribes `RELIABLE`) and **normalizes
frames** (the bridge stamps wheel odom `child_frame=base_link`; the ZED IMU is in
`zed_imu_link`) — everything is restamped to `base_footprint`.

> **IMU mount assumption.** The IMU is relabeled into `base_footprint` (identity)
> rather than chained through a `base→zed_imu_link` TF, to avoid conflicts with the
> ZED wrapper's own static frames. This assumes the **ZED is mounted level and
> forward-facing**. If yours is tilted/rotated, pass `imu_yaw_sign:=-1.0` to flip a
> reversed gyro, and re-verify on the bench (rotate CCW → `/imu/adaptive`
> `angular_velocity.z` must be **positive**).

### Run it

The EKF becomes the **sole owner** of `odom → base_footprint`, so the ZED-only TF
must be turned off. `real_mapping.launch.py` (mapping) and `real_navigation.launch.py`
(localization/nav) both wire that up — they share **one** front-end
(`sensor_frontend.launch.py`, the canonical `map→odom→base_footprint→base_link→base_laser`
tree) and run the fusion layer with the ZED-only TF off. The `use_ekf` arg picks
the odometry owner: `true` (default) = the EKF; `false` = `zed_planar_odom` direct
(bench/no-bridge).

```bash
cd software/ros2_ws
colcon build --packages-select esp32_bridge dicerox_mapping rescue_nav
source install/setup.bash

# Build a map with FUSED odom (run the ZED wrapper with publish_tf:=false first):
ros2 launch rescue_nav real_mapping.launch.py
#   ... drive with the RC, then save BOTH artifacts (grid + posegraph):
ros2 launch rescue_nav save_competition_map.launch.py map_name:=$HOME/maps/track

# Localize against a saved map + navigate with FUSED odom:
ros2 launch rescue_nav real_navigation.launch.py map:=$HOME/maps/track.yaml
```

> On the real robot this is automated: `rescue-mapping.service` runs
> `real_mapping.launch.py` and `rescue-localization.service` runs
> `real_navigation.launch.py nav:=false`, both started from the GUI. See
> [`OPERATIONS.md`](../../../../OPERATIONS.md).

Just the fusion layer (e.g. on top of your own mapping launch):
```bash
ros2 launch rescue_nav odom_fusion.launch.py        # adaptive node + EKF only
```

> **Prereq:** the `esp32_bridge` must be publishing `/odom/wheel`, and the ZED
> wrapper must run with **`publish_tf:=false`** (it must not own `odom→base`
> either). Architecture §18 item 12.

### Verify / tune on the bench

1. **Wheel-odom scale** (architecture §18 item 12): set the bridge params
   `wheel_circumference_m`, `track_width_m`, `gear_ratio` (=`23.333`),
   `tacho_steps_per_erev`. Push the robot 2 m by hand → `/odom/wheel` x ≈ 2 m;
   spin 360° → yaw ≈ 2π.
2. **IMU sign:** rotate CCW, confirm `/imu/adaptive` `angular_velocity.z > 0`,
   else `imu_yaw_sign:=-1.0`.
3. **Watch the weights live** while driving:
   `ros2 topic echo /diagnostics/odom_fusion` → `[slip, tilt, turn_rate, f_wheel, f_zed]`
   (or `rqt_plot`). `f_wheel` should spike during skid turns / climbing; `f_zed`
   during fast spins.
4. **Win check:** drive a closed loop back to the start. Compare return-to-origin
   error of `/odometry/filtered` (EKF) vs raw `/zed/zed_node/odom`. EKF should be
   tighter — especially after fast turns or a brief ZED occlusion.
5. **Tuning knobs:** base trust = `wheel_vx_var`, `zed_*_var`, `imu_vyaw_var`;
   adaptivity = `k_slip`, `k_tilt`, `k_blur` (`config/ekf.yaml` is the EKF;
   the gains are `adaptive_odom_covariance` params, set in `odom_fusion.launch.py`).

## Simulation (no hardware) — start here

```bash
cd software/ros2_ws
colcon build --packages-select dicerox_mapping rescue_nav
source install/setup.bash

# One shot: Gazebo + robot + online SLAM + Nav2 + RViz
ros2 launch rescue_nav demo.launch.py

# In another terminal: drive to the end of the track and back
ros2 run rescue_nav waypoint_runner
```

You can also drive manually to verify the plumbing:
`ros2 run teleop_twist_keyboard teleop_twist_keyboard` (publishes `/cmd_vel`), or
send a single goal with the **Nav2 Goal** tool in RViz.

The `demo.launch.py` uses **online SLAM** (map built live while navigating) for the
quickest result. The map-first workflow (the chosen design) is below.

### Map-first workflow in sim
```bash
ros2 launch rescue_nav sim.launch.py                       # Gazebo + robot
ros2 launch rescue_nav slam_sim.launch.py                  # online SLAM (build)
ros2 run teleop_twist_keyboard teleop_twist_keyboard       # drive to cover the track
ros2 run nav2_map_server map_saver_cli -t /map -f src/rescue_nav/maps/track
# then relaunch with slam_toolbox in localization mode + Nav2 (see Hardware §)
```

## Hardware (on the robot)

Mapping, localization and TF are **`dicerox_mapping`'s** job. `rescue_nav` adds
Nav2 + the drive path.

**1. Build the map** (drive with the **RC** — the tracks already obey it, no
`/cmd_vel` needed):
```bash
ros2 launch dicerox_mapping mapping.launch.py        # ZED + LiDAR + slam_toolbox
ros2 launch dicerox_mapping save_map.launch.py map_name:=$HOME/maps/track
```

**2. Localize + navigate** (set `map_file_name` in
`dicerox_mapping/config/slam_toolbox_localization.yaml` first):
```bash
ros2 launch dicerox_mapping mapping_robot.launch.py \
  slam_params_file:=$(ros2 pkg prefix dicerox_mapping)/share/dicerox_mapping/config/slam_toolbox_localization.yaml
ros2 launch rescue_nav nav2.launch.py use_sim_time:=false
ros2 run rescue_nav waypoint_runner --ros-args -p set_initial_pose:=false
```

**3. Enable the drive path.** Nav2 publishes `/cmd_vel`, but the bridge will not
command the tracks unless you turn it on (safety):
```bash
ros2 launch esp32_bridge esp32_bridge.launch.py \
  -p enable_cmd_vel_drive:=true -p max_track_speed_mps:=<measured full-scale m/s>
```

### Drive-path safety model (firmware arbitration)

The autonomy command (`MSG_TRACTION_CMD`) only moves the tracks when **all** hold:
the **RC link is up**, the **drive sticks are within deadband**, **virtual-flip is
off**, and a **fresh** command arrived (`EXT_DRIVE_TIMEOUT_MS`). So the operator
reclaims control instantly by touching a stick; Ch6-down still e-stops; losing the
RC link fails safe (tracks stop). On the bridge side it is gated by
`enable_cmd_vel_drive` (default **false**) with a `/cmd_vel` watchdog that releases
the tracks when commands stop. It reuses the RC path's eRPM scaling + direction
signs (`Locomotion::setTrackSpeeds`), so once RC driving is correct, autonomy is too.

## Waypoint runner

`waypoint_runner` (uses `nav2_simple_commander`) visits a list of poses, then
publishes `/nav/status`. The default waypoints drive to the **far end of the
straight corridor and back** (`[4.5, 0, 0,  0.3, 0, 3.14]`) — reachable with
**online SLAM** (`demo.launch.py`) because that corridor is visible from the start.
The **L-branch** (`~5.2, 2.6`) is occluded at startup, so to navigate there first
build + save the full map (map-first workflow) and then run with:
```bash
ros2 run rescue_nav waypoint_runner --ros-args \
  -p waypoints:="[5.2, 2.6, 1.57,  0.3, 0.0, 3.14]"
```
`waypoints` is a flat `[x, y, yaw, ...]` list (m, rad, `map` frame). It waits on
the navigator's lifecycle (`localizer:=bt_navigator` by default, since slam_toolbox
isn't an AMCL-style localizer); set `localizer:=amcl` only if you wire AMCL.

## Bench TODOs

- Lidar mount offset in `robot_sim.urdf.xacro` / `dicerox_mapping` `lidar_*` args.
- `esp32_bridge` `max_track_speed_mps` to match the firmware `TRACTION_ERPM_MAX`
  (Nav2 closes the loop on odometry, so approximate is fine to start).
- Verify autonomy turn direction matches odometry (same sign fix as RC, in the
  firmware `TRACTION_DIR_*`).
- Nav2 speed/inflation/controller tuning; footprint vs. the real chassis.

## Not in this PoC (future)

- A second, `map`-frame EKF (global, fuses absolute pose) on top of the current
  `odom`-frame EKF — add when a global heading/position reference is wired in.
- 3D / elevation mapping for ramps and stairs (this PoC is 2D).
- Deep GUI integration (RViz is the progress surface; `/nav/status` is published
  for a future dashboard widget).
