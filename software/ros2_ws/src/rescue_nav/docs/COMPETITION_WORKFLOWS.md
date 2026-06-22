# Dicerox competition navigation — operational workflows

Two operational scenarios, both on the **real robot**, both on **fused odometry**
(`robot_localization` EKF over VESC wheel odom + ZED VIO + ZED IMU).

```
Standard TF contract (real AND sim):
    map -> odom -> base_footprint -> base_link -> base_laser

TF ownership (exactly ONE publisher per edge):
    map -> odom .............. SLAM Toolbox (mapping)  |  AMCL / slam_toolbox (navigation)
    odom -> base_footprint ... robot_localization EKF  (odom_fusion.launch.py)
    base_footprint->base_link  static  (sensor_frontend.launch.py)
    base_link->base_laser ...  static  (sensor_frontend.launch.py)
```

`zed_planar_odom` runs with `publish_tf:=false` in both workflows — it is a pure
`/filtered_odom` *source*; the EKF owns the odom TF. Run the **ZED wrapper with
`publish_tf:=false`** too, so nothing competes for `odom -> base_footprint`.

---

## Build

```bash
cd ~/Projects/Robotics/RoboCorea/software/ros2_ws
colcon build --packages-select dicerox_mapping esp32_bridge rescue_nav
source install/setup.bash
```

## Sensor bringup (BOTH workflows — start these first, each in its own terminal)

```bash
# 1. ZED2 wrapper — MUST NOT publish the odom TF (the EKF owns it)
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 publish_tf:=false

# 2. Slamtec A1 LiDAR driver — publishes /scan
ros2 launch sllidar_ros2 sllidar_launch.py    # or your A1 driver

# 3. ESP32 bridge — gives the EKF /odom/wheel (VESC tachometers).
#    SAFE default: enable_cmd_vel_drive defaults to FALSE, so Nav2 cannot move
#    the tracks until you explicitly opt in (see Workflow B step 6).
ros2 launch esp32_bridge esp32_bridge.launch.py
```

Verify the sensors are alive:
```bash
ros2 topic hz /zed/zed_node/odom
ros2 topic hz /scan
ros2 topic hz /odom/wheel
```

---

## Workflow A — Unknown arena: build a map (Scenario A)

1. **Start sensors** (section above).

2. **Start mapping mode:**
   ```bash
   ros2 launch rescue_nav real_mapping.launch.py
   # headless: ros2 launch rescue_nav real_mapping.launch.py use_rviz:=false
   ```

3. **Drive the arena** (RC teleop / your teleop node) to cover the whole track.
   Drive smoothly; let loop closures happen.

4. **Verify the TF tree** (one publisher per edge, no gaps):
   ```bash
   ros2 run tf2_tools view_frames        # writes frames.pdf
   ros2 run tf2_ros tf2_echo map base_footprint
   ros2 run tf2_ros tf2_echo base_link base_laser
   ```

5. **Verify scan + fused odom:**
   ```bash
   ros2 topic echo /odometry/filtered --once     # EKF output
   ros2 topic echo /scan_flat --once             # frame_id must be base_laser
   ros2 topic hz /map                            # slam_toolbox publishing the grid
   ```

6. **Save the map** (leave mapping running):
   ```bash
   ros2 launch rescue_nav save_competition_map.launch.py map_name:=$HOME/maps/arena1
   ```

7. **Confirm files saved** (grid for AMCL + posegraph for slam_toolbox):
   ```bash
   ls -l $HOME/maps/arena1.pgm $HOME/maps/arena1.yaml \
         $HOME/maps/arena1.posegraph $HOME/maps/arena1.data
   ```

---

## Workflow B — Known map: localize and navigate (Scenario B)

1. **Start sensors** (section above).

2. **Start known-map navigation** with the saved map (AMCL is the default):
   ```bash
   ros2 launch rescue_nav real_navigation.launch.py map:=$HOME/maps/arena1.yaml
   # slam_toolbox localization instead of AMCL:
   ros2 launch rescue_nav real_navigation.launch.py \
       localization:=slam_toolbox slam_map_file:=$HOME/maps/arena1
   ```

3. **Set the initial pose.** The launch publishes `/initialpose` once from
   `initial_pose_{x,y,yaw}` (default 0,0,0). Override at launch:
   ```bash
   ros2 launch rescue_nav real_navigation.launch.py map:=$HOME/maps/arena1.yaml \
       initial_pose_x:=1.0 initial_pose_y:=0.5 initial_pose_yaw:=1.57
   ```
   …or set it interactively in RViz with **2D Pose Estimate**.

4. **Verify AMCL convergence** (particle cloud tightens in RViz; TF appears):
   ```bash
   ros2 run tf2_ros tf2_echo map odom        # published by AMCL
   ros2 topic echo /amcl_pose --once
   ros2 lifecycle get /amcl                    # -> active
   ```

5. **Send a 2D goal** — RViz **Nav2 Goal**, or:
   ```bash
   ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
     "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, \
       orientation: {w: 1.0}}}"
   # or the bundled runner:
   ros2 run rescue_nav waypoint_runner
   ```

6. **Let Nav2 actually drive the base.** Nav2 only *publishes* `/cmd_vel`; the
   ESP32 bridge ignores it until you opt in. Restart the bridge with drive on:
   ```bash
   ros2 launch esp32_bridge esp32_bridge.launch.py enable_cmd_vel_drive:=true
   ```
   The bridge watchdog releases the tracks back to RC if `/cmd_vel` goes stale
   (`cmd_vel_timeout`, default 0.3 s).

7. **Confirm `/cmd_vel` reaches the base and the robot moves safely:**
   ```bash
   ros2 topic echo /cmd_vel                    # nonzero while navigating
   ros2 lifecycle get /bt_navigator            # -> active
   ```

---

## Where `/cmd_vel` reaches the base controller

`nav2 velocity_smoother` → **`/cmd_vel`** → `esp32_bridge` (`/cmd_vel`
subscriber, only when `enable_cmd_vel_drive:=true`) → `MSG_TRACTION_CMD` → ESP32
→ traction VESCs. There is **no remap** by default: Nav2's output topic and the
bridge's input topic are both `/cmd_vel`. If you must isolate Nav2's output (e.g.
to insert a safety mux), remap `velocity_smoother`'s output and point the bridge
at the muxed topic.

## Quick lifecycle / health checks

```bash
ros2 node list | grep -E 'amcl|map_server|controller_server|planner_server|bt_navigator'
ros2 lifecycle get /map_server
ros2 lifecycle get /controller_server
ros2 topic hz /odometry/filtered      # fused odom heartbeat
```

## Reverting to the original ZED-only mapping behavior

The legacy stack is untouched. To map with ZED-only odom (no EKF/fusion):
```bash
ros2 launch dicerox_mapping mapping.launch.py        # original behavior
# or via the production launch, skipping the EKF:
ros2 launch rescue_nav real_mapping.launch.py odom_tf_owner:=zed
```
