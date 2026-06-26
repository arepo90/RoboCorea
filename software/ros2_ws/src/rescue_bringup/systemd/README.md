# rescue_bringup — systemd units + robot_manager

Clean, deterministic start/stop of the robot's sensor stack (ZED + RPLidar),
drivable from the workstation GUI over ROS 2.

## Why systemd

`ros2 launch` + Ctrl-C (especially over SSH) often leaves orphaned nodes/topics —
the ZED component container is slow to release the camera, a double Ctrl-C
SIGKILLs and orphans children, and SSH signal propagation is unreliable. systemd
tracks every process a unit spawns in a **cgroup** and tears the whole tree down
deterministically (`KillSignal=SIGINT` for a graceful stop, then SIGKILL after
`TimeoutStopSec`). The `robot_manager` node just asks systemd to start/stop the
target and reports the state — so the GUI gets one-click control with no orphans.

## Units

| Unit | Role |
|------|------|
| `zed.service` | ZED wrapper (`publish_tf:=false`). `PartOf` the target. |
| `lidar.service` | RPLidar driver on the stable `/dev/rplidar`. `PartOf` the target. |
| `rescue-sensors.target` | Group: start/stop/restart it to control **both** drivers. |
| `jetson-sensors.service` | I2C sensors (MLX90640 thermal + LIS3MDL mag), one process owning the shared bus. On-demand (no `[Install]`). |
| `rescue-mapping.service` | SLAM + odometry (`mapping_ekf.launch.py`, `use_rviz:=false`) — runs on the robot; the workstation only views `/map` over DDS. On-demand; needs the sensor stack up. Odometry source is switchable (EKF vs ZED-direct) via `ROBOCOREA_USE_EKF` — see [Odometry mode](#odometry-mode-prod-vs-bench). |
| `rescue-mapping3d.service` | 3-D OctoMap (`rescue_mapping3d`) — builds an octree from the ZED cloud + TF on the robot; publishes only the compressed binary octree on `/robot/map3d` (latched, ~1 Hz). The raw cloud never leaves the Jetson. On-demand; needs sensors + mapping up. |
| `robot-manager.service` | Always-on node managing **stacks**: exposes `/robot/<stack>/{start,stop,restart}` + `/robot/<stack>/status` for `sensors` (ZED+lidar), `i2c` (thermal+mag), `mapping` (SLAM+EKF) and `mapping3d` (OctoMap). |

Before deploying, check the marked lines in each unit:
- `Environment=ROS_DOMAIN_ID=20` — must match the workstation/GUI.
- workspace path `%h/RoboCorea/software/ros2_ws` — `%h` is `$HOME`; edit if different.
- `camera_model:=zed2`, lidar launch name, and `serial_port:=/dev/rplidar`.

`/dev/rplidar` is the stable symlink from the RPLidar udev rule:
`sudo cp src/sllidar_ros2/scripts/rplidar.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger`

## Odometry mode (prod vs bench)

`rescue-mapping.service` chooses what owns `odom -> base_footprint`, via the
`ROBOCOREA_USE_EKF` env var on the **user** systemd manager (default = prod):

| Mode | `ROBOCOREA_USE_EKF` | Owner of `odom->base_footprint` | When |
|------|---------------------|---------------------------------|------|
| **prod** (default) | unset / `true` | `robot_localization` EKF (wheel `vx` + ZED VIO + ZED IMU) | Real robot: tracks driving, bridge up, ZED rigidly mounted. |
| **bench** | `false` | `zed_planar_odom` (planar ZED VIO directly) | No tracks/bridge, or hand-moving the ZED. The planar ground-robot EKF mis-models that and adds jank; the ZED odom is already visual-**inertial** so the IMU isn't lost. |

```bash
# switch to bench (ZED-direct), then restart from the GUI (Stop SLAM -> Start SLAM)
systemctl --user set-environment ROBOCOREA_USE_EKF=false
# back to prod (EKF)
systemctl --user unset-environment ROBOCOREA_USE_EKF
```
The env var is read at unit **start**, so set it *before* (re)starting the stack.
`robot_manager` / the GUI Start-SLAM button pick it up automatically — no unit edit.

## Deploy (on the robot / Jetson)

```bash
cd ~/RoboCorea/software/ros2_ws
colcon build --symlink-install --packages-select rescue_bringup
source install/setup.bash

# install the unit files into the user systemd dir
mkdir -p ~/.config/systemd/user
SRC=install/rescue_bringup/share/rescue_bringup/systemd
cp "$SRC"/*.service "$SRC"/*.target ~/.config/systemd/user/
# (edit ~/.config/systemd/user/*.service if ROS_DOMAIN_ID / paths differ)

loginctl enable-linger "$USER"          # run without an active login (headless)
systemctl --user daemon-reload

# always-on manager (so the GUI can reach it) + sensor units enabled
systemctl --user enable --now robot-manager.service
systemctl --user enable zed.service lidar.service rescue-sensors.target
```

## Operate

```bash
# manual:
systemctl --user start  rescue-sensors.target
systemctl --user stop   rescue-sensors.target
systemctl --user restart rescue-sensors.target
journalctl --user -u zed.service -f          # logs

# from the workstation, over ROS 2 (robot_manager must be running):
ros2 service call /robot/sensors/start std_srvs/srv/Trigger "{}"   # ZED + lidar
ros2 service call /robot/sensors/stop  std_srvs/srv/Trigger "{}"
ros2 topic echo /robot/sensors/status
ros2 service call /robot/i2c/start std_srvs/srv/Trigger "{}"       # thermal + mag driver
ros2 service call /robot/i2c/stop  std_srvs/srv/Trigger "{}"
ros2 topic echo /robot/i2c/status
```

…or just use the **Sensors ▶ / ⏹** and **I2C Sensors ▶ / ⏹** buttons in the GUI
dashboard (the I2C section also has per-sensor **Thermal**/**Mag** enable toggles,
which gate `/sensors/enable_mask` — a disabled sensor stops touching the bus).

## Verify clean teardown

```bash
systemctl --user start rescue-sensors.target
ros2 node list           # one /sllidar_node + the /zed/* nodes
systemctl --user stop rescue-sensors.target
sleep 25
pgrep -af "zed|sllidar|component_container" || echo "CLEAN — no orphans"
```
(If `ros2 topic list` shows stale entries after stop, that's the ROS daemon
cache: `ros2 daemon stop` refreshes it.)
