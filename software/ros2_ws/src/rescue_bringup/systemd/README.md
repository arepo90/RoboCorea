# rescue_bringup — systemd units + robot_manager

Clean, deterministic start/stop of the robot's relay + perception stacks
(ESP32 bridge, ZED + RPLidar, SLAM, localization, Nav2, C920 video), drivable
from the workstation GUI over ROS 2 (middleware: **Zenoh**, `rmw_zenoh`).

> **Easy path:** don't run these commands by hand — `./jetson.sh` (repo root)
> installs these units, enables headless persistence, and starts the always-on
> ones in one shot. See [`OPERATIONS.md`](../../../../../OPERATIONS.md). This
> README documents what `jetson.sh` does and how to drive the units manually.

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
| *(removed)* `jetson-sensors.service` | The MLX90640 thermal camera + LIS3MDL magnetometer moved to the **arm PCB** (read by the arm ESP32, relayed by `esp32_bridge`), so there is no Jetson I2C sensor service. Enable/disable is the GUI's `/sensors/enable_mask` (→ `MSG_SENSOR_ENABLE`), not a systemd stack. |
| `esp32-bridge.service` | **Always-on** serial⇄ROS 2 relay (drive/flippers/arm, thermal+mag from the arm PCB, wheel odom). Not PartOf any target. Pin a serial device via `ROBOCOREA_SERIAL_PORT` (or `jetson.sh --serial`), else it scans `ttyCH341USB*` + `serial/by-id`. |
| `camera-streamer.service` | **Always-on** multi-camera SRT streamer (a ROS node, `rescue_bringup camera_streamer`). Auto-detects every USB UVC camera, streams each to the GUI over SRT (video-only), gives the primary C920 its mic as Opus (the A/V stream), taps the ZED SDK's published left image into its own SRT stream, and advertises the live list on `/robot/camera_streams` so the GUI auto-populates + handles hot-plug. Tunables are ROS params in `camera_streamer.py` (primary_match, *_port, bitrate, zed_topic, zed_enable, …). Replaces the old `c920-stream.service` / `c920_srt_stream.sh` (kept as a manual fallback). |
| `jetson-speaker.service` | **Always-on** talkback receiver: an SRT listener that plays the operator's mic (from the GUI, reverse of `c920-stream`) on a Jetson speaker. Plain GStreamer, not a ROS node. Pick the output device at the top of `gui/scripts/jetson_speaker_sink.sh` (default = system default sink; USB or a paired+trusted Bluetooth speaker). Runs as a `--user` service to share the session audio server. |
| `rescue-mapping.service` | SLAM + odometry (`real_mapping.launch.py`, `use_rviz:=false`) — runs on the robot; the workstation only views `/map` over the network. **Uses the same `sensor_frontend` front-end as `rescue-localization.service`, so mapping and localization share one canonical TF tree** (`map→odom→base_footprint→base_link→base_laser`). On-demand; needs the sensor stack up. Odometry source switchable (EKF vs ZED-direct) via `ROBOCOREA_USE_EKF` — see [Odometry mode](#odometry-mode-prod-vs-bench). |
| `rescue-mapping3d.service` | 3-D OctoMap (`rescue_mapping3d`) — builds an octree from the ZED cloud + TF on the robot; publishes only the compressed binary octree on `/robot/map3d` (latched, ~1 Hz). The raw cloud never leaves the Jetson. On-demand; needs sensors + mapping up. Also serves `/robot/map3d/{save,load}` (named `.bt` under `~/maps/<name>/`). |
| `rescue-localization.service` | 2-D localization on a **saved** map — **localization only** (AMCL + map_server + EKF, **no Nav2**) via `rescue_nav real_navigation.launch.py nav:=false`. On-demand; (re)started by `map_manager` on a GUI map-load, reading the active map from `~/.config/rescue/active_map.env` (`MAP_DIR=~/maps/<name>`). Needs sensors up. Set the start pose from the GUI (click-drag → `/initialpose`). |
| `rescue-navigation.service` | Nav2 navigation stack (planner + controller + BT + behaviors + smoother) via `rescue_nav nav2.launch.py` on the real-robot params. On-demand; started **explicitly** by the operator from the GUI **after** a map is loaded (localization up). Only publishes `/cmd_vel` — the robot moves only with the GUI **AUTO DRIVE** toggle on (`enable_cmd_vel_drive`). |
| `robot-manager.service` | Always-on node managing **stacks**: exposes `/robot/<stack>/{start,stop,restart}` + `/robot/<stack>/status` for `sensors` (ZED+lidar), `mapping` (SLAM+EKF), `mapping3d` (OctoMap), `localization` (saved-map AMCL) and `navigation` (Nav2). (No `i2c` stack — the thermal camera + magnetometer are on the **arm PCB**, relayed by `esp32_bridge`; their enable toggles are the GUI's `/sensors/enable_mask`.) |
| `map-manager.service` | Always-on node owning the **named map library** (`~/maps/<name>/`): `/robot/maps/{save,list,load,delete}`. 2-D save = `slam_toolbox serialize_map` + `nav2_map_server map_saver_cli`; 3-D save/load forwarded to `octomap_node`; 2-D load writes `active_map.env` + restarts `rescue-localization.service`. |

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

# always-on services (the GUI reaches the managers; the bridge is the core link)
systemctl --user enable --now robot-manager.service map-manager.service \
                              esp32-bridge.service camera-streamer.service \
                              jetson-speaker.service
systemctl --user enable zed.service lidar.service rescue-sensors.target
# rescue-mapping / rescue-localization / rescue-navigation are on-demand (no
# enable): the GUI starts mapping; map_manager starts localization on a map-load;
# the operator starts navigation. Maps live under ~/maps/<name>/ (first save).
# (All of this is exactly what `./jetson.sh` automates.)
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
ros2 service call /robot/mapping/start std_srvs/srv/Trigger "{}"   # SLAM + EKF
ros2 service call /robot/navigation/start std_srvs/srv/Trigger "{}"  # Nav2 (after a map is loaded)

# named maps (map_manager must be running):
ros2 service call /robot/maps/list   rescue_interfaces/srv/ListMaps "{}"
ros2 service call /robot/maps/save   rescue_interfaces/srv/SaveMap  "{name: arena1, kind: '2d'}"
ros2 service call /robot/maps/load   rescue_interfaces/srv/LoadMap  "{name: arena1, kind: both}"
ros2 service call /robot/maps/delete rescue_interfaces/srv/DeleteMap "{name: arena1}"
```

…or just use the GUI **Robot Systems** window (toolbar icon next to ⚙): the
**Perception** tab has the **Sensors / Mapping / 3D / Localization / Navigation ▶ / ⏹**
stack controls, and the **Maps** tab is the named map library (Load/Delete +
previews). Map **save** + the click-drag **Set Start Pose** live on the map
windows. Per-sensor **Thermal**/**Mag** enable toggles (gating
`/sensors/enable_mask`) stay on the main dashboard next to the readouts.

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
