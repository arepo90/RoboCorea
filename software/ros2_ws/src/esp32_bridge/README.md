# esp32_bridge

RoboCorea Jetson-side ROS 2 (Humble) node that bridges two ESP32 binary UART
links to ROS 2 topics. This is the "relay" core described in
[`../../../../reference/architecture.md`](../../../../reference/architecture.md):
it runs on the Jetson Orin Nano, auto-identifies the chassis and arm PCBs from
their firmware identity frames, publishes telemetry and motor status, and
forwards commands to the owning board. The MLX90640 thermal camera + LIS3MDL
magnetometer now live on the **arm PCB**; the bridge decodes their UART frames
(`MSG_SENSOR_THERMAL` / `MSG_SENSOR_MAG`) and republishes `/sensors/thermal`
(+`_raw`,`_status`) and `/sensors/mag`, and relays the GUI's
`/sensors/enable_mask` down to the arm as `MSG_SENSOR_ENABLE`.

The wire format is the packed binary protocol in the firmware's
`include/robot_types.h`; the `struct` formats in `main_bridge.py` and constants
in `protocol.py` must stay byte-identical to it.

## Build

```bash
cd software/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select esp32_bridge
source install/setup.bash
```

Requires `pyserial` (`pip3 install pyserial` or `apt install python3-serial`).

## CAN presence check

With `esp32_bridge` running, this helper reports which expected arm CAN devices
have produced telemetry:

```bash
ros2 run esp32_bridge can_presence_check
ros2 service call /arm/arm std_srvs/srv/Trigger "{}"
```

Expected devices are ODrive nodes `0x10/0x11/0x12`, ZE300 device `13`, and
LKTech motors `14/15`.

## Arm operating mode

```bash
# J1-J6 position controlled
ros2 service call /arm/mode/dexterity std_srvs/srv/Trigger "{}"

# J1-J4 controlled; LKTech J5/J6 motor-stop (torque-off)
ros2 service call /arm/mode/chassis std_srvs/srv/Trigger "{}"

ros2 topic echo /arm/operating_mode
ros2 topic echo /arm/joint_active_mask
```

The active mask is `0x3f` in ready dexterity mode and `0x0f` in ready chassis
mode. It is zero while the arm lifecycle is not `READY`.

## Run

```bash
ros2 launch esp32_bridge esp32_bridge.launch.py

# Optional explicit bench override:
ros2 launch esp32_bridge esp32_bridge.launch.py serial_port:=/dev/serial/by-id/usb-...
```

By default the node scans only the allowlisted globs in `serial_candidates`
(`/dev/serial/by-id/*,/dev/serial/by-path/*`). Raw `/dev/ttyUSB*` probing is
opt-in by changing that parameter. Each candidate is opened asynchronously and
bound only after firmware sends `MSG_BOARD_IDENTITY`.

Discovery re-runs every `discovery_period` seconds (default 2 s) and is keyed by
resolved device (`realpath`), so a board matched by both the by-id and by-path
globs yields a single link. A link whose device has disappeared (unplugged, or a
USB re-enumeration that moved it to a new `/dev/ttyUSB*`) is **reaped** — its
thread is stopped and its role binding cleared — before a fresh link is started,
so the map cannot leak links or open a re-enumerated board twice. Ports are also
opened `exclusive` (POSIX flock) as a second guard against a double-open race.

The chassis role publishes `/robot/*`, `/encoders/*`, `/odom/wheel`, and
`/motors/vesc_status`. The arm role publishes `/arm/*` plus ODrive/LKTech/ZE300
telemetry. Software `/robot/estop` is broadcast to all discovered ESP links;
chassis RC e-stop transitions are mirrored to the arm ESP over the Jetson
bridge. The bridge integrates the two traction VESC tachometers into track
odometry on `/odom/wheel` (`nav_msgs/Odometry`).
The odometry `gear_ratio` parameter is the traction reduction, so its default is
`23.333` to match `TRACTION_GEAR_RATIO` in the ESP firmware.

## Topics

See the module docstring at the top of
[`esp32_bridge/main_bridge.py`](esp32_bridge/main_bridge.py) for the full list of
published and subscribed topics.

### Arm joints

`/joint_states` (`sensor_msgs/JointState`, radians) is mapped by joint **name**
to J1..J6 order using the `joint_names` parameter, converted to the firmware's
physical-degree convention with `joint_command_signs` (default Dicerox signs:
`[-1,-1,-1,-1,-1,+1]`), and sent as `MSG_ARM_JOINTS`. Set `joint_names` to match
your arm URDF:

```bash
ros2 run esp32_bridge esp32_bridge --ros-args \
  -p joint_names:='[shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]'
```

> Not implemented here (by design): the GUI, computer vision, and
> MoveIt/kinematics. The passive sensors (thermal + mag) are no longer separate
> Jetson nodes — they're read on the arm PCB and republished by this bridge.
