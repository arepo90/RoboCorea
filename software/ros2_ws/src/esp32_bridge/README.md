# esp32_bridge

RoboCorea Jetson-side ROS 2 (Humble) node that bridges the ESP32 binary UART
protocol to ROS 2 topics. This is the "relay" core described in
[`../../../../reference/architecture.md`](../../../../reference/architecture.md):
it runs on the Jetson Orin Nano, publishes telemetry/sensor/motor status, and
forwards commands (arm joints, e-stop, keybinds, PPM calibration, sensor enable)
back down to the ESP32.

The wire format is the packed binary protocol in the firmware's
`include/robot_types.h`; the `struct` formats in `main_bridge.py` must stay
byte-identical to it.

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
ros2 launch esp32_bridge esp32_bridge.launch.py serial_port:=/dev/ttyUSB0 baud_rate:=921600
```

The node opens the serial port asynchronously and reconnects on unplug. It
starts with all ESP32 sensors disabled; enable them by publishing a bitmask on
`/sensors/enable_mask` (bit0 = magnetometer; the ESP32 has no IMU — orientation
comes from the ZED2 camera). It also integrates the two traction VESCs'
tachometers into track odometry on `/odom/wheel` (`nav_msgs/Odometry`).

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

> Not implemented here (by design): the GUI, computer vision, MoveIt/kinematics,
> and the thermal-camera node. This package is only the USB-serial bridge.
