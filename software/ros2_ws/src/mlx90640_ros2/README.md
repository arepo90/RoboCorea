# mlx90640_ros2

Low-latency ROS 2 driver and receiver-side visualizer for the 32x24 MLX90640.

## Topics

| Topic | Type | QoS | Purpose |
|---|---|---|---|
| `/thermal/image_raw` | `sensor_msgs/Image` (`32FC1`) | best effort, depth 1 | Unmodified Celsius matrix |
| `/thermal/image_filtered` | `sensor_msgs/Image` (`32FC1`) | best effort, depth 1 | Temporally filtered Celsius matrix |
| `/thermal/status` | `mlx90640_msgs/ThermalStatus` | reliable, depth 10 | Sequence, temperatures, hotspot, rate and errors |
| `/thermal/image_color` | `sensor_msgs/Image` (`rgb8`) | best effort, depth 1 | Local 320x240 bicubic display image |

Only the native 32x24 topics and small status message need to cross the network.
Run the visualizer on the receiving computer so the upscaled image never consumes
internet bandwidth.

## Install and build

Hardware mode requires:

```bash
python3 -m pip install adafruit-circuitpython-mlx90640 adafruit-extended-bus
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select mlx90640_msgs mlx90640_ros2
source install/setup.bash
```

The node user must have access to `/dev/i2c-7`.

## Run

Everything on one computer:

```bash
ros2 launch mlx90640_ros2 mlx90640.launch.py
```

Dummy sensor:

```bash
ros2 launch mlx90640_ros2 mlx90640.launch.py use_dummy:=true
```

For an internet-connected deployment, run only the driver on the Jetson:

```bash
ros2 launch mlx90640_ros2 driver.launch.py
```

Run the display processing on the operator computer:

```bash
ros2 launch mlx90640_ros2 visualizer.launch.py
rqt_image_view /thermal/image_color
```

## Tuning

Settings are in `config/mlx90640.yaml`.

- `refresh_rate` and `publish_rate`: default 8 Hz. Try 16 Hz only with a stable,
  fast I2C bus and short wiring.
- `filter_alpha`: temporal filter strength. `1.0` disables smoothing; smaller
  values are smoother but add lag. `0.55` is a low-latency default.
- `gaussian_sigma`: receiver-side spatial blur in source pixels. It defaults to
  `0.0` because Gaussian blur can hide small hotspots. Try `0.5` to `0.8` only
  for display aesthetics.
- `visualization_scale`: receiver-side bicubic scale. `10` produces 320x240.
- `range_alpha`: smooths auto-color-range changes to prevent flicker.

The raw and filtered images are approximately 3 KB per frame each. If bandwidth
is constrained, bridge only `/thermal/image_filtered` and `/thermal/status`, not
`/thermal/image_raw` or `/thermal/image_color`.

Check operation with:

```bash
ros2 topic hz /thermal/image_filtered --qos-reliability best_effort
ros2 topic echo /thermal/status
ros2 topic info /thermal/image_filtered --verbose
```
