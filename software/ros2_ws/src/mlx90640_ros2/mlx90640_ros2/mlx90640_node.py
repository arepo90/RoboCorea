import math
import time
from typing import Optional

import numpy as np
import rclpy
from mlx90640_msgs.msg import ThermalStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from mlx90640_ros2.common import HEIGHT, WIDTH, sensor_qos


class ExponentialFilter:
    """Low-latency temporal smoothing for complete thermal frames."""

    def __init__(self, alpha: float) -> None:
        if not 0.0 < alpha <= 1.0:
            raise ValueError('filter_alpha must be in (0, 1]')
        self.alpha = alpha
        self.value = None

    def update(self, frame: np.ndarray) -> np.ndarray:
        if self.value is None:
            self.value = np.array(frame, dtype=np.float32, copy=True)
        else:
            self.value += self.alpha * (frame - self.value)
        return self.value


class DummyReader:
    """Generate a moving temperature pattern for ROS-side testing."""

    def __init__(self) -> None:
        self.phase = 0.0
        self.y, self.x = np.mgrid[0:HEIGHT, 0:WIDTH]

    def read(self) -> np.ndarray:
        center_x = 15.5 + 8.0 * math.sin(self.phase)
        center_y = 11.5 + 5.0 * math.cos(self.phase * 0.7)
        hotspot = 12.0 * np.exp(
            -((self.x - center_x) ** 2 + (self.y - center_y) ** 2) / 20.0
        )
        noise = np.random.default_rng().normal(0.0, 0.12, (HEIGHT, WIDTH))
        self.phase += 0.12
        return np.asarray(22.0 + hotspot + noise, dtype=np.float32)


class Mlx90640Reader:
    """Read Celsius values from an MLX90640 over Linux I2C."""

    REFRESH_RATES = {
        1: 'REFRESH_1_HZ',
        2: 'REFRESH_2_HZ',
        4: 'REFRESH_4_HZ',
        8: 'REFRESH_8_HZ',
        16: 'REFRESH_16_HZ',
        32: 'REFRESH_32_HZ',
        64: 'REFRESH_64_HZ',
    }

    def __init__(self, bus: int, address: int, refresh_rate: int) -> None:
        try:
            from adafruit_extended_bus import ExtendedI2C as I2C
            import adafruit_mlx90640
        except ImportError as exc:
            raise RuntimeError(
                'Hardware mode requires adafruit-circuitpython-mlx90640 and '
                'adafruit-extended-bus'
            ) from exc

        rate_name = self.REFRESH_RATES.get(refresh_rate)
        if rate_name is None:
            raise ValueError(
                f'Unsupported refresh_rate {refresh_rate}; use '
                f'{sorted(self.REFRESH_RATES)}'
            )
        self._mlx = adafruit_mlx90640.MLX90640(I2C(bus), address=address)
        self._mlx.refresh_rate = getattr(
            adafruit_mlx90640.RefreshRate, rate_name
        )
        self._frame = [0.0] * (HEIGHT * WIDTH)

    def read(self) -> np.ndarray:
        self._mlx.getFrame(self._frame)
        return np.asarray(self._frame, dtype=np.float32).reshape(HEIGHT, WIDTH)


def array_to_image(thermal: np.ndarray, stamp, frame_id: str) -> Image:
    """Convert a 24x32 Celsius array to a ROS 32FC1 image."""
    image = np.asarray(thermal, dtype=np.float32)
    if image.shape != (HEIGHT, WIDTH):
        raise ValueError(f'Expected {(HEIGHT, WIDTH)}, got {image.shape}')
    image = np.ascontiguousarray(image)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = HEIGHT
    msg.width = WIDTH
    msg.encoding = '32FC1'
    msg.is_bigendian = 0
    msg.step = WIDTH * 4
    msg.data = image.tobytes()
    return msg


class Mlx90640Node(Node):
    def __init__(self) -> None:
        super().__init__('mlx90640_node')
        self.declare_parameter('bus', 7)
        self.declare_parameter('address', 0x33)
        self.declare_parameter('refresh_rate', 8)
        self.declare_parameter('publish_rate', 8.0)
        self.declare_parameter('frame_id', 'mlx90640_link')
        self.declare_parameter('raw_topic', 'thermal/image_raw')
        self.declare_parameter('filtered_topic', 'thermal/image_filtered')
        self.declare_parameter('status_topic', 'thermal/status')
        self.declare_parameter('filter_alpha', 0.55)
        self.declare_parameter('use_dummy', False)

        bus = int(self.get_parameter('bus').value)
        address = int(self.get_parameter('address').value)
        refresh_rate = int(self.get_parameter('refresh_rate').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        raw_topic = str(self.get_parameter('raw_topic').value)
        filtered_topic = str(self.get_parameter('filtered_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        alpha = float(self.get_parameter('filter_alpha').value)
        use_dummy = bool(self.get_parameter('use_dummy').value)

        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')
        if publish_rate > refresh_rate:
            self.get_logger().warning(
                'publish_rate exceeds refresh_rate; duplicate/stale sensor data may result'
            )

        self.reader = (
            DummyReader() if use_dummy
            else Mlx90640Reader(bus, address, refresh_rate)
        )
        self.filter = ExponentialFilter(alpha)
        self.raw_publisher = self.create_publisher(Image, raw_topic, sensor_qos())
        self.filtered_publisher = self.create_publisher(
            Image, filtered_topic, sensor_qos()
        )
        self.status_publisher = self.create_publisher(
            ThermalStatus,
            status_topic,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_frame)
        self.sequence = 0
        self.total_read_errors = 0
        self.consecutive_read_errors = 0
        self.last_frame_time = None
        mode = 'dummy' if use_dummy else f'I2C bus {bus}, address {address:#x}'
        self.get_logger().info(
            f'Acquiring at {publish_rate:g} Hz from {mode}; image QoS is '
            'best-effort/keep-last(1)'
        )

    def publish_frame(self) -> None:
        try:
            thermal = self.reader.read()
        except (ValueError, OSError) as exc:
            self.total_read_errors += 1
            self.consecutive_read_errors += 1
            if self.consecutive_read_errors == 1 or self.consecutive_read_errors % 20 == 0:
                self.get_logger().warning(
                    f'Thermal read failed ({self.consecutive_read_errors} consecutive): {exc}'
                )
            status = ThermalStatus()
            status.header.stamp = self.get_clock().now().to_msg()
            status.header.frame_id = self.frame_id
            status.sequence = self.sequence
            status.sensor_ok = False
            status.min_temperature = math.nan
            status.max_temperature = math.nan
            status.center_temperature = math.nan
            status.total_read_errors = self.total_read_errors
            status.consecutive_read_errors = self.consecutive_read_errors
            status.acquisition_rate_hz = 0.0
            self.status_publisher.publish(status)
            return

        now_monotonic = time.monotonic()
        rate = 0.0
        if self.last_frame_time is not None:
            elapsed = now_monotonic - self.last_frame_time
            if elapsed > 0.0:
                rate = 1.0 / elapsed
        self.last_frame_time = now_monotonic
        self.consecutive_read_errors = 0
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        stamp = self.get_clock().now().to_msg()
        filtered = self.filter.update(thermal)

        self.raw_publisher.publish(array_to_image(thermal, stamp, self.frame_id))
        self.filtered_publisher.publish(
            array_to_image(filtered, stamp, self.frame_id)
        )

        hotspot = int(np.nanargmax(thermal))
        status = ThermalStatus()
        status.header.stamp = stamp
        status.header.frame_id = self.frame_id
        status.sequence = self.sequence
        status.sensor_ok = True
        status.min_temperature = float(np.nanmin(thermal))
        status.max_temperature = float(np.nanmax(thermal))
        status.center_temperature = float(thermal[HEIGHT // 2, WIDTH // 2])
        status.hotspot_x = hotspot % WIDTH
        status.hotspot_y = hotspot // WIDTH
        status.total_read_errors = self.total_read_errors
        status.consecutive_read_errors = self.consecutive_read_errors
        status.acquisition_rate_hz = rate
        self.status_publisher.publish(status)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = Mlx90640Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        try:
            rclpy.shutdown()
        except (KeyboardInterrupt, RuntimeError):
            pass


if __name__ == '__main__':
    main()
