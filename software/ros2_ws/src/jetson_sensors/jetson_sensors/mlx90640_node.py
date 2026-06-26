import math
import time
from typing import Optional

import numpy as np
import rclpy
from mlx90640_msgs.msg import ThermalStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from jetson_sensors.common import EnableMaskGate, HEIGHT, WIDTH, sensor_qos


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
        self.declare_parameter('raw_topic', '/sensors/thermal_raw')
        self.declare_parameter('filtered_topic', '/sensors/thermal')
        self.declare_parameter('status_topic', '/sensors/thermal_status')
        self.declare_parameter('filter_alpha', 0.55)
        self.declare_parameter('use_dummy', False)
        self.declare_parameter('enable_mask_topic', '/sensors/enable_mask')
        self.declare_parameter('enable_mask_bit', 1)
        self.declare_parameter('start_enabled', True)

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
        enable_mask_topic = str(self.get_parameter('enable_mask_topic').value)
        enable_mask_bit = int(self.get_parameter('enable_mask_bit').value)
        start_enabled = bool(self.get_parameter('start_enabled').value)

        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')
        if publish_rate > refresh_rate:
            self.get_logger().warning(
                'publish_rate exceeds refresh_rate; duplicate/stale sensor data may result'
            )

        # Remember how to (re)build the reader so we can retry / self-heal later.
        self._use_dummy = use_dummy
        self._bus = bus
        self._address = address
        self._refresh_rate = refresh_rate
        self._last_reader_retry = 0.0
        self.reader_retry_period = 5.0   # s between lazy re-init attempts

        self.filter = ExponentialFilter(alpha)
        # Advertise the topics FIRST, before touching the sensor, so /sensors/thermal
        # always exists for the GUI to discover — even if the MLX90640 is flaky at
        # boot. Frames only flow once it is enabled AND the reader is healthy.
        self.raw_publisher = self.create_publisher(Image, raw_topic, sensor_qos())
        self.filtered_publisher = self.create_publisher(
            Image, filtered_topic, sensor_qos()
        )
        self.status_publisher = self.create_publisher(
            ThermalStatus,
            status_topic,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )

        # The Adafruit MLX90640 constructor reads the whole EEPROM over I2C and can
        # raise *transiently* on a slow/noisy bus (e.g. "More than 4 outlier
        # pixels"). Retry a few times; if it still fails, keep the NODE ALIVE (the
        # topic stays advertised) and re-init lazily in publish_frame so thermal
        # self-heals instead of the node crashing and the source disappearing.
        self.reader = self._make_reader(attempts=5)

        self.enable_gate = EnableMaskGate(
            self,
            enable_mask_topic,
            enable_mask_bit,
            start_enabled,
            'MLX90640',
        )
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_frame)
        self.sequence = 0
        self.total_read_errors = 0
        self.consecutive_read_errors = 0
        self.last_frame_time = None
        mode = 'dummy' if use_dummy else f'I2C bus {bus}, address {address:#x}'
        enabled = 'enabled' if self.enable_gate.enabled else 'disabled'
        self.get_logger().info(
            f'Acquiring at {publish_rate:g} Hz from {mode}; currently {enabled}; '
            'image QoS is best-effort/keep-last(1)'
        )

    def _make_reader(self, attempts: int = 1):
        """Build the thermal reader, retrying the flaky I2C EEPROM init.

        Returns the reader, or None if every attempt failed (node stays alive).
        """
        last_exc = None
        for i in range(max(1, attempts)):
            try:
                reader = (
                    DummyReader() if self._use_dummy
                    else Mlx90640Reader(self._bus, self._address, self._refresh_rate)
                )
                if i > 0:
                    self.get_logger().info(f'MLX90640 reader ready on attempt {i + 1}')
                return reader
            except Exception as exc:  # noqa: BLE001 - any I2C/driver failure
                last_exc = exc
                time.sleep(0.5)
        self.get_logger().error(
            f'MLX90640 init failed ({type(last_exc).__name__}: {last_exc}); '
            '/sensors/thermal advertised but no frames until the sensor recovers'
        )
        return None

    def _publish_fault_status(self) -> None:
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

    def publish_frame(self) -> None:
        if not self.enable_gate.enabled:
            return
        # Sensor init failed earlier (or was dropped after repeated read errors) —
        # retry building the reader, throttled, so thermal self-heals.
        if self.reader is None:
            now = time.monotonic()
            if now - self._last_reader_retry < self.reader_retry_period:
                return
            self._last_reader_retry = now
            self.reader = self._make_reader(attempts=1)
            if self.reader is None:
                self._publish_fault_status()
                return
        try:
            thermal = self.reader.read()
        except (ValueError, OSError) as exc:
            self.total_read_errors += 1
            self.consecutive_read_errors += 1
            if self.consecutive_read_errors == 1 or self.consecutive_read_errors % 20 == 0:
                self.get_logger().warning(
                    f'Thermal read failed ({self.consecutive_read_errors} consecutive): {exc}'
                )
            # A wedged sensor keeps failing — drop the reader so it gets a clean
            # re-init on the next tick (above).
            if self.consecutive_read_errors % 20 == 0:
                self.reader = None
            self._publish_fault_status()
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
