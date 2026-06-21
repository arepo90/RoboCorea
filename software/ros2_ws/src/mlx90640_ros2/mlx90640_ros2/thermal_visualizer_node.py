from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from mlx90640_ros2.common import HEIGHT, WIDTH, sensor_qos


def image_to_array(msg: Image) -> np.ndarray:
    if msg.encoding != '32FC1' or msg.height != HEIGHT or msg.width != WIDTH:
        raise ValueError(
            f'Expected {WIDTH}x{HEIGHT} 32FC1, got '
            f'{msg.width}x{msg.height} {msg.encoding}'
        )
    return np.frombuffer(msg.data, dtype=np.float32).reshape(HEIGHT, WIDTH)


def gaussian_blur(image: np.ndarray, sigma: float) -> np.ndarray:
    """Small separable Gaussian filter without SciPy/OpenCV dependencies."""
    if sigma <= 0.0:
        return image
    radius = max(1, int(round(3.0 * sigma)))
    x = np.arange(-radius, radius + 1, dtype=np.float32)
    kernel = np.exp(-(x * x) / (2.0 * sigma * sigma))
    kernel /= kernel.sum()
    result = image.astype(np.float32, copy=False)
    for axis in (0, 1):
        pad = [(0, 0), (0, 0)]
        pad[axis] = (radius, radius)
        padded = np.pad(result, pad, mode='edge')
        result = np.apply_along_axis(
            lambda row: np.convolve(row, kernel, mode='valid'), axis, padded
        )
    return result


def _cubic_weights(fraction: np.ndarray) -> np.ndarray:
    """Catmull-Rom weights for offsets -1, 0, 1, 2."""
    f = fraction
    return np.stack((
        -0.5 * f + f * f - 0.5 * f * f * f,
        1.0 - 2.5 * f * f + 1.5 * f * f * f,
        0.5 * f + 2.0 * f * f - 1.5 * f * f * f,
        -0.5 * f * f + 0.5 * f * f * f,
    ), axis=1)


def bicubic_resize(image: np.ndarray, scale: int) -> np.ndarray:
    if scale < 1:
        raise ValueError('visualization_scale must be at least 1')
    if scale == 1:
        return image.astype(np.float32, copy=True)
    out_h, out_w = image.shape[0] * scale, image.shape[1] * scale
    source_y = (np.arange(out_h) + 0.5) / scale - 0.5
    source_x = (np.arange(out_w) + 0.5) / scale - 0.5
    base_y = np.floor(source_y).astype(int)
    base_x = np.floor(source_x).astype(int)
    wy = _cubic_weights(source_y - base_y)
    wx = _cubic_weights(source_x - base_x)
    iy = np.clip(base_y[:, None] + np.arange(-1, 3), 0, image.shape[0] - 1)
    ix = np.clip(base_x[:, None] + np.arange(-1, 3), 0, image.shape[1] - 1)
    horizontal = np.sum(image[:, ix] * wx[None, :, :], axis=2)
    return np.sum(horizontal[iy, :] * wy[:, :, None], axis=1).astype(np.float32)


def thermal_colormap(image: np.ndarray, minimum: float, maximum: float) -> np.ndarray:
    if maximum <= minimum:
        maximum = minimum + 0.1
    value = np.clip((image - minimum) / (maximum - minimum), 0.0, 1.0)
    red = np.clip(4.0 * value - 1.5, 0.0, 1.0)
    green = np.clip(2.0 - np.abs(4.0 * value - 2.0), 0.0, 1.0)
    blue = np.clip(1.5 - 4.0 * value, 0.0, 1.0)
    return (np.stack((red, green, blue), axis=-1) * 255.0).astype(np.uint8)


def rgb_message(rgb: np.ndarray, source: Image) -> Image:
    rgb = np.ascontiguousarray(rgb)
    msg = Image()
    msg.header = source.header
    msg.height, msg.width = rgb.shape[:2]
    msg.encoding = 'rgb8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = rgb.tobytes()
    return msg


class ThermalVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__('thermal_visualizer_node')
        self.declare_parameter('input_topic', 'thermal/image_filtered')
        self.declare_parameter('output_topic', 'thermal/image_color')
        self.declare_parameter('visualization_scale', 10)
        self.declare_parameter('gaussian_sigma', 0.0)
        self.declare_parameter('auto_scale', True)
        self.declare_parameter('min_celsius', 15.0)
        self.declare_parameter('max_celsius', 45.0)
        self.declare_parameter('range_alpha', 0.2)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.scale = int(self.get_parameter('visualization_scale').value)
        self.sigma = float(self.get_parameter('gaussian_sigma').value)
        self.auto_scale = bool(self.get_parameter('auto_scale').value)
        self.minimum = float(self.get_parameter('min_celsius').value)
        self.maximum = float(self.get_parameter('max_celsius').value)
        self.range_alpha = float(self.get_parameter('range_alpha').value)
        if not 0.0 < self.range_alpha <= 1.0:
            raise ValueError('range_alpha must be in (0, 1]')

        self.publisher = self.create_publisher(Image, output_topic, sensor_qos())
        self.subscription = self.create_subscription(
            Image, input_topic, self.receive_frame, sensor_qos()
        )
        self.range_initialized = False
        self.get_logger().info(
            f'Visualizing {input_topic} as {output_topic} at '
            f'{WIDTH * self.scale}x{HEIGHT * self.scale}'
        )

    def receive_frame(self, msg: Image) -> None:
        try:
            thermal = image_to_array(msg)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return
        thermal = gaussian_blur(thermal, self.sigma)
        if self.auto_scale:
            frame_min, frame_max = np.nanpercentile(thermal, (2.0, 98.0))
            if not self.range_initialized:
                self.minimum, self.maximum = float(frame_min), float(frame_max)
                self.range_initialized = True
            else:
                a = self.range_alpha
                self.minimum = (1.0 - a) * self.minimum + a * float(frame_min)
                self.maximum = (1.0 - a) * self.maximum + a * float(frame_max)
        enlarged = bicubic_resize(thermal, self.scale)
        rgb = thermal_colormap(enlarged, self.minimum, self.maximum)
        self.publisher.publish(rgb_message(rgb, msg))


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = ThermalVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
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
