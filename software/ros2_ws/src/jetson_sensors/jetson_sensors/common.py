from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import UInt8


HEIGHT = 24
WIDTH = 32


def sensor_qos() -> QoSProfile:
    """Latest-frame QoS: stale thermal frames are never queued."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def enable_mask_qos() -> QoSProfile:
    """Latched QoS for /sensors/enable_mask.

    transient_local so a node that starts (or is restarted by the GUI's I2C
    start/stop) after the operator has already set the mask immediately receives
    the last enable choice, instead of falling back to start_enabled. The
    publisher (GUI) must match this durability for the two to connect.
    """
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class EnableMaskGate:
    """Shared /sensors/enable_mask gate for Jetson-hosted passive sensors."""

    def __init__(
        self,
        node,
        topic: str,
        bit: int,
        start_enabled: bool,
        sensor_name: str,
    ) -> None:
        self.enabled = bool(start_enabled)
        self._node = node
        self._sensor_name = sensor_name
        self._subscription = None
        self._topic = str(topic or '')
        self._bit = int(bit)
        if not self._topic:
            return
        if self._bit < 0 or self._bit > 7:
            raise ValueError('enable_mask_bit must be in [0, 7]')
        self._subscription = node.create_subscription(
            UInt8,
            self._topic,
            self._on_mask,
            enable_mask_qos(),
        )

    def _on_mask(self, msg: UInt8) -> None:
        enabled = bool(int(msg.data) & (1 << self._bit))
        if enabled != self.enabled:
            state = 'enabled' if enabled else 'disabled'
            self._node.get_logger().info(
                f'{self._sensor_name} {state} by {self._topic} bit {self._bit}'
            )
        self.enabled = enabled
