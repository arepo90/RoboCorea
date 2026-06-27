#!/usr/bin/env python3
"""
nav_preflight — readiness checks for autonomous navigation.

Runs as part of the localization stack (real_navigation.launch.py), so it is up
from the moment a map is loaded — *before* the operator starts the Nav2
navigation stack. It continuously verifies the preconditions for navigation and
exposes them two ways:

  /nav/preflight         std_msgs/String (latched)  "<overall> (k=v k=v ...)"
        overall in {ready, blocked, degraded}
  /nav/preflight_check   std_srvs/Trigger
        success = all CRITICAL checks pass (safe to START navigation);
        message = the full per-check breakdown.

The GUI subscribes to /nav/preflight for a live readout and calls the service
before starting the Navigation stack, so navigation is BLOCKED until the
critical items are satisfied (with the missing ones reported to the operator).

Check classification
--------------------
CRITICAL (block navigation start — perception/localization must be healthy):
  scan   — <scan_topic> (/scan_flat) is publishing fresh frames
  odom   — <odom_topic> (/odometry/filtered, the EKF) is publishing fresh msgs
  tf     — full chain global_frame -> laser_frame (map -> base_laser) resolves
           (implicitly verifies map->odom from AMCL/slam_toolbox AND
            odom->base_footprint from the EKF AND the static base_link/laser TFs)
  map    — <map_topic> (/map) has a publisher (map_server / slam_toolbox)

ADVISORY / post-start (reported, do NOT block — they become true once you start
Nav2 and/or the robot is meant to move):
  nav2   — the Nav2 lifecycle servers are present (controller/planner/bt)
  bridge — something subscribes <cmd_vel_topic> (esp32_bridge is listening)
  auto   — autonomy drive is enabled (/autonomy/state) — robot won't MOVE if off
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSReliabilityPolicy)
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped  # noqa: F401  (doc of TF type)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from tf2_ros import Buffer, TransformListener


class NavPreflight(Node):
    def __init__(self):
        super().__init__('nav_preflight')

        self.declare_parameter('scan_topic', '/scan_flat')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('laser_frame', 'base_laser')
        self.declare_parameter('check_period', 1.0)
        self.declare_parameter('freshness_timeout', 2.0)
        # Nav2 lifecycle servers we expect once navigation is up (advisory).
        self.declare_parameter(
            'nav2_nodes', ['controller_server', 'planner_server', 'bt_navigator'])

        self._scan_topic = str(self.get_parameter('scan_topic').value)
        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self._map_topic = str(self.get_parameter('map_topic').value)
        self._global_frame = str(self.get_parameter('global_frame').value)
        self._laser_frame = str(self.get_parameter('laser_frame').value)
        self._fresh = float(self.get_parameter('freshness_timeout').value)
        self._nav2_nodes = list(self.get_parameter('nav2_nodes').value)
        period = max(0.2, float(self.get_parameter('check_period').value))

        self._scan_last = None
        self._odom_last = None
        self._autonomy = False

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        sensor = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # Lightweight freshness probes (timestamp only; no data is stored).
        self.create_subscription(LaserScan, self._scan_topic, self._on_scan, sensor)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self.create_subscription(Bool, '/autonomy/state', self._on_autonomy, latched)

        self._pub = self.create_publisher(String, '/nav/preflight', latched)
        self.create_service(Trigger, '/nav/preflight_check', self._on_check)

        self._last_msg = None
        self.create_timer(period, self._tick)
        self.get_logger().info(
            'nav_preflight up — publishing /nav/preflight, serving '
            '/nav/preflight_check')

    # ── probes ────────────────────────────────────────────────────────────────
    def _on_scan(self, _msg):
        self._scan_last = self.get_clock().now()

    def _on_odom(self, _msg):
        self._odom_last = self.get_clock().now()

    def _on_autonomy(self, msg: Bool):
        self._autonomy = bool(msg.data)

    def _fresh_ok(self, last) -> bool:
        if last is None:
            return False
        return (self.get_clock().now() - last) < Duration(seconds=self._fresh)

    def _tf_ok(self) -> bool:
        # Whole chain global_frame -> laser_frame. Time() (=0) asks for the latest
        # available, so it tolerates small stamp lag without flapping.
        try:
            return self._tf_buffer.can_transform(
                self._global_frame, self._laser_frame, Time())
        except Exception:  # noqa: BLE001 - any TF error == not ready
            return False

    def _node_present(self, name: str) -> bool:
        # Match on the base node name regardless of namespace.
        return any(n == name for n in self.get_node_names())

    # ── evaluation ──────────────────────────────────────────────────────────
    def _evaluate(self):
        """Return (checks, critical_ok). checks: list of (key, ok, critical, note)."""
        scan_ok = self._fresh_ok(self._scan_last)
        odom_ok = self._fresh_ok(self._odom_last)
        tf_ok = self._tf_ok()
        map_ok = self.count_publishers(self._map_topic) > 0
        nav2_present = [n for n in self._nav2_nodes if self._node_present(n)]
        nav2_ok = len(nav2_present) == len(self._nav2_nodes) and self._nav2_nodes
        bridge_ok = self.count_subscribers(self._cmd_vel_topic) > 0
        auto_ok = self._autonomy

        checks = [
            ('scan', scan_ok, True, self._scan_topic),
            ('odom', odom_ok, True, self._odom_topic),
            ('tf', tf_ok, True, '%s->%s' % (self._global_frame, self._laser_frame)),
            ('map', map_ok, True, self._map_topic),
            ('nav2', nav2_ok, False,
             '%d/%d' % (len(nav2_present), len(self._nav2_nodes))),
            ('bridge', bridge_ok, False, '%s sub' % self._cmd_vel_topic),
            ('auto', auto_ok, False, 'AUTO DRIVE'),
        ]
        critical_ok = all(ok for _k, ok, crit, _n in checks if crit)
        return checks, critical_ok

    def _summary(self, checks, critical_ok) -> str:
        if not critical_ok:
            overall = 'blocked'
        elif all(ok for _k, ok, _c, _n in checks):
            overall = 'ready'
        else:
            overall = 'degraded'   # safe to start, but won't fully drive yet
        detail = ' '.join('%s=%s' % (k, 'ok' if ok else 'NO')
                          for k, ok, _c, _n in checks)
        return '%s (%s)' % (overall, detail)

    def _tick(self):
        checks, critical_ok = self._evaluate()
        msg = self._summary(checks, critical_ok)
        if msg != self._last_msg:
            self._last_msg = msg
            self._pub.publish(String(data=msg))

    def _on_check(self, _req, resp):
        checks, critical_ok = self._evaluate()
        resp.success = bool(critical_ok)
        missing = [k for k, ok, crit, _n in checks if crit and not ok]
        lines = ['%s: %s%s' % (k, 'OK' if ok else 'MISSING',
                               '' if crit else ' (advisory)')
                 for k, ok, crit, _n in checks]
        head = ('navigation preflight OK' if critical_ok
                else 'navigation BLOCKED — missing: ' + ', '.join(missing))
        resp.message = head + '\n' + '\n'.join(lines)
        return resp


def main():
    rclpy.init()
    node = NavPreflight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
