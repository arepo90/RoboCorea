#!/usr/bin/env python3
"""
robot_manager — always-on Jetson node that starts/stops the robot's sensor stack
through systemd and reports its live state, so the workstation GUI can bring the
ZED + RPLidar up and down cleanly over ROS 2 (no SSH, no orphaned nodes).

It does NOT launch the drivers itself — it asks the user systemd instance to
start/stop the `rescue-sensors.target` unit, which owns the ZED and RPLidar via
their cgroups (KillSignal=SIGINT then SIGKILL). That is what guarantees a clean
teardown; this node is just the remote trigger + status reporter. Run it as its
own `systemctl --user` unit (rescue_bringup/systemd/robot-manager.service) so it
is always available to receive service calls.

Services
--------
  /robot/sensors/start    std_srvs/Trigger  -> systemctl --user start   <target>
  /robot/sensors/stop     std_srvs/Trigger  -> systemctl --user stop    <target>
  /robot/sensors/restart  std_srvs/Trigger  -> systemctl --user restart <target>

Published (latched, transient_local depth 1)
  /robot/sensors/status   std_msgs/String   -> "<overall> (zed=<s> lidar=<s>)"
        overall in {active, inactive, activating, failed, partial}

Parameters
  target          (str)        systemd unit to control  [rescue-sensors.target]
  units           (str[])      member units to report    [zed.service, lidar.service]
  status_period   (float, s)   status poll period        [1.0]
"""

import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSReliabilityPolicy)
from std_msgs.msg import String
from std_srvs.srv import Trigger

DEFAULT_TARGET = 'rescue-sensors.target'
DEFAULT_UNITS = ['zed.service', 'lidar.service']


class RobotManager(Node):
    def __init__(self):
        super().__init__('robot_manager')

        self.declare_parameter('target', DEFAULT_TARGET)
        self.declare_parameter('units', DEFAULT_UNITS)
        self.declare_parameter('status_period', 1.0)

        self.target = str(self.get_parameter('target').value)
        self.units = list(self.get_parameter('units').value)
        period = float(self.get_parameter('status_period').value)

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._status_pub = self.create_publisher(String, '/robot/sensors/status', latched)

        self.create_service(Trigger, '/robot/sensors/start', self._on_start)
        self.create_service(Trigger, '/robot/sensors/stop', self._on_stop)
        self.create_service(Trigger, '/robot/sensors/restart', self._on_restart)

        self._last_status = None
        self.create_timer(max(0.2, period), self._publish_status)
        self.get_logger().info(
            f"robot_manager ready — controlling '{self.target}' (units: {self.units})")

    # ── systemctl helpers ────────────────────────────────────────────────────
    def _systemctl(self, *args, timeout=15.0):
        cmd = ['systemctl', '--user', *args]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
            return r.returncode, (r.stdout + r.stderr).strip()
        except Exception as e:  # noqa: BLE001 - surface any failure to the caller
            return 1, str(e)

    def _action(self, verb, resp):
        # --no-block returns immediately; the /robot/sensors/status topic then
        # reflects activating -> active (or failed), so the GUI never blocks on a
        # slow ZED start.
        rc, out = self._systemctl(verb, '--no-block', self.target)
        resp.success = (rc == 0)
        resp.message = f"{verb} {self.target}: {'requested' if resp.success else 'FAILED'} {out}".strip()
        (self.get_logger().info if resp.success else self.get_logger().error)(resp.message)
        self._publish_status()
        return resp

    def _on_start(self, request, response):
        return self._action('start', response)

    def _on_stop(self, request, response):
        return self._action('stop', response)

    def _on_restart(self, request, response):
        return self._action('restart', response)

    # ── status polling ───────────────────────────────────────────────────────
    def _unit_state(self, unit):
        # is-active returns active/inactive/activating/deactivating/failed and a
        # nonzero exit for anything but "active" — we only care about the word.
        _, out = self._systemctl('is-active', unit, timeout=5.0)
        return out.splitlines()[0] if out else 'unknown'

    def _publish_status(self):
        states = [self._unit_state(u) for u in self.units]
        if states and all(s == 'active' for s in states):
            overall = 'active'
        elif states and all(s == 'inactive' for s in states):
            overall = 'inactive'
        elif any(s == 'failed' for s in states):
            overall = 'failed'
        elif any(s in ('activating', 'deactivating') for s in states):
            overall = 'activating'
        else:
            overall = 'partial'
        detail = ' '.join(f"{u.split('.')[0]}={s}" for u, s in zip(self.units, states))
        msg = f"{overall} ({detail})"
        if msg != self._last_status:
            self._last_status = msg
            self._status_pub.publish(String(data=msg))


def main():
    rclpy.init()
    node = RobotManager()
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
