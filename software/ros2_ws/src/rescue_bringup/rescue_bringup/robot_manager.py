#!/usr/bin/env python3
"""
robot_manager — always-on Jetson node that starts/stops the robot's driver
stacks through systemd and reports their live state, so the workstation GUI can
bring perception subsystems up and down cleanly over ROS 2 (no SSH, no orphaned
nodes).

It does NOT launch drivers itself — it asks the user systemd instance to
start/stop a unit, which owns the driver processes via their cgroups
(KillSignal=SIGINT then SIGKILL). That is what guarantees a clean teardown; this
node is just the remote trigger + status reporter. Run it as its own
`systemctl --user` unit (rescue_bringup/systemd/robot-manager.service) so it is
always available to receive service calls.

It manages a configurable set of *stacks*. Each stack has a control unit (what
gets started/stopped) and member units (what is polled for status). For every
stack `<name>` it exposes:

  /robot/<name>/start    std_srvs/Trigger  -> systemctl --user start   <unit>
  /robot/<name>/stop     std_srvs/Trigger  -> systemctl --user stop    <unit>
  /robot/<name>/restart  std_srvs/Trigger  -> systemctl --user restart <unit>
  /robot/<name>/status   std_msgs/String (latched) -> "<overall> (m=<s> ...)"
        overall in {active, inactive, activating, failed, partial}

Default stacks:
  sensors -> rescue-sensors.target   (members: zed.service, lidar.service)
  i2c     -> jetson-sensors.service  (members: jetson-sensors.service)

Parameters
  stacks          (str[])    stack names to manage   [sensors, i2c]
  <name>.unit     (str)      systemd unit to control (default per built-in map)
  <name>.members  (str[])    member units for status (default: [unit])
  status_period   (float, s) status poll period      [1.0]
"""

import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSReliabilityPolicy)
from std_msgs.msg import String
from std_srvs.srv import Trigger

# name -> (control unit, [member units polled for status])
DEFAULT_STACKS = {
    'sensors': ('rescue-sensors.target', ['zed.service', 'lidar.service']),
    'i2c': ('jetson-sensors.service', ['jetson-sensors.service']),
    'mapping': ('rescue-mapping.service', ['rescue-mapping.service']),
    'mapping3d': ('rescue-mapping3d.service', ['rescue-mapping3d.service']),
}


class Stack:
    def __init__(self, name, unit, members):
        self.name = name
        self.unit = unit
        self.members = members
        self.last_status = None
        self.status_pub = None


class RobotManager(Node):
    def __init__(self):
        super().__init__('robot_manager')

        self.declare_parameter('stacks', ['sensors', 'i2c', 'mapping', 'mapping3d'])
        self.declare_parameter('status_period', 0.5)   # 2 Hz: snappier GUI labels
        names = list(self.get_parameter('stacks').value)
        period = float(self.get_parameter('status_period').value)

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self._stacks = []
        for name in names:
            d_unit, d_members = DEFAULT_STACKS.get(name, ('', []))
            self.declare_parameter(f'{name}.unit', d_unit)
            self.declare_parameter(f'{name}.members', d_members)
            unit = str(self.get_parameter(f'{name}.unit').value)
            members = list(self.get_parameter(f'{name}.members').value) or [unit]
            if not unit:
                self.get_logger().error(
                    f"stack '{name}' has no '.unit' parameter — skipping")
                continue

            stack = Stack(name, unit, members)
            stack.status_pub = self.create_publisher(
                String, f'/robot/{name}/status', latched)
            # default-arg binding so each closure captures its own stack
            self.create_service(Trigger, f'/robot/{name}/start',
                                lambda req, resp, s=stack: self._action('start', s, resp))
            self.create_service(Trigger, f'/robot/{name}/stop',
                                lambda req, resp, s=stack: self._action('stop', s, resp))
            self.create_service(Trigger, f'/robot/{name}/restart',
                                lambda req, resp, s=stack: self._action('restart', s, resp))
            self._stacks.append(stack)
            self.get_logger().info(
                f"stack '{name}' -> {unit} (members: {members})")

        # Publish each stack's status right now (latched) so a GUI that connects
        # later gets the state immediately instead of waiting for the first tick.
        self._publish_all()
        self.create_timer(max(0.2, period), self._publish_all)
        self.get_logger().info(
            f"robot_manager ready — managing {len(self._stacks)} stack(s)")

    # ── systemctl helpers ────────────────────────────────────────────────────
    def _systemctl(self, *args, timeout=15.0):
        cmd = ['systemctl', '--user', *args]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
            return r.returncode, (r.stdout + r.stderr).strip()
        except Exception as e:  # noqa: BLE001 - surface any failure to the caller
            return 1, str(e)

    def _action(self, verb, stack, resp):
        # --no-block returns immediately; /robot/<name>/status then reflects
        # activating -> active (or failed), so the GUI never blocks on a slow start.
        rc, out = self._systemctl(verb, '--no-block', stack.unit)
        resp.success = (rc == 0)
        resp.message = (f"{verb} {stack.unit}: "
                        f"{'requested' if resp.success else 'FAILED'} {out}").strip()
        (self.get_logger().info if resp.success else self.get_logger().error)(resp.message)
        self._publish_stack(stack)
        return resp

    # ── status polling ───────────────────────────────────────────────────────
    def _unit_state(self, unit):
        _, out = self._systemctl('is-active', unit, timeout=5.0)
        return out.splitlines()[0] if out else 'unknown'

    def _publish_stack(self, stack):
        states = [self._unit_state(u) for u in stack.members]
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
        detail = ' '.join(f"{u.split('.')[0]}={s}" for u, s in zip(stack.members, states))
        msg = f"{overall} ({detail})"
        if msg != stack.last_status:
            stack.last_status = msg
            stack.status_pub.publish(String(data=msg))

    def _publish_all(self):
        for stack in self._stacks:
            self._publish_stack(stack)


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
