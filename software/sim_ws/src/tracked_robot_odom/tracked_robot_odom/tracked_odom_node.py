"""
tracked_odom_node.py — Planar tracked-robot odometry for Stage A.

Model
-----
State:   x, y, yaw
Inputs:  v_left, v_right  (left/right track linear velocities)

Kinematic equations:
    v     = (v_right + v_left)  / 2
    omega = (v_right - v_left)  / track_separation
    x_dot   = v * cos(yaw)
    y_dot   = v * sin(yaw)
    yaw_dot = omega

cmd_vel → track speeds:
    v_left  = linear_x  -  angular_z * track_separation / 2
    v_right = linear_x  +  angular_z * track_separation / 2

Publishes
---------
/odom  (nav_msgs/Odometry)          — consumed by Nav2 / AMCL
TF odom → base_footprint            — consumed by Nav2 / AMCL

STAGE A NOTE:
  Odometry is computed entirely from /cmd_vel integration.  There is no
  encoder feedback from the simulation.  This means odometry accumulates
  drift over time, which is exactly what AMCL is there to correct.
  In Stage B, replace the cmd_vel integration with actual wheel encoder
  counts bridged from Gazebo (or a hardware driver).

Parameters (set in launch file or via --ros-args -p):
  track_separation   (float, default 0.530)  — distance between track CL in m
  publish_rate       (float, default 50.0)   — Hz for the odom publisher
  odom_frame         (str,   default "odom")
  base_frame         (str,   default "base_footprint")
  use_sim_time       (bool,  default True)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class TrackedOdomNode(Node):

    def __init__(self):
        super().__init__('tracked_odom_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('track_separation', 0.530)
        self.declare_parameter('publish_rate',     50.0)
        self.declare_parameter('odom_frame',       'odom')
        self.declare_parameter('base_frame',       'base_footprint')

        self.track_sep   = self.get_parameter('track_separation').value
        self.rate_hz     = self.get_parameter('publish_rate').value
        self.odom_frame  = self.get_parameter('odom_frame').value
        self.base_frame  = self.get_parameter('base_frame').value

        # ── State ─────────────────────────────────────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        # Track velocities (m/s)
        self.v_left  = 0.0
        self.v_right = 0.0

        # Last integration timestamp (use sim time)
        self._last_time: Time | None = None

        # ── I/O ───────────────────────────────────────────────────────────
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10,
        )

        # ── Timer for odometry integration + publish ───────────────────────
        self._timer = self.create_timer(
            1.0 / self.rate_hz,
            self._integrate_and_publish,
        )

        self.get_logger().info(
            f'TrackedOdomNode started. '
            f'track_separation={self.track_sep:.3f} m, '
            f'rate={self.rate_hz:.0f} Hz, '
            f'frames: {self.odom_frame} → {self.base_frame}'
        )

    # ── cmd_vel callback ──────────────────────────────────────────────────
    def _cmd_vel_callback(self, msg: Twist):
        """
        Convert Twist to individual track speeds.

        Stage A: simple kinematic inversion (no slip, no dynamics).
        Replace with a proper track-speed controller in Stage B.
        """
        linear  = msg.linear.x
        angular = msg.angular.z

        # Skid-steer / differential track model
        self.v_left  = linear - angular * (self.track_sep / 2.0)
        self.v_right = linear + angular * (self.track_sep / 2.0)

    # ── Integration timer callback ────────────────────────────────────────
    def _integrate_and_publish(self):
        now = self.get_clock().now()

        # Guard against publishing wall-clock-stamped transforms.
        # Under use_sim_time the ROS clock briefly reports wall time
        # (~1.78e9 s) before the first /clock message arrives — Gazebo can be
        # slow to start publishing /clock, especially under software
        # rendering. A single TF stamped at wall time permanently poisons the
        # tf2 buffer: because wall time is decades ahead of sim time, every
        # later sim-time transform is rejected as TF_OLD_DATA and Nav2's
        # base_footprint→map lookups fail for the whole session.
        # 1e9 s ≙ 1e18 ns: sim time stays far below this; wall time exceeds it.
        if now.nanoseconds > 1_000_000_000_000_000_000:
            self._last_time = None
            return

        if self._last_time is None:
            self._last_time = now
            return

        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        if dt <= 0.0:
            return

        # ── Kinematic integration ─────────────────────────────────────────
        # Average linear velocity and angular velocity
        v     = (self.v_right + self.v_left)  / 2.0
        omega = (self.v_right - self.v_left)  / self.track_sep

        # Euler integration of planar pose
        # (replace with Runge-Kutta or unicycle exact-integration in Stage B)
        delta_x   = v * math.cos(self.yaw) * dt
        delta_y   = v * math.sin(self.yaw) * dt
        delta_yaw = omega * dt

        self.x   += delta_x
        self.y   += delta_y
        self.yaw += delta_yaw

        # Normalise yaw to (-pi, pi]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # ── Publish Odometry ──────────────────────────────────────────────
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Quaternion from yaw
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Pose covariance (diagonal, conservative estimates for Stage A)
        # Row-major 6×6: [xx, xy, xz, xroll, xpitch, xyaw, yx, ...]
        odom.pose.covariance[0]  = 0.05   # xx
        odom.pose.covariance[7]  = 0.05   # yy
        odom.pose.covariance[14] = 1e6    # zz   (planar robot, not estimated)
        odom.pose.covariance[21] = 1e6    # roll-roll
        odom.pose.covariance[28] = 1e6    # pitch-pitch
        odom.pose.covariance[35] = 0.10   # yaw-yaw

        # Velocity (in body frame)
        odom.twist.twist.linear.x  = v
        odom.twist.twist.linear.y  = 0.0
        odom.twist.twist.angular.z = omega

        odom.twist.covariance[0]  = 0.05
        odom.twist.covariance[7]  = 1e6
        odom.twist.covariance[14] = 1e6
        odom.twist.covariance[21] = 1e6
        odom.twist.covariance[28] = 1e6
        odom.twist.covariance[35] = 0.10

        self._odom_pub.publish(odom)

        # ── Publish TF odom → base_footprint ──────────────────────────────
        tf = TransformStamped()
        tf.header.stamp    = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id  = self.base_frame

        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x    = 0.0
        tf.transform.rotation.y    = 0.0
        tf.transform.rotation.z    = qz
        tf.transform.rotation.w    = qw

        self._tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = TrackedOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
