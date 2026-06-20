#!/usr/bin/env python3
"""
SVD Selectively-Damped Servo
============================
Real-time Cartesian -> joint differential IK for the dicerox 6R arm, built to be
robust through singularities where both MoveIt Servo and the legacy isotropic
DLS (`damped_servo.py`) misbehave.

Why not those two
-----------------
* Isotropic DLS  dq = J^T (J J^T + λ²I)^-1 dx  damps *every* Cartesian direction
  equally, so near a singularity it bends the end-effector off the commanded
  direction and slows the healthy axes too. Its trigger (condition number, a
  unitless ratio) fires on merely-extended poses.
* MoveIt Servo uses the same ratio metric AND hard-stops at its threshold — the
  arm freezes mid-motion, which is the unusable part.

The solver here
---------------
Selectively-Damped Least Squares via the SVD. Each cycle:

    J = U Σ V^T                         (6x6, base frame, unit-weighted)
    dq = Σ_i  f_i (u_i · dx) v_i ,      f_i = σ_i / (σ_i² + λ_i²)

with λ_i = 0 while σ_i is healthy and ramping to λ_max only as σ_i → 0:

    λ_i² = (1 - (σ_i/σ0)²) · λ_max²     for σ_i < σ0 ,  else 0

So the 5 well-conditioned directions are tracked *exactly* (f_i = 1/σ_i) and
only the collapsing direction is damped — f_i → 0 as σ_i → 0, so dq stays
bounded and the EE direction is preserved. It never halts and never explodes,
and it keys off the *absolute* smallest singular value (the true proximity to
singularity), not a ratio.

Units: the linear Jacobian rows are in metres and the angular rows are unitless,
so a raw SVD mixes them. We weight the angular rows by a characteristic length
`char_length` (and dx likewise) so σ0 means the same thing for an elbow/shoulder
(position) singularity and a wrist (orientation) singularity.

Output: the integrated joint command is published as sensor_msgs/JointState on
`/joint_states` (radians) — exactly what esp32_bridge forwards to the ESP32 as
MSG_ARM_JOINTS, and what the GUI digital twin renders. Open-loop integration of
the command (seeded from `initial_positions`) is intentional: the firmware path
is position-command and arm telemetry is too slow/jittery to close the loop on.
"""

from __future__ import annotations

import os
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_msgs.msg import Int8, Bool
from std_srvs.srv import Trigger

from arm_teleop.arm_kinematics import ArmKinematics

# Status codes — compatible with the keyboard/joystick teleop nodes.
STATUS_OK = 0
STATUS_DECEL = 1       # near a singularity, degrading tracking in one direction
STATUS_HALT = 2        # never emitted — this servo does not halt
STATUS_LEAVING = 3
STATUS_COLLISION = 4   # link pair within the collision slow/stop band
STATUS_JLIMIT = 5


def _skew(w):
    return np.array([[0.0, -w[2], w[1]],
                     [w[2], 0.0, -w[0]],
                     [-w[1], w[0], 0.0]])


def _exp_so3(w):
    """Rotation vector (axis·angle) -> 3x3 rotation matrix."""
    th = float(np.linalg.norm(w))
    if th < 1e-9:
        return np.eye(3) + _skew(w)
    k = w / th
    K = _skew(k)
    return np.eye(3) + np.sin(th) * K + (1.0 - np.cos(th)) * (K @ K)


def _log_so3(R):
    """3x3 rotation matrix -> rotation vector (axis·angle)."""
    cos_th = max(-1.0, min(1.0, (np.trace(R) - 1.0) * 0.5))
    th = np.arccos(cos_th)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if th < 1e-9:
        return 0.5 * v
    return (th / (2.0 * np.sin(th))) * v


class SdlsServo(Node):

    def __init__(self):
        super().__init__('servo_node')   # same name as MoveIt Servo → same ~/topics

        # ── parameters ────────────────────────────────────────────────
        self.declare_parameter('robot_description', '')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_frame', 'Link6')
        self.declare_parameter('joint_names',
                               ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'])
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('auto_start', True)

        # command scaling ([-1,1] stick → physical velocity)
        self.declare_parameter('max_linear_speed', 0.25)     # m/s at full deflection
        self.declare_parameter('max_rotational_speed', 0.6)  # rad/s at full deflection
        self.declare_parameter('max_joint_speed', 1.0)       # rad/s ceiling per joint

        # selectively-damped solver
        self.declare_parameter('sigma_threshold', 0.10)      # σ0: damping onset
        self.declare_parameter('lambda_max', 0.04)           # peak per-mode damping
        self.declare_parameter('char_length', 0.25)          # m: angular-row weight

        # safety / feel
        self.declare_parameter('joint_limit_margin', 0.05)       # rad hard stop-out
        self.declare_parameter('joint_limit_scale_zone', 0.20)   # rad soft slow-down
        self.declare_parameter('command_timeout', 0.2)           # s → zero command
        self.declare_parameter('smoothing_alpha', 1.0)           # input EMA (1=off)

        # mesh-on-mesh self-collision avoidance (FCL on low-poly link meshes).
        # Look-ahead damper: any step that pushes an already-close pair closer is
        # scaled (slow band) or vetoed (stop band); retreating is always allowed.
        self.declare_parameter('collision_check', True)          # master enable
        self.declare_parameter('collision_mesh_dir', '')         # '' → pkg collision_lowpoly
        self.declare_parameter('collision_slow_dist', 0.06)      # m: start damping
        self.declare_parameter('collision_stop_dist', 0.02)      # m: veto approach
        self.declare_parameter('collision_acm_samples', 2000)    # ACM build samples

        # closed-loop pose tracking (kills drift, esp. orientation under pure
        # translation): hold the integral of the commanded twist and correct back
        self.declare_parameter('pose_feedback', True)
        self.declare_parameter('k_pos', 5.0)        # 1/s position correction gain
        self.declare_parameter('k_ori', 5.0)        # 1/s orientation correction gain
        self.declare_parameter('max_pos_error', 0.05)   # m: anti-windup leash
        self.declare_parameter('max_ori_error', 0.20)   # rad: anti-windup leash

        # Safety: halt servo output while the firmware reports a latched arm fault
        # (esp32_bridge publishes /arm/fault). Off by default for software-only
        # twin testing where no bridge is running.
        self.declare_parameter('respect_fault', False)
        self.declare_parameter('fault_topic', '/arm/fault')
        # Default seed is a well-conditioned "ready" pose (σ_min≈0.12); the arm's
        # all-zeros pose is an exact singularity, so do NOT default to zeros.
        self.declare_parameter('initial_positions',
                               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        urdf = self.get_parameter('robot_description').value
        if not urdf:
            self.get_logger().fatal('robot_description parameter is empty')
            raise RuntimeError('robot_description not set')

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.ee_frame = str(self.get_parameter('ee_frame').value)
        self.joint_names = [str(n) for n in self.get_parameter('joint_names').value]
        self.rate = float(self.get_parameter('publish_rate').value)
        self.max_lin = float(self.get_parameter('max_linear_speed').value)
        self.max_rot = float(self.get_parameter('max_rotational_speed').value)
        self.max_jvel = float(self.get_parameter('max_joint_speed').value)
        self.sigma0 = float(self.get_parameter('sigma_threshold').value)
        self.lambda_max = float(self.get_parameter('lambda_max').value)
        self.L = float(self.get_parameter('char_length').value)
        self.jl_margin = float(self.get_parameter('joint_limit_margin').value)
        self.jl_zone = float(self.get_parameter('joint_limit_scale_zone').value)
        self.cmd_timeout = float(self.get_parameter('command_timeout').value)
        self.alpha = float(self.get_parameter('smoothing_alpha').value)
        self.pose_feedback = bool(self.get_parameter('pose_feedback').value)
        self.k_pos = float(self.get_parameter('k_pos').value)
        self.k_ori = float(self.get_parameter('k_ori').value)
        self.max_pos_err = float(self.get_parameter('max_pos_error').value)
        self.max_ori_err = float(self.get_parameter('max_ori_error').value)
        self.respect_fault = bool(self.get_parameter('respect_fault').value)
        fault_topic = str(self.get_parameter('fault_topic').value)
        init = [float(v) for v in self.get_parameter('initial_positions').value]
        self.coll_slow = float(self.get_parameter('collision_slow_dist').value)
        self.coll_stop = float(self.get_parameter('collision_stop_dist').value)

        # ── kinematics ────────────────────────────────────────────────
        self.kin = ArmKinematics(urdf, base=self.base_frame, tip=self.ee_frame)
        self.n = self.kin.n_act
        if self.n != len(self.joint_names):
            raise RuntimeError(
                f'URDF has {self.n} actuated joints but joint_names has '
                f'{len(self.joint_names)}: {self.kin.joint_names} vs {self.joint_names}')
        if len(init) != self.n:
            init = [0.0] * self.n

        # task-space weight W = diag(1,1,1, L,L,L) for unit consistency
        self._W = np.diag([1.0, 1.0, 1.0, self.L, self.L, self.L])

        # ── self-collision checker (optional) ─────────────────────────
        self.collision = self._init_collision()

        # ── state (guarded by _lock) ──────────────────────────────────
        self._lock = threading.Lock()
        self._q = np.clip(np.array(init), self.kin.lower, self.kin.upper)
        self._twist = np.zeros(6)
        self._twist_frame = self.base_frame
        self._filtered = np.zeros(6)
        # reference pose for closed-loop tracking (seeded lazily from FK)
        self._ref_valid = False
        self._p_ref = np.zeros(3)
        self._R_ref = np.eye(3)
        self._joint_jog = None              # (index, normalized_velocity) or None
        self._last_cmd = self.get_clock().now()
        self._running = bool(self.get_parameter('auto_start').value)
        self._last_status = -1
        self._faulted = False               # set from /arm/fault when respect_fault

        # ── interfaces ────────────────────────────────────────────────
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.status_pub = self.create_publisher(Int8, '~/status', 10)
        self.create_subscription(TwistStamped, '~/delta_twist_cmds', self._twist_cb, 10)
        self.create_subscription(JointJog, '~/delta_joint_cmds', self._jog_cb, 10)
        if self.respect_fault:
            self.create_subscription(Bool, fault_topic, self._fault_cb, 10)

        cbg = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/start_servo', self._srv_start, callback_group=cbg)
        self.create_service(Trigger, '~/pause_servo', self._srv_pause, callback_group=cbg)

        self.create_timer(1.0 / self.rate, self._loop)

        self.get_logger().info(
            f'SDLS servo ready — {self.n} joints, {self.rate:.0f} Hz, '
            f'σ0={self.sigma0} λ_max={self.lambda_max} L={self.L} '
            f'running={self._running}')

    # ── self-collision setup ─────────────────────────────────────────

    def _init_collision(self):
        """Build the FCL self-collision checker, or return None (disabled).

        Disabled cleanly (warn, not crash) when the master flag is off, the
        deps are missing, or the meshes aren't found — the servo then behaves
        exactly as before (joint-limit + singularity safety only).
        """
        if not bool(self.get_parameter('collision_check').value):
            self.get_logger().info('self-collision check DISABLED (collision_check=false)')
            return None
        try:
            from arm_teleop.self_collision import SelfCollision
        except Exception as exc:
            self.get_logger().warn(f'self-collision DISABLED — import failed: {exc}')
            return None

        mesh_dir = str(self.get_parameter('collision_mesh_dir').value)
        if not mesh_dir:
            try:
                from ament_index_python.packages import get_package_share_directory
                mesh_dir = os.path.join(get_package_share_directory('arm_teleop'),
                                        'meshes', 'collision_lowpoly')
            except Exception as exc:
                self.get_logger().warn(f'self-collision DISABLED — no mesh dir: {exc}')
                return None

        n = int(self.get_parameter('collision_acm_samples').value)
        try:
            t0 = time.time()
            checker = SelfCollision(self.kin, mesh_dir, self.kin.child_links,
                                    acm_samples=n, near_band=self.coll_slow + 0.02,
                                    logger=self.get_logger().info)
            self.get_logger().info(
                f'self-collision ENABLED — {len(checker.pairs)} active pairs, '
                f'slow<{self.coll_slow}m stop<{self.coll_stop}m, '
                f'ACM built in {time.time() - t0:.1f}s')
            return checker
        except Exception as exc:
            self.get_logger().warn(f'self-collision DISABLED — init failed: {exc}')
            return None

    # ── callbacks ─────────────────────────────────────────────────────

    def _twist_cb(self, msg: TwistStamped):
        t = msg.twist
        with self._lock:
            self._twist = np.array([t.linear.x, t.linear.y, t.linear.z,
                                    t.angular.x, t.angular.y, t.angular.z])
            self._twist_frame = msg.header.frame_id or self.base_frame
            self._joint_jog = None
            self._last_cmd = self.get_clock().now()

    def _fault_cb(self, msg: Bool):
        if bool(msg.data) != self._faulted:
            self.get_logger().warn('arm FAULT — pausing servo output'
                                   if msg.data else 'arm fault cleared — resuming')
            with self._lock:
                self._ref_valid = False
        self._faulted = bool(msg.data)

    def _jog_cb(self, msg: JointJog):
        if not msg.joint_names or not msg.velocities:
            return
        name = msg.joint_names[0]
        if name in self.joint_names:
            with self._lock:
                self._joint_jog = (self.joint_names.index(name), float(msg.velocities[0]))
                self._twist[:] = 0.0
                self._last_cmd = self.get_clock().now()

    def _srv_start(self, _req, resp):
        with self._lock:
            self._running = True
            self._filtered[:] = 0.0
            self._ref_valid = False
        resp.success, resp.message = True, 'SDLS servo started'
        self.get_logger().info('Servo STARTED')
        return resp

    def _srv_pause(self, _req, resp):
        with self._lock:
            self._running = False
            self._ref_valid = False
        resp.success, resp.message = True, 'SDLS servo paused'
        self.get_logger().info('Servo PAUSED')
        return resp

    # ── control loop ──────────────────────────────────────────────────

    def _loop(self):
        if self.respect_fault and self._faulted:
            return                     # arm faulted/disarmed → hold, publish nothing
        with self._lock:
            if not self._running:
                return
            q = self._q.copy()
            twist = self._twist.copy()
            frame = self._twist_frame
            jog = self._joint_jog
            cmd_time = self._last_cmd

        dt = 1.0 / self.rate
        age = (self.get_clock().now() - cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            twist[:] = 0.0
            jog = None

        if jog is not None:
            dq, status = self._joint_jog_step(q, jog, dt)
        else:
            dq, status = self._cartesian_step(q, twist, frame, dt)

        # mesh-on-mesh self-collision gate (skipped when idle → no FCL cost)
        if self.collision is not None and np.any(dq):
            dq, hit = self._collision_filter(q, dq)
            if hit:
                status = STATUS_COLLISION

        # hard joint-limit clamp
        q_new = np.clip(q + dq, self.kin.lower + self.jl_margin,
                        self.kin.upper - self.jl_margin)

        with self._lock:
            self._q = q_new
        self._publish(q_new, (q_new - q) / dt if dt > 0 else np.zeros(self.n))
        self._publish_status(status)

    # ── self-collision look-ahead damper ──────────────────────────────

    def _collision_filter(self, q, dq):
        """Scale/veto `dq` so no link pair is driven into another.

        Compares per-pair clearance now (`q`) vs after the candidate step
        (`q + dq`). A pair only constrains the step when the step *reduces* its
        clearance ("approaching"); a step that increases clearance is always
        allowed, so the operator can always drive out of a near-collision.

          clearance after step <= stop band  → veto (hold this tick)
          clearance after step  < slow band  → scale dq toward zero linearly

        Returns (dq, hit) where `hit` is True if the step was scaled or vetoed.
        """
        d_now = self.collision.distances(q)
        d_next = self.collision.distances(q + dq)
        scale = 1.0
        hit = False
        for pair, dn in d_next.items():
            if dn >= d_now.get(pair, float('inf')):
                continue                       # retreating / parallel → unconstrained
            if dn <= self.coll_stop:
                return np.zeros_like(dq), True  # entering stop band while approaching
            if dn < self.coll_slow:
                f = (dn - self.coll_stop) / (self.coll_slow - self.coll_stop)
                scale = min(scale, max(f, 0.0))
                hit = True
        return dq * scale, hit

    # ── joint-space jog (no Jacobian, no singularity) ─────────────────

    def _joint_jog_step(self, q, jog, dt):
        self._ref_valid = False   # re-seed the Cartesian reference after jogging
        idx, vel = jog
        dq = np.zeros(self.n)
        dq[idx] = np.clip(vel, -1.0, 1.0) * self.max_jvel * dt
        status = STATUS_OK
        nxt = q[idx] + dq[idx]
        if nxt < self.kin.lower[idx] + self.jl_margin or \
           nxt > self.kin.upper[idx] - self.jl_margin:
            status = STATUS_JLIMIT
        return dq, status

    # ── Cartesian step via selectively-damped least squares ───────────

    def _cartesian_step(self, q, twist, frame, dt):
        # optional input smoothing (alpha=1 → passthrough)
        self._filtered = self.alpha * twist + (1.0 - self.alpha) * self._filtered
        cmd = self._filtered

        v_cmd = np.empty(6)
        v_cmd[:3] = np.clip(cmd[:3], -1.0, 1.0) * self.max_lin   # m/s, base frame
        v_cmd[3:] = np.clip(cmd[3:], -1.0, 1.0) * self.max_rot   # rad/s

        T = self.kin.fk(q)[0]
        p_cur, R_cur = T[:3, 3], T[:3, :3]

        # express an EE-frame twist in the base frame
        if frame == self.ee_frame:
            v_cmd[:3] = R_cur @ v_cmd[:3]
            v_cmd[3:] = R_cur @ v_cmd[3:]

        # ── closed-loop pose tracking (CLIK) ────────────────────────────────
        # Open-loop velocity integration drifts: commanding zero angular velocity
        # holds orientation only to first order, so the EE slowly tilts during a
        # pure translation. Here we keep a reference pose that integrates the
        # *commanded* twist exactly and feed back the pose error, so any drift
        # (position or orientation) is actively corrected. An anti-windup leash
        # keeps the reference within reach so singularities/limits stay stable.
        v = v_cmd.copy()
        if self.pose_feedback:
            if not self._ref_valid:
                self._p_ref, self._R_ref, self._ref_valid = p_cur.copy(), R_cur.copy(), True
            self._p_ref = self._p_ref + v_cmd[:3] * dt
            self._R_ref = _exp_so3(v_cmd[3:] * dt) @ self._R_ref

            e_pos = self._p_ref - p_cur
            npos = float(np.linalg.norm(e_pos))
            if npos > self.max_pos_err:
                self._p_ref = p_cur + e_pos * (self.max_pos_err / npos)
                e_pos = self._p_ref - p_cur
            e_ori = _log_so3(self._R_ref @ R_cur.T)
            nori = float(np.linalg.norm(e_ori))
            if nori > self.max_ori_err:
                e_ori = e_ori * (self.max_ori_err / nori)
                self._R_ref = _exp_so3(e_ori) @ R_cur

            v[:3] = v_cmd[:3] + self.k_pos * e_pos
            v[3:] = v_cmd[3:] + self.k_ori * e_ori

        if not np.any(v):
            return np.zeros(self.n), STATUS_OK

        # unit-weighted Jacobian + task velocity
        Jw = self._W @ self.kin.jacobian(q)
        dxw = self._W @ v

        U, S, Vt = np.linalg.svd(Jw, full_matrices=False)

        # per-mode selective damping
        dq = np.zeros(self.n)
        for i in range(S.shape[0]):
            s = S[i]
            if s >= self.sigma0:
                lam2 = 0.0
            else:
                r = s / self.sigma0 if self.sigma0 > 0 else 0.0
                lam2 = (1.0 - r * r) * self.lambda_max ** 2
            f = s / (s * s + lam2) if (s * s + lam2) > 1e-12 else 0.0
            dq += f * float(U[:, i] @ dxw) * Vt[i, :]

        status = STATUS_DECEL if S[-1] < self.sigma0 else STATUS_OK

        # direction-preserving velocity clamp (uniform scale, never per-joint here)
        peak = float(np.max(np.abs(dq))) if self.n else 0.0
        if peak > self.max_jvel:
            dq *= self.max_jvel / peak

        # soft slow-down approaching joint limits
        for i in range(self.n):
            lo = q[i] - self.kin.lower[i]
            hi = self.kin.upper[i] - q[i]
            if dq[i] < 0 and lo < self.jl_zone:
                dq[i] *= max(lo / self.jl_zone, 0.0)
                if lo < self.jl_margin:
                    status = STATUS_JLIMIT
            elif dq[i] > 0 and hi < self.jl_zone:
                dq[i] *= max(hi / self.jl_zone, 0.0)
                if hi < self.jl_margin:
                    status = STATUS_JLIMIT

        return dq * dt, status

    # ── publishing ────────────────────────────────────────────────────

    def _publish(self, q, qd):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = q.tolist()
        msg.velocity = qd.tolist()
        self.js_pub.publish(msg)

    def _publish_status(self, code):
        if code == self._last_status:
            return
        self._last_status = code
        m = Int8()
        m.data = int(code)
        self.status_pub.publish(m)


def main():
    rclpy.init()
    node = SdlsServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
