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
import yaml
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, PoseStamped, Pose
from control_msgs.msg import JointJog
from std_msgs.msg import Int8, Bool, String
from std_srvs.srv import Trigger
from rescue_interfaces.srv import SavePose, GoToPose, DeletePose, ListPoses

from arm_teleop.arm_kinematics import ArmKinematics
from arm_teleop import arm_planner
from arm_teleop.arm_ik import AnalyticIK, NotSphericalWrist

# Status codes — compatible with the keyboard/joystick teleop nodes.
STATUS_OK = 0
STATUS_DECEL = 1       # near a singularity, degrading tracking in one direction
STATUS_HALT = 2        # never emitted — this servo does not halt
STATUS_LEAVING = 3
STATUS_COLLISION = 4   # link pair within the collision slow/stop band
STATUS_JLIMIT = 5
STATUS_MOVING = 6      # executing a planned go-to-pose path
STATUS_REACHED = 7     # planned motion finished at the target
STATUS_PLAN_FAILED = 8 # IK/RRT failed; no motion started


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


def _quat_from_R(R):
    """3x3 rotation matrix -> quaternion (x, y, z, w)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def _R_from_quat(x, y, z, w):
    """Quaternion (x, y, z, w) -> 3x3 rotation matrix (normalised first)."""
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def _pose_to_T(pose: Pose) -> np.ndarray:
    """geometry_msgs/Pose -> 4x4 homogeneous transform."""
    T = np.eye(4)
    q = pose.orientation
    T[:3, :3] = _R_from_quat(q.x, q.y, q.z, q.w)
    T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return T


def _T_to_pose(T: np.ndarray) -> Pose:
    """4x4 homogeneous transform -> geometry_msgs/Pose."""
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in T[:3, 3])
    qx, qy, qz, qw = _quat_from_R(T[:3, :3])
    p.orientation.x, p.orientation.y = float(qx), float(qy)
    p.orientation.z, p.orientation.w = float(qz), float(qw)
    return p


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
        self.declare_parameter('sigma_threshold', 0.05)      # σ0: damping onset
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
        self.declare_parameter('collision_mesh_dir', '')         # '' → pkg collision
        self.declare_parameter('collision_slow_dist', 0.06)      # m: start damping
        self.declare_parameter('collision_stop_dist', 0.02)      # m: veto approach
        self.declare_parameter('collision_acm_samples', 2000)    # ACM build samples

        # arm-vs-body avoidance (chassis + the 4 flippers) — same look-ahead
        # damper, obstacle set read from the combined URDF; flippers posed live
        # from /encoders/flipper (so a raised flipper actually blocks the arm).
        self.declare_parameter('body_collision', True)           # master enable
        self.declare_parameter('body_collision_mesh_dir', '')    # '' → arm_description collision
        self.declare_parameter('body_collision_slow_dist', 0.06)  # m: start damping
        self.declare_parameter('body_collision_stop_dist', 0.025)  # m: veto approach
        self.declare_parameter('body_collision_acm_samples', 1500)  # ACM build samples
        self.declare_parameter('flipper_sample_deg', 90.0)       # ± range sampled for the ACM
        self.declare_parameter('flipper_state_topic', '/joint_states')  # source of live flipper angles

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

        # ── Saved poses: store/recall EE poses, return via planned motion ──
        # Library file (name → EE pose + joint snapshot). save_pose snapshots the
        # current EE pose; go_to_pose re-solves IK (seeded from the saved joints)
        # then RRT-plans a collision-free path and executes it through this loop.
        self.declare_parameter('pose_library_path', '')   # '' → ~/.config/robocorea_arm/poses.yaml
        self.declare_parameter('goto_reach_tol', 0.01)     # rad: waypoint-reached tolerance
        # Inverse kinematics (damped least squares, seeded)
        self.declare_parameter('ik_pos_tol', 0.002)        # m
        self.declare_parameter('ik_ori_tol', 0.02)         # rad
        self.declare_parameter('ik_max_iters', 200)
        self.declare_parameter('ik_lambda', 0.05)          # DLS damping
        # RRT-Connect path planner
        self.declare_parameter('rrt_step', 0.10)           # rad: extension step
        self.declare_parameter('rrt_resolution', 0.05)     # rad: edge collision-check step
        self.declare_parameter('rrt_max_time', 5.0)        # s: planning budget
        self.declare_parameter('rrt_max_nodes', 5000)
        self.declare_parameter('rrt_seed', 0)              # RNG seed (repeatable plans)
        self.declare_parameter('plan_collision_margin', 0.02)  # m: clearance the planner keeps

        # ── Resolved-pose (analytic-IK) servo mode ─────────────────────
        # Off by default: the differential SDLS step is the proven, singularity-
        # robust teleop core. When on, each tick solves closed-form IK for the
        # integrated reference pose (nearest branch) and falls back to the SDLS
        # step near singularities / on a branch switch, so robustness is kept.
        self.declare_parameter('ik_servo', False)
        self.declare_parameter('ik_servo_sigma_min', 0.002)  # σ_min (geometric J) below → SDLS
        self.declare_parameter('ik_servo_max_jump', 0.5)     # rad: jump above → use SDLS

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
        self.body_coll_slow = float(self.get_parameter('body_collision_slow_dist').value)
        self.body_coll_stop = float(self.get_parameter('body_collision_stop_dist').value)

        # saved-pose / planner params
        self.goto_reach_tol = float(self.get_parameter('goto_reach_tol').value)
        self.ik_pos_tol = float(self.get_parameter('ik_pos_tol').value)
        self.ik_ori_tol = float(self.get_parameter('ik_ori_tol').value)
        self.ik_max_iters = int(self.get_parameter('ik_max_iters').value)
        self.ik_lambda = float(self.get_parameter('ik_lambda').value)
        self.rrt_step = float(self.get_parameter('rrt_step').value)
        self.rrt_res = float(self.get_parameter('rrt_resolution').value)
        self.rrt_max_time = float(self.get_parameter('rrt_max_time').value)
        self.rrt_max_nodes = int(self.get_parameter('rrt_max_nodes').value)
        self.rrt_seed = int(self.get_parameter('rrt_seed').value)
        self.plan_margin = float(self.get_parameter('plan_collision_margin').value)
        self.ik_servo = bool(self.get_parameter('ik_servo').value)
        self.ik_sigma_min = float(self.get_parameter('ik_servo_sigma_min').value)
        self.ik_max_jump = float(self.get_parameter('ik_servo_max_jump').value)
        self._pose_lib_path = self._resolve_pose_library_path(
            str(self.get_parameter('pose_library_path').value))

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

        # ── closed-form decoupled IK (spherical wrist) ────────────────
        # Primary solver for go_to_pose goals and the optional resolved-pose
        # servo mode. None if the arm isn't a spherical-wrist 6R (→ numerical).
        try:
            self.aik = AnalyticIK(self.kin)
            self.get_logger().info('analytic IK ready (spherical wrist, ZYZ)')
        except NotSphericalWrist as exc:
            self.aik = None
            self.get_logger().warn(f'analytic IK disabled ({exc}); using numerical IK')

        # ── self-collision checker (optional) ─────────────────────────
        self.collision = self._init_collision()
        # ── arm-vs-body (chassis + flippers) checker (optional) ───────
        self.body_collision = self._init_body_collision(urdf)
        # live flipper angles (rad, URDF convention) fed into body_collision;
        # 0 = flippers level (a safe default until the first message arrives).
        self._flip_q = (np.zeros(len(self.body_collision.flip_joints))
                        if self.body_collision is not None else None)
        self._flip_name_idx = ({n: i for i, n in
                                enumerate(self.body_collision.flip_joints)}
                               if self.body_collision is not None else {})

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

        # planned go-to-pose execution (guarded by _lock): a list of joint
        # waypoints the loop streams into _q, plus the next-waypoint index.
        self._path = None
        self._path_idx = 0
        self._collision_lock = threading.Lock()  # serialise FCL access (loop ↔ planner)
        self._planning = False                   # one plan at a time
        self._ee_pub_div = max(1, int(round(self.rate / 20.0)))  # ee_pose ~20 Hz
        self._loop_count = 0

        # name → {'pose': geometry_msgs/Pose, 'joints': [..6]}
        self._poses = {}
        self._load_pose_library()

        # ── interfaces ────────────────────────────────────────────────
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.status_pub = self.create_publisher(Int8, '~/status', 10)
        self.ee_pose_pub = self.create_publisher(PoseStamped, '~/ee_pose', 10)
        # Latched: a late-joining GUI/CLI immediately sees the current plan state.
        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.plan_state_pub = self.create_publisher(String, '~/plan_state', latched)
        self.create_subscription(TwistStamped, '~/delta_twist_cmds', self._twist_cb, 10)
        self.create_subscription(JointJog, '~/delta_joint_cmds', self._jog_cb, 10)
        if self.respect_fault:
            self.create_subscription(Bool, fault_topic, self._fault_cb, 10)
        # Live flipper angles for the body-collision obstacle set. We read them
        # from /joint_states (the URDF-convention radians the flipper_state bridge
        # publishes), so the collision flippers match the digital twin exactly.
        if self.body_collision is not None:
            self.create_subscription(
                JointState, str(self.get_parameter('flipper_state_topic').value),
                self._flipper_state_cb, 10)

        cbg = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/start_servo', self._srv_start, callback_group=cbg)
        self.create_service(Trigger, '~/pause_servo', self._srv_pause, callback_group=cbg)
        self.create_service(SavePose, '~/save_pose', self._srv_save_pose, callback_group=cbg)
        self.create_service(GoToPose, '~/go_to_pose', self._srv_go_to_pose, callback_group=cbg)
        self.create_service(DeletePose, '~/delete_pose', self._srv_delete_pose, callback_group=cbg)
        self.create_service(ListPoses, '~/list_poses', self._srv_list_poses, callback_group=cbg)

        # The 100 Hz loop runs in its own group so a multi-second planning service
        # call (other group, MultiThreadedExecutor) never stalls /joint_states.
        self.create_timer(1.0 / self.rate, self._loop,
                          callback_group=MutuallyExclusiveCallbackGroup())

        self._publish_plan_state('idle')

        self.get_logger().info(
            f'SDLS servo ready — {self.n} joints, {self.rate:.0f} Hz, '
            f'σ0={self.sigma0} λ_max={self.lambda_max} L={self.L} '
            f'running={self._running}, {len(self._poses)} saved pose(s)')

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
                                        'meshes', 'collision')
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

    def _init_body_collision(self, urdf):
        """Build the FCL arm-vs-body checker, or return None (disabled).

        Needs the COMBINED URDF (arm + chassis + flippers) for the body
        transforms; if ``robot_description`` is the arm-only URDF (no
        chassis_base_link) this degrades cleanly to disabled.
        """
        if not bool(self.get_parameter('body_collision').value):
            self.get_logger().info('body-collision check DISABLED (body_collision=false)')
            return None
        try:
            from arm_teleop.body_collision import BodyCollision
        except Exception as exc:
            self.get_logger().warn(f'body-collision DISABLED — import failed: {exc}')
            return None

        from ament_index_python.packages import get_package_share_directory
        arm_dir = str(self.get_parameter('collision_mesh_dir').value)
        if not arm_dir:
            try:
                arm_dir = os.path.join(get_package_share_directory('arm_teleop'),
                                       'meshes', 'collision')
            except Exception as exc:
                self.get_logger().warn(f'body-collision DISABLED — no arm mesh dir: {exc}')
                return None
        body_dir = str(self.get_parameter('body_collision_mesh_dir').value)
        if not body_dir:
            try:
                body_dir = os.path.join(get_package_share_directory('arm_description'),
                                        'meshes', 'collision')
            except Exception as exc:
                self.get_logger().warn(f'body-collision DISABLED — no body mesh dir: {exc}')
                return None

        n = int(self.get_parameter('body_collision_acm_samples').value)
        fr = np.radians(float(self.get_parameter('flipper_sample_deg').value))
        try:
            t0 = time.time()
            checker = BodyCollision(
                self.kin, urdf, arm_dir, body_dir,
                acm_samples=n, near_band=self.body_coll_slow + 0.02,
                flipper_sample_rad=fr, logger=self.get_logger().info)
            self.get_logger().info(
                f'body-collision ENABLED — {len(checker.pairs)} active pairs, '
                f'slow<{self.body_coll_slow}m stop<{self.body_coll_stop}m, '
                f'ACM built in {time.time() - t0:.1f}s')
            return checker
        except Exception as exc:
            self.get_logger().warn(f'body-collision DISABLED — init failed: {exc}')
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

    def _flipper_state_cb(self, msg: JointState):
        """Track the live flipper angles so the body obstacle moves with them.

        Reads only the flipper joints out of /joint_states (ignores the arm
        joints we publish ourselves), then hands the angle vector to the
        body-collision checker.
        """
        changed = False
        for name, pos in zip(msg.name, msg.position):
            i = self._flip_name_idx.get(name)
            if i is not None:
                self._flip_q[i] = pos
                changed = True
        if changed:
            self.body_collision.set_flipper_angles(self._flip_q)

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
            self._abort_path('arm fault')  # hold; never auto-resume a planned move
            return                     # arm faulted/disarmed → hold, publish nothing
        with self._lock:
            if not self._running:
                return
            q = self._q.copy()
            twist = self._twist.copy()
            frame = self._twist_frame
            jog = self._joint_jog
            cmd_time = self._last_cmd
            path = self._path
            path_idx = self._path_idx

        dt = 1.0 / self.rate
        age = (self.get_clock().now() - cmd_time).nanoseconds * 1e-9
        # A fresh, *non-zero* manual command overrides (and aborts) a planned move.
        manual_active = age <= self.cmd_timeout and (np.any(twist) or jog is not None)

        if path is not None and manual_active:
            self._abort_path('operator override')
            path = None

        if path is not None:
            dq, status, new_idx, done = self._goto_step(q, path, path_idx, dt)
            with self._lock:
                self._path_idx = new_idx
            if done:
                self._finish_path()
        else:
            if age > self.cmd_timeout:
                twist[:] = 0.0
                jog = None
            if jog is not None:
                dq, status = self._joint_jog_step(q, jog, dt)
            else:
                dq, status = self._cartesian_step(q, twist, frame, dt)

        # mesh-on-mesh collision gate — self (arm-arm) + body (arm-chassis/
        # flippers). Skipped when idle → no FCL cost.
        if (self.collision is not None or self.body_collision is not None) \
                and np.any(dq):
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

        self._loop_count += 1
        if self._loop_count % self._ee_pub_div == 0:
            self._publish_ee_pose(q_new)

    # ── planned go-to-pose execution ──────────────────────────────────

    def _goto_step(self, q, path, idx, dt):
        """One tick of streaming a planned joint path into `_q`.

        Steps toward `path[idx]` at the per-joint speed ceiling; advances the
        index when the waypoint is reached. Returns (dq, status, new_idx, done).
        """
        max_step = self.max_jvel * dt
        target = path[idx]
        dq = target - q
        if float(np.max(np.abs(dq))) <= self.goto_reach_tol:
            idx += 1
            if idx >= len(path):
                return np.zeros(self.n), STATUS_REACHED, idx, True
            self._publish_plan_state(f'moving {idx}/{len(path) - 1}')
            target = path[idx]
            dq = target - q
        peak = float(np.max(np.abs(dq)))
        if peak > max_step:
            dq = dq * (max_step / peak)
        return dq, STATUS_MOVING, idx, False

    def _finish_path(self):
        with self._lock:
            self._path = None
            self._path_idx = 0
            self._ref_valid = False   # reseed CLIK reference for resumed teleop
        self._publish_plan_state('reached')
        self.get_logger().info('go-to-pose: target reached')

    def _abort_path(self, reason):
        with self._lock:
            if self._path is None:
                return
            self._path = None
            self._path_idx = 0
            self._ref_valid = False
        self._publish_plan_state(f'aborted ({reason})')
        self.get_logger().info(f'go-to-pose aborted: {reason}')

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
        The arm's own links (self-collision) and the body (chassis + flippers)
        are checked with their own slow/stop bands but the same approach logic.
        """
        q_next = q + dq
        with self._collision_lock:     # FCL objects are shared with the planner
            sources = []
            if self.collision is not None:
                sources.append((self.collision.distances(q),
                                self.collision.distances(q_next),
                                self.coll_slow, self.coll_stop))
            if self.body_collision is not None:
                sources.append((self.body_collision.distances(q),
                                self.body_collision.distances(q_next),
                                self.body_coll_slow, self.body_coll_stop))
        scale = 1.0
        hit = False
        for d_now, d_next, slow, stop in sources:
            for pair, dn in d_next.items():
                if dn >= d_now.get(pair, float('inf')):
                    continue                   # retreating / parallel → unconstrained
                if dn <= stop:
                    return np.zeros_like(dq), True  # entering stop band while approaching
                if dn < slow:
                    f = (dn - stop) / (slow - stop)
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
        # The integrated reference pose is needed by CLIK *and* by the resolved-
        # pose IK servo, so maintain it whenever either is active.
        ik_mode = self.ik_servo and self.aik is not None
        if self.pose_feedback or ik_mode:
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

            if self.pose_feedback:
                v[:3] = v_cmd[:3] + self.k_pos * e_pos
                v[3:] = v_cmd[3:] + self.k_ori * e_ori

        # ── resolved-pose IK servo (closed-form, with SDLS fallback) ────────
        # Drive joints straight to the IK solution of the reference pose. Defers
        # to the differential SDLS step (below) near singularities or on a branch
        # switch, so the singularity-robust behaviour is preserved there.
        if ik_mode and self._ref_valid and np.any(v_cmd):
            dq_ik = self._ik_servo_step(q, dt)
            if dq_ik is not None:
                return dq_ik, STATUS_OK

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

    def _ik_servo_step(self, q, dt):
        """One tick of resolved-pose IK servoing. Returns a dq, or None to defer.

        Returns None (→ the SDLS step handles this tick) near a Jacobian
        singularity, when closed-form IK finds no in-limits solution, or when the
        nearest solution would jump (branch switch) — so the differential solver's
        graceful singularity damping is preserved exactly where it matters.
        """
        # Defer near a genuine singularity (σ_min of the geometric Jacobian → 0,
        # e.g. wrist gimbal J5≈0) so SDLS handles it; normal configs sit well above.
        if float(np.linalg.svd(self.kin.jacobian(q), compute_uv=False)[-1]) < self.ik_sigma_min:
            return None
        T = np.eye(4)
        T[:3, :3] = self._R_ref
        T[:3, 3] = self._p_ref
        q_goal = self.aik.solve_nearest(T, q)
        if q_goal is None:
            return None
        dq = q_goal - q
        peak = float(np.max(np.abs(dq)))
        if peak > self.ik_max_jump:
            return None                       # discontinuity / branch switch → SDLS
        cap = self.max_jvel * dt
        if peak > cap:
            dq = dq * (cap / peak)
        return dq

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

    def _publish_ee_pose(self, q):
        T = self.kin.fk(q)[0]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose = _T_to_pose(T)
        self.ee_pose_pub.publish(msg)

    def _publish_plan_state(self, text):
        m = String()
        m.data = str(text)
        self.plan_state_pub.publish(m)

    # ── saved-pose library ────────────────────────────────────────────

    def _resolve_pose_library_path(self, configured):
        if configured:
            return os.path.expanduser(configured)
        return os.path.join(os.path.expanduser('~'), '.config',
                            'robocorea_arm', 'poses.yaml')

    def _load_pose_library(self):
        """Load name → {pose, joints} from the YAML library (best-effort)."""
        self._poses = {}
        try:
            with open(self._pose_lib_path, 'r') as f:
                data = yaml.safe_load(f) or {}
        except FileNotFoundError:
            return
        except Exception as exc:
            self.get_logger().warn(f'could not read pose library: {exc}')
            return
        for name, entry in (data.get('poses') or {}).items():
            try:
                pos, ori = entry['position'], entry['orientation']
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = map(float, pos)
                (pose.orientation.x, pose.orientation.y,
                 pose.orientation.z, pose.orientation.w) = map(float, ori)
                self._poses[str(name)] = {
                    'pose': pose,
                    'joints': [float(v) for v in entry.get('joints', [])],
                }
            except Exception as exc:
                self.get_logger().warn(f"skipping malformed pose '{name}': {exc}")

    def _save_pose_library(self):
        data = {'poses': {}}
        for name, entry in self._poses.items():
            p = entry['pose']
            data['poses'][name] = {
                'position': [p.position.x, p.position.y, p.position.z],
                'orientation': [p.orientation.x, p.orientation.y,
                                p.orientation.z, p.orientation.w],
                'joints': list(entry.get('joints', [])),
            }
        try:
            os.makedirs(os.path.dirname(self._pose_lib_path), exist_ok=True)
            with open(self._pose_lib_path, 'w') as f:
                yaml.safe_dump(data, f, default_flow_style=False)
        except Exception as exc:
            self.get_logger().error(f'could not write pose library: {exc}')
            return False
        return True

    # ── pose services ─────────────────────────────────────────────────

    def _srv_save_pose(self, req, resp):
        name = req.name.strip()
        if not name:
            resp.success, resp.message = False, 'pose name is empty'
            return resp
        with self._lock:
            q = self._q.copy()
        pose = _T_to_pose(self.kin.fk(q)[0])
        self._poses[name] = {'pose': pose, 'joints': q.tolist()}
        ok = self._save_pose_library()
        resp.success = ok
        resp.pose = pose
        resp.message = (f"saved pose '{name}'" if ok
                        else f"captured '{name}' but failed to persist library")
        self.get_logger().info(f"save_pose '{name}': "
                               f"p=({pose.position.x:.3f},{pose.position.y:.3f},"
                               f"{pose.position.z:.3f})")
        return resp

    def _srv_delete_pose(self, req, resp):
        name = req.name.strip()
        if name not in self._poses:
            resp.success, resp.message = False, f"no saved pose '{name}'"
            return resp
        del self._poses[name]
        self._save_pose_library()
        resp.success, resp.message = True, f"deleted pose '{name}'"
        return resp

    def _srv_list_poses(self, _req, resp):
        names, poses = [], []
        for name, entry in sorted(self._poses.items()):
            names.append(name)
            poses.append(entry['pose'])
        resp.names = names
        resp.poses = poses
        return resp

    def _srv_go_to_pose(self, req, resp):
        # Resolve the target EE pose + an IK seed.
        if req.use_pose:
            T_goal = _pose_to_T(req.pose)
            with self._lock:
                seed = self._q.copy()
        else:
            entry = self._poses.get(req.name.strip())
            if entry is None:
                resp.success, resp.message = False, f"no saved pose '{req.name}'"
                return resp
            T_goal = _pose_to_T(entry['pose'])
            seed = (np.array(entry['joints'], dtype=float)
                    if len(entry['joints']) == self.n else None)

        if self.respect_fault and self._faulted:
            resp.success, resp.message = False, 'arm faulted — cannot start motion'
            return resp
        with self._lock:
            if self._planning:
                resp.success, resp.message = False, 'already planning a move'
                return resp
            if not self._running:
                resp.success, resp.message = False, 'servo paused — start it first'
                return resp
            self._planning = True
            q_start = self._q.copy()
        if seed is None:
            seed = q_start

        try:
            ok, msg = self._plan_and_install(q_start, T_goal, seed)
        finally:
            with self._lock:
                self._planning = False
        resp.success, resp.message = ok, msg
        return resp

    def _solve_ik(self, T_goal, seed):
        """IK for an EE transform, nearest to ``seed``. Returns a 6-vector or None.

        Closed-form decoupled solver first (exact, enumerates branches, picks the
        one nearest the seed); numerical DLS as the fallback if it's unavailable
        or finds no in-limits solution.
        """
        if self.aik is not None:
            q = self.aik.solve_nearest(T_goal, seed)
            if q is not None:
                return q
        q, ok = arm_planner.solve_ik(
            self.kin, T_goal, seed, self.kin.lower, self.kin.upper,
            char_length=self.L, pos_tol=self.ik_pos_tol, ori_tol=self.ik_ori_tol,
            max_iters=self.ik_max_iters, lam=self.ik_lambda)
        return q if ok else None

    def _plan_and_install(self, q_start, T_goal, seed):
        """IK → RRT → shortcut → install path. Returns (success, message)."""
        self._publish_plan_state('planning')

        q_goal = self._solve_ik(T_goal, seed)
        if q_goal is None:
            self._publish_plan_state('unreachable')
            self._publish_status(STATUS_PLAN_FAILED)
            return False, 'target pose unreachable (IK found no valid solution)'

        def is_valid(q):
            if np.any(q < self.kin.lower) or np.any(q > self.kin.upper):
                return False
            with self._collision_lock:
                if self.collision is not None and \
                        self.collision.in_collision(q, self.plan_margin):
                    return False
                # Plan around the flippers at their current pose; the per-tick
                # filter is the safety net if they move during execution.
                if self.body_collision is not None and \
                        self.body_collision.in_collision(q, self.plan_margin):
                    return False
            return True

        path, msg = arm_planner.plan_rrt(
            q_start, q_goal, is_valid, self.kin.lower, self.kin.upper,
            step=self.rrt_step, res=self.rrt_res, max_time=self.rrt_max_time,
            max_nodes=self.rrt_max_nodes, seed=self.rrt_seed)
        if path is None:
            self._publish_plan_state(f'plan failed ({msg})')
            self._publish_status(STATUS_PLAN_FAILED)
            return False, f'planning failed: {msg}'

        path = arm_planner.shortcut(path, is_valid, self.rrt_res)
        path = [np.asarray(p, dtype=float) for p in path]

        with self._lock:
            self._path = path
            self._path_idx = 1 if len(path) > 1 else 0
            self._twist[:] = 0.0          # drop any stale command so we don't self-abort
            self._joint_jog = None
            self._ref_valid = False
        self._publish_plan_state(f'moving 0/{len(path) - 1}')
        self.get_logger().info(f'go-to-pose: planned {len(path)} waypoints ({msg})')
        return True, f'planned {len(path)} waypoints'


def main():
    rclpy.init()
    node = SdlsServo()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
