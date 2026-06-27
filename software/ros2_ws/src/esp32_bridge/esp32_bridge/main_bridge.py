"""
RoboCorea ESP32 ⇄ ROS 2 bridge (Jetson side)
=============================================
Runs on the Jetson Orin Nano. Owns one or two USB-serial links to identical
ESP32 PCBs and routes frames by the board identity announced by firmware:

  * CHASSIS PCB: RC/PPM, traction, flippers, wheel odometry.
  * ARM PCB: arm lifecycle, joint commands, and mixed-CAN arm telemetry.

The public ROS API intentionally stays the same as the old one-PCB bridge:
chassis data appears on /robot, /encoders, /odom, and /motors/vesc_*;
arm data appears on /arm and the arm motor topics.
"""

from __future__ import annotations

import ast
import glob
import math
import os
import struct
import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import serial

from std_msgs.msg import (
    Bool, String, UInt8, UInt16, Float32, Float32MultiArray,
    Int16MultiArray, UInt16MultiArray,
)
from sensor_msgs.msg import Image, JointState, MagneticField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Twist
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import Trigger
from mlx90640_msgs.msg import ThermalStatus

from .protocol import (
    ROLE_ARM,
    ROLE_CHASSIS,
    ROLE_NAMES,
    MSG_ARM_DISARM,
    MSG_ARM_INIT,
    MSG_ARM_JOINTS,
    MSG_ARM_LIFECYCLE,
    MSG_ARM_MODE,
    MSG_BOARD_IDENTITY,
    MSG_ENCODER_EXT,
    MSG_ESTOP,
    MSG_ESTOP_CLEAR,
    MSG_GRIPPER,
    MSG_LKTECH_STATUS,
    MSG_MAG,
    MSG_ODRIVE_ERROR,
    MSG_ODRIVE_STATUS,
    MSG_PPM_CALIB,
    MSG_SENSOR_THERMAL,
    MSG_STATUS,
    MSG_TELEMETRY,
    MSG_TRACTION_CMD,
    MSG_VESC_STATUS,
    MSG_ZE300_STATUS,
    ChassisEstopMirror,
    FrameParser,
    RoleRouteTable,
    build_frame,
    build_sensor_enable,
    build_traction_cmd,
    parse_identity,
    parse_thermal_header,
)

MICROTESLA_TO_TESLA = 1e-6

ARM_STATE_NAMES = {0: 'UNINIT', 1: 'INITIALIZING', 2: 'READY', 3: 'FAULT'}
ARM_MODE_NAMES = {0: 'DEXTERITY', 1: 'CHASSIS'}
MODE_NAMES = {0: 'INIT', 1: 'STANDBY', 2: 'NORMAL', 3: 'ARM', 4: 'ESTOP', 5: 'FLIPPER'}

PPM_CHANNELS = 6
# 0-based PPM indices of the traction drive sticks (firmware Ch3 fwd / Ch4 turn).
PPM_CH_TRACTION_FWD = 2
PPM_CH_TRACTION_TURN = 3


def _normalise_candidates(value) -> list[str]:
    if isinstance(value, (list, tuple)):
        return [str(v).strip() for v in value if str(v).strip()]
    text = '' if value is None else str(value).strip()
    if not text:
        return []
    if text.startswith('['):
        try:
            parsed = ast.literal_eval(text)
            if isinstance(parsed, (list, tuple)):
                return [str(v).strip() for v in parsed if str(v).strip()]
        except (SyntaxError, ValueError):
            pass
    return [part.strip() for part in text.split(',') if part.strip()]


class SerialLink:
    """One candidate serial device, read on a background thread."""

    def __init__(self, node: 'ESP32BridgeNode', path: str, realpath: str,
                 baud_rate: int, reconnect_period: float):
        self.node = node
        self.path = path
        self.realpath = realpath
        self.baud_rate = baud_rate
        self.reconnect_period = reconnect_period
        self.parser = FrameParser()
        self.role: int | None = None
        self.identity = None
        self.last_rx_monotonic = 0.0
        self._ser = None
        self._serial_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f'esp32:{os.path.basename(path)}', daemon=True)
        self._last_open_warn = 0.0

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._drop_serial()

    def join(self, timeout: float = 1.0):
        self._thread.join(timeout=timeout)

    def send(self, frame: bytes) -> bool:
        bad_serial = False
        with self._serial_lock:
            ser = self._ser
            if ser is None:
                return False
            try:
                ser.write(frame)
                return True
            except (serial.SerialException, OSError) as e:
                bad_serial = True
                self.node.get_logger().error(f'{self.path} write error: {e}; reconnecting')
        if bad_serial:
            self._drop_serial()
        return False

    def _open_serial(self) -> bool:
        try:
            # exclusive=True (POSIX flock) refuses a second open of the same
            # device, a backstop against a stale link and a fresh one briefly
            # racing for the same port across a USB re-enumeration.
            ser = serial.Serial(self.path, self.baud_rate, timeout=0.1, exclusive=True)
            try:
                ser.reset_input_buffer()
            except Exception:
                pass
            with self._serial_lock:
                self._ser = ser
            self.parser.reset()
            self.node.get_logger().info(f'Opened ESP candidate {self.path} @ {self.baud_rate}')
            return True
        except (serial.SerialException, OSError) as e:
            now = time.monotonic()
            if now - self._last_open_warn >= 10.0:
                self._last_open_warn = now
                self.node.get_logger().warn(
                    f'{self.path} unavailable ({e}); retry in {self.reconnect_period:.0f}s')
            return False

    def _drop_serial(self):
        with self._serial_lock:
            ser = self._ser
            self._ser = None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def _read(self, size: int) -> bytes:
        with self._serial_lock:
            ser = self._ser
        if ser is None:
            return b''
        return ser.read(size)

    def _run(self):
        while rclpy.ok() and not self._stop.is_set():
            with self._serial_lock:
                opened = self._ser is not None
            if not opened:
                if not self._open_serial():
                    self._stop.wait(self.reconnect_period)
                    continue

            try:
                raw = self._read(256)
            except (serial.SerialException, OSError) as e:
                self.node.get_logger().error(f'{self.path} read error: {e}; reconnecting')
                self._drop_serial()
                continue
            if not raw:
                continue

            self.last_rx_monotonic = time.monotonic()
            for msg_type, payload in self.parser.feed(raw):
                self.node._on_serial_frame(self, msg_type, payload)


class ESP32BridgeNode(Node):

    def __init__(self):
        super().__init__('esp32_bridge')

        self.declare_parameter('serial_port', '')
        self.declare_parameter('serial_candidates', '/dev/serial/by-id/*,/dev/serial/by-path/*')
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('reconnect_period', 3.0)
        self.declare_parameter('discovery_period', 2.0)
        self.declare_parameter(
            'joint_names',
            ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'])
        self.declare_parameter('joint_command_signs', [-1.0, -1.0, -1.0, -1.0, -1.0, 1.0])

        self._serial_port = str(self.get_parameter('serial_port').value or '')
        self._serial_candidates = self.get_parameter('serial_candidates').value
        self._baud_rate = int(self.get_parameter('baud_rate').value)
        self._reconnect_period = float(self.get_parameter('reconnect_period').value)
        self._discovery_period = float(self.get_parameter('discovery_period').value)
        self._joint_names = list(self.get_parameter('joint_names').value)
        self._joint_command_signs = [float(v) for v in self.get_parameter('joint_command_signs').value]
        if len(self._joint_command_signs) != 6:
            self.get_logger().warn(
                f'joint_command_signs must have 6 values, got {len(self._joint_command_signs)}; using defaults')
            self._joint_command_signs = [-1.0, -1.0, -1.0, -1.0, -1.0, 1.0]
        # Last-known arm joint pose (radians), merged per-joint from /joint_states.
        # /joint_states carries the arm joints (servo_node) and the flipper joints
        # (flipper_state) on the same topic; we must NOT zero-fill the joints a
        # given message omits, or a flipper-only update would snap the arm to 0.
        self._last_arm_rad = {n: 0.0 for n in self._joint_names}

        # Track odometry parameters.
        self.declare_parameter('traction_id_left', 60)
        self.declare_parameter('traction_id_right', 50)
        self.declare_parameter('traction_dir_left', 1.0)
        self.declare_parameter('traction_dir_right', 1.0)
        self.declare_parameter('wheel_circumference_m', 0.707)
        self.declare_parameter('track_width_m', 0.455)
        self.declare_parameter('motor_pole_pairs', 7)
        self.declare_parameter('gear_ratio', 23.333)
        self.declare_parameter('tacho_steps_per_erev', 6.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('odom_child_frame', 'base_link')
        self.declare_parameter('odom_rate_hz', 50.0)

        # External traction (Nav2 /cmd_vel) → MSG_TRACTION_CMD. Disabled by default:
        # the bridge will NOT command the tracks unless explicitly enabled (safety).
        self.declare_parameter('enable_cmd_vel_drive', False)
        # Linear speed (m/s) that maps to normalised 1.0; should match the firmware's
        # TRACTION_ERPM_MAX. Conservative default — bench-calibrate (Nav2 closes the
        # loop on odometry, so an approximate value still reaches goals).
        self.declare_parameter('max_track_speed_mps', 1.0)
        self.declare_parameter('cmd_vel_timeout', 0.3)

        self._trac_id_l = int(self.get_parameter('traction_id_left').value)
        self._trac_id_r = int(self.get_parameter('traction_id_right').value)
        self._trac_dir_l = float(self.get_parameter('traction_dir_left').value)
        self._trac_dir_r = float(self.get_parameter('traction_dir_right').value)
        self._wheel_circ_m = float(self.get_parameter('wheel_circumference_m').value)
        self._track_width_m = float(self.get_parameter('track_width_m').value)
        self._pole_pairs = int(self.get_parameter('motor_pole_pairs').value)
        self._gear_ratio = float(self.get_parameter('gear_ratio').value)
        self._tacho_steps_per_erev = float(self.get_parameter('tacho_steps_per_erev').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._odom_child_frame = str(self.get_parameter('odom_child_frame').value)
        self._odom_rate_hz = float(self.get_parameter('odom_rate_hz').value)

        # Autonomy (/cmd_vel → tracks) is a RUNTIME gate. The launch param is only the
        # INITIAL state: the GUI toggles it live on /autonomy/enable, and an operator
        # RC override latches it back off (see _handle_telemetry / _set_autonomy). The
        # firmware adds its own stick-neutral + freshness arbitration as the lowest
        # safety layer, so this gate is purely an extra software allow/prevent.
        self._autonomy_enabled = bool(self.get_parameter('enable_cmd_vel_drive').value)
        self._max_track_speed = float(self.get_parameter('max_track_speed_mps').value)
        self._cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self._cmd_vel_last_t = None
        self._cmd_vel_released = True

        # RC stick state for the autonomy→teleop handoff. Defaults mirror the firmware
        # (neutral 1500 µs, ±500 span, deadband 0.05) until /robot/ppm_calib arrives;
        # PpmCalibPayload is (min, neutral, max) per channel + deadband×1000 (index 18).
        self._ppm_min = [1000] * PPM_CHANNELS
        self._ppm_neutral = [1500] * PPM_CHANNELS
        self._ppm_max = [2000] * PPM_CHANNELS
        self._ppm_deadband = 0.05

        self._track_dist = {'L': None, 'R': None}
        self._track_zero = {'L': None, 'R': None}
        self._odom_last = {'L': 0.0, 'R': 0.0}
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._odom_last_t = None

        # Arm-PCB passive sensors (MLX90640 thermal + LIS3MDL mag) arrive over the
        # ARM UART link; republish them on the same topics the old Jetson I2C nodes
        # used, and relay the GUI's /sensors/enable_mask down as MSG_SENSOR_ENABLE.
        self.declare_parameter('thermal_frame_id', 'mlx90640_link')
        self.declare_parameter('mag_frame_id', 'mag_link')
        self.declare_parameter('default_sensor_mask', 3)  # bit0 mag, bit1 thermal (on)
        self._thermal_frame_id = str(self.get_parameter('thermal_frame_id').value)
        self._mag_frame_id = str(self.get_parameter('mag_frame_id').value)
        self._sensor_mask = int(self.get_parameter('default_sensor_mask').value) & 0xFF
        self._thermal_last_t = None
        self._thermal_rate_hz = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, depth=10)
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE, depth=10)

        # Publishers
        self._pub_telemetry = self.create_publisher(Float32MultiArray, '/robot/telemetry', sensor_qos)
        self._pub_mode = self.create_publisher(String, '/robot/mode', sensor_qos)
        self._pub_flags = self.create_publisher(UInt8, '/robot/flags', sensor_qos)
        self._pub_ppm = self.create_publisher(Int16MultiArray, '/robot/ppm', sensor_qos)
        self._pub_status = self.create_publisher(DiagnosticArray, '/robot/status', sensor_qos)
        self._pub_deadband = self.create_publisher(Float32, '/robot/deadband', latched_qos)
        self._pub_tracks = self.create_publisher(Vector3, '/encoders/tracks', sensor_qos)
        self._pub_flipper = self.create_publisher(Float32MultiArray, '/encoders/flipper', sensor_qos)
        self._pub_wheel_odom = self.create_publisher(Odometry, '/odom/wheel', sensor_qos)
        # Actual autonomy-drive state (latched so a late-joining GUI reflects it). The
        # bridge owns this: it goes false on a GUI disable AND on an RC override.
        self._pub_autonomy_state = self.create_publisher(Bool, '/autonomy/state', latched_qos)
        self._pub_vesc = self.create_publisher(Float32MultiArray, '/motors/vesc_status', sensor_qos)
        self._pub_odrive = self.create_publisher(Float32MultiArray, '/motors/odrive_status', sensor_qos)
        self._pub_lktech = self.create_publisher(Float32MultiArray, '/motors/lktech_status', sensor_qos)
        self._pub_ze300 = self.create_publisher(Float32MultiArray, '/motors/ze300_status', sensor_qos)
        self._pub_odrv_err = self.create_publisher(Float32MultiArray, '/motors/odrive_error', sensor_qos)
        # Arm-PCB passive sensors, republished on the legacy topic names/types.
        self._pub_mag = self.create_publisher(MagneticField, '/sensors/mag', sensor_qos)
        self._pub_thermal = self.create_publisher(Image, '/sensors/thermal', sensor_qos)
        self._pub_thermal_raw = self.create_publisher(Image, '/sensors/thermal_raw', sensor_qos)
        self._pub_thermal_status = self.create_publisher(
            ThermalStatus, '/sensors/thermal_status', reliable_qos)
        self._pub_arm_state = self.create_publisher(String, '/arm/state', latched_qos)
        self._pub_arm_fault = self.create_publisher(Bool, '/arm/fault', latched_qos)
        self._pub_arm_presence = self.create_publisher(UInt16, '/arm/can_presence', latched_qos)
        self._pub_arm_mode = self.create_publisher(String, '/arm/operating_mode', latched_qos)
        self._pub_arm_active = self.create_publisher(UInt8, '/arm/joint_active_mask', latched_qos)
        self._last_arm_state = None
        self._last_arm_mode = None

        # Services
        self.create_service(Trigger, '/arm/arm', self._srv_arm)
        self.create_service(Trigger, '/arm/disarm', self._srv_disarm)
        self.create_service(Trigger, '/arm/mode/dexterity', self._srv_dexterity_mode)
        self.create_service(Trigger, '/arm/mode/chassis', self._srv_chassis_mode)

        # Subscribers
        self.create_subscription(Bool, '/robot/estop', self._on_estop, 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        self.create_subscription(UInt16MultiArray, '/robot/ppm_calib', self._on_ppm_calib, latched_qos)
        self.create_subscription(Float32, '/gripper', self._on_gripper, 10)
        # Autonomy drive path: always wired. Forwarding is gated by the runtime
        # _autonomy_enabled flag (GUI toggle + RC-override latch), not by subscription
        # presence, so it can be enabled/disabled live. /autonomy/enable is VOLATILE on
        # purpose: a restarted bridge must come up at its param default (off by default)
        # and wait for a fresh operator request rather than auto-resuming.
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(Bool, '/autonomy/enable', self._on_autonomy_enable, 10)
        # GUI enable mask (bit0 mag, bit1 thermal) → relayed to the ARM PCB. Latched
        # QoS so a late-joining bridge picks up the operator's last choice; until one
        # arrives we use default_sensor_mask, sent when the arm link identifies.
        self.create_subscription(UInt8, '/sensors/enable_mask', self._on_enable_mask, latched_qos)
        # Watchdog: release the tracks back to RC if /cmd_vel goes stale.
        self.create_timer(0.1, self._cmd_vel_watchdog)
        self._publish_autonomy_state()
        if self._autonomy_enabled:
            self.get_logger().warn(
                'Autonomy drive ENABLED at startup (enable_cmd_vel_drive=true): /cmd_vel '
                f'will command the traction VESCs when RC drive sticks are neutral '
                f'(max {self._max_track_speed:.2f} m/s = full scale)')

        self._chassis_handlers = {
            MSG_TELEMETRY: self._handle_telemetry,
            MSG_STATUS: self._handle_status,
            MSG_ENCODER_EXT: self._handle_encoder_ext,
            MSG_VESC_STATUS: self._handle_vesc_status,
        }
        self._arm_handlers = {
            MSG_ODRIVE_STATUS: self._handle_odrive_status,
            MSG_LKTECH_STATUS: self._handle_lktech_status,
            MSG_ZE300_STATUS: self._handle_ze300_status,
            MSG_ODRIVE_ERROR: self._handle_odrive_error,
            MSG_ARM_LIFECYCLE: self._handle_arm_lifecycle,
            MSG_MAG: self._handle_mag,
            MSG_SENSOR_THERMAL: self._handle_thermal,
        }

        self._links_lock = threading.Lock()
        self._links_by_realpath: dict[str, SerialLink] = {}
        self._routes = RoleRouteTable()
        self._estop_mirror = ChassisEstopMirror()
        self._software_estop_active = None
        self._last_missing_role_log: dict[tuple[int, str], float] = {}

        rate = self._odom_rate_hz if self._odom_rate_hz > 0.0 else 50.0
        self._odom_timer = self.create_timer(1.0 / rate, self._integrate_wheel_odom)

        self._discovery_stop = threading.Event()
        self._discovery_thread = threading.Thread(target=self._discover_loop, name='esp32-discovery', daemon=True)
        self._discovery_thread.start()

        candidates = ', '.join(self._candidate_patterns())
        self.get_logger().info(f'RoboCorea ESP32 bridge ready; scanning: {candidates or "(none)"}')

    # Serial discovery and routing.
    def _candidate_patterns(self) -> list[str]:
        patterns = _normalise_candidates(self._serial_candidates)
        if self._serial_port:
            patterns.insert(0, self._serial_port)
        return list(dict.fromkeys(patterns))

    def _expand_candidate_paths(self) -> list[str]:
        out: list[str] = []
        for pattern in self._candidate_patterns():
            pattern = os.path.expanduser(pattern)
            matches = glob.glob(pattern) if glob.has_magic(pattern) else [pattern]
            for path in matches:
                if os.path.exists(path):
                    out.append(path)
        return sorted(dict.fromkeys(out))

    def _discover_loop(self):
        while rclpy.ok() and not self._discovery_stop.is_set():
            try:
                self._discover_once()
            except Exception as e:
                self.get_logger().error(f'ESP discovery error: {e}')
            self._discovery_stop.wait(self._discovery_period)

    def _discover_once(self):
        # Resolve the currently-present candidate devices, keyed by realpath so a
        # device matched by both by-id AND by-path globs yields a single link.
        live: dict[str, str] = {}
        for path in self._expand_candidate_paths():
            live.setdefault(os.path.realpath(path), path)

        reaped: list[SerialLink] = []
        started: list[tuple[SerialLink, str]] = []
        with self._links_lock:
            # Reap links whose device has disappeared (e.g. unplugged, or a USB
            # re-enumeration moved it to a new /dev/ttyUSB*). Without this the
            # link map only grows and a re-enumerated board would be opened twice
            # — once by the stale link (via its stable by-id symlink) and once by
            # a fresh link keyed on the new realpath — racing for the same port.
            for realpath, link in list(self._links_by_realpath.items()):
                if realpath not in live:
                    del self._links_by_realpath[realpath]
                    self._routes.clear_link(link)
                    reaped.append(link)
            # Start a link for each newly-present device.
            for realpath, path in live.items():
                if realpath in self._links_by_realpath:
                    continue
                link = SerialLink(self, path, realpath, self._baud_rate, self._reconnect_period)
                self._links_by_realpath[realpath] = link
                started.append((link, path))

        for link in reaped:
            link.stop()
            self.get_logger().info(f'ESP serial candidate {link.path} disappeared; dropped')
            # If the chassis (the arm's mirrored e-stop source) went away, release
            # the arm so it can still be armed with only the arm PCB connected.
            if link.role == ROLE_CHASSIS:
                self._on_chassis_lost()
        for link, path in started:
            link.start()
            self.get_logger().info(f'Watching ESP serial candidate {path}')

    def _on_serial_frame(self, link: SerialLink, msg_type: int, payload: bytes):
        if msg_type == MSG_BOARD_IDENTITY:
            self._handle_identity(link, payload)
            return

        role = link.role
        if role is None:
            return
        handlers = self._chassis_handlers if role == ROLE_CHASSIS else self._arm_handlers
        handler = handlers.get(msg_type)
        if handler is None:
            return
        try:
            handler(payload)
        except struct.error as e:
            self.get_logger().warn(
                f'Parse error from {ROLE_NAMES.get(role, role)} {link.path} type=0x{msg_type:02X}: {e}')

    def _handle_identity(self, link: SerialLink, payload: bytes):
        try:
            identity = parse_identity(payload)
        except struct.error as e:
            self.get_logger().warn(f'Bad board identity from {link.path}: {e}')
            return
        if identity.role not in ROLE_NAMES:
            self.get_logger().warn(f'{link.path} announced unknown role {identity.role}')
            return

        with self._links_lock:
            changed = link.identity != identity
            previous = self._routes.assign(link, identity)
            link.role = identity.role
            link.identity = identity

        if previous is not None and previous is not link:
            previous.role = None
            previous.identity = None
            self.get_logger().warn(
                f'Replacing {identity.role_name} link {previous.path} with {link.path}')
        if changed:
            self.get_logger().info(
                f'{link.path} identified as {identity.role_name} '
                f'(protocol={identity.protocol_version}, caps=0x{identity.capabilities:04X})')

        if identity.role == ROLE_CHASSIS:
            if self._software_estop_active:
                link.send(build_frame(MSG_ESTOP, b''))
        elif identity.role == ROLE_ARM:
            if self._software_estop_active or self._estop_mirror.active:
                link.send(build_frame(MSG_ESTOP, b''))
            # Push the current sensor enable mask so a freshly-(re)connected arm
            # PCB starts its mag/thermal in the operator's last-chosen state.
            link.send(build_sensor_enable(self._sensor_mask))

    def _send_to_role(self, role: int, frame: bytes, reason: str, log_missing: bool = True) -> bool:
        with self._links_lock:
            link = self._routes.get(role)
        if link is not None and link.send(frame):
            return True
        if log_missing:
            self._warn_missing_role(role, reason)
        return False

    def _broadcast_frame(self, frame: bytes, reason: str) -> bool:
        with self._links_lock:
            links = list(self._links_by_realpath.values())
        sent = False
        for link in links:
            sent = link.send(frame) or sent
        if not sent:
            now = time.monotonic()
            key = (-1, reason)
            if now - self._last_missing_role_log.get(key, 0.0) >= 5.0:
                self._last_missing_role_log[key] = now
                self.get_logger().warn(f'No ESP serial links available for {reason}')
        return sent

    def _warn_missing_role(self, role: int, reason: str):
        now = time.monotonic()
        key = (role, reason)
        if now - self._last_missing_role_log.get(key, 0.0) < 5.0:
            return
        self._last_missing_role_log[key] = now
        self.get_logger().warn(f'Missing {ROLE_NAMES.get(role, role)} ESP link for {reason}')

    def _mirror_chassis_estop(self, active: bool, source: str):
        frame = self._estop_mirror.update(active)
        if frame is None:
            return
        if not active and self._software_estop_active:
            self.get_logger().info('Chassis e-stop clear seen; arm remains e-stopped by software')
            return
        self._send_to_role(ROLE_ARM, frame, f'chassis {source} e-stop mirror')
        if active:
            self.get_logger().warn('Mirrored chassis e-stop to arm ESP')
        else:
            self.get_logger().info('Mirrored chassis e-stop clear to arm ESP')

    def _on_chassis_lost(self):
        # The arm PCB has no e-stop wire of its own; its only hardware e-stop
        # source is the chassis, mirrored to it here (§16). Once the chassis link
        # is gone that mirror is stale, so drop it and release the arm — UNLESS a
        # software e-stop is separately latched. This is what lets the arm be
        # armed with ONLY the arm PCB connected; while the chassis IS present its
        # e-stop still holds the arm as before. (Caller must not hold _links_lock:
        # _send_to_role takes it.)
        frame = self._estop_mirror.reset()
        if frame is None:
            return  # the mirror wasn't holding the arm e-stopped
        if self._software_estop_active:
            self.get_logger().info(
                'Chassis link lost; cleared its e-stop mirror, but arm stays '
                'e-stopped by software')
            return
        self._send_to_role(ROLE_ARM, frame, 'chassis-lost arm release', log_missing=False)
        self.get_logger().warn('Chassis link lost; released arm from mirrored e-stop')

    # ESP32 -> PC handlers.
    def _handle_telemetry(self, payload: bytes):
        fmt = '<BB' + 'H' * PPM_CHANNELS + 'hhhI'
        if len(payload) < struct.calcsize(fmt):
            return
        fields = struct.unpack_from(fmt, payload)
        mode_val, flags = fields[0], fields[1]
        ppm = list(fields[2:2 + PPM_CHANNELS])
        spd_l_x10, spd_r_x10, flip_x10, uptime_ms = fields[2 + PPM_CHANNELS:]

        self._mirror_chassis_estop(bool(flags & 0x08), 'telemetry')

        # Autonomy → teleop handoff. While autonomy is engaged, an operator drive-stick
        # deflection (Ch3/Ch4) or virtual-flip (flags bit4) latches autonomy OFF; it must
        # then be re-enabled from the GUI (explicit re-arm). The firmware overrides the
        # same inputs transiently every loop; this makes the handoff sticky at the bridge.
        if self._autonomy_enabled:
            n_fwd = self._normalise_ppm(PPM_CH_TRACTION_FWD, ppm)
            n_turn = self._normalise_ppm(PPM_CH_TRACTION_TURN, ppm)
            vflip = bool(flags & 0x10)
            if abs(n_fwd) > self._ppm_deadband or abs(n_turn) > self._ppm_deadband or vflip:
                why = 'virtual-flip engaged' if vflip else 'operator moved a drive stick'
                self._set_autonomy(False, reason=f'RC override ({why})')

        m = String()
        m.data = MODE_NAMES.get(mode_val, f'UNKNOWN_{mode_val}')
        self._pub_mode.publish(m)

        f = UInt8()
        f.data = flags
        self._pub_flags.publish(f)

        p = Int16MultiArray()
        p.data = [int(v) for v in ppm]
        self._pub_ppm.publish(p)

        t = Float32MultiArray()
        t.data = [spd_l_x10 / 10.0, spd_r_x10 / 10.0, flip_x10 / 10.0, uptime_ms / 1000.0]
        self._pub_telemetry.publish(t)

        v = Vector3()
        v.x = spd_l_x10 / 10.0
        v.y = spd_r_x10 / 10.0
        self._pub_tracks.publish(v)

    def _handle_encoder_ext(self, payload: bytes):
        fmt = '<hhhh'
        if len(payload) < struct.calcsize(fmt):
            return
        fl, fr, rl, rr = struct.unpack_from(fmt, payload)
        msg = Float32MultiArray()
        msg.data = [fl / 10.0, fr / 10.0, rl / 10.0, rr / 10.0]
        self._pub_flipper.publish(msg)

    def _handle_vesc_status(self, payload: bytes):
        fmt = '<Bihhhhhi'
        if len(payload) < struct.calcsize(fmt):
            return
        vid, erpm, cur10, duty1000, tfet10, tmot10, vin10, tacho = struct.unpack_from(fmt, payload)
        msg = Float32MultiArray()
        msg.data = [float(vid), float(erpm), cur10 / 10.0, duty1000 / 1000.0,
                    tfet10 / 10.0, tmot10 / 10.0, vin10 / 10.0, float(tacho)]
        self._pub_vesc.publish(msg)

        if vid == self._trac_id_l:
            self._update_track_distance('L', tacho, self._trac_dir_l)
        elif vid == self._trac_id_r:
            self._update_track_distance('R', tacho, self._trac_dir_r)

    # Wheel odometry.
    def _tacho_to_metres(self, tacho_counts: int, direction: float) -> float:
        erevs = tacho_counts / self._tacho_steps_per_erev
        mech_revs = erevs / max(self._pole_pairs, 1)
        output_revs = mech_revs / (self._gear_ratio if self._gear_ratio else 1.0)
        return direction * output_revs * self._wheel_circ_m

    def _update_track_distance(self, side: str, tacho_counts: int, direction: float):
        dist = self._tacho_to_metres(tacho_counts, direction)
        if self._track_zero[side] is None:
            self._track_zero[side] = dist
        self._track_dist[side] = dist - self._track_zero[side]

    def _integrate_wheel_odom(self):
        dl_total, dr_total = self._track_dist['L'], self._track_dist['R']
        if dl_total is None or dr_total is None:
            return

        now = self.get_clock().now()
        if self._odom_last_t is None:
            self._odom_last_t = now
            self._odom_last['L'] = dl_total
            self._odom_last['R'] = dr_total
            return

        dt = (now - self._odom_last_t).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        dl = dl_total - self._odom_last['L']
        dr = dr_total - self._odom_last['R']
        self._odom_last['L'] = dl_total
        self._odom_last['R'] = dr_total
        self._odom_last_t = now

        ds = 0.5 * (dl + dr)
        dyaw = (dr - dl) / self._track_width_m if self._track_width_m else 0.0
        yaw_mid = self._odom_yaw + 0.5 * dyaw
        self._odom_x += ds * math.cos(yaw_mid)
        self._odom_y += ds * math.sin(yaw_mid)
        self._odom_yaw = math.atan2(math.sin(self._odom_yaw + dyaw),
                                    math.cos(self._odom_yaw + dyaw))

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._odom_child_frame
        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.orientation.z = math.sin(self._odom_yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(self._odom_yaw * 0.5)
        odom.twist.twist.linear.x = ds / dt
        odom.twist.twist.angular.z = dyaw / dt

        big = 1e6
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[14] = big
        odom.pose.covariance[21] = big
        odom.pose.covariance[28] = big
        odom.pose.covariance[35] = 0.5
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = big
        odom.twist.covariance[14] = big
        odom.twist.covariance[21] = big
        odom.twist.covariance[28] = big
        odom.twist.covariance[35] = 0.5
        self._pub_wheel_odom.publish(odom)

    def _handle_odrive_status(self, payload: bytes):
        fmt = '<Bhhhhh'
        if len(payload) < struct.calcsize(fmt):
            return
        joint, pos100, vel100, iq100, bv10, bi100 = struct.unpack_from(fmt, payload)
        msg = Float32MultiArray()
        msg.data = [float(joint), pos100 / 100.0, vel100 / 100.0,
                    iq100 / 100.0, bv10 / 10.0, bi100 / 100.0]
        self._pub_odrive.publish(msg)

    def _handle_lktech_status(self, payload: bytes):
        fmt = '<BBbhhhh'
        if len(payload) < struct.calcsize(fmt):
            return
        joint, mid, temp, iq100, dps, angle, out10 = struct.unpack_from(fmt, payload)
        msg = Float32MultiArray()
        msg.data = [float(joint), float(mid), float(temp), iq100 / 100.0,
                    float(dps), float(angle), out10 / 10.0]
        self._pub_lktech.publish(msg)

    def _handle_ze300_status(self, payload: bytes):
        fmt = '<Bbhhhih'
        if len(payload) < struct.calcsize(fmt):
            return
        dev, temp, iq1000, rpm100, st, pos, out10 = struct.unpack_from(fmt, payload)
        msg = Float32MultiArray()
        msg.data = [float(dev), float(temp), iq1000 / 1000.0, rpm100 / 100.0,
                    float(st), float(pos), out10 / 10.0]
        self._pub_ze300.publish(msg)

    def _handle_odrive_error(self, payload: bytes):
        fmt = '<BQ'
        if len(payload) < struct.calcsize(fmt):
            return
        node_id, err = struct.unpack_from(fmt, payload)
        msg = Float32MultiArray()
        msg.data = [float(node_id), float(err)]
        self._pub_odrv_err.publish(msg)
        if err != 0:
            self.get_logger().warn(f'ODrive node {node_id} error: 0x{err:016X}')

    def _handle_status(self, payload: bytes):
        if len(payload) < 4:
            return
        mode_val, flags, sensor_mask, _ = struct.unpack_from('<BBBB', payload)
        self._mirror_chassis_estop(bool(flags & 0x08), 'status')

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        st = DiagnosticStatus()
        st.name = 'RoboCorea chassis ESP32'
        st.level = DiagnosticStatus.OK
        st.message = MODE_NAMES.get(mode_val, f'UNKNOWN_{mode_val}')
        st.values = [
            KeyValue(key='mode', value=str(mode_val)),
            KeyValue(key='ppm_ok', value=str(bool(flags & 0x01))),
            KeyValue(key='minipc_connected', value=str(bool(flags & 0x02))),
            KeyValue(key='can_ok', value=str(bool(flags & 0x04))),
            KeyValue(key='estop', value=str(bool(flags & 0x08))),
            KeyValue(key='sensor_mask', value=hex(sensor_mask)),
        ]
        if flags & 0x08:
            st.level = DiagnosticStatus.ERROR
            st.message = 'ESTOP ACTIVE'
        diag.status = [st]
        self._pub_status.publish(diag)

    def _handle_arm_lifecycle(self, payload: bytes):
        if len(payload) < 7:
            return
        state, fault_code, can_fail, motor_fail, eflg = struct.unpack('<BBHHB', payload[:7])
        presence = 0
        if len(payload) >= 9:
            presence, = struct.unpack('<H', payload[7:9])
        operating_mode = payload[9] if len(payload) >= 10 else 0
        active_mask = payload[10] if len(payload) >= 11 else (0x3F if state == 2 else 0)
        name = ARM_STATE_NAMES.get(state, f'UNKNOWN({state})')
        mode_name = ARM_MODE_NAMES.get(operating_mode, f'UNKNOWN({operating_mode})')
        if name != self._last_arm_state or mode_name != self._last_arm_mode:
            self._last_arm_state = name
            self._last_arm_mode = mode_name
            self.get_logger().info(
                f'arm lifecycle: {name} mode={mode_name} active=0x{active_mask:02X} fault={fault_code} '
                f'can_fail={can_fail} motor_fail={motor_fail} eflg=0x{eflg:02X} '
                f'presence=0x{presence:04X}')
        sm = String()
        sm.data = name
        self._pub_arm_state.publish(sm)
        fb = Bool()
        fb.data = (state == 3)
        self._pub_arm_fault.publish(fb)
        pm = UInt16()
        pm.data = presence
        self._pub_arm_presence.publish(pm)
        mm = String()
        mm.data = mode_name
        self._pub_arm_mode.publish(mm)
        am = UInt8()
        am.data = active_mask
        self._pub_arm_active.publish(am)

    # Arm-PCB passive sensors (I2C, read on the arm ESP32).
    def _handle_mag(self, payload: bytes):
        if len(payload) < 6:
            return
        x_ut, y_ut, z_ut = struct.unpack_from('<hhh', payload)
        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._mag_frame_id
        msg.magnetic_field.x = x_ut * MICROTESLA_TO_TESLA
        msg.magnetic_field.y = y_ut * MICROTESLA_TO_TESLA
        msg.magnetic_field.z = z_ut * MICROTESLA_TO_TESLA
        self._pub_mag.publish(msg)

    def _handle_thermal(self, payload: bytes):
        seq, min_c, max_c, cols, rows, q = parse_thermal_header(payload)
        # Dequantise the 8-bit frame back to °C: min + (q/255)·(max − min).
        span = max_c - min_c
        celsius = (min_c + np.frombuffer(q, dtype=np.uint8).astype(np.float32)
                   * (span / 255.0)).reshape(rows, cols)

        now = self.get_clock().now()
        stamp = now.to_msg()
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = self._thermal_frame_id
        img.height = rows
        img.width = cols
        img.encoding = '32FC1'
        img.is_bigendian = 0
        img.step = cols * 4
        img.data = np.ascontiguousarray(celsius, dtype='<f4').tobytes()
        self._pub_thermal.publish(img)
        self._pub_thermal_raw.publish(img)

        now_s = now.nanoseconds * 1e-9
        if self._thermal_last_t is not None:
            dt = now_s - self._thermal_last_t
            if dt > 0.0:
                self._thermal_rate_hz = 1.0 / dt
        self._thermal_last_t = now_s

        hotspot = int(np.argmax(celsius))
        st = ThermalStatus()
        st.header.stamp = stamp
        st.header.frame_id = self._thermal_frame_id
        st.sequence = int(seq)
        st.sensor_ok = True
        st.min_temperature = float(min_c)
        st.max_temperature = float(max_c)
        st.center_temperature = float(celsius[rows // 2, cols // 2])
        st.hotspot_x = hotspot % cols
        st.hotspot_y = hotspot // cols
        st.total_read_errors = 0
        st.consecutive_read_errors = 0
        st.acquisition_rate_hz = float(self._thermal_rate_hz)
        self._pub_thermal_status.publish(st)

    # PC -> ESP32.
    def _on_estop(self, msg: Bool):
        active = bool(msg.data)
        if self._software_estop_active is not None and active == self._software_estop_active:
            return
        self._software_estop_active = active

        frame = build_frame(MSG_ESTOP if active else MSG_ESTOP_CLEAR, b'')
        if active:
            sent = self._broadcast_frame(frame, 'software e-stop')
            message = (
                'Sent software ESTOP to all discovered ESP links'
                if sent else 'Software ESTOP queued, no ESP links')
            self.get_logger().warn(message)
        else:
            chassis_sent = self._send_to_role(ROLE_CHASSIS, frame, 'software e-stop clear', log_missing=False)
            arm_sent = False
            if not self._estop_mirror.active:
                arm_sent = self._send_to_role(ROLE_ARM, frame, 'software e-stop clear', log_missing=False)
            sent = chassis_sent or arm_sent
            if self._estop_mirror.active:
                message = (
                    'Sent software ESTOP clear to chassis; arm remains e-stopped by chassis mirror'
                    if sent else 'Software ESTOP clear queued; arm remains e-stopped by chassis mirror')
            else:
                message = (
                    'Sent software ESTOP clear to discovered ESP links'
                    if sent else 'Software ESTOP clear queued, no ESP links')
            self.get_logger().info(message)

    def _srv_arm(self, request, response):
        sent = self._send_to_role(ROLE_ARM, build_frame(MSG_ARM_INIT, b''), 'arm init service')
        response.success = sent
        response.message = 'arm init/arm requested' if sent else 'arm ESP link unavailable'
        if sent:
            self.get_logger().info('Arm: init/arm requested')
        return response

    def _srv_disarm(self, request, response):
        sent = self._send_to_role(ROLE_ARM, build_frame(MSG_ARM_DISARM, b''), 'arm disarm service')
        response.success = sent
        response.message = 'arm disarm requested' if sent else 'arm ESP link unavailable'
        if sent:
            self.get_logger().warn('Arm: disarm requested')
        return response

    def _request_arm_mode(self, mode: int, name: str, response):
        sent = self._send_to_role(
            ROLE_ARM,
            build_frame(MSG_ARM_MODE, bytes([mode])),
            f'arm {name.lower()} mode service')
        response.success = sent
        response.message = f'{name.lower()} mode requested' if sent else 'arm ESP link unavailable'
        if sent:
            self.get_logger().info(f'Arm mode: {name} requested')
        return response

    def _srv_dexterity_mode(self, request, response):
        return self._request_arm_mode(0, 'DEXTERITY', response)

    def _srv_chassis_mode(self, request, response):
        return self._request_arm_mode(1, 'CHASSIS', response)

    def _on_joint_states(self, msg: JointState):
        if not msg.name or not msg.position:
            return
        name_to_pos = dict(zip(msg.name, msg.position))
        # Merge per-joint into the cached arm pose. Only update joints this
        # message actually carries; a flipper-only update (from flipper_state)
        # carries none of the arm joints, so it leaves the cache untouched and
        # is not forwarded — avoiding a spurious "snap the arm to 0" command.
        present = [n for n in self._joint_names if n in name_to_pos]
        if not present:
            return
        for n in present:
            self._last_arm_rad[n] = name_to_pos[n]
        degs = [
            math.degrees(self._last_arm_rad[n]) * self._joint_command_signs[i]
            for i, n in enumerate(self._joint_names)
        ]
        payload = struct.pack('<' + 'h' * 6, *[int(d * 100.0) for d in degs])
        self._send_to_role(ROLE_ARM, build_frame(MSG_ARM_JOINTS, payload), 'joint state forwarding')

    def _on_gripper(self, msg: Float32):
        # Signed gripper open/close rate (+open / −close), clamped to ±1, routed
        # to the ARM board as MSG_GRIPPER (int16 ×1000). The firmware integrates
        # it into a clamped servo target, so no rate-limiting is needed here.
        val = max(-1.0, min(1.0, float(msg.data)))
        self._send_to_role(ROLE_ARM, build_frame(MSG_GRIPPER, struct.pack('<h', int(val * 1000.0))),
                           'gripper command')

    def _on_enable_mask(self, msg: UInt8):
        # Relay the GUI's passive-sensor enable mask (bit0 mag, bit1 thermal) to the
        # ARM PCB. Cached so _handle_identity can re-send it to a reconnecting arm.
        mask = int(msg.data) & 0xFF
        self._sensor_mask = mask
        self._send_to_role(ROLE_ARM, build_sensor_enable(mask), 'sensor enable mask',
                           log_missing=False)

    def _normalise_ppm(self, ch: int, ppm) -> float:
        # Mirror the firmware RC::normalise: asymmetric min/neutral/max → [-1, 1].
        raw = ppm[ch]
        neu = self._ppm_neutral[ch]
        if raw >= neu:
            half = self._ppm_max[ch] - neu
            v = (raw - neu) / half if half > 0 else 0.0
        else:
            half = neu - self._ppm_min[ch]
            v = -(neu - raw) / half if half > 0 else 0.0
        return max(-1.0, min(1.0, v))

    def _publish_autonomy_state(self):
        m = Bool()
        m.data = self._autonomy_enabled
        self._pub_autonomy_state.publish(m)

    def _set_autonomy(self, enabled: bool, reason: str = ''):
        if enabled == self._autonomy_enabled:
            return
        self._autonomy_enabled = enabled
        tag = f': {reason}' if reason else ''
        if not enabled:
            # Hand the tracks straight back to RC (the firmware also times out, but make
            # the release explicit and immediate).
            self._send_to_role(ROLE_CHASSIS, build_traction_cmd(0.0, 0.0, False),
                               'autonomy disable release', log_missing=False)
            self._cmd_vel_released = True
            self.get_logger().warn(f'Autonomy drive DISABLED{tag}')
        else:
            self._cmd_vel_last_t = None
            self._cmd_vel_released = True
            self.get_logger().warn(
                f'Autonomy drive ENABLED{tag} (/cmd_vel may move the tracks while RC '
                'drive sticks are neutral)')
        self._publish_autonomy_state()

    def _on_autonomy_enable(self, msg: Bool):
        self._set_autonomy(bool(msg.data), reason='operator (GUI)')

    def _on_cmd_vel(self, msg: Twist):
        # Runtime gate: ignore /cmd_vel unless autonomy is engaged (GUI toggle, not
        # latched off by an RC override). The firmware re-checks RC-stick-neutral too.
        if not self._autonomy_enabled:
            return
        # Differential mixing (geometry lives here; the ESP32 reuses the RC path's
        # eRPM scaling + direction signs). Normalise to [-1,1] against the full-scale
        # track speed. The ESP32 only acts on this while the RC sticks are neutral.
        v = msg.linear.x
        w = msg.angular.z
        half = 0.5 * self._track_width_m
        vmax = self._max_track_speed if self._max_track_speed > 1e-6 else 1.0
        left = (v - w * half) / vmax
        right = (v + w * half) / vmax
        self._send_to_role(ROLE_CHASSIS, build_traction_cmd(left, right, True),
                           'cmd_vel traction', log_missing=False)
        self._cmd_vel_last_t = time.monotonic()
        self._cmd_vel_released = False

    def _cmd_vel_watchdog(self):
        # No fresh /cmd_vel within the timeout → tell the ESP32 to release the tracks
        # back to RC (sent once per stale period). The firmware also times out on its
        # own (EXT_DRIVE_TIMEOUT_MS); this is the explicit release.
        if self._cmd_vel_released:
            return
        if (self._cmd_vel_last_t is None
                or (time.monotonic() - self._cmd_vel_last_t) > self._cmd_vel_timeout):
            self._send_to_role(ROLE_CHASSIS, build_traction_cmd(0.0, 0.0, False),
                               'cmd_vel release', log_missing=False)
            self._cmd_vel_released = True

    def _on_ppm_calib(self, msg: UInt16MultiArray):
        if len(msg.data) < 19:
            self.get_logger().warn(f'ppm_calib needs 19 values, got {len(msg.data)}')
            return
        payload = struct.pack('<' + 'HHH' * 6 + 'H', *[int(v) for v in msg.data[:19]])
        self._send_to_role(ROLE_CHASSIS, build_frame(MSG_PPM_CALIB, payload), 'PPM calibration')
        # Cache the same calibration the firmware uses so the autonomy→teleop override
        # check (_handle_telemetry) reads the operator's real neutral/span/deadband.
        d = [int(v) for v in msg.data[:19]]
        self._ppm_min = [d[3 * c] for c in range(PPM_CHANNELS)]
        self._ppm_neutral = [d[3 * c + 1] for c in range(PPM_CHANNELS)]
        self._ppm_max = [d[3 * c + 2] for c in range(PPM_CHANNELS)]
        self._ppm_deadband = d[18] / 1000.0
        db = Float32()
        db.data = msg.data[18] / 1000.0
        self._pub_deadband.publish(db)
        self.get_logger().info(f'PPM calibration sent (deadband={db.data:.3f})')

    def destroy_node(self):
        self._discovery_stop.set()
        if hasattr(self, '_discovery_thread'):
            self._discovery_thread.join(timeout=1.0)
        with self._links_lock:
            links = list(self._links_by_realpath.values())
        for link in links:
            link.stop()
        for link in links:
            link.join(timeout=1.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESP32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
