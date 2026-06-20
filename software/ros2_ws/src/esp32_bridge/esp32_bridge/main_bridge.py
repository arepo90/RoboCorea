"""
RoboCorea ESP32 ⇄ ROS 2 bridge (Jetson side)
=============================================
Runs on the Jetson Orin Nano. Owns the USB-serial link to the ESP32, decodes the
binary protocol into ROS 2 topics, and serializes inbound topics back into frames.

Binary frame: [0xAA][0x55][TYPE:1][LEN_H:1][LEN_L:1][PAYLOAD:LEN][CRC:1]
              CRC = XOR of TYPE ^ LEN_H ^ LEN_L ^ all payload bytes.
The struct formats below MUST stay byte-identical to firmware include/robot_types.h.

Published (ESP32 → PC)
──────────────────────
/robot/telemetry      std_msgs/Float32MultiArray  [spd_l_rpm, spd_r_rpm, flipper_deg, uptime_s]
/robot/mode           std_msgs/String             INIT/STANDBY/NORMAL/ARM/ESTOP/FLIPPER
/robot/flags          std_msgs/UInt8              bit0 ppm, bit1 sensors, bit2 can, bit3 estop
/robot/ppm            std_msgs/Int16MultiArray    raw PPM µs [ch1..ch6]
/robot/status         diagnostic_msgs/DiagnosticArray
/robot/deadband       std_msgs/Float32            normalised RC deadband (echoed from calib)
/encoders/tracks      geometry_msgs/Vector3       x=left_rpm, y=right_rpm (live track speed)
/encoders/flipper     std_msgs/Float32MultiArray  [fl, fr, rl, rr] degrees
/odom/wheel           nav_msgs/Odometry           track odometry from the traction VESC tachometers
/sensors/mag          sensor_msgs/MagneticField   LIS3MDL XYZ
/motors/vesc_status   std_msgs/Float32MultiArray  [id, erpm, current_A, duty, t_fet, t_mot, v_in, tacho]
/motors/odrive_status std_msgs/Float32MultiArray  [joint, pos_turns, vel_turns_s, iq_A, bus_V, bus_A]
/motors/lktech_status std_msgs/Float32MultiArray  [joint, motor_id, temp_C, iq_A, dps, angle, out_deg]
/motors/ze300_status  std_msgs/Float32MultiArray  [id, temp_C, iq_A, rpm, single_turn, pos_counts, out_deg]
/motors/odrive_error  std_msgs/Float32MultiArray  [node_id, motor_error]
/gripper              std_msgs/Float32            normalised gripper command from RC
/arm/operating_mode   std_msgs/String             DEXTERITY or CHASSIS
/arm/joint_active_mask std_msgs/UInt8              bits 0..5 indicate active position control

Subscribed (PC → ESP32)
───────────────────────
/robot/estop          std_msgs/Bool               True = ESTOP, False = ESTOP_CLEAR
/joint_states         sensor_msgs/JointState      arm joint positions (rad) → MSG_ARM_JOINTS
/sensors/enable_mask  std_msgs/UInt8              bit0 mag (thermal/gas/imu bits unused here)
/robot/keybind        std_msgs/UInt8MultiArray    15 bytes (3 modes × 5 channels)
/robot/ppm_calib      std_msgs/UInt16MultiArray   19 values (6ch × min/neu/max + deadband)

Parameters
──────────
serial_port (str)  /dev/ttyUSB0
baud_rate   (int)  921600
joint_names (str[]) URDF joint names in J1..J6 order (for /joint_states mapping)
joint_command_signs (float[]) URDF radians → firmware physical-degree signs

Track odometry (VESC tachometer → /odom/wheel). The two traction VESC tachometers
are converted to per-track distance and integrated into a 2D wheel-odometry pose.
This feeds the (deferred) robot_localization EKF that fuses it with the ZED2; it is
published WITHOUT a TF — the EKF owns odom→base_link. The covariances deliberately
down-weight yaw (a skid-steer tracked robot's wheel heading is unreliable under
track slip). Measure/confirm these on the robot:
  traction_id_left / traction_id_right (int)   VESC CAN ids (config.h: 60 / 50)
  traction_dir_left / traction_dir_right (float) tacho sign (match config.h TRACTION_DIR_*)
  wheel_circumference_m (float)  track sprocket effective circumference  [MEASURE]
  track_width_m (float)          lateral distance between the two tracks  [MEASURE]
  motor_pole_pairs (int)         VESC motor pole pairs (config.h: 7)
  gear_ratio (float)             motor→output reduction (config.h: 100)
  tacho_steps_per_erev (float)   VESC tachometer steps per electrical rev (6; confirm vs VESC fw)
"""

import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import serial

from std_msgs.msg import (
    Bool, String, UInt8, UInt16, Float32, Float32MultiArray,
    Int16MultiArray, UInt8MultiArray, UInt16MultiArray,
)
from sensor_msgs.msg import MagneticField, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import Trigger

# ─── Protocol IDs (must match config.h) ──────────────────────────────────────
MSG_TELEMETRY    = 0x01
MSG_MAG          = 0x03
MSG_STATUS       = 0x05
# 0x06 (MSG_SENSOR_IMU) reserved/unused — no IMU on the ESP32; orientation is from the ZED2.
MSG_ENCODER_EXT  = 0x07
MSG_VESC_STATUS  = 0x08
MSG_ODRIVE_STATUS = 0x0A
MSG_LKTECH_STATUS = 0x0B
MSG_ZE300_STATUS  = 0x0C
MSG_ODRIVE_ERROR  = 0x0D
MSG_ARM_LIFECYCLE = 0x0E
MSG_GRIPPER       = 0x16

MSG_ARM_JOINTS    = 0x10
MSG_SENSOR_ENABLE = 0x11
MSG_ESTOP         = 0x12
MSG_ESTOP_CLEAR   = 0x13
MSG_KEYBIND       = 0x14
MSG_PPM_CALIB     = 0x15
MSG_ARM_INIT      = 0x17
MSG_ARM_DISARM    = 0x18
MSG_ARM_MODE      = 0x19

# Arm lifecycle state codes (match firmware ArmState)
ARM_STATE_NAMES = {0: 'UNINIT', 1: 'INITIALIZING', 2: 'READY', 3: 'FAULT'}
ARM_MODE_NAMES = {0: 'DEXTERITY', 1: 'CHASSIS'}

MODE_NAMES = {0: 'INIT', 1: 'STANDBY', 2: 'NORMAL', 3: 'ARM', 4: 'ESTOP', 5: 'FLIPPER'}

PPM_CHANNELS = 6
MAX_PAYLOAD_LEN = 2048   # any larger "length" is a false SOF match → resync


def _build_frame(msg_type: int, payload: bytes) -> bytes:
    len_h = (len(payload) >> 8) & 0xFF
    len_l = len(payload) & 0xFF
    crc = msg_type ^ len_h ^ len_l
    for b in payload:
        crc ^= b
    return bytes([0xAA, 0x55, msg_type, len_h, len_l]) + payload + bytes([crc])


class ESP32BridgeNode(Node):

    def __init__(self):
        super().__init__('esp32_bridge')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('reconnect_period', 3.0)
        # Must match the names the arm servo publishes on /joint_states and the
        # URDF (arm_teleop / dicerox_arm.urdf uses capitalized Joint1..Joint6).
        self.declare_parameter(
            'joint_names',
            ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'])
        # Match the working Dicerox bridge convention: MoveIt/URDF joint radians
        # are converted to the firmware's physical joint-degree frame before the
        # ESP32 applies its per-motor gear/direction mapping.
        self.declare_parameter('joint_command_signs', [-1.0, -1.0, -1.0, -1.0, -1.0, 1.0])

        self._serial_port = self.get_parameter('serial_port').value
        self._baud_rate = int(self.get_parameter('baud_rate').value)
        self._reconnect_period = float(self.get_parameter('reconnect_period').value)
        self._joint_names = list(self.get_parameter('joint_names').value)
        self._joint_command_signs = [float(v) for v in self.get_parameter('joint_command_signs').value]
        if len(self._joint_command_signs) != 6:
            self.get_logger().warn(
                f'joint_command_signs must have 6 values, got {len(self._joint_command_signs)}; using defaults')
            self._joint_command_signs = [-1.0, -1.0, -1.0, -1.0, -1.0, 1.0]
        self._ser = None

        # ── Track (wheel) odometry from the VESC tachometers ──────────────────
        # Defaults match config.h; the geometry values MUST be measured on the robot.
        self.declare_parameter('traction_id_left', 60)
        self.declare_parameter('traction_id_right', 50)
        self.declare_parameter('traction_dir_left', 1.0)
        self.declare_parameter('traction_dir_right', 1.0)
        self.declare_parameter('wheel_circumference_m', 0.5)   # TODO: measure sprocket effective circumference
        self.declare_parameter('track_width_m', 0.4)           # TODO: measure track centre-to-centre
        self.declare_parameter('motor_pole_pairs', 7)
        self.declare_parameter('gear_ratio', 100.0)
        self.declare_parameter('tacho_steps_per_erev', 6.0)    # VESC commutation steps/erev (confirm vs VESC fw)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('odom_child_frame', 'base_link')
        self.declare_parameter('odom_rate_hz', 50.0)

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

        # Wheel-odometry integrator state.
        self._track_dist = {'L': None, 'R': None}   # per-side distance (m), relative to first reading
        self._track_zero = {'L': None, 'R': None}   # first cumulative reading (m), subtracted out
        self._odom_last = {'L': 0.0, 'R': 0.0}       # last integrated per-side distance (m)
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._odom_last_t = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, depth=10)
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

        # Publishers
        self._pub_telemetry = self.create_publisher(Float32MultiArray, '/robot/telemetry', sensor_qos)
        self._pub_mode      = self.create_publisher(String,            '/robot/mode',      sensor_qos)
        self._pub_flags     = self.create_publisher(UInt8,             '/robot/flags',     sensor_qos)
        self._pub_ppm       = self.create_publisher(Int16MultiArray,   '/robot/ppm',       sensor_qos)
        self._pub_status    = self.create_publisher(DiagnosticArray,   '/robot/status',    sensor_qos)
        self._pub_deadband  = self.create_publisher(Float32,           '/robot/deadband',  latched_qos)
        self._pub_tracks    = self.create_publisher(Vector3,           '/encoders/tracks', sensor_qos)
        self._pub_flipper   = self.create_publisher(Float32MultiArray, '/encoders/flipper', sensor_qos)
        self._pub_wheel_odom = self.create_publisher(Odometry,         '/odom/wheel',      sensor_qos)
        self._pub_mag       = self.create_publisher(MagneticField,     '/sensors/mag',     sensor_qos)
        self._pub_vesc      = self.create_publisher(Float32MultiArray, '/motors/vesc_status',   sensor_qos)
        self._pub_odrive    = self.create_publisher(Float32MultiArray, '/motors/odrive_status', sensor_qos)
        self._pub_lktech    = self.create_publisher(Float32MultiArray, '/motors/lktech_status', sensor_qos)
        self._pub_ze300     = self.create_publisher(Float32MultiArray, '/motors/ze300_status',  sensor_qos)
        self._pub_odrv_err  = self.create_publisher(Float32MultiArray, '/motors/odrive_error',  sensor_qos)
        self._pub_gripper   = self.create_publisher(Float32,           '/gripper',         sensor_qos)
        # Arm safety lifecycle (latched so a late-joining GUI/servo sees current state)
        self._pub_arm_state = self.create_publisher(String,            '/arm/state',       latched_qos)
        self._pub_arm_fault = self.create_publisher(Bool,              '/arm/fault',       latched_qos)
        self._pub_arm_presence = self.create_publisher(UInt16,          '/arm/can_presence', latched_qos)
        self._pub_arm_mode = self.create_publisher(String, '/arm/operating_mode', latched_qos)
        self._pub_arm_active = self.create_publisher(UInt8, '/arm/joint_active_mask', latched_qos)
        self._last_arm_state = None
        self._last_arm_mode = None

        # Arm arm/disarm services (operator → ESP32 explicit lifecycle commands)
        self.create_service(Trigger, '/arm/arm',    self._srv_arm)
        self.create_service(Trigger, '/arm/disarm', self._srv_disarm)
        self.create_service(Trigger, '/arm/mode/dexterity', self._srv_dexterity_mode)
        self.create_service(Trigger, '/arm/mode/chassis', self._srv_chassis_mode)

        # Subscribers
        self.create_subscription(Bool,       '/robot/estop',         self._on_estop,         10)
        self.create_subscription(JointState, '/joint_states',        self._on_joint_states,  10)
        self.create_subscription(UInt8,      '/sensors/enable_mask', self._on_sensor_enable, 10)
        self.create_subscription(UInt8MultiArray,  '/robot/keybind',   self._on_keybind,   latched_qos)
        self.create_subscription(UInt16MultiArray, '/robot/ppm_calib', self._on_ppm_calib, latched_qos)

        # Dispatch table
        self._handlers = {
            MSG_TELEMETRY:     self._handle_telemetry,
            MSG_MAG:           self._handle_mag,
            MSG_STATUS:        self._handle_status,
            MSG_ENCODER_EXT:   self._handle_encoder_ext,
            MSG_VESC_STATUS:   self._handle_vesc_status,
            MSG_ODRIVE_STATUS: self._handle_odrive_status,
            MSG_LKTECH_STATUS: self._handle_lktech_status,
            MSG_ZE300_STATUS:  self._handle_ze300_status,
            MSG_ODRIVE_ERROR:  self._handle_odrive_error,
            MSG_ARM_LIFECYCLE: self._handle_arm_lifecycle,
            MSG_GRIPPER:       self._handle_gripper,
        }

        # Integrate wheel odometry on a fixed-rate timer (decoupled from the async,
        # per-side arrival of the VESC status frames so dt is stable).
        rate = self._odom_rate_hz if self._odom_rate_hz > 0.0 else 50.0
        self._odom_timer = self.create_timer(1.0 / rate, self._integrate_wheel_odom)

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        self.get_logger().info('RoboCorea ESP32 bridge ready (serial handled asynchronously)')

    # ── Serial RX ────────────────────────────────────────────────────────────
    def _open_serial(self) -> bool:
        try:
            self._ser = serial.Serial(self._serial_port, self._baud_rate, timeout=0.1)
            try:
                self._ser.reset_input_buffer()
            except Exception:
                pass
            self.get_logger().info(f'Opened {self._serial_port} @ {self._baud_rate}')
            # Start with all sensors off; the GUI enables them via /sensors/enable_mask.
            self._send(_build_frame(MSG_SENSOR_ENABLE, bytes([0x00])))
            return True
        except (serial.SerialException, OSError) as e:
            self._ser = None
            self.get_logger().warn(
                f'{self._serial_port} unavailable ({e}); retry in {self._reconnect_period:.0f}s')
            return False

    def _rx_loop(self):
        state = 'SOF0'
        msg_type = length = running_crc = 0
        payload = bytearray()

        while rclpy.ok():
            if self._ser is None:
                if not self._open_serial():
                    time.sleep(self._reconnect_period)
                    continue
                state, payload, running_crc = 'SOF0', bytearray(), 0

            try:
                raw = self._ser.read(256)
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Serial read error: {e}; reconnecting')
                self._close_serial()
                continue
            if not raw:
                continue

            for byte in raw:
                if state == 'SOF0':
                    if byte == 0xAA:
                        state = 'SOF1'
                elif state == 'SOF1':
                    state = 'TYPE' if byte == 0x55 else 'SOF0'
                elif state == 'TYPE':
                    msg_type = byte
                    running_crc = byte
                    state = 'LEN_H'
                elif state == 'LEN_H':
                    length = byte << 8
                    running_crc ^= byte
                    state = 'LEN_L'
                elif state == 'LEN_L':
                    length |= byte
                    running_crc ^= byte
                    payload = bytearray()
                    if length > MAX_PAYLOAD_LEN:
                        state = 'SOF0'
                    else:
                        state = 'CRC' if length == 0 else 'PAYLOAD'
                elif state == 'PAYLOAD':
                    payload.append(byte)
                    running_crc ^= byte
                    if len(payload) >= length:
                        state = 'CRC'
                elif state == 'CRC':
                    if byte == running_crc:
                        self._dispatch(msg_type, bytes(payload))
                    state = 'SOF0'

    def _dispatch(self, msg_type: int, payload: bytes):
        handler = self._handlers.get(msg_type)
        if handler is None:
            return
        try:
            handler(payload)
        except struct.error as e:
            self.get_logger().warn(f'Parse error type=0x{msg_type:02X}: {e}')

    # ── ESP32 → PC handlers ──────────────────────────────────────────────────
    def _handle_telemetry(self, payload: bytes):
        fmt = '<BB' + 'H' * PPM_CHANNELS + 'hhhI'
        if len(payload) < struct.calcsize(fmt):
            return
        fields = struct.unpack_from(fmt, payload)
        mode_val, flags = fields[0], fields[1]
        ppm = list(fields[2:2 + PPM_CHANNELS])
        spd_l_x10, spd_r_x10, flip_x10, uptime_ms = fields[2 + PPM_CHANNELS:]

        m = String(); m.data = MODE_NAMES.get(mode_val, f'UNKNOWN_{mode_val}')
        self._pub_mode.publish(m)

        f = UInt8(); f.data = flags
        self._pub_flags.publish(f)

        p = Int16MultiArray(); p.data = [int(v) for v in ppm]
        self._pub_ppm.publish(p)

        t = Float32MultiArray()
        t.data = [spd_l_x10 / 10.0, spd_r_x10 / 10.0, flip_x10 / 10.0, uptime_ms / 1000.0]
        self._pub_telemetry.publish(t)

        v = Vector3(); v.x = spd_l_x10 / 10.0; v.y = spd_r_x10 / 10.0
        self._pub_tracks.publish(v)

    def _handle_encoder_ext(self, payload: bytes):
        fmt = '<hhhh'
        if len(payload) < struct.calcsize(fmt):
            return
        fl, fr, rl, rr = struct.unpack_from(fmt, payload)
        msg = Float32MultiArray()
        msg.data = [fl / 10.0, fr / 10.0, rl / 10.0, rr / 10.0]
        self._pub_flipper.publish(msg)

    def _handle_mag(self, payload: bytes):
        fmt = '<hhh'
        if len(payload) < struct.calcsize(fmt):
            return
        x, y, z = struct.unpack_from(fmt, payload)
        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'mag_link'
        msg.magnetic_field.x = float(x)
        msg.magnetic_field.y = float(y)
        msg.magnetic_field.z = float(z)
        self._pub_mag.publish(msg)

    def _handle_vesc_status(self, payload: bytes):
        fmt = '<Bihhhhhi'
        if len(payload) < struct.calcsize(fmt):
            return
        vid, erpm, cur10, duty1000, tfet10, tmot10, vin10, tacho = struct.unpack_from(fmt, payload)
        msg = Float32MultiArray()
        msg.data = [float(vid), float(erpm), cur10 / 10.0, duty1000 / 1000.0,
                    tfet10 / 10.0, tmot10 / 10.0, vin10 / 10.0, float(tacho)]
        self._pub_vesc.publish(msg)

        # Feed the wheel-odometry integrator from the two traction VESCs.
        if vid == self._trac_id_l:
            self._update_track_distance('L', tacho, self._trac_dir_l)
        elif vid == self._trac_id_r:
            self._update_track_distance('R', tacho, self._trac_dir_r)

    # ── Wheel odometry ────────────────────────────────────────────────────────
    def _tacho_to_metres(self, tacho_counts: int, direction: float) -> float:
        """VESC tachometer (commutation steps) → output-shaft distance in metres."""
        erevs = tacho_counts / self._tacho_steps_per_erev
        mech_revs = erevs / max(self._pole_pairs, 1)
        output_revs = mech_revs / (self._gear_ratio if self._gear_ratio else 1.0)
        return direction * output_revs * self._wheel_circ_m

    def _update_track_distance(self, side: str, tacho_counts: int, direction: float):
        dist = self._tacho_to_metres(tacho_counts, direction)
        if self._track_zero[side] is None:
            self._track_zero[side] = dist        # zero the cumulative count at first reading
        self._track_dist[side] = dist - self._track_zero[side]

    def _integrate_wheel_odom(self):
        dl_total, dr_total = self._track_dist['L'], self._track_dist['R']
        if dl_total is None or dr_total is None:
            return  # need both tracks before integrating

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

        ds = 0.5 * (dl + dr)                                  # forward increment (reliable)
        dyaw = (dr - dl) / self._track_width_m if self._track_width_m else 0.0  # skid-steer (unreliable)
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

        # Skid-steer covariances: forward (x / vx) is trustworthy, yaw is NOT — let
        # the EKF take heading from the ZED2 instead. Unused 2D axes get large values.
        # [x, y, z, roll, pitch, yaw] diagonal, row-major 6×6. Tune on hardware.
        big = 1e6
        odom.pose.covariance[0] = 0.05      # x
        odom.pose.covariance[7] = 0.05      # y
        odom.pose.covariance[14] = big      # z
        odom.pose.covariance[21] = big      # roll
        odom.pose.covariance[28] = big      # pitch
        odom.pose.covariance[35] = 0.5      # yaw (high → distrust)
        odom.twist.covariance[0] = 0.02     # vx
        odom.twist.covariance[7] = big      # vy
        odom.twist.covariance[14] = big     # vz
        odom.twist.covariance[21] = big     # vroll
        odom.twist.covariance[28] = big     # vpitch
        odom.twist.covariance[35] = 0.5     # vyaw (high → distrust)
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

    def _handle_gripper(self, payload: bytes):
        if len(payload) < 2:
            return
        val1000, = struct.unpack_from('<h', payload)
        m = Float32(); m.data = val1000 / 1000.0
        self._pub_gripper.publish(m)

    def _handle_status(self, payload: bytes):
        if len(payload) < 4:
            return
        mode_val, flags, sensor_mask, _ = struct.unpack_from('<BBBB', payload)
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        st = DiagnosticStatus()
        st.name = 'RoboCorea ESP32'
        st.level = DiagnosticStatus.OK
        st.message = MODE_NAMES.get(mode_val, f'UNKNOWN_{mode_val}')
        st.values = [
            KeyValue(key='mode',        value=str(mode_val)),
            KeyValue(key='ppm_ok',      value=str(bool(flags & 0x01))),
            KeyValue(key='sensors_on',  value=str(bool(flags & 0x02))),
            KeyValue(key='can_ok',      value=str(bool(flags & 0x04))),
            KeyValue(key='estop',       value=str(bool(flags & 0x08))),
            KeyValue(key='sensor_mask', value=hex(sensor_mask)),
        ]
        if flags & 0x08:
            st.level = DiagnosticStatus.ERROR
            st.message = 'ESTOP ACTIVE'
        diag.status = [st]
        self._pub_status.publish(diag)

    # ── PC → ESP32 ───────────────────────────────────────────────────────────
    def _send(self, frame: bytes):
        if self._ser is None:
            return
        try:
            self._ser.write(frame)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial write error: {e}; reconnecting')
            self._close_serial()

    def _close_serial(self):
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        self._ser = None

    def _on_estop(self, msg: Bool):
        if msg.data:
            self._send(_build_frame(MSG_ESTOP, b''))
            self.get_logger().warn('Sent ESTOP')
        else:
            self._send(_build_frame(MSG_ESTOP_CLEAR, b''))

    # ── Arm safety lifecycle ─────────────────────────────────────────────────
    def _handle_arm_lifecycle(self, payload):
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
        sm = String(); sm.data = name; self._pub_arm_state.publish(sm)
        fb = Bool(); fb.data = (state == 3); self._pub_arm_fault.publish(fb)
        pm = UInt16(); pm.data = presence; self._pub_arm_presence.publish(pm)
        mm = String(); mm.data = mode_name; self._pub_arm_mode.publish(mm)
        am = UInt8(); am.data = active_mask; self._pub_arm_active.publish(am)

    def _srv_arm(self, request, response):
        self._send(_build_frame(MSG_ARM_INIT, b''))
        response.success = True
        response.message = 'arm init/arm requested'
        self.get_logger().info('Arm: init/arm requested')
        return response

    def _srv_disarm(self, request, response):
        self._send(_build_frame(MSG_ARM_DISARM, b''))
        response.success = True
        response.message = 'arm disarm requested'
        self.get_logger().warn('Arm: disarm requested')
        return response

    def _request_arm_mode(self, mode, name, response):
        self._send(_build_frame(MSG_ARM_MODE, bytes([mode])))
        response.success = True
        response.message = f'{name.lower()} mode requested'
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
        degs = [
            math.degrees(name_to_pos.get(n, 0.0)) * self._joint_command_signs[i]
            for i, n in enumerate(self._joint_names)
        ]
        payload = struct.pack('<' + 'h' * 6, *[int(d * 100.0) for d in degs])
        self._send(_build_frame(MSG_ARM_JOINTS, payload))

    def _on_sensor_enable(self, msg: UInt8):
        self._send(_build_frame(MSG_SENSOR_ENABLE, bytes([msg.data])))
        self.get_logger().info(f'Sensor enable mask: 0x{msg.data:02X}')

    def _on_keybind(self, msg: UInt8MultiArray):
        if len(msg.data) < 15:
            self.get_logger().warn('keybind needs 15 bytes (3 modes × 5 channels)')
            return
        self._send(_build_frame(MSG_KEYBIND, bytes(msg.data[:15])))
        self.get_logger().info('Keybind table sent to ESP32')

    def _on_ppm_calib(self, msg: UInt16MultiArray):
        if len(msg.data) < 19:
            self.get_logger().warn(f'ppm_calib needs 19 values, got {len(msg.data)}')
            return
        payload = struct.pack('<' + 'HHH' * 6 + 'H', *[int(v) for v in msg.data[:19]])
        self._send(_build_frame(MSG_PPM_CALIB, payload))
        db = Float32(); db.data = msg.data[18] / 1000.0
        self._pub_deadband.publish(db)
        self.get_logger().info(f'PPM calibration sent (deadband={db.data:.3f})')


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
