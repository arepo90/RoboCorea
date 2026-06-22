#!/usr/bin/env python3
"""
Joystick teleop (Xbox-style) for the SDLS servo. Reads /joy and publishes
TwistStamped on `/servo_node/delta_twist_cmds` and JointJog on
`/servo_node/delta_joint_cmds`; toggles the servo via its start/pause services.
Adapted from the reference jaguar_teleop joystick_servo (robot-neutral).

Run alongside `joy_node` (ros-<distro>-joy):  ros2 run joy joy_node
"""

import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int8, Float32
from std_srvs.srv import Trigger

DEFAULT_JOINT_NAMES = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
DEFAULT_FRAME_ID = 'base_link'
DEFAULT_DEADZONE = 0.10
DEFAULT_PUBLISH_HZ = 50.0
DEFAULT_JOY_TIMEOUT = 0.5
DEFAULT_WRIST_SPEED_SCALE = 0.6
DEFAULT_WRIST_SLEW_RATE = 8.0
DEFAULT_TRIGGER_DEADZONE = 0.15
EPS = 1e-6

AX_LX, AX_LY, AX_LT, AX_RX, AX_RY, AX_RT, AX_DX, AX_DY = range(8)
BTN_A, BTN_B, BTN_X, BTN_Y, BTN_LB, BTN_RB, BTN_BACK, BTN_START, BTN_GUIDE = range(9)

_STATUS = {
    0: None,
    1: 'WARNING  near singularity — damping one direction',
    2: 'HALT     singularity halt',
    3: 'WARNING  leaving singularity',
    4: 'HALT     collision',
    5: 'WARNING  near joint limit',
}


def apply_deadzone(value: float, deadzone: float) -> float:
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def apply_unipolar_deadzone(value: float, deadzone: float) -> float:
    value = max(0.0, min(1.0, value))
    if value < deadzone:
        return 0.0
    return (value - deadzone) / (1.0 - deadzone)


def button_pair(negative_idx: int, positive_idx: int, buttons) -> float:
    negative = 1.0 if len(buttons) > negative_idx and buttons[negative_idx] else 0.0
    positive = 1.0 if len(buttons) > positive_idx and buttons[positive_idx] else 0.0
    return positive - negative


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class JoystickServo(Node):
    def __init__(self):
        super().__init__('joystick_servo')

        self.declare_parameter('robot_name', 'Dicerox Arm')
        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)
        self.declare_parameter('planning_frame', DEFAULT_FRAME_ID)
        self.declare_parameter('twist_topic', '/servo_node/delta_twist_cmds')
        self.declare_parameter('joint_topic', '/servo_node/delta_joint_cmds')
        self.declare_parameter('status_topic', '/servo_node/status')
        self.declare_parameter('pause_service', '/servo_node/pause_servo')
        self.declare_parameter('start_service', '/servo_node/start_servo')
        self.declare_parameter('start_mode', 'cart')
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('deadzone', DEFAULT_DEADZONE)
        self.declare_parameter('trigger_deadzone', DEFAULT_TRIGGER_DEADZONE)
        self.declare_parameter('publish_hz', DEFAULT_PUBLISH_HZ)
        self.declare_parameter('joy_timeout', DEFAULT_JOY_TIMEOUT)
        self.declare_parameter('wrist_speed_scale', DEFAULT_WRIST_SPEED_SCALE)
        self.declare_parameter('wrist_slew_rate', DEFAULT_WRIST_SLEW_RATE)

        self.robot_name = str(self.get_parameter('robot_name').value)
        self.joint_names = [str(n) for n in self.get_parameter('joint_names').value]
        self.frame_id = str(self.get_parameter('planning_frame').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.trigger_deadzone = float(self.get_parameter('trigger_deadzone').value)
        self.twist_topic = str(self.get_parameter('twist_topic').value)
        self.joint_topic = str(self.get_parameter('joint_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.pause_service = str(self.get_parameter('pause_service').value)
        self.start_service = str(self.get_parameter('start_service').value)
        self.mode = str(self.get_parameter('start_mode').value).lower()
        self.speed = float(self.get_parameter('speed_scale').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.joy_timeout = float(self.get_parameter('joy_timeout').value)
        self.wrist_speed_scale = float(self.get_parameter('wrist_speed_scale').value)
        self.wrist_slew_rate = float(self.get_parameter('wrist_slew_rate').value)

        if not self.joint_names:
            raise ValueError('joint_names must contain at least one joint.')
        if self.mode not in {'cart', 'joint'}:
            raise ValueError("start_mode must be 'cart' or 'joint'.")
        if self.deadzone < 0.0 or self.deadzone >= 1.0:
            raise ValueError('deadzone must be in [0.0, 1.0).')
        if self.trigger_deadzone < 0.0 or self.trigger_deadzone >= 1.0:
            raise ValueError('trigger_deadzone must be in [0.0, 1.0).')
        if self.speed <= 0.0:
            raise ValueError('speed_scale must be > 0.0.')
        if self.publish_hz <= 0.0:
            raise ValueError('publish_hz must be > 0.0.')
        if self.joy_timeout <= 0.0:
            raise ValueError('joy_timeout must be > 0.0.')
        if self.wrist_speed_scale <= 0.0:
            raise ValueError('wrist_speed_scale must be > 0.0.')
        if self.wrist_slew_rate <= 0.0:
            raise ValueError('wrist_slew_rate must be > 0.0.')

        self.declare_parameter('gripper_topic', '/gripper')
        # Gripper open/close on two FREE face buttons (LT/RT/LB/RB are taken by the
        # wrist). Held = step until limit. Defaults: B opens, A closes; remap via
        # these params (Xbox: A=0 B=1 X=2 Y=3 LB=4 RB=5 Back=6 Start=7 Guide=8).
        self.declare_parameter('gripper_open_button', BTN_B)
        self.declare_parameter('gripper_close_button', BTN_A)
        self.gripper_topic = str(self.get_parameter('gripper_topic').value)
        self._grip_open_btn = int(self.get_parameter('gripper_open_button').value)
        self._grip_close_btn = int(self.get_parameter('gripper_close_button').value)

        self.twist_pub = self.create_publisher(TwistStamped, self.twist_topic, 10)
        self.joint_pub = self.create_publisher(JointJog, self.joint_topic, 10)
        # Gripper open/close rate: open button → +1, close button → −1.
        self.gripper_pub = self.create_publisher(Float32, self.gripper_topic, 10)
        self._pause_cli = self.create_client(Trigger, self.pause_service)
        self._start_cli = self.create_client(Trigger, self.start_service)

        self.active_joint = 0
        self._servo_paused = False
        self._last_status = 0
        self._prev_buttons = []
        self._dpad_y_prev = 0.0
        self._trigger_neutral = {}
        self._trigger_direction = {}
        self._state_lock = threading.Lock()
        self._axes = []
        self._buttons = []
        self._joy_seen = False
        self._last_joy_time = 0.0
        self._wrist_cmd = [0.0, 0.0]
        self._last_publish_time = time.monotonic()

        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.create_subscription(Int8, self.status_topic, self._status_cb, 10)
        self.create_timer(1.0 / self.publish_hz, self._publish_cb)

        self.get_logger().info(
            f'Joystick servo ready for {self.robot_name} '
            f'({len(self.joint_names)} joints, start_mode={self.mode}).')
        self._print_help()

    def _print_help(self):
        print('\n' + '=' * 60)
        print(f'  {self.robot_name} — Joystick Servo')
        print('=' * 60)
        print('  CART: LY=+X  LX=+Y  RY=+Z  RX=Yaw  LT/RT=Pitch  LB/RB=Roll')
        print('  JOINT: D-pad Up/Dn select joint, LY jog')
        print('  GRIPPER: B=open  A=close (hold; any mode)')
        print('  Start=toggle mode   Back=stop   Y=pause/resume   Guide=e-stop')
        print('=' * 60 + '\n')

    def _status_cb(self, msg: Int8):
        if msg.data == self._last_status:
            return
        self._last_status = msg.data
        text = _STATUS.get(msg.data)
        if text:
            self.get_logger().warn(f'[SERVO] {text}')
        elif msg.data == 0:
            self.get_logger().info('[SERVO] OK')

    def _call_service(self, client, label):
        if not client.service_is_ready() and not client.wait_for_service(timeout_sec=0.25):
            self.get_logger().warn(f'{label} service not available')
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'{label}: {f.result().message}' if f.result() else f'{label}: no response'))

    def _toggle_pause(self):
        if self._servo_paused:
            self._servo_paused = False
            self._call_service(self._start_cli, 'start_servo')
        else:
            self._publish_zero()
            self._servo_paused = True
            self._call_service(self._pause_cli, 'pause_servo')

    def _reset_motion_filters(self):
        self._wrist_cmd = [0.0, 0.0]
        self._last_publish_time = time.monotonic()

    def _trigger_value(self, axes, idx: int) -> float:
        """Normalize common trigger axis styles to 0.0 released, 1.0 pressed."""
        if len(axes) <= idx:
            return 0.0

        raw = float(axes[idx])
        if idx not in self._trigger_neutral:
            self._trigger_neutral[idx] = raw
            return 0.0

        neutral = self._trigger_neutral[idx]
        if neutral >= 0.5:
            value = (neutral - raw) / max(neutral + 1.0, EPS)
        elif neutral <= -0.5:
            value = (raw - neutral) / max(1.0 - neutral, EPS)
        else:
            delta = raw - neutral
            if idx not in self._trigger_direction and abs(delta) > self.deadzone:
                self._trigger_direction[idx] = 1.0 if delta > 0.0 else -1.0
            direction = self._trigger_direction.get(idx, 1.0)
            value = (delta * direction) / max(1.0 - abs(neutral), EPS)

        return apply_unipolar_deadzone(value, self.trigger_deadzone)

    def _publish_gripper(self, buttons, active):
        """Open button → +1, close button → −1 (held = step until limit);
        0 when inactive (e-stop / paused) or both/neither pressed."""
        if active:
            opening = len(buttons) > self._grip_open_btn and buttons[self._grip_open_btn]
            closing = len(buttons) > self._grip_close_btn and buttons[self._grip_close_btn]
            rate = float(bool(opening)) - float(bool(closing))
        else:
            rate = 0.0
        self.gripper_pub.publish(Float32(data=rate))

    def _joy_cb(self, msg: Joy):
        axes = list(msg.axes)
        buttons = list(msg.buttons)
        if not self._prev_buttons:
            self._prev_buttons = [0] * len(buttons)

        def just_pressed(idx):
            previous = self._prev_buttons[idx] if idx < len(self._prev_buttons) else 0
            return idx < len(buttons) and buttons[idx] == 1 and previous == 0

        with self._state_lock:
            self._axes = axes
            self._buttons = buttons
            self._joy_seen = True
            self._last_joy_time = time.monotonic()

        # Gripper buttons work in any mode; halt them on e-stop / pause.
        guide_held = len(buttons) > BTN_GUIDE and buttons[BTN_GUIDE]
        self._publish_gripper(buttons, active=not (guide_held or self._servo_paused))

        if len(buttons) > BTN_GUIDE and buttons[BTN_GUIDE]:
            self._publish_zero()
            self._prev_buttons = buttons
            return
        if just_pressed(BTN_Y):
            threading.Thread(target=self._toggle_pause, daemon=True).start()
            self._prev_buttons = buttons
            return
        if self._servo_paused:
            self._prev_buttons = buttons
            return
        if just_pressed(BTN_START):
            self.mode = 'joint' if self.mode == 'cart' else 'cart'
            self._reset_motion_filters()
            self.get_logger().info(f'Mode → {self.mode.upper()}')
        if just_pressed(BTN_BACK):
            self._reset_motion_filters()
            self._publish_zero()
            self._prev_buttons = buttons
            return
        self._prev_buttons = buttons

    def _publish_cb(self):
        with self._state_lock:
            axes = list(self._axes)
            buttons = list(self._buttons)
            joy_seen = self._joy_seen
            age = time.monotonic() - self._last_joy_time

        if not joy_seen:
            return
        guide = len(buttons) > BTN_GUIDE and buttons[BTN_GUIDE]
        stop = len(buttons) > BTN_BACK and buttons[BTN_BACK]
        if self._servo_paused or guide or stop or age > self.joy_timeout:
            self._reset_motion_filters()
            self._publish_zero()
            return

        if self.mode == 'cart':
            self._handle_cart(axes, buttons)
        else:
            self._handle_joint(axes, buttons)

    def _slew_wrist(self, roll_target: float, pitch_target: float):
        now = time.monotonic()
        dt = clamp(now - self._last_publish_time, 0.0, 0.1)
        self._last_publish_time = now
        step = self.wrist_slew_rate * dt
        targets = [roll_target, pitch_target]
        for i, target in enumerate(targets):
            target = clamp(target, -1.0, 1.0)
            delta = clamp(target - self._wrist_cmd[i], -step, step)
            self._wrist_cmd[i] = clamp(self._wrist_cmd[i] + delta, -1.0, 1.0)
        return self._wrist_cmd

    def _handle_cart(self, axes, buttons):
        lx = apply_deadzone(axes[AX_LX], self.deadzone) if len(axes) > AX_LX else 0.0
        ly = apply_deadzone(axes[AX_LY], self.deadzone) if len(axes) > AX_LY else 0.0
        rx = apply_deadzone(axes[AX_RX], self.deadzone) if len(axes) > AX_RX else 0.0
        ry = apply_deadzone(axes[AX_RY], self.deadzone) if len(axes) > AX_RY else 0.0
        lt = self._trigger_value(axes, AX_LT)
        rt = self._trigger_value(axes, AX_RT)
        roll, pitch = self._slew_wrist(button_pair(BTN_LB, BTN_RB, buttons), rt - lt)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = ly * self.speed
        msg.twist.linear.y = -lx * self.speed
        msg.twist.linear.z = ry * self.speed
        msg.twist.angular.x = roll * self.speed * self.wrist_speed_scale
        msg.twist.angular.y = pitch * self.speed * self.wrist_speed_scale
        msg.twist.angular.z = -rx * self.speed
        self.twist_pub.publish(msg)

    def _handle_joint(self, axes, buttons):
        dpad_y = axes[AX_DY] if len(axes) > AX_DY else 0.0
        if dpad_y > 0.5 and self._dpad_y_prev <= 0.5:
            self.active_joint = (self.active_joint - 1) % len(self.joint_names)
            self.get_logger().info(f'Joint → {self.joint_names[self.active_joint]}')
        elif dpad_y < -0.5 and self._dpad_y_prev >= -0.5:
            self.active_joint = (self.active_joint + 1) % len(self.joint_names)
            self.get_logger().info(f'Joint → {self.joint_names[self.active_joint]}')
        self._dpad_y_prev = dpad_y

        velocity = (apply_deadzone(axes[AX_LY], self.deadzone) * self.speed
                    if len(axes) > AX_LY else 0.0)
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.joint_names = [self.joint_names[self.active_joint]]
        msg.velocities = [velocity]
        msg.duration = 0.0
        self.joint_pub.publish(msg)

    def _publish_zero(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.twist_pub.publish(msg)


def main():
    rclpy.init()
    node = JoystickServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
