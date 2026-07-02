"""Pure helpers for the RoboCorea ESP32 binary protocol."""

from __future__ import annotations

import struct
from dataclasses import dataclass

SOF = b'\xAA\x55'
MAX_PAYLOAD_LEN = 2048

ROLE_CHASSIS = 1
ROLE_ARM = 2
ROLE_SENSOR = 3
ROLE_NAMES = {
    ROLE_CHASSIS: 'chassis',
    ROLE_ARM: 'arm',
    ROLE_SENSOR: 'sensor',
}

MSG_TELEMETRY = 0x01
MSG_SENSOR_THERMAL = 0x02  # SENSOR ESP32: MLX90640 frame (seq + min/max + 768 quantised px)
MSG_MAG = 0x03  # SENSOR ESP32: QMC5883L XYZ (int16 µT)
MSG_STATUS = 0x05
MSG_ENCODER_EXT = 0x07
MSG_VESC_STATUS = 0x08
MSG_ODRIVE_STATUS = 0x0A
MSG_LKTECH_STATUS = 0x0B
MSG_ZE300_STATUS = 0x0C
MSG_ODRIVE_ERROR = 0x0D
MSG_ARM_LIFECYCLE = 0x0E
MSG_BOARD_IDENTITY = 0x0F

MSG_ARM_JOINTS = 0x10
MSG_SENSOR_ENABLE = 0x11  # RESERVED — sensors are always-on on the SENSOR ESP32; GUI toggles are display-only
MSG_ESTOP = 0x12
MSG_ESTOP_CLEAR = 0x13
MSG_KEYBIND = 0x14
MSG_PPM_CALIB = 0x15
MSG_GRIPPER = 0x16
MSG_ARM_INIT = 0x17
MSG_ARM_DISARM = 0x18
MSG_ARM_MODE = 0x19
MSG_TRACTION_CMD = 0x1A  # 2×int16 normalised L/R track speed ×1000 + u8 enable (Nav2 /cmd_vel)

CAP_CHASSIS_IO = 1 << 0
CAP_ARM_IO = 1 << 1
CAP_RC_PPM = 1 << 2
CAP_MAG = 1 << 3  # QMC5883L (GY-271) on the SENSOR ESP32 I2C bus
CAP_VESC_BASE = 1 << 4
CAP_ARM_CAN = 1 << 5
CAP_THERMAL = 1 << 6  # MLX90640 on the SENSOR ESP32 I2C bus


@dataclass(frozen=True)
class BoardIdentity:
    role: int
    protocol_version: int
    capabilities: int

    @property
    def role_name(self) -> str:
        return ROLE_NAMES.get(self.role, f'unknown_{self.role}')


def build_frame(msg_type: int, payload: bytes) -> bytes:
    len_h = (len(payload) >> 8) & 0xFF
    len_l = len(payload) & 0xFF
    crc = msg_type ^ len_h ^ len_l
    for b in payload:
        crc ^= b
    return SOF + bytes([msg_type, len_h, len_l]) + payload + bytes([crc])


def build_traction_cmd(left_norm: float, right_norm: float, enable: bool) -> bytes:
    """External traction command frame (MSG_TRACTION_CMD).

    left/right are normalised track speeds in [-1, 1] (forward positive); they map
    onto the same VESC eRPM scaling the RC path uses on the ESP32. enable=False
    releases the tracks back to RC control.
    """
    left = max(-1000, min(1000, int(round(left_norm * 1000.0))))
    right = max(-1000, min(1000, int(round(right_norm * 1000.0))))
    payload = struct.pack('<hhB', left, right, 1 if enable else 0)
    return build_frame(MSG_TRACTION_CMD, payload)


THERMAL_COLS = 32
THERMAL_ROWS = 24
THERMAL_PIXELS = THERMAL_COLS * THERMAL_ROWS
# ThermalFramePayload header: seq u16, min_c100 i16, max_c100 i16, cols u8, rows u8.
_THERMAL_HEADER = struct.Struct('<HhhBB')


def parse_thermal_header(payload: bytes):
    """Split a MSG_SENSOR_THERMAL payload into (seq, min_c, max_c, cols, rows, q).

    `q` is the raw quantised pixel block (one byte/pixel, row-major). Dequantise
    with  celsius = min_c + (q / 255) * (max_c - min_c).
    """
    hdr = _THERMAL_HEADER.size
    if len(payload) < hdr:
        raise struct.error('thermal payload too short')
    seq, min_c100, max_c100, cols, rows = _THERMAL_HEADER.unpack_from(payload)
    npix = cols * rows
    q = payload[hdr:hdr + npix]
    if len(q) < npix:
        raise struct.error('thermal payload pixel count mismatch')
    return seq, min_c100 / 100.0, max_c100 / 100.0, cols, rows, q


def parse_identity(payload: bytes) -> BoardIdentity:
    if len(payload) < 4:
        raise struct.error('board identity payload too short')
    role, version, caps = struct.unpack_from('<BBH', payload)
    return BoardIdentity(role=role, protocol_version=version, capabilities=caps)


class FrameParser:
    """Incremental parser for [AA 55 TYPE LEN_H LEN_L PAYLOAD CRC] frames."""

    def __init__(self, max_payload_len: int = MAX_PAYLOAD_LEN):
        self.max_payload_len = max_payload_len
        self.reset()

    def reset(self):
        self.state = 'SOF0'
        self.msg_type = 0
        self.length = 0
        self.running_crc = 0
        self.payload = bytearray()

    def feed(self, raw: bytes) -> list[tuple[int, bytes]]:
        frames: list[tuple[int, bytes]] = []
        for byte in raw:
            if self.state == 'SOF0':
                if byte == 0xAA:
                    self.state = 'SOF1'
            elif self.state == 'SOF1':
                if byte == 0x55:
                    self.state = 'TYPE'
                elif byte == 0xAA:
                    self.state = 'SOF1'
                else:
                    self.state = 'SOF0'
            elif self.state == 'TYPE':
                self.msg_type = byte
                self.running_crc = byte
                self.state = 'LEN_H'
            elif self.state == 'LEN_H':
                self.length = byte << 8
                self.running_crc ^= byte
                self.state = 'LEN_L'
            elif self.state == 'LEN_L':
                self.length |= byte
                self.running_crc ^= byte
                self.payload = bytearray()
                if self.length > self.max_payload_len:
                    self.state = 'SOF0'
                else:
                    self.state = 'CRC' if self.length == 0 else 'PAYLOAD'
            elif self.state == 'PAYLOAD':
                self.payload.append(byte)
                self.running_crc ^= byte
                if len(self.payload) >= self.length:
                    self.state = 'CRC'
            elif self.state == 'CRC':
                if byte == self.running_crc:
                    frames.append((self.msg_type, bytes(self.payload)))
                self.state = 'SOF0'
        return frames


class RoleRouteTable:
    """Small role-to-link map used by the ROS bridge and unit tests."""

    def __init__(self):
        self._by_role: dict[int, object] = {}

    def assign(self, link: object, identity: BoardIdentity) -> object | None:
        previous = self._by_role.get(identity.role)
        self._by_role[identity.role] = link
        return previous

    def get(self, role: int) -> object | None:
        return self._by_role.get(role)

    def clear_link(self, link: object):
        for role, current in list(self._by_role.items()):
            if current is link:
                del self._by_role[role]

    def links(self) -> list[object]:
        return list(dict.fromkeys(self._by_role.values()))


class ChassisEstopMirror:
    """Tracks chassis e-stop state and emits arm e-stop frames on transitions."""

    def __init__(self):
        self.active = False

    def update(self, active: bool) -> bytes | None:
        active = bool(active)
        if active == self.active:
            return None
        self.active = active
        return build_frame(MSG_ESTOP if active else MSG_ESTOP_CLEAR, b'')

    def reset(self) -> bytes | None:
        """The chassis link is gone, so its mirrored e-stop is no longer
        authoritative. Drop to inactive and, if we were holding the arm
        e-stopped, return an MSG_ESTOP_CLEAR frame for the caller to forward to
        the arm (the caller still gates it on any software e-stop)."""
        if not self.active:
            return None
        self.active = False
        return build_frame(MSG_ESTOP_CLEAR, b'')
