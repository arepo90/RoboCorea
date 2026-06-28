#!/usr/bin/env python3
"""
diagnostics.py — RoboCorea hardware diagnostic monitors.

A small suite of focused, read-only checks that turn the esp32_bridge telemetry
into a "what's actually wrong" report. Nothing here commands the robot — they
only subscribe. Run them from the repo-root dispatcher:

    ./diagnose.sh                 # one-shot health snapshot of everything
    ./diagnose.sh link            # comms / which boards are talking   (live)
    ./diagnose.sh ppm             # RC / PPM channels                  (live)
    ./diagnose.sh can             # CAN presence (VESCs + arm joints)  (live)
    ./diagnose.sh sensors         # arm-PCB I2C sensors (thermal/mag)  (live)

…or directly:  ros2 run esp32_bridge diag_link|diag_ppm|diag_can|diag_sensors|diag_all

All of these need the esp32_bridge running on the robot (and the link up). If a
check shows nothing arriving, that itself is the diagnosis — start at `diag_link`.
"""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSReliabilityPolicy)

from std_msgs.msg import (Bool, Float32MultiArray, Int16MultiArray, String,
                          UInt8, UInt16)
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import MagneticField
from nav_msgs.msg import Odometry

try:
    from mlx90640_msgs.msg import ThermalStatus
    _HAVE_THERMAL = True
except Exception:  # pragma: no cover - package may be absent on a minimal build
    _HAVE_THERMAL = False

# ── ANSI colour ───────────────────────────────────────────────────────────────
_OK, _WARN, _BAD, _DIM, _RST = '\033[1;32m', '\033[1;33m', '\033[1;31m', '\033[2m', '\033[0m'


def ok(s):   return f'{_OK}{s}{_RST}'
def warn(s): return f'{_WARN}{s}{_RST}'
def bad(s):  return f'{_BAD}{s}{_RST}'
def dim(s):  return f'{_DIM}{s}{_RST}'


def led(state):
    """state: 'ok' | 'warn' | 'bad' | None -> a coloured dot."""
    return {'ok': ok('●'), 'warn': warn('●'), 'bad': bad('●')}.get(state, dim('○'))


# Expected base-CAN VESC ids (architecture §8.1). Need NOT be contiguous.
EXPECTED_VESC = {
    60: 'traction L', 50: 'traction R',
    20: 'flipper FL', 10: 'flipper FR', 40: 'flipper RL', 30: 'flipper RR',
}
# Arm CAN init-presence mask bits (esp32_bridge /arm/can_presence).
ARM_PRESENCE_BITS = {
    0: 'J1 ODrive (0x10)', 1: 'J2 ODrive (0x11)', 2: 'J3 ODrive (0x12)',
    3: 'J4 ZE300 (id 1)', 4: 'J5 LKTech (14)', 5: 'J6 LKTech (15)',
}
# Fixed RC channel scheme (architecture §13.2), 0-indexed channels.
PPM_ROLES = [
    'Ch1 flipper L/R select (3-way)',
    'Ch2 flipper rate',
    'Ch3 traction fwd/back',
    'Ch4 traction turn',
    'Ch5 flipper pair (front/rear)',
    'Ch6 down=ESTOP / up=virtual-flip',
]


def sensor_qos():
    return QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      history=QoSHistoryPolicy.KEEP_LAST)


def latched_qos():
    return QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                      history=QoSHistoryPolicy.KEEP_LAST)


class Rate:
    """Rolling message-rate + freshness tracker."""
    def __init__(self, window=3.0):
        self.window = window
        self.stamps: list[float] = []
        self.last = None

    def tick(self):
        now = time.monotonic()
        self.last = now
        self.stamps.append(now)
        cut = now - self.window
        self.stamps = [s for s in self.stamps if s >= cut]

    def hz(self):
        if len(self.stamps) < 2:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        return (len(self.stamps) - 1) / span if span > 0 else 0.0

    def age(self):
        return None if self.last is None else time.monotonic() - self.last

    def fresh(self, max_age=2.0):
        a = self.age()
        return a is not None and a <= max_age


def flag_bits(flags: int) -> dict:
    return {
        'ppm_ok': bool(flags & 0x01),
        'minipc_connected': bool(flags & 0x02),
        'can_ok': bool(flags & 0x04),
        'estop': bool(flags & 0x08),
        'virtual_flip': bool(flags & 0x10),
    }


# ── base runner ────────────────────────────────────────────────────────────────
def _run(node: Node, watch: bool, seconds: float):
    """One-shot: collect `seconds`, print once. Watch: print every ~1 s until ^C."""
    try:
        if watch:
            print(dim('(live — Ctrl-C to stop)'))
            next_print = time.monotonic() + 1.0
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                if time.monotonic() >= next_print:
                    node.report()
                    next_print = time.monotonic() + 1.0
        else:
            end = time.monotonic() + seconds
            print(dim(f'(collecting {seconds:.0f}s …)'))
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            node.report()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ── LINK / comms ────────────────────────────────────────────────────────────────
class LinkDiag(Node):
    """Is the robot talking, which boards are bound, and what's the bus health."""
    def __init__(self):
        super().__init__('diag_link')
        self.flags = None
        self.mode = None
        self.diag_kv = {}
        self.r_tel = Rate()      # chassis PCB heartbeat
        self.r_mag = Rate()      # arm PCB (mag, ~50 Hz when enabled)
        self.r_therm = Rate()    # arm PCB (thermal, ~4 Hz when enabled)
        self.arm_state = None
        self.create_subscription(Float32MultiArray, '/robot/telemetry',
                                 lambda m: self.r_tel.tick(), sensor_qos())
        self.create_subscription(UInt8, '/robot/flags', self._flags, sensor_qos())
        self.create_subscription(String, '/robot/mode', self._mode, sensor_qos())
        self.create_subscription(DiagnosticArray, '/robot/status', self._diag, sensor_qos())
        self.create_subscription(MagneticField, '/sensors/mag',
                                 lambda m: self.r_mag.tick(), sensor_qos())
        self.create_subscription(String, '/arm/state', self._arm, latched_qos())
        if _HAVE_THERMAL:
            self.create_subscription(ThermalStatus, '/sensors/thermal_status',
                                     lambda m: self.r_therm.tick(), sensor_qos())

    def _flags(self, m): self.flags = int(m.data)
    def _mode(self, m): self.mode = m.data
    def _arm(self, m): self.arm_state = m.data

    def _diag(self, m):
        for st in m.status:
            for kv in st.values:
                self.diag_kv[kv.key] = kv.value

    def report(self):
        print('\n=== LINK / comms ===')
        chassis = self.r_tel.fresh()
        print(f'{led("ok" if chassis else "bad")} chassis PCB telemetry: '
              f'{ok(f"{self.r_tel.hz():.0f} Hz") if chassis else bad("none")}'
              f'   mode={self.mode or dim("?")}')
        if not chassis:
            print('    ' + bad('-> no /robot/telemetry. esp32-bridge down, chassis USB '
                               'unplugged, or domain/Zenoh mismatch. See OPERATIONS §A/§B.'))
        # Arm board: visible via its sensor stream (needs the enable mask on) or lifecycle.
        arm_seen = self.r_mag.fresh() or self.r_therm.fresh() or self.arm_state is not None
        print(f'{led("ok" if (self.r_mag.fresh() or self.r_therm.fresh()) else ("warn" if arm_seen else "bad"))} '
              f'arm PCB: mag={self.r_mag.hz():.0f}Hz thermal={self.r_therm.hz():.0f}Hz '
              f'state={self.arm_state or dim("?")}')
        if not (self.r_mag.fresh() or self.r_therm.fresh()):
            print('    ' + dim('-> no arm-PCB sensor stream. If the arm works, the '
                               'Thermal/Mag enable toggles are just off. Else arm USB down.'))
        if self.flags is not None:
            b = flag_bits(self.flags)
            print(f'    flags: minipc_connected={led("ok" if b["minipc_connected"] else "bad")} '
                  f'ppm_ok={led("ok" if b["ppm_ok"] else "warn")} '
                  f'can_ok={led("ok" if b["can_ok"] else "bad")} '
                  f'estop={led("bad" if b["estop"] else "ok")}'
                  f'{bad("  E-STOP ACTIVE") if b["estop"] else ""}')
        else:
            print('    ' + bad('flags: none (no /robot/flags)'))


# ── PPM / RC ────────────────────────────────────────────────────────────────────
class PpmDiag(Node):
    def __init__(self):
        super().__init__('diag_ppm')
        self.ppm = None
        self.flags = None
        self.r = Rate()
        self.create_subscription(Int16MultiArray, '/robot/ppm', self._ppm, sensor_qos())
        self.create_subscription(UInt8, '/robot/flags', self._flags, sensor_qos())

    def _ppm(self, m):
        self.ppm = list(m.data)
        self.r.tick()

    def _flags(self, m): self.flags = int(m.data)

    @staticmethod
    def _bar(us):
        # 1000..2000 us -> a 20-wide bar.
        frac = max(0.0, min(1.0, (us - 1000) / 1000.0))
        n = int(round(frac * 20))
        return '[' + '#' * n + '-' * (20 - n) + ']'

    def report(self):
        print('\n=== PPM / RC link ===')
        ppm_ok = self.flags is not None and bool(self.flags & 0x01)
        if not self.r.fresh():
            print(bad('● no /robot/ppm — chassis PCB not reporting (see diag_link).'))
            return
        print(f'{led("ok" if ppm_ok else "bad")} ppm_ok={ppm_ok}   '
              f'frames {self.r.hz():.0f} Hz')
        if not ppm_ok:
            print('    ' + bad('-> RC failsafe. FlySky TX off / not bound, or PPM wire '
                               '(chassis GPIO4). Tracks stop, flippers hold.'))
        if self.flags is not None and (self.flags & 0x08):
            print('    ' + bad('Ch6 is DOWN — E-STOP active (return lever to centre).'))
        if self.flags is not None and (self.flags & 0x10):
            print('    ' + warn('Ch6 is UP — virtual-flip (REVERSE) engaged.'))
        for i, role in enumerate(PPM_ROLES):
            us = self.ppm[i] if self.ppm and i < len(self.ppm) else 0
            stale = us < 800 or us > 2200
            tag = bad('??') if stale else f'{us:4d}us'
            print(f'    {self._bar(us)} {tag}  {dim(role)}')


# ── CAN presence ────────────────────────────────────────────────────────────────
class CanDiag(Node):
    def __init__(self):
        super().__init__('diag_can')
        self.vesc: dict[int, tuple] = {}     # vid -> (last_t, vin, tfet, tmot, erpm)
        self.vesc_rate: dict[int, Rate] = {}
        self.flags = None
        self.arm_presence = None
        self.arm_state = None
        self.odrv_err: dict[int, float] = {}
        self.create_subscription(Float32MultiArray, '/motors/vesc_status', self._vesc, sensor_qos())
        self.create_subscription(UInt8, '/robot/flags', self._flags, sensor_qos())
        self.create_subscription(UInt16, '/arm/can_presence', self._presence, latched_qos())
        self.create_subscription(String, '/arm/state', self._arm, latched_qos())
        self.create_subscription(Float32MultiArray, '/motors/odrive_error', self._oerr, sensor_qos())

    def _vesc(self, m):
        if len(m.data) < 8:
            return
        vid = int(m.data[0])
        self.vesc[vid] = (time.monotonic(), m.data[6], m.data[4], m.data[5], m.data[1])
        self.vesc_rate.setdefault(vid, Rate()).tick()

    def _flags(self, m): self.flags = int(m.data)
    def _presence(self, m): self.arm_presence = int(m.data)
    def _arm(self, m): self.arm_state = m.data

    def _oerr(self, m):
        if len(m.data) >= 2 and int(m.data[1]) != 0:
            self.odrv_err[int(m.data[0])] = m.data[1]

    def report(self):
        print('\n=== CAN presence ===')
        can_ok = self.flags is not None and bool(self.flags & 0x04)
        print(f'{led("ok" if can_ok else ("warn" if self.flags is not None else "bad"))} '
              f'chassis can_ok={can_ok}')
        print(dim('  base VESC bus (chassis PCB):'))
        for vid, label in EXPECTED_VESC.items():
            r = self.vesc_rate.get(vid)
            fresh = r is not None and r.fresh()
            if fresh:
                _, vin, tfet, tmot, erpm = self.vesc[vid]
                print(f'    {led("ok")} {label:11s} id={vid:<3d} '
                      f'{r.hz():.0f}Hz  Vin={vin:4.1f}V fet={tfet:4.1f}C erpm={erpm:.0f}')
            else:
                print(f'    {led("bad")} {label:11s} id={vid:<3d} {bad("MISSING")}')
        miss = [l for v, l in EXPECTED_VESC.items()
                if not (self.vesc_rate.get(v) and self.vesc_rate[v].fresh())]
        if miss:
            print('    ' + bad(f'-> {len(miss)} VESC(s) silent: check 500 kbps CAN baud, '
                               'the VESC id, power, and CAN wiring (architecture §8.1).'))

        print(dim('  arm CAN bus (arm PCB):'))
        if self.arm_presence is None:
            print(f'    {led("warn")} no /arm/can_presence yet — '
                  + dim('arm boots DISARMED; click Arm so it probes the bus.'))
        else:
            print(f'    arm_state={self.arm_state or "?"}  presence_mask=0x{self.arm_presence:04x}')
            for bit, label in ARM_PRESENCE_BITS.items():
                present = bool(self.arm_presence & (1 << bit))
                print(f'    {led("ok" if present else "bad")} {label}'
                      + ('' if present else bad('  MISSING')))
            absent = [l for b, l in ARM_PRESENCE_BITS.items()
                      if not (self.arm_presence & (1 << b))]
            if absent:
                print('    ' + bad('-> ODrive defaults to 250 kbps (set 500), wrong node id, '
                                   'unpowered, or a latched CAN fault (re-Arm). §8.2-8.4.'))
        if self.odrv_err:
            for n, e in sorted(self.odrv_err.items()):
                print(f'    {warn("ODrive err")} node 0x{n:02x}: 0x{int(e):08x}')


# ── arm-PCB I2C sensors ─────────────────────────────────────────────────────────
class SensorsDiag(Node):
    def __init__(self):
        super().__init__('diag_sensors')
        self.r_mag = Rate()
        self.r_therm = Rate()
        self.mag_xyz = None
        self.tstat = None
        self.create_subscription(MagneticField, '/sensors/mag', self._mag, sensor_qos())
        if _HAVE_THERMAL:
            self.create_subscription(ThermalStatus, '/sensors/thermal_status', self._tstat, sensor_qos())
        else:
            self.create_subscription(__import__('sensor_msgs.msg', fromlist=['Image']).Image,
                                     '/sensors/thermal', lambda m: self.r_therm.tick(), sensor_qos())

    def _mag(self, m):
        self.r_mag.tick()
        f = m.magnetic_field
        self.mag_xyz = (f.x, f.y, f.z)

    def _tstat(self, m):
        self.r_therm.tick()
        self.tstat = m

    def report(self):
        print('\n=== arm-PCB I2C sensors ===')
        print(dim('  (MLX90640 thermal + LIS3MDL mag live on the ARM PCB, relayed by the '
                  'bridge — no Jetson I2C.)'))
        # Magnetometer
        if self.r_mag.fresh():
            x, y, z = self.mag_xyz
            print(f'{led("ok")} mag: {self.r_mag.hz():.0f} Hz   '
                  f'B=({x*1e6:+.0f}, {y*1e6:+.0f}, {z*1e6:+.0f}) µT')
        else:
            print(f'{led("bad")} mag: {bad("no data")}  '
                  + dim('-> Mag enable toggle off, or arm-PCB link/I2C down (§E).'))
        # Thermal
        if self.tstat is not None and self.r_therm.fresh():
            t = self.tstat
            state = 'ok' if getattr(t, 'sensor_ok', True) else 'bad'
            print(f'{led(state)} thermal: {self.r_therm.hz():.1f} Hz   '
                  f'sensor_ok={getattr(t, "sensor_ok", "?")}  '
                  f'min/max/center={t.min_temperature:.1f}/{t.max_temperature:.1f}/'
                  f'{t.center_temperature:.1f}°C  errs={t.consecutive_read_errors}')
            if t.consecutive_read_errors > 3:
                print('    ' + warn('-> consecutive I2C read errors — check MLX90640 wiring / '
                                    'SENSOR_I2C_HZ on the arm PCB.'))
        elif self.r_therm.fresh():
            print(f'{led("ok")} thermal: {self.r_therm.hz():.1f} Hz (no status msg)')
        else:
            print(f'{led("bad")} thermal: {bad("no data")}  '
                  + dim('-> Thermal enable toggle off, or arm-PCB link/I2C down (§E).'))


# ── one-shot aggregate ──────────────────────────────────────────────────────────
def _snapshot(seconds: float):
    """Bring up every diag node briefly and print a combined snapshot."""
    nodes = [LinkDiag(), PpmDiag(), CanDiag(), SensorsDiag()]
    print(dim(f'(collecting {seconds:.0f}s — make sure esp32-bridge is running on the robot)'))
    end = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < end:
        for n in nodes:
            rclpy.spin_once(n, timeout_sec=0.02)
    print('\n' + ok('==== RoboCorea diagnostic snapshot ===='))
    for n in nodes:
        n.report()
        n.destroy_node()
    print('\n' + dim('Live monitors: ./diagnose.sh link|ppm|can|sensors'))
    if rclpy.ok():
        rclpy.shutdown()


def _parse_args(argv):
    watch = '--watch' in argv or '-w' in argv
    once = '--once' in argv
    seconds = 6.0
    for i, a in enumerate(argv):
        if a in ('--seconds', '-s') and i + 1 < len(argv):
            try:
                seconds = float(argv[i + 1])
            except ValueError:
                pass
    return watch, seconds, once


def _entry(node_cls, default_watch):
    rclpy.init()
    watch, seconds, once = _parse_args(sys.argv[1:])
    # --once forces a single snapshot; otherwise --watch or the per-command default.
    final_watch = False if once else (watch or default_watch)
    _run(node_cls(), final_watch, seconds)


# Console entry points (registered in setup.py). The focused ones default to LIVE.
def main_link(): _entry(LinkDiag, default_watch=True)
def main_ppm(): _entry(PpmDiag, default_watch=True)
def main_can(): _entry(CanDiag, default_watch=True)
def main_sensors(): _entry(SensorsDiag, default_watch=True)


def main_all():
    rclpy.init()
    _, seconds, _ = _parse_args(sys.argv[1:])
    _snapshot(seconds)


if __name__ == '__main__':
    main_all()
