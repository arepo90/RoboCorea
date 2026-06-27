#!/usr/bin/env python3
"""Standalone MLX90640 thermal-camera tester (no ROS).

Run this directly on the Jetson to check whether the thermal camera is
reading correctly, independent of the ROS 2 stack.

What it does, in order:
  1. Scans every /dev/i2c-* bus for a device at the MLX90640 address (raw
     ioctl, no driver) and reports which bus(es) respond.
  2. On the chosen bus, opens the Adafruit driver. The constructor reads the
     EEPROM over I2C -- this is the step that is flaky on a noisy bus -- so it
     is retried and any failure is reported clearly.
  3. Streams frames and draws a live 32x24 thermal image in the terminal with
     min / max / center / hotspot temperatures, FPS, and a running read-error
     count so you can quantify "not working well".

Press Ctrl-C to stop; a summary is printed on exit.

Deps (same as the ROS node): adafruit-circuitpython-mlx90640, adafruit-extended-bus
    pip3 install adafruit-circuitpython-mlx90640 adafruit-extended-bus

Examples:
    python3 script.py                 # auto-scan buses, live color image
    python3 script.py --bus 7         # force bus 7
    python3 script.py --no-color      # plain ASCII ramp (e.g. over plain ssh)
    python3 script.py --once          # grab a few frames, print stats, exit
    python3 script.py --refresh 4     # slower refresh rate (1,2,4,8,16,32,64 Hz)
"""

import argparse
import ctypes
import fcntl
import glob
import os
import signal
import sys
import time
import warnings

WIDTH, HEIGHT = 32, 24            # MLX90640 native resolution
NPIX = WIDTH * HEIGHT             # 768
CENTER = (HEIGHT // 2) * WIDTH + (WIDTH // 2)
I2C_SLAVE = 0x0703               # linux/i2c-dev.h ioctl
I2C_RDWR = 0x0707
I2C_M_RD = 0x0001
DEFAULT_ADDR = 0x33

RAMP = " .:-=+*o#%@"              # cold -> hot, used in --no-color mode

REFRESH_RATES = {1, 2, 4, 8, 16, 32, 64}

# adafruit_blinka warns "I2C frequency is not settable in python" -- harmless noise.
warnings.filterwarnings('ignore', message='.*I2C frequency.*')


# --------------------------------------------------------------------------- #
# Hard timeout so a wedged driver init / read can never hang the script        #
# --------------------------------------------------------------------------- #
class Timeout(Exception):
    pass


def with_timeout(seconds, fn, *args, **kwargs):
    def _raise(_signum, _frame):
        raise Timeout()
    old = signal.signal(signal.SIGALRM, _raise)
    signal.setitimer(signal.ITIMER_REAL, seconds)
    try:
        return fn(*args, **kwargs)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, old)


# --------------------------------------------------------------------------- #
# Raw 16-bit register / EEPROM read over I2C (combined write-addr + read,      #
# repeated start) -- exactly how the MLX90640 must be addressed.               #
# --------------------------------------------------------------------------- #
class _i2c_msg(ctypes.Structure):
    _fields_ = [('addr', ctypes.c_uint16), ('flags', ctypes.c_uint16),
                ('len', ctypes.c_uint16), ('buf', ctypes.POINTER(ctypes.c_uint8))]


class _i2c_rdwr(ctypes.Structure):
    _fields_ = [('msgs', ctypes.POINTER(_i2c_msg)), ('nmsgs', ctypes.c_uint32)]


def read_words(fd, addr, reg, nwords):
    """Read nwords 16-bit big-endian words starting at 16-bit register `reg`."""
    wbuf = (ctypes.c_uint8 * 2)((reg >> 8) & 0xFF, reg & 0xFF)
    rbuf = (ctypes.c_uint8 * (nwords * 2))()
    msgs = (_i2c_msg * 2)(
        _i2c_msg(addr, 0, 2, ctypes.cast(wbuf, ctypes.POINTER(ctypes.c_uint8))),
        _i2c_msg(addr, I2C_M_RD, nwords * 2, ctypes.cast(rbuf, ctypes.POINTER(ctypes.c_uint8))),
    )
    fcntl.ioctl(fd, I2C_RDWR, _i2c_rdwr(msgs, ctypes.c_uint32(2)))
    return [(rbuf[i * 2] << 8) | rbuf[i * 2 + 1] for i in range(nwords)]


def read_bytes(fd, addr, reg, n=1):
    """Read n bytes from 8-bit sub-register `reg` (repeated-start combined xfer)."""
    wbuf = (ctypes.c_uint8 * 1)(reg & 0xFF)
    rbuf = (ctypes.c_uint8 * n)()
    msgs = (_i2c_msg * 2)(
        _i2c_msg(addr, 0, 1, ctypes.cast(wbuf, ctypes.POINTER(ctypes.c_uint8))),
        _i2c_msg(addr, I2C_M_RD, n, ctypes.cast(rbuf, ctypes.POINTER(ctypes.c_uint8))),
    )
    fcntl.ioctl(fd, I2C_RDWR, _i2c_rdwr(msgs, ctypes.c_uint32(2)))
    return list(rbuf)


def write_byte(fd, addr, reg, val):
    wbuf = (ctypes.c_uint8 * 2)(reg & 0xFF, val & 0xFF)
    msgs = (_i2c_msg * 1)(
        _i2c_msg(addr, 0, 2, ctypes.cast(wbuf, ctypes.POINTER(ctypes.c_uint8))))
    fcntl.ioctl(fd, I2C_RDWR, _i2c_rdwr(msgs, ctypes.c_uint32(1)))


# --------------------------------------------------------------------------- #
# Cross-check: read the LIS3MDL magnetometer on the SAME bus. If it reads      #
# cleanly, the I2C bus/controller is fine and the MLX90640 (or its wiring) is  #
# the fault; if it ALSO fails, the whole bus is suspect.                       #
# --------------------------------------------------------------------------- #
LIS3MDL_WHOAMI = 0x3D
LIS3MDL_SENS_UT = 100.0 / 6842.0   # raw -> microtesla at +/-4 gauss full scale


def check_magnetometer(bus, addrs=(0x1C, 0x1E), samples=5):
    print('\n-- Magnetometer cross-check (LIS3MDL on bus %d) --' % bus)
    try:
        fd = os.open('/dev/i2c-%d' % bus, os.O_RDWR)
    except OSError as exc:
        print('  cannot open /dev/i2c-%d: %s' % (bus, exc))
        return None
    try:
        addr = who = None
        for a in addrs:
            try:
                w = read_bytes(fd, a, 0x0F, 1)[0]
                print('  0x%02x WHO_AM_I = 0x%02X (expect 0x3D)' % (a, w))
                if w == LIS3MDL_WHOAMI:
                    addr, who = a, w
                    break
            except OSError as exc:
                print('  0x%02x : no response (%s)' % (a, exc))
        if addr is None:
            print('  VERDICT: magnetometer not readable -> bus/controller itself is '
                  'suspect (not just the MLX90640).')
            return False

        # Configure: ultra-high-perf XY/Z, 10 Hz, +/-4 gauss, continuous-conversion.
        try:
            write_byte(fd, addr, 0x20, 0x70)   # CTRL1: OM=UHP, DO=10Hz, temp off
            write_byte(fd, addr, 0x21, 0x00)   # CTRL2: FS = +/-4 gauss
            write_byte(fd, addr, 0x23, 0x0C)   # CTRL4: OMZ = UHP
            write_byte(fd, addr, 0x22, 0x00)   # CTRL3: continuous-conversion
        except OSError as exc:
            print('  config write failed: %s -> bus is unreliable for writes too.' % exc)
            return False
        time.sleep(0.15)

        def s16(lo, hi):
            v = lo | (hi << 8)
            return v - 65536 if v >= 32768 else v

        mags, errs = [], 0
        for _ in range(samples):
            try:
                d = read_bytes(fd, addr, 0x28 | 0x80, 6)   # 0x80 = auto-increment
                x, y, z = (s16(d[0], d[1]) * LIS3MDL_SENS_UT,
                           s16(d[2], d[3]) * LIS3MDL_SENS_UT,
                           s16(d[4], d[5]) * LIS3MDL_SENS_UT)
                mag = (x * x + y * y + z * z) ** 0.5
                mags.append(mag)
                print('  X %+7.1f  Y %+7.1f  Z %+7.1f uT   |B| %6.1f uT' % (x, y, z, mag))
            except OSError as exc:
                errs += 1
                print('  read error: %s' % exc)
            time.sleep(0.1)

        # Earth's field is ~25-65 uT; near motors/metal it can read higher but
        # should be stable and non-zero. Mainly we care that reads succeed.
        if errs == 0 and mags and not all(m == mags[0] for m in mags):
            plausible = 5.0 < (sum(mags) / len(mags)) < 1000.0
            print('  VERDICT: magnetometer reads CLEANLY on bus %d%s.' %
                  (bus, '' if plausible else ' (values odd, but I2C transfer works)'))
            print('  => The I2C bus is healthy; the MLX90640 device/wiring/pull-ups '
                  'is the fault, not the Jetson I2C controller.')
            return True
        print('  VERDICT: magnetometer reads are failing/stuck too -> the I2C bus '
              'or controller (not just the MLX90640) is the problem.')
        return False
    finally:
        os.close(fd)


# --------------------------------------------------------------------------- #
# Step 1: raw I2C bus scan (no driver, just ioctl + a 1-byte read)            #
# --------------------------------------------------------------------------- #
def i2c_present(busnum, addr):
    """Return True/False if a device ACKs at addr on /dev/i2c-<busnum>.

    Returns None if the bus node can't be opened (missing / no permission).
    """
    path = '/dev/i2c-%d' % busnum
    try:
        fd = os.open(path, os.O_RDWR)
    except OSError:
        return None
    try:
        fcntl.ioctl(fd, I2C_SLAVE, addr)
        try:
            os.read(fd, 1)        # MLX90640 ACKs a read; absent device -> OSError
            return True
        except OSError:
            return False
    finally:
        os.close(fd)


def scan_buses(addr):
    buses = sorted(int(p.rsplit('-', 1)[1]) for p in glob.glob('/dev/i2c-*')
                   if p.rsplit('-', 1)[1].isdigit())
    if not buses:
        print('  no /dev/i2c-* nodes found -- is i2c enabled / are you on the Jetson?')
        return []
    found = []
    for b in buses:
        present = i2c_present(b, addr)
        if present is None:
            mark = '?? cannot open (try sudo / i2c group)'
        elif present:
            mark = 'DEVICE PRESENT at 0x%02x  <--' % addr
            found.append(b)
        else:
            mark = 'no device at 0x%02x' % addr
        print('  /dev/i2c-%-2d : %s' % (b, mark))
    return found


# --------------------------------------------------------------------------- #
# Step 2: raw register/EEPROM diagnostic -- the driver hangs on corrupt EEPROM #
# --------------------------------------------------------------------------- #
def diagnose(bus, addr, samples=4):
    """Read key registers + an EEPROM sample a few times; judge I2C integrity."""
    path = '/dev/i2c-%d' % bus
    try:
        fd = os.open(path, os.O_RDWR)
    except OSError as exc:
        print('  cannot open %s: %s' % (path, exc))
        return False
    try:
        fcntl.ioctl(fd, I2C_SLAVE, addr)
        # Control reg 0x800D defaults to ~0x1901; status 0x8000. Both should be
        # stable across reads. EEPROM 0x2400.. should be varied & non-zero.
        ctrl = []
        for _ in range(samples):
            try:
                ctrl.append(read_words(fd, addr, 0x800D, 1)[0])
            except OSError as exc:
                ctrl.append(None)
                print('  control-reg read failed: %s' % exc)
            time.sleep(0.02)
        shown = ' '.join('----' if v is None else '%04X' % v for v in ctrl)
        print('  control reg 0x800D x%d : %s' % (samples, shown))

        try:
            ee = read_words(fd, addr, 0x2400, 64)
        except OSError as exc:
            print('  EEPROM read failed: %s' % exc)
            print('  VERDICT: I2C reads are failing outright -- the device ACKs but '
                  'data transfer errors. Bad wiring/pull-ups/bus speed.')
            return False

        zeros = ee.count(0x0000)
        ffs = ee.count(0xFFFF)
        uniq = len(set(ee))
        print('  EEPROM 0x2400 sample  : %s ...' % ' '.join('%04X' % w for w in ee[:8]))
        print('  EEPROM 64-word health : %d zero, %d 0xFFFF, %d distinct values'
              % (zeros, ffs, uniq))

        ctrl_ok = len({v for v in ctrl if v is not None}) == 1 and ctrl[0] not in (None, 0x0000, 0xFFFF)
        ee_ok = zeros < 16 and ffs < 16 and uniq > 20
        if ctrl_ok and ee_ok:
            print('  VERDICT: I2C reads look CLEAN -- registers stable, EEPROM varied.')
            return True
        print('  VERDICT: I2C reads look CORRUPT/UNSTABLE -- this is why the driver '
              'hangs. Likely a noisy/long I2C wire, weak pull-ups, or too-high bus '
              'speed on /dev/i2c-%d.' % bus)
        return False
    finally:
        os.close(fd)


# --------------------------------------------------------------------------- #
# Step 3: open the Adafruit driver (this reads the EEPROM -> the flaky part).  #
# Guarded by a timeout because corrupt EEPROM makes its init loop forever.     #
# --------------------------------------------------------------------------- #
def open_sensor(bus, addr, refresh, attempts=5, init_timeout=6.0):
    try:
        from adafruit_extended_bus import ExtendedI2C as I2C
        import adafruit_mlx90640
    except ImportError as exc:
        sys.exit('\nMissing driver: %s\n'
                 'Install with:\n'
                 '  pip3 install adafruit-circuitpython-mlx90640 adafruit-extended-bus'
                 % exc)

    def _init():
        mlx = adafruit_mlx90640.MLX90640(I2C(bus), address=addr)
        mlx.refresh_rate = getattr(
            adafruit_mlx90640.RefreshRate, 'REFRESH_%d_HZ' % refresh)
        return mlx

    last = None
    for i in range(attempts):
        try:
            mlx = with_timeout(init_timeout, _init)
            print('  EEPROM/init OK%s' % (' on attempt %d' % (i + 1) if i else ''))
            return mlx
        except Timeout:
            last = 'init hung > %.0fs (corrupt EEPROM read -> infinite loop)' % init_timeout
            print('  init attempt %d/%d: %s' % (i + 1, attempts, last))
        except Exception as exc:                       # noqa: BLE001
            last = '%s: %s' % (type(exc).__name__, exc)
            print('  init attempt %d/%d failed: %s' % (i + 1, attempts, last))
        time.sleep(0.5)
    sys.exit('\nCould not initialise the MLX90640 after %d attempts (%s).\n'
             'The device answers the bus but its EEPROM does not read cleanly. '
             'Fix the I2C link: shorter/shielded wires, add/strengthen 4.7k pull-ups '
             'to 3.3V, and/or lower the i2c-%d bus clock (devicetree). '
             'See the diagnostic verdict above.' % (attempts, last, bus))


# --------------------------------------------------------------------------- #
# Rendering                                                                    #
# --------------------------------------------------------------------------- #
def heat_rgb(x):
    """Map x in [0,1] to a thermal-ish RGB (blue -> cyan -> yellow -> red -> white)."""
    x = 0.0 if x < 0.0 else 1.0 if x > 1.0 else x
    stops = [(0.00, (0, 0, 90)), (0.25, (0, 0, 255)), (0.50, (0, 255, 255)),
             (0.70, (255, 255, 0)), (0.88, (255, 60, 0)), (1.00, (255, 255, 255))]
    for (x0, c0), (x1, c1) in zip(stops, stops[1:]):
        if x <= x1:
            f = (x - x0) / (x1 - x0) if x1 > x0 else 0.0
            return tuple(int(c0[j] + (c1[j] - c0[j]) * f) for j in range(3))
    return stops[-1][1]


def render(frame, lo, hi, color):
    span = (hi - lo) or 1.0
    out = []
    for row in range(HEIGHT):
        line = []
        base = row * WIDTH
        for col in range(WIDTH):
            t = (frame[base + col] - lo) / span
            if color:
                r, g, b = heat_rgb(t)
                line.append('\033[48;2;%d;%d;%dm  ' % (r, g, b))
            else:
                idx = int(t * (len(RAMP) - 1) + 0.5)
                line.append(RAMP[0 if idx < 0 else len(RAMP) - 1 if idx >= len(RAMP) else idx] * 2)
        out.append(''.join(line) + ('\033[0m' if color else ''))
    return '\n'.join(out)


def frame_stats(frame):
    mn = min(frame)
    mx = max(frame)
    hot = frame.index(mx)
    return mn, mx, frame[CENTER], hot % WIDTH, hot // WIDTH


# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(description='Standalone MLX90640 thermal tester')
    ap.add_argument('--bus', type=int, default=None,
                    help='I2C bus number (default: auto-scan)')
    ap.add_argument('--address', type=lambda s: int(s, 0), default=DEFAULT_ADDR,
                    help='I2C address (default: 0x33)')
    ap.add_argument('--refresh', type=int, default=8, choices=sorted(REFRESH_RATES),
                    help='sensor refresh rate in Hz (default: 8)')
    ap.add_argument('--no-color', action='store_true',
                    help='ASCII ramp instead of truecolor blocks')
    ap.add_argument('--once', action='store_true',
                    help='grab a few frames, print a pass/fail summary, then exit')
    ap.add_argument('--range', nargs=2, type=float, metavar=('LO', 'HI'),
                    help='fixed display temp range in C (default: auto per frame)')
    ap.add_argument('--mag-address', type=lambda s: int(s, 0), default=None,
                    help='LIS3MDL address for the bus cross-check (default: probe 0x1C/0x1E)')
    ap.add_argument('--no-mag', action='store_true',
                    help='skip the magnetometer (LIS3MDL) bus cross-check')
    ap.add_argument('--no-diagnose', action='store_true',
                    help='skip the raw MLX90640 EEPROM diagnostic')
    args = ap.parse_args()
    color = not args.no_color

    print('== MLX90640 thermal tester ==')
    print('Scanning I2C buses for 0x%02x ...' % args.address)
    if args.bus is not None:
        bus = args.bus
        print('  (bus %d forced via --bus; scan skipped)' % bus)
    else:
        found = scan_buses(args.address)
        if not found:
            sys.exit('\nNo MLX90640 found on any I2C bus. Check wiring/power, or '
                     'pass --bus N explicitly.')
        bus = found[0]
        if len(found) > 1:
            print('  multiple buses responded %s; using %d' % (found, bus))

    # Cross-check the bus with the magnetometer + raw EEPROM read BEFORE the
    # driver, so the diagnosis prints even if the driver init bails out.
    if not args.no_mag:
        addrs = (args.mag_address,) if args.mag_address else (0x1C, 0x1E)
        check_magnetometer(bus, addrs)
    if not args.no_diagnose:
        print('\n-- MLX90640 raw EEPROM diagnostic (bus %d, 0x%02x) --' % (bus, args.address))
        diagnose(bus, args.address)

    print('\nOpening sensor on bus %d (reads EEPROM) ...' % bus)
    mlx = open_sensor(bus, args.address, args.refresh)

    frame = [0.0] * NPIX
    reads = errors = consec = 0
    start = time.monotonic()
    last_t = start

    # --- one-shot mode ---------------------------------------------------- #
    if args.once:
        print('\nGrabbing 5 frames ...')
        last = None
        for _ in range(5):
            try:
                mlx.getFrame(frame)
                reads += 1
                last = list(frame)
            except Exception as exc:                   # noqa: BLE001
                errors += 1
                print('  read error: %s: %s' % (type(exc).__name__, exc))
        if reads and last:
            mn, mx, c, hx, hy = frame_stats(last)
            fps = reads / (time.monotonic() - start)
            print('\nOK: %d/%d frames read (%d errors), ~%.1f FPS' % (reads, reads + errors, errors, fps))
            print('    min %.1fC  max %.1fC  center %.1fC  hotspot=(%d,%d)' % (mn, mx, c, hx, hy))
            sys.exit(0 if errors == 0 else 1)
        sys.exit('\nFAIL: no frames read.')

    # --- live mode -------------------------------------------------------- #
    print('\nStreaming -- Ctrl-C to stop.\n')
    time.sleep(0.6)
    sys.stdout.write('\033[2J')                        # clear once
    try:
        while True:
            try:
                mlx.getFrame(frame)
                reads += 1
                consec = 0
            except Exception as exc:                   # noqa: BLE001
                errors += 1
                consec += 1
                sys.stdout.write('\033[H\033[2K read error #%d (%d in a row): %s: %s\n'
                                 % (errors, consec, type(exc).__name__, exc))
                sys.stdout.flush()
                continue

            now = time.monotonic()
            fps = 1.0 / (now - last_t) if now > last_t else 0.0
            last_t = now
            mn, mx, c, hx, hy = frame_stats(frame)
            if args.range:
                lo, hi = args.range
            else:
                lo, hi = mn, mx
                if hi - lo < 3.0:                      # keep a flat/noisy scene from looking dramatic
                    hi = lo + 3.0

            sys.stdout.write('\033[H')                 # cursor home (no flicker)
            sys.stdout.write(
                '\033[2K frame %d  fps %4.1f  min %5.1f  max %5.1f  center %5.1f  '
                'hotspot=(%2d,%2d)  errors %d\n'
                % (reads, fps, mn, mx, c, hx, hy, errors))
            sys.stdout.write('\033[2K scale %.1fC -> %.1fC%s\n'
                             % (lo, hi, '  (fixed)' if args.range else ''))
            sys.stdout.write(render(frame, lo, hi, color) + '\n')
            sys.stdout.flush()
    except KeyboardInterrupt:
        elapsed = time.monotonic() - start
        if color:
            sys.stdout.write('\033[0m')
        print('\n\n-- summary --')
        print('frames read : %d' % reads)
        print('read errors : %d' % errors)
        print('avg FPS     : %.1f' % (reads / elapsed if elapsed else 0.0))
        print('elapsed     : %.1fs' % elapsed)


if __name__ == '__main__':
    main()
