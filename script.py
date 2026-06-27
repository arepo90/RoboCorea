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
import fcntl
import glob
import os
import sys
import time

WIDTH, HEIGHT = 32, 24            # MLX90640 native resolution
NPIX = WIDTH * HEIGHT             # 768
CENTER = (HEIGHT // 2) * WIDTH + (WIDTH // 2)
I2C_SLAVE = 0x0703               # linux/i2c-dev.h ioctl
DEFAULT_ADDR = 0x33

RAMP = " .:-=+*o#%@"              # cold -> hot, used in --no-color mode

REFRESH_RATES = {1, 2, 4, 8, 16, 32, 64}


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
# Step 2: open the Adafruit driver (this reads the EEPROM -> the flaky part)   #
# --------------------------------------------------------------------------- #
def open_sensor(bus, addr, refresh, attempts=5):
    try:
        from adafruit_extended_bus import ExtendedI2C as I2C
        import adafruit_mlx90640
    except ImportError as exc:
        sys.exit('\nMissing driver: %s\n'
                 'Install with:\n'
                 '  pip3 install adafruit-circuitpython-mlx90640 adafruit-extended-bus'
                 % exc)

    last = None
    for i in range(attempts):
        try:
            mlx = adafruit_mlx90640.MLX90640(I2C(bus), address=addr)
            mlx.refresh_rate = getattr(
                adafruit_mlx90640.RefreshRate, 'REFRESH_%d_HZ' % refresh)
            if i:
                print('  EEPROM/init OK on attempt %d' % (i + 1))
            else:
                print('  EEPROM/init OK')
            return mlx
        except Exception as exc:                       # noqa: BLE001
            last = exc
            print('  init attempt %d/%d failed: %s: %s'
                  % (i + 1, attempts, type(exc).__name__, exc))
            time.sleep(0.5)
    sys.exit('\nCould not initialise the MLX90640 after %d attempts: %s\n'
             'The device answered the bus scan but the driver could not read it '
             '-- usually a noisy/long I2C wire, a pull-up, or a too-high bus '
             'speed.' % (attempts, last))


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
