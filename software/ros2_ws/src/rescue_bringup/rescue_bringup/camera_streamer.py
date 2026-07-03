#!/usr/bin/env python3
"""camera_streamer — robot-side multi-camera streamer + live catalog.

Auto-detects every V4L2 UVC **capture** camera on the Jetson and ships each to
the operator GUI over one of two transports (the ``transport`` parameter):

  * ``srt`` (default) — H.264 in MPEG-TS over SRT, one GStreamer subprocess per
    camera, video-only. The designated **primary** camera (the C920 Pro,
    matched by name) also gets its microphone muxed in as Opus — the A/V
    stream the GUI decodes for the speaker + Vosk.
  * ``ros2`` — every camera (primary included) publishes **JPEG
    ``sensor_msgs/CompressedImage``** on ``<topic_prefix>/<slug>/compressed``.
    Cameras that emit MJPEG natively are *passed through* (no re-encode, ~zero
    CPU); raw-only cameras are JPEG-encoded on the Jetson. The primary's mic
    still rides SRT (an audio-only MPEG-TS on the primary port, which the
    GUI's GstAvStream tolerates) — audio does not fit CompressedImage.

Shared behaviour in both modes:

  * Each camera's formats/sizes/rates are **probed** (V4L2 ioctls) and the
    capture caps are requested **explicitly** — MJPEG preferred, raw fallback —
    at the closest supported mode at or below the configured target, in the
    **sensor's native aspect ratio** (max FOV, `prefer_native_fov`: other
    aspects are centre-crops). Never let GStreamer negotiate freely: on these
    cameras that picks their *maximum* raw-YUYV mode (e.g. 1080p @ 5 fps),
    which both looks like a slideshow and saturates the single shared USB 2.0
    bus, starving every other camera.
  * Multi-node devices (RealSense & co.) get the right node: depth/IR-only
    nodes (Z16/GREY/Y16…) are skipped, colour-capable nodes win.
  * The **ZED is a plain UVC camera** now that the ZED SDK/wrapper is retired:
    it is scanned like any other camera, and since its raw frame is
    **side-by-side stereo** (both eyes in one image), cameras matching
    ``sbs_match`` are cropped to the **left eye** before encode/publish. The
    old SDK arrangement is kept dormant: set ``exclude_match:='ZED'`` +
    ``zed_enable:=true`` to exclude the device and tap the SDK's published
    left-image topic instead.
  * A **latched JSON catalog** on ``/robot/camera_streams`` advertises the
    live video streams (name + transport + port/topic) so the GUI
    auto-populates its source list and follows hot-plug (periodic rescan).
  * Subprocess pipelines log to ``/tmp/camera_streamer/<slug>.log``; when one
    dies the log tail is surfaced in the node log (they used to be silently
    discarded, which made USB-bandwidth failures invisible).
"""

import array
import ctypes
import fcntl
import glob
import json
import os
import re
import signal
import subprocess
import threading
import time
from fractions import Fraction

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

# ── V4L2 probing (ioctls, no external deps) ──────────────────────────────────
# We read the human name from sysfs (no device open needed, so a busy ZED never
# blocks us), VIDIOC_QUERYCAP the survivors to confirm they capture video and to
# get a stable bus_info (dedupes a camera's multiple /dev/video* nodes and keeps
# its port stable across rescans), and enumerate formats/sizes/intervals so the
# pipelines can request an exact, known-good capture mode.

_V4L2_CAP_VIDEO_CAPTURE = 0x00000001
_V4L2_CAP_DEVICE_CAPS = 0x80000000
_V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
# _IOR/_IOWR('V', nr, struct) with the struct sizes baked in (64-bit Linux).
_VIDIOC_QUERYCAP = 0x80685600            # struct v4l2_capability, 104 bytes
_VIDIOC_ENUM_FMT = 0xC0405602            # struct v4l2_fmtdesc, 64 bytes
_VIDIOC_ENUM_FRAMESIZES = 0xC02C564A     # struct v4l2_frmsizeenum, 44 bytes
_VIDIOC_ENUM_FRAMEINTERVALS = 0xC034564B  # struct v4l2_frmivalenum, 52 bytes
_V4L2_FRMSIZE_TYPE_DISCRETE = 1
_V4L2_FRMIVAL_TYPE_DISCRETE = 1


class _v4l2_capability(ctypes.Structure):
    _fields_ = [
        ('driver', ctypes.c_char * 16),
        ('card', ctypes.c_char * 32),
        ('bus_info', ctypes.c_char * 32),
        ('version', ctypes.c_uint32),
        ('capabilities', ctypes.c_uint32),
        ('device_caps', ctypes.c_uint32),
        ('reserved', ctypes.c_uint32 * 3),
    ]


class _v4l2_fmtdesc(ctypes.Structure):
    _fields_ = [
        ('index', ctypes.c_uint32),
        ('type', ctypes.c_uint32),
        ('flags', ctypes.c_uint32),
        ('description', ctypes.c_char * 32),
        ('pixelformat', ctypes.c_uint32),
        ('mbus_code', ctypes.c_uint32),
        ('reserved', ctypes.c_uint32 * 3),
    ]


class _v4l2_frmsize_discrete(ctypes.Structure):
    _fields_ = [('width', ctypes.c_uint32), ('height', ctypes.c_uint32)]


class _v4l2_frmsize_stepwise(ctypes.Structure):
    _fields_ = [
        ('min_width', ctypes.c_uint32), ('max_width', ctypes.c_uint32),
        ('step_width', ctypes.c_uint32),
        ('min_height', ctypes.c_uint32), ('max_height', ctypes.c_uint32),
        ('step_height', ctypes.c_uint32),
    ]


class _v4l2_frmsizeenum(ctypes.Structure):
    class _u(ctypes.Union):
        _fields_ = [('discrete', _v4l2_frmsize_discrete),
                    ('stepwise', _v4l2_frmsize_stepwise)]
    _fields_ = [
        ('index', ctypes.c_uint32),
        ('pixel_format', ctypes.c_uint32),
        ('type', ctypes.c_uint32),
        ('u', _u),
        ('reserved', ctypes.c_uint32 * 2),
    ]


class _v4l2_fract(ctypes.Structure):
    _fields_ = [('numerator', ctypes.c_uint32), ('denominator', ctypes.c_uint32)]


class _v4l2_frmival_stepwise(ctypes.Structure):
    _fields_ = [('min', _v4l2_fract), ('max', _v4l2_fract), ('step', _v4l2_fract)]

class _v4l2_frmivalenum(ctypes.Structure):
    class _u(ctypes.Union):
        _fields_ = [('discrete', _v4l2_fract), ('stepwise', _v4l2_frmival_stepwise)]
    _fields_ = [
        ('index', ctypes.c_uint32),
        ('pixel_format', ctypes.c_uint32),
        ('width', ctypes.c_uint32),
        ('height', ctypes.c_uint32),
        ('type', ctypes.c_uint32),
        ('u', _u),
        ('reserved', ctypes.c_uint32 * 2),
    ]


def _fourcc_to_str(v: int) -> str:
    return ''.join(chr((v >> (8 * i)) & 0xFF) for i in range(4)).rstrip()


def _str_to_fourcc(s: str) -> int:
    s = (s + '    ')[:4]
    return sum(ord(c) << (8 * i) for i, c in enumerate(s))


# Capture-format preference. MJPEG wins by a wide margin: it is ~10× lighter on
# the (single, shared) USB 2.0 bus than raw and, in ros2 mode, publishable with
# no re-encode at all. Formats not listed are unusable for streaming.
_FMT_SCORE = {'MJPG': 100, 'YUYV': 50, 'RGB3': 46, 'BGR3': 45,
              'UYVY': 40, 'NV12': 35, 'YU12': 34, 'YV12': 33}
# Formats that mark a depth/IR node (RealSense depth = Z16, IR = GREY/Y8…).
# A node exposing any of these is penalised so a device's colour node wins,
# and a node with *only* these is skipped outright.
_DEPTH_FMTS = {'Z16', 'GREY', 'Y16', 'Y8', 'Y8I', 'Y10', 'Y12', 'Y12I'}
_GST_RAW_FMT = {'YUYV': 'YUY2', 'UYVY': 'UYVY', 'NV12': 'NV12',
                'YU12': 'I420', 'YV12': 'YV12', 'RGB3': 'RGB', 'BGR3': 'BGR'}


def _sysfs_name(dev_path: str) -> str:
    """Card name from sysfs; readable even while the device is busy (ZED)."""
    node = os.path.basename(dev_path)
    try:
        with open(f'/sys/class/video4linux/{node}/name', 'r') as f:
            return f.read().strip()
    except OSError:
        return ''


def _probe_node(dev_path: str):
    """(card, bus_info, {fourcc,…}) if the node captures video, else None."""
    try:
        fd = os.open(dev_path, os.O_RDWR | os.O_NONBLOCK)
    except OSError:
        return None
    try:
        cap = _v4l2_capability()
        fcntl.ioctl(fd, _VIDIOC_QUERYCAP, cap)
        caps = cap.device_caps if (cap.capabilities & _V4L2_CAP_DEVICE_CAPS) \
            else cap.capabilities
        if not (caps & _V4L2_CAP_VIDEO_CAPTURE):
            return None
        fmts = set()
        for idx in range(32):
            f = _v4l2_fmtdesc()
            f.index = idx
            f.type = _V4L2_BUF_TYPE_VIDEO_CAPTURE
            try:
                fcntl.ioctl(fd, _VIDIOC_ENUM_FMT, f)
            except OSError:
                break
            fmts.add(_fourcc_to_str(f.pixelformat))
        return (cap.card.decode(errors='replace'),
                cap.bus_info.decode(errors='replace'), fmts)
    except OSError:
        return None
    finally:
        os.close(fd)


def _node_score(fmts) -> int:
    """Rank a capture node by its best streamable colour format; depth/IR
    formats on the node drag it below any clean colour node."""
    best = max((_FMT_SCORE.get(f, 0) for f in fmts), default=0)
    if best and (fmts & _DEPTH_FMTS):
        best -= 60
    return best


def _enum_sizes(fd, fourcc: int):
    """([(w, h), …], stepwise) — discrete sizes, or the stepwise bounds."""
    sizes, stepwise = [], None
    for idx in range(64):
        s = _v4l2_frmsizeenum()
        s.index = idx
        s.pixel_format = fourcc
        try:
            fcntl.ioctl(fd, _VIDIOC_ENUM_FRAMESIZES, s)
        except OSError:
            break
        if s.type == _V4L2_FRMSIZE_TYPE_DISCRETE:
            sizes.append((s.u.discrete.width, s.u.discrete.height))
        else:   # CONTINUOUS/STEPWISE report one entry with the bounds
            stepwise = s.u.stepwise
            break
    return sizes, stepwise


def _enum_fps(fd, fourcc: int, w: int, h: int):
    """Discrete frame rates for (fourcc, w, h), highest first; [] if stepwise
    or unreported (caller then just asks for its target)."""
    rates = set()
    for idx in range(32):
        i = _v4l2_frmivalenum()
        i.index = idx
        i.pixel_format = fourcc
        i.width = w
        i.height = h
        try:
            fcntl.ioctl(fd, _VIDIOC_ENUM_FRAMEINTERVALS, i)
        except OSError:
            break
        if i.type != _V4L2_FRMIVAL_TYPE_DISCRETE:
            break
        if i.u.discrete.numerator:   # interval → rate is the reciprocal
            rates.add(Fraction(i.u.discrete.denominator, i.u.discrete.numerator))
    return sorted(rates, reverse=True)


class CameraStreamer(Node):
    def __init__(self):
        super().__init__('camera_streamer')

        # ── Parameters ────────────────────────────────────────────────────────
        p = self.declare_parameter
        self._transport = str(p('transport', 'srt').value).lower()
        if self._transport not in ('srt', 'ros2'):
            self.get_logger().error(
                f'unknown transport "{self._transport}" — falling back to srt')
            self._transport = 'srt'
        self._primary_match = p('primary_match', 'C920').value
        self._primary_port = int(p('primary_port', 8890).value)
        self._primary_audio_device = p('primary_audio_device', 'hw:CARD=C920').value
        self._dynamic_port_start = int(p('dynamic_port_start', 8900).value)
        # No V4L2 exclusion by default: with the ZED SDK/wrapper retired, the
        # ZED streams as a plain UVC camera. Set exclude_match:='ZED' (and
        # zed_enable:=true) to return to the SDK + ROS-topic-tap arrangement.
        self._exclude_match = p('exclude_match', '').value
        # Side-by-side stereo cameras (the ZED without its SDK): crop to the
        # left eye so the operator gets one image, not both eyes side by side.
        self._sbs_match = p('sbs_match', 'ZED').value
        # Primary capture target (the C920 inspection/CV feed)…
        self._width = int(p('width', 1280).value)
        self._height = int(p('height', 720).value)
        self._framerate = int(p('framerate', 20).value)
        # …and the (smaller) target for every other auto-detected camera.
        self._extra_width = int(p('extra_width', 640).value)
        self._extra_height = int(p('extra_height', 480).value)
        self._extra_framerate = int(p('extra_framerate', 30).value)
        # Capture at the sensor's native aspect ratio for maximum FOV: modes in
        # a different aspect than the camera's largest mode are centre-crops
        # (e.g. 4:3 on the natively-16:9 Orbbec/RealSense loses horizontal FOV).
        self._prefer_native_fov = bool(p('prefer_native_fov', True).value)
        self._bitrate = int(p('bitrate', 1500).value)          # kbps, per stream
        self._opus_kbps = int(p('opus_kbps', 24).value)
        self._latency = int(p('latency', 120).value)           # SRT ms
        self._rescan_period = float(p('rescan_period', 5.0).value)
        # ros2 transport
        self._topic_prefix = p('topic_prefix', '/robot/cameras').value.rstrip('/')
        self._jpeg_quality = int(p('jpeg_quality', 80).value)  # raw-only cams
        # ZED SDK tap (dormant by default — see exclude_match above)
        self._zed_enable = bool(p('zed_enable', False).value)
        self._zed_topic = p('zed_topic', '/zed/zed_node/left/image_rect_color').value
        self._zed_port = int(p('zed_port', 8899).value)
        self._zed_name = p('zed_name', 'ZED Left').value
        self._zed_fps = int(p('zed_fps', 15).value)
        self._zed_stale_s = float(p('zed_stale_s', 3.0).value)

        # ── State ─────────────────────────────────────────────────────────────
        # streams: key -> dict(name, port, proc, audio, dev, transport, …).
        # key = bus_info for UVC cams (stable across rescans), 'zed' for the tap.
        self._streams = {}
        self._port_by_key = {}           # stable port assignment per camera key
        self._next_dynamic_port = self._dynamic_port_start
        self._primary_key = None         # which bus_info owns the A/V primary
        self._catalog_json = None        # last-published catalog (dedupe republish)
        self._fail_counts = {}           # key -> consecutive quick pipeline deaths
        self._warned_unusable = set()    # keys logged as "no usable colour node"
        self._log_dir = '/tmp/camera_streamer'
        # Primary-mic audio-only SRT stream (ros2 transport only).
        self._audio_proc = None
        self._audio_log = None           # (file handle, path)
        self._audio_started = 0.0
        self._audio_fails = 0

        # Lazy GStreamer python bindings (gi) — used by the ZED tap and the ros2
        # transport. NOT cv2/cv_bridge, which are broken by NumPy 2.x on JetPack.
        self._Gst = None
        self._gst_failed = False
        self._zed_lock = threading.Lock()
        self._zed_hw = False             # nvv4l2h264enc available
        self._zed_pipe = None            # Gst.Pipeline
        self._zed_appsrc = None          # appsrc element we push frames into
        self._zed_last_frame = 0.0
        self._zed_size = None            # (w, h, gst_format) the pipeline was built for

        # ── Catalog publisher (latched so late-joining GUIs get it) ───────────
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_catalog = self.create_publisher(String, '/robot/camera_streams', latched)
        # Start from a clean slate so a stale catalog from a previous run (kept by
        # the Zenoh router) can't leave phantom cameras in the GUI.
        self._publish_catalog(force=True)

        if self._zed_enable:
            self._setup_zed_subscription()

        self.get_logger().info(
            f'camera_streamer[{self._transport}]: primary~"{self._primary_match}" '
            f'@ {self._primary_port} (audio {self._primary_audio_device}), '
            f'extras {self._extra_width}x{self._extra_height}@{self._extra_framerate} '
            f'from port {self._dynamic_port_start} / {self._topic_prefix}/*, '
            f'ZED tap {"on" if self._zed_enable else "off"} @ {self._zed_port}')

        self._rescan()   # initial detection
        self.create_timer(self._rescan_period, self._rescan)

    # ── GStreamer python bindings (shared by ZED tap + ros2 transport) ────────
    def _ensure_gst(self):
        if self._Gst is not None:
            return self._Gst
        if self._gst_failed:
            return None
        try:
            import gi
            gi.require_version('Gst', '1.0')
            gi.require_version('GstApp', '1.0')
            from gi.repository import Gst
            from gi.repository import GstApp  # noqa: F401 (registers app elements)
            Gst.init(None)
        except Exception as exc:
            self.get_logger().error(f'GStreamer python (gi) unavailable: {exc}')
            self._gst_failed = True
            return None
        self._Gst = Gst
        self._zed_hw = Gst.ElementFactory.find('nvv4l2h264enc') is not None
        return Gst

    # ── V4L2 discovery ────────────────────────────────────────────────────────
    def _detect_cameras(self):
        """{bus_info: (dev_path, card_name, formats)} — the best colour-capable
        capture node per physical camera; depth/IR nodes skipped."""
        groups = {}
        for dev in sorted(glob.glob('/dev/video*'),
                          key=lambda d: int(''.join(filter(str.isdigit, d)) or 0)):
            name = _sysfs_name(dev)
            if self._exclude_match and self._exclude_match.lower() in name.lower():
                continue  # explicitly excluded (e.g. a ZED owned by the SDK)
            probe = _probe_node(dev)
            if probe is None:
                continue  # not a capture node, or busy
            card, bus_info, fmts = probe
            key = bus_info or dev   # bus_info is stable per physical port
            groups.setdefault(key, []).append((dev, card or name or
                                               os.path.basename(dev), fmts))
        found = {}
        for key, nodes in groups.items():
            # Nodes arrive lowest-numbered first; keep that order among ties so
            # the chosen node is stable across rescans.
            best = max(nodes, key=lambda n: _node_score(n[2]))
            if _node_score(best[2]) <= 0:
                if key not in self._warned_unusable:
                    self._warned_unusable.add(key)
                    self.get_logger().warn(
                        f'camera "{nodes[0][1]}" [{key}] has no usable colour '
                        f'format (saw {sorted(set().union(*(n[2] for n in nodes)))}) '
                        '— skipped')
                continue
            found[key] = best
        return found

    def _capture_plan(self, dev, fmts, tw, th, tfps):
        """Pick an exact capture mode ≤ the (tw, th, tfps) target: best format
        by _FMT_SCORE, then — with prefer_native_fov — only sizes matching the
        aspect ratio of the camera's largest mode (the full sensor readout;
        same-aspect smaller modes are scaled/binned and keep the full FOV,
        other aspects are centre-crops), then the largest size not exceeding
        the target (or the smallest available), then the highest rate not
        exceeding the target. Returns dict(fourcc, width, height, fps) or None."""
        usable = sorted((f for f in fmts if f in _FMT_SCORE),
                        key=lambda f: -_FMT_SCORE[f])
        if not usable:
            return None
        fourcc_s = usable[0]
        fourcc = _str_to_fourcc(fourcc_s)
        try:
            fd = os.open(dev, os.O_RDWR | os.O_NONBLOCK)
        except OSError:
            return None
        try:
            sizes, stepwise = _enum_sizes(fd, fourcc)
            if sizes:
                pick_from = sizes
                if self._prefer_native_fov:
                    nw, nh = max(sizes, key=lambda s: s[0] * s[1])
                    native = nw / nh
                    # 5% tolerance keeps the 16:9 family together (424x240 =
                    # 1.767, 432x240 = 1.8) without admitting 4:3 (1.333).
                    matching = [s for s in sizes
                                if abs(s[0] / s[1] - native) / native < 0.05]
                    smallest = min(matching, key=lambda s: s[0] * s[1])
                    # Pathological lists (only an isolated giant odd-aspect
                    # mode matches, e.g. a 2fps full-sensor still mode) fall
                    # back to the plain fit rule instead of forcing it.
                    if smallest[0] * smallest[1] <= 4 * tw * th:
                        pick_from = matching
                fitting = [s for s in pick_from if s[0] <= tw and s[1] <= th]
                w, h = (max(fitting, key=lambda s: s[0] * s[1]) if fitting
                        else min(pick_from, key=lambda s: s[0] * s[1]))
            elif stepwise is not None:
                def _snap(v, lo, hi, step):
                    v = max(lo, min(hi, v))
                    return lo + ((v - lo) // max(1, step)) * max(1, step)
                w = _snap(tw, stepwise.min_width, stepwise.max_width,
                          stepwise.step_width)
                h = _snap(th, stepwise.min_height, stepwise.max_height,
                          stepwise.step_height)
            else:
                w, h = tw, th   # device reports nothing — ask for the target
            rates = _enum_fps(fd, fourcc, w, h)
        finally:
            os.close(fd)
        fitting = [r for r in rates if r <= tfps]
        fps = fitting[0] if fitting else (rates[-1] if rates else Fraction(tfps))
        return dict(fourcc=fourcc_s, width=int(w), height=int(h), fps=fps)

    # ── Rescan / stream reconciliation ────────────────────────────────────────
    def _rescan(self):
        try:
            detected = self._detect_cameras()
        except Exception as exc:   # never let a probe error kill the timer
            self.get_logger().warn(f'camera detect failed: {exc}')
            return

        # Reap any stream that died (camera unplugged / USB bandwidth / encode
        # error) so it can be restarted below if the camera is still present.
        for key, s in list(self._streams.items()):
            if s.get('transport') == 'ros2':
                self._check_ros_stream(key, s)
                continue
            proc = s.get('proc')
            if proc is not None and proc.poll() is not None:
                ran = time.monotonic() - s.get('started', 0.0)
                # Quick deaths count toward the primary's audio fallback; a
                # stream that ran healthily for a while resets the count.
                self._fail_counts[key] = \
                    0 if ran > 30.0 else self._fail_counts.get(key, 0) + 1
                tail = self._log_tail(s.get('logpath'))
                self.get_logger().warn(
                    f'stream for "{s["name"]}" exited (rc={proc.returncode} '
                    f'after {ran:.0f}s); will retry if still present'
                    + (f' — {tail}' if tail else ''))
                self._close_stream_log(s)
                self._streams.pop(key, None)

        # Stop streams whose camera vanished (and forget its failure history —
        # a re-plug starts fresh, including the A/V primary role).
        for key in list(self._streams.keys()):
            if key not in detected:
                self._stop_stream(key)
                self._fail_counts.pop(key, None)
                self._warned_unusable.discard(key)
                if key == self._primary_key:
                    self._primary_key = None

        # Start streams for newly present cameras.
        for key, (dev, card, fmts) in detected.items():
            if key in self._streams:
                continue
            # The primary keeps its role across pipeline restarts (a dead A/V
            # pipeline must NOT demote the C920 to a generic stream — that was
            # a real bug); it is only reconsidered when the camera vanishes.
            is_primary = (key == self._primary_key
                          or (self._primary_key is None
                              and self._primary_match
                              and self._primary_match.lower() in card.lower()))
            tw, th, tfps = ((self._width, self._height, self._framerate)
                            if is_primary else
                            (self._extra_width, self._extra_height,
                             self._extra_framerate))
            plan = self._capture_plan(dev, fmts, tw, th, tfps)
            if plan is None:
                if key not in self._warned_unusable:
                    self._warned_unusable.add(key)
                    self.get_logger().warn(
                        f'no capture plan for "{card}" [{dev}] (formats {sorted(fmts)})')
                continue
            if self._sbs_match and self._sbs_match.lower() in card.lower():
                # Side-by-side stereo frame (ZED): keep only the left eye.
                # out_width even so chroma-subsampled formats stay legal.
                out_w = (plan['width'] // 2) & ~1
                plan['out_width'] = out_w
                plan['crop_right'] = plan['width'] - out_w
            if is_primary:
                self._primary_key = key
            if self._transport == 'ros2':
                self._start_ros_stream(key, dev, card, plan)
            elif is_primary:
                # After repeated quick A/V deaths (usually the mic: device gone
                # or ALSA busy), keep the operator's video alive by dropping to
                # video-only on the same primary port.
                audio = self._fail_counts.get(key, 0) < 3
                if not audio:
                    self.get_logger().error(
                        f'primary A/V stream keeps dying — retrying "{card}" '
                        f'video-only on port {self._primary_port} '
                        f'(check the mic: {self._primary_audio_device})')
                self._start_stream(key, dev, card, self._primary_port,
                                   audio=audio, plan=plan)
            else:
                self._start_stream(key, dev, card, self._port_for(key),
                                   audio=False, plan=plan)

        # The primary mic rides its own audio-only SRT stream in ros2 mode.
        if self._transport == 'ros2':
            self._sync_audio_stream()

        # Refresh the ZED tap's catalog presence (started/stopped by the image cb).
        if self._zed_enable:
            self._sync_zed_state()

        # Republish EVERY tick (force), not only on change: the catalog is latched,
        # but rmw_zenoh does not reliably re-serve a latched sample to a subscriber
        # that joins after the one-shot publish (and a single publish can race
        # discovery). A cheap periodic (re)publish makes the GUI converge within one
        # rescan regardless. _publish_catalog only *logs* when the content changes.
        self._publish_catalog(force=True)

    def _port_for(self, key: str) -> int:
        if key not in self._port_by_key:
            self._port_by_key[key] = self._next_dynamic_port
            self._next_dynamic_port += 1
        return self._port_by_key[key]

    def _display_name(self, key: str, card: str) -> str:
        # A short-and-friendly display name; disambiguate identical models by the
        # tail of the bus path so two C270s don't collide in the GUI dropdown.
        suffix = key.rsplit('/', 1)[-1] if '/' in key else key
        return f'{card} ({suffix})' if suffix and suffix != card else card

    def _slug_for(self, key: str, card: str) -> str:
        """ROS-safe topic slug: card name + the USB path tail (uniqueness)."""
        tail = key.rsplit('usb-', 1)[-1] if 'usb-' in key else key
        slug = re.sub(r'[^a-z0-9]+', '_', f'{card}_{tail}'.lower()).strip('_')
        return slug if slug and not slug[0].isdigit() else f'cam_{slug}'

    # ── Pipeline fragments ────────────────────────────────────────────────────
    def _srtsink(self, port):
        # No shell quotes around the URI: the UVC pipelines run via gst-launch argv
        # (a list, no shell) where quotes would be passed literally, and the URI has
        # no spaces so it's a single token. gst_parse_launch (the ZED/appsink
        # paths) parses the same unquoted token fine.
        return (f'srtsink uri=srt://:{port}?mode=listener&latency={self._latency} '
                'wait-for-connection=false')

    def _capture_src(self, dev, plan):
        # Exact caps from the probe — see the module docstring for why free
        # negotiation is forbidden. The leaky queue decouples the UVC capture
        # thread from the encoder so a slow encode drops frames instead of
        # backpressuring the driver into jitter.
        if plan['fourcc'] == 'MJPG':
            caps = 'image/jpeg'
        else:
            caps = f'video/x-raw,format={_GST_RAW_FMT[plan["fourcc"]]}'
        fps = plan['fps']
        return (f'v4l2src device={dev} ! '
                f'{caps},width={plan["width"]},height={plan["height"]},'
                f'framerate={fps.numerator}/{fps.denominator} ! '
                'queue max-size-buffers=3 leaky=downstream ! ')

    def _h264_branch(self, plan):
        # ~1 keyframe/s so a (re)connecting GUI locks on quickly even at low fps.
        keyint = max(1, int(round(float(plan['fps']))))
        decode = 'jpegdec ! videoconvert ! ' if plan['fourcc'] == 'MJPG' \
            else 'videoconvert ! '
        crop = (f'videocrop right={plan["crop_right"]} ! '
                if plan.get('crop_right') else '')
        return (decode + crop +
                f'x264enc bitrate={self._bitrate} speed-preset=ultrafast '
                f'tune=zerolatency key-int-max={keyint} ! '
                'video/x-h264,profile=constrained-baseline ! '
                'h264parse config-interval=-1 ! ')

    def _video_pipeline(self, dev, port, plan):
        return (self._capture_src(dev, plan) + self._h264_branch(plan) +
                f'queue ! mpegtsmux alignment=7 ! {self._srtsink(port)}').split()

    def _av_pipeline(self, dev, port, plan):
        # The C920 A/V path: probed MJPEG video + the C920 mic as Opus, muxed into
        # one MPEG-TS (what the GUI's GstAvStream decodes for speaker + Vosk).
        opus_bps = self._opus_kbps * 1000
        return (
            f'mpegtsmux name=mux alignment=7 ! {self._srtsink(port)} '
            + self._capture_src(dev, plan) + self._h264_branch(plan) +
            'queue ! mux. '
            f'alsasrc device={self._primary_audio_device} ! audioconvert ! audioresample ! '
            'audio/x-raw,rate=48000,channels=1 ! '
            f'opusenc bitrate={opus_bps} audio-type=voice ! queue ! mux.'
        ).split()

    def _audio_pipeline(self, port):
        # ros2 transport: the mic alone as an audio-only MPEG-TS on the primary
        # port. The GUI's GstAvStream links its demux branches dynamically, so a
        # TS with no video PID still feeds the speaker + Vosk.
        opus_bps = self._opus_kbps * 1000
        return (
            f'alsasrc device={self._primary_audio_device} ! audioconvert ! audioresample ! '
            'audio/x-raw,rate=48000,channels=1 ! '
            f'opusenc bitrate={opus_bps} audio-type=voice ! queue ! '
            f'mpegtsmux alignment=7 ! {self._srtsink(port)}'
        ).split()

    # ── SRT stream lifecycle (gst-launch subprocesses) ────────────────────────
    def _spawn_gst(self, argv, log_name):
        """gst-launch subprocess with stdout+stderr kept in a per-stream log
        (surfaced on death — DEVNULL used to eat every failure reason)."""
        os.makedirs(self._log_dir, exist_ok=True)
        path = os.path.join(self._log_dir, f'{log_name}.log')
        logf = open(path, 'wb', buffering=0)
        try:
            proc = subprocess.Popen(['gst-launch-1.0', '-e', *argv],
                                    stdout=logf, stderr=logf)
        except FileNotFoundError:
            logf.close()
            raise
        return proc, logf, path

    def _log_tail(self, path, max_lines=4):
        if not path:
            return ''
        try:
            with open(path, 'rb') as f:
                f.seek(0, os.SEEK_END)
                f.seek(max(0, f.tell() - 2048))
                lines = [ln.strip() for ln in
                         f.read().decode(errors='replace').splitlines() if ln.strip()]
            return ' | '.join(lines[-max_lines:])
        except OSError:
            return ''

    @staticmethod
    def _close_stream_log(s):
        logf = s.get('logf')
        if logf is not None:
            try:
                logf.close()
            except OSError:
                pass

    def _start_stream(self, key, dev, card, port, audio, plan):
        pipeline = self._av_pipeline(dev, port, plan) if audio \
            else self._video_pipeline(dev, port, plan)
        try:
            proc, logf, logpath = self._spawn_gst(pipeline, self._slug_for(key, card))
        except FileNotFoundError:
            self.get_logger().error('gst-launch-1.0 not found — cannot stream cameras')
            return
        name = self._display_name(key, card)
        self._streams[key] = dict(name=name, port=port, proc=proc, audio=audio,
                                  dev=dev, transport='srt', logf=logf,
                                  logpath=logpath, started=time.monotonic())
        fps = plan['fps']
        self.get_logger().info(
            f'streaming {"A/V" if audio else "video"} "{name}" [{dev} '
            f'{plan["fourcc"]} {plan["width"]}x{plan["height"]}@'
            f'{float(fps):g}'
            + (f' -> left eye {plan["out_width"]}x{plan["height"]}'
               if plan.get('crop_right') else '')
            + f'] -> srt://:{port}')

    def _stop_stream(self, key):
        s = self._streams.pop(key, None)
        if not s:
            return
        proc = s.get('proc')
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        self._close_stream_log(s)
        pipe = s.get('pipe')
        if pipe is not None and self._Gst is not None:
            s['stopped'] = True
            pipe.set_state(self._Gst.State.NULL)
        pub = s.get('pub')
        if pub is not None:
            self.destroy_publisher(pub)
        self.get_logger().info(f'stopped stream "{s["name"]}" [{s.get("dev")}]')

    # ── ros2 transport (CompressedImage via in-process appsink pipelines) ─────
    def _start_ros_stream(self, key, dev, card, plan):
        Gst = self._ensure_gst()
        if Gst is None:
            # gi is a hard requirement of the ros2 transport; keep the operator
            # seeing *something* by falling back to an SRT subprocess.
            self.get_logger().error(
                'ros2 transport needs GStreamer python (gi) — '
                f'falling back to SRT for "{card}"')
            self._start_stream(key, dev, card, self._port_for(key),
                               audio=False, plan=plan)
            return
        slug = self._slug_for(key, card)
        topic = f'{self._topic_prefix}/{slug}/compressed'
        # MJPEG cameras are passed through untouched (zero CPU); raw-only ones
        # are JPEG-encoded here (still far cheaper than H.264 on this CPU). A
        # side-by-side crop forces the encode path even for MJPEG sources.
        crop = (f'videocrop right={plan["crop_right"]} ! '
                if plan.get('crop_right') else '')
        if plan['fourcc'] == 'MJPG' and not crop:
            enc = ''
        else:
            decode = 'jpegdec ! ' if plan['fourcc'] == 'MJPG' else ''
            enc = (decode + 'videoconvert ! ' + crop +
                   f'jpegenc quality={self._jpeg_quality} ! ')
        desc = (self._capture_src(dev, plan) + enc +
                'appsink name=sink emit-signals=true max-buffers=2 drop=true sync=false')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        pub = self.create_publisher(CompressedImage, topic, qos)
        entry = dict(name=self._display_name(key, card), port=0, proc=None,
                     audio=False, dev=dev, transport='ros2', topic=topic,
                     frame_id=slug, pub=pub, pipe=None, stopped=False,
                     last_frame=time.monotonic(), started=time.monotonic())
        try:
            pipe = Gst.parse_launch(desc)
            sink = pipe.get_by_name('sink')
            sink.connect('new-sample', self._on_cam_sample, entry)
            pipe.set_state(Gst.State.PLAYING)
        except Exception as exc:
            self.get_logger().error(f'ros2 stream for "{card}" failed to build: {exc}')
            self.destroy_publisher(pub)
            return
        entry['pipe'] = pipe
        self._streams[key] = entry
        fps = plan['fps']
        self.get_logger().info(
            f'publishing "{entry["name"]}" [{dev} {plan["fourcc"]} '
            f'{plan["width"]}x{plan["height"]}@{float(fps):g}'
            f'{" passthrough" if not enc else ""}'
            + (f' -> left eye {plan["out_width"]}x{plan["height"]}'
               if plan.get('crop_right') else '')
            + f'] -> {topic}')

    def _on_cam_sample(self, sink, entry):
        # Runs on the pipeline's streaming thread. `emit` (action signal) works
        # without a GLib main loop — same pattern as the ZED tap's push-buffer.
        Gst = self._Gst
        sample = sink.emit('pull-sample')
        if sample is None or entry['stopped']:
            return Gst.FlowReturn.OK
        buf = sample.get_buffer()
        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK
        try:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = entry['frame_id']
            msg.format = 'jpeg'
            # array('B') hits the setter's typecode fast path (memcpy). Bytes
            # would take rclpy's per-element validation — pure Python over every
            # JPEG byte, which caps ~110 frames/s of cameras at a few Hz total.
            msg.data = array.array('B', bytes(mapinfo.data))
            entry['pub'].publish(msg)
            entry['last_frame'] = time.monotonic()
        except Exception:
            pass   # racing a stream teardown — the rescan owns recovery
        finally:
            buf.unmap(mapinfo)
        return Gst.FlowReturn.OK

    def _check_ros_stream(self, key, s):
        """Recycle a ros2 pipeline that errored or stalled (USB hiccup, unplug
        mid-frame, …); the camera restarts on the next rescan if still there."""
        Gst = self._Gst
        reason = None
        pipe = s.get('pipe')
        if pipe is not None and Gst is not None:
            bus = pipe.get_bus()
            gmsg = bus.pop_filtered(Gst.MessageType.ERROR) if bus else None
            if gmsg is not None:
                err, _dbg = gmsg.parse_error()
                reason = f'pipeline error: {err.message}'
        stale_after = max(4.0, 2.0 * self._rescan_period)
        if reason is None and (time.monotonic() - s['last_frame']) > stale_after:
            reason = f'no frames for {stale_after:.0f}s'
        if reason is not None:
            self.get_logger().warn(
                f'ros2 stream "{s["name"]}" recycled ({reason}); '
                'will retry if still present')
            self._stop_stream(key)

    def _sync_audio_stream(self):
        """Keep the primary mic's audio-only SRT stream matched to the primary
        camera's presence (ros2 transport only)."""
        if self._audio_proc is not None and self._audio_proc.poll() is not None:
            ran_ok = (time.monotonic() - self._audio_started) > 30.0
            if self._audio_log is not None:
                tail = self._log_tail(self._audio_log[1])
                try:
                    self._audio_log[0].close()
                except OSError:
                    pass
            else:
                tail = ''
            self._audio_fails = 0 if ran_ok else self._audio_fails + 1
            level = self.get_logger().error if self._audio_fails >= 5 \
                else self.get_logger().warn
            level(f'primary audio stream exited'
                  + (f' — {tail}' if tail else '')
                  + ('; giving up (check the mic device)' if self._audio_fails >= 5
                     else '; retrying'))
            self._audio_proc = None
            self._audio_log = None

        primary_up = (self._primary_key is not None
                      and self._primary_key in self._streams)
        if primary_up and self._audio_proc is None and self._audio_fails < 5:
            try:
                proc, logf, path = self._spawn_gst(
                    self._audio_pipeline(self._primary_port), 'primary_audio')
            except FileNotFoundError:
                self.get_logger().error('gst-launch-1.0 not found — no audio stream')
                self._audio_fails = 5
                return
            self._audio_proc, self._audio_log = proc, (logf, path)
            self._audio_started = time.monotonic()
            self.get_logger().info(
                f'streaming audio-only ({self._primary_audio_device}) -> '
                f'srt://:{self._primary_port}')
        elif not primary_up and self._audio_proc is not None:
            self._audio_proc.send_signal(signal.SIGINT)
            try:
                self._audio_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._audio_proc.kill()
            if self._audio_log is not None:
                try:
                    self._audio_log[0].close()
                except OSError:
                    pass
            self._audio_proc = None
            self._audio_log = None
            self._audio_fails = 0

    # ── ZED tap (re-encode the SDK's published left image to SRT) ─────────────
    # ROS Image encoding -> (GStreamer raw format, bytes/pixel).
    _ENC_TO_GST = {
        'bgra8': 'BGRA', 'rgba8': 'RGBA', 'bgr8': 'BGR', 'rgb8': 'RGB',
        'mono8': 'GRAY8', 'bgr16': None, 'rgb16': None,
    }

    def _setup_zed_subscription(self):
        # Any failure just disables the ZED tap — the USB camera streaming must
        # never be taken down by it.
        if self._ensure_gst() is None:
            self.get_logger().warn('ZED tap disabled — GStreamer python (gi) unavailable')
            self._zed_enable = False
            return
        from sensor_msgs.msg import Image
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self._zed_topic, self._on_zed_image, best_effort)

    def _on_zed_image(self, msg):
        if self._Gst is None:
            return
        gst_format = self._ENC_TO_GST.get(msg.encoding)
        if gst_format is None:
            self.get_logger().warn(
                f'ZED tap: unsupported image encoding "{msg.encoding}"',
                throttle_duration_sec=10.0)
            return
        with self._zed_lock:
            want = (msg.width, msg.height, gst_format)
            if self._zed_appsrc is None or self._zed_size != want:
                if not self._open_zed_pipeline(*want):
                    return
            buf = self._Gst.Buffer.new_wrapped(bytes(msg.data))
            # push-buffer is an appsrc action signal; ret is a GstFlowReturn enum.
            self._zed_appsrc.emit('push-buffer', buf)
            self._zed_last_frame = time.monotonic()

    def _open_zed_pipeline(self, w, h, gst_format):
        Gst = self._Gst
        if self._zed_hw:
            enc = (f'nvvidconv ! nvv4l2h264enc insert-sps-pps=true '
                   f'iframeinterval={self._zed_fps} bitrate={self._bitrate * 1000}')
        else:
            enc = (f'x264enc bitrate={self._bitrate} speed-preset=ultrafast '
                   f'tune=zerolatency key-int-max={self._zed_fps} ! '
                   f'video/x-h264,profile=constrained-baseline')
        desc = (
            'appsrc name=src is-live=true do-timestamp=true format=time ! '
            'videoconvert ! ' + enc + ' ! '
            'h264parse config-interval=-1 ! mpegtsmux alignment=7 ! '
            + self._srtsink(self._zed_port)
        )
        try:
            pipe = Gst.parse_launch(desc)
            appsrc = pipe.get_by_name('src')
            appsrc.set_property('caps', Gst.Caps.from_string(
                f'video/x-raw,format={gst_format},width={w},height={h},'
                f'framerate={self._zed_fps}/1'))
            appsrc.set_property('block', True)
            pipe.set_state(Gst.State.PLAYING)
        except Exception as exc:
            self.get_logger().error(f'ZED tap: pipeline build failed: {exc}')
            return False
        self._teardown_zed_pipe()   # drop any previous one
        self._zed_pipe = pipe
        self._zed_appsrc = appsrc
        self._zed_size = (w, h, gst_format)
        self.get_logger().info(
            f'ZED tap: {w}x{h} {gst_format} -> srt://:{self._zed_port} '
            f'({"HW nvv4l2h264enc" if self._zed_hw else "CPU x264enc"})')
        return True

    def _teardown_zed_pipe(self):
        if self._zed_pipe is not None and self._Gst is not None:
            self._zed_pipe.set_state(self._Gst.State.NULL)
        self._zed_pipe = None
        self._zed_appsrc = None
        self._zed_size = None

    def _sync_zed_state(self):
        """Add/remove the ZED entry in _streams based on frame liveness. Returns
        True if the catalog presence changed."""
        with self._zed_lock:
            live = (self._zed_appsrc is not None
                    and (time.monotonic() - self._zed_last_frame) < self._zed_stale_s)
            present = 'zed' in self._streams
            if live and not present:
                self._streams['zed'] = dict(name=self._zed_name, port=self._zed_port,
                                            proc=None, audio=False, dev=self._zed_topic,
                                            transport='srt')
                return True
            if not live and present:
                self._streams.pop('zed', None)
                # Drop the pipeline so a resumed SDK re-opens cleanly at its size.
                self._teardown_zed_pipe()
                return True
        return False

    # ── Catalog ───────────────────────────────────────────────────────────────
    def _publish_catalog(self, force=False):
        # Advertise the video streams. In SRT mode the A/V primary is a static
        # A/V source in the GUI (GstAvStream), so it is intentionally omitted;
        # in ros2 mode the primary's video IS listed (its topic), since the
        # static A/V source then carries audio only.
        cams = []
        for s in self._streams.values():
            if s['audio']:
                continue
            entry = {'name': s['name'], 'transport': s.get('transport', 'srt')}
            if entry['transport'] == 'ros2':
                entry['topic'] = s['topic']
            else:
                entry['port'] = s['port']
            cams.append(entry)
        cams.sort(key=lambda c: (c.get('port', 0), c['name']))
        payload = json.dumps({'cameras': cams})
        changed = (payload != self._catalog_json)
        if not changed and not force:
            return
        self._catalog_json = payload
        self._pub_catalog.publish(String(data=payload))
        if changed:   # only log on real change (force republishes every tick, silent)
            self.get_logger().info(f'catalog: {len(cams)} video stream(s) advertised')

    # ── Teardown ──────────────────────────────────────────────────────────────
    def destroy_node(self):
        for key in list(self._streams.keys()):
            self._stop_stream(key)
        if self._audio_proc is not None and self._audio_proc.poll() is None:
            self._audio_proc.send_signal(signal.SIGINT)
            try:
                self._audio_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._audio_proc.kill()
        if self._audio_log is not None:
            try:
                self._audio_log[0].close()
            except OSError:
                pass
        with self._zed_lock:
            self._teardown_zed_pipe()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass   # Ctrl-C or systemd SIGINT — normal stop
    finally:
        node.destroy_node()
        if rclpy.ok():          # signal handler may have shut the context already
            rclpy.shutdown()


if __name__ == '__main__':
    main()
