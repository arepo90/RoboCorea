#!/usr/bin/env python3
"""camera_streamer — robot-side multi-camera SRT streamer + live catalog.

Replaces the static ``c920_srt_stream.sh`` with an auto-detecting ROS 2 node:

  * Enumerates every V4L2 UVC **capture** camera on the Jetson and streams each
    to the operator GUI over **SRT** (H.264 in MPEG-TS), video-only.
  * The **ZED is excluded** from the V4L2 scan (the ZED SDK grabs the device
    exclusively). Instead its left image — which the SDK already publishes on a
    ROS topic — is re-encoded to its own SRT stream, so it runs *alongside* the
    running SDK.
  * One designated **primary** camera (the C920 Pro, matched by name) also gets
    its microphone muxed in as Opus — the A/V stream the GUI decodes for the
    speaker + Vosk. The GUI keeps that one as a static A/V source, so it is NOT
    advertised in the catalog below.
  * Publishes a **latched JSON catalog** on ``/robot/camera_streams`` listing the
    video-only streams (name + SRT port) so the GUI auto-populates its source
    list and picks up hot-plugged cameras.

The video never touches DDS/Zenoh — it rides SRT exactly like the old script.
Only the ZED tap and the catalog are ROS. A periodic rescan handles hot-plug.
"""

import ctypes
import fcntl
import glob
import json
import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# ── V4L2 capability probe (no external deps) ──────────────────────────────────
# We read the human name from sysfs (no device open needed, so a busy ZED never
# blocks us) and only VIDIOC_QUERYCAP the survivors to confirm they capture video
# and to get a stable bus_info (used to dedupe a camera's multiple /dev/video*
# nodes and to keep a camera's port stable across rescans).

_V4L2_CAP_VIDEO_CAPTURE = 0x00000001
_V4L2_CAP_DEVICE_CAPS = 0x80000000
# _IOR('V', 0, struct v4l2_capability) — struct is 104 bytes.
_VIDIOC_QUERYCAP = 0x80685600


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


def _sysfs_name(dev_path: str) -> str:
    """Card name from sysfs; readable even while the device is busy (ZED)."""
    node = os.path.basename(dev_path)
    try:
        with open(f'/sys/class/video4linux/{node}/name', 'r') as f:
            return f.read().strip()
    except OSError:
        return ''


def _querycap(dev_path: str):
    """(card, bus_info) if the node is a video-capture device, else None."""
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
        return (cap.card.decode(errors='replace'),
                cap.bus_info.decode(errors='replace'))
    except OSError:
        return None
    finally:
        os.close(fd)


class CameraStreamer(Node):
    def __init__(self):
        super().__init__('camera_streamer')

        # ── Parameters ────────────────────────────────────────────────────────
        p = self.declare_parameter
        self._primary_match = p('primary_match', 'C920').value
        self._primary_port = int(p('primary_port', 8890).value)
        self._primary_audio_device = p('primary_audio_device', 'hw:CARD=C920').value
        self._dynamic_port_start = int(p('dynamic_port_start', 8900).value)
        self._exclude_match = p('exclude_match', 'ZED').value   # keep the ZED out of V4L2 scan
        self._width = int(p('width', 1280).value)
        self._height = int(p('height', 720).value)
        self._framerate = int(p('framerate', 20).value)
        self._bitrate = int(p('bitrate', 1500).value)          # kbps, per stream
        self._opus_kbps = int(p('opus_kbps', 24).value)
        self._latency = int(p('latency', 120).value)           # SRT ms
        self._rescan_period = float(p('rescan_period', 5.0).value)
        # ZED tap
        self._zed_enable = bool(p('zed_enable', True).value)
        self._zed_topic = p('zed_topic', '/zed/zed_node/left/image_rect_color').value
        self._zed_port = int(p('zed_port', 8899).value)
        self._zed_name = p('zed_name', 'ZED Left').value
        self._zed_fps = int(p('zed_fps', 15).value)
        self._zed_stale_s = float(p('zed_stale_s', 3.0).value)

        # ── State ─────────────────────────────────────────────────────────────
        # streams: key -> dict(name, port, proc, audio, dev). key = bus_info for
        # UVC cams (stable across rescans), 'zed' for the ZED tap.
        self._streams = {}
        self._port_by_key = {}           # stable port assignment per camera key
        self._next_dynamic_port = self._dynamic_port_start
        self._primary_key = None         # which bus_info owns the A/V primary
        self._catalog_json = None        # last-published catalog (dedupe republish)

        # ZED tap runtime (a GStreamer appsrc pipeline built lazily on the first
        # image). Uses gi/Gst directly — NOT cv2/cv_bridge — so it is immune to the
        # NumPy-1.x-vs-2.x breakage that cripples cv_bridge on JetPack.
        self._zed_lock = threading.Lock()
        self._Gst = None                 # gi.repository.Gst once imported
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
            f'camera_streamer: primary~"{self._primary_match}" @ {self._primary_port} '
            f'(audio {self._primary_audio_device}), extras from {self._dynamic_port_start}, '
            f'ZED tap {"on" if self._zed_enable else "off"} @ {self._zed_port}')

        self._rescan()   # initial detection
        self.create_timer(self._rescan_period, self._rescan)

    # ── V4L2 discovery ────────────────────────────────────────────────────────
    def _detect_cameras(self):
        """Return {bus_info: (dev_path, card_name)} for capture cams (excl. ZED)."""
        found = {}
        for dev in sorted(glob.glob('/dev/video*'),
                          key=lambda d: int(''.join(filter(str.isdigit, d)) or 0)):
            name = _sysfs_name(dev)
            if self._exclude_match and self._exclude_match.lower() in name.lower():
                continue  # the ZED — streamed via its ROS topic instead
            cap = _querycap(dev)
            if cap is None:
                continue  # not a capture node, or busy
            card, bus_info = cap
            key = bus_info or dev   # bus_info is stable per physical port
            # Keep the first (lowest-numbered) capture node per physical camera.
            if key not in found:
                found[key] = (dev, card or name or os.path.basename(dev))
        return found

    def _rescan(self):
        try:
            detected = self._detect_cameras()
        except Exception as exc:   # never let a probe error kill the timer
            self.get_logger().warn(f'camera detect failed: {exc}')
            return

        # Reap any pipeline that died (camera unplugged / decode error) so it can
        # be restarted below if the camera is still present.
        for key, s in list(self._streams.items()):
            if key == 'zed':
                continue
            proc = s.get('proc')
            if proc is not None and proc.poll() is not None:
                self.get_logger().warn(f'stream for "{s["name"]}" exited (rc={proc.returncode}); '
                                       'will retry if still present')
                self._streams.pop(key, None)

        # Stop streams whose camera vanished.
        for key in list(self._streams.keys()):
            if key == 'zed':
                continue
            if key not in detected:
                self._stop_stream(key)
                if key == self._primary_key:
                    self._primary_key = None

        # Start streams for newly present cameras.
        for key, (dev, card) in detected.items():
            if key in self._streams:
                continue
            is_primary = (self._primary_key is None
                          and self._primary_match
                          and self._primary_match.lower() in card.lower())
            if is_primary:
                self._primary_key = key
                self._start_stream(key, dev, card, self._primary_port, audio=True)
            else:
                port = self._port_for(key)
                self._start_stream(key, dev, card, port, audio=False)   # advertised in catalog

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

    # ── Stream lifecycle (gst-launch subprocesses) ────────────────────────────
    def _start_stream(self, key, dev, card, port, audio):
        pipeline = self._av_pipeline(dev, port) if audio else self._video_pipeline(dev, port)
        try:
            proc = subprocess.Popen(['gst-launch-1.0', '-e', *pipeline],
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            self.get_logger().error('gst-launch-1.0 not found — cannot stream cameras')
            return
        # A short-and-friendly display name; disambiguate identical models by the
        # tail of the bus path so two C270s don't collide in the GUI dropdown.
        suffix = key.rsplit('/', 1)[-1] if '/' in key else key
        name = f'{card} ({suffix})' if suffix and suffix != card else card
        self._streams[key] = dict(name=name, port=port, proc=proc, audio=audio, dev=dev)
        self.get_logger().info(
            f'streaming {"A/V" if audio else "video"} "{name}" [{dev}] -> srt://:{port}')

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
        self.get_logger().info(f'stopped stream "{s["name"]}" [{s.get("dev")}]')

    def _srtsink(self, port):
        # No shell quotes around the URI: the UVC pipelines run via gst-launch argv
        # (a list, no shell) where quotes would be passed literally, and the URI has
        # no spaces so it's a single token. gst_parse_launch (the ZED VideoWriter
        # path) parses the same unquoted token fine.
        return (f'srtsink uri=srt://:{port}?mode=listener&latency={self._latency} '
                'wait-for-connection=false')

    def _video_pipeline(self, dev, port):
        # Robust across heterogeneous/older cameras: decodebin copes with MJPEG or
        # raw YUYV, videoconvert normalizes the pixel format, and x264enc caps the
        # output bitrate regardless of the camera's native resolution.
        return (
            f'v4l2src device={dev} ! decodebin ! videoconvert ! '
            f'x264enc bitrate={self._bitrate} speed-preset=ultrafast tune=zerolatency '
            f'key-int-max={self._framerate} ! video/x-h264,profile=constrained-baseline ! '
            f'h264parse config-interval=-1 ! mpegtsmux alignment=7 ! {self._srtsink(port)}'
        ).split()

    def _av_pipeline(self, dev, port):
        # The proven C920 A/V path: MJPEG 720p + the C920 mic as Opus, muxed into
        # one MPEG-TS (what the GUI's GstAvStream decodes for speaker + Vosk).
        opus_bps = self._opus_kbps * 1000
        return (
            f'mpegtsmux name=mux alignment=7 ! {self._srtsink(port)} '
            f'v4l2src device={dev} ! '
            f'image/jpeg,width={self._width},height={self._height},framerate={self._framerate}/1 ! '
            f'jpegdec ! videoconvert ! '
            f'x264enc bitrate={self._bitrate} speed-preset=ultrafast tune=zerolatency '
            f'key-int-max={self._framerate} ! video/x-h264,profile=constrained-baseline ! '
            f'h264parse config-interval=-1 ! queue ! mux. '
            f'alsasrc device={self._primary_audio_device} ! audioconvert ! audioresample ! '
            f'audio/x-raw,rate=48000,channels=1 ! '
            f'opusenc bitrate={opus_bps} audio-type=voice ! queue ! mux.'
        ).split()

    # ── ZED tap (re-encode the SDK's published left image to SRT) ─────────────
    # ROS Image encoding -> (GStreamer raw format, bytes/pixel).
    _ENC_TO_GST = {
        'bgra8': 'BGRA', 'rgba8': 'RGBA', 'bgr8': 'BGR', 'rgb8': 'RGB',
        'mono8': 'GRAY8', 'bgr16': None, 'rgb16': None,
    }

    def _setup_zed_subscription(self):
        # Use GStreamer's Python bindings directly (no cv2/cv_bridge, which are
        # broken by NumPy 2.x on this JetPack). Any failure just disables the ZED
        # tap — the USB camera streaming must never be taken down by it.
        try:
            import gi
            gi.require_version('Gst', '1.0')
            gi.require_version('GstApp', '1.0')
            from gi.repository import Gst
            from gi.repository import GstApp  # noqa: F401 (registers appsrc actions)
            Gst.init(None)
            from sensor_msgs.msg import Image
        except Exception as exc:
            self.get_logger().warn(f'ZED tap disabled — GStreamer python (gi) unavailable: {exc}')
            self._zed_enable = False
            return
        self._Gst = Gst
        self._zed_hw = Gst.ElementFactory.find('nvv4l2h264enc') is not None
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
                                            proc=None, audio=False, dev=self._zed_topic)
                return True
            if not live and present:
                self._streams.pop('zed', None)
                # Drop the pipeline so a resumed SDK re-opens cleanly at its size.
                self._teardown_zed_pipe()
                return True
        return False

    # ── Catalog ───────────────────────────────────────────────────────────────
    def _publish_catalog(self, force=False):
        # Advertise the video-only streams (extras + ZED). The A/V primary is a
        # static A/V source in the GUI (GstAvStream), so it is intentionally omitted.
        cams = [{'name': s['name'], 'port': s['port']}
                for s in self._streams.values() if not s['audio']]
        cams.sort(key=lambda c: c['port'])
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
            if key != 'zed':
                self._stop_stream(key)
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
