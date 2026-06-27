# gui — RoboCorea Operator Console

Qt6 + ROS 2 teleop GUI for the workstation. In-progress port of the legacy `gui`
package (see `reference/architecture.md` §11.1), built in compiling increments.

## Status

| Subsystem | State |
|-----------|-------|
| Video (local cams + C920 SRT streams + thermal) | **implemented** |
| Transport-agnostic `CameraHub` / `SourceManager` | **implemented** |
| C920 **A/V** stream (H.264 video + Opus audio over one SRT) | **implemented** (needs GStreamer dev — see below) |
| Speech-to-text (Vosk) fed from the A/V audio | **implemented** |
| Dashboard (connection, mag, orientation from ZED2, e-stop, sensor toggles, transcription) | **implemented** |
| Odometry (mode/flags, traction, track wheel-odometry, flippers, VESC table, arm telemetry) | **implemented** |
| Digital twin (URDF/OpenGL, posed from `/joint_states`) | **implemented** |
| CV filters (Hazmat YOLO, QR/barcode, shape detect) | **implemented** |
| Settings / PPM-calib dialogs (no keybind editor — fixed RC scheme) | **implemented** |
| Arm lifecycle controls (arm/disarm, dexterity/chassis, CAN presence) in the dashboard | **implemented** |
| Workstation `bringup.launch.py` (GUI + servo + flipper_state + twin) | **implemented** |

GStreamer is **optional at build time**: without its dev packages the GUI still
builds, just without the C920 A/V stream + its audio. The CV filters and digital
twin now pull in ONNX Runtime, Assimp, ZBar and Qt OpenGL as **required** deps
(see below) — the YOLO model itself is an optional drop-in (`assets/vision/`).

### Layout

A horizontal splitter: the **2×2 video grid** on the left, and a right section
with the **Digital Twin** on top over the **Odometry** and **Dashboard** panels
side by side below. Click any video cell to enlarge it (click again to restore).

While a cell is **enlarged** you can **zoom and pan** it:

- **Zoom:** the floating **+ / − / Fit** buttons in the cell's top-right corner,
  or **Ctrl + scroll** (zooms toward the cursor). Trackpad **pinch** also works,
  but only on a **Wayland** session — on X11/GNOME the compositor intercepts the
  pinch and Qt's xcb plugin never delivers it to the app, so the buttons /
  Ctrl+scroll are the path there.
- **Pan:** two-finger scroll, or click-drag.

The zoom/pan **crops the source frame to that region and upscales it before the
filter pipeline runs**, so the CV filters see the target centred and filling the
view (handy for detectors that only fire when the subject is large/centred). Zoom
resets to fit when the cell collapses back to the grid; the grid thumbnails always
show the full frame.

---

## Video + audio architecture

Every cell in the 2×2 video grid can show any source, selected by a URI-style id:

| Source | Id scheme | Path |
|--------|-----------|------|
| RF analog **driving** cams | `local:N` | digitized at the workstation → `/dev/videoN` → V4L2 |
| **C920 Rear** (video only) | `gst:<pipeline>` | onboard H.264 → SRT → OpenCV/GStreamer decode |
| **C920 Front** (video **+ audio**) | `av:<i>` | onboard H.264 **+ Opus mic** in one MPEG-TS/SRT → native GStreamer demux |
| **Thermal** | `thermal:/sensors/thermal` | arm-PCB MLX90640 → `esp32_bridge` → ROS `Image` (colormapped here) |

`CameraHub` owns one capture thread per distinct source (ref-counted, auto-
reconnecting). `local:`/`gst:` open via OpenCV; the `av:` front stream is owned by
a native-GStreamer receiver (`GstAvStream`) registered with the hub. **No DDS
round-trip for video** (the legacy `gst_bridge` is gone).

### Why this design (degraded-link rationale)

- **No encode on the robot.** The Orin Nano has no hardware encoder (NVENC); the
  C920 encodes H.264 itself, so the Jetson only packetizes — near-zero CPU.
- **SRT, not raw RTP/DDS.** SRT does selective retransmission (ARQ) within a
  fixed latency budget — ideal on the low-RTT arena LAN.
- **Audio = Opus muxed into the front camera's stream.** The old design sent raw
  PCM (256 kbps) over best-effort DDS → fragmented, lossy, laggy. Now the C920
  mic is **Opus** (~24 kbps, packet-loss concealment) muxed with the video into
  one synchronized MPEG-TS/SRT connection. The GUI demuxes it: video → display,
  audio → speakers **and** Vosk transcription.
- **Bulk media stays off DDS.** Only the tiny thermal image (32×24) rides ROS.

### Config — `~/.config/robocorea_gui/settings.json`

```json
{
  "default_robot_host": "192.168.1.10",
  "audio_start_enabled": true,
  "vosk_grammar": "",
  "video_streams": [
    { "name": "C920 Front (SRT A/V)", "host": "", "port": 8890, "latency_ms": 120, "audio": true },
    { "name": "C920 Rear (SRT)",      "host": "", "port": 8891, "latency_ms": 120, "audio": false }
  ]
}
```

`host: ""` → use `default_robot_host` (set this to your Jetson's IP). `audio: true`
makes the stream a native A/V receiver (front cam). Ports must match the robot
streamer. The file is created with defaults on first run and edited live by the
in-app **Settings** dialog (the ⚙ button in the dashboard), which also persists
the `ppm_calib` array (pushed to the robot on `/robot/ppm_calib`). There is no
keybind editor — the RC control scheme is fixed in the firmware.

---

## Dependencies

```bash
# ROS + Qt + OpenCV (OpenCV must have GStreamer support for the gst: path)
sudo apt install -y ros-humble-cv-bridge qt6-base-dev libopencv-dev python3-opencv

# Digital twin (URDF parse + OpenGL view + mesh loading) and CV filters (QR/barcode):
sudo apt install -y ros-humble-urdf qt6-base-dev libqt6opengl6-dev \
  libassimp-dev libzbar-dev

# GStreamer — dev (build) + runtime plugins (SRT, Opus, H.264, MPEG-TS).
# The SRT element lives in plugins-bad, which Depends on libsrt (auto-installed);
# no need to name a libsrt package explicitly.
sudo apt install -y \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-libav
```

Verify OpenCV's GStreamer backend (for the rear `gst:` cam):
```bash
python3 -c "import cv2; print(cv2.getBuildInformation())" | grep -i gstreamer   # must say YES
```

**ONNX Runtime** (Hazmat/YOLO inference): install `libonnxruntime.so` +
`onnxruntime_cxx_api.h` under `/usr/local` (legacy `shared/gui_ws/README.md`). The
CUDA execution provider is used when available, with a CPU fallback. Drop the
trained detector into `gui/assets/vision/best.onnx` (see the README there) — absent
the model, the Hazmat filter just overlays "model not loaded" and everything else
works.

**Vosk** (speech): install `libvosk.so` + `vosk_api.h` under `/usr/local` (legacy
`shared/gui_ws/README.md` §2) and drop a model into `gui/assets/audio/` (see the
README there). Without the model, the GUI runs and transcription is just disabled.

> If `libgstreamer1.0-dev` is absent, CMake prints a warning and builds the GUI
> **without** the C920 A/V stream + speech audio. Install it to enable them.

---

## Running

### Workstation (GUI)
```bash
cd software/ros2_ws
colcon build --packages-select gui --symlink-install
source install/setup.bash
ros2 launch gui gui.launch.py
```

### Robot (Jetson) — start the C920 streams
```bash
software/ros2_ws/src/gui/scripts/c920_srt_stream.sh
```
Edit the `CAMERAS`/tunables block at the top: video device, SRT port, and the
optional **ALSA mic device** (`hw:CARD=C920,DEV=0` — find it with `arecord -l`).
A camera with a mic field set streams A/V; an empty mic field streams video only.
Jetson deps: `gstreamer1.0-plugins-bad` (`uvch264src`, `srtsink`, `mpegtsmux`,
`opusenc`; pulls libsrt automatically) and `gstreamer1.0-plugins-good`
(`alsasrc`).

Either side may start first — `CameraHub`/`GstAvStream` keep retrying the SRT
connection, and `srtsink` runs without a client connected.
