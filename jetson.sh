#!/usr/bin/env bash
#
# jetson.sh — ONE-COMMAND robot (Jetson) bring-up for RoboCorea.
#
# Run this on the ROBOT (Jetson Orin Nano) after a fresh boot. It brings up
# everything the robot needs to be operable + ready for the GUI, as managed
# systemd --user services (so they survive an SSH drop and a reboot):
#
#   ALWAYS-ON (started here):
#     • esp32-bridge.service  — serial <-> ROS 2 relay (drive/flippers/arm,
#                               thermal+mag, wheel odom). The core link.
#     • robot-manager.service — /robot/<stack>/{start,stop,status} for the GUI
#     • map-manager.service   — /robot/maps/{save,list,load,delete} for the GUI
#     • camera-streamer.service — auto-detect USB cams + ZED tap over SRT to the
#                               GUI, advertises them on ROS (skip: --no-camera)
#
#   ON-DEMAND (left for the operator to start from the GUI "Robot Systems"
#   window — Sensors → Mapping/Localization → Navigation), UNLESS you pass
#   --sensors, which also starts ZED+Lidar + SLAM now.
#
# Middleware is Zenoh (rmw_zenoh). Run scripts/setup-zenoh.sh ONCE per host
# first (it installs the always-on Zenoh router + makes rmw_zenoh the default
# RMW). This script checks that the router is up and warns if it is not.
#
# Usage:
#   ./jetson.sh                      # full bring-up, perception on-demand from GUI
#   ./jetson.sh --build              # colcon build the Jetson packages first
#   ./jetson.sh --sensors            # also start ZED+Lidar + SLAM immediately
#   ./jetson.sh --serial /dev/ttyCH341USB0   # pin the bridge to one serial device
#   ./jetson.sh --no-ekf             # bench odometry (ZED-only, no EKF fusion)
#   ./jetson.sh --no-camera          # don't start the camera streamer (SRT cams)
#   ./jetson.sh --no-bridge          # don't start esp32-bridge (managers only)
#   ./jetson.sh --domain 30          # use ROS_DOMAIN_ID 30 (default 20)
#   ./jetson.sh --status             # show what's running, change nothing
#   ./jetson.sh --stop               # stop everything this script starts
#
set -euo pipefail

# ── config ────────────────────────────────────────────────────────────────────
DOMAIN="${ROS_DOMAIN_ID:-20}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$SCRIPT_DIR/software/ros2_ws"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
USER_UNIT_DIR="$HOME/.config/systemd/user"

# Jetson-side packages the managers + the stacks they launch need built.
JETSON_PKGS=(rescue_interfaces mlx90640_msgs rescue_bringup esp32_bridge \
             dicerox_mapping rescue_nav rescue_mapping3d)

# Services this script owns. ALWAYS = brought up every run (subject to flags).
ALWAYS_ON=(esp32-bridge.service robot-manager.service map-manager.service camera-streamer.service)
ON_DEMAND=(rescue-sensors.target rescue-mapping.service)   # only with --sensors

log()  { printf '\033[1;32m[jetson]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[jetson] WARN:\033[0m %s\n' "$*" >&2; }
err()  { printf '\033[1;31m[jetson] ERROR:\033[0m %s\n' "$*" >&2; }

# ── args ──────────────────────────────────────────────────────────────────────
DO_BUILD=0 START_SENSORS=0 NO_CAMERA=0 NO_BRIDGE=0 NO_EKF=0 STATUS_ONLY=0 STOP_ALL=0
SERIAL_DEV=""
while [ $# -gt 0 ]; do
  case "$1" in
    --build)     DO_BUILD=1 ;;
    --sensors)   START_SENSORS=1 ;;
    --no-camera) NO_CAMERA=1 ;;
    --no-bridge) NO_BRIDGE=1 ;;
    --no-ekf)    NO_EKF=1 ;;
    --status)    STATUS_ONLY=1 ;;
    --stop)      STOP_ALL=1 ;;
    --serial)    SERIAL_DEV="${2:?--serial needs a device path}"; shift ;;
    --domain)    DOMAIN="${2:?--domain needs a number}"; shift ;;
    -h|--help)   sed -n '2,40p' "$0"; exit 0 ;;
    *) err "unknown arg '$1' (try --help)"; exit 2 ;;
  esac
  shift
done
export ROS_DOMAIN_ID="$DOMAIN"

# ── source ROS + workspace ────────────────────────────────────────────────────
[ -f "$ROS_SETUP" ] || { err "no ROS at $ROS_SETUP (set ROS_SETUP=...)"; exit 1; }
# ROS/ament setup files aren't `set -u`-safe — relax it just around the sources.
set +u; source "$ROS_SETUP"; set -u

if [ "$DO_BUILD" = 1 ]; then
  log "colcon build (${JETSON_PKGS[*]}) ..."
  # -Wno-dev suppresses CMake "dev" warnings from ROS's own modules (e.g. the
  # CMP0148 FindPythonInterp noise every rosidl message package emits).
  ( cd "$WS" && colcon build --symlink-install --packages-up-to "${JETSON_PKGS[@]}" \
      --cmake-args -Wno-dev )
fi
[ -f "$WS/install/setup.bash" ] || {
  err "workspace not built: $WS/install/setup.bash missing — run with --build"; exit 1; }
set +u; source "$WS/install/setup.bash"; set -u

UNITS_SRC="$WS/install/rescue_bringup/share/rescue_bringup/systemd"
[ -d "$UNITS_SRC" ] || { err "units not found at $UNITS_SRC (build rescue_bringup)"; exit 1; }

# ── status-only ───────────────────────────────────────────────────────────────
if [ "$STATUS_ONLY" = 1 ]; then
  log "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=${RMW_IMPLEMENTATION:-<unset>}"
  systemctl --user --no-pager status "${ALWAYS_ON[@]}" "${ON_DEMAND[@]}" 2>/dev/null || true
  log "robot_manager / map_manager services on the bus:"
  ros2 service list 2>/dev/null | grep -E '/robot/(maps|sensors|mapping|localization|navigation)/' || \
    warn "no /robot/* services seen (managers down? wrong domain? Zenoh router down?)"
  exit 0
fi

# ── stop-all ──────────────────────────────────────────────────────────────────
if [ "$STOP_ALL" = 1 ]; then
  log "stopping all RoboCorea services this script starts ..."
  systemctl --user stop rescue-navigation.service rescue-localization.service \
    rescue-mapping3d.service rescue-mapping.service rescue-sensors.target \
    "${ALWAYS_ON[@]}" 2>/dev/null || true
  log "stopped (the Zenoh router system service is left running)."
  exit 0
fi

# ── 1) install / refresh the systemd --user unit files ────────────────────────
log "installing systemd --user units from $UNITS_SRC (domain $DOMAIN)"
mkdir -p "$USER_UNIT_DIR"
cp -f "$UNITS_SRC"/*.service "$UNITS_SRC"/*.target "$USER_UNIT_DIR/"
# Pin the DDS/Zenoh partition domain in every unit to the chosen value.
if [ "$DOMAIN" != "20" ]; then
  sed -i "s/^Environment=ROS_DOMAIN_ID=.*/Environment=ROS_DOMAIN_ID=$DOMAIN/" \
    "$USER_UNIT_DIR"/*.service 2>/dev/null || true
  warn "domain set to $DOMAIN in the user units. The Zenoh ROUTER (system service) is"
  warn "still on its setup-zenoh.sh domain — re-run setup-zenoh.sh with ROS_DOMAIN_ID=$DOMAIN."
fi

# Optional explicit serial device for the bridge, via a drop-in (overrides the
# unit's empty default; removed when --serial is not given, so it scans again).
DROPIN="$USER_UNIT_DIR/esp32-bridge.service.d"
if [ -n "$SERIAL_DEV" ]; then
  mkdir -p "$DROPIN"
  printf '[Service]\nEnvironment=ROBOCOREA_SERIAL_PORT=%s\n' "$SERIAL_DEV" > "$DROPIN/serial.conf"
  log "bridge pinned to serial device: $SERIAL_DEV"
else
  rm -rf "$DROPIN"   # fall back to scanning serial_candidates (incl. ttyCH341USB*)
fi
systemctl --user daemon-reload

# ── 2) odometry mode (prod EKF vs bench ZED-only) ─────────────────────────────
if [ "$NO_EKF" = 1 ]; then
  systemctl --user set-environment ROBOCOREA_USE_EKF=false
  log "odometry: BENCH (ZED-only, no EKF) — rescue-mapping/localization use zed_planar_odom"
else
  systemctl --user unset-environment ROBOCOREA_USE_EKF 2>/dev/null || true
  log "odometry: PROD (robot_localization EKF fuses wheel + ZED VIO + ZED IMU)"
fi

# ── 3) headless persistence (survive SSH drop / logout) ───────────────────────
loginctl enable-linger "$USER" >/dev/null 2>&1 || \
  warn "enable-linger failed — services may stop on logout (sudo loginctl enable-linger $USER)"

# ── 4) Zenoh middleware sanity ────────────────────────────────────────────────
if systemctl is-active --quiet rmw-zenoh-router.service 2>/dev/null; then
  log "Zenoh router: active"
else
  warn "Zenoh router (rmw-zenoh-router.service) is NOT active."
  warn "  Run once:  sudo PEER_IP=<laptop-ip> bash $SCRIPT_DIR/scripts/setup-zenoh.sh"
fi
[ "${RMW_IMPLEMENTATION:-}" = "rmw_zenoh_cpp" ] || \
  warn "RMW_IMPLEMENTATION is '${RMW_IMPLEMENTATION:-<unset>}', expected rmw_zenoh_cpp (setup-zenoh.sh sets it)."

# ── 5) start the always-on services ───────────────────────────────────────────
start_set=(robot-manager.service map-manager.service)
[ "$NO_BRIDGE" = 0 ] && start_set+=(esp32-bridge.service)
[ "$NO_CAMERA" = 0 ] && start_set+=(camera-streamer.service)

log "enabling + (re)starting: ${start_set[*]}"
systemctl --user enable "${start_set[@]}" >/dev/null
systemctl --user restart "${start_set[@]}"

# ── 6) optional: bring perception up now (else the GUI does it on demand) ──────
if [ "$START_SENSORS" = 1 ]; then
  log "starting perception now: rescue-sensors.target + rescue-mapping.service"
  systemctl --user start rescue-sensors.target
  systemctl --user start rescue-mapping.service
fi

sleep 2

# ── report ────────────────────────────────────────────────────────────────────
ok=1
for u in "${start_set[@]}"; do
  if systemctl --user is-active --quiet "$u"; then
    log "$u: active"
  else
    err "$u: NOT active — logs: journalctl --user -u $u -e"; ok=0
  fi
done

log "robot_manager / map_manager services on the bus:"
ros2 service list 2>/dev/null | grep -E '/robot/(maps|sensors|mapping|navigation)/(start|save)' >/dev/null && \
  log "  /robot/* services visible — open the GUI 'Robot Systems' window on the laptop." || \
  warn "  no /robot/* services yet (give it a few seconds, or check the Zenoh router / domain)."

if [ "$ok" = 1 ]; then
  log "ready. logs:  journalctl --user -u esp32-bridge -u robot-manager -u map-manager -f"
else
  exit 1
fi
