#!/usr/bin/env bash
#
# run_robot_manager.sh — one-shot Jetson bring-up for RoboCorea's always-on
# managers. Run this on the ROBOT (Jetson), not the workstation.
#
# It makes the GUI's "Robot Systems" window work by ensuring the two always-on
# controller nodes are up:
#   • robot_manager — /robot/<stack>/{start,stop,restart,status} for
#                     sensors · i2c · mapping · mapping3d · localization
#   • map_manager   — /robot/maps/{save,list,load,delete} (named 2-D/3-D maps)
#
# Those managers don't run drivers themselves — they ask the systemd --user units
# to start/stop (clean teardown via cgroups). So this script also installs/refreshes
# the unit files, then enables the two managers under systemd (persistent: they
# survive an SSH drop / reboot, and re-running this after a `colcon build` updates
# them).
#
# Usage:
#   ./run_robot_manager.sh              # set up + (re)start the managers
#   ./run_robot_manager.sh --build      # colcon build the Jetson packages first
#   ./run_robot_manager.sh --status     # just show what's running, don't change anything
#   ROS_DOMAIN_ID=20 ./run_robot_manager.sh   # override the DDS domain (default 20)
#
set -euo pipefail

# ── config ────────────────────────────────────────────────────────────────────
# MUST match the workstation/GUI domain (and the Environment= line in the units).
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$SCRIPT_DIR/software/ros2_ws"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

# Jetson-side packages the managers + the stacks they launch need built.
JETSON_PKGS=(rescue_interfaces rescue_bringup esp32_bridge jetson_sensors \
             dicerox_mapping rescue_nav rescue_mapping3d)

MANAGERS=(robot-manager.service map-manager.service)

log() { printf '\033[1;32m[run_robot_manager]\033[0m %s\n' "$*"; }
err() { printf '\033[1;31m[run_robot_manager] ERROR:\033[0m %s\n' "$*" >&2; }

# ── args ──────────────────────────────────────────────────────────────────────
DO_BUILD=0; STATUS_ONLY=0
for a in "$@"; do
  case "$a" in
    --build)  DO_BUILD=1 ;;
    --status) STATUS_ONLY=1 ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    *) err "unknown arg '$a'"; exit 2 ;;
  esac
done

# ── sanity ────────────────────────────────────────────────────────────────────
[ -f "$ROS_SETUP" ] || { err "no ROS at $ROS_SETUP (set ROS_SETUP=...)"; exit 1; }
# shellcheck disable=SC1090
source "$ROS_SETUP"

if [ "$DO_BUILD" = 1 ]; then
  log "colcon build (${JETSON_PKGS[*]}) ..."
  ( cd "$WS" && colcon build --symlink-install --packages-up-to "${JETSON_PKGS[@]}" )
fi

[ -f "$WS/install/setup.bash" ] || {
  err "workspace not built: $WS/install/setup.bash missing — run with --build first"; exit 1; }
# shellcheck disable=SC1090
source "$WS/install/setup.bash"

UNITS_SRC="$WS/install/rescue_bringup/share/rescue_bringup/systemd"
[ -d "$UNITS_SRC" ] || { err "units not found at $UNITS_SRC (build rescue_bringup)"; exit 1; }

# ── status-only ───────────────────────────────────────────────────────────────
if [ "$STATUS_ONLY" = 1 ]; then
  log "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  systemctl --user --no-pager status "${MANAGERS[@]}" || true
  log "manager services exposed:"; ros2 service list 2>/dev/null | grep -E '/robot/(maps|sensors|i2c|mapping|mapping3d|localization)/' || true
  exit 0
fi

# ── 1) install / refresh the systemd --user unit files ────────────────────────
log "installing systemd --user units from $UNITS_SRC"
mkdir -p "$HOME/.config/systemd/user"
cp -f "$UNITS_SRC"/*.service "$UNITS_SRC"/*.target "$HOME/.config/systemd/user/"
systemctl --user daemon-reload

# Warn if the units' DDS domain doesn't match what we're using.
if ! grep -q "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" "$HOME/.config/systemd/user/robot-manager.service"; then
  err "unit ROS_DOMAIN_ID differs from $ROS_DOMAIN_ID — edit Environment=ROS_DOMAIN_ID in"
  err "  ~/.config/systemd/user/*.service (must match the GUI), then re-run."
fi

# ── 2) run headless / over SSH without an active login session ────────────────
loginctl enable-linger "$USER" >/dev/null 2>&1 || \
  err "enable-linger failed — managers may stop when you log out (run: sudo loginctl enable-linger $USER)"

# ── 3) (re)start the always-on managers under systemd (persistent) ────────────
# The driver stacks (zed/lidar/SLAM/...) stay ON-DEMAND — started by the GUI via
# the managers — so they are NOT enabled here.
log "enabling + (re)starting: ${MANAGERS[*]}"
systemctl --user enable "${MANAGERS[@]}" >/dev/null
systemctl --user restart "${MANAGERS[@]}"
sleep 2

# ── report ────────────────────────────────────────────────────────────────────
ok=1
for u in "${MANAGERS[@]}"; do
  if systemctl --user is-active --quiet "$u"; then
    log "$u: active"
  else
    err "$u: NOT active — logs: journalctl --user -u $u -e"; ok=0
  fi
done

log "manager services on the bus:"
ros2 service list 2>/dev/null | grep -E '/robot/(maps|sensors|i2c|mapping|mapping3d|localization)/(start|save)' || \
  err "no /robot/* services seen yet (give it a moment, or check ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"

if [ "$ok" = 1 ]; then
  log "ready — open the GUI 'Robot Systems' window on the workstation."
  log "logs:  journalctl --user -u robot-manager.service -u map-manager.service -f"
else
  exit 1
fi
