#!/usr/bin/env bash
#
# laptop.sh — ONE-COMMAND operator (workstation) bring-up for RoboCorea.
#
# Run this on the OPERATOR LAPTOP. It launches the full operator console in the
# foreground (Ctrl-C to quit):
#
#   • the Qt6 GUI (video, dashboard, odometry, digital twin, Robot Systems
#     window: perception stacks + named maps),
#   • the SDLS arm servo + flipper_state + robot_state_publisher (digital twin),
#   • optional gamepad arm teleop (default on; --no-joystick to disable).
#
# The robot-side relay (esp32_bridge), perception and maps run on the JETSON —
# start those with ./jetson.sh over there. This host talks to the robot over
# Zenoh (rmw_zenoh); run scripts/setup-zenoh.sh ONCE on this laptop first.
#
# Usage:
#   ./laptop.sh                 # GUI + twin + servo + gamepad teleop
#   ./laptop.sh --build         # colcon build the workstation packages first
#   ./laptop.sh --no-joystick   # no gamepad teleop (keyboard/RViz only)
#   ./laptop.sh --rviz          # also open RViz with the twin config
#   ./laptop.sh --domain 30     # ROS_DOMAIN_ID 30 (default 20; must match robot)
#
set -euo pipefail

# ── config ────────────────────────────────────────────────────────────────────
DOMAIN="${ROS_DOMAIN_ID:-20}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$SCRIPT_DIR/software/ros2_ws"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

# Workstation packages the GUI bring-up needs.
WS_PKGS=(rescue_interfaces mlx90640_msgs arm_description arm_teleop gui)

log()  { printf '\033[1;36m[laptop]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[laptop] WARN:\033[0m %s\n' "$*" >&2; }
err()  { printf '\033[1;31m[laptop] ERROR:\033[0m %s\n' "$*" >&2; }

# ── args ──────────────────────────────────────────────────────────────────────
DO_BUILD=0 JOYSTICK=true USE_RVIZ=false
while [ $# -gt 0 ]; do
  case "$1" in
    --build)       DO_BUILD=1 ;;
    --no-joystick) JOYSTICK=false ;;
    --rviz)        USE_RVIZ=true ;;
    --domain)      DOMAIN="${2:?--domain needs a number}"; shift ;;
    -h|--help)     sed -n '2,28p' "$0"; exit 0 ;;
    *) err "unknown arg '$1' (try --help)"; exit 2 ;;
  esac
  shift
done
export ROS_DOMAIN_ID="$DOMAIN"

# ── source ROS + workspace ────────────────────────────────────────────────────
[ -f "$ROS_SETUP" ] || { err "no ROS at $ROS_SETUP (set ROS_SETUP=...)"; exit 1; }
set +u; source "$ROS_SETUP"; set -u

if [ "$DO_BUILD" = 1 ]; then
  log "colcon build (${WS_PKGS[*]}) ..."
  ( cd "$WS" && colcon build --symlink-install --packages-up-to "${WS_PKGS[@]}" )
fi
[ -f "$WS/install/setup.bash" ] || {
  err "workspace not built: $WS/install/setup.bash missing — run with --build"; exit 1; }
set +u; source "$WS/install/setup.bash"; set -u

# ── Zenoh middleware sanity ────────────────────────────────────────────────────
if systemctl is-active --quiet rmw-zenoh-router.service 2>/dev/null; then
  log "Zenoh router: active"
else
  warn "Zenoh router (rmw-zenoh-router.service) is NOT active on this laptop."
  warn "  Run once:  sudo PEER_IP=<jetson-ip> bash $SCRIPT_DIR/scripts/setup-zenoh.sh"
fi
[ "${RMW_IMPLEMENTATION:-}" = "rmw_zenoh_cpp" ] || \
  warn "RMW_IMPLEMENTATION is '${RMW_IMPLEMENTATION:-<unset>}', expected rmw_zenoh_cpp (setup-zenoh.sh sets it)."

# ── launch the operator console (foreground) ──────────────────────────────────
log "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  joystick=$JOYSTICK  rviz=$USE_RVIZ"
log "launching GUI + digital twin + arm servo ...  (Ctrl-C to quit)"
exec ros2 launch gui bringup.launch.py joystick:="$JOYSTICK" use_rviz:="$USE_RVIZ"
