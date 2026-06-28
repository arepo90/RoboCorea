#!/usr/bin/env bash
#
# diagnose.sh — RoboCorea hardware diagnostics dispatcher.
#
# Read-only checks that turn esp32_bridge telemetry into a "what's wrong" report.
# Nothing here commands the robot. Run it on EITHER machine (it just needs to see
# the robot on the ROS graph — i.e. the Zenoh link up + esp32-bridge running).
#
# Usage:
#   ./diagnose.sh                 # one-shot health snapshot of everything (~6s)
#   ./diagnose.sh link            # comms / which boards are talking      (live)
#   ./diagnose.sh ppm             # RC / PPM channels + e-stop/vflip       (live)
#   ./diagnose.sh can             # CAN presence: VESCs + arm joints       (live)
#   ./diagnose.sh sensors         # arm-PCB I2C sensors (thermal/mag)      (live)
#   ./diagnose.sh all --seconds 10   # longer snapshot window
#   ./diagnose.sh <sub> --once    # one snapshot of a subsystem, then exit
#   ./diagnose.sh --domain 30 ... # override ROS_DOMAIN_ID (default 20)
#
# Each live monitor refreshes ~1 Hz; Ctrl-C to stop.
#
set -euo pipefail

DOMAIN="${ROS_DOMAIN_ID:-20}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$SCRIPT_DIR/software/ros2_ws"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

err() { printf '\033[1;31m[diagnose] ERROR:\033[0m %s\n' "$*" >&2; }

# Pull --domain out of the args; pass the rest straight to the diag node.
SUB=""
PASS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --domain) DOMAIN="${2:?--domain needs a number}"; shift ;;
    -h|--help) sed -n '2,22p' "$0"; exit 0 ;;
    link|ppm|can|sensors|all) SUB="$1" ;;
    -*) PASS+=("$1") ;;                       # a flag for the diag node
    *) err "unknown subcommand '$1' (want: link|ppm|can|sensors|all)"; exit 2 ;;
  esac
  shift
done
export ROS_DOMAIN_ID="$DOMAIN"

[ -f "$ROS_SETUP" ] || { err "no ROS at $ROS_SETUP"; exit 1; }
set +u; source "$ROS_SETUP"; set -u
[ -f "$WS/install/setup.bash" ] || { err "workspace not built ($WS) — run ./jetson.sh --build or ./laptop.sh --build"; exit 1; }
set +u; source "$WS/install/setup.bash"; set -u

# Map the subcommand to a diag entry point. `--once` on a subsystem disables the
# live default by collecting a fixed window instead.
case "$SUB" in
  ""|all)  exec ros2 run esp32_bridge diag_all "${PASS[@]}" ;;
  link)    exec ros2 run esp32_bridge diag_link "${PASS[@]}" ;;
  ppm)     exec ros2 run esp32_bridge diag_ppm "${PASS[@]}" ;;
  can)     exec ros2 run esp32_bridge diag_can "${PASS[@]}" ;;
  sensors) exec ros2 run esp32_bridge diag_sensors "${PASS[@]}" ;;
  *)       err "unknown subcommand '$SUB'"; exit 2 ;;
esac
