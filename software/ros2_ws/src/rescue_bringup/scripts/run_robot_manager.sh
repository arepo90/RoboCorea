#!/usr/bin/env bash
#
# Run the RoboCorea robot_manager by hand (the sensor / mapping start-stop
# service the GUI talks to). Handy when you're not using the systemd unit
# (robot-manager.service). Ctrl-C to stop.
#
# Works whether it's run from inside the repo OR copied somewhere like ~/.
# Override the workspace with:  ROBOCOREA_WS=/path/to/software/ros2_ws ./run_robot_manager.sh
#
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Try, in order: explicit override, common clone locations, then relative to the
# script (when it's still inside the repo). First one with install/setup.bash wins.
CANDIDATES=(
    "$ROBOCOREA_WS"
    "$HOME/RoboCorea/software/ros2_ws"
    "$HOME/Projects/Robotics/RoboCorea/software/ros2_ws"
    "$SCRIPT_DIR/../../.."
)
WS=""
for c in "${CANDIDATES[@]}"; do
    if [ -n "$c" ] && [ -f "$c/install/setup.bash" ]; then
        WS="$(cd "$c" && pwd)"
        break
    fi
done
if [ -z "$WS" ]; then
    echo "ERROR: could not find the ros2_ws (looked for install/setup.bash)."
    echo "       Set ROBOCOREA_WS=/path/to/software/ros2_ws and re-run."
    exit 1
fi

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# Must match the workstation/GUI domain.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"

echo "robot_manager: ws=$WS  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
exec ros2 run rescue_bringup robot_manager
