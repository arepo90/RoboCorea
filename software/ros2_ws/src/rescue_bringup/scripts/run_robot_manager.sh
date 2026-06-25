#!/usr/bin/env bash
#
# Run the RoboCorea robot_manager by hand (the sensor / i2c / mapping start-stop
# service the GUI talks to). Handy when you're not using the systemd unit
# (robot-manager.service). Ctrl-C to stop.
#
set -e

# Resolve the ros2_ws root from this script's own location
# (…/software/ros2_ws/src/rescue_bringup/scripts), so it works wherever the repo
# is cloned and from any current directory.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$SCRIPT_DIR/../../.." && pwd)"   # …/software/ros2_ws

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# Must match the workstation/GUI domain.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"

echo "robot_manager: ws=$WS  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
exec ros2 run rescue_bringup robot_manager
