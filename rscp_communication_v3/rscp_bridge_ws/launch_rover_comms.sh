#!/usr/bin/env bash
#
# launch_rover_comms.sh
# ---------------------
# Launches the RSCP bridge and mission orchestrator in separate tmux panes
# so their logs appear independently, colour-coded, and all in one window.
#
# If tmux is not available it falls back to a single terminal (both nodes
# in the same window, same as a plain ros2 launch call).
#
# EDIT THESE VALUES to match your setup:
ROS_DISTRO_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_DIR="$HOME/rscp_communication_v3/rscp_bridge_ws"
SERIAL_PORT="/dev/ttyUSB0"
BAUDRATE="115200"
# ---------------------------------------------------------------------------

set -e

source "$ROS_DISTRO_SETUP"
source "$WORKSPACE_DIR/install/setup.bash"

SESSION="rover_comms"


    echo "=============================================="
    echo " Launching RSCP bridge + mission orchestrator"
    echo " Port: $SERIAL_PORT"
    echo "=============================================="
    ros2 launch rscp_mission_orchestrator rover_bringup.launch.py \
        port:="$SERIAL_PORT" baudrate:="$BAUDRATE"

echo ""
echo "Nodes exited. Press Enter to close."
read -r
