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
WORKSPACE_DIR="$HOME/rscp_communication_LR/rscp_communication_v3/rscp_bridge_ws"
SERIAL_PORT="/dev/pts/1"
BAUDRATE="115200"
# ---------------------------------------------------------------------------

set -e

source "$ROS_DISTRO_SETUP"
source "$WORKSPACE_DIR/install/setup.bash"

SESSION="rover_comms"

# ---- tmux path (separate panes) -------------------------------------------
if command -v tmux &> /dev/null; then
    echo "Starting rover nodes in tmux session '$SESSION'..."
    echo "  Attach with:  tmux attach -t $SESSION"
    echo "  Detach with:  Ctrl+B then D"
    echo "  Kill all:     tmux kill-session -t $SESSION"

    # Kill any existing session with the same name
    tmux kill-session -t "$SESSION" 2>/dev/null || true

    # Create a new session with the first pane for the bridge
    tmux new-session -d -s "$SESSION" -x 220 -y 50 \
        -e "ROS_DISTRO_SETUP=$ROS_DISTRO_SETUP" \
        -e "WORKSPACE_DIR=$WORKSPACE_DIR"

    # Source and run bridge in pane 0
    tmux send-keys -t "$SESSION:0" \
        "source $ROS_DISTRO_SETUP && source $WORKSPACE_DIR/install/setup.bash && \
         echo '=== RSCP BRIDGE ===' && \
         ros2 run rscp_bridge_node rscp_bridge_node \
           --ros-args -p port:=$SERIAL_PORT -p baudrate:=$BAUDRATE" Enter

    # Split horizontally — pane 1 for orchestrator
    tmux split-window -v -t "$SESSION:0"
    tmux send-keys -t "$SESSION:0.1" \
        "source $ROS_DISTRO_SETUP && source $WORKSPACE_DIR/install/setup.bash && \
         echo '=== MISSION ORCHESTRATOR ===' && \
         ros2 run rscp_mission_orchestrator mission_orchestrator" Enter

    # Give pane 0 slightly more height (bridge is busier)
    tmux resize-pane -t "$SESSION:0.0" -y 30

    # Attach to the session
    tmux attach-session -t "$SESSION"

# ---- Fallback: no tmux (single terminal) ----------------------------------
else
    echo "tmux not found — running both nodes in one terminal."
    echo "Install tmux for separate panes:  sudo apt install tmux"
    echo ""
    echo "=============================================="
    echo " Launching RSCP bridge + mission orchestrator"
    echo " Port: $SERIAL_PORT"
    echo "=============================================="
    ros2 launch rscp_mission_orchestrator rover_bringup.launch.py \
        port:="$SERIAL_PORT" baudrate:="$BAUDRATE"
fi

echo ""
echo "Nodes exited. Press Enter to close."
read -r
