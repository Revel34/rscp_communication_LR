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
BAUDRATE="115200"
# ---------------------------------------------------------------------------
# Cable fingerprint — Prolific PL2303, serial EIAXb153609
# These must match the physical cable; run `udevadm info --name=/dev/ttyUSBx`
# to verify if you ever swap cables.
CABLE_VENDOR="067b"
CABLE_PRODUCT="23a3"
CABLE_SERIAL="EIAXb153609"
# ---------------------------------------------------------------------------

set -e

# Always keep the terminal open so any error message is visible.
# Without this, set -e silently closes the window on the first failure.
trap 'echo ""; echo "--- Script stopped. Press Enter to close. ---"; read -r' EXIT

# ---------------------------------------------------------------------------
# Auto-detect the RSCP serial port by matching vendor, product, and serial
# number. Walks every /dev/ttyUSB* and /dev/ttyACM* device and returns the
# first one whose udev properties match all three identifiers.
# ---------------------------------------------------------------------------
detect_serial_port() {
    for dev in /dev/ttyUSB* /dev/ttyACM*; do
        # Skip glob literals (no devices present)
        [ -e "$dev" ] || continue

        # Pull the three properties we care about in one udevadm call
        props=$(udevadm info --query=property --name="$dev" 2>/dev/null)

        vendor=$(echo "$props"  | grep '^ID_VENDOR_ID='   | cut -d= -f2)
        product=$(echo "$props" | grep '^ID_MODEL_ID='    | cut -d= -f2)
        serial=$(echo "$props"  | grep '^ID_SERIAL_SHORT=' | cut -d= -f2)

        if [ "$vendor"  = "$CABLE_VENDOR"  ] &&
           [ "$product" = "$CABLE_PRODUCT" ] &&
           [ "$serial"  = "$CABLE_SERIAL"  ]; then
            echo "$dev"
            return 0
        fi
    done
    return 1   # not found
}

# ---------------------------------------------------------------------------
# Resolve the port — manual override first, then udev symlink, then live scan.
# To override:  SERIAL_PORT=/dev/ttyUSB1 ./launch_rover_comms.sh
# ---------------------------------------------------------------------------
if [ -n "$SERIAL_PORT" ]; then
    echo "[port] Manual override: $SERIAL_PORT"
elif [ -L /dev/rscp_host ] && [ -e /dev/rscp_host ]; then
    SERIAL_PORT=$(readlink -f /dev/rscp_host)
    echo "[port] Using udev symlink: /dev/rscp_host -> $SERIAL_PORT"
else  # no override, no symlink — scan
    echo "[port] udev symlink not present; scanning USB devices..."
    SERIAL_PORT=$(detect_serial_port) || true

    if [ -z "$SERIAL_PORT" ]; then
        echo ""
        echo "ERROR: RSCP cable not found!"
        echo "  Expected: idVendor=$CABLE_VENDOR  idProduct=$CABLE_PRODUCT  serial=$CABLE_SERIAL"
        echo ""
        echo "Troubleshooting:"
        echo "  1. Check the cable is plugged in."
        echo "  2. Run: dmesg | tail -20"
        echo "  3. Run: ls /dev/ttyUSB* /dev/ttyACM*"
        echo "  4. If a different cable: update CABLE_VENDOR/PRODUCT/SERIAL in this script."
        echo ""
        read -rp "Press Enter to exit."
        exit 1
    fi

    echo "[port] Cable found at: $SERIAL_PORT"
fi

source "$ROS_DISTRO_SETUP"
source "$WORKSPACE_DIR/install/setup.bash"

SESSION="rover_comms"

echo "=============================================="
echo " Launching RSCP bridge + mission orchestrator"
echo " Port   : $SERIAL_PORT"
echo " Baudrate: $BAUDRATE"
echo "=============================================="
ros2 launch rscp_mission_orchestrator rover_bringup.launch.py \
    port:="$SERIAL_PORT" baudrate:="$BAUDRATE"

echo "Nodes exited cleanly."
