#!/usr/bin/env bash
#
# run_on_NavigateToGPS.sh
# Invoked by RSCP.py on 'navigate_to_gps' messages.
# Parses RSCP_PAYLOAD for latitude & longitude, then
# launches straight_to_point.py with those coordinates.

set -euo pipefail

# Make sure RSCP_PAYLOAD is set
if [[ -z "${RSCP_PAYLOAD-}" ]]; then
  echo "Error: RSCP_PAYLOAD is empty or unset." >&2
  exit 1
fi

echo "Running NavigateToGPS script! "
echo "Got $RSCP_MESSAGE_TYPE with payload: $RSCP_PAYLOAD"

payload="$RSCP_PAYLOAD"

# A simple regex for floats (e.g. 51.123, -0.456)
float_re='-?[0-9]+(\.[0-9]+)?'

# Extract latitude and longitude
lat=$(echo "$payload" | grep -oP '(?<=latitude: )'"$float_re")
lon=$(echo "$payload" | grep -oP '(?<=longitude: )'"$float_re")

if [[ -z "$lat" || -z "$lon" ]]; then
  echo "Error: Could not parse latitude and/or longitude from payload:" >&2
  echo "  $payload" >&2
  exit 2
fi

# Path to your straight_to_point.py — adjust as needed:
SCRIPT_PATH="$HOME/eric_anatolian/rscp_communication/rover_scripts/straight_to_point.py"
stage_state=$(cat stage_state.txt)

echo "-------------------------- $lat   $lon   -----------------"

ros2 topic pub -1 /send_response std_msgs/msg/String \
"{data: '{\"type\":\"ack\"}'}"

ros2 topic pub --once /ESP32_GIZ/led_state_topic std_msgs/Int8MultiArray "data: [0, 1, 0]"

# Launch the ROS2‐based navigation script

python3 "$SCRIPT_PATH" --lat "$lat" --lon "$lon"


if [[ "${stage_state}" == "4" ]]; then
  ros2 topic pub -1 /send_response std_msgs/msg/String \
  "{data: '{\"type\":\"task_finished\"}'}"
  ros2 topic pub --once /ESP32_GIZ/led_state_topic std_msgs/Int8MultiArray "data: [1, 0, 0]"
 # time.sleep(1)
  ros2 topic pub --once /ESP32_GIZ/led_state_topic std_msgs/Int8MultiArray "data: [0, 1, 0]"
  bash /home/legendary/TrailblazerML/src/trailblazer_master/scripts/docker/autonomy_aruco_searching.sh
  bash /home/legendary/TrailblazerML/src/trailblazer_master/scripts/docker/autonomy_exploring.sh
else
  ros2 topic pub -1 /send_response std_msgs/msg/String \
  "{data: '{\"type\":\"task_finished\"}'}"
  ros2 topic pub --once /ESP32_GIZ/led_state_topic std_msgs/Int8MultiArray "data: [1, 0, 0]"
  bash /home/legendary/TrailblazerML/src/trailblazer_master/scripts/docker/autonomy_aruco_searching.sh
fi
