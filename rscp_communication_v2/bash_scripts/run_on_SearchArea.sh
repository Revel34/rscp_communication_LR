#!/usr/bin/env bash
#
set -euo pipefail
# -e means: Exit immediately if a pipeline (see Pipelines), which may consist of a single simple command (see Simple Commands), a list (see Lists of Commands), or a compound command (see Compound Commands) returns a non-zero status.

# - u means: Treat unset variables and parameters other than the special parameters ‘@’ or ‘*’, or array variables subscripted with ‘@’ or ‘*’, as an error when performing parameter expansion. An error message will be written to the standard error, and a non-interactive shell will exit.

# -o means: Set the option corresponding to option-name. If -o is supplied with no option-name, set prints the current shell options settings. If +o is supplied with no option-name, set prints a series of set commands to recreate the current option settings on the standard output. Valid option names are: - pipefail: If set, the return value of a pipeline is the value of the last (rightmost) command to exit with a non-zero status, or zero if all commands in the pipeline exit successfully. This option is disabled by default.


# RSCP_MESSAGE_TYPE=""
# RSCP_PAYLOAD="latitude: 1.00,longitude: 2.00,radius: 3.001"

# Make sure RSCP_PAYLOAD is set
echo "Entering bash script."

if [[ -z "${RSCP_PAYLOAD}" || -z "${RSCP_MESSAGE_TYPE}" ]]; then
  echo "Error: RSCP_PAYLOAD or RSCP_MESSAGE_TYPE is emptyor unset." >&2
  exit 1
fi

echo "Running SearchArea script!"
echo "Got $RSCP_MESSAGE_TYPE with payload: $RSCP_PAYLOAD"

payload="$RSCP_PAYLOAD"

# A simple regex for floats (e.g. 51.123, -0.456)
float_re='-?[0-9]+(\.[0-9]+)?'

# Extract latitude and longitude and radius
lat=$(echo "$payload" | grep -oP '(?<=latitude: )'"$float_re")
lon=$(echo "$payload" | grep -oP '(?<=longitude: )'"$float_re")
radius=$(echo "$payload" | grep -oP '(?<=radius: )'"$float_re")

if [[ -z "$radius" || -z "$lat" || -z "$lon" ]]; then
  echo "Error: Could not parse latitude, longitude or radius from payload:" >&2
  echo "  $payload" >&2
  exit 2
fi

SCRIPT_PATH_GO_TO_CENTER_FIRST="$HOME/eric_anatolian/rscp_communication/rover_scripts/straight_to_point.py"
stage_state=$(cat stage_state.txt)

if [[ "${stage_state}" == "1" ]]; then
  echo "Running alt script!"
  SCRIPT_PATH_SEARCH_AREA="$HOME/eric_anatolian/rscp_communication/rover_scripts/shelly_main_alt.py"
fi

if [[ "${stage_state}" == "2" ]]; then
  echo "Running cold script!"
  SCRIPT_PATH_SEARCH_AREA="$HOME/eric_anatolian/rscp_communication/rover_scripts/shelly_main_cold.py"
fi

ros2 topic pub -1 /send_response std_msgs/msg/String \
"{data: '{\"type\":\"ack\"}'}"

echo "Decoded by bash from payload: ${lat}${lon}${radius}"
ros2 topic pub --once /ESP32_GIZ/led_state_topic std_msgs/Int8MultiArray "data: [0, 1, 0]"


if [[ "${stage_state}" == "1" ]]; then
  echo "Running drop script!"
  python3 "$SCRIPT_PATH_GO_TO_CENTER_FIRST" --lat "$lat" --lon "$lon"
  python3 "$SCRIPT_PATH_GO_TO_CENTER_FIRST" --lat "39.9011357" --lon "32.7702004"
  python3 "$HOME/eric_anatolian/rscp_communication/rover_scripts/otworz_kalpe.py"
  ros2 topic pub -1 /send_response std_msgs/msg/String \
  "{data: '{\"type\":\"gps\",\"latitude\":39.9011357,\"longitude\":32.7702004,\"altitude\":0.0}'}"
else
  python3 "$HOME/eric_anatolian/rscp_communication/rover_scripts/rob_spirale.py" --lat "$lat" --lon "$lon"	
 # python3 "$SCRIPT_PATH_GO_TO_CENTER_FIRST" --lat "$lat" --lon "$lon"
 # python3 "$SCRIPT_PATH_SEARCH_AREA" -r "$radius"
fi

ros2 topic pub -1 /send_response std_msgs/msg/String \
"{data: '{\"type\":\"task_finished\"}'}"

ros2 topic pub --once /ESP32_GIZ/led_state_topic std_msgs/Int8MultiArray "data: [1, 0, 0]"

echo "Success in executing rover_scripts"
echo "Exiting bash script..."
