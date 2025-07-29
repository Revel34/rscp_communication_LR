#!/usr/bin/env bash
#
set -euo pipefail
# -e means: Exit immediately if a pipeline (see Pipelines), which may consist of a single simple command (see Simple Commands), a list (see Lists of Commands), or a compound command (see Compound Commands) returns a non-zero status.

# - u means: Treat unset variables and parameters other than the special parameters ‘@’ or ‘*’, or array variables subscripted with ‘@’ or ‘*’, as an error when performing parameter expansion. An error message will be written to the standard error, and a non-interactive shell will exit.

# -o means: Set the option corresponding to option-name. If -o is supplied with no option-name, set prints the current shell options settings. If +o is supplied with no option-name, set prints a series of set commands to recreate the current option settings on the standard output. Valid option names are: - pipefail: If set, the return value of a pipeline is the value of the last (rightmost) command to exit with a non-zero status, or zero if all commands in the pipeline exit successfully. This option is disabled by default.


# Make sure RSCP_PAYLOAD is set
# RSCP_MESSAGE_TYPE="string"
# RSCP_PAYLOAD="value: 7"

if [[ -z "${RSCP_PAYLOAD}" || -z "${RSCP_MESSAGE_TYPE}" ]]; then
  echo "Error: RSCP_PAYLOAD or RSCP_MESSAGE_TYPE is empty or unset." >&2
  exit 1
fi

echo "Running SetStage script! "
echo "Got ${RSCP_MESSAGE_TYPE} with payload: ${RSCP_PAYLOAD}"

payload="$RSCP_PAYLOAD"

# A simple regex for floats (e.g. 51.123, -0.456)
int_re='(([0-9])?)'
# Extract latitude and longitude
stage_num=$(echo "$payload" | grep -oP '(?<=value: )'"$int_re")

if [[ -z "${stage_num}" ]]; then
  echo "Error: Could not parse stage from payload:" >&2
  echo "  ${payload}" >&2
  exit 2
fi


# Path to your straight_to_point.py — adjust as needed:
SCRIPT_PATH="$HOME/eric_anatolian/rscp_communication/rover_scripts/set_stage.py"

ros2 topic pub -1 /send_response std_msgs/msg/String \
"{data: '{\"type\":\"ack\"}'}"

# Launch the ROS2‐based navigation script
exec python3 "$SCRIPT_PATH" -s "$stage_num"



