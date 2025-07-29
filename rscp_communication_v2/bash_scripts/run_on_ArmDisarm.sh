#!/bin/bash
# ================================================================
# RSCP Arm/Disarm handler – V2
# ================================================================

# ---------- EDIT THESE LISTS ------------------------------------
# Scripts to kill when the system is ARMED  (payload == true)
SCRIPTS_TO_KILL_ARM=(
    "disarm.py"
    
)

# Scripts to kill when the system is DISARMED (payload == false)
SCRIPTS_TO_KILL_DISARM=(
    "straight_to_point.py"
    "shelly_main.py"
    "shelly_main_cold.py"
    "shelly_main_alt.py"
    "set_stage.py"
    "boxy_spiral.py"
    "archimedean_spiral.py"
    "autonomy_exploring.sh"
    "autonomy_aruco_searching.sh"
    "circle_anon.py"
    "rob_spirale.py"
)

# Action scripts
SCRIPT_TRUE_PATH="$HOME/eric_anatolian/rscp_communication/rover_scripts/arm.py"      # runs when armed
SCRIPT_FALSE_PATH="$HOME/eric_anatolian/rscp_communication/rover_scripts/disarm.py"   # runs when disarmed

LOG_FILE="$HOME/logs/rscp_payload_handler.log"
# ---------------------------------------------------------------

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$LOG_FILE"; }

kill_scripts() {
    local -n list=$1                          # nameref → pass array name
    for script in "${list[@]}"; do
        pids=$(pgrep -f "python.*${script}")
        if [[ -n $pids ]]; then
            log "Killing $script (PIDs: $pids)"
            xargs -r kill -TERM <<<"$pids"
            sleep 2
            xargs -r kill -KILL <<<"$(pgrep -f "python.*${script}")"
        else
            log "No running instance of $script"
        fi
    done
}

run_py() {
    local path=$1 ctx=$2
    if [[ -f $path ]]; then
        log "Launching $path ($ctx)"
        ros2 topic pub -1 /send_response std_msgs/msg/String \
"{data: '{\"type\":\"ack\"}'}"
        python3 "$path"
        log "Started PID $!"
    else
        log "ERROR – script not found: $path"
    fi
}

# ============================ MAIN ==============================
mkdir -p "$(dirname "$LOG_FILE")"
payload="${RSCP_PAYLOAD:-}"

case $payload in
    *[Tt][Rr][Uu][Ee]*)
        log "Payload TRUE – ARM"
        kill_scripts SCRIPTS_TO_KILL_ARM
        run_py "$SCRIPT_TRUE_PATH"  "payload_true"
        ;;
    *[Ff][Aa][Ll][Ss][Ee]*)
        log "Payload FALSE – DISARM"
        kill_scripts SCRIPTS_TO_KILL_DISARM
        run_py "$SCRIPT_FALSE_PATH" "payload_false"
        ;;
    *)
        log "ERROR – unknown payload: $payload"
        exit 1
        ;;
esac
