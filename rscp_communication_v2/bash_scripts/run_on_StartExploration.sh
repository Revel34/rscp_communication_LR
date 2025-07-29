echo "Running StartExploration script! "
echo "Got $RSCP_MESSAGE_TYPE with payload: $RSCP_PAYLOAD"

ros2 topic pub -1 /send_response std_msgs/msg/String \
"{data: '{\"type\":\"ack\"}'}"

ros2 topic pub --once /ESP32_GIZ/led_state_topic std_msgs/Int8MultiArray "data: [0, 1, 0]"

bash /home/legendary/TrailblazerML/src/trailblazer_master/scripts/docker/autonomy_exploring.sh

ros2 topic pub -1 /send_response std_msgs/msg/String \
"{data: '{\"type\":\"distance\",\"distance\":5.41}'}"

ros2 topic pub -1 /send_response std_msgs/msg/String \
"{data: '{\"type\":\"task_finished\"}'}"

ros2 topic pub --once /ESP32_GIZ/led_state_topic std_msgs/Int8MultiArray "data: [1, 0, 0]"
