exec /home/legendary/TrailblazerML/src/trailblazer_master/scripts/docker/autonomy_bringup.sh

socat -d -d pty,raw,echo=0 pty,raw,echo=0

cd ~/eric_anatolian/rscp_communication/comms_ws
. install/setup.bash
ros2 run comms_rscp listener_node --ros-args -p device_path:=/dev/pts/

cd ~/eric_anatolian/rscp_communication/scripts
./rscp_cli goto 38.135742 27.821507 1180 | xxd -r -p > /dev/pts/
./rscp_cli setstage 1 | xxd -r -p > /dev/pts/
./rscp_cli explore | xxd -r -p > /dev/pts/
./rscp_cli search 38.135742 27.821507 50 | xxd -r -p > /dev/pts/
./rscp_cli disarm | xxd -r -p > /dev/pts/
./rscp_cli arm | xxd -r -p > /dev/pts/

cd ~/eric_anatolian/rscp_communication/comms_ws
. install/setup.bash
ros2 run comms_rscp_send rscp_response_node --ros-args -p serial_port:=/dev/pts/ -p baud:=9600 
