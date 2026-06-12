# RSCP Bridge Node

A ROS2 node that bridges a rover's internal ROS2 network with an external
Host Module using the **Rover Satellite Communications Protocol (RSCP)**,
developed for the **Anatolian Rover Challenge (ARC)**.

This node acts purely as a **translator**. It does not control motors, run
navigation, or execute any mission logic. Its only job is to move messages
between a serial link (RSCP / Protobuf / COBS) and two ROS2 topics
(JSON-encoded `std_msgs/String`). All actual rover behaviour — navigation,
arming, exploration, science tasks — lives in separate ROS2 nodes that talk
to this bridge over those topics.

```
Host Module  <--serial (COBS + Protobuf)-->  [ rscp_bridge_node ]  <--ROS2 topics (JSON)-->  Rover nodes
                                                                                              (state machine,
                                                                                               Nav2, arm
                                                                                               controller, etc.)
```

---

## How it works

### Protocol background

RSCP messages are Google Protobuf objects (`RequestEnvelope` from the host to
the rover, `ResponseEnvelope` from the rover to the host), serialized to
bytes, encoded with **COBS** (Consistent Overhead Byte Stuffing), and
terminated with a single `0x00` delimiter byte on the serial line.

- **Receiving (Host → Rover):** the node reads raw bytes from the serial
  port, buffers them, and on every `0x00` byte it COBS-decodes the buffered
  frame and parses it as a `RequestEnvelope`.
- **Sending (Rover → Host):** the node builds a `ResponseEnvelope`,
  serializes it, COBS-encodes the bytes, appends `0x00`, and writes the
  result to the serial port.

### ROS2 interface

| Direction | Topic | Type | Description |
|---|---|---|---|
| Host → ROS2 | `rover_missions/recv_rscp` | `std_msgs/String` (JSON) | Commands decoded from the Host Module |
| ROS2 → Host | `rover_missions/send_rscp` | `std_msgs/String` (JSON) | Status/results to send to the Host Module |

#### Messages published on `rover_missions/recv_rscp`

| RSCP request | JSON published |
|---|---|
| `SetStage(stage)` | `{"type": "set_stage", "stage": <int>}` |
| `ArmDisarm(value)` | `{"type": "arm_disarm", "arm": <bool>}` |
| `SearchArea(center, radius)` | `{"type": "search_area", "lat": <float>, "lon": <float>, "rad": <float>}` |
| `NavigateToGPS(coordinate)` | `{"type": "navigate_to_gps", "lat": <float>, "lon": <float>}` |
| `StartExploration` | `{"type": "start_exploration"}` |

#### Messages accepted on `rover_missions/send_rscp`

| JSON published by your node | RSCP response sent |
|---|---|
| `{"type": "acknowledge"}` | `Acknowledge {}` |
| `{"type": "task_completed"}` (or `"task_finished"`) | `TaskFinished {}` |
| `{"type": "gps", "lat": <float>, "lon": <float>, "alt": <float optional>}` | `GPSCoordinate` |
| `{"type": "distance", "distance": <float>}` | `distance` (double) |
| `{"type": "message", "message": "<text>"}` | `message` (string) |

> **Note on `ArmDisarm`:** this is a command **from** the host. The bridge
> publishes it on `recv_rscp` as `{"type": "arm_disarm", "arm": true/false}`.
> Your arm-control node should perform the arm/disarm action and then publish
> `{"type": "acknowledge"}` on `send_rscp` once it's done. RSCP acknowledges
> carry no identifying information — the host infers what was acknowledged
> from message order, so make sure your nodes publish acks in the same order
> the corresponding commands arrived.

### Robustness features

- **Non-blocking serial I/O:** a ROS2 timer (default 50 Hz) polls
  `serial.in_waiting` and reads available bytes without blocking the ROS2
  executor.
- **Automatic reconnection:** if the serial port is missing or drops, the
  node logs a warning and retries opening it periodically (default every 2
  seconds) without crashing.
- **Buffer overflow protection:** if more than 8 KB accumulate without a
  frame delimiter, the buffer is discarded to resynchronise with the stream.
- **Per-frame error isolation:** a corrupted COBS frame or malformed
  protobuf message is logged and dropped — it never takes down the node.
- **Thread-safe serial access:** all reads and writes are guarded by a lock,
  so the node is safe to run under a multi-threaded executor.
- **Detailed logging:** every inbound and outbound message is logged at
  `info` level; raw byte frames are logged at `debug` level.

---

## Requirements

- **OS:** Ubuntu 22.04
- **ROS2:** Humble Hawksbill (recommended for Ubuntu 22.04)
- **Python:** 3.10 (ships with Ubuntu 22.04 / ROS2 Humble)
- **Python packages:**
  - `pyserial`
  - `cobs`
  - `rscp_protobuf` (generated classes from the [RSCP repository](https://github.com/anatolianroverchallenge/rscp))

---

## Installation

### 1. Install ROS2 Humble

If you don't already have ROS2 installed, follow the
[official ROS2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
for Ubuntu 22.04.

### 2. Install Python dependencies

```bash
sudo apt update
sudo apt install python3-pip

pip3 install pyserial cobs
pip3 install https://github.com/anatolianroverchallenge/rscp/releases/latest/download/rscp_protobuf.zip
```

> Always install the latest `rscp_protobuf` release to stay compatible with
> the current RSCP protocol version. Check the
> [releases page](https://github.com/anatolianroverchallenge/rscp/releases)
> for updates.

### 3. Add the node to a ROS2 package

Place `rscp_bridge_node.py` inside the `scripts/` (or equivalent) directory of
a ROS2 Python package, e.g.:

```
your_ros2_ws/
└── src/
    └── rscp_bridge_node/
        ├── package.xml
        ├── setup.py
        └── rscp_bridge_node/
            └── rscp_bridge_node.py
```

Register it as an entry point in `setup.py`:

```python
entry_points={
    'console_scripts': [
        'rscp_bridge_node = rscp_bridge_node.rscp_bridge_node:main',
    ],
},
```

### 4. Build the workspace

```bash
cd ~/your_ros2_ws
colcon build --packages-select rscp_bridge_node
source install/setup.bash
```

---

## Usage

### Connecting the hardware

Connect the RSCP serial link (e.g. USB-to-serial adapter) to your computer.
Identify the device name:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

If needed, grant your user permission to access the serial port:

```bash
sudo usermod -a -G dialout $USER
```

(Log out and back in for this to take effect.)

### Running the node

With default parameters (`/dev/ttyUSB0` at `115200` baud):

```bash
ros2 run rscp_bridge_node rscp_bridge_node
```

With custom parameters:

```bash
ros2 run rscp_bridge_node rscp_bridge_node --ros-args \
  -p port:=/dev/ttyACM0 \
  -p baudrate:=115200 \
  -p read_rate_hz:=50.0 \
  -p reconnect_period_sec:=2.0
```

### With a launch file

A launch file is provided at `launch/rscp_bridge.launch.py`, which exposes
the same parameters (plus `log_level`) as overridable launch arguments.

With default values:

```bash
ros2 launch rscp_bridge_node rscp_bridge.launch.py
```

With custom arguments:

```bash
ros2 launch rscp_bridge_node rscp_bridge.launch.py \
  port:=/dev/ttyACM0 \
  baudrate:=115200 \
  read_rate_hz:=50.0 \
  reconnect_period_sec:=2.0 \
  log_level:=debug
```

### Available parameters

| Parameter | Default | Description |
|---|---|---|
| `port` | `/dev/ttyUSB0` | Serial device path |
| `baudrate` | `115200` | Serial baud rate |
| `read_rate_hz` | `50.0` | Frequency of the non-blocking serial read timer |
| `serial_timeout` | `0.0` | `pyserial` read timeout (0 = non-blocking) |
| `reconnect_period_sec` | `2.0` | How often to retry opening the serial port if it's unavailable |

### Monitoring traffic

To see decoded commands arriving from the Host Module:

```bash
ros2 topic echo /rover_missions/recv_rscp
```

To send a status update to the Host Module manually (for testing):

```bash
ros2 topic pub --once /rover_missions/send_rscp std_msgs/String \
  "data: '{\"type\": \"acknowledge\"}'"
```

To see detailed byte-level logging, run the node with debug logging enabled:

```bash
ros2 run rscp_bridge_node rscp_bridge_node --ros-args --log-level debug
```

### Launch file example

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rscp_bridge_node',
            executable='rscp_bridge_node',
            name='rscp_bridge_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 115200,
                'read_rate_hz': 50.0,
            }]
        )
    ])
```

---

## Integrating with rover

This bridge expects other ROS2 nodes to:

1. **Subscribe** to `rover_missions/recv_rscp` and act on incoming commands
   (`set_stage`, `arm_disarm`, `search_area`, `navigate_to_gps`,
   `start_exploration`).
2. **Publish** to `rover_missions/send_rscp` with the appropriate JSON
   message once a command has been handled or a result is ready
   (`acknowledge`, `gps`, `task_completed`, `distance`, `message`).

A typical flow for a single command:

```
Host -> [bridge: decode] -> recv_rscp: {"type": "set_stage", "stage": 1}
                                    |
                                    v
                        Mission node receives, applies stage
                                    |
                                    v
send_rscp: {"type": "acknowledge"} -> [bridge: encode] -> Host
```

### Example worker node

Below is a minimal but complete worker node template. It:

1. Subscribes to `rover_missions/recv_rscp` and reacts to `search_area`
   commands (the same pattern applies to `navigate_to_gps`,
   `start_exploration`, `arm_disarm`, etc.).
2. Immediately publishes `{"type": "acknowledge"}` to confirm receipt.
3. "Performs" the search (replace the placeholder with your real
   navigation/search logic — this should NOT block; use a timer, action
   client, or background task instead of `time.sleep`).
4. Once finished, publishes the result as `{"type": "gps", "lat": ..., "lon": ...}`
   followed by `{"type": "task_completed"}`.

```python
#!/usr/bin/env python3
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SearchAreaWorker(Node):
    def __init__(self):
        super().__init__("search_area_worker")

        self._sub = self.create_subscription(
            String, "rover_missions/recv_rscp", self._on_command, 10
        )
        self._pub = self.create_publisher(
            String, "rover_missions/send_rscp", 10
        )

    def _on_command(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Ignoring non-JSON message: {msg.data!r}")
            return

        if data.get("type") != "search_area":
            return  # not for us

        lat = data["lat"]
        lon = data["lon"]
        rad = data["rad"]
        self.get_logger().info(
            f"Received search_area: center=({lat}, {lon}), radius={rad} m"
        )

        # 1. Acknowledge receipt immediately.
        self._publish({"type": "acknowledge"})

        # 2. Start the search. Replace this with a real, non-blocking
        #    navigation/search call (e.g. a Nav2 action goal). For this
        #    example we just simulate a result directly.
        found_lat, found_lon = lat + 0.0005, lon + 0.0005

        # 3. Report the result, then signal completion.
        self._publish({"type": "gps", "lat": found_lat, "lon": found_lon})
        self._publish({"type": "task_completed"})

    def _publish(self, payload: dict):
        out = String()
        out.data = json.dumps(payload)
        self._pub.publish(out)
        self.get_logger().info(f"ROS2 -> bridge: {out.data}")


def main(args=None):
    rclpy.init(args=args)
    node = SearchAreaWorker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

> **Tip:** every worker node should filter on `data.get("type")` and ignore
> messages it doesn't care about, since all worker nodes share the same
> `recv_rscp` topic. Likewise, multiple worker nodes can publish to
> `send_rscp` — the bridge forwards everything it receives to the Host in
> the order it arrives, so make sure your nodes publish `acknowledge` for a
> command before publishing results for a *later* command (see the note on
> `ArmDisarm` above about ack ordering).

---

## Testing without hardware

You can test the full bridge — both directions — without any physical RSCP
hardware using a **virtual serial port pair** and the included Host Module
simulator (`rscp_host_simulator.py`).

### 1. Create a virtual serial pair

Install `socat` and create two linked pseudo-terminals:

```bash
sudo apt install socat
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```

This prints two device paths, e.g.:

```
PTY is /dev/pts/3
PTY is /dev/pts/4
```

Leave this process running — anything written to one end appears on the
other, simulating the serial link between the bridge and the Host Module.

### 2. Launch the bridge on one end

```bash
ros2 launch rscp_bridge_node rscp_bridge.launch.py port:=/dev/pts/3 log_level:=debug
```

### 3. Run the host simulator on the other end

`rscp_host_simulator.py` plays the role of the Host Module: it sends a
scripted sequence of RSCP commands (`SetStage`, `ArmDisarm`, `SearchArea`,
`NavigateToGPS`, `StartExploration`) and prints any responses sent back by
the rover.

```bash
python3 rscp_host_simulator.py --port /dev/pts/4
```

### 4. Observe and interact via ROS2

In another terminal, watch decoded commands arrive:

```bash
ros2 topic echo /rover_missions/recv_rscp
```

Publish a response as if it came from a worker node, and watch it appear in
the simulator's output:

```bash
ros2 topic pub --once /rover_missions/send_rscp std_msgs/String \
  "data: '{\"type\": \"acknowledge\"}'"
```

```bash
ros2 topic pub --once /rover_missions/send_rscp std_msgs/String \
  "data: '{\"type\": \"gps\", \"lat\": 39.95, \"lon\": 32.85}'"
```

This exercises the entire bridge — serial framing, COBS encode/decode,
protobuf parsing, and ROS2 topic translation — in both directions, with no
hardware attached.

> `rscp_host_simulator.py` only depends on `pyserial`, `cobs`, and
> `rscp_protobuf` (the same dependencies as the bridge itself) and does not
> require ROS2 to run — it can be placed anywhere, e.g. in a `test/` or
> `scripts/` directory of this package.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `Could not open serial port` warning repeating | Wrong `port` parameter, device not connected, or permission denied (check `dialout` group membership) |
| No messages on `recv_rscp` | Check wiring/baud rate match the Host Module; enable `--log-level debug` to see raw frames |
| `COBS decode failed` warnings | Baud rate mismatch, electrical noise, or a non-RSCP device on that port |
| `Protobuf parse failed` warnings | `rscp_protobuf` version mismatch with the Host Module — reinstall the latest release |
| Outbound messages silently dropped | Serial port not connected (check for the "Dropping ... response" warning) |

---

## License

This bridge node is intended for use with the
[Anatolian Rover Challenge RSCP protocol](https://github.com/anatolianroverchallenge/rscp),
licensed under BSD-3-Clause.
