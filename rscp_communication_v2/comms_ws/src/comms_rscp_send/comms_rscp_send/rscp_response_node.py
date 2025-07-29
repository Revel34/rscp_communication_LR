#!/usr/bin/env python3
"""RSCP Response Node (ROS 2)
--------------------------------

This node owns the serial connection to the RSCP *host* module and is used by
other ROS 2 nodes to **send RSCP `ResponseEnvelope` messages from the rover**.

### Key Features
* Configurable ROS 2 parameters: **`serial_port`** (default `/dev/ttyUSB0`) and
  **`baud`** (default `115200`).
* Accepts *send* requests via a topic: **`~/send_response`** (`std_msgs/String`
  containing a JSON command).
* Provides a *Python helper API* (`send_rscp_response_sync`) that other Python
  nodes can import & call directly (no JSON needed).
* Frames outbound bytes exactly like your example script: `SerializeToString()`
  → COBS encode → append `0x00` → write to serial.  See
  `send_rover_status_message.py` for the reference implementation.  fileciteturn0file0

### Supported message types
`ack`, `task_finished`, `gps`, `distance`, `text`, `rover_status`.

Field expectations for the JSON / helper call are documented with each builder
function below.  Message field layout follows the union in
`rscp.pb.h`'s `rscp_ResponseEnvelope` definition.  fileciteturn0file1

Example JSON publish (rover_status):
```json
{
  "type": "rover_status",
  "state": "DISARMED",
  "battery": {"voltage":12.3, "current":0.5, "soc":0.95},
  "coordinate": {"latitude":1.234,"longitude":5.678,"altitude":0.987},
  "heading": 0.0
}
```

Usage:
    ros2 run <your_pkg> rscp_response_node --ros-args -p serial_port:=/dev/ttyUSB1 -p baud:=57600
"""

from __future__ import annotations

import json
import threading
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String

# Serial / COBS ---------------------------------------------------------------
import serial
import cobs.cobs as cobs

# Import your protobuf module generated from rscp.proto -----------------------
try:
    import rscp_protobuf as rscp
except ImportError as exc:  # pragma: no cover - import guard
    raise ImportError(
        "Could not import 'rscp_protobuf'. Ensure your rscp protobuf Python module is on PYTHONPATH.\n"
        f"Original error: {exc}"
    ) from exc


# --------------------------------------------------------------------------- #
# Framing helper                                                              #
# --------------------------------------------------------------------------- #
def _serialize_and_frame(resp: 'rscp.ResponseEnvelope') -> bytes:
    """Serialize a ResponseEnvelope, COBS encode, append 0x00 delimiter."""
    raw = resp.SerializeToString()
    encoded = cobs.encode(raw)
    return encoded + b"\x00"


# --------------------------------------------------------------------------- #
# Builder helpers                                                             #
# --------------------------------------------------------------------------- #
def _build_ack(_: Dict[str, Any]) -> 'rscp.ResponseEnvelope':
    """Acknowledge message (no fields in nanopb version)."""
    resp = rscp.ResponseEnvelope()
    resp.acknowledge.SetInParent()
    return resp


def _build_task_finished(_: Dict[str, Any]) -> 'rscp.ResponseEnvelope':
    resp = rscp.ResponseEnvelope()
    resp.task_finished.SetInParent()
    return resp


def _build_gps(d: Dict[str, Any]) -> 'rscp.ResponseEnvelope':
    resp = rscp.ResponseEnvelope()
    gps = resp.gps_coordinate
    gps.latitude = float(d['latitude'])
    gps.longitude = float(d['longitude'])
    gps.altitude = float(d.get('altitude', 0.0))
    return resp


def _build_distance(d: Dict[str, Any]) -> 'rscp.ResponseEnvelope':
    resp = rscp.ResponseEnvelope()
    resp.distance = float(d['distance'])
    return resp


def _build_text(d: Dict[str, Any]) -> 'rscp.ResponseEnvelope':
    resp = rscp.ResponseEnvelope()
    resp.message = str(d['message'])
    return resp


def _build_rover_status(d: Dict[str, Any]) -> 'rscp.ResponseEnvelope':
    """RoverStatus builder.

    Expected keys in *d*:
        state            - string name of rscp_protobuf.RoverState enum (DISARMED, AUTONOMOUS, MANUAL) or int
        battery.voltage  - float
        battery.current  - float
        battery.soc      - float (0.0-1.0)
        coordinate.lat   - float
        coordinate.lon   - float
        coordinate.alt   - float (optional, default 0)
        heading          - float (optional)
    """
    resp = rscp.ResponseEnvelope()
    rs = resp.rover_status

    # state
    state_val = d.get('state', 'DISARMED')
    if isinstance(state_val, str):
        state_val = getattr(rscp.RoverState, state_val.upper(), rscp.RoverState.DISARMED)
    rs.state = state_val

    # battery
    bat = d.get('battery', {})
    rs.battery_state.voltage = float(bat.get('voltage', 0.0))
    rs.battery_state.current = float(bat.get('current', 0.0))
    rs.battery_state.state_of_charge = float(bat.get('soc', bat.get('state_of_charge', 0.0)))

    # coordinate
    coord = d.get('coordinate', d.get('gps', {}))
    rs.coordinate.latitude = float(coord.get('latitude', 0.0))
    rs.coordinate.longitude = float(coord.get('longitude', 0.0))
    rs.coordinate.altitude = float(coord.get('altitude', 0.0))

    # heading
    if 'heading' in d:
        rs.heading = float(d['heading'])

    return resp


_BUILDERS = {
    'ack': _build_ack,
    'acknowledge': _build_ack,
    'task_finished': _build_task_finished,
    'gps': _build_gps,
    'gps_coordinate': _build_gps,
    'distance': _build_distance,
    'text': _build_text,
    'message': _build_text,
    'rover_status': _build_rover_status,
}


def build_response_from_dict(d: Dict[str, Any]) -> 'rscp.ResponseEnvelope':
    """Dispatch builder based on d['type']."""
    try:
        t = d['type'].lower()
    except KeyError as exc:
        raise ValueError("JSON command must include a 'type' field") from exc
    try:
        builder = _BUILDERS[t]
    except KeyError as exc:  # pragma: no cover
        raise ValueError(f"Unknown response type '{t}'. Known: {list(_BUILDERS)}") from exc
    return builder(d)


# --------------------------------------------------------------------------- #
# Serial transport wrapper                                                    #
# --------------------------------------------------------------------------- #
class RscpSerialTransport:
    """Thread-safe wrapper around `serial.Serial`."""

    def __init__(self, port: str, baud: int, timeout: float = 0.1) -> None:
        self._port_name = port
        self._baud = baud
        self._timeout = timeout
        self._ser: serial.Serial | None = None
        self._lock = threading.Lock()
        self._open()

    # internal ---------------------------------------------------------------
    def _open(self) -> None:
        with self._lock:
            if self._ser and self._ser.is_open:
                if self._ser.port != self._port_name or self._ser.baudrate != self._baud:
                    self._ser.close()
                    self._ser = None
            if self._ser is None:
                self._ser = serial.Serial(self._port_name, self._baud, timeout=self._timeout)

    def reconfigure(self, port: str, baud: int) -> None:
        self._port_name = port
        self._baud = baud
        self._open()

    def write_frame(self, frame: bytes) -> int:
        with self._lock:
            if not self._ser:
                raise RuntimeError('Serial not open')
            return self._ser.write(frame)

    def close(self) -> None:
        with self._lock:
            if self._ser and self._ser.is_open:
                self._ser.close()


# --------------------------------------------------------------------------- #
# ROS 2 node                                                                  #
# --------------------------------------------------------------------------- #
class RscpResponseNode(Node):
    """Send RSCP ResponseEnvelope messages over serial on request."""

    def __init__(self) -> None:
        super().__init__('rscp_response_node')
        self._cbg = ReentrantCallbackGroup()

        # Parameters ---------------------------------------------------------
        self.declare_parameter('serial_port', '/dev/ttyUSB3')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud').value
        self._transport = RscpSerialTransport(port, baud)

        # react to param changes
        self.add_on_set_parameters_callback(self._on_param_change)

        # Subscriber ---------------------------------------------------------
        self._cmd_sub = self.create_subscription(
            String,
            'send_response',
            self._on_cmd_msg,
            10,
            callback_group=self._cbg,
        )

        self.get_logger().info(f'RSCP Response Node using port={port} baud={baud}')

    # ROS parameter callback -------------------------------------------------
    def _on_param_change(self, params: list[Parameter]):
        port = None
        baud = None
        for p in params:
            if p.name == 'serial_port' and p.type_ == Parameter.Type.STRING:
                port = p.value
            elif p.name == 'baud' and p.type_ == Parameter.Type.INTEGER:
                baud = p.value
        if port is not None or baud is not None:
            self.get_logger().info('Reconfiguring serial (parameter change).')
            self._transport.reconfigure(port or self._transport._port_name,
                                        baud or self._transport._baud)
        return rclpy.parameter.SetParametersResult(successful=True)

    # Subscriber callback ----------------------------------------------------
    def _on_cmd_msg(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().error(f'Invalid JSON in send_response message: {exc}')
            return
        try:
            resp = build_response_from_dict(data)
        except Exception as exc:
            self.get_logger().error(f'Failed to build ResponseEnvelope: {exc}')
            return
        frame = _serialize_and_frame(resp)
        try:
            self._transport.write_frame(frame)
            self.get_logger().info(f'Sent RSCP response type={data.get("type")}, {len(frame)} bytes.')
        except Exception as exc:
            self.get_logger().error(f'Error writing to serial: {exc}')

    # Public API -------------------------------------------------------------
    def send_response(self, response_type: str, **fields: Any) -> bool:
        """Send a response *programmatically* (no JSON parsing).

        Example:
            node.send_response('gps', latitude=1.0, longitude=2.0, altitude=3.0)

        Returns True on local publish/write success (exceptions logged & False).
        """
        try:
            resp = _BUILDERS[response_type](fields | {'type': response_type})  # type: ignore[arg-type]
        except Exception:
            # fallback to full dict path
            try:
                resp = build_response_from_dict({'type': response_type, **fields})
            except Exception as exc:
                self.get_logger().error(f'Failed to build response: {exc}')
                return False
        frame = _serialize_and_frame(resp)
        try:
            self._transport.write_frame(frame)
            self.get_logger().debug(f'Sent RSCP response via API type={response_type}.')
            return True
        except Exception as exc:
            self.get_logger().error(f'Error writing serial: {exc}')
            return False


# --------------------------------------------------------------------------- #
# Convenience function for external Python scripts                            #
# --------------------------------------------------------------------------- #
def send_rscp_response_sync(node: Node, response_type: str, **fields: Any) -> bool:
    """Send a response through an existing *RscpResponseNode*.

    The helper looks up (or creates) a hidden client object in the given node's
    context to publish on the RscpResponseNode topic.  This avoids having to
    import/construct the RscpResponseNode in every script (fire-and-forget).
    Returns True if publish was attempted.
    """
    topic_name = '/rscp_response_node/send_response'
    pub = node.create_publisher(String, topic_name, 10)
    d = dict(type=response_type, **fields)
    msg = String()
    msg.data = json.dumps(d)
    pub.publish(msg)
    node.get_logger().debug(f'Published RSCP response request to {topic_name}.')
    return True


# --------------------------------------------------------------------------- #
# main                                                                        #
# --------------------------------------------------------------------------- #
def main(args=None) -> None:
    rclpy.init(args=args)
    node = RscpResponseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
