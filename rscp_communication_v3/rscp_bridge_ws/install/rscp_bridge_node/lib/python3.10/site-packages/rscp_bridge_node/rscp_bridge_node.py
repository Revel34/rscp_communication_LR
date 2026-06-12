#!/usr/bin/env python3
"""
rscp_bridge_node.py
===================

A robust ROS2 (rclpy) bridge between a rover's internal ROS2 network and an
external Host Module that speaks the Rover Satellite Communications Protocol
(RSCP) used in the Anatolian Rover Challenge.

This node is a *pure translator*. It does NOT drive motors, navigate, or block
on physical tasks. It only:

  * reads COBS-framed protobuf ``RequestEnvelope`` messages off the serial port,
    decodes them, and re-publishes them as JSON on a ROS2 topic, and
  * subscribes to a ROS2 topic of JSON status updates, packs them into a
    protobuf ``ResponseEnvelope``, COBS-encodes them, and writes them to serial.

The rest of the rover (state machine, Nav2, arm controller, science nodes) lives
in separate nodes and talks to this bridge only over the two ROS2 topics below.

------------------------------------------------------------------------------
ROS2 interface
------------------------------------------------------------------------------
Publisher   : ``rover_missions/recv_rscp``  (std_msgs/String, JSON)   <- from Host
Subscriber  : ``rover_missions/send_rscp``  (std_msgs/String, JSON)   -> to Host

Inbound JSON (Host -> ROS2), one of:
  {"type": "set_stage",        "stage": <int>}
  {"type": "arm_disarm",       "arm":   <bool>}
  {"type": "search_area",      "lat": <float>, "lon": <float>, "rad": <float>}
  {"type": "navigate_to_gps",  "lat": <float>, "lon": <float>}
  {"type": "start_exploration"}

Outbound JSON (ROS2 -> Host), one of:
  {"type": "acknowledge"}
  {"type": "gps",            "lat": <float>, "lon": <float> [, "alt": <float>]}
  {"type": "task_completed"}            # mapped to protobuf field `task_finished`
  {"type": "distance",       "distance": <float>}
  {"type": "message",        "message": <str>}        # bonus: string passthrough

------------------------------------------------------------------------------
Wire format (verified against proto/rscp.proto + the package's own examples)
------------------------------------------------------------------------------
  encode:  ResponseEnvelope.SerializeToString() -> cobs.cobs.encode(...) -> + b"\\x00"
  decode:  split stream on b"\\x00" -> cobs.cobs.decode(frame) -> RequestEnvelope.ParseFromString(...)

NOTE on names: the installed package exposes its classes at the top level
(``rscp_protobuf.RequestEnvelope``), and COBS lives at ``cobs.cobs``. Empty
protobuf messages used inside a ``oneof`` (Acknowledge, TaskFinished) must be
activated with ``.SetInParent()`` so ``WhichOneof`` reports them as set.

Dependencies (pip):  pyserial, cobs, rscp_protobuf
    python3 -m pip install pyserial cobs \\
        https://github.com/anatolianroverchallenge/rscp/releases/latest/download/rscp_protobuf.zip
"""

import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import cobs.cobs as cobs_codec
import rscp_protobuf


# Frame delimiter. COBS guarantees a zero-free payload, so 0x00 is a safe marker.
FRAME_DELIMITER = 0x00

# Hard cap on the inbound reassembly buffer. If we somehow accumulate this many
# bytes without seeing a delimiter the stream is almost certainly corrupt, so we
# drop the buffer rather than grow without bound.
MAX_BUFFER_BYTES = 8192


class RscpBridgeNode(Node):
    """Translator node between the RSCP serial link and the ROS2 network."""

    def __init__(self):
        super().__init__("rscp_bridge_node")

        # ---- Parameters -----------------------------------------------------
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("read_rate_hz", 50.0)
        self.declare_parameter("serial_timeout", 0.0)  # non-blocking reads
        self.declare_parameter("reconnect_period_sec", 2.0)

        self._port = self.get_parameter("port").value
        self._baudrate = int(self.get_parameter("baudrate").value)
        self._read_rate_hz = float(self.get_parameter("read_rate_hz").value)
        self._serial_timeout = float(self.get_parameter("serial_timeout").value)
        self._reconnect_period = float(self.get_parameter("reconnect_period_sec").value)

        # ---- State ----------------------------------------------------------
        self._ser = None
        self._rx_buffer = bytearray()
        self._serial_lock = threading.Lock()  # guards all reads/writes to _ser
        self._sec_since_reconnect_try = 0.0

        # ---- ROS2 interface -------------------------------------------------
        # Inbound (Host -> ROS2). Keep a small depth; commands are infrequent.
        self._recv_pub = self.create_publisher(String, "rover_missions/recv_rscp", 10)
        # Outbound (ROS2 -> Host). Larger depth so bursts of telemetry are not lost.
        self._send_sub = self.create_subscription(
            String, "rover_missions/send_rscp", self._on_send, 50
        )

        # ---- Serial + read timer -------------------------------------------
        self._connect_serial()
        period = 1.0 / self._read_rate_hz if self._read_rate_hz > 0 else 0.02
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"RSCP bridge started. port={self._port} baud={self._baudrate} "
            f"read_rate={self._read_rate_hz}Hz"
        )
        self.get_logger().info(
            "Listening Host->ROS2 on 'rover_missions/recv_rscp', "
            "sending ROS2->Host from 'rover_missions/send_rscp'."
        )

    # ========================================================================
    # Serial connection management
    # ========================================================================
    def _connect_serial(self) -> bool:
        """(Re)open the serial port. Returns True on success. Never raises."""
        with self._serial_lock:
            if self._ser is not None and self._ser.is_open:
                return True
            try:
                self._ser = serial.Serial(
                    port=self._port,
                    baudrate=self._baudrate,
                    timeout=self._serial_timeout,
                    write_timeout=1.0,
                )
                self._rx_buffer.clear()
                self.get_logger().info(f"Serial port {self._port} opened.")
                return True
            except (serial.SerialException, OSError, ValueError) as exc:
                self._ser = None
                self.get_logger().warn(
                    f"Could not open serial port {self._port}: {exc}. "
                    f"Retrying every {self._reconnect_period:.1f}s."
                )
                return False

    def _serial_ok(self) -> bool:
        return self._ser is not None and self._ser.is_open

    # ========================================================================
    # Inbound:  Serial (Host) -> ROS2
    # ========================================================================
    def _on_timer(self):
        """High-frequency, non-blocking serial pump driven by a ROS2 timer."""
        if not self._serial_ok():
            # Throttled reconnect attempts so we don't spin the OS open() call.
            self._sec_since_reconnect_try += (
                1.0 / self._read_rate_hz if self._read_rate_hz > 0 else 0.02
            )
            if self._sec_since_reconnect_try >= self._reconnect_period:
                self._sec_since_reconnect_try = 0.0
                self._connect_serial()
            return

        # Read everything currently available without blocking the executor.
        try:
            with self._serial_lock:
                n = self._ser.in_waiting
                chunk = self._ser.read(n) if n else b""
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f"Serial read failed: {exc}. Closing port.")
            self._close_serial()
            return

        if chunk:
            self._rx_buffer.extend(chunk)
            self._drain_buffer()

        # Corruption guard: a buffer this large with no delimiter is junk.
        if len(self._rx_buffer) > MAX_BUFFER_BYTES:
            self.get_logger().warn(
                f"RX buffer exceeded {MAX_BUFFER_BYTES} bytes with no frame "
                f"delimiter; discarding to resynchronise."
            )
            self._rx_buffer.clear()

    def _drain_buffer(self):
        """Split the reassembly buffer on 0x00 and process each complete frame."""
        while True:
            idx = self._rx_buffer.find(FRAME_DELIMITER)
            if idx == -1:
                break  # no complete frame yet; keep partial bytes for next tick
            frame = bytes(self._rx_buffer[:idx])
            del self._rx_buffer[: idx + 1]  # drop frame + delimiter
            if frame:  # skip empty frames (e.g. back-to-back 0x00)
                self._process_frame(frame)

    def _process_frame(self, frame: bytes):
        """COBS-decode one frame, parse as RequestEnvelope, publish as JSON."""
        self.get_logger().debug(f"RX frame ({len(frame)} B): {frame.hex()}")
        try:
            decoded = cobs_codec.decode(frame)
        except cobs_codec.DecodeError as exc:
            self.get_logger().warn(f"COBS decode failed, dropping frame: {exc}")
            return

        request = rscp_protobuf.RequestEnvelope()
        try:
            request.ParseFromString(decoded)
        except Exception as exc:  # protobuf DecodeError and friends
            self.get_logger().warn(f"Protobuf parse failed, dropping frame: {exc}")
            return

        payload = self._request_to_dict(request)
        if payload is None:
            kind = request.WhichOneof("request")
            self.get_logger().warn(f"Unhandled or empty request type: {kind!r}")
            return

        out = String()
        out.data = json.dumps(payload)
        self._recv_pub.publish(out)
        self.get_logger().info(f"Host -> ROS2: {out.data}")

    @staticmethod
    def _request_to_dict(request) -> dict | None:
        """Map a parsed RequestEnvelope oneof into the ROS-side JSON dict."""
        kind = request.WhichOneof("request")
        if kind == "set_stage":
            return {"type": "set_stage", "stage": int(request.set_stage.value)}
        if kind == "arm_disarm":
            return {"type": "arm_disarm", "arm": bool(request.arm_disarm.value)}
        if kind == "search_area":
            sa = request.search_area
            return {
                "type": "search_area",
                "lat": float(sa.center_coordinate.latitude),
                "lon": float(sa.center_coordinate.longitude),
                "rad": float(sa.radius),
            }
        if kind == "navigate_to_gps":
            c = request.navigate_to_gps.coordinate
            return {
                "type": "navigate_to_gps",
                "lat": float(c.latitude),
                "lon": float(c.longitude),
            }
        if kind == "start_exploration":
            return {"type": "start_exploration"}
        return None

    # ========================================================================
    # Outbound:  ROS2 -> Serial (Host)
    # ========================================================================
    def _on_send(self, msg: String):
        """Subscription callback: JSON status -> ResponseEnvelope -> serial."""
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as exc:
            self.get_logger().error(f"Outbound message is not valid JSON: {exc}")
            return
        if not isinstance(data, dict):
            self.get_logger().error("Outbound JSON must be an object/dict.")
            return

        response = self._dict_to_response(data)
        if response is None:
            self.get_logger().warn(f"Unhandled outbound message: {data!r}")
            return

        self._write_response(response, data.get("type"))

    def _dict_to_response(self, data: dict):
        """Build a ResponseEnvelope from the ROS-side JSON dict, or None."""
        mtype = data.get("type")
        response = rscp_protobuf.ResponseEnvelope()
        try:
            if mtype == "acknowledge":
                response.acknowledge.SetInParent()
            elif mtype in ("task_completed", "task_finished"):
                # Spec uses "task_completed"; the proto field is `task_finished`.
                response.task_finished.SetInParent()
            elif mtype == "gps":
                response.gps_coordinate.latitude = float(data["lat"])
                response.gps_coordinate.longitude = float(data["lon"])
                if "alt" in data and data["alt"] is not None:
                    response.gps_coordinate.altitude = float(data["alt"])
            elif mtype == "distance":
                response.distance = float(data["distance"])
            elif mtype == "message":
                response.message = str(data["message"])
            else:
                return None
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(
                f"Malformed '{mtype}' message {data!r}: {exc}"
            )
            return None
        return response

    def _write_response(self, response, mtype):
        """Serialize + COBS-encode + delimit + write, guarded and non-fatal."""
        try:
            encoded = cobs_codec.encode(response.SerializeToString())
            wire = encoded + bytes([FRAME_DELIMITER])
        except Exception as exc:
            self.get_logger().error(f"Failed to encode '{mtype}' response: {exc}")
            return

        if not self._serial_ok():
            self.get_logger().warn(
                f"Dropping '{mtype}' response: serial port not connected."
            )
            return

        try:
            with self._serial_lock:
                self._ser.write(wire)
                self._ser.flush()
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f"Serial write failed: {exc}. Closing port.")
            self._close_serial()
            return

        self.get_logger().info(f"ROS2 -> Host: type={mtype}")
        self.get_logger().debug(f"TX frame ({len(wire)} B): {wire.hex()}")

    # ========================================================================
    # Teardown
    # ========================================================================
    def _close_serial(self):
        with self._serial_lock:
            try:
                if self._ser is not None and self._ser.is_open:
                    self._ser.close()
            except (serial.SerialException, OSError):
                pass
            finally:
                self._ser = None
                self._rx_buffer.clear()

    def destroy_node(self):
        self.get_logger().info("Shutting down RSCP bridge; closing serial port.")
        self._close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RscpBridgeNode()
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
