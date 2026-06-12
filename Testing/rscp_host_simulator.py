#!/usr/bin/env python3
"""
rscp_host_simulator.py
======================

A standalone test tool that imitates the RSCP **Host Module** so you can test
the rscp_bridge_node without any physical hardware.

It connects to one end of a virtual serial pair (created with socat), sends
RSCP RequestEnvelope commands to the bridge, and prints any ResponseEnvelope
messages the rover side sends back.

------------------------------------------------------------------------------
Typical workflow (no hardware required)
------------------------------------------------------------------------------
1. Create a virtual serial pair:
       socat -d -d pty,raw,echo=0 pty,raw,echo=0
   Note the two printed paths, e.g. /dev/pts/3 and /dev/pts/4.

2. Launch the bridge on one end:
       ros2 launch rscp_bridge_node rscp_bridge.launch.py port:=/dev/pts/3 log_level:=debug

3. Run this simulator on the OTHER end:
       python3 rscp_host_simulator.py --port /dev/pts/4

4. (Optional) Watch the decoded commands arrive on the ROS2 side:
       ros2 topic echo /rover_missions/recv_rscp

5. (Optional) Send a response back from the ROS2 side and watch it print here:
       ros2 topic pub --once /rover_missions/send_rscp std_msgs/String \\
           "data: '{\\"type\\": \\"acknowledge\\"}'"

Dependencies:  pyserial, cobs, rscp_protobuf  (same as the bridge node)
"""

import argparse
import threading
import time

import serial
import cobs.cobs as cobs_codec
import rscp_protobuf


FRAME_DELIMITER = b"\x00"


# ---------------------------------------------------------------------------
# Encoding helper: RequestEnvelope -> COBS frame
# ---------------------------------------------------------------------------
def frame(request) -> bytes:
    return cobs_codec.encode(request.SerializeToString()) + FRAME_DELIMITER


def build_set_stage(stage: int):
    req = rscp_protobuf.RequestEnvelope()
    req.set_stage.value = stage
    return req


def build_arm_disarm(arm: bool):
    req = rscp_protobuf.RequestEnvelope()
    req.arm_disarm.value = arm
    return req


def build_search_area(lat: float, lon: float, rad: float):
    req = rscp_protobuf.RequestEnvelope()
    req.search_area.center_coordinate.latitude = lat
    req.search_area.center_coordinate.longitude = lon
    req.search_area.radius = rad
    return req


def build_navigate_to_gps(lat: float, lon: float):
    req = rscp_protobuf.RequestEnvelope()
    req.navigate_to_gps.coordinate.latitude = lat
    req.navigate_to_gps.coordinate.longitude = lon
    return req


def build_start_exploration():
    req = rscp_protobuf.RequestEnvelope()
    req.start_exploration.SetInParent()
    return req


# ---------------------------------------------------------------------------
# Reader thread: prints any ResponseEnvelope the rover sends back
# ---------------------------------------------------------------------------
def reader_loop(ser: serial.Serial, stop: threading.Event):
    buffer = bytearray()
    while not stop.is_set():
        try:
            n = ser.in_waiting
            chunk = ser.read(n) if n else b""
        except (serial.SerialException, OSError):
            break
        if not chunk:
            time.sleep(0.01)
            continue
        buffer.extend(chunk)
        while True:
            idx = buffer.find(0x00)
            if idx == -1:
                break
            raw = bytes(buffer[:idx])
            del buffer[: idx + 1]
            if not raw:
                continue
            try:
                decoded = cobs_codec.decode(raw)
                resp = rscp_protobuf.ResponseEnvelope()
                resp.ParseFromString(decoded)
            except Exception as exc:  # noqa: BLE001
                print(f"  [<- rover] (undecodable frame: {exc})")
                continue
            kind = resp.WhichOneof("response")
            print(f"  [<- rover] {kind}: {str(resp).strip() or '(empty)'}")


# ---------------------------------------------------------------------------
# Main: send a scripted sequence of commands
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="RSCP Host Module simulator")
    parser.add_argument("--port", required=True,
                        help="Serial device (the OTHER end of the socat pair)")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--delay", type=float, default=2.0,
                        help="Seconds to wait between commands")
    args = parser.parse_args()

    with serial.Serial(args.port, args.baudrate, timeout=0) as ser:
        stop = threading.Event()
        reader = threading.Thread(target=reader_loop, args=(ser, stop), daemon=True)
        reader.start()

        # A scripted Stage-1-style sequence. Edit freely to test other flows.
        sequence = [
            ("SetStage(1)",            build_set_stage(1)),
            ("ArmDisarm(arm=True)",    build_arm_disarm(True)),
            ("SearchArea(39.9,32.8,25)", build_search_area(39.9, 32.8, 25.0)),
            ("NavigateToGPS(39.95,32.85)", build_navigate_to_gps(39.95, 32.85)),
            ("StartExploration",       build_start_exploration()),
            ("ArmDisarm(arm=False)",   build_arm_disarm(False)),
        ]

        print(f"Connected on {args.port}. Sending {len(sequence)} commands "
              f"({args.delay}s apart). Watch the bridge logs and the ROS2 topic.\n")

        for label, req in sequence:
            wire = frame(req)
            ser.write(wire)
            ser.flush()
            print(f"[-> rover] {label}  ({len(wire)} B: {wire.hex()})")
            time.sleep(args.delay)

        print("\nAll commands sent. Listening for responses for 5 more seconds...")
        time.sleep(5.0)
        stop.set()
        reader.join(timeout=1.0)
        print("Done.")


if __name__ == "__main__":
    main()
