#!/usr/bin/env python3
"""RSCP Response Decoder

A small host-side utility to read RSCP *ResponseEnvelope* messages from a serial
port, using the same framing that the rover-side send script uses:

    SerializeToString() -> COBS encode -> append 0x00 byte delimiter -> write.

This script is based on (and fully compatible with) the example
`receive_commands.py` you provided, but adds:

* Command-line arguments for serial *port* and *baud*.
* Robust framing buffer (ignores empty frames).
* Optional --hex flag to print raw decoded bytes.
* Graceful error handling (decode / parse exceptions reported, frame skipped).
* Clean shutdown on Ctrl-C.

References:
- Your original send script demonstrates the encode+0x00 framing pattern. 
- Protobuf type and enum usage follow the RSCP definitions used there and in
  your `rscp.pb.h` header.

"""
import argparse
import sys
import serial
import cobs
import cobs.cobs  # we call cobs.cobs.decode / encode
import rscp_protobuf  # top-level import (as in your examples)

def decode_and_print(frame: bytes, show_hex: bool = False) -> None:
    """COBS-decode a frame and parse as ResponseEnvelope; print result."""
    if not frame:
        return
    try:
        decoded = cobs.cobs.decode(frame)
    except Exception as exc:
        print(f"[WARN] COBS decode error: {exc}; raw={frame.hex()}", file=sys.stderr)
        return

    if show_hex:
        print(f"Decoded bytes ({len(decoded)}): {decoded.hex()}")

    try:
        msg = rscp_protobuf.ResponseEnvelope()
        msg.ParseFromString(decoded)
        print("Received ResponseEnvelope:\n", msg)
    except Exception as exc:
        print(f"[WARN] Protobuf parse error: {exc}; decoded={decoded.hex()}", file=sys.stderr)

def read_stream(port: str, baud: int, show_hex: bool = False, timeout: float = 1.0) -> None:
    """Continuously read framed RSCP Response messages from serial."""
    with serial.Serial(port, baud, timeout=timeout) as ser:
        print(f"Listening on {port} @ {baud} baud. Ctrl-C to exit.")
        buf = bytearray()
        while True:
            try:
                b = ser.read(1)
                if not b:
                    continue
                if b == b"\x00":  # end-of-frame marker used by sender. 
                    decode_and_print(bytes(buf), show_hex=show_hex)
                    buf.clear()
                else:
                    buf += b
            except KeyboardInterrupt:
                print("\nExiting.")
                break

def main(argv=None):
    ap = argparse.ArgumentParser(description="Decode RSCP ResponseEnvelope frames from serial.")
    ap.add_argument("port", help="Serial device path (e.g., /dev/ttyUSB0 or /dev/pts/6)")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    ap.add_argument("--hex", action="store_true", help="Print decoded bytes in hex before parsing.")
    ap.add_argument("--timeout", type=float, default=1.0, help="Serial read timeout.")
    args = ap.parse_args(argv)
    read_stream(args.port, args.baud, show_hex=args.hex, timeout=args.timeout)

if __name__ == "__main__":
    main()
