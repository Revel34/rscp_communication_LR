#!/usr/bin/env python3
import sys
import argparse
import serial
from cobs import cobs
import rscp_protobuf

def parse_value(v):
    """Try int, float, bool, else leave string."""
    vl = v.lower()
    if vl in ("true","false"):
        return vl == "true"
    try:
        return int(v)
    except ValueError:
        pass
    try:
        return float(v)
    except ValueError:
        pass
    return v

def main():
    # discover your available request names:
    oneof = rscp_protobuf.RequestEnvelope.DESCRIPTOR.oneofs_by_name["request"]
    cmds = [f.name for f in oneof.fields]

    p = argparse.ArgumentParser(
        description="Send any RSCP RequestEnvelope over serial"
    )
    p.add_argument("-p", "--port", default="/dev/ttyUSB0",
                   help="serial port (e.g. /dev/pts/6 or COM3)")
    p.add_argument("-b", "--baud", type=int, default=9600,
                   help="baud rate (must match rover)")
    p.add_argument("cmd", choices=cmds,
                   help="which RSCP request to send")
    p.add_argument("fields", nargs="*",
                   help="optional key=value pairs for the request payload")
    args = p.parse_args()

    # build the envelope
    req = rscp_protobuf.RequestEnvelope()
    sub = getattr(req, args.cmd)  # e.g. req.arm_disarm, req.set_stage, etc.
    # Dynamically grab the sub‐message object...
    sub = getattr(req, args.cmd)     # e.g. req.start_exploration
    # ...and copy an empty instance of its exact class back into it.
    # This CopyFrom call is what actually marks the oneof as "present",
    # even if you don't set any fields.
    sub.CopyFrom(type(sub)())

    # apply any fields the user passed in
    for kv in args.fields:
        if "=" not in kv:
            p.error(f"Bad field syntax: {kv!r}, expected key=value")
        k, v = kv.split("=", 1)
        val = parse_value(v)
        setattr(sub, k, val)

    # serialize + COBS + 0x00 delimiter
    raw = req.SerializeToString()
    frame = cobs.encode(raw) + b"\x00"

    # send it
    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        ser.write(frame)

    print(f"→ Sent {args.cmd} with {args.fields} over {args.port}@{args.baud}")

if __name__ == "__main__":
    main()