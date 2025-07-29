#!/usr/bin/env python3
import serial, sys
from cobs import cobs
import rscp_protobuf

ser = serial.Serial("/dev/pts/3", 9600, timeout=5)
buf = bytearray()
print("Listening for one response…")

while True:
    b = ser.read(1)
    if not b:
        print("Timeout waiting for response")
        sys.exit(1)
    if b == b"\x00":
        frame = bytes(buf)
        try:
            decoded = cobs.decode(frame)
            resp = rscp_protobuf.ResponseEnvelope()
            resp.ParseFromString(decoded)
            print("Response:", resp.WhichOneof("response"))
            print(resp)
        except Exception as e:
            print("Failed to decode/parse:", e)
        break
    buf += b