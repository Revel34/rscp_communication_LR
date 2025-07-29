#!/usr/bin/env python3
import sys

# Adjust this if your generated module is named differently or lives elsewhere
try:
    import rscp_protobuf
except ImportError:
    print("ERROR: Cannot import rscp_protobuf – check your PYTHONPATH", file=sys.stderr)
    sys.exit(1)

def main():
    desc = rscp_protobuf.RequestEnvelope.DESCRIPTOR

    # Make sure the 'request' oneof exists...
    if 'request' not in desc.oneofs_by_name:
        print("❗ No 'request' oneof found on RequestEnvelope")
        return

    print("Fields in 'request' oneof:")
    for f in desc.oneofs_by_name['request'].fields:
        # f.name is the oneof field name, f.number is its tag
        print(f" • {f.name}   (field #{f.number})")

if __name__ == "__main__":
    main()