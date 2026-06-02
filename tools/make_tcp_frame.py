#!/usr/bin/env python3
"""Build one ADDA TCP protocol frame for NetAssist HEX testing.

Usage:
    python tools/make_tcp_frame.py 02 00 04
    python tools/make_tcp_frame.py 01 00 05 02 00 00 00 00 22

The first byte is cmd. Remaining bytes are payload.
"""

import sys
import zlib


SOF = (0x12, 0x34)
EOF = (0x56, 0x78)


def parse_hex_byte(text):
    value = int(text, 16)
    if value < 0 or value > 0xFF:
        raise ValueError(f"not a byte: {text}")
    return value


def build_frame(cmd, payload):
    payload_len = len(payload)
    frame = [
        SOF[0],
        SOF[1],
        cmd,
        (payload_len >> 8) & 0xFF,
        payload_len & 0xFF,
    ]
    frame.extend(payload)

    crc = zlib.crc32(bytes(frame)) & 0xFFFFFFFF
    frame.append((crc >> 24) & 0xFF)
    frame.append((crc >> 16) & 0xFF)
    frame.append((crc >> 8) & 0xFF)
    frame.append(crc & 0xFF)
    frame.extend(EOF)

    return frame


def format_hex(data):
    return " ".join(f"{byte:02X}" for byte in data)


def main(argv):
    if len(argv) < 2:
        print("Usage: python tools/make_tcp_frame.py <cmd_hex> [payload_hex ...]")
        print("Example: python tools/make_tcp_frame.py 02 00 04")
        return 1

    try:
        values = [parse_hex_byte(arg) for arg in argv[1:]]
    except ValueError as exc:
        print(f"error: {exc}")
        return 2

    frame = build_frame(values[0], values[1:])
    print(format_hex(frame))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
