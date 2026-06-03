#!/usr/bin/env python3
"""Write a simple AD calibration block for testing converted stream output."""

import argparse
import socket
import zlib


SOF = b"\x12\x34"
EOF = b"\x56\x78"

CMD_WRITE_PARAM = 0x01
BLOCK_CAL_DATA = 0x0001

CHANNEL_COUNT = 12
DEFAULT_K_RAW = 100_000_000
DEFAULT_B_RAW = 0


def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    head = bytes(
        [
            SOF[0],
            SOF[1],
            cmd & 0xFF,
            (len(payload) >> 8) & 0xFF,
            len(payload) & 0xFF,
        ]
    )
    body = head + payload
    crc = zlib.crc32(body) & 0xFFFFFFFF
    return body + crc.to_bytes(4, "big") + EOF


def build_cal_payload(test_channel: int, k_raw: int, b_raw: int) -> bytes:
    payload = bytearray()

    payload.extend(BLOCK_CAL_DATA.to_bytes(2, "big"))

    for ch in range(CHANNEL_COUNT):
        if ch == test_channel:
            payload.extend(k_raw.to_bytes(4, "big", signed=True))
            payload.extend(b_raw.to_bytes(4, "big", signed=True))
        else:
            payload.extend(DEFAULT_K_RAW.to_bytes(4, "big", signed=True))
            payload.extend(DEFAULT_B_RAW.to_bytes(4, "big", signed=True))

    return bytes(payload)


def parse_write_status(data: bytes):
    if len(data) < 12:
        return None

    if data[0:2] != SOF or data[-2:] != EOF:
        return None

    payload_len = (data[3] << 8) | data[4]
    if data[2] != CMD_WRITE_PARAM or payload_len < 3:
        return None

    payload = data[5 : 5 + payload_len]
    block_id = (payload[0] << 8) | payload[1]
    status = payload[2]
    return block_id, status


def main():
    parser = argparse.ArgumentParser(description="Write test AD calibration parameters")
    parser.add_argument("--host", default="192.168.1.21", help="device IP")
    parser.add_argument("--port", type=int, default=8080, help="device TCP port")
    parser.add_argument("--bind", default=None, help="optional local source IP")
    parser.add_argument("--channel", type=int, default=1, help="1-based channel number")
    parser.add_argument("--k-raw", type=int, default=200_000_000, help="raw k value, 1e8 means 1.0")
    parser.add_argument("--b-raw", type=int, default=0, help="raw b value")
    args = parser.parse_args()

    if args.channel < 1 or args.channel > CHANNEL_COUNT:
        raise SystemExit("channel must be 1..12")

    payload = build_cal_payload(args.channel - 1, args.k_raw, args.b_raw)
    frame = build_frame(CMD_WRITE_PARAM, payload)

    print(
        f"write calibration: CH{args.channel} k_raw={args.k_raw} "
        f"b_raw={args.b_raw}, payload={len(payload)} bytes"
    )

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3.0)

    if args.bind:
        sock.bind((args.bind, 0))

    try:
        sock.connect((args.host, args.port))
        sock.sendall(frame)
        reply = sock.recv(1024)
    finally:
        sock.close()

    status = parse_write_status(reply)
    if status is None:
        print("write status: no valid reply")
        return 1

    block_id, status_code = status
    print(f"write status: block=0x{block_id:04X}, status=0x{status_code:02X}")
    return 0 if status_code == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
