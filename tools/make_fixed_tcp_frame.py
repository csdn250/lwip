#!/usr/bin/env python3
"""Build one fixed-length ADDA TCP command frame for NetAssist HEX testing.

The fixed command frame is 150 bytes:
SOF(2) + CMD(1) + LEN(2) + BLOCK_ID(2) + DATA/PAD(137) + CRC32(4) + EOF(2)

LEN is the DATA length only. It does not include BLOCK_ID.

Examples:
    python tools/make_fixed_tcp_frame.py 07 00 00
    python tools/make_fixed_tcp_frame.py 02 00 05
    python tools/make_fixed_tcp_frame.py 01 00 02 01 81 00 00 00 00 00 00
"""

import argparse
import zlib


SOF = (0x12, 0x34)
EOF = (0x56, 0x78)

FIXED_FRAME_SIZE = 150
FIXED_CRC_OFFSET = 144
FIXED_EOF_OFFSET = 148
FIXED_DATA_OFFSET = 5
FIXED_BLOCK_ID_SIZE = 2
FIXED_DATA_CAPACITY = FIXED_CRC_OFFSET - FIXED_DATA_OFFSET - FIXED_BLOCK_ID_SIZE


def parse_hex_byte(text: str) -> int:
    value = int(text, 16)
    if value < 0 or value > 0xFF:
        raise ValueError(f"not a byte: {text}")
    return value


def build_fixed_frame(cmd: int, block_id: int, data: bytes) -> bytes:
    if len(data) > FIXED_DATA_CAPACITY:
        raise ValueError(f"data too long: {len(data)} > {FIXED_DATA_CAPACITY}")

    frame = bytearray(FIXED_FRAME_SIZE)
    frame[0] = SOF[0]
    frame[1] = SOF[1]
    frame[2] = cmd & 0xFF
    frame[3] = (len(data) >> 8) & 0xFF
    frame[4] = len(data) & 0xFF
    frame[5] = (block_id >> 8) & 0xFF
    frame[6] = block_id & 0xFF
    frame[7 : 7 + len(data)] = data

    crc = zlib.crc32(bytes(frame[:FIXED_CRC_OFFSET])) & 0xFFFFFFFF
    frame[FIXED_CRC_OFFSET : FIXED_CRC_OFFSET + 4] = crc.to_bytes(4, "big")
    frame[FIXED_EOF_OFFSET] = EOF[0]
    frame[FIXED_EOF_OFFSET + 1] = EOF[1]
    return bytes(frame)


def format_hex(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def main() -> int:
    parser = argparse.ArgumentParser(description="Build a fixed 150-byte ADDA TCP frame")
    parser.add_argument("cmd", help="command byte, hex")
    parser.add_argument("block_hi", help="block id high byte, hex")
    parser.add_argument("block_lo", help="block id low byte, hex")
    parser.add_argument("data", nargs="*", help="data bytes, hex")
    args = parser.parse_args()

    try:
        cmd = parse_hex_byte(args.cmd)
        block_id = (parse_hex_byte(args.block_hi) << 8) | parse_hex_byte(args.block_lo)
        data = bytes(parse_hex_byte(item) for item in args.data)
        frame = build_fixed_frame(cmd, block_id, data)
    except ValueError as exc:
        print(f"error: {exc}")
        return 2

    print(f"len={len(frame)} data_len={len(data)} block=0x{block_id:04X}")
    print(format_hex(frame))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
