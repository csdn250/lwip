#!/usr/bin/env python3
"""Read and decode the ADDA heartbeat status frame."""

import argparse
import socket
import struct
import zlib


SOF = b"\x12\x34"
EOF = b"\x56\x78"

CMD_HEARTBEAT = 0x07

FIXED_FRAME_LEN = 150
FIXED_CRC_OFFSET = 144
FIXED_EOF_OFFSET = 148
FIXED_DATA_OFFSET = 5
FIXED_BLOCK_ID_SIZE = 2
FIXED_DATA_CAPACITY = FIXED_CRC_OFFSET - FIXED_DATA_OFFSET - FIXED_BLOCK_ID_SIZE

HEARTBEAT_STATUS_LEN = 69


STATE_BITS = {
    0: "TCP_CONNECTED",
    1: "ADC_STREAMING",
    2: "DAC_CASCADE",
    3: "EEPROM_READY",
    4: "ADC_DMA_ACTIVE",
    5: "DAC_MANUAL",
}

ALARM_BITS = {
    0: "EEPROM_NOT_READY",
    1: "ADC_DMA_STOPPED",
    2: "TCP_SEND_FAIL",
}


def crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def build_fixed_frame(cmd: int, block_id: int, data: bytes = b"") -> bytes:
    if len(data) > FIXED_DATA_CAPACITY:
        raise ValueError(f"data too long: {len(data)}")

    frame = bytearray(FIXED_FRAME_LEN)
    frame[0:2] = SOF
    frame[2] = cmd & 0xFF
    frame[3:5] = len(data).to_bytes(2, "big")
    frame[5:7] = block_id.to_bytes(2, "big")
    frame[7 : 7 + len(data)] = data

    crc = crc32(bytes(frame[:FIXED_CRC_OFFSET]))
    frame[FIXED_CRC_OFFSET : FIXED_CRC_OFFSET + 4] = crc.to_bytes(4, "big")
    frame[FIXED_EOF_OFFSET : FIXED_EOF_OFFSET + 2] = EOF
    return bytes(frame)


def recv_exact(sock: socket.socket, size: int) -> bytes:
    data = bytearray()

    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError("connection closed before full frame was received")
        data.extend(chunk)

    return bytes(data)


def parse_fixed_reply(frame: bytes):
    if len(frame) != FIXED_FRAME_LEN:
        raise ValueError(f"bad frame length: {len(frame)}")

    if frame[0:2] != SOF:
        raise ValueError("bad frame header")

    if frame[FIXED_EOF_OFFSET : FIXED_EOF_OFFSET + 2] != EOF:
        raise ValueError("bad frame tail")

    crc_recv = int.from_bytes(frame[FIXED_CRC_OFFSET : FIXED_CRC_OFFSET + 4], "big")
    crc_calc = crc32(frame[:FIXED_CRC_OFFSET])
    if crc_recv != crc_calc:
        raise ValueError(f"bad crc: recv=0x{crc_recv:08X}, calc=0x{crc_calc:08X}")

    cmd = frame[2]
    data_len = int.from_bytes(frame[3:5], "big")
    block_id = int.from_bytes(frame[5:7], "big")

    data_start = FIXED_DATA_OFFSET + FIXED_BLOCK_ID_SIZE
    data = frame[data_start : data_start + data_len]
    return cmd, block_id, data


def names_from_flags(flags: int, names: dict[int, str]) -> list[str]:
    result = []

    for bit, name in names.items():
        if flags & (1 << bit):
            result.append(name)

    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Read ADDA heartbeat status")
    parser.add_argument("--host", default="192.168.1.21")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--bind", default=None)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3.0)

    if args.bind:
        sock.bind((args.bind, 0))

    try:
        print(f"connect {args.host}:{args.port}")
        sock.connect((args.host, args.port))

        sock.sendall(build_fixed_frame(CMD_HEARTBEAT, 0x0000))
        cmd, block_id, data = parse_fixed_reply(recv_exact(sock, FIXED_FRAME_LEN))

        print(f"reply: cmd=0x{cmd:02X}, block=0x{block_id:04X}, data_len={len(data)}")

        if cmd != CMD_HEARTBEAT or block_id != 0x0000:
            raise ValueError("reply is not heartbeat")

        if len(data) != HEARTBEAT_STATUS_LEN:
            raise ValueError(f"heartbeat length mismatch: {len(data)}")

        offset = 0
        status = data[offset]
        offset += 1

        alarm_flags = int.from_bytes(data[offset : offset + 4], "big")
        offset += 4

        state_flags = int.from_bytes(data[offset : offset + 4], "big")
        offset += 4

        avg_sample_count = int.from_bytes(data[offset : offset + 4], "big")
        offset += 4

        adc_avg = []
        for _ in range(12):
            adc_avg.append(struct.unpack(">f", data[offset : offset + 4])[0])
            offset += 4

        dac_codes = []
        for _ in range(4):
            dac_codes.append(int.from_bytes(data[offset : offset + 2], "big"))
            offset += 2

        alarm_names = names_from_flags(alarm_flags, ALARM_BITS)
        state_names = names_from_flags(state_flags, STATE_BITS)

        print(f"status: 0x{status:02X}")
        print(f"alarm_flags: 0x{alarm_flags:08X} {alarm_names}")
        print(f"state_flags: 0x{state_flags:08X} {state_names}")
        print(f"avg_sample_count: {avg_sample_count}")

        for index, value in enumerate(adc_avg, start=1):
            print(f"adc_avg[{index:02d}]: {value:.3f}")

        print(f"dac_codes: {dac_codes}")

    finally:
        sock.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
