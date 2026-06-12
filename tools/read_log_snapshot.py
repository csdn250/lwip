#!/usr/bin/env python3
"""Read and decode the ADDA log snapshot block."""

import argparse
import socket
import zlib


SOF = b"\x12\x34"
EOF = b"\x56\x78"

CMD_READ_PARAM = 0x02
BLOCK_LOG_SNAPSHOT = 0x000D

LOG_ACTION_READ_RAM = 0x00
LOG_ACTION_READ_PERSIST = 0x02

FIXED_FRAME_LEN = 150
FIXED_CRC_OFFSET = 144
FIXED_EOF_OFFSET = 148
FIXED_DATA_CAPACITY = 137

LOG_HEADER_LEN = 4


EVENT_NAMES = {
    1: "LOGGER_STARTED",
    2: "PERIPHERALS_READY",
    3: "LWIP_READY",
    4: "ETH_LINK_UP",
    5: "ETH_LINK_DOWN",
    6: "ADC_DMA_STARTED",
    7: "ADC_DMA_ERROR",
    8: "SPI_DAC_ERROR",
    9: "RS485_ERROR",
    10: "I2C_EEPROM_ERROR",
    11: "TCP_ACCEPT",
    12: "TCP_CLOSE",
    13: "TCP_ERROR",
    14: "TCP_RX_OVERFLOW",
    15: "TCP_BAD_FRAME",
    16: "PROTO_HEARTBEAT",
    17: "PROTO_READ_PARAM",
    18: "PROTO_WRITE_PARAM",
    19: "PARAM_WRITE_RESULT",
    20: "NETIF_APPLIED",
    21: "ADC_STREAM_START",
    22: "ADC_STREAM_STOP",
    23: "ADC_STREAM_SEND_FAIL",
    24: "DAC_PARAM_APPLIED",
    25: "DAC_OUTPUT_LIMIT",
    26: "LOG_CLEARED",
    27: "RESET_CAUSE",
    28: "WATCHDOG_TEST",
    29: "CONFIG_LOADED",
    30: "CONFIG_DEFAULT",
    31: "ADC_INIT_FAILED",
    32: "ADC_START_FAILED",
    33: "SYSTEM_READY",
    34: "TCP_STREAM_STOPPED",
    35: "TCP_STREAM_STARTED",
    36: "IWDG_REFRESH_FAILED",
    37: "NETIF_FAILED",
    38: "ERROR_HANDLER",
    39: "CONFIG_SAVED",
    40: "CONFIG_SAVE_FAILED",
    41: "TCP_IDLE_TIMEOUT",
}


def crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def build_fixed_frame(cmd: int, block_id: int, data: bytes = b"") -> bytes:
    if len(data) > FIXED_DATA_CAPACITY:
        raise ValueError(f"fixed frame data too long: {len(data)}")

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

    if data_len > FIXED_DATA_CAPACITY:
        raise ValueError(f"bad data length: {data_len}")

    data = frame[7 : 7 + data_len]
    return cmd, block_id, data


def parse_log_snapshot(data: bytes):
    if len(data) < LOG_HEADER_LEN:
        raise ValueError("log snapshot data too short")

    version = data[0]
    record_count = data[1]
    record_size = data[2]

    if record_size < 16:
        raise ValueError(f"unsupported record size: {record_size}")

    records = []
    offset = LOG_HEADER_LEN

    for _ in range(record_count):
        if offset + record_size > len(data):
            break

        item = data[offset : offset + record_size]
        tick_ms = int.from_bytes(item[0:4], "big")
        event = int.from_bytes(item[4:6], "big")
        arg0 = int.from_bytes(item[6:8], "big")
        arg1 = int.from_bytes(item[8:12], "big")
        arg2 = int.from_bytes(item[12:16], "big")

        records.append((tick_ms, event, arg0, arg1, arg2))
        offset += record_size

    return version, record_size, records


def main() -> int:
    parser = argparse.ArgumentParser(description="Read ADDA log snapshot")
    parser.add_argument("--host", default="192.168.1.21", help="device IP")
    parser.add_argument("--port", type=int, default=8080, help="device TCP port")
    parser.add_argument("--bind", default=None, help="optional local source IP")
    parser.add_argument(
        "--source",
        choices=("ram", "persist"),
        default="ram",
        help="log source: RAM snapshot or EEPROM persistent snapshot",
    )
    args = parser.parse_args()

    action = LOG_ACTION_READ_RAM
    if args.source == "persist":
        action = LOG_ACTION_READ_PERSIST

    request = build_fixed_frame(CMD_READ_PARAM, BLOCK_LOG_SNAPSHOT, bytes([action]))

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3.0)

    if args.bind:
        sock.bind((args.bind, 0))

    try:
        sock.connect((args.host, args.port))
        sock.sendall(request)
        reply = recv_exact(sock, FIXED_FRAME_LEN)
    finally:
        sock.close()

    cmd, block_id, data = parse_fixed_reply(reply)

    print(f"reply: cmd=0x{cmd:02X}, block=0x{block_id:04X}, data_len={len(data)}")

    if cmd != CMD_READ_PARAM or block_id != BLOCK_LOG_SNAPSHOT:
        raise ValueError("reply is not a log snapshot")

    version, record_size, records = parse_log_snapshot(data)
    print(
        f"log snapshot: source={args.source}, "
        f"version={version}, record_size={record_size}, count={len(records)}"
    )
    print("idx | tick_ms | event | name                 | arg0 | arg1       | arg2")
    print("----+---------+-------+----------------------+------+------------+------------")

    for index, (tick_ms, event, arg0, arg1, arg2) in enumerate(records, start=1):
        name = EVENT_NAMES.get(event, "UNKNOWN")
        print(
            f"{index:>3} | {tick_ms:>7} | {event:>5} | {name:<20} | "
            f"{arg0:>4} | {arg1:>10} | {arg2:>10}"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
