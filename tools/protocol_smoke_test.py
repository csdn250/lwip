#!/usr/bin/env python3
"""Run a small ADDA TCP protocol smoke test.

The script verifies the fixed 150-byte command path:
- heartbeat
- read MAC block
- write/read DA1 manual voltage
- read DAC code status block
"""

import argparse
import socket
import struct
import time
import zlib


SOF = b"\x12\x34"
EOF = b"\x56\x78"

CMD_WRITE_PARAM = 0x01
CMD_READ_PARAM = 0x02
CMD_HEARTBEAT = 0x07

BLOCK_MAC = 0x0005
BLOCK_DA1 = 0x0009
BLOCK_DAC_STATUS = 0x000E

WRITE_STATUS_OK = 0x00
WRITE_STATUS_SAVE_PENDING = 0x01

BLOCK_DAC_CAL = 0x0010


FIXED_FRAME_LEN = 150
FIXED_CRC_OFFSET = 144
FIXED_EOF_OFFSET = 148
FIXED_DATA_OFFSET = 5
FIXED_BLOCK_ID_SIZE = 2
FIXED_DATA_CAPACITY = FIXED_CRC_OFFSET - FIXED_DATA_OFFSET - FIXED_BLOCK_ID_SIZE

DEFAULT_DA1_VOLTAGE = 2.5
DEFAULT_EXPECTED_DA1_CODE = 2048

CASCADE_ADC_CHANNEL = 0
CASCADE_K = 100.0
CASCADE_B = 0.0

UPDATED_DAC_K = 409.5
UPDATED_EXPECTED_DA1_CODE = 1024


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

def make_dac_cal_data(k: float = 819.0, b: float = 0.0) -> bytes:
    payload = bytearray()
    for _ in range(4):
        payload.extend(struct.pack(">ff", k, b))
    return bytes(payload)

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
        raise ValueError(f"bad fixed frame length: {len(frame)}")

    if frame[0:2] != SOF:
        raise ValueError("bad fixed frame header")

    if frame[FIXED_EOF_OFFSET : FIXED_EOF_OFFSET + 2] != EOF:
        raise ValueError("bad fixed frame tail")

    crc_recv = int.from_bytes(frame[FIXED_CRC_OFFSET : FIXED_CRC_OFFSET + 4], "big")
    crc_calc = crc32(frame[:FIXED_CRC_OFFSET])
    if crc_recv != crc_calc:
        raise ValueError(f"bad crc: recv=0x{crc_recv:08X}, calc=0x{crc_calc:08X}")

    cmd = frame[2]
    data_len = int.from_bytes(frame[3:5], "big")
    block_id = int.from_bytes(frame[5:7], "big")

    if data_len > FIXED_DATA_CAPACITY:
        raise ValueError(f"bad fixed data length: {data_len}")

    data_start = FIXED_DATA_OFFSET + FIXED_BLOCK_ID_SIZE
    data = frame[data_start : data_start + data_len]
    return cmd, block_id, data


def request(sock: socket.socket, cmd: int, block_id: int, data: bytes = b""):
    sock.sendall(build_fixed_frame(cmd, block_id, data))
    return parse_fixed_reply(recv_exact(sock, FIXED_FRAME_LEN))


def expect_reply(actual_cmd: int, actual_block: int, expected_cmd: int, expected_block: int) -> None:
    if actual_cmd != expected_cmd or actual_block != expected_block:
        raise AssertionError(
            f"unexpected reply: cmd=0x{actual_cmd:02X}, block=0x{actual_block:04X}"
        )


def make_da_manual_data(voltage: float) -> bytes:
    mode_manual = 0
    return struct.pack(">Bf", mode_manual, voltage)

def make_da_cascade_data(adc_channel: int, k: float, b: float) -> bytes:
    mode_cascade = 1
    return struct.pack(">BBff", mode_cascade, adc_channel, k, b)


def run_smoke_test(host: str, port: int, bind: str | None) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3.0)

    if bind:
        sock.bind((bind, 0))

    try:
        print(f"connect {host}:{port}")
        sock.connect((host, port))

        cmd, block, data = request(sock, CMD_HEARTBEAT, 0x0000)
        expect_reply(cmd, block, CMD_HEARTBEAT, 0x0000)
        if data != bytes([WRITE_STATUS_OK]):
            raise AssertionError(f"heartbeat status is not OK: {data.hex(' ')}")
        print("ok heartbeat")

        cmd, block, data = request(sock, CMD_READ_PARAM, BLOCK_MAC)
        expect_reply(cmd, block, CMD_READ_PARAM, BLOCK_MAC)
        if len(data) != 6:
            raise AssertionError(f"MAC block length mismatch: {len(data)}")
        print(f"ok read MAC: {data.hex(':')}")

        da_data = make_da_manual_data(DEFAULT_DA1_VOLTAGE)
        cmd, block, data = request(sock, CMD_WRITE_PARAM, BLOCK_DA1, da_data)
        expect_reply(cmd, block, CMD_WRITE_PARAM, BLOCK_DA1)
        if data != bytes([WRITE_STATUS_OK]):
            raise AssertionError(f"DA1 write status is not OK: {data.hex(' ')}")

        print(f"ok write DA1 manual voltage: status=0x{data[0]:02X}")

        time.sleep(0.1)

        cmd, block, data = request(sock, CMD_READ_PARAM, BLOCK_DA1)
        expect_reply(cmd, block, CMD_READ_PARAM, BLOCK_DA1)
        if data != da_data:
            raise AssertionError(f"DA1 readback mismatch: {data.hex(' ')}")
        print("ok read DA1 manual voltage")

        cmd, block, data = request(sock, CMD_READ_PARAM, BLOCK_DAC_STATUS)
        expect_reply(cmd, block, CMD_READ_PARAM, BLOCK_DAC_STATUS)
        if len(data) != 8:
            raise AssertionError(f"DAC status length mismatch: {len(data)}")

        codes = struct.unpack(">HHHH", data)
        if codes[0] != DEFAULT_EXPECTED_DA1_CODE:
            raise AssertionError(f"DA1 code mismatch: {codes[0]} != {DEFAULT_EXPECTED_DA1_CODE}")
        print(f"ok DAC status: DA codes={codes}")

        updated_cal_data = make_dac_cal_data(UPDATED_DAC_K, 0.0)
        cmd, block, data = request(sock, CMD_WRITE_PARAM, BLOCK_DAC_CAL, updated_cal_data)
        expect_reply(cmd, block, CMD_WRITE_PARAM, BLOCK_DAC_CAL)
        if data != bytes([WRITE_STATUS_SAVE_PENDING]):
            raise AssertionError(f"DAC cal update status is not SAVE_PENDING: {data.hex(' ')}")
        print("ok update DAC calibration")

        time.sleep(0.2)

        cmd, block, data = request(sock, CMD_READ_PARAM, BLOCK_DAC_STATUS)
        expect_reply(cmd, block, CMD_READ_PARAM, BLOCK_DAC_STATUS)
        if len(data) != 8:
            raise AssertionError(f"DAC status length mismatch after cal update: {len(data)}")

        codes = struct.unpack(">HHHH", data)
        if codes[0] != UPDATED_EXPECTED_DA1_CODE:
            raise AssertionError(
                f"DA1 code mismatch after cal update: {codes[0]} != {UPDATED_EXPECTED_DA1_CODE}"
            )
        print(f"ok DAC status after cal update: DA codes={codes}")

        default_cal_data = make_dac_cal_data()
        cmd, block, data = request(sock, CMD_WRITE_PARAM, BLOCK_DAC_CAL, default_cal_data)
        expect_reply(cmd, block, CMD_WRITE_PARAM, BLOCK_DAC_CAL)
        if data != bytes([WRITE_STATUS_SAVE_PENDING]):
            raise AssertionError(f"DAC cal restore status is not SAVE_PENDING: {data.hex(' ')}")
        print("ok restore DAC calibration")

        dac_cal_data = make_dac_cal_data()
        cmd, block, data = request(sock, CMD_WRITE_PARAM, BLOCK_DAC_CAL, dac_cal_data)
        expect_reply(cmd, block, CMD_WRITE_PARAM, BLOCK_DAC_CAL)
        if data != bytes([WRITE_STATUS_SAVE_PENDING]):
            raise AssertionError(f"DAC cal write status is not SAVE_PENDING: {data.hex(' ')}")
        print("ok write DAC calibration")

        time.sleep(0.1)

        cmd, block, data = request(sock, CMD_READ_PARAM, BLOCK_DAC_CAL)
        expect_reply(cmd, block, CMD_READ_PARAM, BLOCK_DAC_CAL)
        if data != dac_cal_data:
            raise AssertionError(f"DAC cal readback mismatch: {data.hex(' ')}")
        print("ok read DAC calibration")

        cascade_data = make_da_cascade_data(CASCADE_ADC_CHANNEL, CASCADE_K, CASCADE_B)
        cmd, block, data = request(sock, CMD_WRITE_PARAM, BLOCK_DA1, cascade_data)
        expect_reply(cmd, block, CMD_WRITE_PARAM, BLOCK_DA1)
        if data != bytes([WRITE_STATUS_OK]):
            raise AssertionError(f"DA1 cascade write status is not OK: {data.hex(' ')}")
        print("ok write DA1 cascade control")

        time.sleep(0.1)

        cmd, block, data = request(sock, CMD_READ_PARAM, BLOCK_DA1)
        expect_reply(cmd, block, CMD_READ_PARAM, BLOCK_DA1)
        if data != cascade_data:
            raise AssertionError(f"DA1 cascade readback mismatch: {data.hex(' ')}")
        print("ok read DA1 cascade control")

        da_data = make_da_manual_data(DEFAULT_DA1_VOLTAGE)
        cmd, block, data = request(sock, CMD_WRITE_PARAM, BLOCK_DA1, da_data)
        expect_reply(cmd, block, CMD_WRITE_PARAM, BLOCK_DA1)
        if data != bytes([WRITE_STATUS_OK]):
            raise AssertionError(f"DA1 restore manual status is not OK: {data.hex(' ')}")
        print("ok restore DA1 manual control")

        print("smoke test passed")
    finally:
        sock.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Run ADDA fixed protocol smoke test")
    parser.add_argument("--host", default="192.168.1.21", help="device IP")
    parser.add_argument("--port", type=int, default=8080, help="device TCP port")
    parser.add_argument("--bind", default=None, help="optional local source IP")
    args = parser.parse_args()

    try:
        run_smoke_test(args.host, args.port, args.bind)
    except (OSError, ConnectionError, TimeoutError, ValueError, AssertionError) as exc:
        print(f"smoke test failed: {exc}")
        print("hint: close NetAssist or other TCP clients first; firmware accepts one client at a time.")
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
