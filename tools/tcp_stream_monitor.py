#!/usr/bin/env python3
"""Connect to the ADDA TCP server, start streaming, parse frames, and print stats."""

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
CMD_RAW_DATA = 0x81
CMD_CONVERTED_DATA = 0x82
CMD_DEBUG_STATUS = 0x83

BLOCK_CONTROL = 0x0002

FRAME_HEADER_LEN = 5
FRAME_CRC_LEN = 4
FRAME_EOF_LEN = 2
FRAME_OVERHEAD = FRAME_HEADER_LEN + FRAME_CRC_LEN + FRAME_EOF_LEN
FIXED_FRAME_LEN = 150
FIXED_CRC_OFFSET = 144
FIXED_EOF_OFFSET = 148
FIXED_DATA_OFFSET = 5
FIXED_BLOCK_ID_SIZE = 2
FIXED_DATA_CAPACITY = FIXED_CRC_OFFSET - FIXED_DATA_OFFSET - FIXED_BLOCK_ID_SIZE

ADC_PAYLOAD_HEADER_LEN = 20
ADC_FRAME_MAGIC = 0x41444331
SCRIPT_VERSION = "2026-06-03-batch-seq-v2"


def crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


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
    crc = crc32(body)
    return body + crc.to_bytes(4, "big") + EOF


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


def build_control_data(enable: bool, stream_cmd: int) -> bytes:
    start_stop = 1 if enable else 0
    # control[8]: enable + stream type + reserved bytes
    return bytes([start_stop, stream_cmd, 0, 0, 0, 0, 0, 0])


def is_fixed_cmd(cmd: int) -> bool:
    return cmd in (CMD_WRITE_PARAM, CMD_READ_PARAM, CMD_HEARTBEAT)


def parse_one_frame(buf: bytearray):
    sof_index = buf.find(SOF)
    if sof_index < 0:
        buf.clear()
        return None

    if sof_index > 0:
        del buf[:sof_index]

    if len(buf) < FRAME_OVERHEAD:
        return None

    cmd = buf[2]
    payload_len = (buf[3] << 8) | buf[4]

    if is_fixed_cmd(cmd):
        frame_len = FIXED_FRAME_LEN

        if len(buf) < frame_len:
            return None

        frame = bytes(buf[:frame_len])
        del buf[:frame_len]

        if frame[FIXED_EOF_OFFSET : FIXED_EOF_OFFSET + 2] != EOF:
            return {"ok": False, "reason": "bad_fixed_eof", "raw": frame}

        if payload_len > FIXED_DATA_CAPACITY:
            return {"ok": False, "reason": "bad_fixed_len", "raw": frame}

        crc_recv = int.from_bytes(frame[FIXED_CRC_OFFSET : FIXED_CRC_OFFSET + 4], "big")
        crc_calc = crc32(frame[:FIXED_CRC_OFFSET])
        if crc_recv != crc_calc:
            return {"ok": False, "reason": "bad_fixed_crc", "raw": frame}

        block_id = int.from_bytes(frame[FIXED_DATA_OFFSET : FIXED_DATA_OFFSET + 2], "big")
        data_start = FIXED_DATA_OFFSET + FIXED_BLOCK_ID_SIZE
        data = frame[data_start : data_start + payload_len]

        return {
            "ok": True,
            "cmd": cmd,
            "payload": block_id.to_bytes(2, "big") + data,
            "raw_len": frame_len,
        }

    frame_len = FRAME_OVERHEAD + payload_len

    if len(buf) < frame_len:
        return None

    frame = bytes(buf[:frame_len])
    del buf[:frame_len]

    if frame[-2:] != EOF:
        return {"ok": False, "reason": "bad_eof", "raw": frame}

    crc_recv = int.from_bytes(frame[FRAME_HEADER_LEN + payload_len : FRAME_HEADER_LEN + payload_len + 4], "big")
    crc_calc = crc32(frame[: FRAME_HEADER_LEN + payload_len])
    if crc_recv != crc_calc:
        return {"ok": False, "reason": "bad_crc", "raw": frame}

    return {
        "ok": True,
        "cmd": cmd,
        "payload": frame[FRAME_HEADER_LEN : FRAME_HEADER_LEN + payload_len],
        "raw_len": frame_len,
    }


def parse_adc_payload(payload: bytes, cmd: int):
    if len(payload) < ADC_PAYLOAD_HEADER_LEN:
        return None

    magic, seq, timestamp_us, channel_mask, channel_count, sample_format, payload_bytes = struct.unpack_from(
        ">IIIHHHH", payload, 0
    )

    if magic != ADC_FRAME_MAGIC:
        return None

    data = payload[ADC_PAYLOAD_HEADER_LEN : ADC_PAYLOAD_HEADER_LEN + payload_bytes]
    values = []

    if cmd == CMD_RAW_DATA:
        for i in range(0, len(data), 2):
            if i + 1 < len(data):
                values.append((data[i] << 8) | data[i + 1])
        bytes_per_group = channel_count * 2
    elif cmd == CMD_CONVERTED_DATA:
        for i in range(0, len(data), 4):
            if i + 3 < len(data):
                values.append(struct.unpack_from(">f", data, i)[0])
        bytes_per_group = channel_count * 4
    else:
        bytes_per_group = 0

    if bytes_per_group > 0:
        group_count = payload_bytes // bytes_per_group
    else:
        group_count = 0

    return {
        "seq": seq,
        "timestamp_us": timestamp_us,
        "channel_mask": channel_mask,
        "channel_count": channel_count,
        "sample_format": sample_format,
        "payload_bytes": payload_bytes,
        "group_count": group_count,
        "values": values,
    }


def parse_debug_status(payload: bytes):
    if len(payload) < 42:
        return None

    values = struct.unpack_from(">IIIIIIIIIIH", payload, 0)

    return {
        "adc1_half": values[0],
        "adc2_half": values[1],
        "adc3_half": values[2],
        "adc1_full": values[3],
        "adc2_full": values[4],
        "adc3_full": values[5],
        "sample_seq": values[6],
        "no_block": values[7],
        "tcp_send_fail": values[8],
        "stream_seq": values[9],
        "tcp_sndbuf_min": values[10],
    }


def format_rate(byte_count: int, dt: float) -> str:
    if dt <= 0:
        return "0 B/s"
    bps = byte_count / dt
    mbps = (bps * 8.0) / 1_000_000.0
    return f"{bps:,.0f} B/s, {mbps:.3f} Mbps"


def main():
    parser = argparse.ArgumentParser(description="ADDA TCP stream monitor")
    parser.add_argument("--host", default="192.168.1.21", help="device IP")
    parser.add_argument("--port", type=int, default=8080, help="device TCP port")
    parser.add_argument("--bind", default=None, help="optional local source IP, e.g. 192.168.1.20")
    parser.add_argument("--type", choices=["raw", "converted"], default="raw", help="stream type")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds to run, 0 means until Ctrl+C")
    parser.add_argument("--no-stop", action="store_true", help="do not send stop command on exit")
    args = parser.parse_args()

    stream_cmd = CMD_RAW_DATA if args.type == "raw" else CMD_CONVERTED_DATA
    start_frame = build_fixed_frame(CMD_WRITE_PARAM, BLOCK_CONTROL, build_control_data(True, stream_cmd))
    stop_frame = build_fixed_frame(CMD_WRITE_PARAM, BLOCK_CONTROL, build_control_data(False, stream_cmd))

    rx_buf = bytearray()
    total_bytes = 0
    total_frames = 0
    data_frames = 0
    sample_groups = 0
    bad_frames = 0
    gap_samples = 0
    overlap_samples = 0
    last_seq = None
    expected_seq = None
    last_values = []
    last_group_count = 0
    channel_count = 0
    last_debug = None

    interval_bytes = 0
    interval_data_frames = 0
    interval_sample_groups = 0
    start_time = time.monotonic()
    last_report = start_time

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(1.0)

    if args.bind:
        sock.bind((args.bind, 0))

    print(f"monitor version: {SCRIPT_VERSION}")
    print(f"connect {args.host}:{args.port}")
    sock.connect((args.host, args.port))
    print(f"send start: cmd=0x{stream_cmd:02X}")
    sock.sendall(start_frame)

    try:
        while True:
            now = time.monotonic()
            if args.duration > 0 and (now - start_time) >= args.duration:
                break

            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                chunk = b""

            if not chunk:
                pass
            else:
                rx_buf.extend(chunk)
                total_bytes += len(chunk)
                interval_bytes += len(chunk)

            while True:
                frame = parse_one_frame(rx_buf)
                if frame is None:
                    break

                total_frames += 1
                if not frame["ok"]:
                    bad_frames += 1
                    continue

                cmd = frame["cmd"]
                payload = frame["payload"]

                if cmd == CMD_WRITE_PARAM and len(payload) >= 3:
                    block_id = (payload[0] << 8) | payload[1]
                    status = payload[2]
                    print(f"write status: block=0x{block_id:04X}, status=0x{status:02X}")
                elif cmd == CMD_DEBUG_STATUS:
                    debug = parse_debug_status(payload)
                    if debug is None:
                        bad_frames += 1
                    else:
                        last_debug = debug
                elif cmd in (CMD_RAW_DATA, CMD_CONVERTED_DATA):
                    info = parse_adc_payload(payload, cmd)
                    if info is None:
                        bad_frames += 1
                        continue

                    seq = info["seq"]
                    group_count = info["group_count"]
                    if expected_seq is not None and seq != expected_seq:
                        delta = (seq - expected_seq) & 0xFFFFFFFF
                        if delta < 0x80000000:
                            # 序号向前跳，说明中间有采样组没有收到。
                            gap_samples += delta
                        else:
                            # 序号回退或重复，通常是批量发送时读取到了重叠采样块，
                            # 不能按“丢包”计算，否则会得到非常大的假 lost 数。
                            overlap_samples += (expected_seq - seq) & 0xFFFFFFFF
                    last_seq = seq
                    expected_seq = (seq + group_count) & 0xFFFFFFFF

                    data_frames += 1
                    sample_groups += group_count
                    interval_data_frames += 1
                    interval_sample_groups += group_count
                    channel_count = info["channel_count"]
                    last_group_count = group_count
                    last_values = info["values"][: min(4, len(info["values"]))]

            now = time.monotonic()
            if now - last_report >= 1.0:
                dt = now - last_report
                tcp_frame_rate = interval_data_frames / dt
                per_channel_sps = interval_sample_groups / dt
                total_channel_sps = per_channel_sps * channel_count
                debug_text = ""

                if last_debug is not None:
                    debug_text = (
                        f", adc_half=({last_debug['adc1_half']},{last_debug['adc2_half']},{last_debug['adc3_half']})"
                        f", adc_full=({last_debug['adc1_full']},{last_debug['adc2_full']},{last_debug['adc3_full']})"
                        f", adc_seq={last_debug['sample_seq']}"
                        f", no_block={last_debug['no_block']}"
                        f", tcp_fail={last_debug['tcp_send_fail']}"
                        f", stream_seq={last_debug['stream_seq']}"
                        f", sndbuf_min={last_debug['tcp_sndbuf_min']}"
                    )

                print(
                    "stats: "
                    f"tcp_frames={data_frames}, "
                    f"samples={sample_groups}, "
                    f"tcp_rate={tcp_frame_rate:.1f} frame/s, "
                    f"per_ch={per_channel_sps:.1f} Sa/s, "
                    f"total_ch={total_channel_sps:.1f} Sa/s, "
                    f"net={format_rate(interval_bytes, dt)}, "
                    f"gap={gap_samples}, overlap={overlap_samples}, bad={bad_frames}, "
                    f"group={last_group_count}, "
                    f"last_seq={last_seq}, first_values={last_values}"
                    f"{debug_text}"
                )
                interval_bytes = 0
                interval_data_frames = 0
                interval_sample_groups = 0
                last_report = now

    except KeyboardInterrupt:
        print("stopping by Ctrl+C")
    finally:
        if not args.no_stop:
            try:
                print("send stop")
                sock.sendall(stop_frame)
                time.sleep(0.1)
            except OSError:
                pass
        sock.close()

    elapsed = time.monotonic() - start_time
    print(
        "summary: "
        f"elapsed={elapsed:.2f}s, bytes={total_bytes}, tcp_frames={data_frames}, samples={sample_groups}, "
        f"avg_tcp_frame_rate={(data_frames / elapsed) if elapsed > 0 else 0:.1f} frame/s, "
        f"avg_per_ch={(sample_groups / elapsed) if elapsed > 0 else 0:.1f} Sa/s, "
        f"gap={gap_samples}, overlap={overlap_samples}, bad={bad_frames}"
    )


if __name__ == "__main__":
    main()
