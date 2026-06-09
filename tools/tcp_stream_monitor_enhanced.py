#!/usr/bin/env python3
"""Enhanced ADDA TCP stream monitor.

Compared with the original script, this version keeps the same protocol parser
and start/stop behavior, but adds:
- aligned terminal dashboard
- per-interval deltas for gap/overlap/bad frames
- optional CSV export
- optional matplotlib live chart
"""

import argparse
import csv
import socket
import struct
import sys
import time
import zlib
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

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
SCRIPT_VERSION = "2026-06-05-enhanced-dashboard-v1"


@dataclass
class Totals:
    bytes: int = 0
    total_frames: int = 0
    data_frames: int = 0
    sample_groups: int = 0
    bad_frames: int = 0
    gap_samples: int = 0
    overlap_samples: int = 0


@dataclass
class Interval:
    bytes: int = 0
    data_frames: int = 0
    sample_groups: int = 0
    bad_frames: int = 0
    gap_samples: int = 0
    overlap_samples: int = 0

    def reset(self) -> None:
        self.bytes = 0
        self.data_frames = 0
        self.sample_groups = 0
        self.bad_frames = 0
        self.gap_samples = 0
        self.overlap_samples = 0


def crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    head = bytes([SOF[0], SOF[1], cmd & 0xFF, (len(payload) >> 8) & 0xFF, len(payload) & 0xFF])
    body = head + payload
    return body + crc32(body).to_bytes(4, "big") + EOF


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


def build_control_payload(enable: bool, stream_cmd: int) -> bytes:
    start_stop = 1 if enable else 0
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

    group_count = payload_bytes // bytes_per_group if bytes_per_group > 0 else 0

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


def format_rate(byte_count: int, dt: float) -> Tuple[float, float]:
    if dt <= 0:
        return 0.0, 0.0
    bps = byte_count / dt
    mbps = (bps * 8.0) / 1_000_000.0
    return bps, mbps


def latest_channel_values(values: List[float], channel_count: int, max_channels: int) -> List[float]:
    if channel_count <= 0 or not values:
        return []
    return values[-channel_count:][:max_channels]


def interval_status(interval: Interval) -> str:
    if interval.bad_frames:
        return "BAD_FRAME"
    if interval.gap_samples:
        return "GAP"
    if interval.overlap_samples:
        return "OVERLAP"
    return "OK"


def print_header() -> None:
    print()
    print(
        f"{'time':>8} | {'frame/s':>8} | {'per_ch Sa/s':>11} | {'total Sa/s':>11} | "
        f"{'net Mbps':>8} | {'gap':>6} | {'ovlp':>6} | {'bad':>5} | {'grp':>4} | {'seq':>10} | {'status':>9}"
    )
    print("-" * 111)


def make_csv_writer(path: Optional[str]):
    if not path:
        return None, None
    f = open(path, "w", newline="", encoding="utf-8")
    writer = csv.DictWriter(
        f,
        fieldnames=[
            "time_s", "tcp_frames_total", "sample_groups_total", "tcp_frame_rate_fps",
            "per_channel_sps", "total_channel_sps", "net_Bps", "net_Mbps",
            "gap_total", "gap_delta", "overlap_total", "overlap_delta",
            "bad_total", "bad_delta", "group_count", "last_seq", "channel_count", "status",
            "adc_seq", "no_block", "tcp_send_fail", "stream_seq", "tcp_sndbuf_min",
        ],
    )
    writer.writeheader()
    return f, writer


class LivePlot:
    def __init__(self, max_points: int = 120):
        import matplotlib.pyplot as plt
        self.plt = plt
        self.t: Deque[float] = deque(maxlen=max_points)
        self.per_ch: Deque[float] = deque(maxlen=max_points)
        self.net_mbps: Deque[float] = deque(maxlen=max_points)
        self.gap: Deque[int] = deque(maxlen=max_points)

        plt.ion()
        self.fig, self.ax1 = plt.subplots()
        self.ax2 = self.ax1.twinx()
        (self.line_sps,) = self.ax1.plot([], [], label="per-channel Sa/s")
        (self.line_gap,) = self.ax1.plot([], [], label="gap delta")
        (self.line_net,) = self.ax2.plot([], [], label="network Mbps")
        self.ax1.set_xlabel("time (s)")
        self.ax1.set_ylabel("Sa/s and gap")
        self.ax2.set_ylabel("Mbps")
        self.ax1.grid(True)
        self.fig.legend(loc="upper left")

    def update(self, elapsed: float, per_ch_sps: float, net_mbps: float, gap_delta: int) -> None:
        self.t.append(elapsed)
        self.per_ch.append(per_ch_sps)
        self.net_mbps.append(net_mbps)
        self.gap.append(gap_delta)
        self.line_sps.set_data(self.t, self.per_ch)
        self.line_gap.set_data(self.t, self.gap)
        self.line_net.set_data(self.t, self.net_mbps)
        if self.t:
            self.ax1.set_xlim(max(0.0, self.t[0]), max(1.0, self.t[-1]))
        self.ax1.set_ylim(0, max([1.0, *self.per_ch, *self.gap]) * 1.2)
        self.ax2.set_ylim(0, max([1.0, *self.net_mbps]) * 1.2)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main() -> int:
    parser = argparse.ArgumentParser(description="Enhanced ADDA TCP stream monitor")
    parser.add_argument("--host", default="192.168.1.21", help="device IP")
    parser.add_argument("--port", type=int, default=8080, help="device TCP port")
    parser.add_argument("--bind", default=None, help="optional local source IP, e.g. 192.168.1.20")
    parser.add_argument("--type", choices=["raw", "converted"], default="raw", help="stream type")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds to run, 0 means until Ctrl+C")
    parser.add_argument("--interval", type=float, default=1.0, help="report interval in seconds")
    parser.add_argument("--no-stop", action="store_true", help="do not send stop command on exit")
    parser.add_argument("--csv", default=None, help="save per-interval statistics to CSV")
    parser.add_argument("--plot", action="store_true", help="show live matplotlib chart")
    parser.add_argument("--show-values", type=int, default=4, help="print latest N channel values, 0 disables")
    args = parser.parse_args()

    if args.interval <= 0:
        print("--interval must be > 0", file=sys.stderr)
        return 2

    stream_cmd = CMD_RAW_DATA if args.type == "raw" else CMD_CONVERTED_DATA
    start_frame = build_fixed_frame(CMD_WRITE_PARAM, BLOCK_CONTROL, build_control_payload(True, stream_cmd))
    stop_frame = build_fixed_frame(CMD_WRITE_PARAM, BLOCK_CONTROL, build_control_payload(False, stream_cmd))

    rx_buf = bytearray()
    totals = Totals()
    interval = Interval()
    last_seq = None
    expected_seq = None
    last_values: List[float] = []
    last_group_count = 0
    channel_count = 0
    last_debug = None
    previous_gap_total = 0
    previous_overlap_total = 0
    previous_bad_total = 0

    csv_file, csv_writer = make_csv_writer(args.csv)
    live_plot = None
    if args.plot:
        try:
            live_plot = LivePlot()
        except Exception as exc:
            print(f"plot disabled: {exc}", file=sys.stderr)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(1.0)
    if args.bind:
        sock.bind((args.bind, 0))

    start_time = time.monotonic()
    last_report = start_time

    print(f"monitor version: {SCRIPT_VERSION}")
    print(f"connect {args.host}:{args.port}")
    sock.connect((args.host, args.port))
    print(f"send start: cmd=0x{stream_cmd:02X}, type={args.type}")
    sock.sendall(start_frame)
    print_header()

    try:
        while True:
            now = time.monotonic()
            if args.duration > 0 and (now - start_time) >= args.duration:
                break

            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                chunk = b""

            if chunk:
                rx_buf.extend(chunk)
                totals.bytes += len(chunk)
                interval.bytes += len(chunk)

            while True:
                frame = parse_one_frame(rx_buf)
                if frame is None:
                    break
                totals.total_frames += 1
                if not frame["ok"]:
                    totals.bad_frames += 1
                    interval.bad_frames += 1
                    continue

                cmd = frame["cmd"]
                payload = frame["payload"]

                if cmd == CMD_WRITE_PARAM and len(payload) >= 3:
                    block_id = (payload[0] << 8) | payload[1]
                    status = payload[2]
                    print(f"\nwrite status: block=0x{block_id:04X}, status=0x{status:02X}")
                elif cmd == CMD_DEBUG_STATUS:
                    debug = parse_debug_status(payload)
                    if debug is None:
                        totals.bad_frames += 1
                        interval.bad_frames += 1
                    else:
                        last_debug = debug
                elif cmd in (CMD_RAW_DATA, CMD_CONVERTED_DATA):
                    info = parse_adc_payload(payload, cmd)
                    if info is None:
                        totals.bad_frames += 1
                        interval.bad_frames += 1
                        continue

                    seq = info["seq"]
                    group_count = info["group_count"]
                    if expected_seq is not None and seq != expected_seq:
                        delta = (seq - expected_seq) & 0xFFFFFFFF
                        if delta < 0x80000000:
                            totals.gap_samples += delta
                            interval.gap_samples += delta
                        else:
                            overlap = (expected_seq - seq) & 0xFFFFFFFF
                            totals.overlap_samples += overlap
                            interval.overlap_samples += overlap

                    last_seq = seq
                    expected_seq = (seq + group_count) & 0xFFFFFFFF
                    totals.data_frames += 1
                    totals.sample_groups += group_count
                    interval.data_frames += 1
                    interval.sample_groups += group_count
                    channel_count = info["channel_count"]
                    last_group_count = group_count
                    last_values = info["values"]

            now = time.monotonic()
            if now - last_report >= args.interval:
                dt = now - last_report
                elapsed = now - start_time
                tcp_frame_rate = interval.data_frames / dt
                per_ch_sps = interval.sample_groups / dt
                total_ch_sps = per_ch_sps * channel_count
                net_Bps, net_Mbps = format_rate(interval.bytes, dt)
                status = interval_status(interval)
                gap_delta = totals.gap_samples - previous_gap_total
                overlap_delta = totals.overlap_samples - previous_overlap_total
                bad_delta = totals.bad_frames - previous_bad_total

                print(
                    f"{elapsed:8.1f} | {tcp_frame_rate:8.1f} | {per_ch_sps:11.1f} | "
                    f"{total_ch_sps:11.1f} | {net_Mbps:8.3f} | {gap_delta:6d} | "
                    f"{overlap_delta:6d} | {bad_delta:5d} | {last_group_count:4d} | "
                    f"{str(last_seq):>10} | {status:>9}"
                )

                if args.show_values > 0:
                    latest = latest_channel_values(last_values, channel_count, args.show_values)
                    if latest:
                        print(f"         latest ch0..ch{len(latest)-1}: {latest}")

                if last_debug is not None:
                    print(
                        "         debug: "
                        f"half=({last_debug['adc1_half']},{last_debug['adc2_half']},{last_debug['adc3_half']}), "
                        f"full=({last_debug['adc1_full']},{last_debug['adc2_full']},{last_debug['adc3_full']}), "
                        f"adc_seq={last_debug['sample_seq']}, no_block={last_debug['no_block']}, "
                        f"tcp_fail={last_debug['tcp_send_fail']}, stream_seq={last_debug['stream_seq']}, "
                        f"sndbuf_min={last_debug['tcp_sndbuf_min']}"
                    )

                if csv_writer is not None:
                    csv_writer.writerow({
                        "time_s": f"{elapsed:.3f}",
                        "tcp_frames_total": totals.data_frames,
                        "sample_groups_total": totals.sample_groups,
                        "tcp_frame_rate_fps": f"{tcp_frame_rate:.3f}",
                        "per_channel_sps": f"{per_ch_sps:.3f}",
                        "total_channel_sps": f"{total_ch_sps:.3f}",
                        "net_Bps": f"{net_Bps:.3f}",
                        "net_Mbps": f"{net_Mbps:.6f}",
                        "gap_total": totals.gap_samples,
                        "gap_delta": gap_delta,
                        "overlap_total": totals.overlap_samples,
                        "overlap_delta": overlap_delta,
                        "bad_total": totals.bad_frames,
                        "bad_delta": bad_delta,
                        "group_count": last_group_count,
                        "last_seq": last_seq if last_seq is not None else "",
                        "channel_count": channel_count,
                        "status": status,
                        "adc_seq": last_debug["sample_seq"] if last_debug else "",
                        "no_block": last_debug["no_block"] if last_debug else "",
                        "tcp_send_fail": last_debug["tcp_send_fail"] if last_debug else "",
                        "stream_seq": last_debug["stream_seq"] if last_debug else "",
                        "tcp_sndbuf_min": last_debug["tcp_sndbuf_min"] if last_debug else "",
                    })
                    csv_file.flush()

                if live_plot is not None:
                    live_plot.update(elapsed, per_ch_sps, net_Mbps, gap_delta)

                previous_gap_total = totals.gap_samples
                previous_overlap_total = totals.overlap_samples
                previous_bad_total = totals.bad_frames
                interval.reset()
                last_report = now

    except KeyboardInterrupt:
        print("\nstopping by Ctrl+C")
    finally:
        if not args.no_stop:
            try:
                print("send stop")
                sock.sendall(stop_frame)
                time.sleep(0.1)
            except OSError:
                pass
        sock.close()
        if csv_file is not None:
            csv_file.close()

    elapsed = time.monotonic() - start_time
    print(
        "summary: "
        f"elapsed={elapsed:.2f}s, bytes={totals.bytes}, tcp_frames={totals.data_frames}, "
        f"samples={totals.sample_groups}, "
        f"avg_tcp_frame_rate={(totals.data_frames / elapsed) if elapsed > 0 else 0:.1f} frame/s, "
        f"avg_per_ch={(totals.sample_groups / elapsed) if elapsed > 0 else 0:.1f} Sa/s, "
        f"gap={totals.gap_samples}, overlap={totals.overlap_samples}, bad={totals.bad_frames}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
