"""Capture [BRIDGE]-prefixed lines (RANGE, STAT, CONNECTED, ...) from the ESP32 bridge's
own side-channel over a live TCP connection, writing each line to disk as it arrives -
NOT buffered in memory until the end, so stopping the capture early (Ctrl+C, a tool
timeout, an agent killing the process) never loses already-captured data the way an
end-of-run-only write does.

Usage: python capture_bridge_lines.py <host> [--port 3333] [--out FILE] [--duration SECONDS]
"""
import argparse
import re
import socket
import sys
import time

RANGE_RE = re.compile(r"\[BRIDGE\] RANGE cm=([\d.]+) age_ms=(\d+)")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("host")
    parser.add_argument("--port", type=int, default=3333)
    parser.add_argument("--out", default=None, help="default: bridge_capture_<timestamp>.csv")
    parser.add_argument("--duration", type=float, default=180.0)
    args = parser.parse_args()

    out_path = args.out or f"bridge_capture_{time.strftime('%Y%m%d_%H%M%S')}.csv"

    sock = socket.create_connection((args.host, args.port), timeout=5.0)
    sock.settimeout(1.0)
    buf = bytearray()
    n_range = 0
    n_other = 0
    deadline = time.time() + args.duration

    with open(out_path, "w", buffering=1) as f:  # line-buffered - each write() hits disk promptly
        f.write("t_epoch,kind,cm,age_ms,raw\n")
        print(f"Capturing to {out_path} for up to {args.duration:.0f}s (Ctrl+C to stop early - "
              f"already-written rows are safe either way)...", file=sys.stderr)
        try:
            while time.time() < deadline:
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    print("connection closed by peer", file=sys.stderr)
                    break
                buf.extend(chunk)
                while b"\n" in buf:
                    idx = buf.index(b"\n")
                    line = bytes(buf[:idx]).decode(errors="replace").strip()
                    del buf[:idx + 1]
                    if not line:
                        continue
                    t = time.time()
                    m = RANGE_RE.search(line)
                    if m:
                        n_range += 1
                        f.write(f"{t:.3f},RANGE,{m.group(1)},{m.group(2)},\n")
                    elif line.startswith("[BRIDGE]"):
                        n_other += 1
                        f.write(f"{t:.3f},OTHER,,,\"{line}\"\n")
        except KeyboardInterrupt:
            print("stopped by user", file=sys.stderr)
    sock.close()
    print(f"{n_range} RANGE samples, {n_other} other [BRIDGE] lines saved to {out_path}", file=sys.stderr)


if __name__ == "__main__":
    main()
