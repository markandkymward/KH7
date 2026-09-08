"""Capture EVERY line from the ESP32 bridge's live TCP stream during a real flight,
writing each to disk as it arrives (line-buffered, Ctrl+C-safe - same incremental-write
pattern as capture_bridge_lines.py). Unlike that script, this does NOT filter to
[BRIDGE]-prefixed lines only: it keeps FC-originated telemetry (VEKF[...], IMU[...],
SENSOR ..., etc.) too, since those are relayed as-is over the same UART6 link and are
exactly what a real-liftoff vert_ekf.c validation needs (real 500Hz predict, real
ESP32 confidence, real pre-LPF baro - none of which the offline SD-log replay could
reproduce).

Usage: python capture_liftoff_telemetry.py <host> [--port 3333] [--out FILE] [--duration SECONDS]
"""
import argparse
import socket
import sys
import time


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("host")
    parser.add_argument("--port", type=int, default=3333)
    parser.add_argument("--out", default=None, help="default: liftoff_capture_<timestamp>.txt")
    parser.add_argument("--duration", type=float, default=180.0)
    args = parser.parse_args()

    out_path = args.out or f"liftoff_capture_{time.strftime('%Y%m%d_%H%M%S')}.txt"

    n_lines = 0
    n_vekf = 0
    deadline = time.time() + args.duration
    RECONNECT_DELAY_S = 1.0
    STALE_CONN_S = 5.0  # no bytes at all for this long -> assume the bridge/FC power-cycled and reconnect

    # encoding="utf-8" explicitly - the platform default (cp1252 on Windows) can't
    # represent U+FFFD, which a corrupted/binary byte in the stream decodes to
    # (errors="replace" below), crashing the whole capture on one bad byte.
    with open(out_path, "w", buffering=1, encoding="utf-8") as f:  # line-buffered - each write() hits disk promptly
        print(f"Capturing ALL lines to {out_path} for up to {args.duration:.0f}s "
              f"(Ctrl+C to stop early - already-written lines are safe either way). "
              f"Auto-reconnects if the bridge/FC power-cycles mid-capture.",
              file=sys.stderr)
        try:
            while time.time() < deadline:
                try:
                    sock = socket.create_connection((args.host, args.port), timeout=5.0)
                except OSError as exc:
                    print(f"connect failed ({exc}), retrying...", file=sys.stderr)
                    time.sleep(RECONNECT_DELAY_S)
                    continue
                print("connected", file=sys.stderr)
                sock.settimeout(1.0)
                buf = bytearray()
                last_data_ts = time.time()
                try:
                    while time.time() < deadline:
                        try:
                            chunk = sock.recv(4096)
                        except socket.timeout:
                            if (time.time() - last_data_ts) > STALE_CONN_S:
                                print("no data for a while - reconnecting "
                                      "(bridge/FC likely power-cycled)", file=sys.stderr)
                                break
                            continue
                        if not chunk:
                            print("connection closed by peer - reconnecting", file=sys.stderr)
                            break
                        last_data_ts = time.time()
                        buf.extend(chunk)
                        while b"\n" in buf:
                            idx = buf.index(b"\n")
                            line = bytes(buf[:idx]).decode(errors="replace").strip()
                            del buf[:idx + 1]
                            if not line:
                                continue
                            t = time.time()
                            f.write(f"{t:.3f} {line}\n")
                            n_lines += 1
                            if line.startswith("VEKF["):
                                n_vekf += 1
                finally:
                    sock.close()
                if time.time() < deadline:
                    time.sleep(RECONNECT_DELAY_S)
        except KeyboardInterrupt:
            print("stopped by user", file=sys.stderr)
    print(f"{n_lines} lines ({n_vekf} VEKF) saved to {out_path}", file=sys.stderr)


if __name__ == "__main__":
    main()
