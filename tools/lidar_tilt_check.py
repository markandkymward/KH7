"""Isolate whether raw LUNA (TF-Luna) + tilt compensation alone produces a flat
height reading, independent of VertEkf's own Kalman blending - the point being to
tell apart "the sensor/geometry/attitude chain is fine, the Kalman filter's own
predict/correct dynamics are what's still lagging during fast tilts" from "no,
even the raw geometry-only compensation is wrong."

Formula: tilt-compensated height = raw_luna_cm * cos(pitch_deg) * cos(roll_deg).
This matches Attitude_GetWorldUpInBodyFrame()'s gz for the ZYX convention this
codebase uses, and does NOT include the small lever-arm term VertEkf also applies
(that only matters by a few cm even at large tilt - see kh7_vertekf_tilt_divergence
memory) - deliberately the simplest possible correct-formula check.

Requires the roll sin/cos extraction bug fix and the AHRS LPF speedup already
flashed (see kh7_vertekf_tilt_divergence_2026_09_05 memory) - this script is
useless for isolating the Kalman-filter-specific question on a board without
those fixes, since the underlying pitch/roll telemetry itself would be suspect.

Usage:
    python tools/lidar_tilt_check.py 10.0.0.41                  # live, 60s, WiFi bridge
    python tools/lidar_tilt_check.py 10.0.0.41 --duration 30
    python tools/lidar_tilt_check.py --usb --port COM6           # live, USB serial
    python tools/lidar_tilt_check.py --replay tools/some_capture.txt   # re-analyze a saved capture
"""
import argparse
import math
import re
import socket
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

IMU_RE = re.compile(
    r"IMU\[x100/x10\]=\[(-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+)\]"
)
LUNA_RE = re.compile(r"\[BRIDGE\] LUNA cm=([\d.]+)")
RANGE_RE = re.compile(r"\[BRIDGE\] RANGE cm=([\d.]+)")
VEKF_RE = re.compile(
    r"VEKF\[healthy h_cm vz_cms bias_mm_s2 lidar_h_cm sonar_h_cm\]=\[(\d) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+)\]"
)


def capture_wifi(host: str, port: int, duration: float, out_path: str) -> None:
    sock = socket.create_connection((host, port), timeout=5.0)
    sock.settimeout(1.0)
    buf = bytearray()
    deadline = time.time() + duration
    with open(out_path, "w", buffering=1) as f:
        print(f"Capturing to {out_path} for {duration:.0f}s (Ctrl+C to stop early)...", file=sys.stderr)
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
                    if line:
                        f.write(f"{time.time():.3f} {line}\n")
        except KeyboardInterrupt:
            pass
    sock.close()


def capture_usb(port: str, baud: int, duration: float, out_path: str) -> None:
    import serial

    ser = serial.Serial(port, baud, timeout=1)
    deadline = time.time() + duration
    with open(out_path, "w", buffering=1) as f:
        print(f"Capturing to {out_path} for {duration:.0f}s (Ctrl+C to stop early)...", file=sys.stderr)
        try:
            while time.time() < deadline:
                line = ser.readline()
                if not line:
                    continue
                decoded = line.decode(errors="replace").strip()
                if decoded:
                    f.write(f"{time.time():.3f} {decoded}\n")
        except KeyboardInterrupt:
            pass
    ser.close()


def analyze(path: str, out_png: str) -> None:
    imu_t, pitch_v, roll_v = [], [], []
    luna_t, luna_v = [], []
    range_t, range_v = [], []
    vekf_t, vekf_h = [], []

    with open(path) as f:
        for line in f:
            parts = line.split(" ", 1)
            if len(parts) < 2:
                continue
            try:
                t = float(parts[0])
            except ValueError:
                continue
            m = IMU_RE.search(parts[1])
            if m:
                g = [int(x) for x in m.groups()]
                imu_t.append(t)
                pitch_v.append(g[6] / 10.0)
                roll_v.append(g[7] / 10.0)
                continue
            m = LUNA_RE.search(parts[1])
            if m:
                luna_t.append(t)
                luna_v.append(float(m.group(1)))
                continue
            m = RANGE_RE.search(parts[1])
            if m:
                range_t.append(t)
                range_v.append(float(m.group(1)))
                continue
            m = VEKF_RE.search(parts[1])
            if m:
                g = m.groups()
                vekf_t.append(t)
                vekf_h.append(int(g[1]))

    if not luna_t or not imu_t:
        print("ERROR: capture has no LUNA and/or IMU lines - nothing to analyze.", file=sys.stderr)
        sys.exit(1)

    import bisect

    def make_lookup(ts, vs):
        pairs = sorted(zip(ts, vs))
        tt = [p[0] for p in pairs]
        vv = [p[1] for p in pairs]

        def f(t):
            idx = bisect.bisect_left(tt, t)
            best = None
            for j in (idx - 1, idx):
                if 0 <= j < len(tt):
                    if best is None or abs(tt[j] - t) < abs(tt[best] - t):
                        best = j
            return vv[best] if best is not None else None

        return f

    pitch_at = make_lookup(imu_t, pitch_v)
    roll_at = make_lookup(imu_t, roll_v)
    vekf_at = make_lookup(vekf_t, vekf_h) if vekf_t else (lambda t: None)

    t0 = min(imu_t[0], luna_t[0])
    comp_t, comp_v, raw_t_rel, raw_v = [], [], [], []
    for t, raw_cm in zip(luna_t, luna_v):
        p = pitch_at(t)
        r = roll_at(t)
        if p is None or r is None:
            continue
        gz = math.cos(math.radians(p)) * math.cos(math.radians(r))
        if gz <= 0.05:
            continue
        comp_t.append(t - t0)
        comp_v.append(raw_cm * gz)
        raw_t_rel.append(t - t0)
        raw_v.append(raw_cm)

    fused_t_rel = [t - t0 for t in vekf_t]

    mean_v = sum(comp_v) / len(comp_v)
    dev = [abs(v - mean_v) for v in comp_v]
    dev.sort()
    n = len(dev)
    print(f"Tilt-compensated LUNA (raw*cos(pitch)*cos(roll)), n={n} samples:")
    print(f"  mean={mean_v:.1f}cm  mean_abs_dev={sum(dev)/n:.1f}cm  median_dev={dev[n//2]:.1f}cm"
          f"  p90_dev={dev[int(n*0.9)]:.1f}cm  max_dev={max(dev):.1f}cm")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 9), sharex=True)
    ax1.plot(raw_t_rel, raw_v, label="raw LUNA (cm, uncompensated)", color="tab:blue", alpha=0.4)
    ax1.plot(comp_t, comp_v, label="tilt-compensated LUNA (raw*cos(pitch)*cos(roll))", color="tab:green", linewidth=1.5)
    if fused_t_rel:
        ax1.plot(fused_t_rel, vekf_h, label="VertEkf fused h_cm (for comparison)", color="black", linewidth=1.5, alpha=0.8)
    ax1.axhline(mean_v, color="gray", linewidth=0.5, linestyle=":")
    ax1.set_ylabel("height (cm)")
    ax1.set_title("LIDAR tilt-compensation check: does raw*cos(pitch)*cos(roll) alone read flat?")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    pitch_t_rel = [t - t0 for t in imu_t]
    ax2.plot(pitch_t_rel, pitch_v, label="pitch (deg)", color="tab:orange")
    ax2.plot(pitch_t_rel, roll_v, label="roll (deg)", color="tab:purple")
    ax2.axhline(0, color="gray", linewidth=0.5)
    ax2.set_ylabel("attitude (deg)")
    ax2.set_xlabel("time (s)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(out_png, dpi=120)
    print(f"saved {out_png}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("host", nargs="?", help="ESP32 bridge IP/hostname (WiFi mode)")
    parser.add_argument("--port", default=None, help="TCP port (WiFi) or serial port (USB)")
    parser.add_argument("--usb", action="store_true", help="capture over USB serial instead of WiFi")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--out", default=None, help="raw capture file path")
    parser.add_argument("--png", default=None, help="output plot path")
    parser.add_argument("--replay", default=None, help="re-analyze an existing capture file instead of capturing")
    args = parser.parse_args()

    if args.replay:
        path = args.replay
    else:
        ts = time.strftime("%Y%m%d_%H%M%S")
        path = args.out or f"tools/lidar_tilt_check_{ts}.txt"
        if args.usb:
            capture_usb(args.port or "COM6", args.baud, args.duration, path)
        else:
            if not args.host:
                print("ERROR: host required unless --usb or --replay is given.", file=sys.stderr)
                sys.exit(1)
            capture_wifi(args.host, int(args.port) if args.port else 3333, args.duration, path)

    png = args.png or (path.rsplit(".", 1)[0] + ".png")
    analyze(path, png)


if __name__ == "__main__":
    main()
