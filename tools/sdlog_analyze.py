#!/usr/bin/env python3
"""Pull the SD-card flight data logger from KH7 and visualize it for troubleshooting.

The firmware records a compact 32-byte sample (setpoint, gyro, PID output,
motor outputs, battery, flight mode) at ~125Hz to the SD card for every arm-
to-disarm cycle, appending after previous flights so nothing is overwritten.
This tool pulls the whole log over serial ("SDLOG DUMP"), splits it back into
individual flights, and renders a troubleshooting dashboard per flight:
  - setpoint vs. gyro (tracking) and PID output, per axis (roll/pitch/yaw)
  - all four motor outputs and battery voltage over time
  - gyro noise/vibration spectrum (FFT), per axis

Usage:
    python tools/sdlog_analyze.py --list                  # list flights on the card
    python tools/sdlog_analyze.py                         # analyze the most recent flight
    python tools/sdlog_analyze.py --flight 0               # analyze a specific flight
    python tools/sdlog_analyze.py --all                    # one dashboard PNG per flight
    python tools/sdlog_analyze.py --save-raw log.txt        # archive the raw dump for later
    python tools/sdlog_analyze.py --raw-in log.txt          # re-analyze a saved dump, no board needed

Board must be disarmed - "SDLOG DUMP" is refused while armed (motors spinning).
"""
import argparse
import os
import re
import struct
import sys
import time
from typing import List, Optional, Tuple

import numpy as np
import serial

DUMP_START_RE = re.compile(r"SDLOG_DUMP\[START first=(\d+) last=(\d+) record_bytes=(\d+)\]")
DUMP_BUSY_RE = re.compile(r"SDLOG_DUMP\[BUSY armed\]")
DUMP_EMPTY_RE = re.compile(r"SDLOG_DUMP\[EMPTY\]")
DUMP_END_RE = re.compile(r"SDLOG_DUMP\[END\]")
DUMP_ERR_RE = re.compile(r"SDLOG_DUMP\[READ_ERR block=(\d+)\]")
BLOCK_RE = re.compile(r"SDLOG\[(\d+)\]=([0-9A-Fa-f]+)")

# Anchor default output to the script's own folder so the PNG always lands in a
# predictable place regardless of the caller's current working directory.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

RECORD_STRUCT = struct.Struct("<I9hHHHHBB")
RECORD_SIZE = RECORD_STRUCT.size  # 32 bytes, matches App_SdLogRecord_t in Core/Src/app.c
FIELD_NAMES = (
    "time_ms",
    "setpoint_roll_dps", "setpoint_pitch_dps", "setpoint_yaw_dps",
    "gyro_roll_dps_x10", "gyro_pitch_dps_x10", "gyro_yaw_dps_x10",
    "pid_roll_us", "pid_pitch_us", "pid_yaw_us",
    "motor_fl_us", "motor_fr_us", "motor_rr_us", "motor_rl_us",
    "battery_decivolts", "flags",
)
FLIGHT_MODE_NAMES = {0: "RATE", 1: "ATTITUDE"}
FLIGHT_GAP_MS = 1500  # a stored-sample time gap bigger than this means a new flight
PID_TERM_LIMIT_US = 320  # APP_RATE_TERM_LIMIT_US in Core/Src/app.c - used for saturation %
LANDING_CLIP_S = 1.0  # trailing seconds always trimmed - landing/disarm produces a large
                      # non-representative tracking-error/PID spike right before the log ends


def collect_raw(port: str, baud: int, idle_timeout_s: float, max_timeout_s: float,
                 save_raw_path: Optional[str]) -> str:
    with serial.Serial(port, baud, timeout=1.0) as ser:
        ser.reset_input_buffer()
        ser.write(b"SDLOG DUMP\r\n")

        lines: List[str] = []
        started = False
        last_progress = time.time()
        deadline = time.time() + max_timeout_s

        while time.time() < deadline:
            raw = ser.readline()
            if not raw:
                if started and (time.time() - last_progress) > idle_timeout_s:
                    print("Warning: no data for a while, stopping early.", file=sys.stderr)
                    break
                continue

            line = raw.decode(errors="replace").strip()
            if not line:
                continue

            if DUMP_BUSY_RE.match(line):
                raise RuntimeError("Board reports armed - disarm before running SDLOG DUMP.")
            if DUMP_EMPTY_RE.match(line):
                raise RuntimeError("No flights logged on the SD card yet.")

            if DUMP_START_RE.match(line):
                started = True
                last_progress = time.time()
                print(line)
                continue

            if BLOCK_RE.match(line) or DUMP_ERR_RE.match(line):
                lines.append(line)
                last_progress = time.time()
                continue

            if DUMP_END_RE.match(line):
                lines.append(line)
                break

        if not started:
            raise RuntimeError("No SDLOG_DUMP response - is the board connected and the "
                                "firmware SD card logger built in?")

    text = "\n".join(lines)
    if save_raw_path:
        with open(save_raw_path, "w", encoding="ascii") as f:
            f.write(text)
        print(f"Saved raw dump: {save_raw_path}")
    return text


def parse_records(text: str) -> List[dict]:
    blocks = {}
    err_blocks = []
    for line in text.splitlines():
        m = BLOCK_RE.match(line)
        if m:
            block_idx = int(m.group(1))
            blocks[block_idx] = bytes.fromhex(m.group(2))
            continue
        m = DUMP_ERR_RE.match(line)
        if m:
            err_blocks.append(int(m.group(1)))

    if err_blocks:
        print(f"Warning: {len(err_blocks)} block(s) failed to read: {err_blocks}", file=sys.stderr)
    if not blocks:
        raise RuntimeError("No SDLOG[...] block lines found in the dump.")

    records = []
    for block_idx in sorted(blocks):
        data = blocks[block_idx]
        for offset in range(0, len(data) - RECORD_SIZE + 1, RECORD_SIZE):
            chunk = data[offset:offset + RECORD_SIZE]
            if chunk == bytes(RECORD_SIZE):
                continue  # zero-padded tail of a flight's last (partial) block
            values = RECORD_STRUCT.unpack(chunk)
            records.append(dict(zip(FIELD_NAMES, values)))

    if not records:
        raise RuntimeError("Dump contained blocks but no non-padding records were decoded.")
    return records


def segment_flights(records: List[dict]) -> List[Tuple[int, int]]:
    flights = []
    start = 0
    for i in range(1, len(records)):
        dt = records[i]["time_ms"] - records[i - 1]["time_ms"]
        if dt < 0 or dt > FLIGHT_GAP_MS:
            flights.append((start, i))
            start = i
    flights.append((start, len(records)))
    return flights


def list_flights(records: List[dict], flights: List[Tuple[int, int]]) -> None:
    print(f"\n{len(flights)} flight(s), {len(records)} total samples\n")
    print(f"{'#':>3s} {'samples':>8s} {'duration':>9s} {'mode':>10s}")
    for i, (start, end) in enumerate(flights):
        sub = records[start:end]
        duration_s = (sub[-1]["time_ms"] - sub[0]["time_ms"]) / 1000.0
        modes = {FLIGHT_MODE_NAMES.get((r["flags"] >> 1) & 0x03, "?") for r in sub}
        print(f"{i:3d} {len(sub):8d} {duration_s:8.2f}s {'/'.join(sorted(modes)):>10s}")


def _flight_arrays(sub: List[dict]) -> dict:
    a = {name: np.array([r[name] for r in sub], dtype=float) for name in FIELD_NAMES}
    a["t_s"] = (a["time_ms"] - a["time_ms"][0]) / 1000.0
    for axis in ("roll", "pitch", "yaw"):
        a[f"gyro_{axis}_dps"] = a[f"gyro_{axis}_dps_x10"] / 10.0
    a["battery_v"] = a["battery_decivolts"] / 10.0
    dt = np.diff(a["time_ms"])
    dt = dt[dt > 0]
    a["fs_hz"] = 1000.0 / np.median(dt) if dt.size else 125.0

    if a["t_s"][-1] > (2.0 * LANDING_CLIP_S):
        keep = a["t_s"] <= (a["t_s"][-1] - LANDING_CLIP_S)
        n_before = keep.shape[0]
        for key, val in a.items():
            if isinstance(val, np.ndarray) and val.shape[0] == n_before:
                a[key] = val[keep]
    return a


def print_summary(a: dict) -> None:
    n = len(a["t_s"])
    duration = a["t_s"][-1] if n else 0.0
    print(f"\n{n} samples, {duration:.2f}s @ ~{a['fs_hz']:.0f}Hz effective rate\n")
    print(f"{'axis':8s} {'rms err':>9s} {'peak err':>9s} {'sat %':>7s} {'peak vib':>12s}")
    for axis in ("roll", "pitch", "yaw"):
        err = a[f"setpoint_{axis}_dps"] - a[f"gyro_{axis}_dps"]
        pid = a[f"pid_{axis}_us"]
        sat_pct = 100.0 * np.mean(np.abs(pid) >= (0.95 * PID_TERM_LIMIT_US))

        sig = a[f"gyro_{axis}_dps"] - a[f"gyro_{axis}_dps"].mean()
        spectrum = np.abs(np.fft.rfft(sig)) / max(n, 1)
        freqs = np.fft.rfftfreq(n, d=1.0 / a["fs_hz"])
        min_bin = max(1, int(5.0 / (a["fs_hz"] / max(n, 1))))
        if spectrum.size > min_bin:
            peak_idx = min_bin + int(np.argmax(spectrum[min_bin:]))
            peak_str = f"{freqs[peak_idx]:.1f}Hz/{spectrum[peak_idx]:.2f}"
        else:
            peak_str = "-"

        print(f"{axis:8s} {np.sqrt(np.mean(err ** 2)):9.2f} {np.max(np.abs(err)):9.2f} "
              f"{sat_pct:6.1f}% {peak_str:>12s}")

    print(f"\nbattery: start={a['battery_v'][0]:.2f}V end={a['battery_v'][-1]:.2f}V "
          f"min={a['battery_v'].min():.2f}V sag={a['battery_v'][0] - a['battery_v'].min():.2f}V")
    for motor, label in (("motor_fl_us", "FL"), ("motor_fr_us", "FR"),
                         ("motor_rr_us", "RR"), ("motor_rl_us", "RL")):
        print(f"motor {label}: min={a[motor].min():.0f} max={a[motor].max():.0f} "
              f"mean={a[motor].mean():.0f}us")


_PYPLOT_CACHE: dict = {}


def _get_pyplot():
    """Import pyplot with a real GUI backend when one is available, falling back to
    the headless Agg backend (savefig only, no on-screen window) otherwise. Cached
    so the backend probe only happens once even when rendering multiple flights."""
    if not _PYPLOT_CACHE:
        import matplotlib
        try:
            import matplotlib.pyplot as plt
            plt.figure()
            plt.close("all")
            _PYPLOT_CACHE["plt"] = plt
            _PYPLOT_CACHE["interactive"] = True
        except Exception:
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            _PYPLOT_CACHE["plt"] = plt
            _PYPLOT_CACHE["interactive"] = False
    return _PYPLOT_CACHE["plt"], _PYPLOT_CACHE["interactive"]


def plot_dashboard(a: dict, out_path: str):
    plt, interactive = _get_pyplot()

    t = a["t_s"]
    fig = plt.figure(figsize=(16, 18))
    gs = fig.add_gridspec(5, 6, hspace=0.4, wspace=0.5)

    track_axes = {}
    for row, axis in enumerate(("roll", "pitch", "yaw")):
        ax_track = fig.add_subplot(gs[row, 0:3], sharex=track_axes.get("roll"))
        ax_track.plot(t, a[f"setpoint_{axis}_dps"], label="setpoint", linewidth=0.8)
        ax_track.plot(t, a[f"gyro_{axis}_dps"], label="gyro", linewidth=0.8, alpha=0.85)
        ax_track.set_ylabel(f"{axis}\ndps")
        ax_track.legend(loc="upper right", fontsize=7)
        track_axes.setdefault("roll", ax_track)

        ax_pid = fig.add_subplot(gs[row, 3:6], sharex=track_axes["roll"])
        ax_pid.plot(t, a[f"pid_{axis}_us"], color="tab:red", linewidth=0.8)
        ax_pid.axhline(PID_TERM_LIMIT_US, color="gray", linestyle="--", linewidth=0.6)
        ax_pid.axhline(-PID_TERM_LIMIT_US, color="gray", linestyle="--", linewidth=0.6)
        ax_pid.set_ylabel(f"{axis} PID\nus")

    ax_motors = fig.add_subplot(gs[3, 0:3], sharex=track_axes["roll"])
    for motor, label in (("motor_fl_us", "FL"), ("motor_fr_us", "FR"),
                         ("motor_rr_us", "RR"), ("motor_rl_us", "RL")):
        ax_motors.plot(t, a[motor], label=label, linewidth=0.7)
    ax_motors.set_ylabel("motors\nus")
    ax_motors.legend(loc="upper right", fontsize=7, ncol=4)

    ax_batt = fig.add_subplot(gs[3, 3:6], sharex=track_axes["roll"])
    ax_batt.plot(t, a["battery_v"], color="tab:green", linewidth=0.8)
    ax_batt.set_ylabel("battery\nV")

    freqs = np.fft.rfftfreq(len(t), d=1.0 / a["fs_hz"])
    for col, axis in enumerate(("roll", "pitch", "yaw")):
        ax_fft = fig.add_subplot(gs[4, col * 2:col * 2 + 2])
        sig = a[f"gyro_{axis}_dps"] - a[f"gyro_{axis}_dps"].mean()
        spectrum = np.abs(np.fft.rfft(sig)) / max(len(t), 1)
        ax_fft.plot(freqs, spectrum, linewidth=0.8)
        ax_fft.set_xlim(0, min(a["fs_hz"] / 2, 250))
        ax_fft.set_xlabel("Hz")
        ax_fft.set_title(f"{axis} vibration", fontsize=9)

    ax_yaw_pid = fig.axes[5]
    ax_yaw_pid.set_xlabel("time (s)")
    ax_motors.set_xlabel("time (s)")
    ax_batt.set_xlabel("time (s)")

    fig.suptitle(f"KH7 SD flight log - {t[-1]:.1f}s, ~{a['fs_hz']:.0f}Hz", fontsize=13)
    fig.savefig(out_path, dpi=130)
    print(f"Saved dashboard: {os.path.abspath(out_path)}")
    return fig, interactive


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--out", default="sdlog", help="output PNG prefix")
    parser.add_argument("--flight", type=int, default=-1,
                         help="flight index to analyze (default: most recent, -1)")
    parser.add_argument("--all", action="store_true", help="render every flight, not just one")
    parser.add_argument("--list", action="store_true", help="list flights and exit, no plots")
    parser.add_argument("--raw-in", help="parse a previously saved raw dump instead of the board")
    parser.add_argument("--save-raw", help="save the raw dump text to this file")
    parser.add_argument("--idle-timeout", type=float, default=8.0)
    parser.add_argument("--max-timeout", type=float, default=900.0)
    parser.add_argument("--no-open", action="store_true", help="don't display the plot window(s)")
    args = parser.parse_args()

    if args.raw_in:
        with open(args.raw_in, "r", encoding="ascii") as f:
            text = f.read()
    else:
        text = collect_raw(args.port, args.baud, args.idle_timeout, args.max_timeout, args.save_raw)

    records = parse_records(text)
    flights = segment_flights(records)

    if args.list:
        list_flights(records, flights)
        return

    targets = range(len(flights)) if args.all else [args.flight]
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    figs = []
    for idx in targets:
        try:
            start, end = flights[idx]
        except IndexError:
            print(f"Flight index {idx} out of range (0..{len(flights) - 1}).", file=sys.stderr)
            continue
        real_idx = idx if idx >= 0 else len(flights) + idx
        a = _flight_arrays(records[start:end])
        print(f"\n=== Flight {real_idx} ===")
        print_summary(a)
        out_name = (f"{args.out}_{timestamp}.png" if not args.all
                    else f"{args.out}_{timestamp}_flight{real_idx}.png")
        out_path = out_name if os.path.isabs(out_name) else os.path.join(SCRIPT_DIR, out_name)
        fig, interactive = plot_dashboard(a, out_path)
        figs.append(fig)

    if figs and not args.no_open:
        if _PYPLOT_CACHE.get("interactive"):
            _PYPLOT_CACHE["plt"].show()
        else:
            print("No GUI backend available - PNG(s) saved but not displayed.", file=sys.stderr)
    for fig in figs:
        _PYPLOT_CACHE["plt"].close(fig)


if __name__ == "__main__":
    main()
