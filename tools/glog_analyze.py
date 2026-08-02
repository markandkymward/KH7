#!/usr/bin/env python3
"""Collect a GLOG gyro capture from KH7 over serial and analyze it for vibration.

Usage:
    python tools/glog_analyze.py [--port COM6] [--baud 115200] [--out glog]

Sends "GLOG DUMP", parses the printed GLOG[...] lines, then reports per-axis
RMS/peak-to-peak noise and the dominant vibration frequency (FFT), and saves
a time-domain + spectrum plot as <out>.png.
"""
import argparse
import re
import sys
import time

import numpy as np
import serial

START_RE = re.compile(r"GLOG\[START count=(\d+) rate_hz=(\d+)\]")
SAMPLE_RE = re.compile(r"GLOG\[(\d+)\]=\[(-?\d+)\s+(-?\d+)\s+(-?\d+)\]")
BUSY_RE = re.compile(r"GLOG\[BUSY armed\]")
END_RE = re.compile(r"GLOG\[END\]")


def collect(port: str, baud: int, timeout_s: float) -> tuple[int, np.ndarray]:
    with serial.Serial(port, baud, timeout=1.0) as ser:
        ser.reset_input_buffer()
        ser.write(b"GLOG DUMP\r\n")

        rate_hz = None
        expected_count = None
        samples = []
        deadline = time.time() + timeout_s

        while time.time() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if not line:
                continue

            if BUSY_RE.match(line):
                raise RuntimeError("Board reports armed - disarm before running GLOG DUMP.")

            m = START_RE.match(line)
            if m:
                expected_count = int(m.group(1))
                rate_hz = int(m.group(2))
                print(f"GLOG start: count={expected_count} rate_hz={rate_hz}")
                continue

            m = SAMPLE_RE.match(line)
            if m:
                samples.append((int(m.group(2)) / 10.0,
                                 int(m.group(3)) / 10.0,
                                 int(m.group(4)) / 10.0))
                continue

            if END_RE.match(line):
                break

        if rate_hz is None or not samples:
            raise RuntimeError("No GLOG data received - is the board connected, disarmed, "
                                "and has it been armed at least once since boot?")
        if expected_count is not None and len(samples) != expected_count:
            print(f"Warning: expected {expected_count} samples, got {len(samples)}", file=sys.stderr)

        return rate_hz, np.array(samples, dtype=float)


def analyze(rate_hz: int, data: np.ndarray, out_prefix: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n = data.shape[0]
    t = np.arange(n) / rate_hz
    axis_names = ("roll (gx)", "pitch (gy)", "yaw (gz)")

    freqs = np.fft.rfftfreq(n, d=1.0 / rate_hz)
    fig, axes = plt.subplots(3, 2, figsize=(11, 8), sharex="col")

    print(f"\n{n} samples @ {rate_hz} Hz ({n / rate_hz:.2f} s)\n")
    print(f"{'axis':12s} {'rms dps':>9s} {'p2p dps':>9s} {'peak freq':>10s} {'peak mag':>9s}")

    for axis in range(3):
        sig = data[:, axis]
        rms = float(np.sqrt(np.mean(sig ** 2)))
        p2p = float(sig.max() - sig.min())

        spectrum = np.abs(np.fft.rfft(sig - sig.mean())) / n
        # Skip the first few bins (~<5Hz) so slow stick/attitude drift doesn't win as "vibration".
        min_bin = max(1, int(5.0 / (rate_hz / n)))
        peak_idx = min_bin + int(np.argmax(spectrum[min_bin:]))
        peak_freq = freqs[peak_idx]
        peak_mag = spectrum[peak_idx]

        print(f"{axis_names[axis]:12s} {rms:9.2f} {p2p:9.2f} {peak_freq:9.1f}Hz {peak_mag:9.3f}")

        axes[axis][0].plot(t, sig, linewidth=0.6)
        axes[axis][0].set_ylabel(f"{axis_names[axis]}\ndps")
        axes[axis][1].plot(freqs, spectrum, linewidth=0.8)
        axes[axis][1].axvline(peak_freq, color="r", linestyle="--", linewidth=0.8)
        axes[axis][1].set_xlim(0, min(rate_hz / 2, 250))

    axes[0][0].set_title("Time domain")
    axes[0][1].set_title("FFT magnitude")
    axes[2][0].set_xlabel("time (s)")
    axes[2][1].set_xlabel("frequency (Hz)")
    fig.tight_layout()

    png_path = f"{out_prefix}.png"
    fig.savefig(png_path, dpi=130)
    print(f"\nSaved plot: {png_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=15.0, help="seconds to wait for the full dump")
    parser.add_argument("--out", default="glog")
    args = parser.parse_args()

    rate_hz, data = collect(args.port, args.baud, args.timeout)
    analyze(rate_hz, data, args.out)


if __name__ == "__main__":
    main()
