#!/usr/bin/env python3
"""Pull the SD-card flight data logger from KH7 and visualize it for troubleshooting.

The firmware records a compact 32-byte sample (setpoint, gyro, PID output,
motor outputs, battery, flight mode) at ~125Hz to the SD card for every arm-
to-disarm cycle, appending after previous flights so nothing is overwritten.
This tool pulls the whole log over serial ("SDLOG DUMP"), splits it back into
individual flights, and renders a troubleshooting dashboard per flight:
  - setpoint vs. gyro (tracking), per axis (roll/pitch/yaw), and PID output (all axes, one graph)
  - all four motor outputs, and baro altitude/climb rate, over time
  - gyro noise/vibration spectrum (FFT), per axis

Usage:
    python tools/sdlog_analyze.py --list                  # list flights on the card (full history)
    python tools/sdlog_analyze.py                         # pull + analyze just the most recent flight (WiFi)
    python tools/sdlog_analyze.py --usb                     # pull over USB serial instead of WiFi
    python tools/sdlog_analyze.py --full-dump --flight 0   # pull the whole card to see an older flight
    python tools/sdlog_analyze.py --full-dump --all       # one dashboard PNG per flight on the card
    python tools/sdlog_analyze.py --save-raw log.txt        # archive the raw dump for later
    python tools/sdlog_analyze.py --save-raw               # same, auto-named sdlog_raw_<timestamp>.txt
    python tools/sdlog_analyze.py --raw-in log.txt          # re-analyze a saved dump, no board needed

Board must be disarmed - "SDLOG DUMP"/"SDLOG DUMP LAST" is refused while armed (motors spinning).
By default only the latest flight is pulled (fast) over the ESP32 WiFi bridge (kh7bridge.local);
pass --usb for wired serial, or --full-dump to stream the entire card history instead (slow -
it grows every arm and is never trimmed).
"""
import argparse
import datetime
import os
import re
import socket
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

RECORD_STRUCT = struct.Struct(
    "<I9hHHHHBBhhhhhhhhHhH"  # original fields (54 bytes)
    "B"                       # mag_field_dev_pct
    "BBBB"                    # nav_flags, nav_invalid_reason, nav_fix_type, nav_num_sv
    "HHHHH"                   # nav_h_acc_cm, nav_age_ms, nav_update_period_ms, nav_consecutive_valid, nav_dropout_count
    + "h" * 16                # nav north/east/vel(raw,filt,desired,error)/accel(n,e,fwd,right) + pilot stick, all int16
)
RECORD_SIZE = RECORD_STRUCT.size  # 101 bytes, matches App_SdLogRecord_t in Core/Src/app.c
FIELD_NAMES = (
    "time_ms",
    "setpoint_roll_dps", "setpoint_pitch_dps", "setpoint_yaw_dps",
    "gyro_roll_dps_x10", "gyro_pitch_dps_x10", "gyro_yaw_dps_x10",
    "pid_roll_us", "pid_pitch_us", "pid_yaw_us",
    "motor_fl_us", "motor_fr_us", "motor_rr_us", "motor_rl_us",
    "battery_decivolts", "flags",
    "pitch_deg_x10", "roll_deg_x10",
    "target_pitch_deg_x10", "target_roll_deg_x10",
    "baro_alt_cm", "baro_vz_cms",
    "throttle_cmd_us", "throttle_actual_us",
    "arm_us",
    "yaw_deg_x10", "mag_heading_x10",
    "mag_field_dev_pct",
    "nav_flags", "nav_invalid_reason", "nav_fix_type", "nav_num_sv",
    "nav_h_acc_cm", "nav_age_ms", "nav_update_period_ms", "nav_consecutive_valid", "nav_dropout_count",
    "nav_north_m_x10", "nav_east_m_x10",
    "nav_raw_vel_n_x100", "nav_raw_vel_e_x100",
    "nav_filt_vel_n_x100", "nav_filt_vel_e_x100",
    "nav_desired_vel_n_x100", "nav_desired_vel_e_x100",
    "nav_vel_error_n_x100", "nav_vel_error_e_x100",
    "nav_accel_cmd_n_x1000", "nav_accel_cmd_e_x1000",
    "nav_accel_cmd_fwd_x1000", "nav_accel_cmd_right_x1000",
    "pilot_roll_stick_us", "pilot_pitch_stick_us",
)
APP_SDLOG_FLAG_LINK_ACTIVE = 0x08  # bit set when receiver_state.link_active was true
APP_SDLOG_FLAG_MAG_HEALTHY = 0x10  # bit set when Mag_IsHealthy() was true
APP_SDLOG_FLAG_MAG_NUDGE_GATED = 0x20  # bit set when the field-deviation interference gate suppressed the nudge this sample
APP_SDLOG_NAV_FLAG_REQUESTED = 0x01
APP_SDLOG_NAV_FLAG_ACTIVE = 0x02
APP_SDLOG_NAV_FLAG_TILT_LIMITED = 0x04
APP_SDLOG_NAV_FLAG_ACCEL_LIMITED = 0x08
APP_SDLOG_NAV_FLAG_NEW_SAMPLE = 0x10
APP_SDLOG_NAV_FLAG_REJECTED = 0x20
APP_SDLOG_NAV_FLAG_REF_VALID = 0x40
NAV_INVALID_REASON_NAMES = {
    0: "NONE", 1: "NO_3D_FIX", 2: "GPS_STALE", 3: "BAD_UPDATE_INTERVAL",
    4: "POOR_ACCURACY", 5: "VELOCITY_INVALID", 6: "POSITION_JUMP", 7: "VELOCITY_JUMP",
    8: "REACQUIRING", 9: "NO_REFERENCE", 10: "NONFINITE",
}
FLIGHT_MODE_NAMES = {0: "RATE", 1: "ATTITUDE", 2: "ALTHOLD", 3: "NAVBRAKE"}
FLIGHT_GAP_MS = 1500  # a stored-sample time gap bigger than this means a new flight
PID_TERM_LIMIT_US = 320  # APP_RATE_TERM_LIMIT_US in Core/Src/app.c - used for saturation %
LANDING_CLIP_S = 1.0  # trailing seconds always trimmed - landing/disarm produces a large
                      # non-representative tracking-error/PID spike right before the log ends
LIPO_NOMINAL_CELL_V = 3.7  # standard LiPo nominal-per-cell voltage, used to infer pack S-count


class _TcpLineSource:
    """Wraps the ESP32 WiFi bridge's raw TCP socket (see
    tools/esp32_s3_uart6_wifi_bridge) with the same readline()/write()/
    reset_input_buffer() surface collect_raw() already uses for pyserial, so
    the dump loop below doesn't need to care which transport it's using.
    """

    def __init__(self, host: str, tcp_port: int, timeout_s: float = 1.0):
        self._sock = socket.create_connection((host, tcp_port), timeout=5.0)
        self._sock.settimeout(timeout_s)
        self._buf = bytearray()

    def reset_input_buffer(self) -> None:
        self._buf.clear()

    def write(self, data: bytes) -> None:
        self._sock.sendall(data)

    def readline(self) -> bytes:
        while b"\n" not in self._buf:
            try:
                chunk = self._sock.recv(4096)
            except socket.timeout:
                return b""
            if not chunk:
                return b""
            self._buf.extend(chunk)
        idx = self._buf.index(b"\n") + 1
        line = bytes(self._buf[:idx])
        del self._buf[:idx]
        return line

    def close(self) -> None:
        self._sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()


CONNECT_RETRIES = 6          # matches the manual retry loops this tool used to require by hand
CONNECT_RETRY_DELAY_S = 4.0
RESUME_ATTEMPTS = 8          # cap on stall->resume cycles before giving up, so a genuinely dead
                              # board/bridge fails loudly instead of looping forever


def _open_connection(port: str, baud: int, host: Optional[str], tcp_port: int):
    """(Re)connects with retry/backoff - the WiFi bridge in particular has proven to drop
    for 10s of seconds to over a minute at a time, previously requiring a human to notice
    and manually re-run a probe loop before every real attempt (2026-08-21)."""
    last_exc: Optional[Exception] = None
    for attempt in range(1, CONNECT_RETRIES + 1):
        try:
            if host:
                return _TcpLineSource(host, tcp_port)
            return serial.Serial(port, baud, timeout=1.0)
        except (OSError, serial.SerialException) as exc:
            last_exc = exc
            print(f"  connect attempt {attempt}/{CONNECT_RETRIES} failed ({exc}), "
                  f"retrying in {CONNECT_RETRY_DELAY_S:.0f}s...", file=sys.stderr)
            if attempt < CONNECT_RETRIES:
                time.sleep(CONNECT_RETRY_DELAY_S)
    raise RuntimeError(f"Could not connect after {CONNECT_RETRIES} attempts: {last_exc}")


def _dump_session(conn, command: bytes, idle_timeout_s: float, deadline: float,
                   blocks: dict, err_blocks: set) -> Tuple[bool, Optional[int]]:
    """Runs one SDLOG DUMP/DUMP FROM session against an already-open connection, filling
    `blocks` (block_num -> hex str) and `err_blocks` in place. Returns (completed, total_blocks) -
    completed=True only if DUMP[END] was actually seen (a real stall/timeout returns False even
    if every block happened to arrive, since END is what confirms the board agrees it's done)."""
    with conn as ser:
        ser.reset_input_buffer()
        ser.write(command)

        started = False
        total_blocks = None
        last_progress = time.time()
        last_report = time.time()

        while time.time() < deadline:
            raw = ser.readline()
            if not raw:
                if started and (time.time() - last_progress) > idle_timeout_s:
                    print("Warning: no data for a while, stopping early.", file=sys.stderr)
                    return False, total_blocks
                continue

            line = raw.decode(errors="replace").strip()
            if not line:
                continue

            if DUMP_BUSY_RE.match(line):
                raise RuntimeError("Board reports armed - disarm before running SDLOG DUMP.")
            if DUMP_EMPTY_RE.match(line):
                raise RuntimeError("No flights logged on the SD card yet.")

            start_match = DUMP_START_RE.match(line)
            if start_match:
                started = True
                last_progress = time.time()
                total_blocks = int(start_match.group(2)) - int(start_match.group(1)) + 1
                print(line)
                continue

            block_match = BLOCK_RE.match(line)
            if block_match:
                blocks[int(block_match.group(1))] = block_match.group(2)
                last_progress = time.time()
                if (time.time() - last_report) > 2.0:
                    last_report = time.time()
                    if total_blocks:
                        print(f"  ...{len(blocks)}/{total_blocks} blocks", file=sys.stderr)
                    else:
                        print(f"  ...{len(blocks)} blocks", file=sys.stderr)
                continue

            err_match = DUMP_ERR_RE.match(line)
            if err_match:
                err_blocks.add(int(err_match.group(1)))
                last_progress = time.time()
                continue

            if DUMP_END_RE.match(line):
                return True, total_blocks

        if not started:
            raise RuntimeError("No SDLOG_DUMP response - is the board connected and the "
                                "firmware SD card logger built in?")
    return False, total_blocks


def collect_raw(port: str, baud: int, idle_timeout_s: float, max_timeout_s: float,
                 save_raw_path: Optional[str], full: bool = False,
                 host: Optional[str] = None, tcp_port: int = 3333) -> str:
    # LAST only streams the most recent flight (fast) - the card's full history
    # only grows over time and is rarely what you actually want to re-pull.
    first_command = b"SDLOG DUMP\r\n" if full else b"SDLOG DUMP LAST\r\n"
    blocks: dict = {}
    err_blocks: set = set()
    deadline = time.time() + max_timeout_s
    command = first_command
    completed = False
    total_blocks = None

    for resume_attempt in range(1, RESUME_ATTEMPTS + 1):
        conn = _open_connection(port, baud, host, tcp_port)
        completed, total_blocks = _dump_session(conn, command, idle_timeout_s, deadline,
                                                 blocks, err_blocks)
        if completed:
            break
        if time.time() >= deadline:
            print("Warning: overall timeout reached, giving up on resuming further.",
                  file=sys.stderr)
            break
        # Resume from just past the highest block actually received - a stall never skips
        # blocks out of order, so this is safe even though a handful of blocks from an
        # earlier failed attempt could in principle still be missing from the low end.
        resume_block = (max(blocks) + 1) if blocks else 1
        print(f"Stalled after {len(blocks)} blocks - resuming from block {resume_block} "
              f"(attempt {resume_attempt + 1}/{RESUME_ATTEMPTS})...", file=sys.stderr)
        command = f"SDLOG DUMP FROM {resume_block}\r\n".encode()

    if not completed:
        print(f"Warning: dump did not complete cleanly after {RESUME_ATTEMPTS} attempts "
              f"({len(blocks)}/{total_blocks or '?'} blocks) - working with what was received.",
              file=sys.stderr)

    lines: List[str] = [f"SDLOG[{b}]={blocks[b]}" for b in sorted(blocks)]
    lines.extend(f"SDLOG_DUMP[READ_ERR block={b}]" for b in sorted(err_blocks))
    if completed:
        lines.append("SDLOG_DUMP[END]")

    text = "\n".join(lines)
    if save_raw_path:
        with open(save_raw_path, "w", encoding="ascii") as f:
            f.write(text)
        print(f"Saved raw dump: {save_raw_path}")
    return text


def erase_sdlog(port: str, baud: int, host: Optional[str], tcp_port: int) -> None:
    """Sends SDLOG ERASE after a successful pull so the next dump only has to
    fetch the one new flight instead of re-downloading the whole accumulated
    history from block 1 every time (found 2026-08-21 - dumps were getting
    slower every flight because nothing was ever clearing the card). The
    firmware already refuses this while armed (see app.c APP_SDLOGCMD_ERASE),
    so this is safe to fire automatically."""
    conn = _TcpLineSource(host, tcp_port) if host else serial.Serial(port, baud, timeout=1.0)
    with conn as ser:
        ser.reset_input_buffer()
        ser.write(b"SDLOG ERASE\r\n")
        deadline = time.time() + 5.0
        while time.time() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if line == "SDLOG_ERASE[OK]":
                print("SD card erased for next flight.")
                return
            if line == "SDLOG_ERASE[FAIL armed]":
                print("Warning: SD erase skipped - board reports armed.", file=sys.stderr)
                return
    print("Warning: no SDLOG_ERASE response - card may not have been erased.", file=sys.stderr)


def parse_records(text: str) -> List[dict]:
    blocks = {}
    err_blocks = []
    bad_blocks = []
    for line in text.splitlines():
        m = BLOCK_RE.match(line)
        if m:
            block_idx = int(m.group(1))
            try:
                blocks[block_idx] = bytes.fromhex(m.group(2))
            except ValueError:
                bad_blocks.append(block_idx)
            continue
        m = DUMP_ERR_RE.match(line)
        if m:
            err_blocks.append(int(m.group(1)))

    if err_blocks:
        print(f"Warning: {len(err_blocks)} block(s) failed to read: {err_blocks}", file=sys.stderr)
    if bad_blocks:
        print(f"Warning: {len(bad_blocks)} block(s) had corrupt/truncated hex and were skipped: {bad_blocks}",
              file=sys.stderr)
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
    a["pitch_deg"] = a["pitch_deg_x10"] / 10.0
    a["roll_deg"] = a["roll_deg_x10"] / 10.0
    a["yaw_deg"] = a["yaw_deg_x10"] / 10.0
    a["mag_heading_deg"] = a["mag_heading_x10"] / 10.0
    a["target_pitch_deg"] = a["target_pitch_deg_x10"] / 10.0
    a["target_roll_deg"] = a["target_roll_deg_x10"] / 10.0
    a["baro_alt_m"] = a["baro_alt_cm"] / 100.0
    a["baro_vz_mps"] = a["baro_vz_cms"] / 100.0
    a["battery_v"] = a["battery_decivolts"] / 10.0
    a["nav_h_acc_m"] = a["nav_h_acc_cm"] / 100.0
    a["nav_north_m"] = a["nav_north_m_x10"] / 10.0
    a["nav_east_m"] = a["nav_east_m_x10"] / 10.0
    for suffix in ("raw_vel_n", "raw_vel_e", "filt_vel_n", "filt_vel_e",
                   "desired_vel_n", "desired_vel_e", "vel_error_n", "vel_error_e"):
        a[f"nav_{suffix}_mps"] = a[f"nav_{suffix}_x100"] / 100.0
    for suffix in ("accel_cmd_n", "accel_cmd_e", "accel_cmd_fwd", "accel_cmd_right"):
        a[f"nav_{suffix}_mps2"] = a[f"nav_{suffix}_x1000"] / 1000.0
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


LIPO_MAX_CHARGE_CELL_V = 4.25  # full-charge ceiling per cell (4.20V + margin)


def infer_nominal_battery_v(measured_v: float) -> float:
    """Infer nominal pack voltage from a measured voltage: the smallest LiPo cell
    count whose full-charge ceiling (4.25V/cell) could explain the reading - keeps
    the dashboard label correct automatically as packs change (2S/3S/4S/6S/...).
    Plain round(measured_v / 3.7) misclassifies fresh packs: a full 4S reads ~16.8V,
    which rounds to "5S" (16.8/3.7 = 4.54) instead of the correct 4S."""
    for cells in range(1, 13):
        if measured_v <= (cells * LIPO_MAX_CHARGE_CELL_V):
            return cells * LIPO_NOMINAL_CELL_V
    return round(measured_v / LIPO_NOMINAL_CELL_V) * LIPO_NOMINAL_CELL_V


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
    print(f"attitude estimate: pitch min={a['pitch_deg'].min():.1f} max={a['pitch_deg'].max():.1f}deg  "
          f"roll min={a['roll_deg'].min():.1f} max={a['roll_deg'].max():.1f}deg")

    mode_bits = ((a["flags"].astype(int) >> 1) & 0x03)
    in_attitude = (mode_bits == 1) | (mode_bits == 2) | (mode_bits == 3)
    if np.any(in_attitude):
        print(f"\nATTITUDE/ALTHOLD/NAVBRAKE mode angle tracking ({int(in_attitude.sum())} samples):")
        for axis in ("roll", "pitch"):
            cmd = a[f"target_{axis}_deg"][in_attitude]
            act = a[f"{axis}_deg"][in_attitude]
            err = cmd - act
            print(f"  {axis:5s} commanded min={cmd.min():6.1f} max={cmd.max():6.1f}deg  "
                  f"actual min={act.min():6.1f} max={act.max():6.1f}deg  "
                  f"err rms={np.sqrt(np.mean(err ** 2)):5.2f} peak={np.max(np.abs(err)):5.2f}deg")

    for motor, label in (("motor_fl_us", "FL"), ("motor_fr_us", "FR"),
                         ("motor_rr_us", "RR"), ("motor_rl_us", "RL")):
        print(f"motor {label}: min={a[motor].min():.0f} max={a[motor].max():.0f} "
              f"mean={a[motor].mean():.0f}us")

    if np.any(a["baro_alt_m"] != 0.0) or np.any(a["baro_vz_mps"] != 0.0):
        vz = a["baro_vz_mps"]
        print(f"\nbaro: alt min={a['baro_alt_m'].min():.2f} max={a['baro_alt_m'].max():.2f}m  "
              f"climb rate min={vz.min():.2f} max={vz.max():.2f} rms={np.sqrt(np.mean(vz ** 2)):.2f}m/s")

    link_active = (a["flags"].astype(int) & APP_SDLOG_FLAG_LINK_ACTIVE) != 0
    link_drops = int(np.sum(~link_active))
    print(f"\narm channel: min={a['arm_us'].min():.0f} max={a['arm_us'].max():.0f}us  "
          f"link_active samples={int(link_active.sum())}/{n} (drops={link_drops})")

    setpoint_yaw = a["setpoint_yaw_dps"]
    yaw_stick_centered = np.abs(setpoint_yaw) < 5.0  # near-zero yaw stick -> pilot holding heading
    print(f"yaw stick: rms={np.sqrt(np.mean(setpoint_yaw ** 2)):.1f} max={np.max(np.abs(setpoint_yaw)):.1f}dps  "
          f"centered(<5dps)={100.0 * np.mean(yaw_stick_centered):.0f}% of flight")

    mag_healthy = (a["flags"].astype(int) & APP_SDLOG_FLAG_MAG_HEALTHY) != 0
    if np.any(mag_healthy):
        # Both are wrapped/relative signals - compare ROTATION SINCE THE FIRST HEALTHY
        # SAMPLE (not absolute heading vs absolute yaw, which aren't on the same
        # reference/zero) to see whether the compass-nudge is keeping AHRS yaw from
        # drifting away from what the compass independently says it should be.
        # Both mag_heading_deg and yaw_deg are wrapped to a fixed range by the
        # firmware, so a plain difference produces a spurious ~360deg jump every
        # time either one crosses its wrap boundary - unwrap first so a real
        # multi-turn rotation (or none at all) doesn't look like a huge divergence.
        first_idx = int(np.argmax(mag_healthy))
        mag_unwrapped = np.unwrap(a["mag_heading_deg"], period=360.0)
        yaw_unwrapped = np.unwrap(a["yaw_deg"], period=360.0)
        # mag_heading_deg and yaw_deg have OPPOSITE native sign conventions
        # (bench-verified 2026-08-21 with real MAG+IMU telemetry - a physical
        # CW rotation increases mag_heading_deg but decreases yaw_deg). The
        # firmware's own mag-yaw nudge negates its internal mag_delta_deg to
        # account for this (see app.c) - negate here too so this comparison
        # matches reality instead of reporting a near-maximal, opposite-sign
        # "divergence" that isn't real.
        mag_delta = -(mag_unwrapped - mag_unwrapped[first_idx])
        yaw_delta = yaw_unwrapped - yaw_unwrapped[first_idx]
        err_all = mag_delta - yaw_delta
        err = err_all[mag_healthy]
        print(f"compass vs yaw drift: mag_delta rms={np.sqrt(np.mean(mag_delta[mag_healthy] ** 2)):.1f}deg  "
              f"yaw_delta rms={np.sqrt(np.mean(yaw_delta[mag_healthy] ** 2)):.1f}deg  "
              f"tracking err rms={np.sqrt(np.mean(err ** 2)):.1f} peak={np.max(np.abs(err)):.1f}deg  "
              f"mag_healthy={int(mag_healthy.sum())}/{n}")

        # Split by whether the pilot was actively commanding yaw at the time -
        # a drift/error while the stick is centered (trying to hold heading) is a
        # real AHRS/compass-tracking concern; error while actively yawing is more
        # likely just the compass-nudge's slow gain lagging a real, intended turn.
        mask_centered = mag_healthy & yaw_stick_centered
        mask_active = mag_healthy & ~yaw_stick_centered
        if np.any(mask_centered):
            err_c = err_all[mask_centered]
            print(f"  while holding heading (stick centered, {int(mask_centered.sum())} samples): "
                  f"tracking err rms={np.sqrt(np.mean(err_c ** 2)):.1f} peak={np.max(np.abs(err_c)):.1f}deg")
        if np.any(mask_active):
            err_a = err_all[mask_active]
            print(f"  while commanding yaw (stick active, {int(mask_active.sum())} samples): "
                  f"tracking err rms={np.sqrt(np.mean(err_a ** 2)):.1f} peak={np.max(np.abs(err_a)):.1f}deg")

        nudge_gated = (a["flags"].astype(int) & APP_SDLOG_FLAG_MAG_NUDGE_GATED) != 0
        dev_pct = a["mag_field_dev_pct"]
        print(f"  mag field deviation from boot ref: mean={dev_pct.mean():.1f}% max={dev_pct.max():.0f}%  "
              f"nudge gated (interference) {100.0 * np.mean(nudge_gated):.0f}% of samples "
              f"({int(nudge_gated.sum())}/{n})")

    nav_flags = a["nav_flags"].astype(int)
    nav_requested = (nav_flags & APP_SDLOG_NAV_FLAG_REQUESTED) != 0
    nav_active = (nav_flags & APP_SDLOG_NAV_FLAG_ACTIVE) != 0
    if np.any(nav_requested) or np.any(a["nav_fix_type"] > 0):
        reasons = a["nav_invalid_reason"].astype(int)
        print(f"\nGPS nav: fix_type min={a['nav_fix_type'].min():.0f} max={a['nav_fix_type'].max():.0f}  "
              f"sats min={a['nav_num_sv'].min():.0f} max={a['nav_num_sv'].max():.0f}  "
              f"hAcc min={a['nav_h_acc_m'].min():.1f} max={a['nav_h_acc_m'].max():.1f}m  "
              f"dropout_count max={int(a['nav_dropout_count'].max())}")
        print(f"  NAVBRAKE requested={int(nav_requested.sum())}/{n} active={int(nav_active.sum())}/{n} "
              f"tilt_limited={int(np.sum((nav_flags & APP_SDLOG_NAV_FLAG_TILT_LIMITED) != 0))} "
              f"accel_limited={int(np.sum((nav_flags & APP_SDLOG_NAV_FLAG_ACCEL_LIMITED) != 0))}")
        if np.any(nav_active):
            print(f"  while active: north min={a['nav_north_m'][nav_active].min():.1f} max={a['nav_north_m'][nav_active].max():.1f}m  "
                  f"east min={a['nav_east_m'][nav_active].min():.1f} max={a['nav_east_m'][nav_active].max():.1f}m  "
                  f"filt_vel_n rms={np.sqrt(np.mean(a['nav_filt_vel_n_mps'][nav_active] ** 2)):.2f} "
                  f"filt_vel_e rms={np.sqrt(np.mean(a['nav_filt_vel_e_mps'][nav_active] ** 2)):.2f} m/s")
        invalid = reasons[~nav_active & nav_requested]
        if invalid.size:
            uniq, counts = np.unique(invalid, return_counts=True)
            reason_str = ", ".join(f"{NAV_INVALID_REASON_NAMES.get(int(u), u)}={int(c)}" for u, c in zip(uniq, counts))
            print(f"  invalid-while-requested reasons: {reason_str}")


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
    fig = plt.figure(figsize=(16, 21))
    gs = fig.add_gridspec(6, 6, hspace=0.4, wspace=0.5)

    track_axes = {}
    for row, axis in enumerate(("roll", "pitch", "yaw")):
        ax_track = fig.add_subplot(gs[row, 0:3], sharex=track_axes.get("roll"))
        ax_track.plot(t, a[f"setpoint_{axis}_dps"], label="setpoint", linewidth=0.8)
        ax_track.plot(t, a[f"gyro_{axis}_dps"], label="gyro", linewidth=0.8, alpha=0.85)
        ax_track.set_ylabel(f"{axis}\ndps")
        ax_track.legend(loc="upper right", fontsize=7)
        track_axes.setdefault("roll", ax_track)

    ax_throttle = fig.add_subplot(gs[0:3, 3:6], sharex=track_axes["roll"])
    ax_throttle.plot(t, a["throttle_cmd_us"], label="commanded", linewidth=0.8)
    ax_throttle.plot(t, a["throttle_actual_us"], label="actual", linewidth=0.8, alpha=0.85)
    ax_throttle.set_ylabel("throttle\nus")
    ax_throttle.legend(loc="upper right", fontsize=8)

    ax_motors = fig.add_subplot(gs[3, 0:3], sharex=track_axes["roll"])
    for motor, label in (("motor_fl_us", "FL"), ("motor_fr_us", "FR"),
                         ("motor_rr_us", "RR"), ("motor_rl_us", "RL")):
        ax_motors.plot(t, a[motor], label=label, linewidth=0.7)
    ax_motors.set_ylabel("motors\nus")
    ax_motors.legend(loc="upper right", fontsize=7, ncol=4)

    ax_baro = fig.add_subplot(gs[3, 3:6], sharex=track_axes["roll"])
    ax_baro.plot(t, a["baro_alt_m"], color="tab:blue", linewidth=0.8, label="altitude")
    ax_baro.set_ylabel("baro alt\nm")
    ax_baro_vz = ax_baro.twinx()
    ax_baro_vz.plot(t, a["baro_vz_mps"], color="tab:orange", linewidth=0.7, alpha=0.8, label="climb rate")
    ax_baro_vz.set_ylabel("climb rate\nm/s")
    lines1, labels1 = ax_baro.get_legend_handles_labels()
    lines2, labels2 = ax_baro_vz.get_legend_handles_labels()
    ax_baro.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=7)

    angle_axes = []
    for col, axis in enumerate(("roll", "pitch")):
        ax_angle = fig.add_subplot(gs[4, col * 3:col * 3 + 3], sharex=track_axes["roll"])
        ax_angle.plot(t, a[f"target_{axis}_deg"], label="commanded", linewidth=0.8)
        ax_angle.plot(t, a[f"{axis}_deg"], label="actual", linewidth=0.8, alpha=0.85)
        ax_angle.set_ylabel(f"{axis} angle\ndeg")
        ax_angle.legend(loc="upper right", fontsize=7)
        angle_axes.append(ax_angle)

    freqs = np.fft.rfftfreq(len(t), d=1.0 / a["fs_hz"])
    for col, axis in enumerate(("roll", "pitch", "yaw")):
        ax_fft = fig.add_subplot(gs[5, col * 2:col * 2 + 2])
        sig = a[f"gyro_{axis}_dps"] - a[f"gyro_{axis}_dps"].mean()
        spectrum = np.abs(np.fft.rfft(sig)) / max(len(t), 1)
        ax_fft.plot(freqs, spectrum, linewidth=0.8)
        ax_fft.set_xlim(0, min(a["fs_hz"] / 2, 250))
        ax_fft.set_xlabel("Hz")
        ax_fft.set_title(f"{axis} vibration", fontsize=9)

    ax_motors.set_xlabel("time (s)")
    ax_baro.set_xlabel("time (s)")
    for ax_angle in angle_axes:
        ax_angle.set_xlabel("time (s)")

    nominal_battery_v = infer_nominal_battery_v(a["battery_v"].max())
    fig.suptitle(f"KH7 SD flight log - {t[-1]:.1f}s, ~{a['fs_hz']:.0f}Hz  |  nominal battery: {nominal_battery_v:.1f}V",
                 fontsize=24)
    fig.savefig(out_path, dpi=130)
    print(f"Saved dashboard: {os.path.abspath(out_path)}")
    return fig, interactive


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="COM6", help="serial port, only used with --usb")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--host", default="kh7bridge.local",
                        help="ESP32 WiFi bridge hostname/IP - this is the default transport; "
                             "use --usb to pull over USB serial (--port) instead")
    parser.add_argument("--tcp-port", type=int, default=3333, help="WiFi bridge TCP port")
    parser.add_argument("--usb", action="store_true",
                         help="pull over USB serial (--port) instead of the WiFi bridge")
    parser.add_argument("--out", default="sdlog", help="output PNG prefix")
    parser.add_argument("--flight", type=int, default=-1,
                         help="flight index to analyze (default: most recent, -1)")
    parser.add_argument("--all", action="store_true", help="render every flight, not just one")
    parser.add_argument("--full-dump", action="store_true",
                         help="pull the whole card history instead of just the latest flight")
    parser.add_argument("--list", action="store_true", help="list flights and exit, no plots")
    parser.add_argument("--raw-in", help="parse a previously saved raw dump instead of the board")
    parser.add_argument("--save-raw", nargs="?", const="",
                         help="save the raw dump text to this file (default: auto-named sdlog_raw_<timestamp>.txt)")
    parser.add_argument("--idle-timeout", type=float, default=8.0)
    parser.add_argument("--max-timeout", type=float, default=900.0)
    parser.add_argument("--no-open", action="store_true", help="don't display the plot window(s)")
    parser.add_argument("--no-erase", action="store_true",
                         help="skip the automatic SDLOG ERASE after a successful pull "
                              "(default: erase, so the next dump doesn't re-fetch the whole card)")
    args = parser.parse_args()

    if args.raw_in:
        with open(args.raw_in, "r", encoding="ascii") as f:
            text = f.read()
    else:
        # Anything other than "just the latest flight" (listing, --all, or an
        # explicit older --flight index) needs the full card history to satisfy.
        need_full = args.full_dump or args.list or args.all or (args.flight >= 0)
        host = None if args.usb else args.host
        save_raw_path = args.save_raw
        if save_raw_path == "":
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            save_raw_path = f"sdlog_raw_{stamp}.txt"
        text = collect_raw(args.port, args.baud, args.idle_timeout, args.max_timeout, save_raw_path,
                            full=need_full, host=host, tcp_port=args.tcp_port)

    records = parse_records(text)
    flights = segment_flights(records)

    if not args.raw_in and not args.no_erase:
        erase_sdlog(args.port, args.baud, host, args.tcp_port)

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
