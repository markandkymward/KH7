#!/usr/bin/env python3
"""Analyze a live telemetry capture (from capture_liftoff_telemetry.py) of a real
liftoff flown WITHOUT ALTHOLD, specifically to see the real onboard vert_ekf.c
behavior (real 500Hz-domain accel, real ESP32 confidence, real pre-LPF baro) during
liftoff - the things the earlier SD-log offline replay had to approximate.
"""
import argparse
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

VEKF_RE = re.compile(r"VEKF\[healthy h_cm vz_cms bias_mm_s2 lidar_h_cm sonar_h_cm\]=\[(\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+)\]")
ALTHOLD_RE = re.compile(
    r"ALTHOLD\[hold auth target_cm fused_cm setpt_cms err_cms trim_us damp_us hover_us\]="
    r"\[(\d+) (\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+)\]"
)
BARO_RE = re.compile(r"BARO\[healthy cm cm_s\]=\[(\d+) (-?\d+) (-?\d+)\]")
IMU_RE = re.compile(r"IMU\[x100/x10\]=\[(-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+)\]")
RANGE_RE = re.compile(r"\[BRIDGE\] RANGE cm=([\d.]+) age_ms=(\d+)")
LUNA_RE = re.compile(r"\[BRIDGE\] LUNA cm=([\d.]+)")
ARM_RE = re.compile(r"ARM\[a=(\d) sw=(\d)")


def load(path):
    lines = open(path, encoding="utf-8", errors="replace").readlines()
    t0 = float(lines[0].split(maxsplit=1)[0])

    vekf_t, vekf_h, vekf_vz, vekf_bias, vekf_lidar, vekf_sonar = [], [], [], [], [], []
    baro_t, baro_h, baro_alt, baro_vz = [], [], [], []
    imu_t, imu_ax, imu_ay, imu_az, imu_pitch, imu_roll = [], [], [], [], [], []
    range_t, range_cm = [], []
    luna_t, luna_cm = [], []
    arm_t, arm_a = [], []
    ah_t, ah_hold, ah_auth, ah_target, ah_fused, ah_setpt, ah_err, ah_trim, ah_damp, ah_hover = (
        [], [], [], [], [], [], [], [], [], []
    )

    for line in lines:
        parts = line.split(maxsplit=1)
        if len(parts) < 2:
            continue
        t = float(parts[0]) - t0
        rest = parts[1]

        m = VEKF_RE.search(rest)
        if m:
            vekf_t.append(t)
            vekf_h.append(int(m.group(2)) / 100.0)
            vekf_vz.append(int(m.group(3)) / 100.0)
            vekf_bias.append(int(m.group(4)) / 1000.0)
            vekf_lidar.append(int(m.group(5)) / 100.0)
            vekf_sonar.append(int(m.group(6)) / 100.0)
            continue
        m = ALTHOLD_RE.search(rest)
        if m:
            ah_t.append(t)
            ah_hold.append(int(m.group(1)))
            ah_auth.append(int(m.group(2)))
            ah_target.append(int(m.group(3)) / 100.0)
            ah_fused.append(int(m.group(4)) / 100.0)
            ah_setpt.append(int(m.group(5)) / 100.0)
            ah_err.append(int(m.group(6)) / 100.0)
            ah_trim.append(int(m.group(7)))
            ah_damp.append(int(m.group(8)))
            ah_hover.append(int(m.group(9)))
            continue
        m = BARO_RE.search(rest)
        if m:
            baro_t.append(t)
            baro_h.append(int(m.group(1)))
            baro_alt.append(int(m.group(2)) / 100.0)
            baro_vz.append(int(m.group(3)) / 100.0)
            continue
        m = IMU_RE.search(rest)
        if m:
            imu_t.append(t)
            imu_ax.append(int(m.group(1)) / 100.0)
            imu_ay.append(int(m.group(2)) / 100.0)
            imu_az.append(int(m.group(3)) / 100.0)
            imu_pitch.append(int(m.group(7)) / 10.0)
            imu_roll.append(int(m.group(8)) / 10.0)
            continue
        m = RANGE_RE.search(rest)
        if m:
            range_t.append(t)
            range_cm.append(float(m.group(1)))
            continue
        m = LUNA_RE.search(rest)
        if m:
            luna_t.append(t)
            luna_cm.append(float(m.group(1)))
            continue
        m = ARM_RE.search(rest)
        if m:
            arm_t.append(t)
            arm_a.append(int(m.group(1)))
            continue

    return dict(
        vekf=(np.array(vekf_t), np.array(vekf_h), np.array(vekf_vz), np.array(vekf_bias),
              np.array(vekf_lidar), np.array(vekf_sonar)),
        baro=(np.array(baro_t), np.array(baro_h), np.array(baro_alt), np.array(baro_vz)),
        imu=(np.array(imu_t), np.array(imu_ax), np.array(imu_ay), np.array(imu_az),
             np.array(imu_pitch), np.array(imu_roll)),
        range_=(np.array(range_t), np.array(range_cm)),
        luna=(np.array(luna_t), np.array(luna_cm)),
        arm=(np.array(arm_t), np.array(arm_a)),
        althold=(np.array(ah_t), np.array(ah_hold), np.array(ah_auth), np.array(ah_target),
                 np.array(ah_fused), np.array(ah_setpt), np.array(ah_err), np.array(ah_trim),
                 np.array(ah_damp), np.array(ah_hover)),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture_file")
    ap.add_argument("--out", default="tools/liftoff_capture_analysis.png")
    args = ap.parse_args()

    d = load(args.capture_file)
    vekf_t, vekf_h, vekf_vz, vekf_bias, vekf_lidar, vekf_sonar = d["vekf"]
    baro_t, baro_healthy, baro_alt, baro_vz = d["baro"]
    imu_t, imu_ax, imu_ay, imu_az, imu_pitch, imu_roll = d["imu"]
    range_t, range_cm = d["range_"]
    luna_t, luna_cm = d["luna"]
    arm_t, arm_a = d["arm"]
    ah_t, ah_hold, ah_auth, ah_target, ah_fused, ah_setpt, ah_err, ah_trim, ah_damp, ah_hover = d["althold"]

    armed_idx = np.where(arm_a == 1)[0]
    if len(armed_idx):
        arm_on_t = arm_t[armed_idx[0]]
        arm_off_t = arm_t[armed_idx[-1]]
        print(f"Armed window: t={arm_on_t:.2f}s .. t={arm_off_t:.2f}s ({arm_off_t-arm_on_t:.1f}s)")
    else:
        arm_on_t = arm_off_t = None
        print("No armed samples found")

    have_althold = len(ah_t) > 0
    n_rows = 6 if have_althold else 4
    fig, axes = plt.subplots(n_rows, 1, figsize=(13, 3.5 * n_rows), sharex=True)

    ax = axes[0]
    ax.plot(baro_t, baro_alt, label="baro_alt (raw telemetry)", color="0.6", lw=1)
    ax.plot(vekf_t, vekf_h, label="VertEkf height (REAL onboard EKF)", color="C0", lw=1.5)
    rmask = range_cm > 0
    lmask = luna_cm > 0
    ax.scatter(range_t[rmask], range_cm[rmask] / 100.0, s=4, color="C1", alpha=0.4, label="sonar raw range (m, uncompensated)")
    ax.scatter(luna_t[lmask], luna_cm[lmask] / 100.0, s=4, color="C2", alpha=0.4, label="lidar raw range (m, uncompensated)")
    ax.scatter(vekf_t, vekf_sonar, s=6, color="C1", label="sonar implied (tilt+arm, from EKF)")
    ax.scatter(vekf_t, vekf_lidar, s=6, color="C2", label="lidar implied (tilt+arm, from EKF)")
    if arm_on_t is not None:
        ax.axvspan(arm_on_t, arm_off_t, color="green", alpha=0.08, label="armed")
    ax.set_ylabel("height (m)")
    ax.legend(loc="upper right", fontsize=7)
    ax.set_title("Real liftoff capture (ATTITUDE mode, no ALTHOLD) - " + args.capture_file)

    ax = axes[1]
    ax.plot(baro_t, baro_vz, label="baro_vz (raw telemetry)", color="0.6", lw=1)
    ax.plot(vekf_t, vekf_vz, label="VertEkf vz (REAL onboard EKF)", color="C0", lw=1.5)
    ax.set_ylabel("climb rate (m/s)")
    ax.legend(loc="upper right", fontsize=7)

    ax = axes[2]
    ax.plot(vekf_t, vekf_bias, color="C3", label="VertEkf accel_bias (m/s^2)")
    ax.set_ylabel("accel bias (m/s^2)")
    ax.legend(loc="upper right", fontsize=7)

    ax = axes[3]
    ax.plot(imu_t, imu_az, label="az (g, body frame, REAL raw accel)", color="C4", lw=0.8)
    ax.plot(imu_t, imu_pitch, label="pitch (deg)", color="C5", lw=0.6, alpha=0.7)
    ax.plot(imu_t, imu_roll, label="roll (deg)", color="C6", lw=0.6, alpha=0.7)
    ax.set_ylabel("az (g) / attitude (deg)")
    if not have_althold:
        ax.set_xlabel("time (s)")
    ax.legend(loc="upper right", fontsize=7)

    if have_althold:
        # Root-cause view for the ALTHOLD altitude-variance investigation - see
        # Telemetry_PrintAltholdState()'s comment in telemetry.c for what each
        # field means and why it was added.
        ax = axes[4]
        ax.plot(ah_t, ah_target, label="held target (m) - should be FLAT while holding", color="C0", lw=1.3)
        ax.plot(ah_t, ah_fused, label="fused height (m)", color="0.5", lw=1)
        hold_mask = ah_hold == 1
        ax.fill_between(ah_t, 0, 1, where=hold_mask, transform=ax.get_xaxis_transform(),
                        color="purple", alpha=0.06, label="holding=1")
        ax.set_ylabel("height (m)")
        ax.legend(loc="upper right", fontsize=7)
        ax.set_title("ALTHOLD internal state")

        ax = axes[5]
        ax.plot(ah_t, ah_setpt, label="climb-rate setpoint (m/s) - output of position P-term", color="C0", lw=1)
        ax.plot(ah_t, ah_err, label="climb-rate error (m/s) - setpoint minus Baro_GetClimbRateMps()", color="C3", lw=1)
        ax.set_ylabel("m/s")
        ax.legend(loc="upper right", fontsize=7)
        ax2 = ax.twinx()
        ax2.plot(ah_t, ah_trim, label="trim_us", color="C1", lw=0.7, alpha=0.6)
        ax2.plot(ah_t, ah_damp, label="damp_us", color="C2", lw=0.7, alpha=0.6)
        ax2.plot(ah_t, ah_hover, label="hover_throttle_us", color="C4", lw=0.7, alpha=0.6)
        ax2.set_ylabel("us")
        lines1, labels1 = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=7)
        ax.set_xlabel("time (s)")

    fig.tight_layout()
    fig.savefig(args.out, dpi=130)
    print(f"Saved {args.out}")

    if arm_on_t is not None:
        window = (vekf_t >= arm_on_t - 1.0) & (vekf_t <= arm_on_t + 4.0)
        print(f"\nVertEkf height near liftoff (t=arm{'+' if True else ''}, arm_on={arm_on_t:.2f}s):")
        for t, h, sh, lh in zip(vekf_t[window], vekf_h[window], vekf_sonar[window], vekf_lidar[window]):
            print(f"  t={t-arm_on_t:+.2f}s  h={h:+.3f}  sonar_implied={sh:+.3f}  lidar_implied={lh:+.3f}")


if __name__ == "__main__":
    main()
