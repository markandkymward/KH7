#!/usr/bin/env python3
"""Offline replay of Core/Src/vert_ekf.c against a logged flight, to sanity-check
the untuned EKF constants before ever considering wiring it into a control path.

FIDELITY GAPS (the SD log predates several of vert_ekf.c's real inputs - flagged
here rather than silently approximated):
  - No raw accelerometer is logged, so VertEkf_Predict()'s accel_up_mps2 input is
    approximated as the finite-difference derivative of the logged (already
    complementary-filtered) baro_vz_cms, not a true IMU-rate accel sample. This
    will not reproduce the real firmware's ~2ms predict-rate dynamics faithfully;
    predicts here run at the SD log's own sample rate (~35ms), not 500Hz.
  - No ESP32 confidence score is logged (added to the wire format after this log
    was captured) - every valid range sample is replayed with confidence=1.0
    (best case), so R inflation from low-confidence readings is NOT exercised.
  - The logged baro_alt_cm is Baro_GetAltitudeM() (5Hz-LPF'd), not the new
    Baro_GetRawAltitudeM() the real firmware feeds the EKF - this replay's baro
    input is smoother/more lagged than what vert_ekf.c actually sees in flight.
  - GPS is not exercised (this flight was not NAV_VELOCITY_BRAKE, so real GPS
    parsing was inactive; VertEkf_UpdateGps() is not called here either).

This is a behavioral sanity check of the predict/update/R-scheduling/cross-
validation LOGIC and the physical reasonableness of the untuned constants, not a
bit-exact reproduction of onboard behavior.
"""
import argparse
import math
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, "tools")
import sdlog_analyze as sd

# ---- Mirrors Core/Src/vert_ekf.c constants exactly (2026-08-29) ----
LIDAR_ARM_X_M = 0.0990
LIDAR_ARM_Y_M = -0.0990
SONAR_ARM_X_M = -0.0566
SONAR_ARM_Y_M = -0.0566

ROTOR_DIAMETER_M = 0.127
GROUND_EFFECT_ZONE_M = 2.0 * ROTOR_DIAMETER_M
GROUND_EFFECT_THRUST_REF_US = 350.0
BARO_GROUND_EFFECT_R_GAIN = 20.0

BARO_R_BASE_M2 = 0.06 * 0.06
LIDAR_R_BASE_M2 = 0.03 * 0.03
SONAR_R_BASE_M2 = 0.02 * 0.02
CONFIDENCE_FLOOR = 0.05

BARO_CROSSCHECK_SIGMA_MULT = 3.0
BARO_CROSSCHECK_R_PENALTY = 8.0
BARO_POST_RESET_SETTLE_MS = 2500
BARO_POST_RESET_R_GAIN = 40.0

SONAR_MAX_RANGE_M = 3.0
SONAR_FADE_START_M = 2.3
SONAR_FADE_R_GAIN = 25.0
SONAR_TILT_REJECT_GZ_MIN = 0.9537

LIDAR_MAX_RANGE_M = 8.0
LIDAR_FADE_START_M = 7.0
LIDAR_FADE_R_GAIN = 20.0

CROSSCHECK_MAX_M = 3.0
CROSSCHECK_SIGMA_MULT = 2.0
CROSSCHECK_MAX_AGE_MS = 150
CROSSCHECK_R_PENALTY = 8.0

ACCEL_NOISE_MPS2 = 0.5
BIAS_RW_MPS2_PER_S2 = 0.0004

INIT_P_HEIGHT_M2 = 1.0
INIT_P_VEL_M2S2 = 1.0
INIT_P_BIAS_M2S4 = 0.01

APP_MOTOR_IDLE_US = 1080.0


def cross_check_penalty(disagreement, combined_sigma, sigma_mult, base_penalty):
    """Severity-scaled cross-check R penalty - mirrors vert_ekf.c's
    VertEkf_CrossCheckPenalty(), added after a real ALTHOLD-on-EKF flight showed a
    27-sigma sonar/lidar disagreement getting the same flat 8x penalty as a
    borderline 2-sigma one, leaving R far too small to stop a bad reading from
    pulling the fused height and triggering a visible throttle correction."""
    if combined_sigma <= 0.0:
        return base_penalty
    sigma_ratio = disagreement / combined_sigma
    excess_ratio = sigma_ratio / sigma_mult
    return base_penalty * excess_ratio * excess_ratio


class VertEkf:
    def __init__(self):
        self.x = np.zeros(3)  # [height_m, vz_mps, accel_bias_mps2]
        self.P = np.diag([INIT_P_HEIGHT_M2, INIT_P_VEL_M2S2, INIT_P_BIAS_M2S4])
        self.lidar_last_h = 0.0
        self.lidar_last_r = 0.0
        self.lidar_last_ms = 0
        self.sonar_last_h = 0.0
        self.sonar_last_r = 0.0
        self.sonar_last_ms = 0
        self.reset_ms = 0

    def predict(self, accel_up_mps2, dt_s):
        dt_s = max(dt_s, 0.0001)
        accel_corrected = accel_up_mps2 - self.x[2]

        self.x[0] += (self.x[1] * dt_s) + (0.5 * accel_corrected * dt_s * dt_s)
        self.x[1] += accel_corrected * dt_s

        dt2 = dt_s * dt_s
        dt3 = dt2 * dt_s
        F = np.array([[1.0, dt_s, -0.5 * dt2],
                      [0.0, 1.0, -dt_s],
                      [0.0, 0.0, 1.0]])
        self.P = F @ self.P @ F.T

        sigma_a2 = ACCEL_NOISE_MPS2 * ACCEL_NOISE_MPS2
        self.P[0, 0] += sigma_a2 * dt3 / 3.0
        self.P[0, 1] += sigma_a2 * dt2 / 2.0
        self.P[1, 0] += sigma_a2 * dt2 / 2.0
        self.P[1, 1] += sigma_a2 * dt_s
        self.P[2, 2] += BIAS_RW_MPS2_PER_S2 * dt_s

    def _scalar_update(self, z, R):
        if R <= 0.0:
            R = 1e-6
        innovation = z - self.x[0]
        S = self.P[0, 0] + R
        if S <= 0.0:
            return
        K = self.P[:, 0] / S
        self.x += K * innovation
        self.P -= np.outer(K, self.P[0, :])

    def update_baro(self, raw_alt_m, baro_healthy, motor_power_delta_us, now_ms):
        if not baro_healthy:
            return
        thrust_factor = np.clip(motor_power_delta_us / GROUND_EFFECT_THRUST_REF_US, 0.0, 1.0)
        height_factor = np.clip(1.0 - (self.x[0] / GROUND_EFFECT_ZONE_M), 0.0, 1.0)
        severity = thrust_factor * height_factor
        r = BARO_R_BASE_M2 * (1.0 + (BARO_GROUND_EFFECT_R_GAIN * severity))

        have_lidar = self.lidar_last_ms != 0 and (now_ms - self.lidar_last_ms) < CROSSCHECK_MAX_AGE_MS
        have_sonar = self.sonar_last_ms != 0 and (now_ms - self.sonar_last_ms) < CROSSCHECK_MAX_AGE_MS
        ref_height = ref_r = None
        if have_lidar and have_sonar:
            if self.lidar_last_ms >= self.sonar_last_ms:
                ref_height, ref_r = self.lidar_last_h, self.lidar_last_r
            else:
                ref_height, ref_r = self.sonar_last_h, self.sonar_last_r
        elif have_lidar:
            ref_height, ref_r = self.lidar_last_h, self.lidar_last_r
        elif have_sonar:
            ref_height, ref_r = self.sonar_last_h, self.sonar_last_r

        if ref_height is not None:
            combined_sigma = math.sqrt(r + ref_r)
            disagreement = abs(raw_alt_m - ref_height)
            if disagreement > (BARO_CROSSCHECK_SIGMA_MULT * combined_sigma):
                r *= cross_check_penalty(disagreement, combined_sigma,
                                          BARO_CROSSCHECK_SIGMA_MULT, BARO_CROSSCHECK_R_PENALTY)

        elapsed_ms = now_ms - self.reset_ms
        if elapsed_ms < BARO_POST_RESET_SETTLE_MS:
            frac = elapsed_ms / BARO_POST_RESET_SETTLE_MS
            settle_shape = 0.5 + (0.5 * math.cos(frac * math.pi))
            r *= (1.0 + (BARO_POST_RESET_R_GAIN * settle_shape))

        self._scalar_update(raw_alt_m, r)

    def update_range(self, raw_range_cm, confidence, is_lidar, now_ms, gx, gy, gz):
        if raw_range_cm <= 0.0:
            return
        range_m = raw_range_cm * 0.01

        if is_lidar:
            arm_x, arm_y = LIDAR_ARM_X_M, LIDAR_ARM_Y_M
            max_range_m, fade_start_m, fade_r_gain = LIDAR_MAX_RANGE_M, LIDAR_FADE_START_M, LIDAR_FADE_R_GAIN
            r_base = LIDAR_R_BASE_M2
        else:
            arm_x, arm_y = SONAR_ARM_X_M, SONAR_ARM_Y_M
            max_range_m, fade_start_m, fade_r_gain = SONAR_MAX_RANGE_M, SONAR_FADE_START_M, SONAR_FADE_R_GAIN
            r_base = SONAR_R_BASE_M2
            if gz < SONAR_TILT_REJECT_GZ_MIN:
                return

        if range_m >= max_range_m:
            return

        implied_h = (range_m * gz) - ((arm_x * gx) + (arm_y * gy))

        conf = np.clip(confidence, CONFIDENCE_FLOOR, 1.0)
        r = r_base / (conf * conf)

        gz_floor = max(abs(gz), 0.15)
        r /= (gz_floor * gz_floor)

        if range_m > fade_start_m:
            fade_frac = min((range_m - fade_start_m) / (max_range_m - fade_start_m), 1.0)
            fade_shape = 0.5 - (0.5 * math.cos(fade_frac * math.pi))
            r *= (1.0 + (fade_r_gain * fade_shape))

        if range_m < CROSSCHECK_MAX_M:
            if is_lidar:
                other_h, other_r, other_ms = self.sonar_last_h, self.sonar_last_r, self.sonar_last_ms
            else:
                other_h, other_r, other_ms = self.lidar_last_h, self.lidar_last_r, self.lidar_last_ms
            if other_ms != 0 and (now_ms - other_ms) < CROSSCHECK_MAX_AGE_MS:
                combined_sigma = math.sqrt(r + other_r)
                disagreement = abs(implied_h - other_h)
                if disagreement > (CROSSCHECK_SIGMA_MULT * combined_sigma):
                    this_err = abs(implied_h - self.x[0])
                    other_err = abs(other_h - self.x[0])
                    if this_err > other_err:
                        r *= cross_check_penalty(disagreement, combined_sigma,
                                                  CROSSCHECK_SIGMA_MULT, CROSSCHECK_R_PENALTY)

        if is_lidar:
            self.lidar_last_h, self.lidar_last_r, self.lidar_last_ms = implied_h, r, now_ms
        else:
            self.sonar_last_h, self.sonar_last_r, self.sonar_last_ms = implied_h, r, now_ms

        self._scalar_update(implied_h, r)


def world_up_in_body_frame(pitch_deg, roll_deg):
    """Approximates Attitude_GetWorldUpInBodyFrame() from logged Euler angles
    (the log has no quaternion). gx/gy are only used here for the lever-arm
    term; gz is the dominant term for both tilt-compensation and the sonar
    tilt-reject gate."""
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)
    gx = -math.sin(pitch)
    gy = math.sin(roll) * math.cos(pitch)
    gz = math.cos(roll) * math.cos(pitch)
    return gx, gy, gz


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw-in", default="sdlog_raw_20260825_213713.txt")
    ap.add_argument("--flight", type=int, default=0)
    ap.add_argument("--out", default="tools/vert_ekf_replay.png")
    args = ap.parse_args()

    raw = open(args.raw_in).read()
    recs = sd.parse_records(raw)
    flights = sd.segment_flights(recs)
    a, b = flights[args.flight]
    recs = recs[a:b]
    print(f"Replaying flight {args.flight}: {len(recs)} samples, "
          f"{(recs[-1]['time_ms'] - recs[0]['time_ms']) / 1000.0:.1f}s")

    ekf = VertEkf()
    ekf.reset_ms = recs[0]["time_ms"]  # VertEkf_Reset() happens at arm, i.e. the start of this flight segment

    t_s = np.array([(r["time_ms"] - recs[0]["time_ms"]) / 1000.0 for r in recs])
    baro_alt_m = np.array([r["baro_alt_cm"] / 100.0 for r in recs])
    baro_vz_mps = np.array([r["baro_vz_cms"] / 100.0 for r in recs])
    pitch_deg = np.array([r["pitch_deg_x10"] / 10.0 for r in recs])
    roll_deg = np.array([r["roll_deg_x10"] / 10.0 for r in recs])
    rangefinder_cm = np.array([r["rangefinder_cm_x10"] / 10.0 for r in recs])
    luna_cm = np.array([r["luna_cm_x10"] / 10.0 for r in recs])
    motor_avg_us = np.array([
        (r["motor_fl_us"] + r["motor_fr_us"] + r["motor_rr_us"] + r["motor_rl_us"]) / 4.0
        for r in recs
    ])
    motor_delta_us = motor_avg_us - APP_MOTOR_IDLE_US

    # accel_up_mps2 proxy: finite-difference of the logged (already-filtered)
    # baro climb rate - see module docstring's FIDELITY GAPS.
    accel_proxy = np.zeros(len(recs))
    for i in range(1, len(recs)):
        dt = t_s[i] - t_s[i - 1]
        if dt > 0.0005:
            accel_proxy[i] = (baro_vz_mps[i] - baro_vz_mps[i - 1]) / dt

    fused_h = np.zeros(len(recs))
    fused_vz = np.zeros(len(recs))
    fused_bias = np.zeros(len(recs))
    lidar_implied = np.full(len(recs), np.nan)
    sonar_implied = np.full(len(recs), np.nan)

    for i, r in enumerate(recs):
        dt = 0.0 if i == 0 else (t_s[i] - t_s[i - 1])
        if i > 0:
            ekf.predict(accel_proxy[i], dt)

        now_ms = r["time_ms"]
        gx, gy, gz = world_up_in_body_frame(pitch_deg[i], roll_deg[i])

        ekf.update_baro(baro_alt_m[i], True, motor_delta_us[i], now_ms)

        if rangefinder_cm[i] > 0.0:
            ekf.update_range(rangefinder_cm[i], 1.0, False, now_ms, gx, gy, gz)
        if luna_cm[i] > 0.0:
            ekf.update_range(luna_cm[i], 1.0, True, now_ms, gx, gy, gz)

        fused_h[i] = ekf.x[0]
        fused_vz[i] = ekf.x[1]
        fused_bias[i] = ekf.x[2]
        lidar_implied[i] = ekf.lidar_last_h
        sonar_implied[i] = ekf.sonar_last_h

    fig, axes = plt.subplots(4, 1, figsize=(13, 14), sharex=True)

    ax = axes[0]
    ax.plot(t_s, baro_alt_m, label="baro_alt (logged, LPF'd)", color="0.6", lw=1)
    ax.plot(t_s, fused_h, label="EKF height", color="C0", lw=1.5)
    rng_mask = rangefinder_cm > 0
    luna_mask = luna_cm > 0
    ax.scatter(t_s[rng_mask], sonar_implied[rng_mask], s=4, color="C1", label="sonar implied (tilt+arm)")
    ax.scatter(t_s[luna_mask], lidar_implied[luna_mask], s=4, color="C2", label="lidar implied (tilt+arm)")
    ax.axhline(GROUND_EFFECT_ZONE_M, color="r", ls=":", lw=0.8, label=f"ground-effect zone ({GROUND_EFFECT_ZONE_M:.2f}m)")
    ax.axhline(CROSSCHECK_MAX_M, color="purple", ls=":", lw=0.8, label=f"crosscheck band ({CROSSCHECK_MAX_M:.0f}m)")
    ax.set_ylabel("height (m)")
    ax.legend(loc="upper right", fontsize=7)
    ax.set_title("VertEkf offline replay - " + args.raw_in)

    ax = axes[1]
    ax.plot(t_s, baro_vz_mps, label="baro_vz (logged)", color="0.6", lw=1)
    ax.plot(t_s, fused_vz, label="EKF vz", color="C0", lw=1.5)
    ax.set_ylabel("climb rate (m/s)")
    ax.legend(loc="upper right", fontsize=7)

    ax = axes[2]
    ax.plot(t_s, fused_bias, color="C3", label="EKF accel_bias (m/s^2)")
    ax.set_ylabel("accel bias (m/s^2)")
    ax.legend(loc="upper right", fontsize=7)

    ax = axes[3]
    ax.plot(t_s, pitch_deg, label="pitch (deg)", color="C4", lw=0.8)
    ax.plot(t_s, roll_deg, label="roll (deg)", color="C5", lw=0.8)
    ax2 = ax.twinx()
    ax2.plot(t_s, motor_delta_us, label="motor delta (us)", color="0.4", lw=0.6, alpha=0.6)
    ax.set_ylabel("attitude (deg)")
    ax2.set_ylabel("motor delta (us)")
    ax.set_xlabel("time (s)")
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=7)

    fig.tight_layout()
    fig.savefig(args.out, dpi=130)
    print(f"Saved {args.out}")
    print(f"Fused height range: {fused_h.min():.2f}m .. {fused_h.max():.2f}m")
    print(f"Fused vz range: {fused_vz.min():.2f} .. {fused_vz.max():.2f} m/s")
    print(f"Final accel bias estimate: {fused_bias[-1]:.4f} m/s^2")


if __name__ == "__main__":
    main()
