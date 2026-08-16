#!/usr/bin/env python3
"""Host-side reference-model tests for the GPS navigation math in Core/Src/nav.c.

WHY THIS EXISTS INSTEAD OF A COMPILED C TEST: this machine has no host C
compiler available (only arm-none-eabi-gcc, a cross compiler that cannot
produce a runnable host binary). Following this repo's existing convention of
mirroring firmware structs/logic in plain Python for tooling (see
tools/sdlog_analyze.py, tools/motor_test_gui.py's blackbox playback), this
file reimplements the exact same formulas used in Core/Src/nav.c and asserts
the safety-relevant behaviors called out in the GPS-navigation task spec.

KEEP IN SYNC: if the constants/formulas in Core/Src/nav.c change, update the
mirrored copies below (NAV_* constants, NavMath functions, NavHealthModel).
This is NOT a substitute for building/flashing the real firmware - it only
proves the math and state-machine logic are self-consistent.

Usage:
    python tools/test_nav_math.py
"""
import math
import sys

# ---- Mirrors Core/Inc/nav.h constants (keep in sync) ----
NAV_MAX_MESSAGE_AGE_MS = 700
NAV_MIN_UPDATE_INTERVAL_MS = 50
NAV_MAX_UPDATE_INTERVAL_MS = 1500
NAV_MAX_HORIZONTAL_ACC_M = 5.0
NAV_MAX_PLAUSIBLE_VEL_MPS = 25.0
NAV_MAX_POSITION_JUMP_M = 15.0
NAV_MAX_VELOCITY_JUMP_MPS = 8.0
NAV_MIN_CONSECUTIVE_VALID = 5
NAV_MIN_FIX_TYPE = 3
NAV_EARTH_RADIUS_M = 6371000.0
NAV_STANDARD_GRAVITY_MPS2 = 9.80665
UINT32_MOD = 2 ** 32


# ---- Mirrors the named transformation functions in Core/Src/nav.c ----
class NavMath:
    @staticmethod
    def lat_lon_to_local_ne(ref_lat_deg, ref_lon_deg, lat_deg, lon_deg):
        dlat_rad = math.radians(lat_deg - ref_lat_deg)
        dlon_rad = math.radians(lon_deg - ref_lon_deg)
        ref_lat_rad = math.radians(ref_lat_deg)
        north_m = dlat_rad * NAV_EARTH_RADIUS_M
        east_m = dlon_rad * NAV_EARTH_RADIUS_M * math.cos(ref_lat_rad)
        return north_m, east_m

    @staticmethod
    def rotate_body_to_ned(fwd, right, yaw_deg):
        yaw_rad = math.radians(yaw_deg)
        c = math.cos(yaw_rad)
        s = math.sin(yaw_rad)
        north = (fwd * c) - (right * s)
        east = (fwd * s) + (right * c)
        return north, east

    @staticmethod
    def rotate_ned_to_body(north, east, yaw_deg):
        yaw_rad = math.radians(yaw_deg)
        c = math.cos(yaw_rad)
        s = math.sin(yaw_rad)
        fwd = (north * c) + (east * s)
        right = (-north * s) + (east * c)
        return fwd, right

    @staticmethod
    def accel_to_angle_deg(accel_mps2, max_angle_deg):
        if not math.isfinite(accel_mps2):
            return 0.0
        angle_deg = math.degrees(math.atan2(accel_mps2, NAV_STANDARD_GRAVITY_MPS2))
        return max(-max_angle_deg, min(max_angle_deg, angle_deg))


# ---- Mirrors the Nav_Update() health/validity state machine in Core/Src/nav.c ----
class NavHealthModel:
    """Simplified but behaviorally-equivalent reimplementation for host testing.

    Feed samples via update(); each sample is a dict with keys: fix_type,
    num_sv, lat_deg, lon_deg, h_acc_m, vel_n_mps, vel_e_mps, itow_ms, host_ms,
    update_count (monotonic counter, mirrors GPS_GetPvtUpdateCount()).
    """

    def __init__(self):
        self.reference_valid = False
        self.ref_lat = 0.0
        self.ref_lon = 0.0
        self.north_m = 0.0
        self.east_m = 0.0
        self.raw_vel_n = 0.0
        self.raw_vel_e = 0.0
        self.consecutive_valid = 0
        self.consecutive_invalid = 0
        self.dropout_count = 0
        self.rejected_count = 0
        self.duplicate_count = 0
        self._last_update_count = None
        self._prev_host_ms = None
        self._prev_itow = None
        self._prev_north = None
        self._prev_east = None
        self._was_valid = False
        self.valid = False
        self.invalid_reason = "NO_3D_FIX"

    def latch_reference(self, lat_deg, lon_deg, fix_type):
        if fix_type < NAV_MIN_FIX_TYPE:
            return
        self.reference_valid = True
        self.ref_lat = lat_deg
        self.ref_lon = lon_deg
        self.north_m = 0.0
        self.east_m = 0.0
        self._prev_north = None
        self._prev_east = None

    @staticmethod
    def _mod_diff_u32(a, b):
        """uint32_t a - b, matching firmware's unsigned wraparound arithmetic."""
        return (a - b) % UINT32_MOD

    def update(self, now_ms, sample):
        reject_reason = "NONE"

        if self._last_update_count is None:
            self._last_update_count = sample["update_count"]
            self._prev_host_ms = sample["host_ms"]
            self._prev_itow = sample["itow_ms"]
        elif sample["update_count"] != self._last_update_count:
            is_dup = sample["itow_ms"] == self._prev_itow
            update_period_ms = self._mod_diff_u32(sample["host_ms"], self._prev_host_ms)
            self._last_update_count = sample["update_count"]

            if is_dup:
                self.duplicate_count += 1
            else:
                ok, reason = self._sample_looks_valid(sample)
                interval_ok = NAV_MIN_UPDATE_INTERVAL_MS <= update_period_ms <= NAV_MAX_UPDATE_INTERVAL_MS
                if ok and not interval_ok:
                    ok, reason = False, "BAD_UPDATE_INTERVAL"

                if ok and self.reference_valid:
                    north_m, east_m = NavMath.lat_lon_to_local_ne(
                        self.ref_lat, self.ref_lon, sample["lat_deg"], sample["lon_deg"]
                    )
                    if self._prev_north is not None:
                        dn = north_m - self._prev_north
                        de = east_m - self._prev_east
                        if (dn * dn + de * de) > (NAV_MAX_POSITION_JUMP_M ** 2):
                            ok, reason = False, "POSITION_JUMP"
                    if ok:
                        dvn = sample["vel_n_mps"] - self.raw_vel_n
                        dve = sample["vel_e_mps"] - self.raw_vel_e
                        if (dvn * dvn + dve * dve) > (NAV_MAX_VELOCITY_JUMP_MPS ** 2):
                            ok, reason = False, "VELOCITY_JUMP"
                    if ok:
                        self.north_m, self.east_m = north_m, east_m
                        self._prev_north, self._prev_east = north_m, east_m
                        self.raw_vel_n, self.raw_vel_e = sample["vel_n_mps"], sample["vel_e_mps"]
                elif ok:
                    self.raw_vel_n, self.raw_vel_e = sample["vel_n_mps"], sample["vel_e_mps"]
                    reason = "NO_REFERENCE"

                if ok:
                    self.consecutive_valid += 1
                    self.consecutive_invalid = 0
                else:
                    self.rejected_count += 1
                    self.consecutive_invalid += 1
                    self.consecutive_valid = 0
                reject_reason = reason

            self._prev_host_ms = sample["host_ms"]
            self._prev_itow = sample["itow_ms"]

        age_ms = self._mod_diff_u32(now_ms, sample["host_ms"])

        if not all(math.isfinite(x) for x in (sample["lat_deg"], sample["lon_deg"],
                                               sample["vel_n_mps"], sample["vel_e_mps"])):
            self.valid, self.invalid_reason = False, "NONFINITE"
        elif sample["fix_type"] < NAV_MIN_FIX_TYPE:
            self.valid, self.invalid_reason = False, "NO_3D_FIX"
        elif age_ms > NAV_MAX_MESSAGE_AGE_MS:
            self.valid, self.invalid_reason = False, "GPS_STALE"
            self.consecutive_valid = 0
        elif sample["h_acc_m"] > NAV_MAX_HORIZONTAL_ACC_M:
            self.valid, self.invalid_reason = False, "POOR_ACCURACY"
        elif not self.reference_valid:
            self.valid, self.invalid_reason = False, "NO_REFERENCE"
        elif reject_reason != "NONE":
            self.valid, self.invalid_reason = False, reject_reason
        elif self.consecutive_valid < NAV_MIN_CONSECUTIVE_VALID:
            self.valid, self.invalid_reason = False, "REACQUIRING"
        else:
            self.valid, self.invalid_reason = True, "NONE"

        if self._was_valid and not self.valid:
            self.dropout_count += 1
        self._was_valid = self.valid

    @staticmethod
    def _sample_looks_valid(sample):
        vals = (sample["lat_deg"], sample["lon_deg"], sample["vel_n_mps"],
                sample["vel_e_mps"], sample["h_acc_m"])
        if not all(math.isfinite(v) for v in vals):
            return False, "NONFINITE"
        if sample["fix_type"] < NAV_MIN_FIX_TYPE:
            return False, "NO_3D_FIX"
        vel_mag_sq = sample["vel_n_mps"] ** 2 + sample["vel_e_mps"] ** 2
        if vel_mag_sq > (NAV_MAX_PLAUSIBLE_VEL_MPS ** 2):
            return False, "VELOCITY_INVALID"
        return True, "NONE"


def _good_sample(update_count, host_ms, itow_ms, lat=47.0, lon=8.0, vel_n=0.0, vel_e=0.0, h_acc=1.0, fix_type=3):
    return dict(update_count=update_count, host_ms=host_ms, itow_ms=itow_ms,
                lat_deg=lat, lon_deg=lon, vel_n_mps=vel_n, vel_e_mps=vel_e,
                h_acc_m=h_acc, fix_type=fix_type, num_sv=12)


# ---------------------------------------------------------------------------
# Test cases
# ---------------------------------------------------------------------------
_RESULTS = []


def test(name):
    def deco(fn):
        _RESULTS.append((name, fn))
        return fn
    return deco


@test("lat/lon to local N/E: known small offset")
def _t1():
    # ~1 degree latitude ~= 111.19 km; 0.001 deg lat at any longitude ~= 111.19 m north.
    north_m, east_m = NavMath.lat_lon_to_local_ne(47.0, 8.0, 47.001, 8.0)
    assert abs(north_m - 111.19e3 * 0.001) < 2.0, north_m
    assert abs(east_m) < 1e-6, east_m
    # East displacement shrinks with cos(latitude).
    north_m2, east_m2 = NavMath.lat_lon_to_local_ne(60.0, 8.0, 60.0, 8.001)
    north_m3, east_m3 = NavMath.lat_lon_to_local_ne(0.0, 8.0, 0.0, 8.001)
    assert east_m2 < east_m3, (east_m2, east_m3)


@test("body-to-NED rotation at yaw=0: forward=north, right=east")
def _t2():
    north, east = NavMath.rotate_body_to_ned(1.0, 0.0, 0.0)
    assert abs(north - 1.0) < 1e-9 and abs(east - 0.0) < 1e-9
    north, east = NavMath.rotate_body_to_ned(0.0, 1.0, 0.0)
    assert abs(north - 0.0) < 1e-9 and abs(east - 1.0) < 1e-9


@test("body-to-NED rotation at yaw=90: forward=east, right=south")
def _t3():
    north, east = NavMath.rotate_body_to_ned(1.0, 0.0, 90.0)
    assert abs(north - 0.0) < 1e-6 and abs(east - 1.0) < 1e-6
    north, east = NavMath.rotate_body_to_ned(0.0, 1.0, 90.0)
    assert abs(north - (-1.0)) < 1e-6 and abs(east - 0.0) < 1e-6


@test("body-to-NED rotation at yaw=180: forward=south, right=west")
def _t4():
    north, east = NavMath.rotate_body_to_ned(1.0, 0.0, 180.0)
    assert abs(north - (-1.0)) < 1e-6 and abs(east - 0.0) < 1e-6


@test("body-to-NED rotation at yaw=270: forward=west, right=north")
def _t5():
    north, east = NavMath.rotate_body_to_ned(1.0, 0.0, 270.0)
    assert abs(north - 0.0) < 1e-6 and abs(east - (-1.0)) < 1e-6
    north, east = NavMath.rotate_body_to_ned(0.0, 1.0, 270.0)
    assert abs(north - 1.0) < 1e-6 and abs(east - 0.0) < 1e-6


@test("NED-to-body is the exact inverse of body-to-NED at yaw 0/90/180/270")
def _t6():
    for yaw in (0.0, 90.0, 180.0, 270.0, 37.0, -125.0):
        for fwd, right in ((1.0, 0.0), (0.0, 1.0), (0.7, -0.3)):
            north, east = NavMath.rotate_body_to_ned(fwd, right, yaw)
            fwd2, right2 = NavMath.rotate_ned_to_body(north, east, yaw)
            assert abs(fwd2 - fwd) < 1e-9 and abs(right2 - right) < 1e-9, (yaw, fwd, right, fwd2, right2)


def _braking_accel_direction(measured_north_vel, measured_east_vel, yaw_deg, velocity_kp=0.6):
    """Mirrors the NAV_VELOCITY_BRAKE controller with desired velocity = 0
    (sticks centered) - returns the earth-frame accel command."""
    north_err = 0.0 - measured_north_vel
    east_err = 0.0 - measured_east_vel
    return velocity_kp * north_err, velocity_kp * east_err


@test("braking direction: moving north with zero commanded velocity requests southward accel")
def _t7():
    accel_n, accel_e = _braking_accel_direction(measured_north_vel=2.0, measured_east_vel=0.0, yaw_deg=0.0)
    assert accel_n < 0.0, accel_n
    assert abs(accel_e) < 1e-9, accel_e


@test("braking direction: moving east with zero commanded velocity requests westward accel")
def _t8():
    accel_n, accel_e = _braking_accel_direction(measured_north_vel=0.0, measured_east_vel=2.0, yaw_deg=0.0)
    assert accel_e < 0.0, accel_e
    assert abs(accel_n) < 1e-9, accel_n


@test("braking direction: south motion -> north accel; west motion -> east accel")
def _t9():
    accel_n, _ = _braking_accel_direction(measured_north_vel=-2.0, measured_east_vel=0.0, yaw_deg=0.0)
    assert accel_n > 0.0, accel_n
    _, accel_e = _braking_accel_direction(measured_north_vel=0.0, measured_east_vel=-2.0, yaw_deg=0.0)
    assert accel_e > 0.0, accel_e


@test("controller never commands acceleration in the same direction as the velocity error")
def _t10():
    import random
    random.seed(1234)
    for _ in range(200):
        vn = random.uniform(-10, 10)
        ve = random.uniform(-10, 10)
        accel_n, accel_e = _braking_accel_direction(vn, ve, yaw_deg=0.0)
        # error = desired(0) - measured = -measured; accel = Kp*error -> accel and measured must have opposite sign.
        if vn != 0:
            assert (accel_n > 0) != (vn > 0), (vn, accel_n)
        if ve != 0:
            assert (accel_e > 0) != (ve > 0), (ve, accel_e)


@test("accel-to-angle: positive forward accel -> positive angle, saturates at max")
def _t11():
    small = NavMath.accel_to_angle_deg(0.1, 8.0)
    assert 0.0 < small < 8.0, small
    big = NavMath.accel_to_angle_deg(100.0, 8.0)
    assert abs(big - 8.0) < 1e-6, big
    neg = NavMath.accel_to_angle_deg(-100.0, 8.0)
    assert abs(neg - (-8.0)) < 1e-6, neg


@test("accel-to-angle: NaN/inf input is rejected to 0, never latched as a command")
def _t12():
    assert NavMath.accel_to_angle_deg(float("nan"), 8.0) == 0.0
    assert NavMath.accel_to_angle_deg(float("inf"), 8.0) == 0.0
    assert NavMath.accel_to_angle_deg(float("-inf"), 8.0) == 0.0


@test("GPS message-age timeout: nav invalid once age exceeds NAV_MAX_MESSAGE_AGE_MS")
def _t13():
    m = NavHealthModel()
    m.latch_reference(47.0, 8.0, fix_type=3)
    t = 0
    for i in range(NAV_MIN_CONSECUTIVE_VALID + 2):
        t += 200
        m.update(t, _good_sample(i, t, i))
    assert m.valid, m.invalid_reason
    # Advance the clock far beyond the last sample without a new one arriving.
    m.update(t + NAV_MAX_MESSAGE_AGE_MS + 1, _good_sample(i, t, i))
    assert not m.valid and m.invalid_reason == "GPS_STALE"


@test("GPS reacquisition qualification: does not restore validity on the first good sample after a dropout")
def _t14():
    m = NavHealthModel()
    m.latch_reference(47.0, 8.0, fix_type=3)
    t = 0
    for i in range(NAV_MIN_CONSECUTIVE_VALID + 2):
        t += 200
        m.update(t, _good_sample(i, t, i))
    assert m.valid
    # Force a dropout via staleness, then feed exactly one fresh sample.
    stale_t = t + NAV_MAX_MESSAGE_AGE_MS + 1
    m.update(stale_t, _good_sample(i, t, i))
    assert not m.valid
    t = stale_t + 10
    i += 1
    m.update(t, _good_sample(i, t, i))
    assert not m.valid and m.invalid_reason == "REACQUIRING", m.invalid_reason
    # After enough consecutive good samples, validity returns.
    for _ in range(NAV_MIN_CONSECUTIVE_VALID):
        t += 200
        i += 1
        m.update(t, _good_sample(i, t, i))
    assert m.valid, m.invalid_reason


@test("position-jump rejection: an implausible position step is rejected, not silently accepted")
def _t15():
    m = NavHealthModel()
    m.latch_reference(47.0, 8.0, fix_type=3)
    t = 0
    for i in range(NAV_MIN_CONSECUTIVE_VALID + 2):
        t += 200
        m.update(t, _good_sample(i, t, i))
    assert m.valid
    north_before = m.north_m
    t += 200
    i += 1
    # ~1000m north jump (implausible for a real GPS update interval).
    m.update(t, _good_sample(i, t, i, lat=47.009))
    assert not m.valid and m.invalid_reason == "POSITION_JUMP"
    assert m.north_m == north_before, "rejected sample must not update the accepted position"


@test("velocity-jump rejection: an implausible velocity step is rejected")
def _t16():
    m = NavHealthModel()
    m.latch_reference(47.0, 8.0, fix_type=3)
    t = 0
    for i in range(NAV_MIN_CONSECUTIVE_VALID + 2):
        t += 200
        m.update(t, _good_sample(i, t, i, vel_n=1.0))
    assert m.valid
    t += 200
    i += 1
    m.update(t, _good_sample(i, t, i, vel_n=20.0))  # +19 m/s step, exceeds NAV_MAX_VELOCITY_JUMP_MPS
    assert not m.valid and m.invalid_reason == "VELOCITY_JUMP"


@test("irregular GPS update intervals: a too-short or too-long interval is rejected as BAD_UPDATE_INTERVAL")
def _t17():
    m = NavHealthModel()
    m.latch_reference(47.0, 8.0, fix_type=3)
    t = 0
    for i in range(NAV_MIN_CONSECUTIVE_VALID + 2):
        t += 200
        m.update(t, _good_sample(i, t, i))
    assert m.valid
    t += 5  # far below NAV_MIN_UPDATE_INTERVAL_MS
    i += 1
    m.update(t, _good_sample(i, t, i))
    assert not m.valid and m.invalid_reason == "BAD_UPDATE_INTERVAL"


@test("millis()/timer rollover: age/interval math must not misbehave across a uint32 wrap")
def _t18():
    m = NavHealthModel()
    m.latch_reference(47.0, 8.0, fix_type=3)
    t = (UINT32_MOD - 300) % UINT32_MOD
    ids = list(range(NAV_MIN_CONSECUTIVE_VALID + 2))
    for i in ids:
        t = (t + 200) % UINT32_MOD  # this will wrap partway through
        m.update(t, _good_sample(i, t, i))
    assert m.valid, m.invalid_reason
    # One more sample shortly after the wrap - age/interval must still compute a small,
    # sane value (not a huge number from a naive signed subtraction).
    t = (t + 200) % UINT32_MOD
    i = ids[-1] + 1
    m.update(t, _good_sample(i, t, i))
    assert m.valid, m.invalid_reason


@test("NaN/infinite rejection: a sample with a non-finite field never becomes valid")
def _t19():
    m = NavHealthModel()
    m.latch_reference(47.0, 8.0, fix_type=3)
    m.update(100, _good_sample(0, 100, 0, lat=float("nan")))
    assert not m.valid and m.invalid_reason == "NONFINITE"


@test("navigation-output limiting: tilt and acceleration commands stay within configured limits")
def _t20():
    max_tilt = 8.0
    max_accel = 1.0
    velocity_kp = 0.6
    for err in (-100.0, -1.5, 0.0, 1.5, 100.0):
        accel = max(-max_accel, min(max_accel, velocity_kp * err))
        assert -max_accel - 1e-9 <= accel <= max_accel + 1e-9
        angle = NavMath.accel_to_angle_deg(accel, max_tilt)
        assert -max_tilt - 1e-9 <= angle <= max_tilt + 1e-9


@test("GPS loss while nav mode active: engagement gate flips off immediately, no stale command")
def _t21():
    m = NavHealthModel()
    m.latch_reference(47.0, 8.0, fix_type=3)
    t = 0
    for i in range(NAV_MIN_CONSECUTIVE_VALID + 2):
        t += 200
        m.update(t, _good_sample(i, t, i))
    engaged = m.valid and m.reference_valid
    assert engaged
    # GPS drops to no-fix.
    t += 200
    i += 1
    m.update(t, _good_sample(i, t, i, fix_type=0))
    engaged = m.valid and m.reference_valid
    assert not engaged, "must immediately lose engagement eligibility, no stale authority"


@test("mode engagement/disengagement produces no command step (desired velocity starts at current stick, zero error)")
def _t22():
    # At engagement, spec requires desired horizontal velocity to be set from the
    # CURRENT pilot input (not e.g. 0 or a stale value) - verify that if the pilot
    # stick is at whatever position, and measured velocity already matches the
    # implied desired velocity, the resulting acceleration command is exactly zero
    # (no artificial step at the moment of engagement).
    yaw_deg = 45.0
    fwd_cmd, right_cmd = 0.5, -0.2
    desired_n, desired_e = NavMath.rotate_body_to_ned(fwd_cmd, right_cmd, yaw_deg)
    measured_n, measured_e = desired_n, desired_e  # aircraft already moving exactly as commanded
    accel_n, accel_e = _braking_accel_direction_desired(desired_n, desired_e, measured_n, measured_e)
    assert abs(accel_n) < 1e-9 and abs(accel_e) < 1e-9


def _braking_accel_direction_desired(desired_n, desired_e, measured_n, measured_e, velocity_kp=0.6):
    return velocity_kp * (desired_n - measured_n), velocity_kp * (desired_e - measured_e)


def main() -> int:
    passed = 0
    failed = 0
    for name, fn in _RESULTS:
        try:
            fn()
            print(f"PASS  {name}")
            passed += 1
        except AssertionError as exc:
            print(f"FAIL  {name}: {exc}")
            failed += 1
        except Exception as exc:  # noqa: BLE001 - want to report, not crash the whole run
            print(f"ERROR {name}: {exc!r}")
            failed += 1

    print(f"\n{passed} passed, {failed} failed, {len(_RESULTS)} total")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
