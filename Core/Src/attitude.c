#include "attitude.h"

#include "arm_math.h"

#include <math.h>

#define DEG_PER_RAD 57.2957795f

typedef struct
{
  float q0;
  float q1;
  float q2;
  float q3;
  float integral_x;
  float integral_y;
  float integral_z;
} AHRS_State_t;

static AHRS_State_t g_ahrs;

/* Low-pass filtered accel magnitude used only for the trust-gating decision
 * below - vibration during real (prop-spinning) flight makes the raw
 * instantaneous |accel|-1g deviation chronically nonzero even at true level,
 * which would otherwise chronically under-trust the accelerometer and let
 * gyro bias drift go uncorrected for the whole flight. */
static float g_accel_mag_filt_g;

float Attitude_WrapAngle180(float angle_deg)
{
  while (angle_deg <= -180.0f)
  {
    angle_deg += 360.0f;
  }
  while (angle_deg > 180.0f)
  {
    angle_deg -= 360.0f;
  }
  return angle_deg;
}

void Attitude_Init(void)
{
  g_ahrs.q0 = 1.0f;
  g_ahrs.q1 = 0.0f;
  g_ahrs.q2 = 0.0f;
  g_ahrs.q3 = 0.0f;
  g_ahrs.integral_x = 0.0f;
  g_ahrs.integral_y = 0.0f;
  g_ahrs.integral_z = 0.0f;
  g_accel_mag_filt_g = 1.0f;
}

/* Below this deviation from 1g, trust the accelerometer fully; above it, fade
 * trust down (but never to exactly zero - see ATTITUDE_ACCEL_TRUST_MIN_WEIGHT
 * below) so thrust vibration/transients (which grow with throttle) cannot
 * drag the attitude estimate away from true level. */
#define ATTITUDE_ACCEL_TRUST_MIN_DEV_G 0.10f
/* Was 0.60f - a secured (props-on, airframe restrained so it physically
 * cannot rotate) bench test on 2026-08-18 proved a throttle ramp alone
 * produces a ~5deg false pitch/roll estimate with gyro confirming zero real
 * rotation the whole time: a sustained (non-vibration-frequency) thrust
 * transient passes right through the 5Hz g_accel_mag_filt_g LPF, so
 * accel_dev_g climbs into this gating band for a couple of seconds and even
 * a partially-trusted (let alone floor-weighted) correction at kp=3.0 is
 * enough to drag the estimate off if there's any transverse-axis coupling
 * during the transient (frame flex, mounting tolerance, vibration harmonics
 * shifting with RPM - anything not perfectly axial with true thrust).
 * Halving this reaches the ATTITUDE_ACCEL_TRUST_MIN_WEIGHT floor at half the
 * deviation, shrinking the window a throttle-ramp-scale disturbance still
 * gets meaningful accelerometer weight, without touching MIN_DEV_G (leaves
 * steady-hover trust behavior, which was never the reported problem, alone). */
#define ATTITUDE_ACCEL_TRUST_MAX_DEV_G 0.30f
/* Gyro bias is only re-calibrated on the ground (see App_UpdateGyroBias() in
 * app.c) - once armed and flying, the frozen ground bias is all that's
 * subtracted, so any bias drift or shift caused by heat/vibration once props
 * are spinning has no way back to zero except this Mahony integral term.
 * Letting accel_weight hit exactly 0.0 during a sustained high-vibration
 * flight (e.g. from a damaged/unbalanced prop) fully stops that correction,
 * letting residual bias integrate unboundedly into a false, one-directional
 * angle drift even while the aircraft is physically level. A small nonzero
 * floor keeps a slow trickle of correction alive at all times. */
#define ATTITUDE_ACCEL_TRUST_MIN_WEIGHT 0.05f
/* Cutoff for the trust-gating magnitude filter. Raised 5.0f -> 20.0f
 * (2026-09-05) after a hand-held bench test (hold at constant height, tilt by
 * hand, motors OFF so vibration is fully ruled out) showed VertEkf's fused
 * height violently diverging - 15-20cm+ during real flight tilts, 40cm+ at
 * the bench test's larger tilts - on every single pitch/roll excursion, while
 * both raw range sensors (LUNA, sonar) stayed smooth throughout. Root cause:
 * at the old 5Hz cutoff (~200ms lag), a fast real tilt's accelerometer
 * deviation takes ~200ms to be detected and trust faded down, and in that
 * window a still-highly-trusted correction at two_kp=3.0 drags the AHRS
 * quaternion off true attitude - the EXACT same mechanism already found and
 * partially mitigated for throttle-ramp transients on 2026-08-18 (see
 * ATTITUDE_ACCEL_TRUST_MAX_DEV_G's comment above), just triggered by tilt
 * instead of thrust. That corrupted quaternion then feeds BOTH
 * Attitude_GetVerticalAccelMps2() (vert_ekf.c's predict step) and
 * VertEkf_UpdateRange()'s tilt compensation, so the error shows up doubled.
 * 20Hz (~50ms lag, a 4x cut) reacts to a real disturbance far faster while
 * staying well below APP_GYRO_RATE_LPF_HZ=70Hz - this project's own
 * already-flight-validated cutoff for rejecting this airframe's actual
 * motor/prop vibration content - so genuine vibration should still average
 * out here same as before; only the reaction time to a genuine sustained
 * disturbance changed. NOT YET RE-VALIDATED on the bench as of this note -
 * rerun the same hand-held tilt test and confirm the fused height stays
 * close to LUNA/sonar through a tilt before trusting this in flight. */
#define ATTITUDE_ACCEL_MAG_LPF_HZ 20.0f

/* Gyro-rate accelerometer trust gate - TRIED AND REVERTED (2026-09-05, same
 * night). Added a second trust signal gating on raw gyro rate (thresholds
 * copied directly from iNav's imuCalculateAccelerometerWeightRateIgnore()
 * defaults: full trust below 10deg/s, ramping to ZERO by 20deg/s), on the
 * theory that gyro rate reacts to a fast rotation with no filtering lag,
 * unlike the magnitude-deviation gate above. REVERTED after the very next
 * bench test: user reported never tilting the aircraft more than ~20-30deg,
 * but telemetry showed pitch/roll spiking to 90-128deg. Root-caused directly
 * from that capture: real hand-tilt gyro rates were ABOVE 20deg/s in 54% of
 * all samples, sustained for seconds at a time (ordinary repositioning
 * easily produces 50-100+ deg/s, not just violent disturbances) - so this
 * gate was killing accelerometer correction almost continuously during any
 * real tilting, not just brief fast rotations, letting the quaternion
 * integrate on gyro alone (with whatever real bias/drift the gyro has) for
 * long enough to run away by 60+ degrees from true attitude (confirmed
 * directly in that capture: pitch climbed smoothly 36->88deg over 2.5s while
 * gyro rate stayed mostly 50-90dps the whole time - textbook uncorrected
 * integration drift, not measurement noise). iNav's specific numeric
 * defaults evidently don't transfer safely to this airframe/gyro without
 * further validation this session didn't do before flashing it - do not
 * reintroduce a gyro-rate trust gate without bench-testing candidate
 * thresholds MUCH higher than 20deg/s (real hand-tilt/flight rates routinely
 * exceed that) and confirming reported attitude tracks a known real motion
 * before trusting it again. */

void Attitude_UpdateIMU(float gx_rad_s,
                        float gy_rad_s,
                        float gz_rad_s,
                        float ax_g,
                        float ay_g,
                        float az_g,
                        float dt_s)
{
  const float two_kp = 3.0f;
  const float two_ki = 0.2f;
  float recip_norm;
  float half_vx;
  float half_vy;
  float half_vz;
  float half_ex;
  float half_ey;
  float half_ez;
  float norm_sq;
  float accel_dev_g;
  float accel_weight;
  arm_status status;

  norm_sq = (ax_g * ax_g) + (ay_g * ay_g) + (az_g * az_g);
  status = arm_sqrt_f32(norm_sq, &recip_norm);
  if ((status == ARM_MATH_SUCCESS) && (recip_norm > 0.0001f))
  {
    ax_g /= recip_norm;
    ay_g /= recip_norm;
    az_g /= recip_norm;

    half_vx = g_ahrs.q1 * g_ahrs.q3 - g_ahrs.q0 * g_ahrs.q2;
    half_vy = g_ahrs.q0 * g_ahrs.q1 + g_ahrs.q2 * g_ahrs.q3;
    half_vz = g_ahrs.q0 * g_ahrs.q0 - 0.5f + g_ahrs.q3 * g_ahrs.q3;

    half_ex = (ay_g * half_vz) - (az_g * half_vy);
    half_ey = (az_g * half_vx) - (ax_g * half_vz);
    half_ez = (ax_g * half_vy) - (ay_g * half_vx);

    {
      float mag_lpf_tau_s = 1.0f / (2.0f * 3.14159265f * ATTITUDE_ACCEL_MAG_LPF_HZ);
      float mag_lpf_alpha = 1.0f - expf(-dt_s / mag_lpf_tau_s);
      g_accel_mag_filt_g += mag_lpf_alpha * (recip_norm - g_accel_mag_filt_g);
    }

    accel_dev_g = fabsf(g_accel_mag_filt_g - 1.0f);
    if (accel_dev_g <= ATTITUDE_ACCEL_TRUST_MIN_DEV_G)
    {
      accel_weight = 1.0f;
    }
    else if (accel_dev_g >= ATTITUDE_ACCEL_TRUST_MAX_DEV_G)
    {
      accel_weight = ATTITUDE_ACCEL_TRUST_MIN_WEIGHT;
    }
    else
    {
      accel_weight = 1.0f - ((1.0f - ATTITUDE_ACCEL_TRUST_MIN_WEIGHT) *
                             ((accel_dev_g - ATTITUDE_ACCEL_TRUST_MIN_DEV_G) /
                             (ATTITUDE_ACCEL_TRUST_MAX_DEV_G - ATTITUDE_ACCEL_TRUST_MIN_DEV_G)));
    }

    half_ex *= accel_weight;
    half_ey *= accel_weight;
    half_ez *= accel_weight;

    g_ahrs.integral_x += two_ki * half_ex * dt_s;
    g_ahrs.integral_y += two_ki * half_ey * dt_s;
    g_ahrs.integral_z += two_ki * half_ez * dt_s;

    gx_rad_s += g_ahrs.integral_x + (two_kp * half_ex);
    gy_rad_s += g_ahrs.integral_y + (two_kp * half_ey);
    gz_rad_s += g_ahrs.integral_z + (two_kp * half_ez);
  }

  gx_rad_s *= 0.5f * dt_s;
  gy_rad_s *= 0.5f * dt_s;
  gz_rad_s *= 0.5f * dt_s;

  {
    float qa = g_ahrs.q0;
    float qb = g_ahrs.q1;
    float qc = g_ahrs.q2;

    g_ahrs.q0 += (-qb * gx_rad_s - qc * gy_rad_s - g_ahrs.q3 * gz_rad_s);
    g_ahrs.q1 += (qa * gx_rad_s + qc * gz_rad_s - g_ahrs.q3 * gy_rad_s);
    g_ahrs.q2 += (qa * gy_rad_s - qb * gz_rad_s + g_ahrs.q3 * gx_rad_s);
    g_ahrs.q3 += (qa * gz_rad_s + qb * gy_rad_s - qc * gx_rad_s);
  }

  norm_sq = (g_ahrs.q0 * g_ahrs.q0) +
            (g_ahrs.q1 * g_ahrs.q1) +
            (g_ahrs.q2 * g_ahrs.q2) +
            (g_ahrs.q3 * g_ahrs.q3);
  status = arm_sqrt_f32(norm_sq, &recip_norm);
  if ((status == ARM_MATH_SUCCESS) && (recip_norm > 0.0001f))
  {
    g_ahrs.q0 /= recip_norm;
    g_ahrs.q1 /= recip_norm;
    g_ahrs.q2 /= recip_norm;
    g_ahrs.q3 /= recip_norm;
  }
}

/* World-up (earth +Z) unit vector, expressed in body-frame coordinates - the
 * same quantity independently recomputed inline in both
 * Attitude_GetVerticalAccelMps2() and Attitude_GetBoardAnglesDeg() below,
 * factored out here (2026-08-29) so a third consumer (vert_ekf.c's off-center
 * range-sensor lever-arm correction - see VertEkf_UpdateRange()) doesn't need
 * a fourth copy. For any body-frame vector v_body, v_body . (gx,gy,gz) gives
 * that vector's world-up component - e.g. Attitude_GetVerticalAccelMps2()
 * dots this with the raw accelerometer reading, and vert_ekf.c dots it with a
 * sensor's fixed body-frame mounting offset. */
void Attitude_GetWorldUpInBodyFrame(float *gx, float *gy, float *gz)
{
  *gx = 2.0f * ((g_ahrs.q1 * g_ahrs.q3) - (g_ahrs.q0 * g_ahrs.q2));
  *gy = 2.0f * ((g_ahrs.q0 * g_ahrs.q1) + (g_ahrs.q2 * g_ahrs.q3));
  *gz = (g_ahrs.q0 * g_ahrs.q0) - (g_ahrs.q1 * g_ahrs.q1) - (g_ahrs.q2 * g_ahrs.q2) + (g_ahrs.q3 * g_ahrs.q3);
}

/* Net (gravity-removed) EARTH-frame vertical acceleration, in m/s^2,
 * positive = accelerating upward. Added 2026-08-21 to feed a baro/accel
 * complementary filter for climb rate (see Baro_Update()) - baro-derived
 * climb rate alone is noisy (differentiation amplifies sample noise) and
 * laggy (needs filtering to tame that noise, which adds delay); accelerometer
 * integration is fast/low-lag short-term but drifts long-term, so blending
 * the two (baro correcting the accel-integration drift) is standard practice
 * and beats either alone.
 *
 * (g_x,g_y,g_z) below is the EXACT same "expected gravity direction in body
 * frame" vector already computed in Attitude_GetBoardAnglesDeg() (and, before
 * normalization, in Attitude_UpdateIMU()'s Mahony correction step) - for a
 * unit vector v_body representing the earth Z axis expressed in body
 * coordinates, the dot product v_body . a_body extracts exactly the
 * earth-frame vertical component of any body-frame vector a_body, by the
 * standard rotation-matrix-transpose identity (no separate matrix multiply or
 * quaternion exposure needed - this reuses the same underlying math the
 * attitude estimate itself depends on, so it can't disagree with it). Result
 * is in g's until the final unit conversion; a level, stationary board reads
 * ~1.0g "up" (accelerometer's normal reaction to gravity), so 1.0f is
 * subtracted to get the NET (dynamic, non-gravity) component only. */
float Attitude_GetVerticalAccelMps2(float ax_g, float ay_g, float az_g)
{
  float g_x;
  float g_y;
  float g_z;
  float up_g;

  Attitude_GetWorldUpInBodyFrame(&g_x, &g_y, &g_z);

  up_g = (g_x * ax_g) + (g_y * ay_g) + (g_z * az_g);

  return (up_g - 1.0f) * 9.80665f;
}

/* Tilt-compensated horizontal specific force in the vehicle's OWN current-
 * heading frame (2026-09-05, feeds horiz_ekf.c's dead-reckoning predict step
 * the same way Attitude_GetVerticalAccelMps2() feeds vert_ekf.c). Deliberately
 * stops short of true north/east: this AHRS is IMU-only (gyro+accel, see
 * Attitude_UpdateIMU()) with no magnetometer input of its own, so its
 * internal yaw is corrected only indirectly, via app.c's mag_yaw_nudge_dps
 * being subtracted from gz_rad_s BEFORE it reaches Attitude_UpdateIMU() -
 * that keeps the externally-tracked yaw_deg (Attitude_GetBoardAnglesDeg()'s
 * yaw output, minus app.c's one-time startup_yaw_offset_deg calibration)
 * accurate, but this function has no reason to duplicate that by reading the
 * quaternion's own absolute yaw. Instead it removes ONLY pitch/roll tilt -
 * exact for any tilt angle (no small-angle approximation), not yaw-dependent
 * at all - and leaves the final body-yaw-frame -> true-NED rotation to
 * Nav_RotateBodyToNed(..., yaw_deg, ...), the SAME already-validated function
 * every other body-frame-to-NED conversion in this codebase already uses.
 * That split guarantees this estimator's NED frame can never silently
 * disagree with NAVPOSHOLD's own stick-to-NED mapping over a startup-offset
 * or yaw-source mismatch.
 *
 * Method: project body +X (forward) onto the plane perpendicular to world-up
 * to get "forward" as it would read if the vehicle's current yaw were
 * leveled out, then take "right" as up-cross-forward (consistent with this
 * codebase's X-forward/Y-right/Z-up right-handed body frame - a level board
 * reads world-up as its own +Z, per Attitude_GetWorldUpInBodyFrame()). Exact
 * for combined pitch+roll, unlike the common ax*cos(pitch)+az*sin(pitch)-
 * style approximation - worth the extra few lines given NAV_POSHOLD's own
 * tilt authority now reaches the same ~35deg ATTITUDE-mode ceiling as manual
 * flying, well outside where a small-angle approximation stays accurate. */
void Attitude_GetHorizontalAccelBodyYaw(float ax_g, float ay_g, float az_g,
                                        float *accel_fwd_mps2, float *accel_right_mps2)
{
  float gx;
  float gy;
  float gz;
  float fwd_x;
  float fwd_y;
  float fwd_z;
  float fwd_norm_sq;
  float fwd_norm;
  float right_x;
  float right_y;
  float right_z;

  Attitude_GetWorldUpInBodyFrame(&gx, &gy, &gz);

  fwd_x = 1.0f - (gx * gx);
  fwd_y = -(gx * gy);
  fwd_z = -(gx * gz);
  fwd_norm_sq = (fwd_x * fwd_x) + (fwd_y * fwd_y) + (fwd_z * fwd_z);
  if (fwd_norm_sq < 1e-6f)
  {
    /* Body +X is (near-)vertical - vehicle is pitched ~90deg, "forward" in
     * the horizontal plane is undefined. Should never happen in normal
     * flight; return zero rather than divide by ~zero. */
    *accel_fwd_mps2 = 0.0f;
    *accel_right_mps2 = 0.0f;
    return;
  }
  fwd_norm = sqrtf(fwd_norm_sq);
  fwd_x /= fwd_norm;
  fwd_y /= fwd_norm;
  fwd_z /= fwd_norm;

  right_x = (gy * fwd_z) - (gz * fwd_y);
  right_y = (gz * fwd_x) - (gx * fwd_z);
  right_z = (gx * fwd_y) - (gy * fwd_x);

  *accel_fwd_mps2 = ((ax_g * fwd_x) + (ay_g * fwd_y) + (az_g * fwd_z)) * 9.80665f;
  *accel_right_mps2 = ((ax_g * right_x) + (ay_g * right_y) + (az_g * right_z)) * 9.80665f;
}

void Attitude_GetBoardAnglesDeg(float *pitch_deg,
                                float *roll_deg,
                                float *yaw_deg)
{
  float g_x;
  float g_y;
  float g_z;
  float pitch_denom;
  float yaw;

  Attitude_GetWorldUpInBodyFrame(&g_x, &g_y, &g_z);

  /* pitch_denom uses sqrt(gy^2+gz^2) deliberately (this simplifies to exactly
   * |cos(pitch)| for any roll, keeping the pitch extraction well-behaved and
   * roll-independent - correct as originally written, verified 2026-09-05).
   * roll does NOT get the equivalent treatment - roll is simply atan2(gy,gz),
   * no denominator combination needed, because gy/gz = tan(roll) exactly
   * (the cos(pitch) factor common to both cancels on its own).
   *
   * BUG FOUND AND FIXED 2026-09-05: this used to compute
   * atan2f(g_y, sqrtf(gx^2+gz^2)) instead of atan2f(g_y, g_z) - only correct
   * when pitch happens to be near zero (then sqrt(gx^2+gz^2) ~= |gz|).  At
   * any combined large pitch+roll it diverges badly: numerically verified
   * against a real bench-test moment (pitch=-50deg, roll=-65deg) - the old
   * formula returned -35.6deg, 29 degrees off the true -65deg. Found while
   * investigating a real fused-height divergence bug (vert_ekf.c's own tilt
   * compensation reads gx/gy/gz directly from Attitude_GetWorldUpInBodyFrame()
   * and was NOT affected by this - this bug hit ATTITUDE-mode's actual roll
   * control and all roll_deg telemetry instead, independent of that other
   * investigation, but found by the same person insisting the sin/cos math
   * had to be wrong somewhere - they were right, just about a different
   * consumer of it than first suspected). */
  pitch_denom = sqrtf((g_y * g_y) + (g_z * g_z));

  /* Negated vs. the raw gravity-vector components so pitch/roll increase in the
   * same sense as gx_dps/gy_dps (the unflipped rate convention used everywhere
   * else, e.g. RATE mode's measured_roll/pitch_rate_dps and ATTITUDE mode's
   * target_roll/pitch_deg) - without this, ATTITUDE mode's angle error runs
   * away in the same direction as a real disturbance instead of opposing it. */
  *pitch_deg = Attitude_WrapAngle180(atan2f(-g_x, pitch_denom) * DEG_PER_RAD);
  *roll_deg = Attitude_WrapAngle180(atan2f(g_y, g_z) * DEG_PER_RAD);

  yaw = -atan2f(2.0f * ((g_ahrs.q0 * g_ahrs.q3) + (g_ahrs.q1 * g_ahrs.q2)),
                1.0f - 2.0f * ((g_ahrs.q2 * g_ahrs.q2) + (g_ahrs.q3 * g_ahrs.q3))) * DEG_PER_RAD;
  *yaw_deg = Attitude_WrapAngle180(yaw);
}
