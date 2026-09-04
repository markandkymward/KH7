#include "vert_ekf.h"

#include "attitude.h"

#include <math.h>

/* Vertical-channel state estimator (2026-08-29) - height, vertical velocity, and
 * accelerometer bias, predicted from IMU vertical acceleration at the control-loop
 * rate and corrected asynchronously by baro, TF-Luna, and HC-SR04 sonar measurement
 * updates, each at its own native rate. Replaces App_GetFusedAltitudeM()'s hand-rolled
 * single-state (position-only) complementary filter in app.c - that filter had no
 * velocity state of its own (it borrowed Baro_GetClimbRateMps(), itself a SEPARATE
 * complementary filter in baro.c), no way to properly weight two disagreeing range
 * sensors against each other, and no principled way to make TF-Luna vs baro authority
 * shift smoothly with altitude - all things a real Kalman filter gives for free once
 * height and velocity are tracked jointly with per-measurement noise (R) instead of
 * ad-hoc blend rates.
 *
 * DESIGN CHOICE - hand-rolled fixed 3x3 matrices, no CMSIS-DSP matrix helpers, no
 * external EKF library: this codebase is bare-metal C with zero dynamic allocation
 * and no STL anywhere (confirmed across attitude.c/baro.c/gps.c). CMSIS-DSP
 * (arm_math.h) IS already linked and used elsewhere (attitude.c's arm_sqrt_f32), but
 * setting up arm_matrix_instance_f32 wrappers for a fixed 3x3 adds indirection with
 * no real benefit at this size - explicit scalar code is more auditable and matches
 * the very explicit, no-abstraction style already used throughout this codebase's
 * other estimators. A matrix library only earns its keep at state sizes where manual
 * index-tracking becomes error-prone; 3 states isn't that.
 *
 * DESIGN CHOICE - all three measurement types (baro, lidar, sonar) use the SAME
 * scalar update (see VertEkf_ScalarHeightUpdate()) with H always [1,0,0]: after
 * tilt/lever-arm projection, every one of them is fundamentally "here is an
 * independent estimate of height," so there is exactly one measurement model in this
 * file, not three. This is a genuine simplification, not corner-cutting: attitude is
 * NOT a jointly-estimated state here (it comes from the already-good, separately
 * running Mahony filter in attitude.c as a trusted input), so there is no nonlinearity
 * inside the measurement model itself that would require true EKF linearization - the
 * only nonlinear step (range * cos(tilt), the lever-arm projection) happens as
 * PRE-PROCESSING on the raw measurement before it ever reaches the linear update. That
 * makes this, strictly, a linear Kalman filter with a nonlinear pre-processing step on
 * two of its three measurement types - simpler than a "real" EKF and exactly as
 * accurate for this problem, since attitude genuinely isn't a state we need this
 * filter to estimate.
 *
 * DESIGN CHOICE - smooth sensor handoff comes from smoothly-scheduled R, not explicit
 * blend logic: rather than hand-coding a 6-8m "transition zone" that blends two
 * outputs, TF-Luna's own R ramps up (soft, raised-cosine) approaching its range
 * ceiling while baro's R relaxes as ground-effect severity fades with height - the
 * Kalman gain shifts weighting between them automatically wherever their R's happen
 * to cross over. No separate handoff mechanism, no discontinuity to avoid by
 * construction.
 *
 * UNTUNED, BY DESIGN, PENDING REAL DATA: every noise/gain constant below is a
 * physically-reasoned starting point (baro's R_base from
 * kh7-baro-hover-noise-characterization's real ~5-7cm measurement; the ground-effect
 * zone from a rotor-diameter rule of thumb; everything else a defensible guess) - none
 * of it has been validated against a logged flight yet. That validation (replaying
 * real SD log data through this filter offline, same workflow used all session for
 * every other estimator/control change) is the explicit next step after this file
 * compiles, not part of this change. Do NOT wire this estimator's output into the live
 * ALTHOLD control loop (replacing Baro_GetAltitudeM()/Baro_GetClimbRateMps()/
 * App_GetFusedAltitudeM()) before that validation - see kh7-althold-full-authority-
 * redesign and kh7-althold-hover-est-ceiling-incident memory for exactly how much
 * damage an unvalidated change to this exact control path has done this session. */

/* --- Physical layout (2026-08-29, user-provided: quad-X, 5-inch props) ---
 * Body frame assumed +X forward, +Y right, +Z up (a level board has body +Z aligned
 * with world +Z) - matches Attitude_GetWorldUpInBodyFrame()'s convention.
 *
 * UNVERIFIED SIGN CONVENTION: derived from "left-forward" (LiDAR) / "left-aft"
 * (sonar), both at the standard 45deg X-arm angle, under the above assumed body-axis
 * handedness - NOT yet bench-confirmed against this specific airframe's actual X/Y
 * sign convention. Before trusting the lever-arm term in flight: tilt the aircraft a
 * known amount (e.g. nose-down ~15deg) while holding a fixed height over a flat floor,
 * and confirm VertEkf_GetHeightM() moves the RIGHT way relative to a known-good
 * reference (a tape measure, or whichever range sensor isn't near its own limit at
 * that height). If it moves the wrong way, negate ALL FOUR arm constants together
 * (they share one convention) rather than re-deriving the geometry from scratch. */
#define VERT_EKF_LIDAR_ARM_X_M    0.0990f   /* 140mm at 45deg, left-FORWARD -> +X */
#define VERT_EKF_LIDAR_ARM_Y_M   -0.0990f   /* left -> -Y */
#define VERT_EKF_SONAR_ARM_X_M   -0.0566f   /* 80mm at 45deg, left-AFT -> -X */
#define VERT_EKF_SONAR_ARM_Y_M   -0.0566f   /* left -> -Y */

/* --- Ground-effect R scheduling --- */
#define VERT_EKF_ROTOR_DIAMETER_M            0.127f  /* 5-inch props */
/* Height below which baro's residual (POST-bias-correction) noise gets inflated,
 * scaled further by thrust demand below. 2x rotor diameter is a common rule-of-thumb
 * boundary for meaningful single-rotor ground effect - used here as a physically
 * motivated STARTING POINT for a multirotor's combined/recirculating downwash, not a
 * value characterized for this specific airframe. */
#define VERT_EKF_GROUND_EFFECT_ZONE_M         (2.0f * VERT_EKF_ROTOR_DIAMETER_M)
/* motor_power_delta_us (PWM above idle) treated as "full ground-effect-relevant
 * thrust" for scaling - roughly matches the throttle range Baro_Update()'s own
 * propwash BIAS model was bench-characterized over (see BARO_PROPWASH_ALT_*_M in
 * baro.c) - this is a separate NOISE (not bias) inflation on top of that. */
#define VERT_EKF_GROUND_EFFECT_THRUST_REF_US  350.0f
#define VERT_EKF_BARO_GROUND_EFFECT_R_GAIN    20.0f

/* --- Measurement noise bases --- */
/* Baseline (clear of ground effect, hovering) baro noise, from real flight
 * characterization - see kh7-baro-hover-noise-characterization memory (~5-7cm std
 * observed). Squared for variance. */
#define VERT_EKF_BARO_R_BASE_M2               (0.06f * 0.06f)
#define VERT_EKF_LIDAR_R_BASE_M2              (0.03f * 0.03f)
#define VERT_EKF_SONAR_R_BASE_M2              (0.02f * 0.02f)
/* Baro-vs-range cross-check (2026-08-29, added after offline replay against
 * sdlog_raw_20260825_213713.txt - see kh7-baro-liftoff-transient memory -
 * showed the ~2-3s liftoff baro transient pulling fused height down ~2m 1:1.
 * Two earlier approaches were tried and rejected by that same replay:
 *   1) An innovation gate (reject/inflate R when baro disagrees with the
 *      filter's OWN current state) did almost nothing, because the transient
 *      is a multi-sample RAMP, not a single-sample spike - each step's
 *      innovation relative to the state, which has already partially
 *      tracked the previous steps down, never looks anomalous enough to
 *      trip a same-sensor gate. Worse, once the state HAD been dragged down,
 *      the gate then resisted the correction back to truth (the recovery
 *      looked like the "outlier" relative to the now-wrong state),
 *      prolonging the error - replay-confirmed, not theoretical.
 *   2) A fixed-duration post-reset R boost (raised-cosine decay, independent
 *      of any measurement) avoided that specific failure mode but only
 *      marginally reduced the dip, because it is mathematically incapable of
 *      doing more: while baro is the ONLY active measurement, inflating its
 *      own R can only slow how fast the filter believes it, not prevent
 *      eventual convergence toward a sustained, self-consistent wrong
 *      reading - no amount of intrinsic R-shaping on a sensor can reject a
 *      sustained error in that sensor when nothing else is checking it.
 * The replay data showed exactly why this transient is representative of a
 * SOLVABLE case, though: the sonar reading was live, valid, and rock-steady
 * (~5.8cm - the aircraft genuinely had not left the ground yet) for the
 * entire duration baro was diverging to -2m. vert_ekf.c already computes and
 * stores this (g_sonar_last_implied_height_m / g_lidar_last_implied_height_m)
 * for the existing range-vs-range cross-check - baro just never consulted
 * it. Reusing the freshest still-valid range reading as a reference for baro
 * too (same disagreement-vs-combined-sigma test as the range-vs-range check)
 * uses real independent evidence instead of a guess, and only applies when
 * that evidence actually exists.
 *
 * That "only when evidence exists" turned out to matter concretely in this
 * same replay: sonar itself drops out for ~800ms (t=1.65-2.45s in the
 * replayed flight) landing right in the worst part of the baro transient, so
 * the cross-check alone leaves that sub-window uncovered. The two rejected-
 * then-revived post-reset settle boost (below) fills exactly that gap - it
 * does not need a live reference, only wall-clock time since arm/reset - so
 * the two mechanisms are complementary (evidence-based when evidence exists,
 * generic caution when it does not) rather than redundant. Both confirmed
 * via replay: cross-check alone left the dip essentially unchanged (a fresh
 * sonar reference wasn't available exactly when needed); settle boost alone
 * only marginally reduced it (rejected on its own above, revived here as a
 * complement, not a replacement). */
#define VERT_EKF_BARO_CROSSCHECK_SIGMA_MULT   3.0f
#define VERT_EKF_BARO_CROSSCHECK_R_PENALTY    8.0f
#define VERT_EKF_BARO_POST_RESET_SETTLE_MS    2500U
#define VERT_EKF_BARO_POST_RESET_R_GAIN       40.0f
/* Confidence floor - the ESP32 side's confidence==0 means "weakest real reading it
 * still considered worth reporting," not "zero trust" (see
 * esp32_s3_uart6_wifi_bridge.ino's LUNA_STRENGTH_FLOOR comment) - this keeps R large
 * but finite rather than blowing up at a literal division by zero. */
#define VERT_EKF_CONFIDENCE_FLOOR             0.05f

#define VERT_EKF_SONAR_MAX_RANGE_M            3.0f
#define VERT_EKF_SONAR_FADE_START_M           2.3f
#define VERT_EKF_SONAR_FADE_R_GAIN            25.0f
/* cos(17.5deg) - middle of the requested "~15-20deg" sonar tilt-reject band.
 * Comparing gz directly against this avoids computing an acosf() just to throw the
 * angle away again. */
#define VERT_EKF_SONAR_TILT_REJECT_GZ_MIN     0.9537f

#define VERT_EKF_LIDAR_MAX_RANGE_M            8.0f
#define VERT_EKF_LIDAR_FADE_START_M           7.0f
#define VERT_EKF_LIDAR_FADE_R_GAIN            20.0f

/* 0-3m lidar/sonar cross-validation band. */
#define VERT_EKF_CROSSCHECK_MAX_M             3.0f
#define VERT_EKF_CROSSCHECK_SIGMA_MULT        2.0f
/* A bit more than the sonar's own ~100ms native trigger period - how recent the
 * OTHER sensor's last reading must be to cross-check against. */
#define VERT_EKF_CROSSCHECK_MAX_AGE_MS        150U
/* Penalty AT the trigger threshold (VERT_EKF_CROSSCHECK_SIGMA_MULT sigma) - scales
 * quadratically with additional sigma beyond that via VertEkf_CrossCheckPenalty()
 * below. See that function's comment for why a FLAT multiplier here (the original
 * 2026-08-29 design) was a real bug caught by live ALTHOLD-on-EKF flight testing: a
 * confirmed 27-sigma sonar/lidar disagreement got the exact same 8x penalty as a
 * borderline 2-sigma one, leaving R at ~5.7cm std - nowhere near enough to stop a
 * ~98cm-wrong single-sample sonar glitch from pulling the fused height down ~25cm
 * and triggering a real, visible ALTHOLD throttle correction ("blips up" the pilot
 * could see and feel). */
#define VERT_EKF_CROSSCHECK_R_PENALTY         8.0f

/* --- Process noise --- */
#define VERT_EKF_ACCEL_NOISE_MPS2             0.5f     /* untuned starting point */
#define VERT_EKF_BIAS_RW_MPS2_PER_S2          0.0004f  /* slow, deliberately small drift rate */

/* Accel-magnitude outlier clamp (2026-08-30) - real flight telemetry
 * (liftoff_capture_20260830_velgate.txt) showed a single decimated print
 * sample with raw az=3.0g (vs. a normal ~1.0g hover baseline) producing a
 * ~19 m/s^2 net (gravity-removed) vertical acceleration, which this
 * function's un-gated integration turned into a one-sample fused-height
 * glitch to 1.64m and vz to +5.76 m/s/-4.01 m/s - confirmed NOT a real height
 * change because lidar/sonar stayed rock-steady at ~0.47-0.52m through the
 * entire event (both are separate sensors, immune to an IMU-side fault, and
 * update fast enough to have caught a real 1m+ excursion). accel_bias
 * (g_x[2]) also got kicked ~10x outside its normal range in the same instant,
 * since the scalar height update's Kalman gain couples height/velocity
 * innovation straight into the bias state.
 * Every capture across this whole session (this session's calmest ALTHOLD
 * hovering through its most actively-flown moments, pitch/roll up to ~10deg)
 * never showed anything BUT this handful of spike events reach anywhere near
 * this magnitude - real commanded maneuvering accel for this airframe/flying
 * style stays well under it, so this clamp only ever engages on the kind of
 * single-sample outlier demonstrated above, not genuine flight dynamics.
 * Clamping (not rejecting outright) still lets a real, if unusually hard,
 * bump register as SOME acceleration rather than silently vanishing - it
 * just can't inject an unbounded one-sample velocity/height kick anymore. */
#define VERT_EKF_ACCEL_CLAMP_MPS2             8.0f

/* --- Reset covariance --- */
#define VERT_EKF_INIT_P_HEIGHT_M2             1.0f
#define VERT_EKF_INIT_P_VEL_M2S2              1.0f
#define VERT_EKF_INIT_P_BIAS_M2S4             0.01f

/* --- GPS bias trim / divergence watchdog --- */
#define VERT_EKF_GPS_TRIM_TAU_S               30.0f
/* Below this the receiver's own reported vertical accuracy is too coarse to use even
 * for a slow trim. */
#define VERT_EKF_GPS_MIN_VACC_M               15.0f
#define VERT_EKF_GPS_DIVERGENCE_SIGMA_MULT    3.0f
#define VERT_EKF_GPS_DIVERGENCE_MIN_M         3.0f
#define VERT_EKF_GPS_DIVERGENCE_DWELL_MS      10000U

static float g_x[3]; /* [0]=height_m, [1]=vz_mps, [2]=accel_bias_mps2 */
static float g_P[3][3];
static uint8_t g_initialized = 0U;
static uint32_t g_reset_ms = 0U; /* HAL_GetTick() at last VertEkf_Reset() - see
                                   * VERT_EKF_BARO_POST_RESET_SETTLE_MS */

/* GPS bias trim state - deliberately NOT part of g_x/g_P. See VertEkf_UpdateGps()'s
 * header comment in vert_ekf.h for why GPS never becomes a formal EKF measurement:
 * folding it into the state/covariance would let a single bad fix act like a real
 * measurement update, exactly what the design explicitly avoids. */
static float g_gps_baro_bias_trim_m = 0.0f;
static uint8_t g_gps_alt_ref_captured = 0U;
static float g_gps_alt_ref_m = 0.0f;
static uint32_t g_gps_last_update_ms = 0U;
static uint32_t g_gps_divergence_since_ms = 0U;
static uint8_t g_gps_divergence_fault = 0U;

/* Cross-validation state - most recent implied height + R from each range sensor, so
 * a fresh reading from ONE can be checked against the OTHER's still-recent value. */
static float g_lidar_last_implied_height_m = 0.0f;
static float g_lidar_last_r = 0.0f;
static uint32_t g_lidar_last_update_ms = 0U;
static float g_sonar_last_implied_height_m = 0.0f;
static float g_sonar_last_r = 0.0f;
static uint32_t g_sonar_last_update_ms = 0U;

void VertEkf_Init(void)
{
  VertEkf_Reset();
}

void VertEkf_Reset(void)
{
  int i;
  int j;

  g_x[0] = 0.0f;
  g_x[1] = 0.0f;
  g_x[2] = 0.0f;
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      g_P[i][j] = 0.0f;
    }
  }
  g_P[0][0] = VERT_EKF_INIT_P_HEIGHT_M2;
  g_P[1][1] = VERT_EKF_INIT_P_VEL_M2S2;
  g_P[2][2] = VERT_EKF_INIT_P_BIAS_M2S4;
  g_initialized = 1U;
  g_reset_ms = HAL_GetTick();

  g_gps_baro_bias_trim_m = 0.0f;
  g_gps_alt_ref_captured = 0U;
  g_gps_last_update_ms = 0U;
  g_gps_divergence_since_ms = 0U;
  g_gps_divergence_fault = 0U;

  g_lidar_last_update_ms = 0U;
  g_sonar_last_update_ms = 0U;
}

void VertEkf_Predict(float accel_up_mps2, float dt_s)
{
  float accel_corrected;
  float dt2;
  float dt3;
  float sigma_a2;
  float FP[3][3];
  int i;
  int j;

  if (g_initialized == 0U)
  {
    VertEkf_Reset();
  }
  if (dt_s < 0.0001f)
  {
    dt_s = 0.0001f; /* guard against a zero/stalled-clock dt */
  }

  /* See VERT_EKF_ACCEL_CLAMP_MPS2's comment - reject a single-sample
   * accelerometer outlier before it can inject an unbounded velocity/height
   * kick, rather than trusting every raw sample at face value. */
  if (accel_up_mps2 > VERT_EKF_ACCEL_CLAMP_MPS2)
  {
    accel_up_mps2 = VERT_EKF_ACCEL_CLAMP_MPS2;
  }
  else if (accel_up_mps2 < -VERT_EKF_ACCEL_CLAMP_MPS2)
  {
    accel_up_mps2 = -VERT_EKF_ACCEL_CLAMP_MPS2;
  }

  accel_corrected = accel_up_mps2 - g_x[2];

  g_x[0] += (g_x[1] * dt_s) + (0.5f * accel_corrected * dt_s * dt_s);
  g_x[1] += accel_corrected * dt_s;
  /* g_x[2] (accel bias): no deterministic term - pure random walk, driven only by
   * the process noise added to g_P[2][2] below. */

  /* P = F*P*F^T + Q, hand-expanded for
   * F = [[1, dt, -0.5*dt^2], [0, 1, -dt], [0, 0, 1]] (the Jacobian of the state
   * update above w.r.t. the state itself). Computed as (F*P) first, then
   * (F*P)*F^T, to avoid a generic loop-based matmul for a fixed 3x3 - see the
   * design comment at the top of this file for why.
   *
   * BUG FIXED 2026-08-29 (found via a real-liftoff telemetry capture showing
   * accel_bias pinned at exactly 0.0 for an entire flight): the second loop
   * below multiplied FP by F again instead of by F^T. Since F is not
   * symmetric that is a different matrix, not an equivalent shortcut - it
   * silently zeroed P[2][0]/P[2][1] (bias's covariance with height/velocity)
   * on every call, which are the ONLY route by which a measurement update's
   * Kalman gain (K[2] = P[2][0]/S) can ever become nonzero. With the bug,
   * K[2] was provably always exactly zero, so accel_bias could never be
   * corrected by any measurement, ever - confirmed by the flat telemetry
   * trace, not just by inspection. Column 0 must use F's ROW 0
   * [1, dt, -0.5dt^2] (i.e. F^T's column 0), column 1 must use F's row 1
   * [0, 1, -dt], and column 2 must use F's row 2 [0, 0, 1] (i.e. unchanged) -
   * the fix below applies those instead of re-applying F's own columns. */
  dt2 = dt_s * dt_s;
  dt3 = dt2 * dt_s;

  for (j = 0; j < 3; j++)
  {
    FP[0][j] = g_P[0][j] + (dt_s * g_P[1][j]) - (0.5f * dt2 * g_P[2][j]);
    FP[1][j] = g_P[1][j] - (dt_s * g_P[2][j]);
    FP[2][j] = g_P[2][j];
  }
  for (i = 0; i < 3; i++)
  {
    g_P[i][0] = FP[i][0] + (dt_s * FP[i][1]) - (0.5f * dt2 * FP[i][2]);
    g_P[i][1] = FP[i][1] - (dt_s * FP[i][2]);
    g_P[i][2] = FP[i][2];
  }

  sigma_a2 = VERT_EKF_ACCEL_NOISE_MPS2 * VERT_EKF_ACCEL_NOISE_MPS2;
  g_P[0][0] += sigma_a2 * dt3 / 3.0f;
  g_P[0][1] += sigma_a2 * dt2 / 2.0f;
  g_P[1][0] += sigma_a2 * dt2 / 2.0f;
  g_P[1][1] += sigma_a2 * dt_s;
  g_P[2][2] += VERT_EKF_BIAS_RW_MPS2_PER_S2 * dt_s;
}

/* Every measurement in this file (baro, lidar, sonar) reduces to "here is an
 * independent estimate of height" after pre-processing - see the design comment at
 * the top of this file for why that means one shared H=[1,0,0] update instead of
 * three separate measurement models. */
static void VertEkf_ScalarHeightUpdate(float z, float R)
{
  float innovation;
  float S;
  float K[3];
  int i;
  int j;

  if (g_initialized == 0U)
  {
    VertEkf_Reset();
  }
  if (R <= 0.0f)
  {
    R = 1e-6f; /* R should never legitimately be <=0 - numerical safety only */
  }

  innovation = z - g_x[0];
  S = g_P[0][0] + R;
  if (S <= 0.0f)
  {
    return; /* shouldn't happen with a sane P - numerical safety only */
  }

  K[0] = g_P[0][0] / S;
  K[1] = g_P[1][0] / S;
  K[2] = g_P[2][0] / S;

  g_x[0] += K[0] * innovation;
  g_x[1] += K[1] * innovation;
  g_x[2] += K[2] * innovation;

  /* P = (I - K*H)*P simplifies to P[i][j] -= K[i]*P[0][j] when H=[1,0,0] - no need
   * for a general matrix subtract. */
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      g_P[i][j] -= K[i] * g_P[0][j];
    }
  }
}

/* Severity-scaled cross-check R penalty (2026-08-29, replaces a flat multiplier -
 * see VERT_EKF_CROSSCHECK_R_PENALTY's comment for the real-flight bug this fixes).
 * base_penalty applies exactly AT the trigger threshold (sigma_mult sigma) and grows
 * quadratically for anything further beyond it - a NIS/chi-squared-style scaling, so
 * a borderline 2-sigma disagreement still gets the same treatment as before, but a
 * blatant 27-sigma one (an obviously faulty reading, not a marginal call) gets
 * inflated by a proportionally enormous amount instead of the same fixed 8x. */
static float VertEkf_CrossCheckPenalty(float disagreement, float combined_sigma,
                                        float sigma_mult, float base_penalty)
{
  float sigma_ratio;
  float excess_ratio;

  if (combined_sigma <= 0.0f)
  {
    return base_penalty; /* numerical safety only - combined_sigma is a sum of two R's */
  }
  sigma_ratio = disagreement / combined_sigma;
  excess_ratio = sigma_ratio / sigma_mult; /* 1.0 exactly at the trigger threshold */
  return base_penalty * excess_ratio * excess_ratio;
}

void VertEkf_UpdateBaro(float raw_alt_m, uint8_t baro_healthy, float motor_power_delta_us)
{
  float thrust_factor;
  float height_factor;
  float ground_effect_severity;
  float r;

  if (baro_healthy == 0U)
  {
    return;
  }

  thrust_factor = motor_power_delta_us / VERT_EKF_GROUND_EFFECT_THRUST_REF_US;
  if (thrust_factor < 0.0f) { thrust_factor = 0.0f; }
  if (thrust_factor > 1.0f) { thrust_factor = 1.0f; }

  height_factor = 1.0f - (g_x[0] / VERT_EKF_GROUND_EFFECT_ZONE_M);
  if (height_factor < 0.0f) { height_factor = 0.0f; }
  if (height_factor > 1.0f) { height_factor = 1.0f; }

  ground_effect_severity = thrust_factor * height_factor;

  r = VERT_EKF_BARO_R_BASE_M2 * (1.0f + (VERT_EKF_BARO_GROUND_EFFECT_R_GAIN * ground_effect_severity));

  /* Cross-check against whichever range sensor has the freshest still-valid
   * reading - see VERT_EKF_BARO_CROSSCHECK_SIGMA_MULT's comment above. Picks
   * the more recent of the two if both are fresh, same tie-break the
   * range-vs-range check would reach via its own freshness test. */
  {
    uint32_t now_ms = HAL_GetTick();
    uint8_t have_lidar = (g_lidar_last_update_ms != 0U) &&
                         ((now_ms - g_lidar_last_update_ms) < VERT_EKF_CROSSCHECK_MAX_AGE_MS);
    uint8_t have_sonar = (g_sonar_last_update_ms != 0U) &&
                         ((now_ms - g_sonar_last_update_ms) < VERT_EKF_CROSSCHECK_MAX_AGE_MS);
    float ref_height = 0.0f;
    float ref_r = 0.0f;
    uint8_t have_ref = 0U;

    if (have_lidar && have_sonar)
    {
      if (g_lidar_last_update_ms >= g_sonar_last_update_ms)
      {
        ref_height = g_lidar_last_implied_height_m;
        ref_r = g_lidar_last_r;
      }
      else
      {
        ref_height = g_sonar_last_implied_height_m;
        ref_r = g_sonar_last_r;
      }
      have_ref = 1U;
    }
    else if (have_lidar)
    {
      ref_height = g_lidar_last_implied_height_m;
      ref_r = g_lidar_last_r;
      have_ref = 1U;
    }
    else if (have_sonar)
    {
      ref_height = g_sonar_last_implied_height_m;
      ref_r = g_sonar_last_r;
      have_ref = 1U;
    }

    if (have_ref != 0U)
    {
      float combined_sigma = sqrtf(r + ref_r);
      float disagreement = fabsf(raw_alt_m - ref_height);
      if (disagreement > (VERT_EKF_BARO_CROSSCHECK_SIGMA_MULT * combined_sigma))
      {
        r *= VertEkf_CrossCheckPenalty(disagreement, combined_sigma,
                                        VERT_EKF_BARO_CROSSCHECK_SIGMA_MULT,
                                        VERT_EKF_BARO_CROSSCHECK_R_PENALTY);
      }
    }

    /* Post-reset settle boost - see its comment above. Stacks multiplicatively
     * with both ground-effect inflation and the cross-check, same pattern as
     * the range update's confidence/tilt/fade/cross-check terms all stacking. */
    {
      uint32_t elapsed_ms = now_ms - g_reset_ms;
      if (elapsed_ms < VERT_EKF_BARO_POST_RESET_SETTLE_MS)
      {
        float frac = ((float)elapsed_ms) / ((float)VERT_EKF_BARO_POST_RESET_SETTLE_MS);
        float settle_shape = 0.5f + (0.5f * cosf(frac * 3.14159265f));
        r *= (1.0f + (VERT_EKF_BARO_POST_RESET_R_GAIN * settle_shape));
      }
    }
  }

  VertEkf_ScalarHeightUpdate(raw_alt_m, r);
}

void VertEkf_UpdateRange(float raw_range_cm, float confidence, uint8_t is_lidar, uint8_t valid)
{
  float range_m;
  float gx;
  float gy;
  float gz;
  float arm_x_m;
  float arm_y_m;
  float max_range_m;
  float fade_start_m;
  float fade_r_gain;
  float r_base;
  float implied_height_m;
  float r;
  float conf_clamped;
  float gz_floor;
  uint32_t now_ms;

  /* Trust the explicit flag, not raw_range_cm<=0.0f - see this function's header
   * comment in vert_ekf.h for why that used to silently drop genuine 0cm readings.
   * raw_range_cm<0.0f kept as a numerical safety net only - the wire protocol should
   * never send a negative range regardless of valid. */
  if ((valid == 0U) || (raw_range_cm < 0.0f))
  {
    return;
  }

  range_m = raw_range_cm * 0.01f;
  now_ms = HAL_GetTick();

  Attitude_GetWorldUpInBodyFrame(&gx, &gy, &gz);

  if (is_lidar != 0U)
  {
    arm_x_m = VERT_EKF_LIDAR_ARM_X_M;
    arm_y_m = VERT_EKF_LIDAR_ARM_Y_M;
    max_range_m = VERT_EKF_LIDAR_MAX_RANGE_M;
    fade_start_m = VERT_EKF_LIDAR_FADE_START_M;
    fade_r_gain = VERT_EKF_LIDAR_FADE_R_GAIN;
    r_base = VERT_EKF_LIDAR_R_BASE_M2;
  }
  else
  {
    arm_x_m = VERT_EKF_SONAR_ARM_X_M;
    arm_y_m = VERT_EKF_SONAR_ARM_Y_M;
    max_range_m = VERT_EKF_SONAR_MAX_RANGE_M;
    fade_start_m = VERT_EKF_SONAR_FADE_START_M;
    fade_r_gain = VERT_EKF_SONAR_FADE_R_GAIN;
    r_base = VERT_EKF_SONAR_R_BASE_M2;

    /* Sonar-specific hard tilt reject (per spec - NOT applied to lidar, not
     * requested and lidar's narrow optical beam doesn't share the same
     * off-axis/multipath echo failure mode a wide acoustic cone does at high
     * tilt). */
    if (gz < VERT_EKF_SONAR_TILT_REJECT_GZ_MIN)
    {
      return;
    }
  }

  if (range_m >= max_range_m)
  {
    return; /* hard sensor-range ceiling, not a soft-confidence situation */
  }

  /* Lever-arm + tilt projection - see the design comment at the top of this file
   * (and VERT_EKF_LIDAR_ARM_X_M's comment for the unverified sign caveat). gz alone
   * (no arm term) is the standard center-mounted tilt compensation; the arm term
   * corrects for this specific sensor not actually being at the FC's own reference
   * point. */
  implied_height_m = (range_m * gz) - ((arm_x_m * gx) + (arm_y_m * gy));

  conf_clamped = confidence;
  if (conf_clamped < VERT_EKF_CONFIDENCE_FLOOR) { conf_clamped = VERT_EKF_CONFIDENCE_FLOOR; }
  if (conf_clamped > 1.0f) { conf_clamped = 1.0f; }

  /* Confidence scales R inversely-squared (halved confidence = 4x variance) - a
   * simple, defensible mapping for a 0-1 quality score with no sensor-specific
   * calibration data yet to justify anything more elaborate. */
  r = r_base / (conf_clamped * conf_clamped);

  /* Tilt inflates the EFFECTIVE height-measurement noise too, not just the
   * projection itself - the same raw range noise translates to more height
   * uncertainty at a steeper angle. Floored well above zero so a near-90deg tilt
   * (gz->0) inflates R sharply without a divide-by-zero. */
  gz_floor = fabsf(gz);
  if (gz_floor < 0.15f) { gz_floor = 0.15f; }
  r /= (gz_floor * gz_floor);

  /* Soft fade approaching this sensor's range ceiling (per spec: "soft fade, not a
   * cliff") - raised-cosine ramp from 1x at fade_start_m to (1+fade_r_gain)x right
   * at the ceiling, so the Kalman gain hands off to whatever else is available
   * without a discontinuity in the estimate - see the design comment at the top of
   * this file for why this replaces an explicit "handoff" mechanism entirely. */
  if (range_m > fade_start_m)
  {
    float fade_frac = (range_m - fade_start_m) / (max_range_m - fade_start_m);
    float fade_shape;
    if (fade_frac > 1.0f) { fade_frac = 1.0f; }
    fade_shape = 0.5f - (0.5f * cosf(fade_frac * 3.14159265f));
    r *= (1.0f + (fade_r_gain * fade_shape));
  }

  /* Cross-validate against the OTHER range sensor's still-recent reading in the
   * 0-3m overlap band (per spec) - if they disagree beyond ~2-sigma combined,
   * penalize whichever ONE disagrees more with the filter's own current
   * (pre-update) estimate, rather than feeding both in at face value and
   * effectively averaging a bad reading in. This trusts the filter's recent state
   * as the tie-breaker, which is reasonable for an isolated single-sensor fault
   * (the realistic failure mode here) but not foolproof if the filter has already
   * diverged for some unrelated reason.
   *
   * BUG FIXED 2026-08-30 (found via a real flight capture,
   * liftoff_capture_20260830_accelclamp.txt): this used to gate on ONLY the
   * fresh reading's own range_m being under the band, so a spurious FAR
   * reading (e.g. a bad TF-Luna sample claiming ~4m while the aircraft was
   * genuinely at ~0.4m) skipped the cross-check entirely by construction, even
   * though sonar had a perfectly good, current ~0.4m reading sitting right
   * there that could have caught it. Confirmed in that capture: lidar=4.05m,
   * sonar=0.47m at the same instant, fused height pulled from ~0.43m to 1.07m
   * in one update because the false-far reading got treated as fully trusted
   * (its own R schedule has no reason to distrust a 4m reading, and the
   * cross-check that could have distrusted it never ran). Now triggers
   * whenever EITHER sensor's reading is within the band, not just the new
   * one's - a false-far reading is exactly the case a valid nearby
   * corroborating reading should be able to challenge, and there is no reason
   * to exempt it just because the false reading itself claims to be far
   * away. */
  {
    float other_height;
    float other_r;
    uint32_t other_ms;
    uint8_t either_in_band;

    if (is_lidar != 0U)
    {
      other_height = g_sonar_last_implied_height_m;
      other_r = g_sonar_last_r;
      other_ms = g_sonar_last_update_ms;
    }
    else
    {
      other_height = g_lidar_last_implied_height_m;
      other_r = g_lidar_last_r;
      other_ms = g_lidar_last_update_ms;
    }

    either_in_band = (range_m < VERT_EKF_CROSSCHECK_MAX_M) ||
                      (fabsf(other_height) < VERT_EKF_CROSSCHECK_MAX_M);

    if ((either_in_band != 0U) && (other_ms != 0U) &&
        ((now_ms - other_ms) < VERT_EKF_CROSSCHECK_MAX_AGE_MS))
    {
      float combined_sigma = sqrtf(r + other_r);
      float disagreement = fabsf(implied_height_m - other_height);
      if (disagreement > (VERT_EKF_CROSSCHECK_SIGMA_MULT * combined_sigma))
      {
        float this_err = fabsf(implied_height_m - g_x[0]);
        float other_err = fabsf(other_height - g_x[0]);
        if (this_err > other_err)
        {
          r *= VertEkf_CrossCheckPenalty(disagreement, combined_sigma,
                                          VERT_EKF_CROSSCHECK_SIGMA_MULT,
                                          VERT_EKF_CROSSCHECK_R_PENALTY);
        }
      }
    }
  }

  /* Remember this reading for the OTHER sensor's cross-check on ITS next update -
   * stored after the check above so this reading never checks against itself. */
  if (is_lidar != 0U)
  {
    g_lidar_last_implied_height_m = implied_height_m;
    g_lidar_last_r = r;
    g_lidar_last_update_ms = now_ms;
  }
  else
  {
    g_sonar_last_implied_height_m = implied_height_m;
    g_sonar_last_r = r;
    g_sonar_last_update_ms = now_ms;
  }

  VertEkf_ScalarHeightUpdate(implied_height_m, r);
}

void VertEkf_UpdateGps(float gps_alt_m, float gps_vacc_m, uint8_t gps_healthy, uint8_t armed)
{
  float gps_relative_alt_m;
  float dt_s;
  float alpha;
  float divergence;
  float divergence_threshold_m;
  uint32_t now_ms;

  if ((gps_healthy == 0U) || (gps_vacc_m <= 0.0f) || (gps_vacc_m > VERT_EKF_GPS_MIN_VACC_M))
  {
    /* Not trustworthy enough even for a slow trim - and don't let a stale
     * reference silently persist across a long GPS dropout either. */
    g_gps_alt_ref_captured = 0U;
    return;
  }

  now_ms = HAL_GetTick();

  if (g_gps_alt_ref_captured == 0U)
  {
    /* Capture a fresh "GPS altitude at this reference moment" the instant GPS
     * becomes trustworthy - GPS altitude is MSL, baro/range are height above
     * wherever the aircraft armed, so only DRIFT relative to this reference is
     * ever meaningful, never the absolute value. */
    g_gps_alt_ref_m = gps_alt_m;
    g_gps_alt_ref_captured = 1U;
    g_gps_last_update_ms = now_ms;
    return;
  }

  dt_s = ((float)(now_ms - g_gps_last_update_ms)) * 0.001f;
  if (dt_s < 0.001f) { dt_s = 0.001f; }
  g_gps_last_update_ms = now_ms;

  gps_relative_alt_m = gps_alt_m - g_gps_alt_ref_m;

  /* Long-time-constant trim: slowly walks g_gps_baro_bias_trim_m so that
   * (fused height - trim) tracks GPS's relative altitude over tens of seconds,
   * correcting baro's slow drift without ever treating one GPS fix like a real
   * fast-state measurement. */
  alpha = dt_s / VERT_EKF_GPS_TRIM_TAU_S;
  if (alpha > 1.0f) { alpha = 1.0f; } /* guard against overcorrecting in one step
                                        * after a long GPS dropout */

  divergence = (g_x[0] - g_gps_baro_bias_trim_m) - gps_relative_alt_m;
  g_gps_baro_bias_trim_m += alpha * divergence;

  /* Divergence/failsafe cross-check - separate from, and much less patient than,
   * the slow trim above. Only evaluated once armed - pre-arm, the fused estimate
   * and a fresh GPS reference have no reason to agree yet. */
  if (armed != 0U)
  {
    divergence_threshold_m = VERT_EKF_GPS_DIVERGENCE_SIGMA_MULT * gps_vacc_m;
    if (divergence_threshold_m < VERT_EKF_GPS_DIVERGENCE_MIN_M)
    {
      divergence_threshold_m = VERT_EKF_GPS_DIVERGENCE_MIN_M;
    }

    if (fabsf(divergence) > divergence_threshold_m)
    {
      if (g_gps_divergence_since_ms == 0U)
      {
        g_gps_divergence_since_ms = now_ms;
      }
      else if ((now_ms - g_gps_divergence_since_ms) >= VERT_EKF_GPS_DIVERGENCE_DWELL_MS)
      {
        g_gps_divergence_fault = 1U;
      }
    }
    else
    {
      g_gps_divergence_since_ms = 0U;
      g_gps_divergence_fault = 0U;
    }
  }
}

float VertEkf_GetHeightM(void)
{
  return g_x[0] - g_gps_baro_bias_trim_m;
}

float VertEkf_GetClimbRateMps(void)
{
  return g_x[1];
}

float VertEkf_GetAccelBiasMps2(void)
{
  return g_x[2];
}

float VertEkf_GetGroundEffectZoneM(void)
{
  return VERT_EKF_GROUND_EFFECT_ZONE_M;
}

uint8_t VertEkf_IsHealthy(void)
{
  return (uint8_t)(g_gps_divergence_fault == 0U);
}

float VertEkf_GetLidarImpliedHeightM(void)
{
  return g_lidar_last_implied_height_m;
}

float VertEkf_GetSonarImpliedHeightM(void)
{
  return g_sonar_last_implied_height_m;
}
