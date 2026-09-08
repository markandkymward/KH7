#include "horiz_ekf.h"

#include "nav.h"

#include <math.h>

/* Horizontal-plane state estimator (2026-09-05) - north/east position,
 * velocity, and accelerometer bias, predicted from IMU horizontal
 * acceleration at the control-loop rate and corrected by GPS position+
 * velocity measurements at GPS's own native (~3-5Hz) rate. Replaces nav.c's
 * simple exponential low-pass filter of raw GPS position/velocity
 * (filtered_north_m/east_m, filtered_north_vel_mps/east_vel_mps).
 *
 * WHY: an entire session of NAVPOSHOLD flight testing (see
 * kh7-gps-data-and-poshold-design memory) kept finding the hold's correction
 * arriving a beat late even after repeated control-loop gain/authority
 * increases - researching what ArduPilot (AP_NavEKF3) and iNav
 * (navigation_pos_estimator.c) do for the same problem confirmed neither
 * low-pass-filters raw GPS directly: both integrate the accelerometer
 * continuously between GPS fixes (dead reckoning) and use each GPS sample as
 * a CORRECTION to that continuously-updating estimate, not as the estimate's
 * only source of new information. A low-pass filter can only ever react to
 * GPS data that has already arrived - dead reckoning fills the gap between
 * fixes with real physics instead of extrapolating/waiting. This is the
 * horizontal-plane analog of vert_ekf.c, which already does exactly this for
 * the vertical axis (baro/range instead of GPS providing the corrections) -
 * see that file's own design-choice comments for the shared reasoning
 * (hand-rolled scalar Kalman math, no matrix library, at this state count).
 *
 * DESIGN CHOICE - two independent 3-state scalar filters (north, east), not
 * one coupled state: north/east position and velocity have no physical
 * cross-coupling in this problem (no Coriolis terms worth modeling at
 * multirotor speeds, no shared process/measurement noise term between the
 * axes) - exactly the same reasoning vert_ekf.c gives for treating height as
 * one clean scalar problem. Each axis's state is [pos_m, vel_mps,
 * accel_bias_mps2], identical in structure to vert_ekf's [height, vz,
 * accel_bias].
 *
 * DESIGN CHOICE - GPS IS a real, primary measurement here, unlike
 * vert_ekf.c's GPS handling: vert_ekf.c deliberately keeps GPS as a slow
 * bias-trim + divergence watchdog ONLY, because baro/range are both
 * available and more accurate for height. There is no horizontal equivalent
 * sensor on this airframe - GPS is the ONLY absolute horizontal reference
 * that exists, so it must be treated as a genuine Kalman measurement update
 * (position AND velocity), not merely a trim.
 *
 * DESIGN CHOICE - no cross-axis outlier/divergence watchdog (yet): vert_ekf.c
 * has one because it has two independent range sensors to cross-check against
 * each other, plus GPS as a third opinion. This file has exactly one sensor
 * (GPS) feeding two independent axes - there's nothing else to cross-check a
 * bad GPS sample against except nav.c's own pre-existing fix-type/accuracy/
 * jump gates, which already run before HorizEkf_UpdateGps() is ever called.
 *
 * UNTUNED, BY DESIGN, PENDING REAL DATA - same disclaimer as vert_ekf.c: every
 * noise constant below is a physically-reasoned starting point (GPS R from
 * this session's own logged hAcc/sAcc, process noise a defensible guess), not
 * yet validated by replaying a real logged flight through this filter
 * offline. UNFLOWN as of this commit. */

#define HORIZ_EKF_AXIS_NORTH  0
#define HORIZ_EKF_AXIS_EAST   1

/* --- Process noise --- */
/* Slightly higher than vert_ekf's 0.5 - horizontal disturbance (wind, prop
 * wash side-loading) is less bounded than vertical thrust-vs-gravity, and
 * this file has no second sensor to lean on if this is set too low the way
 * vert_ekf can lean on range when baro's own noise model is wrong. Untuned
 * starting point, same caveat as the rest of this file. */
#define HORIZ_EKF_ACCEL_NOISE_MPS2         0.8f
#define HORIZ_EKF_BIAS_RW_MPS2_PER_S2      0.0004f  /* same slow drift rate as vert_ekf */

/* Accel-magnitude outlier clamp, mirroring VERT_EKF_ACCEL_CLAMP_MPS2's
 * incident (a single decimated-telemetry outlier sample producing an
 * un-gated ~19 m/s^2 spike). Set above APP_NAVPOS_MAX_ACCEL_MPS2 (8.0, the
 * hold's own commanded ceiling) plus real maneuvering headroom, so this only
 * ever engages on a genuine sensor-glitch magnitude, not commanded flight. */
#define HORIZ_EKF_ACCEL_CLAMP_MPS2         14.0f

/* --- GPS measurement noise --- */
/* R = h_acc_m^2 directly - h_acc_m IS the receiver's own reported 1-sigma
 * horizontal position uncertainty, no additional scaling needed (unlike
 * vert_ekf's range sensors, which need noise MODELED because they report a
 * raw range, not an accuracy estimate). Floored so a receiver momentarily
 * reporting an implausibly tiny hAcc can't make the filter over-trust one
 * sample completely. */
#define HORIZ_EKF_GPS_POS_R_FLOOR_M2       (0.15f * 0.15f)
/* Same floor logic for velocity R; s_acc_mps is 0 or absent on some
 * receivers/messages (this module's own quirk history - see
 * kh7-gps-m100pro-cfgprt-quirk memory for its pattern of incomplete
 * fields on otherwise-working messages), so this is also the fallback value
 * used whenever s_acc_mps <= 0. GPS Doppler velocity is normally quite
 * accurate (better than position, in most u-blox receivers) - 0.15 m/s is a
 * conservative untuned default, not a measured figure for this module. */
#define HORIZ_EKF_GPS_VEL_R_DEFAULT_M2S2   (0.15f * 0.15f)

/* --- Reset covariance --- */
#define HORIZ_EKF_INIT_P_POS_M2            1.0f
#define HORIZ_EKF_INIT_P_VEL_M2S2          1.0f
#define HORIZ_EKF_INIT_P_BIAS_M2S4         0.01f

typedef struct
{
  float x[3]; /* [0]=pos_m, [1]=vel_mps, [2]=accel_bias_mps2 */
  float P[3][3];
} HorizEkfAxis_t;

static HorizEkfAxis_t g_axis[2]; /* [HORIZ_EKF_AXIS_NORTH], [HORIZ_EKF_AXIS_EAST] */
static uint8_t g_initialized = 0U;

static void HorizEkf_ResetAxis(HorizEkfAxis_t *a)
{
  int i;
  int j;

  a->x[0] = 0.0f;
  a->x[1] = 0.0f;
  a->x[2] = 0.0f;
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      a->P[i][j] = 0.0f;
    }
  }
  a->P[0][0] = HORIZ_EKF_INIT_P_POS_M2;
  a->P[1][1] = HORIZ_EKF_INIT_P_VEL_M2S2;
  a->P[2][2] = HORIZ_EKF_INIT_P_BIAS_M2S4;
}

void HorizEkf_Init(void)
{
  HorizEkf_Reset();
}

void HorizEkf_Reset(void)
{
  HorizEkf_ResetAxis(&g_axis[HORIZ_EKF_AXIS_NORTH]);
  HorizEkf_ResetAxis(&g_axis[HORIZ_EKF_AXIS_EAST]);
  g_initialized = 1U;
}

/* Identical structure to VertEkf_Predict() - see that file's comment on the
 * F/F^T bug it fixed once (2026-08-29) for exactly why the second covariance
 * loop below multiplies by F's ROWS (F^T's columns), not F's own columns
 * again. F = [[1, dt, -0.5*dt^2], [0, 1, -dt], [0, 0, 1]] either way, same
 * physical model (position/velocity kinematics driven by bias-corrected
 * accel, bias itself a pure random walk). */
static void HorizEkf_PredictAxis(HorizEkfAxis_t *a, float accel_mps2, float dt_s)
{
  float accel_corrected;
  float dt2;
  float dt3;
  float sigma_a2;
  float FP[3][3];
  int i;
  int j;

  if (accel_mps2 > HORIZ_EKF_ACCEL_CLAMP_MPS2)
  {
    accel_mps2 = HORIZ_EKF_ACCEL_CLAMP_MPS2;
  }
  else if (accel_mps2 < -HORIZ_EKF_ACCEL_CLAMP_MPS2)
  {
    accel_mps2 = -HORIZ_EKF_ACCEL_CLAMP_MPS2;
  }

  accel_corrected = accel_mps2 - a->x[2];

  a->x[0] += (a->x[1] * dt_s) + (0.5f * accel_corrected * dt_s * dt_s);
  a->x[1] += accel_corrected * dt_s;

  dt2 = dt_s * dt_s;
  dt3 = dt2 * dt_s;

  for (j = 0; j < 3; j++)
  {
    FP[0][j] = a->P[0][j] + (dt_s * a->P[1][j]) - (0.5f * dt2 * a->P[2][j]);
    FP[1][j] = a->P[1][j] - (dt_s * a->P[2][j]);
    FP[2][j] = a->P[2][j];
  }
  for (i = 0; i < 3; i++)
  {
    a->P[i][0] = FP[i][0] + (dt_s * FP[i][1]) - (0.5f * dt2 * FP[i][2]);
    a->P[i][1] = FP[i][1] - (dt_s * FP[i][2]);
    a->P[i][2] = FP[i][2];
  }

  sigma_a2 = HORIZ_EKF_ACCEL_NOISE_MPS2 * HORIZ_EKF_ACCEL_NOISE_MPS2;
  a->P[0][0] += sigma_a2 * dt3 / 3.0f;
  a->P[0][1] += sigma_a2 * dt2 / 2.0f;
  a->P[1][0] += sigma_a2 * dt2 / 2.0f;
  a->P[1][1] += sigma_a2 * dt_s;
  a->P[2][2] += HORIZ_EKF_BIAS_RW_MPS2_PER_S2 * dt_s;
}

/* Scalar update, H=[1,0,0] (position measurement) or effectively a second
 * independent scalar update with H=[0,1,0] (velocity measurement) - done as
 * two sequential scalar updates per axis per GPS sample (position then
 * velocity), same "one measurement model per call, no generic matrix" style
 * as vert_ekf's VertEkf_ScalarHeightUpdate(), just parameterized on which
 * state index H picks out. */
static void HorizEkf_ScalarUpdate(HorizEkfAxis_t *a, int state_idx, float z, float R)
{
  float innovation;
  float S;
  float K[3];
  int i;
  int j;

  if (R <= 0.0f)
  {
    R = 1e-6f;
  }

  innovation = z - a->x[state_idx];
  S = a->P[state_idx][state_idx] + R;
  if (S <= 0.0f)
  {
    return;
  }

  K[0] = a->P[0][state_idx] / S;
  K[1] = a->P[1][state_idx] / S;
  K[2] = a->P[2][state_idx] / S;

  a->x[0] += K[0] * innovation;
  a->x[1] += K[1] * innovation;
  a->x[2] += K[2] * innovation;

  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      a->P[i][j] -= K[i] * a->P[state_idx][j];
    }
  }
}

void HorizEkf_Predict(float accel_fwd_mps2, float accel_right_mps2, float yaw_deg, float dt_s)
{
  float accel_north_mps2;
  float accel_east_mps2;

  if (g_initialized == 0U)
  {
    HorizEkf_Reset();
  }
  if (dt_s < 0.0001f)
  {
    dt_s = 0.0001f;
  }

  /* Same body-yaw-frame -> NED rotation NAVPOSHOLD's own stick mapping
   * already uses (see Nav_RotateBodyToNed()'s own header comment) - reusing
   * it here, rather than hand-rolling a second sin/cos pair, guarantees this
   * estimator's NED frame can never quietly disagree with the control law
   * that consumes its output. */
  Nav_RotateBodyToNed(accel_fwd_mps2, accel_right_mps2, yaw_deg, &accel_north_mps2, &accel_east_mps2);

  HorizEkf_PredictAxis(&g_axis[HORIZ_EKF_AXIS_NORTH], accel_north_mps2, dt_s);
  HorizEkf_PredictAxis(&g_axis[HORIZ_EKF_AXIS_EAST], accel_east_mps2, dt_s);
}

void HorizEkf_UpdateGps(float north_m, float east_m, float vel_n_mps, float vel_e_mps,
                         float h_acc_m, float s_acc_mps)
{
  float pos_r;
  float vel_r;

  if (g_initialized == 0U)
  {
    HorizEkf_Reset();
  }

  pos_r = h_acc_m * h_acc_m;
  if (pos_r < HORIZ_EKF_GPS_POS_R_FLOOR_M2)
  {
    pos_r = HORIZ_EKF_GPS_POS_R_FLOOR_M2;
  }

  vel_r = (s_acc_mps > 0.0f) ? (s_acc_mps * s_acc_mps) : HORIZ_EKF_GPS_VEL_R_DEFAULT_M2S2;
  if (vel_r < HORIZ_EKF_GPS_VEL_R_DEFAULT_M2S2)
  {
    vel_r = HORIZ_EKF_GPS_VEL_R_DEFAULT_M2S2;
  }

  HorizEkf_ScalarUpdate(&g_axis[HORIZ_EKF_AXIS_NORTH], 0, north_m, pos_r);
  HorizEkf_ScalarUpdate(&g_axis[HORIZ_EKF_AXIS_NORTH], 1, vel_n_mps, vel_r);
  HorizEkf_ScalarUpdate(&g_axis[HORIZ_EKF_AXIS_EAST], 0, east_m, pos_r);
  HorizEkf_ScalarUpdate(&g_axis[HORIZ_EKF_AXIS_EAST], 1, vel_e_mps, vel_r);
}

float HorizEkf_GetNorthM(void) { return g_axis[HORIZ_EKF_AXIS_NORTH].x[0]; }
float HorizEkf_GetEastM(void) { return g_axis[HORIZ_EKF_AXIS_EAST].x[0]; }
float HorizEkf_GetNorthVelMps(void) { return g_axis[HORIZ_EKF_AXIS_NORTH].x[1]; }
float HorizEkf_GetEastVelMps(void) { return g_axis[HORIZ_EKF_AXIS_EAST].x[1]; }
float HorizEkf_GetAccelBiasNorthMps2(void) { return g_axis[HORIZ_EKF_AXIS_NORTH].x[2]; }
float HorizEkf_GetAccelBiasEastMps2(void) { return g_axis[HORIZ_EKF_AXIS_EAST].x[2]; }
