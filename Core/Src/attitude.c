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
#define ATTITUDE_ACCEL_TRUST_MAX_DEV_G 0.60f
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
/* Cutoff for the trust-gating magnitude filter - fast enough to still gate
 * out a real sustained tilt/acceleration event within ~200ms, slow enough to
 * average out prop/motor vibration (tens of Hz) so a level hover reads as
 * near-1g on average despite noisy instantaneous samples. */
#define ATTITUDE_ACCEL_MAG_LPF_HZ 5.0f

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

void Attitude_GetBoardAnglesDeg(float *pitch_deg,
                                float *roll_deg,
                                float *yaw_deg)
{
  float g_x;
  float g_y;
  float g_z;
  float pitch_denom;
  float roll_denom;
  float yaw;

  g_x = 2.0f * ((g_ahrs.q1 * g_ahrs.q3) - (g_ahrs.q0 * g_ahrs.q2));
  g_y = 2.0f * ((g_ahrs.q0 * g_ahrs.q1) + (g_ahrs.q2 * g_ahrs.q3));
  g_z = (g_ahrs.q0 * g_ahrs.q0) - (g_ahrs.q1 * g_ahrs.q1) - (g_ahrs.q2 * g_ahrs.q2) + (g_ahrs.q3 * g_ahrs.q3);

  pitch_denom = sqrtf((g_y * g_y) + (g_z * g_z));
  roll_denom = sqrtf((g_x * g_x) + (g_z * g_z));

  /* Negated vs. the raw gravity-vector components so pitch/roll increase in the
   * same sense as gx_dps/gy_dps (the unflipped rate convention used everywhere
   * else, e.g. RATE mode's measured_roll/pitch_rate_dps and ATTITUDE mode's
   * target_roll/pitch_deg) - without this, ATTITUDE mode's angle error runs
   * away in the same direction as a real disturbance instead of opposing it. */
  *pitch_deg = Attitude_WrapAngle180(atan2f(-g_x, pitch_denom) * DEG_PER_RAD);
  *roll_deg = Attitude_WrapAngle180(atan2f(g_y, roll_denom) * DEG_PER_RAD);

  yaw = -atan2f(2.0f * ((g_ahrs.q0 * g_ahrs.q3) + (g_ahrs.q1 * g_ahrs.q2)),
                1.0f - 2.0f * ((g_ahrs.q2 * g_ahrs.q2) + (g_ahrs.q3 * g_ahrs.q3))) * DEG_PER_RAD;
  *yaw_deg = Attitude_WrapAngle180(yaw);
}
