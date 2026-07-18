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
}

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

  *pitch_deg = Attitude_WrapAngle180(atan2f(g_x, pitch_denom) * DEG_PER_RAD);
  *roll_deg = Attitude_WrapAngle180(atan2f(-g_y, roll_denom) * DEG_PER_RAD);

  yaw = -atan2f(2.0f * ((g_ahrs.q0 * g_ahrs.q3) + (g_ahrs.q1 * g_ahrs.q2)),
                1.0f - 2.0f * ((g_ahrs.q2 * g_ahrs.q2) + (g_ahrs.q3 * g_ahrs.q3))) * DEG_PER_RAD;
  *yaw_deg = Attitude_WrapAngle180(yaw);
}
