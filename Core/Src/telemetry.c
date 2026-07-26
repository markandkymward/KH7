#include "telemetry.h"

#include <stdio.h>

#define CRSF_CH_MIN_RAW 172U
#define CRSF_CH_MAX_RAW 1811U
#define PWM_MIN_US      988U
#define PWM_MAX_US      2012U

static uint16_t Telemetry_CrsfRawToUs(uint16_t raw)
{
  uint32_t scaled;

  if (raw <= CRSF_CH_MIN_RAW)
  {
    return PWM_MIN_US;
  }

  if (raw >= CRSF_CH_MAX_RAW)
  {
    return PWM_MAX_US;
  }

  scaled = (uint32_t)(raw - CRSF_CH_MIN_RAW) * (PWM_MAX_US - PWM_MIN_US);
  scaled = (scaled / (CRSF_CH_MAX_RAW - CRSF_CH_MIN_RAW)) + PWM_MIN_US;
  return (uint16_t)scaled;
}

void Telemetry_PrintImuLoggerStart(void)
{
  printf("IMU logger start on SPI4\r\n");
}

void Telemetry_PrintImuDetected(IMU_TypeDef type, uint8_t whoami)
{
  printf("IMU detected: type=%d whoami=0x%02X\r\n", (int)type, whoami);
}

void Telemetry_PrintImuDetectionFailed(void)
{
  printf("IMU detection failed on SPI4\r\n");
}

void Telemetry_PrintImuReadFailed(IMU_TypeDef type, uint8_t whoami)
{
  printf("IMU read failed (type=%d whoami=0x%02X)\r\n", (int)type, whoami);
}

void Telemetry_PrintImuState(float ax_g,
                             float ay_g,
                             float az_g,
                             float gx_dps,
                             float gy_dps,
                             float gz_dps,
                             float pitch_deg,
                             float roll_deg,
                             float yaw_deg)
{
  int32_t ax_cg;
  int32_t ay_cg;
  int32_t az_cg;
  int32_t gx_d10;
  int32_t gy_d10;
  int32_t gz_d10;
  int32_t p_d10;
  int32_t r_d10;
  int32_t y_d10;

  ax_cg = (int32_t)(ax_g * 100.0f);
  ay_cg = (int32_t)(ay_g * 100.0f);
  az_cg = (int32_t)(az_g * 100.0f);

  gx_d10 = (int32_t)(gx_dps * 10.0f);
  gy_d10 = (int32_t)(gy_dps * 10.0f);
  gz_d10 = (int32_t)(gz_dps * 10.0f);

  p_d10 = (int32_t)(pitch_deg * 10.0f);
  r_d10 = (int32_t)(roll_deg * 10.0f);
  y_d10 = (int32_t)(yaw_deg * 10.0f);

  printf("IMU[x100/x10]=[%ld %ld %ld %ld %ld %ld %ld %ld %ld]\r\n",
         (long)ax_cg,
         (long)ay_cg,
         (long)az_cg,
         (long)gx_d10,
         (long)gy_d10,
         (long)gz_d10,
         (long)p_d10,
         (long)r_d10,
         (long)y_d10);
}

void Telemetry_PrintMotorTestStep(uint8_t step_index)
{
  if (step_index == 0U)
  {
    printf("MOTOR TEST: all off\r\n");
  }
  else
  {
    printf("MOTOR TEST: S%u active\r\n", (unsigned int)step_index);
  }
}

void Telemetry_PrintReceiverState(const receiver_state_t *state)
{
  uint16_t ch0_us;
  uint16_t ch1_us;
  uint16_t ch2_us;
  uint16_t ch3_us;

  if (state == NULL)
  {
    return;
  }

  ch0_us = Telemetry_CrsfRawToUs(state->channels[0]);
  ch1_us = Telemetry_CrsfRawToUs(state->channels[1]);
  ch2_us = Telemetry_CrsfRawToUs(state->channels[2]);
  ch3_us = Telemetry_CrsfRawToUs(state->channels[3]);

  printf("RX[link=%u frames=%lu us]=[%u %u %u %u]\r\n",
         (unsigned int)state->link_active,
         (unsigned long)state->frame_count,
         (unsigned int)ch0_us,
         (unsigned int)ch1_us,
         (unsigned int)ch2_us,
         (unsigned int)ch3_us);
}

void Telemetry_PrintReceiverState16(const receiver_state_t *state)
{
  uint16_t ch_us[RECEIVER_CHANNEL_COUNT];
  uint8_t i;

  if (state == NULL)
  {
    return;
  }

  for (i = 0U; i < RECEIVER_CHANNEL_COUNT; i++)
  {
    ch_us[i] = Telemetry_CrsfRawToUs(state->channels[i]);
  }

  printf("RX16[link=%u frames=%lu us]=[%u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u]\r\n",
         (unsigned int)state->link_active,
         (unsigned long)state->frame_count,
         (unsigned int)ch_us[0],
         (unsigned int)ch_us[1],
         (unsigned int)ch_us[2],
         (unsigned int)ch_us[3],
         (unsigned int)ch_us[4],
         (unsigned int)ch_us[5],
         (unsigned int)ch_us[6],
         (unsigned int)ch_us[7],
         (unsigned int)ch_us[8],
         (unsigned int)ch_us[9],
         (unsigned int)ch_us[10],
         (unsigned int)ch_us[11],
         (unsigned int)ch_us[12],
         (unsigned int)ch_us[13],
         (unsigned int)ch_us[14],
         (unsigned int)ch_us[15]);
}

void Telemetry_PrintArmState(uint8_t armed,
                             uint8_t arm_switch_high,
                             uint8_t arm_low_seen,
                             uint16_t throttle_us,
                             uint16_t s1_us,
                             uint16_t s2_us,
                             uint16_t s3_us,
                             uint16_t s4_us)
{
  printf("ARM[a=%u sw=%u lowSeen=%u thr=%u m]=[%u %u %u %u]\r\n",
         (unsigned int)armed,
         (unsigned int)arm_switch_high,
         (unsigned int)arm_low_seen,
         (unsigned int)throttle_us,
         (unsigned int)s1_us,
         (unsigned int)s2_us,
         (unsigned int)s3_us,
         (unsigned int)s4_us);
}

void Telemetry_PrintRatePid(const App_RatePidGains_t *gains, const char *source)
{
  if (gains == NULL)
  {
    return;
  }

  if (source == NULL)
  {
    source = "-";
  }

  printf("PID[src=%s]=[R %.4f %.4f %.4f P %.4f %.4f %.4f Y %.4f %.4f %.4f]\r\n",
         source,
         (double)gains->roll.kp,
         (double)gains->roll.ki,
         (double)gains->roll.kd,
         (double)gains->pitch.kp,
         (double)gains->pitch.ki,
         (double)gains->pitch.kd,
         (double)gains->yaw.kp,
         (double)gains->yaw.ki,
         (double)gains->yaw.kd);
}

void Telemetry_PrintAngles(float pitch_deg, float roll_deg, float yaw_deg)
{
  int32_t p10;
  int32_t r10;
  int32_t y10;
  int32_t p10_frac;
  int32_t r10_frac;
  int32_t y10_frac;

  p10 = (int32_t)(pitch_deg * 10.0f + 0.5f);
  r10 = (int32_t)(roll_deg * 10.0f + 0.5f);
  y10 = (int32_t)(yaw_deg * 10.0f + 0.5f);

  p10_frac = p10 % 10;
  r10_frac = r10 % 10;
  y10_frac = y10 % 10;

  if (p10_frac < 0)
  {
    p10_frac = -p10_frac;
  }
  if (r10_frac < 0)
  {
    r10_frac = -r10_frac;
  }
  if (y10_frac < 0)
  {
    y10_frac = -y10_frac;
  }

  printf("ANGLES[p r y]=[%ld.%01ld %ld.%01ld %ld.%01ld]\r\n",
         (long)(p10 / 10),
         (long)p10_frac,
         (long)(r10 / 10),
         (long)r10_frac,
         (long)(y10 / 10),
         (long)y10_frac);
}

void Telemetry_PrintBatteryState(float battery_voltage_v, uint32_t adc_raw)
{
  int32_t battery_mv;

  battery_mv = (int32_t)(battery_voltage_v * 1000.0f + 0.5f);
  if (battery_mv < 0)
  {
    battery_mv = 0;
  }

  printf("VBAT[mV raw]=[%ld %lu]\r\n",
         (long)battery_mv,
         (unsigned long)adc_raw);
}
