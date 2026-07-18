#include "app.h"

#include "attitude.h"
#include "imu.h"
#include "motors.h"
#include "receiver.h"
#include "telemetry.h"

#define APP_MOTOR_TEST_MODE 0U
#define APP_CONTROL_LOOP_MS 2U
#define RAD_PER_DEG       0.0174532925f
#define APP_ENABLE_ANGLE_TELEMETRY 0U
#define APP_CH_ROLL_INDEX      0U
#define APP_CH_PITCH_INDEX     1U
#define APP_CH_THROTTLE_INDEX  2U
#define APP_CH_YAW_INDEX       3U
#define APP_CH_ARM_INDEX       4U
#define APP_ARM_THRESHOLD_US   1500U
#define APP_THROTTLE_LOW_US    1050U
#define APP_ARM_HOLD_MS        300U
#define APP_CONTROL_DEADBAND_US 20U
#define APP_MIX_ATTENUATION_DIV 3
#define APP_MOTOR_IDLE_US      1080U
#define APP_PWM_MIN_US         988U
#define APP_PWM_MID_US         1500U
#define APP_PWM_MAX_US         2012U
#define APP_CRSF_MIN_RAW       172U
#define APP_CRSF_MAX_RAW       1811U

static uint16_t App_CrsfRawToUs(uint16_t raw)
{
  uint32_t scaled;

  if (raw <= APP_CRSF_MIN_RAW)
  {
    return APP_PWM_MIN_US;
  }

  if (raw >= APP_CRSF_MAX_RAW)
  {
    return APP_PWM_MAX_US;
  }

  scaled = (uint32_t)(raw - APP_CRSF_MIN_RAW) * (APP_PWM_MAX_US - APP_PWM_MIN_US);
  scaled = (scaled / (APP_CRSF_MAX_RAW - APP_CRSF_MIN_RAW)) + APP_PWM_MIN_US;
  return (uint16_t)scaled;
}

static uint16_t App_ClampPulseUs(int32_t pulse_us)
{
  if (pulse_us < (int32_t)APP_PWM_MIN_US)
  {
    return APP_PWM_MIN_US;
  }

  if (pulse_us > (int32_t)APP_PWM_MAX_US)
  {
    return APP_PWM_MAX_US;
  }

  return (uint16_t)pulse_us;
}

static int32_t App_ApplyDeadbandAndAttenuate(int32_t value)
{
  if (value > 0)
  {
    if (value <= (int32_t)APP_CONTROL_DEADBAND_US)
    {
      return 0;
    }

    value -= (int32_t)APP_CONTROL_DEADBAND_US;
  }
  else if (value < 0)
  {
    if (value >= -(int32_t)APP_CONTROL_DEADBAND_US)
    {
      return 0;
    }

    value += (int32_t)APP_CONTROL_DEADBAND_US;
  }

  return value / APP_MIX_ATTENUATION_DIV;
}

void App_Init(void)
{
  Motors_Init();
  Motors_SetOutputEnabled(APP_MOTOR_TEST_MODE);
  Motors_StopAll();

  Attitude_Init();
  Receiver_Init();

  Telemetry_PrintImuLoggerStart();

  if (IMU_DetectAndInit() == HAL_OK)
  {
    Telemetry_PrintImuDetected(IMU_GetType(), IMU_GetWhoAmI());
  }
  else
  {
    Telemetry_PrintImuDetectionFailed();
  }
}

void App_Update(void)
{
  static uint32_t detect_retry_counter = 0U;
  static uint32_t last_tick_ms = 0U;
  static uint8_t last_motor_test_step = 0xFFU;
  static uint32_t last_receiver_telemetry_ms = 0U;
  static uint8_t motors_armed = 0U;
  static uint8_t trim_captured = 0U;
  static uint32_t arm_hold_start_ms = 0U;
  static uint16_t roll_center_us = APP_PWM_MID_US;
  static uint16_t pitch_center_us = APP_PWM_MID_US;
  static uint16_t yaw_center_us = APP_PWM_MID_US;
  static uint8_t yaw_zero_captured = 0U;
  static float startup_yaw_offset_deg = 0.0f;
  IMU_RawData_t imu_raw;
  uint8_t motor_test_step;
  uint32_t now_ms;
  receiver_state_t receiver_state;
  float dt_s;
  float ax_g;
  float ay_g;
  float az_g;
  float gx_dps;
  float gy_dps;
  float gz_dps;
  float pitch_deg;
  float roll_deg;
  float yaw_deg;
  uint16_t roll_us;
  uint16_t pitch_us;
  uint16_t throttle_us;
  uint16_t yaw_us;
  uint16_t arm_us;
  int32_t roll_term;
  int32_t pitch_term;
  int32_t yaw_term;
  int32_t throttle_term;
  uint16_t s1_us;
  uint16_t s2_us;
  uint16_t s3_us;
  uint16_t s4_us;
  uint8_t arm_switch_high;
  uint8_t throttle_low;

  s1_us = APP_PWM_MIN_US;
  s2_us = APP_PWM_MIN_US;
  s3_us = APP_PWM_MIN_US;
  s4_us = APP_PWM_MIN_US;
  arm_switch_high = 0U;
  throttle_low = 0U;
  throttle_us = APP_PWM_MIN_US;

  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);

  if (APP_MOTOR_TEST_MODE != 0U)
  {
    motor_test_step = Motors_RunTestPattern(HAL_GetTick());
    if (motor_test_step != last_motor_test_step)
    {
      Telemetry_PrintMotorTestStep(motor_test_step);
      last_motor_test_step = motor_test_step;
    }
  }

  now_ms = HAL_GetTick();
  if (last_tick_ms == 0U)
  {
    dt_s = ((float)APP_CONTROL_LOOP_MS) * 0.001f;
  }
  else
  {
    dt_s = ((float)(now_ms - last_tick_ms)) * 0.001f;
  }
  last_tick_ms = now_ms;

  Receiver_Update(now_ms);
  Receiver_GetState(&receiver_state);

  if (APP_MOTOR_TEST_MODE == 0U)
  {
    arm_us = App_CrsfRawToUs(receiver_state.channels[APP_CH_ARM_INDEX]);
    roll_us = App_CrsfRawToUs(receiver_state.channels[APP_CH_ROLL_INDEX]);
    pitch_us = App_CrsfRawToUs(receiver_state.channels[APP_CH_PITCH_INDEX]);
    throttle_us = App_CrsfRawToUs(receiver_state.channels[APP_CH_THROTTLE_INDEX]);
    yaw_us = App_CrsfRawToUs(receiver_state.channels[APP_CH_YAW_INDEX]);
    arm_switch_high = (uint8_t)(arm_us >= APP_ARM_THRESHOLD_US);
    throttle_low = (uint8_t)(throttle_us <= APP_THROTTLE_LOW_US);

    if ((receiver_state.link_active == 0U) || (arm_switch_high == 0U))
    {
      motors_armed = 0U;
      trim_captured = 0U;
      arm_hold_start_ms = 0U;
    }

    Motors_SetOutputEnabled(1U);

    if (motors_armed == 0U)
    {
      Motors_StopAll();
      s1_us = APP_PWM_MIN_US;
      s2_us = APP_PWM_MIN_US;
      s3_us = APP_PWM_MIN_US;
      s4_us = APP_PWM_MIN_US;

      if ((receiver_state.link_active != 0U) &&
          (arm_switch_high != 0U) &&
          (throttle_low != 0U))
      {
        if (arm_hold_start_ms == 0U)
        {
          arm_hold_start_ms = now_ms;
        }
        else if ((now_ms - arm_hold_start_ms) >= APP_ARM_HOLD_MS)
        {
          motors_armed = 1U;
          trim_captured = 0U;
        }
      }
      else
      {
        arm_hold_start_ms = 0U;
      }
    }

    if (motors_armed != 0U)
    {
      if (trim_captured == 0U)
      {
        roll_center_us = roll_us;
        pitch_center_us = pitch_us;
        yaw_center_us = yaw_us;
        trim_captured = 1U;
      }

      roll_term = App_ApplyDeadbandAndAttenuate((int32_t)roll_us - (int32_t)roll_center_us);
      pitch_term = App_ApplyDeadbandAndAttenuate((int32_t)pitch_us - (int32_t)pitch_center_us);
      yaw_term = App_ApplyDeadbandAndAttenuate((int32_t)yaw_us - (int32_t)yaw_center_us);
      throttle_term = (int32_t)throttle_us;
      if (throttle_term < (int32_t)APP_MOTOR_IDLE_US)
      {
        throttle_term = APP_MOTOR_IDLE_US;
      }

      s1_us = App_ClampPulseUs(throttle_term + pitch_term + roll_term - yaw_term);
      s2_us = App_ClampPulseUs(throttle_term + pitch_term - roll_term + yaw_term);
      s3_us = App_ClampPulseUs(throttle_term - pitch_term - roll_term - yaw_term);
      s4_us = App_ClampPulseUs(throttle_term - pitch_term + roll_term + yaw_term);

      Motors_WriteUs(s1_us, s2_us, s3_us, s4_us);
    }
  }

  if ((now_ms - last_receiver_telemetry_ms) >= 500U)
  {
    Telemetry_PrintReceiverState(&receiver_state);
    Telemetry_PrintArmState(motors_armed,
                            arm_switch_high,
                            throttle_low,
                            throttle_us,
                            s1_us,
                            s2_us,
                            s3_us,
                            s4_us);
    last_receiver_telemetry_ms = now_ms;
  }

  if ((IMU_GetType() == IMU_TYPE_UNKNOWN) && ((detect_retry_counter++ % 10U) == 0U))
  {
    if (IMU_DetectAndInit() == HAL_OK)
    {
      Telemetry_PrintImuDetected(IMU_GetType(), IMU_GetWhoAmI());
    }
  }

  if ((IMU_GetType() != IMU_TYPE_UNKNOWN) && (IMU_ReadRawAligned(&imu_raw) == HAL_OK))
  {
    ax_g = ((float)imu_raw.accel_x) / IMU_ACCEL_LSB_PER_G;
    ay_g = ((float)imu_raw.accel_y) / IMU_ACCEL_LSB_PER_G;
    az_g = ((float)imu_raw.accel_z) / IMU_ACCEL_LSB_PER_G;
    gx_dps = ((float)imu_raw.gyro_x) / IMU_GYRO_LSB_PER_DPS;
    gy_dps = ((float)imu_raw.gyro_y) / IMU_GYRO_LSB_PER_DPS;
    gz_dps = ((float)imu_raw.gyro_z) / IMU_GYRO_LSB_PER_DPS;

    Attitude_UpdateIMU(gx_dps * RAD_PER_DEG,
                       gy_dps * RAD_PER_DEG,
                       gz_dps * RAD_PER_DEG,
                       ax_g,
                       ay_g,
                       az_g,
                       dt_s);
    Attitude_GetBoardAnglesDeg(&pitch_deg, &roll_deg, &yaw_deg);

    if (yaw_zero_captured == 0U)
    {
      startup_yaw_offset_deg = yaw_deg;
      yaw_zero_captured = 1U;
    }

    yaw_deg = Attitude_WrapAngle180(yaw_deg - startup_yaw_offset_deg);
#if APP_ENABLE_ANGLE_TELEMETRY
    Telemetry_PrintAngles(pitch_deg, roll_deg, yaw_deg);
#endif
  }
  else
  {
    Telemetry_PrintImuReadFailed(IMU_GetType(), IMU_GetWhoAmI());
  }

  HAL_Delay(APP_CONTROL_LOOP_MS);
}
