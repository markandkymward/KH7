#include "app.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "attitude.h"
#include "imu.h"
#include "motors.h"
#include "communications.h"
#include "receiver.h"
#include "telemetry.h"
#include "main.h"

#define APP_MOTOR_TEST_MODE 0U
#define APP_CONTROL_LOOP_MS 2U
#define RAD_PER_DEG       0.0174532925f
#define APP_ENABLE_IMU_RUNTIME_TELEMETRY 1U
#define APP_ENABLE_ARM_RUNTIME_TELEMETRY 1U
#define APP_ENABLE_RECEIVER_RUNTIME_TELEMETRY 0U
#define APP_ENABLE_ANGLE_TELEMETRY 0U
#define APP_CH_ROLL_INDEX      0U
#define APP_CH_PITCH_INDEX     1U
#define APP_CH_THROTTLE_INDEX  2U
#define APP_CH_YAW_INDEX       3U
#define APP_CH_ARM_INDEX       4U
#define APP_ARM_THRESHOLD_US   1500U
#define APP_THROTTLE_LOW_US    1100U
#define APP_ARM_HOLD_MS        300U
#define APP_BEEPER_TOGGLE_MS   150U
#define APP_USB_TEST_ARM_DELAY_MS 2000U
#define APP_IMU_TELEMETRY_MS    120U
#define APP_ARM_TELEMETRY_MS    150U
#define APP_RX16_TELEMETRY_MS   120U
#define APP_LOW_THROTTLE_MIX_DISABLE_US 40U
#define APP_CONTROL_DEADBAND_US 20U
#define APP_MOTOR_IDLE_US      1080U
#define APP_ROLL_SIGN          (1)
#define APP_PITCH_SIGN         (-1)
#define APP_YAW_SIGN           (1)
#define APP_GYRO_YAW_SIGN      (-1)
#define APP_GYRO_STILL_DPS     0.5f
#define APP_ACCEL_STILL_TOL_G  0.08f
#define APP_YAW_BIAS_ALPHA     0.001f
#define APP_YAW_BIAS_SETTLE_SAMPLES 1000U
#define APP_ADC_MAX_COUNT      65535.0f
#define APP_ADC_REF_V          3.3f
#define APP_BATTERY_DIVIDER_RATIO 11.13f
#define APP_BATTERY_FILTER_ALPHA 0.60f
#define APP_BATTERY_ADC_SAMPLES 2U
#define APP_BATTERY_SAMPLE_MS   120U
#define APP_BATTERY_CHANNEL      ADC_CHANNEL_10
#define APP_RATE_CMD_MAX_ROLL_DPS   300.0f
#define APP_RATE_CMD_MAX_PITCH_DPS  300.0f
#define APP_RATE_CMD_MAX_YAW_DPS    220.0f
#define APP_RATE_KP_ROLL_DEFAULT_US_PER_DPS 0.90f
#define APP_RATE_KP_PITCH_DEFAULT_US_PER_DPS 0.90f
#define APP_RATE_KP_YAW_DEFAULT_US_PER_DPS  0.80f
#define APP_RATE_KI_ROLL_DEFAULT_US_PER_DPS_S 0.00f
#define APP_RATE_KI_PITCH_DEFAULT_US_PER_DPS_S 0.00f
#define APP_RATE_KI_YAW_DEFAULT_US_PER_DPS_S  0.00f
#define APP_RATE_KD_ROLL_DEFAULT_US_PER_DPS_PER_S 0.00f
#define APP_RATE_KD_PITCH_DEFAULT_US_PER_DPS_PER_S 0.00f
#define APP_RATE_KD_YAW_DEFAULT_US_PER_DPS_PER_S  0.00f
#define APP_RATE_TERM_LIMIT_US      320
#define APP_RATE_KP_MIN_US_PER_DPS  0.0f
#define APP_RATE_KP_MAX_US_PER_DPS  4.0f
#define APP_RATE_KI_MIN_US_PER_DPS_S 0.0f
#define APP_RATE_KI_MAX_US_PER_DPS_S 2.0f
#define APP_RATE_KD_MIN_US_PER_DPS_PER_S 0.0f
#define APP_RATE_KD_MAX_US_PER_DPS_PER_S 0.2f
#define APP_PWM_MIN_US         988U
#define APP_PWM_MID_US         1500U
#define APP_PWM_MAX_US         2012U
#define APP_THROTTLE_MAX_US    1880U
#define APP_CRSF_MIN_RAW       172U
#define APP_CRSF_MAX_RAW       1811U

#define APP_PID_FLASH_MAGIC    0x50494447UL
#define APP_PID_FLASH_VERSION  1UL
#define APP_PID_FLASH_ADDRESS  0x081E0000UL

/* Motor position mapping used by the mixer:
 * S1 = Front-Left, S2 = Front-Right, S3 = Rear-Right, S4 = Rear-Left
 */

static volatile uint8_t g_usb_motor_test_enabled = 0U;
static volatile uint8_t g_usb_motor_test_motor_index = 1U;
static volatile uint16_t g_usb_motor_test_pulse_us = 1100U;

typedef enum
{
  APP_PID_CMD_NONE = 0,
  APP_PID_CMD_SET_AND_SAVE = 1,
  APP_PID_CMD_SAVE = 2,
  APP_PID_CMD_LOAD = 3,
  APP_PID_CMD_DEFAULT = 4,
} App_PidCommand_t;

typedef struct
{
  uint32_t magic;
  uint32_t version;
  App_RatePidGains_t gains;
  uint32_t crc32;
  uint32_t reserved[2];
} App_PidFlashBlob_t;

typedef union
{
  App_PidFlashBlob_t blob;
  uint32_t words[16];
} App_PidFlashPage_t;

#if defined(__GNUC__)
#define APP_FLASHWORD_ALIGN __attribute__((aligned(32)))
#else
#define APP_FLASHWORD_ALIGN
#endif

_Static_assert(sizeof(App_PidFlashBlob_t) <= sizeof(App_PidFlashPage_t), "PID flash blob too large");

static App_RatePidGains_t g_rate_pid_gains = {
  {APP_RATE_KP_ROLL_DEFAULT_US_PER_DPS, APP_RATE_KI_ROLL_DEFAULT_US_PER_DPS_S, APP_RATE_KD_ROLL_DEFAULT_US_PER_DPS_PER_S},
  {APP_RATE_KP_PITCH_DEFAULT_US_PER_DPS, APP_RATE_KI_PITCH_DEFAULT_US_PER_DPS_S, APP_RATE_KD_PITCH_DEFAULT_US_PER_DPS_PER_S},
  {APP_RATE_KP_YAW_DEFAULT_US_PER_DPS, APP_RATE_KI_YAW_DEFAULT_US_PER_DPS_S, APP_RATE_KD_YAW_DEFAULT_US_PER_DPS_PER_S},
};
static volatile App_PidCommand_t g_pid_command = APP_PID_CMD_NONE;
static volatile App_RatePidGains_t g_pid_command_gains;
static volatile uint32_t g_pid_command_queued_count = 0U;
static volatile uint32_t g_pid_command_handled_count = 0U;

static uint8_t g_boot_log_pending = 1U;
static uint8_t g_boot_pid_loaded = 0U;
static float g_yaw_gyro_bias_dps = 0.0f;
static float g_yaw_gyro_bias_sum_dps = 0.0f;
static uint32_t g_yaw_stationary_sample_count = 0U;
static uint8_t g_yaw_gyro_bias_ready = 0U;

extern ADC_HandleTypeDef hadc1;

#define APP_BOOT_LOG_SIZE 2048U
static char g_boot_log_buffer[APP_BOOT_LOG_SIZE];
static uint16_t g_boot_log_pos = 0U;

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

static uint8_t App_UpdateYawGyroBias(float ax_g,
                                     float ay_g,
                                     float az_g,
                                     float gx_dps,
                                     float gy_dps,
                                     float gz_dps,
                                     uint8_t motors_armed,
                                     uint16_t throttle_us)
{
  float accel_mag_sq;
  float accel_min_sq;
  float accel_max_sq;
  uint8_t stationary;

  if ((motors_armed != 0U) && (throttle_us > APP_THROTTLE_LOW_US))
  {
    return 0U;
  }

  accel_mag_sq = (ax_g * ax_g) + (ay_g * ay_g) + (az_g * az_g);
  accel_min_sq = (1.0f - APP_ACCEL_STILL_TOL_G) * (1.0f - APP_ACCEL_STILL_TOL_G);
  accel_max_sq = (1.0f + APP_ACCEL_STILL_TOL_G) * (1.0f + APP_ACCEL_STILL_TOL_G);
  stationary = ((accel_mag_sq >= accel_min_sq) &&
                (accel_mag_sq <= accel_max_sq) &&
                (gx_dps > -APP_GYRO_STILL_DPS) && (gx_dps < APP_GYRO_STILL_DPS) &&
                (gy_dps > -APP_GYRO_STILL_DPS) && (gy_dps < APP_GYRO_STILL_DPS) &&
                (gz_dps > -APP_GYRO_STILL_DPS) && (gz_dps < APP_GYRO_STILL_DPS)) ? 1U : 0U;

  if (stationary == 0U)
  {
    return 0U;
  }

  if (g_yaw_gyro_bias_ready == 0U)
  {
    g_yaw_gyro_bias_sum_dps += gz_dps;
    g_yaw_stationary_sample_count++;

    if (g_yaw_stationary_sample_count >= APP_YAW_BIAS_SETTLE_SAMPLES)
    {
      g_yaw_gyro_bias_dps = g_yaw_gyro_bias_sum_dps / ((float)g_yaw_stationary_sample_count);
      g_yaw_gyro_bias_ready = 1U;
    }
  }
  else
  {
    g_yaw_gyro_bias_dps = ((1.0f - APP_YAW_BIAS_ALPHA) * g_yaw_gyro_bias_dps) + (APP_YAW_BIAS_ALPHA * gz_dps);
  }

  return g_yaw_gyro_bias_ready;
}

static uint8_t App_ReadBatteryVoltage(float *battery_voltage_v, uint32_t *adc_raw)
{
  ADC_ChannelConfTypeDef sConfig;
  uint32_t sum_samples;
  uint32_t sample;
  uint32_t throwaway_sample;
  uint32_t i;
  float adc_pin_voltage_v;

  if ((battery_voltage_v == NULL) || (adc_raw == NULL))
  {
    return 0U;
  }

  sConfig.Channel = APP_BATTERY_CHANNEL;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    return 0U;
  }

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return 0U;
  }

  if (HAL_ADC_PollForConversion(&hadc1, 3U) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    return 0U;
  }

  throwaway_sample = HAL_ADC_GetValue(&hadc1);
  (void)throwaway_sample;
  (void)HAL_ADC_Stop(&hadc1);

  sum_samples = 0U;
  for (i = 0U; i < APP_BATTERY_ADC_SAMPLES; i++)
  {
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
      return 0U;
    }

    if (HAL_ADC_PollForConversion(&hadc1, 3U) != HAL_OK)
    {
      (void)HAL_ADC_Stop(&hadc1);
      return 0U;
    }

    sum_samples += HAL_ADC_GetValue(&hadc1);
    (void)HAL_ADC_Stop(&hadc1);
  }

  sample = sum_samples / APP_BATTERY_ADC_SAMPLES;

  adc_pin_voltage_v = (((float)sample) / APP_ADC_MAX_COUNT) * APP_ADC_REF_V;
  *battery_voltage_v = adc_pin_voltage_v * APP_BATTERY_DIVIDER_RATIO;
  *adc_raw = sample;
  return 1U;
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

static int32_t App_ApplyDeadbandUs(int32_t value, int32_t deadband_us)
{
  if (value > 0)
  {
    if (value <= deadband_us)
    {
      return 0;
    }

    value -= deadband_us;
  }
  else if (value < 0)
  {
    if (value >= -deadband_us)
    {
      return 0;
    }

    value += deadband_us;
  }

  return value;
}

static float App_ClampFloat(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}

static float App_StickOffsetUsToRateDps(int32_t stick_offset_us, float max_rate_dps)
{
  float normalized;

  stick_offset_us = App_ApplyDeadbandUs(stick_offset_us, (int32_t)APP_CONTROL_DEADBAND_US);
  normalized = ((float)stick_offset_us) / ((float)((int32_t)APP_PWM_MAX_US - (int32_t)APP_PWM_MID_US));

  if (normalized > 1.0f)
  {
    normalized = 1.0f;
  }
  else if (normalized < -1.0f)
  {
    normalized = -1.0f;
  }

  return normalized * max_rate_dps;
}

static int32_t App_ClampControlTerm(int32_t value, int32_t limit)
{
  if (value > limit)
  {
    return limit;
  }

  if (value < -limit)
  {
    return -limit;
  }

  return value;
}

static int32_t App_ClampInt32(int32_t value, int32_t min_value, int32_t max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}

static uint32_t App_Crc32(const uint8_t *data, size_t len)
{
  uint32_t crc = 0xFFFFFFFFUL;
  size_t i;
  uint8_t bit;

  if (data == NULL)
  {
    return 0U;
  }

  for (i = 0U; i < len; i++)
  {
    crc ^= (uint32_t)data[i];
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 1UL) != 0U)
      {
        crc = (crc >> 1U) ^ 0xEDB88320UL;
      }
      else
      {
        crc >>= 1U;
      }
    }
  }

  return ~crc;
}

static uint8_t App_IsFiniteInRange(float value, float min_value, float max_value)
{
  if (value != value)
  {
    return 0U;
  }

  if ((value < min_value) || (value > max_value))
  {
    return 0U;
  }

  return 1U;
}

static uint8_t App_AreRatePidGainsValid(const App_RatePidGains_t *gains)
{
  if (gains == NULL)
  {
    return 0U;
  }

  if ((App_IsFiniteInRange(gains->roll.kp, APP_RATE_KP_MIN_US_PER_DPS, APP_RATE_KP_MAX_US_PER_DPS) == 0U) ||
      (App_IsFiniteInRange(gains->pitch.kp, APP_RATE_KP_MIN_US_PER_DPS, APP_RATE_KP_MAX_US_PER_DPS) == 0U) ||
      (App_IsFiniteInRange(gains->yaw.kp, APP_RATE_KP_MIN_US_PER_DPS, APP_RATE_KP_MAX_US_PER_DPS) == 0U) ||
      (App_IsFiniteInRange(gains->roll.ki, APP_RATE_KI_MIN_US_PER_DPS_S, APP_RATE_KI_MAX_US_PER_DPS_S) == 0U) ||
      (App_IsFiniteInRange(gains->pitch.ki, APP_RATE_KI_MIN_US_PER_DPS_S, APP_RATE_KI_MAX_US_PER_DPS_S) == 0U) ||
      (App_IsFiniteInRange(gains->yaw.ki, APP_RATE_KI_MIN_US_PER_DPS_S, APP_RATE_KI_MAX_US_PER_DPS_S) == 0U) ||
      (App_IsFiniteInRange(gains->roll.kd, APP_RATE_KD_MIN_US_PER_DPS_PER_S, APP_RATE_KD_MAX_US_PER_DPS_PER_S) == 0U) ||
      (App_IsFiniteInRange(gains->pitch.kd, APP_RATE_KD_MIN_US_PER_DPS_PER_S, APP_RATE_KD_MAX_US_PER_DPS_PER_S) == 0U) ||
      (App_IsFiniteInRange(gains->yaw.kd, APP_RATE_KD_MIN_US_PER_DPS_PER_S, APP_RATE_KD_MAX_US_PER_DPS_PER_S) == 0U))
  {
    return 0U;
  }

  return 1U;
}

void App_AppendBootLog(const char *str)
{
  size_t len;
  
  if ((str == NULL) || (g_boot_log_pos >= APP_BOOT_LOG_SIZE))
  {
    return;
  }

  len = strlen(str);
  if (len == 0U)
  {
    return;
  }

  if ((g_boot_log_pos + len) >= APP_BOOT_LOG_SIZE)
  {
    len = APP_BOOT_LOG_SIZE - g_boot_log_pos - 1U;
  }

  memcpy(&g_boot_log_buffer[g_boot_log_pos], str, len);
  g_boot_log_pos += len;
  g_boot_log_buffer[g_boot_log_pos] = '\0';
}

const char *App_GetBootLog(void)
{
  return g_boot_log_buffer;
}

void App_GetRatePidGains(App_RatePidGains_t *gains)
{
  if (gains == NULL)
  {
    return;
  }

  __disable_irq();
  *gains = g_rate_pid_gains;
  __enable_irq();
}

uint8_t App_SetRatePidGains(const App_RatePidGains_t *gains)
{
  if (App_AreRatePidGainsValid(gains) == 0U)
  {
    return 0U;
  }

  __disable_irq();
  g_rate_pid_gains = *gains;
  __enable_irq();

  return 1U;
}

void App_ResetRatePidDefaults(void)
{
  App_RatePidGains_t defaults = {
    {APP_RATE_KP_ROLL_DEFAULT_US_PER_DPS, APP_RATE_KI_ROLL_DEFAULT_US_PER_DPS_S, APP_RATE_KD_ROLL_DEFAULT_US_PER_DPS_PER_S},
    {APP_RATE_KP_PITCH_DEFAULT_US_PER_DPS, APP_RATE_KI_PITCH_DEFAULT_US_PER_DPS_S, APP_RATE_KD_PITCH_DEFAULT_US_PER_DPS_PER_S},
    {APP_RATE_KP_YAW_DEFAULT_US_PER_DPS, APP_RATE_KI_YAW_DEFAULT_US_PER_DPS_S, APP_RATE_KD_YAW_DEFAULT_US_PER_DPS_PER_S},
  };

  (void)App_SetRatePidGains(&defaults);
}

uint8_t App_LoadRatePidGains(void)
{
  const App_PidFlashBlob_t *stored;
  uint32_t expected_crc;

  stored = (const App_PidFlashBlob_t *)APP_PID_FLASH_ADDRESS;

  printf("PID_LOAD_DBG: magic=0x%08lX ver=%lu\r\n",
         (unsigned long)stored->magic, (unsigned long)stored->version);

  if ((stored->magic != APP_PID_FLASH_MAGIC) || (stored->version != APP_PID_FLASH_VERSION))
  {
    printf("PID_LOAD_DBG: bad header (expected magic=0x%08lX ver=%lu)\r\n",
           (unsigned long)APP_PID_FLASH_MAGIC, (unsigned long)APP_PID_FLASH_VERSION);
    return 0U;
  }

  expected_crc = App_Crc32((const uint8_t *)stored, offsetof(App_PidFlashBlob_t, crc32));
  if (expected_crc != stored->crc32)
  {
    printf("PID_LOAD_DBG: crc mismatch stored=0x%08lX computed=0x%08lX\r\n",
           (unsigned long)stored->crc32, (unsigned long)expected_crc);
    return 0U;
  }

  if (App_AreRatePidGainsValid(&stored->gains) == 0U)
  {
    printf("PID_LOAD_DBG: gains out of range\r\n");
    return 0U;
  }

  (void)App_SetRatePidGains(&stored->gains);
  return 1U;
}

uint8_t App_SaveRatePidGains(void)
{
  FLASH_EraseInitTypeDef erase;
  uint32_t sector_error = 0U;
  uint32_t address;
  App_PidFlashPage_t APP_FLASHWORD_ALIGN page;
  const App_PidFlashBlob_t *written;
  uint8_t write_index;

  memset(&page, 0xFF, sizeof(page));
  page.blob.magic = APP_PID_FLASH_MAGIC;
  page.blob.version = APP_PID_FLASH_VERSION;
  App_GetRatePidGains(&page.blob.gains);
  page.blob.crc32 = App_Crc32((const uint8_t *)&page.blob, offsetof(App_PidFlashBlob_t, crc32));

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    printf("PID_SAVE_DBG: unlock fail err=0x%08lX\r\n", (unsigned long)HAL_FLASH_GetError());
    return 0U;
  }

  memset(&erase, 0, sizeof(erase));
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Banks = FLASH_BANK_2;
  erase.Sector = FLASH_SECTOR_7;
  erase.NbSectors = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
  {
    printf("PID_SAVE_DBG: erase fail sector_err=%lu flash_err=0x%08lX\r\n",
           (unsigned long)sector_error, (unsigned long)HAL_FLASH_GetError());
    (void)HAL_FLASH_Lock();
    return 0U;
  }

  address = APP_PID_FLASH_ADDRESS;
  for (write_index = 0U; write_index < 2U; write_index++)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                          address,
                          (uint32_t)&page.words[write_index * 8U]) != HAL_OK)
    {
      printf("PID_SAVE_DBG: write fail word=%u addr=0x%08lX err=0x%08lX\r\n",
             (unsigned)write_index, (unsigned long)address, (unsigned long)HAL_FLASH_GetError());
      (void)HAL_FLASH_Lock();
      return 0U;
    }
    address += 32U;
  }

  (void)HAL_FLASH_Lock();

  written = (const App_PidFlashBlob_t *)APP_PID_FLASH_ADDRESS;
  if ((written->magic != APP_PID_FLASH_MAGIC) || (written->version != APP_PID_FLASH_VERSION))
  {
    printf("PID_SAVE_DBG: verify header fail magic=0x%08lX ver=%lu\r\n",
           (unsigned long)written->magic, (unsigned long)written->version);
    return 0U;
  }

  if (App_Crc32((const uint8_t *)written, offsetof(App_PidFlashBlob_t, crc32)) != written->crc32)
  {
    printf("PID_SAVE_DBG: verify crc fail stored=0x%08lX computed=0x%08lX\r\n",
           (unsigned long)written->crc32,
           (unsigned long)App_Crc32((const uint8_t *)written, offsetof(App_PidFlashBlob_t, crc32)));
    return 0U;
  }

  return 1U;
}

uint8_t App_RequestRatePidSetAndSave(const App_RatePidGains_t *gains)
{
  if (App_AreRatePidGainsValid(gains) == 0U)
  {
    return 0U;
  }

  __disable_irq();
  g_pid_command_gains = *gains;
  g_pid_command = APP_PID_CMD_SET_AND_SAVE;
  g_pid_command_queued_count++;
  __enable_irq();

  return 1U;
}

void App_RequestRatePidSave(void)
{
  __disable_irq();
  g_pid_command = APP_PID_CMD_SAVE;
  g_pid_command_queued_count++;
  __enable_irq();
}

void App_RequestRatePidLoad(void)
{
  __disable_irq();
  g_pid_command = APP_PID_CMD_LOAD;
  g_pid_command_queued_count++;
  __enable_irq();
}

void App_RequestRatePidDefaults(void)
{
  __disable_irq();
  g_pid_command = APP_PID_CMD_DEFAULT;
  g_pid_command_queued_count++;
  __enable_irq();
}

void App_GetPidCommandDebug(uint32_t *queued_count,
							uint32_t *handled_count,
							uint32_t *pending_cmd)
{
  __disable_irq();
  if (queued_count != NULL)
  {
    *queued_count = g_pid_command_queued_count;
  }

  if (handled_count != NULL)
  {
    *handled_count = g_pid_command_handled_count;
  }

  if (pending_cmd != NULL)
  {
    *pending_cmd = (uint32_t)g_pid_command;
  }
  __enable_irq();
}

void App_PrintPidDebug(void)
{
  const App_PidFlashBlob_t *stored;
  uint32_t queued_count;
  uint32_t handled_count;
  uint32_t pending_cmd;
  uint32_t computed_crc;
  uint8_t header_ok;
  uint8_t crc_ok;
  uint8_t gains_ok;
  App_RatePidGains_t active;

  stored = (const App_PidFlashBlob_t *)APP_PID_FLASH_ADDRESS;
  computed_crc = App_Crc32((const uint8_t *)stored, offsetof(App_PidFlashBlob_t, crc32));
  header_ok = (uint8_t)((stored->magic == APP_PID_FLASH_MAGIC) &&
                        (stored->version == APP_PID_FLASH_VERSION));
  crc_ok = (uint8_t)(computed_crc == stored->crc32);
  gains_ok = App_AreRatePidGainsValid(&stored->gains);

  App_GetPidCommandDebug(&queued_count, &handled_count, &pending_cmd);
  App_GetRatePidGains(&active);

  printf("PID_DEBUG[q=%lu h=%lu p=%lu]\r\n",
         (unsigned long)queued_count,
         (unsigned long)handled_count,
         (unsigned long)pending_cmd);
  printf("PID_FLASH[addr=0x%08lX magic=0x%08lX ver=%lu crc=0x%08lX calc=0x%08lX header=%u crc_ok=%u gains_ok=%u]\r\n",
         (unsigned long)APP_PID_FLASH_ADDRESS,
         (unsigned long)stored->magic,
         (unsigned long)stored->version,
         (unsigned long)stored->crc32,
         (unsigned long)computed_crc,
         (unsigned)header_ok,
         (unsigned)crc_ok,
         (unsigned)gains_ok);
  Telemetry_PrintRatePid(&active, "debug_active");

  if ((header_ok != 0U) && (crc_ok != 0U) && (gains_ok != 0U))
  {
    Telemetry_PrintRatePid(&stored->gains, "debug_flash");
  }
}

static void App_ProcessPendingPidCommand(void)
{
  App_PidCommand_t cmd;
  App_RatePidGains_t cmd_gains;
  App_RatePidGains_t active;
  uint8_t op_ok;

  __disable_irq();
  cmd = g_pid_command;
  cmd_gains = g_pid_command_gains;
  g_pid_command = APP_PID_CMD_NONE;
  __enable_irq();

  if (cmd == APP_PID_CMD_NONE)
  {
    return;
  }

  __disable_irq();
  g_pid_command_handled_count++;
  __enable_irq();

  switch (cmd)
  {
    case APP_PID_CMD_SET_AND_SAVE:
      if (App_SetRatePidGains(&cmd_gains) == 0U)
      {
        printf("PID_SET[FAIL]\r\n");
        break;
      }

      op_ok = App_SaveRatePidGains();
      App_GetRatePidGains(&active);
      Telemetry_PrintRatePid(&active, op_ok != 0U ? "set_saved" : "set_unsaved");
      printf("PID_SET[%s]\r\n", (op_ok != 0U) ? "OK" : "OK_NO_SAVE");
      printf("PID_SAVE[%s]\r\n", (op_ok != 0U) ? "OK" : "FAIL");
      break;

    case APP_PID_CMD_SAVE:
      op_ok = App_SaveRatePidGains();
      App_GetRatePidGains(&active);
      Telemetry_PrintRatePid(&active, op_ok != 0U ? "save_ok" : "save_fail");
      printf("PID_SAVE[%s]\r\n", (op_ok != 0U) ? "OK" : "FAIL");
      break;

    case APP_PID_CMD_LOAD:
      op_ok = App_LoadRatePidGains();
      if (op_ok == 0U)
      {
        App_ResetRatePidDefaults();
      }
      App_GetRatePidGains(&active);
      Telemetry_PrintRatePid(&active, op_ok != 0U ? "load_ok" : "load_default");
      printf("PID_LOAD[%s]\r\n", (op_ok != 0U) ? "OK" : "DEFAULT");
      break;

    case APP_PID_CMD_DEFAULT:
      App_ResetRatePidDefaults();
      App_GetRatePidGains(&active);
      Telemetry_PrintRatePid(&active, "default");
      break;

    default:
      break;
  }
}

void App_SetUsbMotorTest(uint8_t enabled, uint8_t motor_index, uint16_t pulse_us)
{
  if (motor_index < 1U)
  {
    motor_index = 1U;
  }
  else if (motor_index > 4U)
  {
    motor_index = 4U;
  }

  if (pulse_us < APP_PWM_MIN_US)
  {
    pulse_us = APP_PWM_MIN_US;
  }
  else if (pulse_us > APP_PWM_MAX_US)
  {
    pulse_us = APP_PWM_MAX_US;
  }

  __disable_irq();
  g_usb_motor_test_enabled = (enabled != 0U) ? 1U : 0U;
  g_usb_motor_test_motor_index = motor_index;
  g_usb_motor_test_pulse_us = pulse_us;
  __enable_irq();
}

static void App_EmitBootLog(void)
{
  Telemetry_PrintImuLoggerStart();
  Telemetry_PrintRatePid(&g_rate_pid_gains, g_boot_pid_loaded ? "boot" : "boot_default");
  if (IMU_GetType() != IMU_TYPE_UNKNOWN)
  {
    Telemetry_PrintImuDetected(IMU_GetType(), IMU_GetWhoAmI());
  }
  else
  {
    Telemetry_PrintImuDetectionFailed();
  }
}

void App_Init(void)
{
  Motors_Init();
  Motors_SetOutputEnabled(APP_MOTOR_TEST_MODE);
  Motors_StopAll();

  (void)HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

  Attitude_Init();
  Receiver_Init();

  g_boot_pid_loaded = App_LoadRatePidGains();
  if (g_boot_pid_loaded == 0U)
  {
    App_ResetRatePidDefaults();
  }

  (void)IMU_DetectAndInit();
}

void App_Update(void)
{
  static uint32_t detect_retry_counter = 0U;
  static uint32_t last_tick_ms = 0U;
  static uint8_t last_motor_test_step = 0xFFU;

  if (g_boot_log_pending != 0U)
  {
    if (HAL_GetTick() >= 2000U)
    {
      g_boot_log_pending = 0U;
      App_EmitBootLog();
    }
  }
  static uint32_t last_receiver_telemetry_ms = 0U;
  static uint32_t last_rx16_telemetry_ms = 0U;
#if APP_ENABLE_IMU_RUNTIME_TELEMETRY
  static uint32_t last_imu_telemetry_ms = 0U;
#endif
#if APP_ENABLE_ARM_RUNTIME_TELEMETRY
  static uint32_t last_arm_telemetry_ms = 0U;
#endif
  static uint32_t last_imu_error_ms = 0U;
  static uint8_t usb_test_was_enabled = 0U;
  static uint32_t usb_test_arm_start_ms = 0U;
  static uint8_t motors_armed = 0U;
  static uint8_t trim_captured = 0U;
  static uint32_t arm_hold_start_ms = 0U;
  static uint8_t startup_safety_checked = 0U;
  static uint8_t startup_arm_blocked = 0U;
  static uint8_t beeper_on = 0U;
  static uint32_t last_beeper_toggle_ms = 0U;
  static uint16_t roll_center_us = APP_PWM_MID_US;
  static uint16_t pitch_center_us = APP_PWM_MID_US;
  static uint16_t yaw_center_us = APP_PWM_MID_US;
  static float measured_roll_rate_dps = 0.0f;
  static float measured_pitch_rate_dps = 0.0f;
  static float measured_yaw_rate_dps = 0.0f;
  static float roll_integral_dps_s = 0.0f;
  static float pitch_integral_dps_s = 0.0f;
  static float yaw_integral_dps_s = 0.0f;
  static float prev_roll_rate_error_dps = 0.0f;
  static float prev_pitch_rate_error_dps = 0.0f;
  static float prev_yaw_rate_error_dps = 0.0f;
  static uint8_t pid_state_initialized = 0U;
  static uint8_t yaw_zero_captured = 0U;
  static float startup_yaw_offset_deg = 0.0f;
  static float battery_voltage_filtered_v = 0.0f;
  static uint8_t battery_voltage_valid = 0U;
  static uint32_t battery_adc_raw = 0U;
  static uint32_t last_battery_sample_ms = 0U;
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
  float battery_voltage_v;
  uint16_t roll_us;
  uint16_t pitch_us;
  uint16_t throttle_us;
  uint16_t yaw_us;
  uint16_t arm_us;
  float cmd_roll_rate_dps;
  float cmd_pitch_rate_dps;
  float cmd_yaw_rate_dps;
  float roll_rate_error_dps;
  float pitch_rate_error_dps;
  float yaw_rate_error_dps;
  float roll_rate_derivative_dps_per_s;
  float pitch_rate_derivative_dps_per_s;
  float yaw_rate_derivative_dps_per_s;
  float roll_term_f;
  float pitch_term_f;
  float yaw_term_f;
  float integral_limit_roll;
  float integral_limit_pitch;
  float integral_limit_yaw;
  App_RatePidGains_t active_pid_gains;
  int32_t roll_term;
  int32_t pitch_term;
  int32_t yaw_term;
  int32_t throttle_term;
  int32_t m_front_left;
  int32_t m_front_right;
  int32_t m_rear_right;
  int32_t m_rear_left;
  int32_t mix_max;
  int32_t mix_min;
  int32_t mix_offset;
  uint16_t s1_us;
  uint16_t s2_us;
  uint16_t s3_us;
  uint16_t s4_us;
  uint8_t arm_switch_high;
  uint8_t throttle_low;
  uint8_t usb_motor_test_enabled;
  uint8_t usb_motor_test_motor_index;
  uint16_t usb_motor_test_pulse_us;

  s1_us = APP_PWM_MIN_US;
  s2_us = APP_PWM_MIN_US;
  s3_us = APP_PWM_MIN_US;
  s4_us = APP_PWM_MIN_US;
  arm_switch_high = 0U;
  throttle_low = 0U;
  throttle_us = APP_PWM_MIN_US;

  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);

  __disable_irq();
  usb_motor_test_enabled = g_usb_motor_test_enabled;
  usb_motor_test_motor_index = g_usb_motor_test_motor_index;
  usb_motor_test_pulse_us = g_usb_motor_test_pulse_us;
  __enable_irq();

  now_ms = HAL_GetTick();

  Communications_ServiceEscPassthrough();
  Communications_ServiceUart6Commands();
  App_ProcessPendingPidCommand();

  if (usb_motor_test_enabled != 0U)
  {
    Motors_SetOutputEnabled(1U);

    if (usb_test_was_enabled == 0U)
    {
      usb_test_was_enabled = 1U;
      usb_test_arm_start_ms = now_ms;
    }

    if ((now_ms - usb_test_arm_start_ms) >= APP_USB_TEST_ARM_DELAY_MS)
    {
      switch (usb_motor_test_motor_index)
      {
        case 1U:
          s1_us = usb_motor_test_pulse_us;
          break;
        case 2U:
          s2_us = usb_motor_test_pulse_us;
          break;
        case 3U:
          s3_us = usb_motor_test_pulse_us;
          break;
        case 4U:
          s4_us = usb_motor_test_pulse_us;
          break;
        default:
          break;
      }
    }

    /* Physical channels map as: CH1=LA, CH2=LF, CH3=RA, CH4=RF. */
    Motors_WriteUs(s4_us, s1_us, s3_us, s2_us);

    if ((now_ms - last_receiver_telemetry_ms) >= 1000U)
    {
      last_receiver_telemetry_ms = now_ms;
    }

    if ((now_ms - last_rx16_telemetry_ms) >= APP_RX16_TELEMETRY_MS)
    {
      Receiver_GetState(&receiver_state);
      Telemetry_PrintReceiverState16(&receiver_state);
      last_rx16_telemetry_ms = now_ms;
    }

#if APP_ENABLE_ARM_RUNTIME_TELEMETRY
    if ((now_ms - last_arm_telemetry_ms) >= APP_ARM_TELEMETRY_MS)
    {
      Telemetry_PrintArmState(1U,
                              0U,
                              0U,
                              usb_motor_test_pulse_us,
                              s1_us,
                              s2_us,
                              s3_us,
                              s4_us);
      last_arm_telemetry_ms = now_ms;
    }
#endif

    if ((now_ms - last_battery_sample_ms) >= APP_BATTERY_SAMPLE_MS)
    {
      if (App_ReadBatteryVoltage(&battery_voltage_v, &battery_adc_raw) != 0U)
      {
        if (battery_voltage_valid == 0U)
        {
          battery_voltage_filtered_v = battery_voltage_v;
          battery_voltage_valid = 1U;
        }
        else
        {
          battery_voltage_filtered_v += APP_BATTERY_FILTER_ALPHA * (battery_voltage_v - battery_voltage_filtered_v);
        }
      }
      last_battery_sample_ms = now_ms;
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
      gz_dps = ((float)APP_GYRO_YAW_SIGN) * (((float)imu_raw.gyro_z) / IMU_GYRO_LSB_PER_DPS);
      (void)App_UpdateYawGyroBias(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, 1U, usb_motor_test_pulse_us);
      gz_dps -= g_yaw_gyro_bias_dps;

      measured_roll_rate_dps = gx_dps;
      measured_pitch_rate_dps = gy_dps;
      measured_yaw_rate_dps = gz_dps;

      Attitude_UpdateIMU(gx_dps * RAD_PER_DEG,
                         gy_dps * RAD_PER_DEG,
                         gz_dps * RAD_PER_DEG,
                         ax_g,
                         ay_g,
                         az_g,
                         ((float)APP_CONTROL_LOOP_MS) * 0.001f);
      Attitude_GetBoardAnglesDeg(&pitch_deg, &roll_deg, &yaw_deg);

      if (yaw_zero_captured == 0U)
      {
        startup_yaw_offset_deg = yaw_deg;
        yaw_zero_captured = 1U;
      }

      yaw_deg = Attitude_WrapAngle180(yaw_deg - startup_yaw_offset_deg);

#if APP_ENABLE_IMU_RUNTIME_TELEMETRY
      if ((now_ms - last_imu_telemetry_ms) >= APP_IMU_TELEMETRY_MS)
      {
        Telemetry_PrintImuState(ax_g,
                                ay_g,
                                az_g,
                                gx_dps,
                                gy_dps,
                                gz_dps,
                                pitch_deg,
                                roll_deg,
                                yaw_deg);
        if (battery_voltage_valid != 0U)
        {
          Telemetry_PrintBatteryState(battery_voltage_filtered_v, battery_adc_raw);
        }
        last_imu_telemetry_ms = now_ms;
      }
#endif
    }
    else if ((now_ms - last_imu_error_ms) >= 1000U)
    {
#if APP_ENABLE_IMU_RUNTIME_TELEMETRY
      Telemetry_PrintImuReadFailed(IMU_GetType(), IMU_GetWhoAmI());
#endif
      last_imu_error_ms = now_ms;
    }

    HAL_Delay(APP_CONTROL_LOOP_MS);
    return;
  }

  usb_test_was_enabled = 0U;

  if (APP_MOTOR_TEST_MODE != 0U)
  {
    motor_test_step = Motors_RunTestPattern(now_ms);
    if (motor_test_step != last_motor_test_step)
    {
      Telemetry_PrintMotorTestStep(motor_test_step);
      last_motor_test_step = motor_test_step;
    }
  }

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

    if ((startup_safety_checked == 0U) && (receiver_state.link_active != 0U))
    {
      startup_safety_checked = 1U;
      if (arm_switch_high != 0U)
      {
        startup_arm_blocked = 1U;
      }
    }

    if (startup_arm_blocked != 0U)
    {
      motors_armed = 0U;
      trim_captured = 0U;
      arm_hold_start_ms = 0U;

      /* Keep valid minimum PWM while startup arm safety is active so ESCs
       * can still complete their own ready sequence. */
      Motors_SetOutputEnabled(1U);
      Motors_StopAll();

      if ((now_ms - last_beeper_toggle_ms) >= APP_BEEPER_TOGGLE_MS)
      {
        beeper_on ^= 1U;
        HAL_GPIO_WritePin(BEEPER_GPIO_Port,
                          BEEPER_Pin,
                          (beeper_on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        last_beeper_toggle_ms = now_ms;
      }

      if (arm_switch_high == 0U)
      {
        startup_arm_blocked = 0U;
        beeper_on = 0U;
        HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_RESET);
      }
    }
    else
    {
      if (beeper_on != 0U)
      {
        beeper_on = 0U;
        HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_RESET);
      }

      if ((receiver_state.link_active == 0U) || (arm_switch_high == 0U))
      {
        motors_armed = 0U;
        trim_captured = 0U;
        arm_hold_start_ms = 0U;
        roll_integral_dps_s = 0.0f;
        pitch_integral_dps_s = 0.0f;
        yaw_integral_dps_s = 0.0f;
        pid_state_initialized = 0U;
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
        App_GetRatePidGains(&active_pid_gains);

        if (trim_captured == 0U)
        {
          roll_center_us = roll_us;
          pitch_center_us = pitch_us;
          yaw_center_us = yaw_us;
          trim_captured = 1U;
        }

        cmd_roll_rate_dps = ((float)APP_ROLL_SIGN) * App_StickOffsetUsToRateDps((int32_t)roll_us - (int32_t)roll_center_us,
                           APP_RATE_CMD_MAX_ROLL_DPS);
        cmd_pitch_rate_dps = ((float)APP_PITCH_SIGN) * App_StickOffsetUsToRateDps((int32_t)pitch_us - (int32_t)pitch_center_us,
                             APP_RATE_CMD_MAX_PITCH_DPS);
        cmd_yaw_rate_dps = ((float)APP_YAW_SIGN) * App_StickOffsetUsToRateDps((int32_t)yaw_us - (int32_t)yaw_center_us,
                               APP_RATE_CMD_MAX_YAW_DPS);

        roll_rate_error_dps = cmd_roll_rate_dps - measured_roll_rate_dps;
        pitch_rate_error_dps = cmd_pitch_rate_dps - measured_pitch_rate_dps;
        yaw_rate_error_dps = cmd_yaw_rate_dps - measured_yaw_rate_dps;

        if ((dt_s < 0.0005f) || (dt_s > 0.050f))
        {
          dt_s = ((float)APP_CONTROL_LOOP_MS) * 0.001f;
        }

        if (pid_state_initialized == 0U)
        {
          prev_roll_rate_error_dps = roll_rate_error_dps;
          prev_pitch_rate_error_dps = pitch_rate_error_dps;
          prev_yaw_rate_error_dps = yaw_rate_error_dps;
          pid_state_initialized = 1U;
        }

        roll_integral_dps_s += roll_rate_error_dps * dt_s;
        pitch_integral_dps_s += pitch_rate_error_dps * dt_s;
        yaw_integral_dps_s += yaw_rate_error_dps * dt_s;

        if (active_pid_gains.roll.ki > 0.000001f)
        {
          integral_limit_roll = ((float)APP_RATE_TERM_LIMIT_US) / active_pid_gains.roll.ki;
          roll_integral_dps_s = App_ClampFloat(roll_integral_dps_s, -integral_limit_roll, integral_limit_roll);
        }

        if (active_pid_gains.pitch.ki > 0.000001f)
        {
          integral_limit_pitch = ((float)APP_RATE_TERM_LIMIT_US) / active_pid_gains.pitch.ki;
          pitch_integral_dps_s = App_ClampFloat(pitch_integral_dps_s, -integral_limit_pitch, integral_limit_pitch);
        }

        if (active_pid_gains.yaw.ki > 0.000001f)
        {
          integral_limit_yaw = ((float)APP_RATE_TERM_LIMIT_US) / active_pid_gains.yaw.ki;
          yaw_integral_dps_s = App_ClampFloat(yaw_integral_dps_s, -integral_limit_yaw, integral_limit_yaw);
        }

        roll_rate_derivative_dps_per_s = (roll_rate_error_dps - prev_roll_rate_error_dps) / dt_s;
        pitch_rate_derivative_dps_per_s = (pitch_rate_error_dps - prev_pitch_rate_error_dps) / dt_s;
        yaw_rate_derivative_dps_per_s = (yaw_rate_error_dps - prev_yaw_rate_error_dps) / dt_s;

        prev_roll_rate_error_dps = roll_rate_error_dps;
        prev_pitch_rate_error_dps = pitch_rate_error_dps;
        prev_yaw_rate_error_dps = yaw_rate_error_dps;

        roll_term_f = (active_pid_gains.roll.kp * roll_rate_error_dps) +
                      (active_pid_gains.roll.ki * roll_integral_dps_s) +
                      (active_pid_gains.roll.kd * roll_rate_derivative_dps_per_s);
        pitch_term_f = (active_pid_gains.pitch.kp * pitch_rate_error_dps) +
                       (active_pid_gains.pitch.ki * pitch_integral_dps_s) +
                       (active_pid_gains.pitch.kd * pitch_rate_derivative_dps_per_s);
        yaw_term_f = (active_pid_gains.yaw.kp * yaw_rate_error_dps) +
                     (active_pid_gains.yaw.ki * yaw_integral_dps_s) +
                     (active_pid_gains.yaw.kd * yaw_rate_derivative_dps_per_s);

        roll_term = App_ClampControlTerm((int32_t)roll_term_f, APP_RATE_TERM_LIMIT_US);
        pitch_term = App_ClampControlTerm((int32_t)pitch_term_f, APP_RATE_TERM_LIMIT_US);
        yaw_term = App_ClampControlTerm((int32_t)yaw_term_f, APP_RATE_TERM_LIMIT_US);
        throttle_term = (int32_t)throttle_us;
        if (throttle_term < (int32_t)APP_MOTOR_IDLE_US)
        {
          throttle_term = APP_MOTOR_IDLE_US;
        }
        throttle_term = App_ClampInt32(throttle_term,
                                       (int32_t)APP_MOTOR_IDLE_US,
                                       (int32_t)APP_THROTTLE_MAX_US);

        /* Keep startup spool-up symmetric: suppress attitude/yaw correction
         * very close to idle so one motor does not start noticeably earlier. */
        if (throttle_term <= ((int32_t)APP_MOTOR_IDLE_US + (int32_t)APP_LOW_THROTTLE_MIX_DISABLE_US))
        {
          roll_term = 0;
          pitch_term = 0;
          yaw_term = 0;
          roll_integral_dps_s = 0.0f;
          pitch_integral_dps_s = 0.0f;
          yaw_integral_dps_s = 0.0f;
          pid_state_initialized = 0U;
        }

        m_front_left = throttle_term + pitch_term + roll_term - yaw_term;
        m_front_right = throttle_term + pitch_term - roll_term + yaw_term;
        m_rear_right = throttle_term - pitch_term - roll_term - yaw_term;
        m_rear_left = throttle_term - pitch_term + roll_term + yaw_term;

        /* Keep mixer authority at high throttle by shifting all motors together
         * instead of clipping each output independently. */
        mix_max = m_front_left;
        if (m_front_right > mix_max)
        {
          mix_max = m_front_right;
        }
        if (m_rear_right > mix_max)
        {
          mix_max = m_rear_right;
        }
        if (m_rear_left > mix_max)
        {
          mix_max = m_rear_left;
        }

        mix_min = m_front_left;
        if (m_front_right < mix_min)
        {
          mix_min = m_front_right;
        }
        if (m_rear_right < mix_min)
        {
          mix_min = m_rear_right;
        }
        if (m_rear_left < mix_min)
        {
          mix_min = m_rear_left;
        }

        if (mix_max > (int32_t)APP_PWM_MAX_US)
        {
          mix_offset = mix_max - (int32_t)APP_PWM_MAX_US;
          m_front_left -= mix_offset;
          m_front_right -= mix_offset;
          m_rear_right -= mix_offset;
          m_rear_left -= mix_offset;
        }

        mix_min = m_front_left;
        if (m_front_right < mix_min)
        {
          mix_min = m_front_right;
        }
        if (m_rear_right < mix_min)
        {
          mix_min = m_rear_right;
        }
        if (m_rear_left < mix_min)
        {
          mix_min = m_rear_left;
        }

        if (mix_min < (int32_t)APP_PWM_MIN_US)
        {
          mix_offset = (int32_t)APP_PWM_MIN_US - mix_min;
          m_front_left += mix_offset;
          m_front_right += mix_offset;
          m_rear_right += mix_offset;
          m_rear_left += mix_offset;
        }

        s1_us = App_ClampPulseUs(m_front_left);
        s2_us = App_ClampPulseUs(m_front_right);
        s3_us = App_ClampPulseUs(m_rear_right);
        s4_us = App_ClampPulseUs(m_rear_left);

        /* Physical channels map as: CH1=LA, CH2=LF, CH3=RA, CH4=RF. */
        Motors_WriteUs(s4_us, s1_us, s3_us, s2_us);
      }
    }
  }

  if ((now_ms - last_receiver_telemetry_ms) >= 1000U)
  {
#if APP_ENABLE_RECEIVER_RUNTIME_TELEMETRY
    Telemetry_PrintReceiverState(&receiver_state);
#endif
    last_receiver_telemetry_ms = now_ms;
  }

  if ((now_ms - last_rx16_telemetry_ms) >= APP_RX16_TELEMETRY_MS)
  {
    Telemetry_PrintReceiverState16(&receiver_state);
    last_rx16_telemetry_ms = now_ms;
  }

#if APP_ENABLE_ARM_RUNTIME_TELEMETRY
  if ((now_ms - last_arm_telemetry_ms) >= APP_ARM_TELEMETRY_MS)
  {
    Telemetry_PrintArmState(motors_armed,
                            arm_switch_high,
                            throttle_low,
                            throttle_us,
                            s1_us,
                            s2_us,
                            s3_us,
                            s4_us);
    last_arm_telemetry_ms = now_ms;
  }
#endif

  if ((now_ms - last_battery_sample_ms) >= APP_BATTERY_SAMPLE_MS)
  {
    if (App_ReadBatteryVoltage(&battery_voltage_v, &battery_adc_raw) != 0U)
    {
      if (battery_voltage_valid == 0U)
      {
        battery_voltage_filtered_v = battery_voltage_v;
        battery_voltage_valid = 1U;
      }
      else
      {
        battery_voltage_filtered_v += APP_BATTERY_FILTER_ALPHA * (battery_voltage_v - battery_voltage_filtered_v);
      }
    }
    last_battery_sample_ms = now_ms;
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
    gz_dps = ((float)APP_GYRO_YAW_SIGN) * (((float)imu_raw.gyro_z) / IMU_GYRO_LSB_PER_DPS);
    (void)App_UpdateYawGyroBias(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, motors_armed, throttle_us);
    gz_dps -= g_yaw_gyro_bias_dps;

    measured_roll_rate_dps = gx_dps;
    measured_pitch_rate_dps = gy_dps;
    measured_yaw_rate_dps = gz_dps;

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

#if APP_ENABLE_IMU_RUNTIME_TELEMETRY
    if ((now_ms - last_imu_telemetry_ms) >= APP_IMU_TELEMETRY_MS)
    {
      Telemetry_PrintImuState(ax_g,
                              ay_g,
                              az_g,
                              gx_dps,
                              gy_dps,
                              gz_dps,
                              pitch_deg,
                              roll_deg,
                              yaw_deg);
      if (battery_voltage_valid != 0U)
      {
        Telemetry_PrintBatteryState(battery_voltage_filtered_v, battery_adc_raw);
      }
      last_imu_telemetry_ms = now_ms;
    }
#endif
#if APP_ENABLE_ANGLE_TELEMETRY
    Telemetry_PrintAngles(pitch_deg, roll_deg, yaw_deg);
#endif
  }
  else
  {
#if APP_ENABLE_IMU_RUNTIME_TELEMETRY
    Telemetry_PrintImuReadFailed(IMU_GetType(), IMU_GetWhoAmI());
#endif
  }

  HAL_Delay(APP_CONTROL_LOOP_MS);
}
