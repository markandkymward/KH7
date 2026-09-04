#include "baro.h"

#include <math.h>
#include <string.h>

#define BARO_I2C_TIMEOUT_MS      20U
#define BARO_I2C_ADDR_PRIMARY    (0x76U << 1)
#define BARO_I2C_ADDR_SECONDARY  (0x77U << 1)

/* SPA06/SPL06/DPS310-compatible register map (Goertek chip family - this board's
 * onboard barometer per Holybro's KH7 datasheet is an SPA06, not a BMP280). */
#define SPL06_REG_PRESSURE_B2    0x00U
#define SPL06_REG_TEMPERATURE_B2 0x03U
#define SPL06_REG_PRS_CFG        0x06U
#define SPL06_REG_TMP_CFG        0x07U
#define SPL06_REG_MEAS_CFG       0x08U
#define SPL06_REG_RESET          0x0CU
#define SPL06_REG_CHIP_ID        0x0DU
#define SPL06_REG_COEF           0x10U
#define SPL06_REG_COEF_SRCE      0x28U
#define SPL06_COEF_LEN_BASE      18U
#define SPL06_COEF_LEN_SPL07     21U

#define SPL06_CHIP_ID_BASE       0x10U
#define SPL06_CHIP_ID_SPL07      0x11U
#define SPL06_RESET_SOFT_RST     0x09U

#define SPL06_MEAS_CFG_COEF_RDY   (1U << 7)
#define SPL06_MEAS_CFG_SENSOR_RDY (1U << 6)
#define SPL06_MEAS_CFG_IDLE       0x00U
#define SPL06_MEAS_CFG_TEMP_SINGLE 0x02U
#define SPL06_MEAS_CFG_CONT_PT    0x07U
#define SPL06_COEF_SRCE_TMP_BIT   (1U << 7)

/* PM_RATE=32Hz (code 5), oversampling=8x (code 3) - no result bit-shift needed
 * since that's only required above 8x oversampling. */
#define SPL06_CFG_RATE32_OSR8    0x53U
#define SPL06_SCALE_FACTOR_OSR8  7864320.0f

/* Ground-level reference pressure is averaged over this many Baro_Update() calls
 * right after init/power-up, before altitude/climb-rate readings are trusted. */
#define BARO_GROUND_REF_SAMPLES  50U

#define BARO_ALT_LPF_HZ          5.0f

/* Propwash-induced altitude bias, as a function of motor PWM above idle -
 * bench-characterized 2026-08-21 (restrained ground run, props on, secured
 * stationary): a real, systematic ~8m APPARENT altitude drop appeared as
 * throttle rose from idle to ~1420us, confirmed via a quadratic least-squares
 * fit (n=240 armed samples) - meaningfully better residual than a linear fit
 * (43% lower SSE), consistent with an aerodynamic effect scaling roughly with
 * airspeed/RPM^2 rather than linearly with PWM. Subtracted from the raw
 * altitude before filtering, same "compensate the raw measurement" approach
 * used for the magnetometer's motor-current fix in mag.c. */
#define BARO_PROPWASH_ALT_A_M_PER_US2 -0.0000000245759f
#define BARO_PROPWASH_ALT_B_M_PER_US  -0.0000134765f
#define BARO_PROPWASH_ALT_C_M          -0.001635f

/* Complementary filter time constant for climb rate: fuses accelerometer-
 * integrated vertical velocity (fast/low-lag short-term response, but drifts
 * over time from accel bias/integration error) with the baro-derived raw
 * climb rate (accurate long-term/DC, but noisy - differentiating a noisy
 * altitude signal amplifies that noise, and low-pass-filtering the result
 * alone trades noise for lag). This is the standard technique used across
 * real flight controllers for exactly this reason - added 2026-08-21 after a
 * restrained ground run showed the pure-baro-derivative approach swinging
 * wildly (over 1000cm/s peak-to-peak) even while completely stationary.
 * Larger tau = more accel-trust/less baro-correction (smoother but slower to
 * correct accel drift); smaller tau = more baro-trust (noisier but tighter
 * long-term accuracy).
 *
 * Lowered from 2.0f to 0.5f on 2026-08-22: the vertical-accel input comes from
 * the same gravity-direction vector the AHRS uses (Attitude_GetVerticalAccelMps2()),
 * which transiently corrupts during hard maneuvering (large/fast pitch-roll
 * changes couple real translational acceleration into what's supposed to be a
 * pure-gravity reading). At tau=2.0s the filter trusted that corrupted
 * prediction ~98-99%/sample and only corrected toward the honest baro trend
 * over a ~2s window, so a hard maneuver could leave the reported climb rate
 * sign-wrong for a couple of seconds afterward - confirmed directly in a real
 * flight log: altitude climbing 1.0m->2.17m over ~2.5s (t=62-67s) while
 * baro_vz_cms read NEGATIVE (-0.4 to -0.7 m/s) through most of that window,
 * concurrent with large pitch/roll swings. That fed straight into ALTHOLD's
 * climb_rate_error, meaning the trim could actively fight the wrong direction
 * during real maneuvering - reported by the pilot as fighting the aircraft to
 * hold altitude. 0.5s corrects back to the honest baro trend much faster,
 * trading back some of the noise-smoothing tau=2.0s bought. */
#define BARO_VZ_COMPL_FILTER_TAU_S 0.5f

typedef struct
{
  int16_t c0;
  int16_t c1;
  int32_t c00;
  int32_t c10;
  int16_t c01;
  int16_t c11;
  int16_t c20;
  int16_t c21;
  int16_t c30;
  /* SPL07-003 only (chip_id 0x11) - stay 0 on the base SPL06/DPS310/SPA06 chip,
   * which makes the extended compensation formula collapse to the base formula. */
  int16_t c31;
  int16_t c40;
} Baro_Calib_t;

extern I2C_HandleTypeDef hi2c1;

static Baro_Calib_t g_calib;
static uint16_t g_i2c_addr = 0U;
static uint8_t g_healthy = 0U;
static uint8_t g_last_chip_id = 0xFFU;
static uint8_t g_is_spl07 = 0U;

static uint8_t g_ground_ref_captured = 0U;
static uint16_t g_ground_ref_count = 0U;
static double g_ground_ref_accum_pa = 0.0;
static float g_ground_pressure_pa = 0.0f;

static float g_alt_filt_m = 0.0f;
static float g_vz_filt_mps = 0.0f;
static float g_prev_alt_m = 0.0f;
static uint32_t g_last_update_ms = 0U;
static uint8_t g_have_prev_alt = 0U;
/* Raw (ground-reference-zeroed, propwash-bias-corrected, but NOT yet
 * BARO_ALT_LPF_HZ-smoothed) altitude, added 2026-08-29 for vert_ekf.c - an EKF
 * measurement update wants the sensor's own noise characteristics, not a signal
 * that's already been through a separate filter (double-filtering adds lag and
 * breaks the white-noise-residual assumption a Kalman R term relies on).
 * Baro_GetAltitudeM() (the LPF'd value) is unaffected - still what everything
 * else in this codebase should keep using. */
static float g_alt_raw_m = 0.0f;

static float Baro_LpfAlpha(float dt_s, float cutoff_hz)
{
  float tau_s = 1.0f / (2.0f * 3.14159265f * cutoff_hz);
  return 1.0f - expf(-dt_s / tau_s);
}

/* Sign-extends the low 'bits' bits of a raw register value (2's complement). */
static int32_t Baro_SignExtend(uint32_t raw, uint8_t bits)
{
  uint32_t sign_bit = 1UL << (bits - 1U);
  return (int32_t)((raw ^ sign_bit) - sign_bit);
}

/* I2C1 is shared with mag.c - see mag.c's I2C1_Recover() for why a failed
 * transfer here resets the whole peripheral rather than just this driver's
 * state (2026-08-21). */
static void I2C1_Recover(void)
{
  (void)HAL_I2C_DeInit(&hi2c1);
  (void)HAL_I2C_Init(&hi2c1);
}

static HAL_StatusTypeDef Baro_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len)
{
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, g_i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, BARO_I2C_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    I2C1_Recover();
  }
  return status;
}

static HAL_StatusTypeDef Baro_WriteReg(uint8_t reg, uint8_t value)
{
  uint8_t val = value;
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, g_i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, &val, 1U, BARO_I2C_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    I2C1_Recover();
  }
  return status;
}

static HAL_StatusTypeDef Baro_ProbeAddress(uint16_t addr)
{
  uint8_t chip_id = 0U;

  g_i2c_addr = addr;
  if (Baro_ReadRegs(SPL06_REG_CHIP_ID, &chip_id, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  g_last_chip_id = chip_id;
  if ((chip_id != SPL06_CHIP_ID_BASE) && (chip_id != SPL06_CHIP_ID_SPL07))
  {
    return HAL_ERROR;
  }
  g_is_spl07 = (chip_id == SPL06_CHIP_ID_SPL07) ? 1U : 0U;
  return HAL_OK;
}

/* SPL06/DPS310/SPA06 pressure compensation (datasheet section "Calculate Compensated
 * Pressure Values") - p_sc/t_sc are raw ADC counts scaled by the oversampling factor.
 * c31/c40 are 0 on the base chip, so this formula covers both variants. */
static double Baro_CompensatePressurePa(float p_sc, float t_sc)
{
  double p_sc_d = (double)p_sc;
  double t_sc_d = (double)t_sc;

  return (double)g_calib.c00 +
         p_sc_d * ((double)g_calib.c10 + p_sc_d * ((double)g_calib.c20 + p_sc_d * ((double)g_calib.c30 + p_sc_d * (double)g_calib.c40))) +
         t_sc_d * ((double)g_calib.c01 + p_sc_d * ((double)g_calib.c11 + p_sc_d * ((double)g_calib.c21 + p_sc_d * (double)g_calib.c31)));
}

HAL_StatusTypeDef Baro_Init(void)
{
  uint8_t calib_raw[SPL06_COEF_LEN_SPL07];
  uint8_t status = 0U;
  uint8_t coef_srce = 0U;
  uint8_t coef_len;

  g_healthy = 0U;
  g_ground_ref_captured = 0U;
  g_ground_ref_count = 0U;
  g_ground_ref_accum_pa = 0.0;
  g_have_prev_alt = 0U;
  g_alt_filt_m = 0.0f;
  g_vz_filt_mps = 0.0f;
  g_calib.c31 = 0;
  g_calib.c40 = 0;

  if ((Baro_ProbeAddress(BARO_I2C_ADDR_PRIMARY) != HAL_OK) &&
      (Baro_ProbeAddress(BARO_I2C_ADDR_SECONDARY) != HAL_OK))
  {
    g_i2c_addr = 0U;
    return HAL_ERROR;
  }

  if (Baro_WriteReg(SPL06_REG_RESET, SPL06_RESET_SOFT_RST) != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(40U);

  if (Baro_ReadRegs(SPL06_REG_MEAS_CFG, &status, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if ((status & (SPL06_MEAS_CFG_COEF_RDY | SPL06_MEAS_CFG_SENSOR_RDY)) !=
      (SPL06_MEAS_CFG_COEF_RDY | SPL06_MEAS_CFG_SENSOR_RDY))
  {
    return HAL_ERROR;
  }

  coef_len = (g_is_spl07 != 0U) ? SPL06_COEF_LEN_SPL07 : SPL06_COEF_LEN_BASE;
  if (Baro_ReadRegs(SPL06_REG_COEF, calib_raw, coef_len) != HAL_OK)
  {
    return HAL_ERROR;
  }

  g_calib.c0 = (int16_t)Baro_SignExtend(((uint32_t)calib_raw[0] << 4) | ((uint32_t)calib_raw[1] >> 4), 12U);
  g_calib.c1 = (int16_t)Baro_SignExtend((((uint32_t)calib_raw[1] & 0x0FU) << 8) | (uint32_t)calib_raw[2], 12U);
  g_calib.c00 = Baro_SignExtend(((uint32_t)calib_raw[3] << 12) | ((uint32_t)calib_raw[4] << 4) | ((uint32_t)calib_raw[5] >> 4), 20U);
  g_calib.c10 = Baro_SignExtend((((uint32_t)calib_raw[5] & 0x0FU) << 16) | ((uint32_t)calib_raw[6] << 8) | (uint32_t)calib_raw[7], 20U);
  g_calib.c01 = (int16_t)(((uint16_t)calib_raw[8] << 8) | (uint16_t)calib_raw[9]);
  g_calib.c11 = (int16_t)(((uint16_t)calib_raw[10] << 8) | (uint16_t)calib_raw[11]);
  g_calib.c20 = (int16_t)(((uint16_t)calib_raw[12] << 8) | (uint16_t)calib_raw[13]);
  g_calib.c21 = (int16_t)(((uint16_t)calib_raw[14] << 8) | (uint16_t)calib_raw[15]);
  g_calib.c30 = (int16_t)(((uint16_t)calib_raw[16] << 8) | (uint16_t)calib_raw[17]);
  if (g_is_spl07 != 0U)
  {
    g_calib.c31 = (int16_t)Baro_SignExtend(((uint32_t)calib_raw[18] << 4) | ((uint32_t)calib_raw[19] >> 4), 12U);
    g_calib.c40 = (int16_t)Baro_SignExtend((((uint32_t)calib_raw[19] & 0x0FU) << 8) | (uint32_t)calib_raw[20], 12U);
  }

  /* Idle before reconfiguring, then apply the vendor-documented fuse-bit fix-up
   * (harmless on chips without the underlying issue) before the first temp read. */
  if (Baro_WriteReg(SPL06_REG_MEAS_CFG, SPL06_MEAS_CFG_IDLE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  (void)Baro_WriteReg(0x0EU, 0xA5U);
  (void)Baro_WriteReg(0x0FU, 0x96U);
  (void)Baro_WriteReg(0x62U, 0x02U);
  (void)Baro_WriteReg(0x0EU, 0x00U);
  if (Baro_WriteReg(SPL06_REG_MEAS_CFG, SPL06_MEAS_CFG_TEMP_SINGLE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(40U);

  if (Baro_WriteReg(SPL06_REG_PRS_CFG, SPL06_CFG_RATE32_OSR8) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (g_is_spl07 != 0U)
  {
    if (Baro_WriteReg(SPL06_REG_TMP_CFG, SPL06_CFG_RATE32_OSR8) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }
  else
  {
    /* TMP_EXT bit must mirror the chip's own coefficient-source bit (register 0x28) -
     * harmless if that register is unimplemented on this particular chip (reads 0). */
    (void)Baro_ReadRegs(SPL06_REG_COEF_SRCE, &coef_srce, 1U);
    if (Baro_WriteReg(SPL06_REG_TMP_CFG, SPL06_CFG_RATE32_OSR8 | (coef_srce & SPL06_COEF_SRCE_TMP_BIT)) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  if (Baro_WriteReg(SPL06_REG_MEAS_CFG, SPL06_MEAS_CFG_CONT_PT) != HAL_OK)
  {
    return HAL_ERROR;
  }

  g_healthy = 1U;
  return HAL_OK;
}

HAL_StatusTypeDef Baro_Update(float motor_power_delta_us, float vertical_accel_mps2)
{
  uint8_t raw[6];
  int32_t raw_p;
  int32_t raw_t;
  float p_sc;
  float t_sc;
  double pressure_pa;
  float altitude_m;
  float propwash_bias_m;
  float dt_s;
  float alpha;
  uint32_t now_ms;

  if (g_healthy == 0U)
  {
    return HAL_ERROR;
  }

  if (Baro_ReadRegs(SPL06_REG_PRESSURE_B2, raw, sizeof(raw)) != HAL_OK)
  {
    g_healthy = 0U;
    return HAL_ERROR;
  }

  raw_p = Baro_SignExtend(((uint32_t)raw[0] << 16) | ((uint32_t)raw[1] << 8) | (uint32_t)raw[2], 24U);
  raw_t = Baro_SignExtend(((uint32_t)raw[3] << 16) | ((uint32_t)raw[4] << 8) | (uint32_t)raw[5], 24U);

  p_sc = (float)raw_p / SPL06_SCALE_FACTOR_OSR8;
  t_sc = (float)raw_t / SPL06_SCALE_FACTOR_OSR8;

  pressure_pa = Baro_CompensatePressurePa(p_sc, t_sc);
  if (pressure_pa <= 0.0)
  {
    return HAL_ERROR;
  }

  now_ms = HAL_GetTick();

  if (g_ground_ref_captured == 0U)
  {
    g_ground_ref_accum_pa += pressure_pa;
    g_ground_ref_count++;
    if (g_ground_ref_count >= BARO_GROUND_REF_SAMPLES)
    {
      g_ground_pressure_pa = (float)(g_ground_ref_accum_pa / (double)g_ground_ref_count);
      g_ground_ref_captured = 1U;
      g_last_update_ms = now_ms;
    }
    return HAL_OK;
  }

  altitude_m = 44330.0f * (1.0f - powf((float)(pressure_pa / (double)g_ground_pressure_pa), 0.1902949571836346f));

  /* Propwash compensation - subtract the bench-characterized bias from the
   * RAW altitude before any filtering, same as the mag.c motor-current fix -
   * see the constants' comment above. */
  propwash_bias_m = (BARO_PROPWASH_ALT_A_M_PER_US2 * motor_power_delta_us * motor_power_delta_us) +
                     (BARO_PROPWASH_ALT_B_M_PER_US * motor_power_delta_us) +
                     BARO_PROPWASH_ALT_C_M;
  altitude_m -= propwash_bias_m;
  g_alt_raw_m = altitude_m;

  dt_s = ((float)(now_ms - g_last_update_ms)) * 0.001f;
  if (dt_s < 0.001f)
  {
    dt_s = 0.001f;
  }

  alpha = Baro_LpfAlpha(dt_s, BARO_ALT_LPF_HZ);
  g_alt_filt_m += alpha * (altitude_m - g_alt_filt_m);

  if (g_have_prev_alt != 0U)
  {
    /* Complementary filter: accelerometer-integrated vertical velocity for
     * fast/low-lag short-term response, corrected toward the raw
     * baro-derived climb rate (noisy, but accurate long-term/DC) at a rate
     * set by BARO_VZ_COMPL_FILTER_TAU_S - see that constant's comment. */
    float raw_vz_mps = (altitude_m - g_prev_alt_m) / dt_s;
    float compl_alpha = BARO_VZ_COMPL_FILTER_TAU_S / (BARO_VZ_COMPL_FILTER_TAU_S + dt_s);
    float vz_predicted_mps = g_vz_filt_mps + (vertical_accel_mps2 * dt_s);
    g_vz_filt_mps = (compl_alpha * vz_predicted_mps) + ((1.0f - compl_alpha) * raw_vz_mps);
  }
  g_prev_alt_m = altitude_m;
  g_have_prev_alt = 1U;
  g_last_update_ms = now_ms;

  return HAL_OK;
}

uint8_t Baro_IsHealthy(void)
{
  return (uint8_t)((g_healthy != 0U) && (g_ground_ref_captured != 0U));
}

uint8_t Baro_IsInitialized(void)
{
  return g_healthy;
}

uint8_t Baro_GetDetectedAddr7(void)
{
  return (uint8_t)(g_i2c_addr >> 1);
}

uint8_t Baro_GetLastChipId(void)
{
  return g_last_chip_id;
}

float Baro_GetAltitudeM(void)
{
  return g_alt_filt_m;
}

float Baro_GetRawAltitudeM(void)
{
  return g_alt_raw_m;
}

float Baro_GetClimbRateMps(void)
{
  return g_vz_filt_mps;
}
