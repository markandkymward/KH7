#include "mag.h"

#include <math.h>
#include <string.h>
#include <stddef.h>

#define QMC5883L_I2C_ADDR       (0x0DU << 1)
#define QMC5883L_REG_DATA       0x00U
#define QMC5883L_REG_STATUS     0x06U
#define QMC5883L_REG_CTRL1      0x09U
#define QMC5883L_REG_CTRL2      0x0AU
#define QMC5883L_REG_SETRESET   0x0BU
#define QMC5883L_REG_CHIP_ID    0x0DU
#define QMC5883L_CHIP_ID_VALUE  0xFFU

#define QMC5883L_STATUS_DRDY    0x01U

/* Continuous mode, ODR=100Hz, RNG=8G, OSR=512 - common defaults for this chip. */
#define QMC5883L_CTRL1_CONFIG   0x19U
#define QMC5883L_CTRL2_SOFT_RST 0x80U
#define QMC5883L_SETRESET_DEFAULT 0x01U

#define QMC5883L_SCALE_LSB_PER_GAUSS 3000.0f

#define MAG_I2C_TIMEOUT_MS      20U

/* Hard/soft-iron calibration, stored in the unused flash sector right before
 * the PID blob's sector (Bank2 Sector7, see app.c APP_PID_FLASH_ADDRESS). */
#define MAG_CAL_FLASH_MAGIC     0x4D414743UL
#define MAG_CAL_FLASH_VERSION   1UL
#define MAG_CAL_FLASH_ADDRESS   0x081C0000UL
#define MAG_CAL_MIN_RANGE_GAUSS 0.15f /* reject a too-small (not really rotated) capture */

extern I2C_HandleTypeDef hi2c1;

static uint8_t g_healthy = 0U;
static uint8_t g_last_chip_id = 0xFFU;
static float g_x_g = 0.0f;
static float g_y_g = 0.0f;
static float g_z_g = 0.0f;

static uint8_t g_cal_active = 0U;
static uint8_t g_cal_loaded = 0U;
static float g_cal_x_min = 0.0f;
static float g_cal_x_max = 0.0f;
static float g_cal_y_min = 0.0f;
static float g_cal_y_max = 0.0f;
static float g_offset_x = 0.0f;
static float g_offset_y = 0.0f;
static float g_scale_x = 1.0f;
static float g_scale_y = 1.0f;

typedef struct
{
  uint32_t magic;
  uint32_t version;
  float offset_x;
  float offset_y;
  float scale_x;
  float scale_y;
  uint32_t crc32;
  uint32_t reserved[1];
} Mag_CalFlashBlob_t;

#if defined(__GNUC__)
#define MAG_FLASHWORD_ALIGN __attribute__((aligned(32)))
#else
#define MAG_FLASHWORD_ALIGN
#endif

typedef union
{
  Mag_CalFlashBlob_t blob;
  uint32_t words[8];
} Mag_CalFlashPage_t;

_Static_assert(sizeof(Mag_CalFlashBlob_t) <= sizeof(Mag_CalFlashPage_t), "Mag cal flash blob too large");

static uint32_t Mag_Crc32(const uint8_t *data, size_t len)
{
  uint32_t crc = 0xFFFFFFFFUL;
  size_t i;
  uint8_t bit;

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

static void Mag_CalLoad(void)
{
  const Mag_CalFlashBlob_t *stored = (const Mag_CalFlashBlob_t *)MAG_CAL_FLASH_ADDRESS;

  if ((stored->magic == MAG_CAL_FLASH_MAGIC) && (stored->version == MAG_CAL_FLASH_VERSION) &&
      (Mag_Crc32((const uint8_t *)stored, offsetof(Mag_CalFlashBlob_t, crc32)) == stored->crc32))
  {
    g_offset_x = stored->offset_x;
    g_offset_y = stored->offset_y;
    g_scale_x = stored->scale_x;
    g_scale_y = stored->scale_y;
    g_cal_loaded = 1U;
    return;
  }

  g_offset_x = 0.0f;
  g_offset_y = 0.0f;
  g_scale_x = 1.0f;
  g_scale_y = 1.0f;
  g_cal_loaded = 0U;
}

static uint8_t Mag_CalSave(void)
{
  FLASH_EraseInitTypeDef erase;
  uint32_t sector_error = 0U;
  Mag_CalFlashPage_t MAG_FLASHWORD_ALIGN page;
  const Mag_CalFlashBlob_t *written;

  memset(&page, 0xFF, sizeof(page));
  page.blob.magic = MAG_CAL_FLASH_MAGIC;
  page.blob.version = MAG_CAL_FLASH_VERSION;
  page.blob.offset_x = g_offset_x;
  page.blob.offset_y = g_offset_y;
  page.blob.scale_x = g_scale_x;
  page.blob.scale_y = g_scale_y;
  page.blob.crc32 = Mag_Crc32((const uint8_t *)&page.blob, offsetof(Mag_CalFlashBlob_t, crc32));

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return 0U;
  }

  memset(&erase, 0, sizeof(erase));
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Banks = FLASH_BANK_2;
  erase.Sector = FLASH_SECTOR_6;
  erase.NbSectors = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
  {
    (void)HAL_FLASH_Lock();
    return 0U;
  }

  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, MAG_CAL_FLASH_ADDRESS, (uint32_t)&page.words[0]) != HAL_OK)
  {
    (void)HAL_FLASH_Lock();
    return 0U;
  }

  (void)HAL_FLASH_Lock();

  written = (const Mag_CalFlashBlob_t *)MAG_CAL_FLASH_ADDRESS;
  if ((written->magic != MAG_CAL_FLASH_MAGIC) || (written->version != MAG_CAL_FLASH_VERSION))
  {
    return 0U;
  }
  if (Mag_Crc32((const uint8_t *)written, offsetof(Mag_CalFlashBlob_t, crc32)) != written->crc32)
  {
    return 0U;
  }

  return 1U;
}

static HAL_StatusTypeDef Mag_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len)
{
  return HAL_I2C_Mem_Read(&hi2c1, QMC5883L_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, MAG_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef Mag_WriteReg(uint8_t reg, uint8_t value)
{
  uint8_t val = value;
  return HAL_I2C_Mem_Write(&hi2c1, QMC5883L_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1U, MAG_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef Mag_Init(void)
{
  uint8_t chip_id = 0U;

  g_healthy = 0U;

  if (Mag_ReadRegs(QMC5883L_REG_CHIP_ID, &chip_id, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  g_last_chip_id = chip_id;
  if (chip_id != QMC5883L_CHIP_ID_VALUE)
  {
    return HAL_ERROR;
  }

  if (Mag_WriteReg(QMC5883L_REG_CTRL2, QMC5883L_CTRL2_SOFT_RST) != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(10U);

  if (Mag_WriteReg(QMC5883L_REG_SETRESET, QMC5883L_SETRESET_DEFAULT) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (Mag_WriteReg(QMC5883L_REG_CTRL1, QMC5883L_CTRL1_CONFIG) != HAL_OK)
  {
    return HAL_ERROR;
  }

  g_healthy = 1U;
  Mag_CalLoad();
  return HAL_OK;
}

HAL_StatusTypeDef Mag_Update(void)
{
  uint8_t status = 0U;
  uint8_t raw[6];
  int16_t raw_x;
  int16_t raw_y;
  int16_t raw_z;

  if (g_healthy == 0U)
  {
    return HAL_ERROR;
  }

  if (Mag_ReadRegs(QMC5883L_REG_STATUS, &status, 1U) != HAL_OK)
  {
    g_healthy = 0U;
    return HAL_ERROR;
  }
  if ((status & QMC5883L_STATUS_DRDY) == 0U)
  {
    return HAL_OK;
  }

  if (Mag_ReadRegs(QMC5883L_REG_DATA, raw, sizeof(raw)) != HAL_OK)
  {
    g_healthy = 0U;
    return HAL_ERROR;
  }

  raw_x = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
  raw_y = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
  raw_z = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8));

  g_x_g = (float)raw_x / QMC5883L_SCALE_LSB_PER_GAUSS;
  g_y_g = (float)raw_y / QMC5883L_SCALE_LSB_PER_GAUSS;
  g_z_g = (float)raw_z / QMC5883L_SCALE_LSB_PER_GAUSS;

  if (g_cal_active != 0U)
  {
    if (g_x_g < g_cal_x_min) { g_cal_x_min = g_x_g; }
    if (g_x_g > g_cal_x_max) { g_cal_x_max = g_x_g; }
    if (g_y_g < g_cal_y_min) { g_cal_y_min = g_y_g; }
    if (g_y_g > g_cal_y_max) { g_cal_y_max = g_y_g; }
  }

  return HAL_OK;
}

void Mag_CalStart(void)
{
  g_cal_active = 1U;
  g_cal_x_min = 1.0e6f;
  g_cal_x_max = -1.0e6f;
  g_cal_y_min = 1.0e6f;
  g_cal_y_max = -1.0e6f;
}

uint8_t Mag_CalStop(void)
{
  float x_range;
  float y_range;
  float avg_range;

  if (g_cal_active == 0U)
  {
    return 0U;
  }
  g_cal_active = 0U;

  x_range = g_cal_x_max - g_cal_x_min;
  y_range = g_cal_y_max - g_cal_y_min;

  /* A real 360-degree rotation should sweep well past this - reject a too-small
   * capture (board barely moved) rather than silently saving a bogus calibration. */
  if ((x_range < MAG_CAL_MIN_RANGE_GAUSS) || (y_range < MAG_CAL_MIN_RANGE_GAUSS))
  {
    return 0U;
  }

  g_offset_x = (g_cal_x_max + g_cal_x_min) * 0.5f;
  g_offset_y = (g_cal_y_max + g_cal_y_min) * 0.5f;
  avg_range = (x_range + y_range) * 0.5f;
  g_scale_x = avg_range / x_range;
  g_scale_y = avg_range / y_range;
  g_cal_loaded = 1U;

  return Mag_CalSave();
}

uint8_t Mag_IsCalActive(void)
{
  return g_cal_active;
}

uint8_t Mag_IsCalibrated(void)
{
  return g_cal_loaded;
}

float Mag_GetCalOffsetX(void)
{
  return g_offset_x;
}

float Mag_GetCalOffsetY(void)
{
  return g_offset_y;
}

float Mag_GetCalScaleX(void)
{
  return g_scale_x;
}

float Mag_GetCalScaleY(void)
{
  return g_scale_y;
}

uint8_t Mag_IsHealthy(void)
{
  return g_healthy;
}

uint8_t Mag_IsInitialized(void)
{
  return g_healthy;
}

uint8_t Mag_GetLastChipId(void)
{
  return g_last_chip_id;
}

float Mag_GetXGauss(void)
{
  return g_x_g;
}

float Mag_GetYGauss(void)
{
  return g_y_g;
}

float Mag_GetZGauss(void)
{
  return g_z_g;
}

float Mag_GetHeadingDeg(void)
{
  float heading_deg;
  float cal_x;
  float cal_y;

  cal_x = (g_x_g - g_offset_x) * g_scale_x;
  cal_y = (g_y_g - g_offset_y) * g_scale_y;

  /* Swapping the atan2 args (vs. atan2(y,x)) both rotates 90deg and mirrors the
   * rotation sense in one step - matches this board's physical chip mounting
   * (reported as "90 CW and inverted" relative to true heading). */
  heading_deg = atan2f(cal_x, cal_y) * (180.0f / 3.14159265f);
  if (heading_deg < 0.0f)
  {
    heading_deg += 360.0f;
  }
  return heading_deg;
}
