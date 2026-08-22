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

/* AMR (anisotropic magnetoresistive) sensing bridges like this chip's can get
 * desensitized/"pinned" toward one field direction after exposure to a strong
 * enough external field, and stay that way until a fresh SET/RESET pulse
 * restores full sensitivity - that's what QMC5883L_REG_SETRESET is for. Added
 * 2026-08-21: Mag_Init() only wrote it once at boot, which matched real bench
 * data on the assembled aircraft (motor magnets close enough to matter,
 * confirmed present at all mounting distances/locations tested) - heading
 * tracked correctly for roughly the first 90-130deg of a real, gyro-confirmed
 * full rotation, then went flat (near-constant, still noisy - not a stuck
 * read, not I2C failure, not saturation) for the rest, regardless of
 * recalibration or moving the whole rig to a different room. Re-arming
 * periodically rather than once fixes exactly that failure mode. */
#define MAG_SETRESET_INTERVAL_SAMPLES 20U

/* Hard/soft-iron calibration, stored in the unused flash sector right before
 * the PID blob's sector (Bank2 Sector7, see app.c APP_PID_FLASH_ADDRESS).
 * Bumped to a full 3D (X/Y/Z) calibration 2026-08-21 alongside the HGLRC M100
 * Pro module swap - the old calibration only ever tracked X/Y (see
 * Mag_GetHeadingDeg()'s old "mz not hard/soft-iron calibrated" comment), which
 * only corrects distortion in the level plane and requires a flat 360-degree
 * spin.
 * Bumped again (V4) 2026-08-21 to a proper ellipsoid (cross-axis/soft-iron)
 * fit - real bench data (raw firmware heading vs a handheld reference compass
 * at 0/90/180/270deg) showed errors from -5deg up to +92deg that don't fit
 * any constant offset or per-axis scale, the signature of an uncorrected
 * elliptical (not just axis-aligned) distortion. The old model was
 * center + independent per-axis scale (3+3 floats); this one is
 * center + a full symmetric 3x3 correction matrix (3+6 floats) - see
 * Mag_CalStop() for the fit. Still fits the same 2 flash-words. */
#define MAG_CAL_FLASH_MAGIC     0x4D414743UL
#define MAG_CAL_FLASH_VERSION   4UL
#define MAG_CAL_FLASH_ADDRESS   0x081C0000UL
#define MAG_CAL_MIN_RANGE_GAUSS 0.15f /* reject a too-small (not really rotated) capture, per axis */

/* Minimum accepted full-3D field magnitude feeding atan2f(), as a fraction of
 * the expected post-calibration magnitude. Added 2026-08-21 after bench
 * rotation data showed the corrected field magnitude was NOT constant through
 * a yaw sweep - a symptom of residual hard/soft-iron calibration error. At the
 * magnitude minimum, atan2f becomes hypersensitive to noise, producing a real
 * ~100deg spurious heading swing mid-rotation with no actual attitude change
 * to explain it. Rather than trust an unstable reading in that window, hold
 * the last good heading. With the ellipsoid fit below, the corrected vector's
 * expected magnitude is exactly 1.0 by construction (the fit normalizes to a
 * unit sphere), so this is now a fixed constant rather than a per-calibration
 * value. */
#define MAG_HEADING_MIN_MAG_RATIO 0.5f
#define MAG_CAL_EXPECTED_MAGNITUDE 1.0f

/* Scratch buffer for raw (x,y,z) samples collected during a calibration
 * tumble, used for the ellipsoid least-squares fit in Mag_CalStop() - needs
 * every sample (not just min/max) to fit a general 3x3 correction matrix, not
 * just per-axis offset/scale. RAM_D2 (0x30000000, 288KB) is otherwise
 * entirely unused in this project (same fixed-address pattern as RAM_D3 in
 * fault_record.h) - plenty of room for a generous sample count without
 * touching the tightly-budgeted DTCM used for real-time control-loop data.
 * Transient/scratch only - not persisted, not meaningful outside an active
 * calibration session. */
#define MAG_CAL_SAMPLES_ADDR 0x30000000UL
#define MAG_CAL_MAX_SAMPLES  3000U /* ~30s at the 100Hz QMC5883L ODR */

typedef struct
{
  float x;
  float y;
  float z;
} Mag_CalSample_t;

#define MAG_CAL_SAMPLES ((volatile Mag_CalSample_t *)MAG_CAL_SAMPLES_ADDR)

extern I2C_HandleTypeDef hi2c1;

static uint8_t g_healthy = 0U;
static uint8_t g_last_chip_id = 0xFFU;
static uint8_t g_setreset_counter = 0U;
static float g_x_g = 0.0f;
static float g_y_g = 0.0f;
static float g_z_g = 0.0f;

static uint8_t g_cal_active = 0U;
static uint8_t g_cal_loaded = 0U;
static float g_cal_x_min = 0.0f;
static float g_cal_x_max = 0.0f;
static float g_cal_y_min = 0.0f;
static float g_cal_y_max = 0.0f;
static float g_cal_z_min = 0.0f;
static float g_cal_z_max = 0.0f;
static uint32_t g_cal_sample_count = 0U;
/* Ellipsoid fit result: corrected = W * (raw - center). W is symmetric, so
 * only 6 unique entries are stored/needed (w_xx,w_yy,w_zz on the diagonal,
 * w_xy,w_xz,w_yz off-diagonal, mirrored). Identity + zero center is the
 * uncalibrated default (matches the old offset=0/scale=1 default). */
static float g_center_x = 0.0f;
static float g_center_y = 0.0f;
static float g_center_z = 0.0f;
static float g_w_xx = 1.0f;
static float g_w_yy = 1.0f;
static float g_w_zz = 1.0f;
static float g_w_xy = 0.0f;
static float g_w_xz = 0.0f;
static float g_w_yz = 0.0f;
static float g_last_heading_deg = 0.0f;
static uint8_t g_last_heading_valid = 0U;

typedef struct
{
  uint32_t magic;
  uint32_t version;
  float center_x;
  float center_y;
  float center_z;
  float w_xx;
  float w_yy;
  float w_zz;
  float w_xy;
  float w_xz;
  float w_yz;
  uint32_t crc32;
} Mag_CalFlashBlob_t;

#if defined(__GNUC__)
#define MAG_FLASHWORD_ALIGN __attribute__((aligned(32)))
#else
#define MAG_FLASHWORD_ALIGN
#endif

typedef union
{
  Mag_CalFlashBlob_t blob;
  uint32_t words[16]; /* 2 flash-words (32 bytes each) - blob is 48 bytes */
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
    g_center_x = stored->center_x;
    g_center_y = stored->center_y;
    g_center_z = stored->center_z;
    g_w_xx = stored->w_xx;
    g_w_yy = stored->w_yy;
    g_w_zz = stored->w_zz;
    g_w_xy = stored->w_xy;
    g_w_xz = stored->w_xz;
    g_w_yz = stored->w_yz;
    g_cal_loaded = 1U;
    return;
  }

  g_center_x = 0.0f;
  g_center_y = 0.0f;
  g_center_z = 0.0f;
  g_w_xx = 1.0f;
  g_w_yy = 1.0f;
  g_w_zz = 1.0f;
  g_w_xy = 0.0f;
  g_w_xz = 0.0f;
  g_w_yz = 0.0f;
  g_cal_loaded = 0U;
}

static uint8_t Mag_CalSave(void)
{
  FLASH_EraseInitTypeDef erase;
  uint32_t sector_error = 0U;
  uint32_t address;
  Mag_CalFlashPage_t MAG_FLASHWORD_ALIGN page;
  const Mag_CalFlashBlob_t *written;
  uint8_t write_index;

  memset(&page, 0xFF, sizeof(page));
  page.blob.magic = MAG_CAL_FLASH_MAGIC;
  page.blob.version = MAG_CAL_FLASH_VERSION;
  page.blob.center_x = g_center_x;
  page.blob.center_y = g_center_y;
  page.blob.center_z = g_center_z;
  page.blob.w_xx = g_w_xx;
  page.blob.w_yy = g_w_yy;
  page.blob.w_zz = g_w_zz;
  page.blob.w_xy = g_w_xy;
  page.blob.w_xz = g_w_xz;
  page.blob.w_yz = g_w_yz;
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

  /* Blob grew past one flash-word (32 bytes) with the Z axis added - two
   * consecutive 32-byte-aligned writes now, same pattern as app.c's
   * App_PidFlashBlob_t save (also multi-word). */
  address = MAG_CAL_FLASH_ADDRESS;
  for (write_index = 0U; write_index < 2U; write_index++)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address,
                          (uint32_t)&page.words[write_index * 8U]) != HAL_OK)
    {
      (void)HAL_FLASH_Lock();
      return 0U;
    }
    address += 32U;
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

/* I2C1 is shared with baro.c. A timed-out transfer can leave hi2c1.State stuck
 * (HAL refuses to start a new transfer unless State==READY), which would wedge
 * both devices on the bus, not just this one - so recover on any failure here
 * rather than just marking this sensor unhealthy and leaving the peripheral in
 * a bad state for baro's next attempt too. See imu.c's IMU_SpiRecover() for the
 * same pattern on the SPI4/IMU side (2026-08-21). */
static void I2C1_Recover(void)
{
  (void)HAL_I2C_DeInit(&hi2c1);
  (void)HAL_I2C_Init(&hi2c1);
}

static HAL_StatusTypeDef Mag_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len)
{
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, QMC5883L_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, MAG_I2C_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    I2C1_Recover();
  }
  return status;
}

static HAL_StatusTypeDef Mag_WriteReg(uint8_t reg, uint8_t value)
{
  uint8_t val = value;
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, QMC5883L_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1U, MAG_I2C_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    I2C1_Recover();
  }
  return status;
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

  g_setreset_counter++;
  if (g_setreset_counter >= MAG_SETRESET_INTERVAL_SAMPLES)
  {
    g_setreset_counter = 0U;
    (void)Mag_WriteReg(QMC5883L_REG_SETRESET, QMC5883L_SETRESET_DEFAULT);
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
    if (g_z_g < g_cal_z_min) { g_cal_z_min = g_z_g; }
    if (g_z_g > g_cal_z_max) { g_cal_z_max = g_z_g; }

    if (g_cal_sample_count < MAG_CAL_MAX_SAMPLES)
    {
      MAG_CAL_SAMPLES[g_cal_sample_count].x = g_x_g;
      MAG_CAL_SAMPLES[g_cal_sample_count].y = g_y_g;
      MAG_CAL_SAMPLES[g_cal_sample_count].z = g_z_g;
      g_cal_sample_count++;
    }
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
  g_cal_z_min = 1.0e6f;
  g_cal_z_max = -1.0e6f;
  g_cal_sample_count = 0U;
}

/* Solves the n x n linear system a*x = b via Gaussian elimination with
 * partial pivoting. a is row-major (a[(i*n)+j]) and is destroyed; b becomes
 * the solution x on return. Returns 0 if a pivot is too close to zero
 * (singular/ill-conditioned system - e.g. a degenerate calibration capture),
 * 1 on success. Used both for the 9-parameter ellipsoid fit (n=9) and the 3x3
 * center solve (n=3) below - validated offline (Python) against a synthetic
 * ellipsoid with known ground truth before being ported here. */
static uint8_t Mag_SolveLinear(float *a, float *b, uint8_t n)
{
  uint8_t i;
  uint8_t j;
  uint8_t k;
  uint8_t pivot_row;
  uint8_t row;
  float pivot_val;
  float factor;
  float tmp;
  float sum;

  for (k = 0U; k < n; k++)
  {
    pivot_row = k;
    pivot_val = fabsf(a[(k * n) + k]);
    for (i = (uint8_t)(k + 1U); i < n; i++)
    {
      if (fabsf(a[(i * n) + k]) > pivot_val)
      {
        pivot_val = fabsf(a[(i * n) + k]);
        pivot_row = i;
      }
    }
    if (pivot_val < 1.0e-12f)
    {
      return 0U;
    }
    if (pivot_row != k)
    {
      for (j = 0U; j < n; j++)
      {
        tmp = a[(k * n) + j];
        a[(k * n) + j] = a[(pivot_row * n) + j];
        a[(pivot_row * n) + j] = tmp;
      }
      tmp = b[k];
      b[k] = b[pivot_row];
      b[pivot_row] = tmp;
    }

    for (i = (uint8_t)(k + 1U); i < n; i++)
    {
      factor = a[(i * n) + k] / a[(k * n) + k];
      for (j = k; j < n; j++)
      {
        a[(i * n) + j] -= factor * a[(k * n) + j];
      }
      b[i] -= factor * b[k];
    }
  }

  for (i = n; i > 0U; i--)
  {
    row = (uint8_t)(i - 1U);
    sum = b[row];
    for (j = (uint8_t)(row + 1U); j < n; j++)
    {
      sum -= a[(row * n) + j] * b[j];
    }
    b[row] = sum / a[(row * n) + row];
  }

  return 1U;
}

/* Classic cyclic Jacobi eigenvalue algorithm, specialized for a symmetric 3x3
 * matrix - converges in just a few sweeps for a matrix this small, so a
 * fixed iteration count (rather than a convergence check) is simpler and has
 * bounded runtime, which matters more than raw precision here since this only
 * runs once per calibration. On return, a's diagonal holds the eigenvalues
 * and v's columns hold the corresponding (orthonormal) eigenvectors.
 * Validated offline against NumPy's eigh() on non-trivial test matrices
 * before being ported here. */
static void Mag_JacobiEigen3x3(float a[3][3], float v[3][3])
{
  uint8_t sweep;
  uint8_t p;
  uint8_t q;
  uint8_t i;
  uint8_t jcol;
  float theta;
  float t;
  float c;
  float s;
  float app;
  float aqq;
  float apq;
  float vip;
  float viq;
  float apk;
  float aqk;

  for (i = 0U; i < 3U; i++)
  {
    for (jcol = 0U; jcol < 3U; jcol++)
    {
      v[i][jcol] = (i == jcol) ? 1.0f : 0.0f;
    }
  }

  for (sweep = 0U; sweep < 12U; sweep++)
  {
    for (p = 0U; p < 2U; p++)
    {
      for (q = (uint8_t)(p + 1U); q < 3U; q++)
      {
        apq = a[p][q];
        if (fabsf(apq) < 1.0e-9f)
        {
          continue;
        }

        app = a[p][p];
        aqq = a[q][q];
        theta = (aqq - app) / (2.0f * apq);
        if (theta >= 0.0f)
        {
          t = 1.0f / (theta + sqrtf(1.0f + (theta * theta)));
        }
        else
        {
          t = -1.0f / (-theta + sqrtf(1.0f + (theta * theta)));
        }
        c = 1.0f / sqrtf(1.0f + (t * t));
        s = t * c;

        a[p][p] = app - (t * apq);
        a[q][q] = aqq + (t * apq);
        a[p][q] = 0.0f;
        a[q][p] = 0.0f;

        for (i = 0U; i < 3U; i++)
        {
          if ((i != p) && (i != q))
          {
            apk = a[p][i];
            aqk = a[q][i];
            a[p][i] = (c * apk) - (s * aqk);
            a[i][p] = a[p][i];
            a[q][i] = (s * apk) + (c * aqk);
            a[i][q] = a[q][i];
          }
        }

        for (i = 0U; i < 3U; i++)
        {
          vip = v[i][p];
          viq = v[i][q];
          v[i][p] = (c * vip) - (s * viq);
          v[i][q] = (s * vip) + (c * viq);
        }
      }
    }
  }
}

uint8_t Mag_CalStop(void)
{
  float x_range;
  float y_range;
  float z_range;
  uint32_t sample_index;
  float dtd[81];
  float dtb[9];
  float row_vec[9];
  uint8_t r;
  uint8_t col;
  float m[3][3];
  float ghi[3];
  float neg_ghi[3];
  float center[3];
  float scale_r;
  float mnorm[3][3];
  float eigvals[3][3];
  float eigvecs[3][3];
  float w[3][3];
  uint8_t k;

  if (g_cal_active == 0U)
  {
    return 0U;
  }
  g_cal_active = 0U;

  x_range = g_cal_x_max - g_cal_x_min;
  y_range = g_cal_y_max - g_cal_y_min;
  z_range = g_cal_z_max - g_cal_z_min;

  /* A full 3D tumble (not just a flat spin) should sweep every axis well past
   * this - in particular the Z requirement forces genuine tilting through
   * multiple orientations, rejecting a "spun flat but never tilted" capture
   * that would leave Z hard-iron-uncorrected the same way the old 2D-only
   * calibration always did. */
  if ((x_range < MAG_CAL_MIN_RANGE_GAUSS) || (y_range < MAG_CAL_MIN_RANGE_GAUSS) ||
      (z_range < MAG_CAL_MIN_RANGE_GAUSS))
  {
    return 0U;
  }

  /* Need enough points for a well-conditioned 9-parameter fit - a bare
   * minimum of 9 would make the system exactly (not over-)determined, which
   * is numerically fragile; require a healthy margin. */
  if (g_cal_sample_count < 200U)
  {
    return 0U;
  }

  /* Ellipsoid fit: A x^2 + B y^2 + C z^2 + 2D xy + 2E xz + 2F yz + 2G x + 2H y
   * + 2I z = 1, solved via least squares (normal equations) over every
   * collected sample - accumulate D^T D (9x9) and D^T*1 (9-vector) in one
   * pass, matching the offline-validated Python reference exactly. */
  memset(dtd, 0, sizeof(dtd));
  memset(dtb, 0, sizeof(dtb));

  for (sample_index = 0U; sample_index < g_cal_sample_count; sample_index++)
  {
    float sx = MAG_CAL_SAMPLES[sample_index].x;
    float sy = MAG_CAL_SAMPLES[sample_index].y;
    float sz = MAG_CAL_SAMPLES[sample_index].z;

    row_vec[0] = sx * sx;
    row_vec[1] = sy * sy;
    row_vec[2] = sz * sz;
    row_vec[3] = 2.0f * sx * sy;
    row_vec[4] = 2.0f * sx * sz;
    row_vec[5] = 2.0f * sy * sz;
    row_vec[6] = 2.0f * sx;
    row_vec[7] = 2.0f * sy;
    row_vec[8] = 2.0f * sz;

    for (r = 0U; r < 9U; r++)
    {
      dtb[r] += row_vec[r];
      for (col = 0U; col < 9U; col++)
      {
        dtd[(r * 9U) + col] += row_vec[r] * row_vec[col];
      }
    }
  }

  if (Mag_SolveLinear(dtd, dtb, 9U) == 0U)
  {
    return 0U;
  }

  /* dtb now holds [A,B,C,D,E,F,G,H,I]. */
  m[0][0] = dtb[0]; m[0][1] = dtb[3]; m[0][2] = dtb[4];
  m[1][0] = dtb[3]; m[1][1] = dtb[1]; m[1][2] = dtb[5];
  m[2][0] = dtb[4]; m[2][1] = dtb[5]; m[2][2] = dtb[2];
  ghi[0] = dtb[6]; ghi[1] = dtb[7]; ghi[2] = dtb[8];

  /* center = -M^-1 * ghi (solve M*center = -ghi). */
  neg_ghi[0] = -ghi[0];
  neg_ghi[1] = -ghi[1];
  neg_ghi[2] = -ghi[2];
  {
    float m_copy[9];
    m_copy[0] = m[0][0]; m_copy[1] = m[0][1]; m_copy[2] = m[0][2];
    m_copy[3] = m[1][0]; m_copy[4] = m[1][1]; m_copy[5] = m[1][2];
    m_copy[6] = m[2][0]; m_copy[7] = m[2][1]; m_copy[8] = m[2][2];
    if (Mag_SolveLinear(m_copy, neg_ghi, 3U) == 0U)
    {
      return 0U;
    }
  }
  center[0] = neg_ghi[0];
  center[1] = neg_ghi[1];
  center[2] = neg_ghi[2];

  /* R = 1 - center . ghi (the ellipsoid's constant term after re-centering -
   * see the offline-validated derivation). Must be positive for a valid,
   * bounded ellipsoid. */
  scale_r = 1.0f - ((center[0] * ghi[0]) + (center[1] * ghi[1]) + (center[2] * ghi[2]));
  if (scale_r <= 1.0e-9f)
  {
    return 0U;
  }

  for (r = 0U; r < 3U; r++)
  {
    for (col = 0U; col < 3U; col++)
    {
      mnorm[r][col] = m[r][col] / scale_r;
    }
  }

  Mag_JacobiEigen3x3(mnorm, eigvecs);
  eigvals[0][0] = mnorm[0][0];
  eigvals[1][1] = mnorm[1][1];
  eigvals[2][2] = mnorm[2][2];

  /* All three eigenvalues must be positive for mnorm to represent a genuine
   * (not degenerate/hyperbolic) ellipsoid - reject rather than take sqrtf of
   * a negative number. */
  if ((eigvals[0][0] <= 0.0f) || (eigvals[1][1] <= 0.0f) || (eigvals[2][2] <= 0.0f))
  {
    return 0U;
  }

  /* W = V * diag(sqrt(eigenvalues)) * V^T - the symmetric matrix square root
   * of mnorm, so that W*(raw-center) has magnitude ~1 for a point on the
   * fitted ellipsoid. */
  for (r = 0U; r < 3U; r++)
  {
    for (col = 0U; col < 3U; col++)
    {
      float acc = 0.0f;
      for (k = 0U; k < 3U; k++)
      {
        acc += eigvecs[r][k] * sqrtf(eigvals[k][k]) * eigvecs[col][k];
      }
      w[r][col] = acc;
    }
  }

  g_center_x = center[0];
  g_center_y = center[1];
  g_center_z = center[2];
  g_w_xx = w[0][0];
  g_w_yy = w[1][1];
  g_w_zz = w[2][2];
  g_w_xy = w[0][1];
  g_w_xz = w[0][2];
  g_w_yz = w[1][2];
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

float Mag_GetCalCenterX(void)
{
  return g_center_x;
}

float Mag_GetCalCenterY(void)
{
  return g_center_y;
}

float Mag_GetCalCenterZ(void)
{
  return g_center_z;
}

float Mag_GetCalMatrixXX(void)
{
  return g_w_xx;
}

float Mag_GetCalMatrixYY(void)
{
  return g_w_yy;
}

float Mag_GetCalMatrixZZ(void)
{
  return g_w_zz;
}

float Mag_GetCalMatrixXY(void)
{
  return g_w_xy;
}

float Mag_GetCalMatrixXZ(void)
{
  return g_w_xz;
}

float Mag_GetCalMatrixYZ(void)
{
  return g_w_yz;
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

/* Z-axis sign convention for the tilt-compensation terms below - independent of
 * (and unaffected by) the Z hard/soft-iron calibration added 2026-08-21. Flip
 * to -1.0f if bench validation (see below) shows heading swinging the wrong
 * way specifically when tilting rather than yawing. Not yet re-verified for
 * the flat remount below - re-check with a roll/pitch tilt test once that
 * settles. */
#define MAG_Z_SIGN 1.0f

/* Fixed mounting-rotation correction for the HGLRC M100 Pro. This is a
 * ONE-TIME, CONSTANT correction (how the chip sits on the board) - completely
 * separate from the per-sample, dynamic tilt-compensation (using the
 * aircraft's CURRENT roll/pitch) done further down in Mag_GetHeadingDeg().
 * Both stages are necessary and compose: this stage first converts raw sensor
 * axes to body FRD axes (as if the chip were mounted perfectly flat/aligned),
 * then the existing tilt-compensation formula uses the aircraft's live
 * attitude on top of that.
 *
 * SIMPLIFIED 2026-08-21 (fourth revision, same day): the module was
 * physically REMOUNTED FLAT (no more ~20deg nose-up tilt) specifically to
 * eliminate the whole class of problems chased earlier today - a single
 * rotation-angle "pitch-mix" model for an arbitrary tilted mount is
 * inherently a poor fit (confirmed twice via real bench data: ~58deg heading
 * swing over a ~25deg roll with the old 20deg-mix form, and a full 3x3
 * data-fitted replacement that fixed tilt-sensitivity but reversed yaw
 * direction - see git history / [[kh7_compass_calibration_state]] memory).
 * With a genuinely flat mount, no pitch-mix is needed at all - just the
 * already-validated Y-axis flip. mx1=cal_x, my1=-cal_y was validated against
 * a real full 360deg CW bench rotation (net change +352deg, start/end near
 * the reported "about north" point) - see git history for the original
 * 8-combination sweep that found it. That axis mapping should still hold
 * after the remount (same chip, same PCB axes, only the TILT ANGLE changed,
 * not which raw axis is which) - re-verify with a fresh rotation test rather
 * than assuming, since the calibration and physical setup both changed.
 *   mx1 = cal_x,  my1 = -cal_y,  mz1 = cal_z
 *   mx = mx1
 *   my = my1
 *   mz = MAG_Z_SIGN * mz1
 * REQUIRES A FRESH CALIBRATION TUMBLE (old ellipsoid fit was for the raw
 * sensor's OLD physical orientation, now invalid) before this will produce
 * sensible headings - the ~70-74deg-regardless-of-orientation symptom seen
 * right after the remount (old calibration + old code, before this edit) is
 * exactly what a stale, orientation-mismatched calibration looks like. */

/* Empirical zero-point correction, applied after the mounting-rotation and
 * tilt-compensation math below - see the comment at its use site. Re-derived
 * AGAIN 2026-08-21: a fresh check at true north with the prior -11.8 offset
 * already applied read ~349.0deg instead of 0deg. CORRECTED FORMULA (a
 * -12.55 value was flashed briefly and was WRONG - it incorrectly added the
 * new correction on top of the old offset instead of replacing it, since the
 * underlying raw_atan2 value is independent of whatever offset is currently
 * applied): raw_atan2 = measured - old_offset = 348.95 - (-11.8) = 0.75deg,
 * so new_offset = -raw_atan2 = -0.75deg (a REPLACEMENT of the old value, not
 * an addition to it). The ~11deg discrepancy between this and the original
 * -11.8 derivation is consistent with (not distinct from) the same
 * ~10-40deg residual calibration/mounting noise floor found across the
 * 8-point 45deg-increment sweep the same day - i.e. this offset is only as
 * precise as the system's overall current accuracy, not a separately-broken
 * constant. Don't expect this to be meaningfully more "correct" than the
 * prior value until the underlying accuracy improves. */
#define MAG_HEADING_NORTH_OFFSET_DEG -0.75f

/* Motor-current-induced field disturbance, as a VECTOR in the chip's own raw
 * X/Y/Z axes (Gauss per microsecond of motor PWM above idle), NOT a constant
 * degrees-per-us heading offset. A 2026-08-21 first pass tried the simpler
 * "subtract a fixed number of degrees from the final heading" approach - it
 * matched one bench test (idle ~305deg -> peak ~328deg, fit slope ~0.05-0.07)
 * but then badly OVERSHOT and even reversed sign on a follow-up test started
 * at a different aircraft orientation (~25deg instead of ~305deg). Root cause:
 * the disturbance is a roughly-fixed vector in body/chip frame (nearby
 * current-carrying wires don't move relative to the sensor), and a fixed
 * vector's effect on the COMPUTED HEADING angle depends on how it combines
 * with Earth's field - which depends on the aircraft's actual orientation.
 * There is no constant-degrees correction that works at every heading. Fixed
 * by characterizing and compensating in the RAW CARTESIAN axes instead (at a
 * FIXED bench orientation, throttle swept idle-to-above-hover and back,
 * confirmed reversible): slopes below are least-squares fits of raw X/Y/Z (in
 * Gauss, before hard/soft-iron cal) vs motor PWM above idle. Applying the
 * correction here, before atan2, is orientation-independent by construction.
 *
 * RE-DERIVED 2026-08-21 after the flat remount (n=195 points, armed samples
 * only) - the old slopes (0.000457/0.000308/-0.000117) were fit under the
 * previous tilted mount and are now stale: a fresh restrained ground run
 * showed a real ~58deg heading swing from idle to ~1440us motor PWM with
 * those old constants still applied. New fit: X and Z both show a real
 * effect, Y is negligible (~0). */
#define MAG_MOTOR_COMP_X_G_PER_US 0.00006113f
#define MAG_MOTOR_COMP_Y_G_PER_US 0.0f
#define MAG_MOTOR_COMP_Z_G_PER_US 0.00014256f

float Mag_GetHeadingDeg(float roll_deg, float pitch_deg, float motor_power_delta_us)
{
  float heading_deg;
  float cal_x;
  float cal_y;
  float cal_z;
  float mx1;
  float my1;
  float mz1;
  float mx;
  float my;
  float mz;
  float roll_rad;
  float pitch_rad;
  float sin_roll;
  float cos_roll;
  float sin_pitch;
  float cos_pitch;
  float xh;
  float yh;
  float full3_mag;
  float raw_x_g;
  float raw_y_g;
  float raw_z_g;

  /* Subtract the motor-current disturbance vector (see comment above) from
   * the RAW field, before hard/soft-iron calibration - this is where it
   * physically enters the measurement, and correcting here (rather than as a
   * post-hoc degree offset on the final heading) is orientation-independent. */
  raw_x_g = g_x_g - (MAG_MOTOR_COMP_X_G_PER_US * motor_power_delta_us);
  raw_y_g = g_y_g - (MAG_MOTOR_COMP_Y_G_PER_US * motor_power_delta_us);
  raw_z_g = g_z_g - (MAG_MOTOR_COMP_Z_G_PER_US * motor_power_delta_us);

  /* Full 3x3 hard/soft-iron correction (ellipsoid fit, see Mag_CalStop()):
   * cal = W * (raw - center), replacing the old independent-per-axis
   * offset/scale (which couldn't correct cross-axis/soft-iron coupling - see
   * the version-4 flash comment above for why that mattered in practice). */
  {
    float ux = raw_x_g - g_center_x;
    float uy = raw_y_g - g_center_y;
    float uz = raw_z_g - g_center_z;

    cal_x = (g_w_xx * ux) + (g_w_xy * uy) + (g_w_xz * uz);
    cal_y = (g_w_xy * ux) + (g_w_yy * uy) + (g_w_yz * uz);
    cal_z = (g_w_xz * ux) + (g_w_yz * uy) + (g_w_zz * uz);
  }

  /* Mounting correction - see the block comment above (simplified 2026-08-21
   * to just the Y-flip after the module was remounted flat - no more pitch-mix
   * needed). */
  mx1 = cal_x;
  my1 = -cal_y;
  mz1 = cal_z;
  mx = mx1;
  my = my1;
  mz = MAG_Z_SIGN * mz1;

  /* Standard tilt-compensated heading (e.g. NXP AN4248), in body FRD axes
   * (mx=forward, my=right, mz=down), roll/pitch in the standard aviation
   * sign convention this project already uses elsewhere (+roll=right down,
   * +pitch=nose up - see Attitude_GetBoardAnglesDeg()). mx/my/mz above are
   * already mounting-corrected into body FRD axes, so this is exactly the
   * textbook formula with no further axis substitution needed:
   *   Xh = mx*cos(pitch) + my*sin(roll)*sin(pitch) + mz*cos(roll)*sin(pitch)
   *   Yh = my*cos(roll) - mz*sin(roll)
   *   heading = atan2(Yh, Xh)
   *
   * mz's SIGN (MAG_Z_SIGN, independent of the mounting-rotation math above)
   * is the one remaining unverified assumption - it only scales the
   * sin(roll)/sin(pitch) terms, so it's zero-impact at zero tilt and
   * small-impact at small tilt. Currently 1.0f (reverted 2026-08-21 along
   * with the mounting-rotation revert above - see that comment for why). */
  roll_rad = roll_deg * (3.14159265f / 180.0f);
  pitch_rad = pitch_deg * (3.14159265f / 180.0f);
  sin_roll = sinf(roll_rad);
  cos_roll = cosf(roll_rad);
  sin_pitch = sinf(pitch_rad);
  cos_pitch = cosf(pitch_rad);

  xh = (mx * cos_pitch) + (my * sin_roll * sin_pitch) + (mz * cos_roll * sin_pitch);
  yh = (my * cos_roll) - (mz * sin_roll);

  /* Guard against the low-magnitude instability described above - compare
   * against the FULL 3D field magnitude (all three calibrated axes), which is
   * the quantity that stays roughly constant under rotation for a properly
   * calibrated sensor, regardless of orientation. The tilt-compensated 2-axis
   * (xh,yh) is NOT that quantity - it's naturally smaller by roughly
   * cos(magnetic dip angle), which is well under 50% of the full radius at
   * most latitudes even with a perfect calibration and perfectly healthy
   * sensor. An earlier version of this gate compared against (xh,yh)'s own
   * magnitude instead of the full 3D one, which froze the heading almost
   * permanently during ordinary level yaw rotation (matches real bench data:
   * heading stuck bit-for-bit constant across large swings in the raw axes) -
   * not a hardware fault at all, just the wrong quantity being gated on. */
  if ((g_cal_loaded != 0U) && (g_last_heading_valid != 0U))
  {
    full3_mag = sqrtf((cal_x * cal_x) + (cal_y * cal_y) + (cal_z * cal_z));
    if (full3_mag < (MAG_HEADING_MIN_MAG_RATIO * MAG_CAL_EXPECTED_MAGNITUDE))
    {
      return g_last_heading_deg;
    }
  }

  heading_deg = atan2f(yh, xh) * (180.0f / 3.14159265f);
  if (heading_deg < 0.0f)
  {
    heading_deg += 360.0f;
  }

  /* Empirical constant offset so heading=0/360 means "facing magnetic north" -
   * bench-verified 2026-08-21: with the aircraft oriented exactly at magnetic
   * north, raw output read 315deg, so +45deg lines it up. This is separate
   * from (and downstream of) the mounting-rotation correction above, which
   * only fixed the rotation DIRECTION/rate, not the absolute zero-point -
   * re-derive this offset (repeat the exact-north bench check) if the module
   * is ever re-seated, since the mounting fix's empirical axis mapping could
   * shift it again. */
  heading_deg += MAG_HEADING_NORTH_OFFSET_DEG;

  while (heading_deg >= 360.0f)
  {
    heading_deg -= 360.0f;
  }
  while (heading_deg < 0.0f)
  {
    heading_deg += 360.0f;
  }

  g_last_heading_deg = heading_deg;
  g_last_heading_valid = 1U;
  return heading_deg;
}
