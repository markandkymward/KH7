#include "imu.h"

#include <string.h>

#define IMU_READ_BIT              0x80U
#define IMU_SPI_TIMEOUT_MS        100U

#define IMU_WHOAMI_ICM42688       0x47U

#define IMU_ICM_PWR_MGMT0         0x4EU
#define IMU_ICM_GYRO_CFG0         0x4FU
#define IMU_ICM_ACCEL_CFG0        0x50U
#define IMU_ICM_BURST_START       0x1FU
#define IMU_ICM_WHOAMI_REG        0x75U
#define IMU_ICM_BANK_SEL_REG      0x76U
#define IMU_ICM_DEVICE_CONFIG_REG 0x11U

extern SPI_HandleTypeDef hspi4;

static IMU_TypeDef g_imu_type = IMU_TYPE_UNKNOWN;
static uint8_t g_imu_whoami = 0U;

static HAL_StatusTypeDef IMU_WriteReg(uint8_t reg, uint8_t value)
{
  uint8_t frame[2] = {reg, value};
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&hspi4, frame, 2U, IMU_SPI_TIMEOUT_MS) != HAL_OK)
  {
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    return HAL_ERROR;
  }
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  return HAL_OK;
}

static HAL_StatusTypeDef IMU_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len)
{
  uint8_t tx[32];
  uint8_t rx[32];

  if ((len == 0U) || ((len + 1U) > sizeof(tx)))
  {
    return HAL_ERROR;
  }

  memset(tx, 0, sizeof(tx));
  memset(rx, 0, sizeof(rx));
  tx[0] = reg | IMU_READ_BIT;

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_TransmitReceive(&hspi4, tx, rx, (uint16_t)(len + 1U), IMU_SPI_TIMEOUT_MS) != HAL_OK)
  {
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    return HAL_ERROR;
  }
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  memcpy(data, &rx[1], len);
  return HAL_OK;
}

static void IMU_ApplyOrientationCW270(IMU_RawData_t *raw)
{
  int16_t old_ax;
  int16_t old_ay;
  int16_t old_az;
  int16_t old_gx;
  int16_t old_gy;
  int16_t old_gz;

  old_ax = raw->accel_x;
  old_ay = raw->accel_y;
  old_az = raw->accel_z;
  old_gx = raw->gyro_x;
  old_gy = raw->gyro_y;
  old_gz = raw->gyro_z;

  raw->accel_x = (int16_t)(-old_ay);
  raw->accel_y = (int16_t)(-old_ax);
  raw->accel_z = old_az;
  raw->gyro_x = (int16_t)(-old_gy);
  raw->gyro_y = (int16_t)(-old_gx);
  raw->gyro_z = old_gz;
}

HAL_StatusTypeDef IMU_DetectAndInit(void)
{
  uint8_t whoami = 0U;

  (void)IMU_WriteReg(IMU_ICM_BANK_SEL_REG, 0x00U);
  if (IMU_ReadRegs(IMU_ICM_WHOAMI_REG, &whoami, 1U) == HAL_OK)
  {
    if (whoami == IMU_WHOAMI_ICM42688)
    {
      g_imu_type = IMU_TYPE_ICM42688;
      g_imu_whoami = whoami;

      (void)IMU_WriteReg(IMU_ICM_DEVICE_CONFIG_REG, 0x01U);
      HAL_Delay(2);
      (void)IMU_WriteReg(IMU_ICM_BANK_SEL_REG, 0x00U);
      (void)IMU_WriteReg(IMU_ICM_PWR_MGMT0, 0x0FU);
      HAL_Delay(10);
      (void)IMU_WriteReg(IMU_ICM_GYRO_CFG0, 0x06U);
      (void)IMU_WriteReg(IMU_ICM_ACCEL_CFG0, 0x06U);
      return HAL_OK;
    }
  }

  g_imu_type = IMU_TYPE_UNKNOWN;
  g_imu_whoami = whoami;

  return HAL_ERROR;
}

HAL_StatusTypeDef IMU_ReadRawAligned(IMU_RawData_t *raw)
{
  uint8_t burst[14];

  if (raw == NULL)
  {
    return HAL_ERROR;
  }

  if (g_imu_type == IMU_TYPE_ICM42688)
  {
    (void)IMU_WriteReg(IMU_ICM_BANK_SEL_REG, 0x00U);
    if (IMU_ReadRegs(IMU_ICM_BURST_START, burst, 12U) != HAL_OK)
    {
      return HAL_ERROR;
    }

    raw->accel_x = (int16_t)((burst[0] << 8) | burst[1]);
    raw->accel_y = (int16_t)((burst[2] << 8) | burst[3]);
    raw->accel_z = (int16_t)((burst[4] << 8) | burst[5]);
    raw->gyro_x = (int16_t)((burst[6] << 8) | burst[7]);
    raw->gyro_y = (int16_t)((burst[8] << 8) | burst[9]);
    raw->gyro_z = (int16_t)((burst[10] << 8) | burst[11]);

    IMU_ApplyOrientationCW270(raw);
    return HAL_OK;
  }

  return HAL_ERROR;
}

IMU_TypeDef IMU_GetType(void)
{
  return g_imu_type;
}

uint8_t IMU_GetWhoAmI(void)
{
  return g_imu_whoami;
}
