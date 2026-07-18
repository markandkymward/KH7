#ifndef IMU_H
#define IMU_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_ACCEL_LSB_PER_G 2048.0f
#define IMU_GYRO_LSB_PER_DPS 16.4f

typedef enum
{
  IMU_TYPE_UNKNOWN = 0,
  IMU_TYPE_ICM42688
} IMU_TypeDef;

typedef struct
{
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
} IMU_RawData_t;

HAL_StatusTypeDef IMU_DetectAndInit(void);
HAL_StatusTypeDef IMU_ReadRawAligned(IMU_RawData_t *raw);
IMU_TypeDef IMU_GetType(void);
uint8_t IMU_GetWhoAmI(void);

#ifdef __cplusplus
}
#endif

#endif
