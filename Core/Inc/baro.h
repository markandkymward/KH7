#ifndef BARO_H
#define BARO_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef Baro_Init(void);
HAL_StatusTypeDef Baro_Update(void);
uint8_t Baro_IsHealthy(void);
uint8_t Baro_IsInitialized(void);
uint8_t Baro_GetDetectedAddr7(void);
uint8_t Baro_GetLastChipId(void);
float Baro_GetAltitudeM(void);
float Baro_GetClimbRateMps(void);

#ifdef __cplusplus
}
#endif

#endif
