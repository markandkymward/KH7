#ifndef MAG_H
#define MAG_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* QMC5883L compass on the GPS module (I2C1, shared with the onboard SPA06 baro). */
HAL_StatusTypeDef Mag_Init(void);
HAL_StatusTypeDef Mag_Update(void);
uint8_t Mag_IsHealthy(void);
uint8_t Mag_IsInitialized(void);
uint8_t Mag_GetLastChipId(void);
float Mag_GetXGauss(void);
float Mag_GetYGauss(void);
float Mag_GetZGauss(void);
/* Flat-mount heading (atan2 of X/Y), NOT tilt-compensated, but IS corrected
 * with the hard/soft-iron calibration from Mag_CalStart()/Mag_CalStop() (or
 * flash defaults, or identity if never calibrated). */
float Mag_GetHeadingDeg(void);

/* Hard/soft-iron calibration: call Mag_CalStart(), slowly rotate the vehicle
 * a full 360 degrees in yaw (kept level), then call Mag_CalStop() - it computes
 * offset/scale from the recorded min/max and saves to flash. Mag_CalStop()
 * does a blocking flash erase (~1-2s) - caller must not invoke it while armed. */
void Mag_CalStart(void);
uint8_t Mag_CalStop(void);
uint8_t Mag_IsCalActive(void);
uint8_t Mag_IsCalibrated(void);
float Mag_GetCalOffsetX(void);
float Mag_GetCalOffsetY(void);
float Mag_GetCalScaleX(void);
float Mag_GetCalScaleY(void);

#ifdef __cplusplus
}
#endif

#endif
