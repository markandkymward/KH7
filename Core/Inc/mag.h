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
/* Tilt-compensated heading (clockwise-positive from north, matching yaw_deg's
 * convention - see nav.h), corrected with the hard/soft-iron calibration from
 * Mag_CalStart()/Mag_CalStop() (or flash defaults, or identity if never
 * calibrated). roll_deg/pitch_deg must be the board's TRUE physical tilt
 * relative to level (Attitude_GetBoardAnglesDeg()'s raw output, standard
 * aviation sign convention: +roll = right side down, +pitch = nose up) - NOT
 * an attitude-zero-offset-corrected value, since gravity-referenced tilt
 * compensation needs the real tilt regardless of any chosen control-loop
 * reference attitude. Reduces to the old flat atan2(X,Y) formula exactly at
 * roll=pitch=0. See mag.c for the mounting-orientation derivation and the one
 * remaining unverified assumption (magnetometer Z-axis sign - Z was never
 * referenced before this, only X/Y). */
float Mag_GetHeadingDeg(float roll_deg, float pitch_deg);

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
