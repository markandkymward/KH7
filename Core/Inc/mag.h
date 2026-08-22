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
 * referenced before this, only X/Y).
 *
 * motor_power_delta_us: average commanded motor PWM minus idle (e.g.
 * s1..s4_us averaged, minus APP_MOTOR_IDLE_US) - compensates a real,
 * bench-characterized motor-current-induced heading bias (see mag.c). Pass
 * 0.0f if unknown/not applicable (disarmed, no motor context). */
float Mag_GetHeadingDeg(float roll_deg, float pitch_deg, float motor_power_delta_us);

/* Full 3D (X/Y/Z) hard/soft-iron calibration: call Mag_CalStart(), then
 * physically tumble the vehicle through a variety of orientations for several
 * seconds - NOT just a flat 360-degree yaw spin, it needs real roll/pitch
 * tilting too so all three axes sweep their full range (Mag_CalStop() rejects
 * the capture if any single axis didn't move enough - see
 * MAG_CAL_MIN_RANGE_GAUSS in mag.c). Then call Mag_CalStop() - it fits a
 * general ellipsoid (least squares over every sample collected, not just
 * per-axis min/max) to get a center point and a full 3x3 soft-iron correction
 * matrix, corrects for cross-axis coupling that a simple independent
 * per-axis offset/scale cannot (see the version-4 flash comment in mag.c -
 * bench data showed real heading errors up to ~90deg without this).
 * Mag_CalStop() does a blocking flash erase (~1-2s) - caller must not invoke
 * it while armed. */
void Mag_CalStart(void);
uint8_t Mag_CalStop(void);
uint8_t Mag_IsCalActive(void);
uint8_t Mag_IsCalibrated(void);
float Mag_GetCalCenterX(void);
float Mag_GetCalCenterY(void);
float Mag_GetCalCenterZ(void);
float Mag_GetCalMatrixXX(void);
float Mag_GetCalMatrixYY(void);
float Mag_GetCalMatrixZZ(void);
float Mag_GetCalMatrixXY(void);
float Mag_GetCalMatrixXZ(void);
float Mag_GetCalMatrixYZ(void);

#ifdef __cplusplus
}
#endif

#endif
