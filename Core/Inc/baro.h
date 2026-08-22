#ifndef BARO_H
#define BARO_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef Baro_Init(void);
/* motor_power_delta_us: average commanded motor PWM minus idle (same
 * quantity fed to Mag_GetHeadingDeg() - see app.c's g_avg_motor_power_delta_us)
 * - compensates a real, bench-characterized propwash-induced altitude bias
 * (see baro.c). Pass 0.0f if unknown/not applicable.
 * vertical_accel_mps2: net (gravity-removed) earth-frame vertical
 * acceleration - see Attitude_GetVerticalAccelMps2() - feeds the climb-rate
 * complementary filter (see baro.c). Pass 0.0f if unavailable (climb rate
 * will fall back to pure baro-derivative behavior in that case). */
HAL_StatusTypeDef Baro_Update(float motor_power_delta_us, float vertical_accel_mps2);
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
