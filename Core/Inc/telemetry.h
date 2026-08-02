#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

#include "app.h"
#include "imu.h"
#include "receiver.h"

#ifdef __cplusplus
extern "C" {
#endif

void Telemetry_PrintImuLoggerStart(void);
void Telemetry_PrintImuDetected(IMU_TypeDef type, uint8_t whoami);
void Telemetry_PrintImuDetectionFailed(void);
void Telemetry_PrintImuReadFailed(IMU_TypeDef type, uint8_t whoami);
void Telemetry_PrintImuState(float ax_g,
							 float ay_g,
							 float az_g,
							 float gx_dps,
							 float gy_dps,
							 float gz_dps,
							 float pitch_deg,
							 float roll_deg,
							 float yaw_deg);
void Telemetry_PrintMotorTestStep(uint8_t step_index);
void Telemetry_PrintReceiverState(const receiver_state_t *state);
void Telemetry_PrintReceiverState16(const receiver_state_t *state);
void Telemetry_PrintArmState(uint8_t armed,
							 uint8_t arm_switch_high,
							 uint8_t arm_low_seen,
							 uint16_t throttle_us,
							 uint16_t s1_us,
							 uint16_t s2_us,
							 uint16_t s3_us,
							 uint16_t s4_us);
void Telemetry_PrintRatePid(const App_RatePidGains_t *gains, const char *source);
void Telemetry_PrintFlightMode(const char *mode_name, uint16_t mode_us);
void Telemetry_PrintAngles(float pitch_deg, float roll_deg, float yaw_deg);
void Telemetry_PrintBatteryState(float battery_voltage_v, uint32_t adc_raw);

#ifdef __cplusplus
}
#endif

#endif
