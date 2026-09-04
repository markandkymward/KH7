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
void Telemetry_PrintBaroState(float altitude_m, float climb_rate_mps, uint8_t healthy);
void Telemetry_PrintVertEkfState(uint8_t healthy,
                                 float height_m,
                                 float climb_rate_mps,
                                 float accel_bias_mps2,
                                 float lidar_implied_height_m,
                                 float sonar_implied_height_m);
void Telemetry_PrintAltholdState(uint8_t holding,
                                 uint8_t authority_active,
                                 float target_alt_m,
                                 float fused_alt_m,
                                 float climb_setpoint_mps,
                                 float climb_error_mps,
                                 int32_t trim_us,
                                 int32_t damp_us,
                                 float hover_throttle_us);
void Telemetry_PrintGpsState(uint8_t configured,
							 uint8_t healthy,
							 uint8_t fix_type,
							 uint8_t num_sv,
							 float lat_deg,
							 float lon_deg,
							 float alt_m);
void Telemetry_PrintMagState(uint8_t healthy,
							 float x_g,
							 float y_g,
							 float z_g,
							 float heading_deg);
/* Debug-only: the PRE-attitude-zero-offset roll/pitch actually fed into
 * Mag_GetHeadingDeg() (mag_tilt_roll_deg/mag_tilt_pitch_deg in app.c) - NOT
 * the same as the post-offset pitch/roll in IMU[...]. Added 2026-08-21 for
 * bench mounting-rotation re-derivation work, where reconstructing the exact
 * firmware heading computation from telemetry alone was unreliable without
 * this (the post-offset IMU values don't match what Mag_GetHeadingDeg()
 * actually used whenever there's a nonzero startup tilt offset). */
void Telemetry_PrintMagTiltState(float tilt_roll_deg, float tilt_pitch_deg);
void Telemetry_PrintNavState(uint8_t valid,
							 uint8_t reference_valid,
							 uint8_t invalid_reason,
							 uint8_t fix_type,
							 uint8_t num_sv,
							 float h_acc_m,
							 uint32_t age_ms,
							 uint32_t update_period_ms,
							 uint32_t consecutive_valid,
							 uint32_t consecutive_invalid,
							 uint32_t duplicate_count,
							 uint32_t rejected_count,
							 uint32_t dropout_count);
void Telemetry_PrintNavPosVel(float north_m,
							 float east_m,
							 float raw_vel_n_mps,
							 float raw_vel_e_mps,
							 float filt_vel_n_mps,
							 float filt_vel_e_mps);
/* NAV_POSHOLD (2026-09-04 rewrite - single unified GPS mode, see APP_NAVPOS_*
 * in app.c) - one print covering both the velocity-command chain and the
 * position-hold target/error, replacing the old separate NAVBRK/POSHOLD lines. */
void Telemetry_PrintNavPos(uint8_t requested,
							 uint8_t active,
							 uint8_t tilt_limited,
							 uint8_t accel_limited,
							 float desired_vel_n_mps,
							 float desired_vel_e_mps,
							 float vel_error_n_mps,
							 float vel_error_e_mps,
							 float accel_cmd_n_mps2,
							 float accel_cmd_e_mps2,
							 float accel_cmd_fwd_mps2,
							 float accel_cmd_right_mps2,
							 float target_roll_deg,
							 float target_pitch_deg,
							 float target_north_m,
							 float target_east_m,
							 float err_north_m,
							 float err_east_m);

#ifdef __cplusplus
}
#endif

#endif
