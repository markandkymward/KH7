#ifndef VERT_EKF_H
#define VERT_EKF_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void VertEkf_Init(void);
/* Same reset-on-arm discipline as every other ALTHOLD-adjacent estimator in
 * this codebase (see kh7-althold-throttle-incident memory for why a stale
 * cross-flight value is a real hazard, not just a cosmetic issue) - call this
 * from the same arm-transition block that resets App_ResetFusedAltitude(). */
void VertEkf_Reset(void);

/* Predict step - call at IMU/control-loop rate, right alongside
 * Attitude_UpdateIMU() for the same sample (both IMU-processing branches in
 * app.c need this call, same as they both already call Attitude_UpdateIMU()).
 * accel_up_mps2: Attitude_GetVerticalAccelMps2() output for this same sample -
 * net, gravity-removed, earth-frame vertical acceleration, positive = up. */
void VertEkf_Predict(float accel_up_mps2, float dt_s);

/* Baro measurement update - call once per fresh baro sample (alongside
 * Baro_Update(), same ~50Hz rate). raw_alt_m MUST be Baro_GetRawAltitudeM(),
 * NOT Baro_GetAltitudeM() - see Baro_GetRawAltitudeM()'s comment for why
 * (feeding an already-LPF'd signal into a second filter double-filters and
 * breaks the white-noise-residual assumption the R term relies on).
 * motor_power_delta_us: the same ground-effect/thrust-demand proxy already fed
 * to Baro_Update() (g_avg_motor_power_delta_us in app.c) - used here to
 * inflate baro's measurement noise near the ground under thrust, on top of
 * (not instead of) the bias correction Baro_Update() already applies. */
void VertEkf_UpdateBaro(float raw_alt_m, uint8_t baro_healthy, float motor_power_delta_us);

/* Range sensor measurement update - call directly from the point a fresh
 * reading arrives (App_SetRangefinderCm()/App_SetLunaCm() in app.c), NOT
 * polled - this is what makes it a true asynchronous, native-rate update per
 * sensor rather than a synchronized one.
 * raw_range_cm: the RAW slant range from the ESP32's SENSOR packet.
 * confidence: 0.0-1.0 from the same packet.
 * is_lidar: 1 for TF-Luna, 0 for the HC-SR04 sonar - selects which sensor's
 * range ceiling, R-scheduling, and mounting offset apply (see
 * VERT_EKF_LIDAR_ARM_X_M etc. in vert_ekf.c).
 * valid: the SENSOR packet's own valid flag, passed through unmodified - a
 * no-op (not an update) when 0. NOT re-derived from raw_range_cm<=0.0f (a real
 * 2026-08-29 bug: a genuine 0cm reading - a downward point sensor mounted low
 * enough to sit right at ground level, confirmed via a real liftoff capture
 * showing strong signal strength at exactly raw_cm=0 - is indistinguishable
 * from "no reading" under that convention, so real ground-level readings were
 * silently dropped for several seconds of every flight, including all of
 * liftoff). Trust this flag instead. */
void VertEkf_UpdateRange(float raw_range_cm, float confidence, uint8_t is_lidar, uint8_t valid);

/* GPS long-time-constant baro-bias trim, PLUS a coarse divergence/failsafe
 * cross-check - deliberately never a direct EKF height measurement (GPS
 * vertical accuracy is too coarse for that, and its altitude reference - MSL -
 * doesn't match baro/range's "height above arm point" convention anyway). Call
 * whenever a fresh GPS fix is available; armed gates whether the divergence
 * watchdog can trip (avoids flagging a fault before there's a real fused
 * estimate to compare against, e.g. while still on the ground pre-arm). */
void VertEkf_UpdateGps(float gps_alt_m, float gps_vacc_m, uint8_t gps_healthy, uint8_t armed);

float VertEkf_GetHeightM(void);
float VertEkf_GetClimbRateMps(void);
float VertEkf_GetAccelBiasMps2(void);
/* Height below which this file's own baro-noise model considers ground effect
 * meaningfully active (2x rotor diameter, see VERT_EKF_GROUND_EFFECT_ZONE_M's
 * comment in vert_ekf.c) - exposed so app.c's ALTHOLD authority gate
 * (APP_BARO_VZ_DAMP_MIN_ALT_M) can share this SAME physically-reasoned value
 * instead of keeping its own independent, potentially-drifting copy. */
float VertEkf_GetGroundEffectZoneM(void);
/* Most recent tilt+lever-arm-compensated height implied by each range sensor's last
 * reading (before it went into the Kalman update) - the exact value
 * VertEkf_UpdateRange() computed and cross-checked, not re-derived. For live bench
 * validation via telemetry (see Telemetry_PrintVertEkfState()) - watch these move the
 * expected way while manually tilting the aircraft. Returns the last-known value
 * regardless of staleness (no separate "unavailable" sentinel) - this is a live
 * bench-check aid, not a control input, so the caller is expected to be watching it
 * update in real time. */
float VertEkf_GetLidarImpliedHeightM(void);
float VertEkf_GetSonarImpliedHeightM(void);
/* False only if the GPS divergence watchdog has tripped (fused height
 * disagreeing with GPS-implied height for too long, while armed) - a coarse
 * "something may be seriously wrong" flag, not a normal staleness indicator.
 * The estimate itself degrades gracefully on its own whenever baro/range
 * measurements go stale (falls back to pure IMU-predicted height, same idiom
 * as App_GetFusedAltitudeM()) - that doesn't affect this flag. */
uint8_t VertEkf_IsHealthy(void);

#ifdef __cplusplus
}
#endif

#endif
