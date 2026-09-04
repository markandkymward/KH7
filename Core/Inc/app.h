#ifndef APP_H
#define APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	float kp;
	float ki;
	float kd;
	float kff;
} App_RatePidAxisGains_t;

typedef struct
{
	App_RatePidAxisGains_t roll;
	App_RatePidAxisGains_t pitch;
	App_RatePidAxisGains_t yaw;
} App_RatePidGains_t;

typedef struct
{
	float roll_kp;
	float pitch_kp;
	float max_angle_deg;
} App_AttitudeGains_t;

void App_Init(void);
void App_Update(void);
void App_SetUsbMotorTest(uint8_t enabled, uint8_t motor_index, uint16_t pulse_us);
void App_GetRatePidGains(App_RatePidGains_t *gains);
uint8_t App_SetRatePidGains(const App_RatePidGains_t *gains);
void App_ResetRatePidDefaults(void);
uint8_t App_LoadRatePidGains(void);
uint8_t App_SaveRatePidGains(void);
uint8_t App_RequestRatePidSetAndSave(const App_RatePidGains_t *gains);
void App_RequestRatePidSave(void);
void App_RequestRatePidLoad(void);
void App_RequestRatePidDefaults(void);
void App_GetAttitudeGains(App_AttitudeGains_t *gains);
uint8_t App_SetAttitudeGains(const App_AttitudeGains_t *gains);
void App_ResetAttitudeDefaults(void);
uint8_t App_RequestAttitudeSetAndSave(const App_AttitudeGains_t *gains);
void App_RequestAttitudeDefaults(void);
void App_RequestAttitudeZero(void);
void App_RequestMagCalStart(void);
void App_RequestMagCalStop(void);
void App_GetPidCommandDebug(uint32_t *queued_count,
							uint32_t *handled_count,
							uint32_t *pending_cmd);
void App_PrintPidDebug(void);
const char *App_GetBootLog(void);
void App_AppendBootLog(const char *str);
void App_RequestGyroLogDump(void);
void App_RequestSdInit(void);
void App_RequestSdStatus(void);
void App_RequestSdReadBlock(uint32_t block);
void App_RequestSdWriteBlock(uint32_t block);
void App_RequestSdLogStatus(void);
void App_RequestSdLogDump(void);
void App_RequestSdLogDumpLast(void);
void App_RequestSdLogDumpFrom(uint32_t block);
void App_RequestGpsScan(void);
void App_RequestGpsFactoryReset(void);
void App_RequestI2c1Scan(void);
void App_RequestSdLogErase(void);
void App_RequestArmedTelemetryEnabled(uint8_t enabled);
void App_PrintArmedTelemetryStatus(void);
void App_SetRangefinderCm(float cm, float confidence, uint32_t sensor_ts_us, uint8_t valid);
void App_SetLunaCm(float cm, float confidence, uint32_t sensor_ts_us, uint8_t valid);

/* Live-tunable ALTHOLD position-hold P-gain - see APP_ALTHOLD_ALT_HOLD_KP_MPS_PER_M's
 * comment in app.c for the real incident (an unarrestable in-flight climb, emergency
 * disarm) that makes MAX a hard safety bound, not just a sanity check. RAM-only,
 * resets to the compiled-in default on every boot. */
#define APP_ALTHOLD_ALT_HOLD_KP_MIN          0.1f
#define APP_ALTHOLD_ALT_HOLD_KP_MAX          1.3f
float App_GetAltholdAltHoldKp(void);
uint8_t App_SetAltholdAltHoldKp(float kp);

/* Live-tunable ceiling on ALTHOLD's climb/descend rate (both manual off-center stick
 * and the position-hold loop's own correction) - see APP_ALTHOLD_MAX_CLIMB_MPS's
 * comment in app.c. RAM-only, resets to the compiled-in default (2.0 m/s) on boot. */
#define APP_ALTHOLD_MAX_CLIMB_MIN            0.5f
#define APP_ALTHOLD_MAX_CLIMB_MAX            4.0f
float App_GetAltholdMaxClimbMps(void);
uint8_t App_SetAltholdMaxClimbMps(float max_climb_mps);

/* Live-tunable integral gain on the OUTER position-hold loop - see
 * APP_ALTHOLD_POS_KI_PER_S2's comment in app.c for why this was added (a real,
 * measured steady-state hold droop). 0.0 fully disables it. RAM-only, resets to the
 * compiled-in default (0.05) on boot. */
#define APP_ALTHOLD_POS_KI_MIN               0.0f
#define APP_ALTHOLD_POS_KI_MAX               0.15f
float App_GetAltholdPosKi(void);
uint8_t App_SetAltholdPosKi(float ki);

/* Live-tunable overrides for the INNER climb-rate loop's P/I gains - see
 * APP_ALTHOLD_VZ_KP_US_PER_MPS's comment in app.c for why these were exposed
 * (real flight data showing trim using <10% of its authority all flight, after
 * this session's other real bugs on this control path were fixed separately).
 * RAM-only, resets to the compiled-in defaults (12.0/4.0) on boot. Bounds cap
 * at this loop's ORIGINAL pre-2026-08-29 values (25/8), not unbounded. */
#define APP_ALTHOLD_VZ_KP_MIN                6.0f
#define APP_ALTHOLD_VZ_KP_MAX                25.0f
#define APP_ALTHOLD_VZ_KI_MIN                2.0f
#define APP_ALTHOLD_VZ_KI_MAX                8.0f
float App_GetAltholdVzKp(void);
uint8_t App_SetAltholdVzKp(float kp);
float App_GetAltholdVzKi(void);
uint8_t App_SetAltholdVzKi(float ki);

/* Live-tunable overrides for the baro/vert_ekf climb-rate damping term - see
 * APP_BARO_VZ_DAMP_GAIN_US_PER_MPS's comment in app.c for why (raising the inner
 * climb-rate loop's P/I gain did not fix a real, repeating ~8-10s-period height
 * wander - a delay-dominated oscillation calls for damping, not more P/I authority).
 * RAM-only, resets to the compiled-in defaults (15.0/60) on boot. */
#define APP_BARO_VZ_DAMP_GAIN_MIN            5.0f
#define APP_BARO_VZ_DAMP_GAIN_MAX            45.0f
#define APP_BARO_VZ_DAMP_LIMIT_MIN           30U
#define APP_BARO_VZ_DAMP_LIMIT_MAX           150U
float App_GetBaroVzDampGain(void);
uint8_t App_SetBaroVzDampGain(float gain);
uint32_t App_GetBaroVzDampLimit(void);
uint8_t App_SetBaroVzDampLimit(float limit_us);

/* Flash persistence for all 7 ALTHOLD live-tunables above (2026-08-30). Unlike the
 * rate-PID save/load, not gated behind a deferred command queue - App_SaveAltholdSettings()
 * checks the armed state itself (returns 0 / refuses while armed, printing
 * "ALTHOLD_SAVE_DBG: refused while armed") since HAL_FLASHEx_Erase() blocks ~1-2s same
 * as the PID save path and must never freeze the aircraft mid-flight. Load is safe to
 * call anytime (read-only, no erase) - called once at boot in App_Init(). */
uint8_t App_LoadAltholdSettings(void);
uint8_t App_SaveAltholdSettings(void);

/* Live-tunable outer loop for NAV_POSHOLD (2026-09-04 rewrite - replaces the old
 * separate NAVBRAKE mode + independent poshold aux-switch overlay with ONE unified
 * GPS mode). Position error (Kp+Ki) -> desired NE velocity -> feeds the shared
 * velocity-error/accel/angle chain (APP_NAV_POSHOLD_VELOCITY_KP,
 * APP_NAV_POSHOLD_MAX_ACCEL_MPS2, APP_NAV_POSHOLD_MAX_TILT_DEG in app.c) - no
 * separate mixer/rate-PID injection point. RAM-only, resets to the compiled-in
 * defaults on every boot, same pattern as every other live-tunable gain in this
 * file. */
#define APP_NAVPOS_KP_MIN                   0.0f
#define APP_NAVPOS_KP_MAX                   0.8f
#define APP_NAVPOS_KI_MIN                   0.0f
#define APP_NAVPOS_KI_MAX                   0.15f
float App_GetNavPosKp(void);
uint8_t App_SetNavPosKp(float kp);
float App_GetNavPosKi(void);
uint8_t App_SetNavPosKi(float ki);

void App_SetRangefinderMountDescriptor(const char *axis, float offset_deg);
void App_SetLunaMountDescriptor(const char *axis, float offset_deg);

#ifdef __cplusplus
}
#endif

#endif
