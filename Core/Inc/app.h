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
void App_RequestSdLogErase(void);
void App_RequestArmedTelemetryEnabled(uint8_t enabled);
void App_PrintArmedTelemetryStatus(void);

#ifdef __cplusplus
}
#endif

#endif
