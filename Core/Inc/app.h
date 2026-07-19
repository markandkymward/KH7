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
} App_RatePidAxisGains_t;

typedef struct
{
	App_RatePidAxisGains_t roll;
	App_RatePidAxisGains_t pitch;
	App_RatePidAxisGains_t yaw;
} App_RatePidGains_t;

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

#ifdef __cplusplus
}
#endif

#endif
