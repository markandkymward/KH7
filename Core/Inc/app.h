#ifndef APP_H
#define APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void App_Init(void);
void App_Update(void);
void App_SetUsbMotorTest(uint8_t enabled, uint8_t motor_index, uint16_t pulse_us);

#ifdef __cplusplus
}
#endif

#endif
