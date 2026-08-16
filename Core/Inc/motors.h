#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Motors_Init(void);
void Motors_SetOutputEnabled(uint8_t enabled);
void Motors_WriteUs(uint16_t s1_us,
                    uint16_t s2_us,
                    uint16_t s3_us,
                    uint16_t s4_us);
uint8_t Motors_RunTestPattern(uint32_t now_ms);
void Motors_StopAll(void);
void Motors_ForceIdleRegistersOnly(void);

#ifdef __cplusplus
}
#endif

#endif
