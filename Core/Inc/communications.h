#ifndef COMMUNICATIONS_H
#define COMMUNICATIONS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Communications_Init(void);
void Communications_HandleUart6Byte(uint8_t byte);
void Communications_ServiceUart6Commands(void);
void Communications_ServiceEscPassthrough(void);
void Communications_EscPassthroughSetEnabled(uint8_t enabled);
uint8_t Communications_IsEscPassthroughEnabled(void);
void Communications_EscPassthroughFromUsb(const uint8_t *data, uint16_t len);
void Communications_GetUart6Debug(uint32_t *rx_bytes,
								  uint32_t *rx_lines,
								  uint32_t *pending_ready,
								  uint32_t *last_byte,
								  uint32_t *cmd_len,
								  uint32_t *nonprint_bytes);
int __io_putchar(int ch);

#ifdef __cplusplus
}
#endif

#endif
