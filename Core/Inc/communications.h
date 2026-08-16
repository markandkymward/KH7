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
/* Bytes free in the UART6 TX ring buffer right now. At 115200 baud the queue
 * (2048 bytes) drains far slower than a burst of printf() calls can fill it -
 * a producer pushing a lot of data in one go (SD log dump) must check this
 * before deciding whether to produce more this iteration, rather than pushing
 * a fixed amount regardless of backlog and letting most of it get silently
 * dropped by the queue's own overflow handling. */
uint16_t Communications_Uart6TxQueueFreeBytes(void);
int __io_putchar(int ch);

#ifdef __cplusplus
}
#endif

#endif
