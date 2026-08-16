#ifndef FAULT_RECORD_H
#define FAULT_RECORD_H

#include <stdint.h>

/* Small crash-record header placed at a fixed address in RAM_D3 (0x38000000,
 * 64KB, D3/SRD domain SRAM). Nothing else in this project uses RAM_D3 (only
 * DTCMRAM holds .data/.bss per STM32H743XX_FLASH.ld), and its contents survive
 * any reset that isn't a full power cycle (IWDG reset, software reset) - so a
 * fault handler can stash diagnostics here and the NEXT boot can report/persist
 * them even if nothing was listening on UART6 at the exact moment of the crash.
 * Written by stm32h7xx_it.c's fault handlers, read + cleared by app.c at boot. */
#define FAULT_RECORD_ADDR  0x38000000UL
#define FAULT_RECORD_MAGIC 0x4B4E4655u /* 'KNFU' */

typedef struct __attribute__((packed))
{
  uint32_t magic;
  char name[12];
  uint32_t pc;
  uint32_t lr;
  uint32_t cfsr;
  uint32_t hfsr;
  uint32_t mmfar;
  uint32_t bfar;
} FaultRecord_t;

#define FAULT_RECORD ((volatile FaultRecord_t *)FAULT_RECORD_ADDR)

#endif /* FAULT_RECORD_H */
