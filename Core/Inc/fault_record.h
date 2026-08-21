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

/* Every boot's RCC->RSR reset-cause flags (IWDG/WWDG/BOR/PIN/POR/SOFT), stashed
 * in RAM_D3 unconditionally - not just on a crash - so "RESET STATUS" can
 * answer "why did the last boot happen" on demand, even when nobody had a
 * terminal open live and main.c's one-time boot printf was never seen. Added
 * 2026-08-21 chasing repeated in-flight hangs that leave the SD log stopped
 * mid-armed with no disarm ever recorded: a hard brown-out reset (BOR) from a
 * battery-sag-under-load event would produce exactly that signature and is a
 * genuine STM32-silicon-level failure mode, entirely independent of IWDG -
 * see watchdog_disabled.md. Placed well clear of FAULT_RECORD_ADDR (used up to
 * ~40 bytes) and below APP_BLACKBOX_RING_ADDR (0x38000200) in app.c. */
#define RESET_INFO_ADDR  0x38000080UL
#define RESET_INFO_MAGIC 0x524B5354u /* 'RKST' */

typedef struct __attribute__((packed))
{
  uint32_t magic;
  uint32_t reset_cause_flags; /* RCC->RSR from the boot that just happened */
  uint32_t boot_count;        /* increments every boot since RAM_D3 last lost power */
} ResetInfo_t;

#define RESET_INFO ((volatile ResetInfo_t *)RESET_INFO_ADDR)

#endif /* FAULT_RECORD_H */
