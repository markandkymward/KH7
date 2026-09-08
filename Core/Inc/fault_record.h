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

/* Second copy of the fault record, written to genuine internal FLASH instead of
 * RAM_D3 (2026-09-06). RAM_D3 "survives any reset that isn't a full power cycle"
 * (see comment above) - but the one incident this exists for (a total in-flight
 * hang with no watchdog to recover it) can ONLY be stopped by a full battery
 * disconnect, which erases RAM_D3 right when its contents matter most. Flash
 * survives that. Purely additive: written best-effort by stm32h7xx_it.c's
 * Fault_ReportAndHalt() AFTER the existing RAM_D3 write/UART report, and does
 * not change the existing while(1) hang or the RAM_D3 path in any way.
 * Bank 2 Sector 3 (0x08160000) - one sector below the lowest sector any other
 * flash-persisted blob in this codebase currently uses (Sector 4 = ALTHOLD
 * tunables, per app.c's own "pick the sector immediately below the lowest
 * used one, and grep to confirm it's free" discipline - see app.c ~line 970
 * for why that discipline exists, after a prior sector collision silently
 * corrupted saved data). Confirmed free by grepping every FLASH_SECTOR_/
 * FLASH_BANK_ use under Core/Src before picking it. */
#define FAULT_FLASH_ADDRESS  0x08160000UL
#define FAULT_FLASH_MAGIC    0x4655464CUL /* "FUFL" */
#define FAULT_FLASH_VERSION  1UL

typedef struct __attribute__((packed))
{
  uint32_t magic;
  uint32_t version;
  char name[12];
  uint32_t pc;
  uint32_t lr;
  uint32_t cfsr;
  uint32_t hfsr;
  uint32_t mmfar;
  uint32_t bfar;
  uint32_t crc32;
} FaultFlashBlob_t;

/* Loop breadcrumb (2026-09-06): "what was App_Update() last doing" - added after
 * a night of real in-flight hangs left NO fault record at all (neither this
 * header's RAM_D3 FaultRecord_t nor the flash-backed FaultFlashBlob_t above
 * ever populated across several confirmed hang incidents and their following
 * power cycles) - strong evidence these are plain main-loop livelocks/blocking
 * calls, not CPU exceptions, so nothing ever reaches Fault_ReportAndHalt() at
 * all. This is a much coarser instrument than a real fault record - just "which
 * phase of the loop were we in the last time we checked in" - but it works for
 * ANY kind of stall, not just ones that trip a hardware fault.
 *
 * IMPORTANT CAVEAT: like FaultRecord_t/ResetInfo_t above, this lives in RAM_D3
 * and only survives a reset that keeps power applied (IWDG/software reset) -
 * it does NOT by itself survive the full battery pull that's currently the
 * ONLY way to recover from a hang with the IWDG disabled (see
 * watchdog_disabled.md). Written every breadcrumb call in app.c; read + reported
 * once at boot in App_Init(). Placed in the gap between RESET_INFO_ADDR (used up
 * to ~12 bytes at 0x38000080) and APP_BLACKBOX_RING_ADDR (0x38000200 in app.c). */
#define LOOP_BREADCRUMB_ADDR  0x38000100UL
#define LOOP_BREADCRUMB_MAGIC 0x42524344UL /* "BRCD" */

typedef enum
{
  LOOP_STAGE_UNKNOWN = 0,
  LOOP_STAGE_START,
  LOOP_STAGE_AFTER_RC_READ,
  LOOP_STAGE_AFTER_IMU_READ,
  LOOP_STAGE_BEFORE_SDLOG_WRITE,
  LOOP_STAGE_AFTER_SDLOG_WRITE,
  LOOP_STAGE_AFTER_MOTOR_WRITE,
  LOOP_STAGE_END,
  LOOP_STAGE_TEST_HANG /* deliberate, commanded via "TEST HANG" USB command - see
                        * App_RequestTestHang()'s comment in app.c. Distinguishes a
                        * bench-test hang from a real one in the boot report. */
} LoopStage_t;

typedef struct __attribute__((packed))
{
  uint32_t magic;
  uint32_t stage;          /* a LoopStage_t value */
  uint32_t last_update_ms; /* HAL_GetTick() at the last stamp before whatever happened */
  uint32_t iteration;      /* free-running count of stamps since this magic was last (re)initialized -
                             * i.e. since the last full power-on, surviving any reset in between */
} LoopBreadcrumb_t;

#define LOOP_BREADCRUMB ((volatile LoopBreadcrumb_t *)LOOP_BREADCRUMB_ADDR)

#endif /* FAULT_RECORD_H */
