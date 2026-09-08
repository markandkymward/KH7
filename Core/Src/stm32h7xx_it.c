/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include "fault_record.h"
#include "motors.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void HardFault_HandlerC(uint32_t *stack_frame);
void MemManage_HandlerC(uint32_t *stack_frame);
void BusFault_HandlerC(uint32_t *stack_frame);
void UsageFault_HandlerC(uint32_t *stack_frame);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern UART_HandleTypeDef huart6;

/* Flash-word-aligned (32-byte) page for writing FaultFlashBlob_t (fault_record.h)
 * to FAULT_FLASH_ADDRESS via HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, ...),
 * same union-of-blob-plus-words pattern app.c uses for its own flash blobs
 * (App_LevelTrimFlashPage_t etc). Local to this file - only this file programs
 * flash for the fault record; app.c only ever reads/erases it. */
typedef union
{
  FaultFlashBlob_t blob;
  uint32_t words[16]; /* 64 bytes = 2 flash words, >= sizeof(FaultFlashBlob_t) (48 bytes) */
} FaultFlashPage_t;

_Static_assert(sizeof(FaultFlashBlob_t) <= sizeof(FaultFlashPage_t), "fault flash blob too large");

#if defined(__GNUC__)
#define FAULT_FLASHWORD_ALIGN __attribute__((aligned(32)))
#else
#define FAULT_FLASHWORD_ALIGN
#endif

/* Same CRC32 (poly 0xEDB88320, init/final 0xFFFFFFFF) as app.c's App_Crc32() -
 * duplicated here (rather than shared) because App_Crc32() is static to app.c
 * and this runs in a fault context where pulling in app.c's dependencies isn't
 * worth it for one small function. Must stay bit-for-bit identical to
 * App_Crc32() or App_ReportAndClearFaultRecord()'s CRC check will never pass. */
static uint32_t Fault_Crc32(const uint8_t *data, size_t len)
{
  uint32_t crc = 0xFFFFFFFFUL;
  size_t i;
  uint8_t bit;

  for (i = 0U; i < len; i++)
  {
    crc ^= (uint32_t)data[i];
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 1UL) != 0U)
      {
        crc = (crc >> 1U) ^ 0xEDB88320UL;
      }
      else
      {
        crc >>= 1U;
      }
    }
  }

  return ~crc;
}

/* Best-effort write of the same fault data to internal FLASH (survives a full
 * power cycle, unlike the RAM_D3 record below) so the cause of a fault survives
 * even a battery pull - the only way to stop a hang with no IWDG in this build.
 * "Best effort": we're already committed to hanging forever right after this
 * regardless of outcome, so on any HAL failure this just falls through to the
 * hang exactly as before - no retries, no error reporting (this runs in a fault
 * context where we can't fully trust things). A 1-2s blocking sector erase here
 * is fine: the "don't block the control loop" rule elsewhere doesn't apply once
 * we're already spinning in while(1) forever. */
static void Fault_WriteFlashRecord(const char *name, uint32_t *stack_frame)
{
  FLASH_EraseInitTypeDef erase;
  uint32_t sector_error = 0U;
  uint32_t address;
  FaultFlashPage_t FAULT_FLASHWORD_ALIGN page;
  uint8_t write_index;

  memset(&page, 0xFF, sizeof(page));
  page.blob.magic = FAULT_FLASH_MAGIC;
  page.blob.version = FAULT_FLASH_VERSION;
  memset(page.blob.name, 0, sizeof(page.blob.name));
  strncpy(page.blob.name, name, sizeof(page.blob.name) - 1U);
  page.blob.pc = stack_frame[6];
  page.blob.lr = stack_frame[5];
  page.blob.cfsr = SCB->CFSR;
  page.blob.hfsr = SCB->HFSR;
  page.blob.mmfar = SCB->MMFAR;
  page.blob.bfar = SCB->BFAR;
  page.blob.crc32 = Fault_Crc32((const uint8_t *)&page.blob, offsetof(FaultFlashBlob_t, crc32));

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return;
  }

  memset(&erase, 0, sizeof(erase));
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Banks = FLASH_BANK_2;
  erase.Sector = FLASH_SECTOR_3;
  erase.NbSectors = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  if (HAL_FLASHEx_Erase(&erase, &sector_error) == HAL_OK)
  {
    address = FAULT_FLASH_ADDRESS;
    for (write_index = 0U; write_index < 2U; write_index++)
    {
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                            address,
                            (uint32_t)&page.words[write_index * 8U]) != HAL_OK)
      {
        break;
      }
      address += 32U;
    }
  }

  (void)HAL_FLASH_Lock();
}

/* Reports fault registers over UART6 (bypassing all buffering) and persists them in
 * RAM_D3 (survives the IWDG reset that follows) so the next boot can report/log them
 * even if nothing was listening on UART6 at the exact moment of the crash. */
static void Fault_ReportAndHalt(const char *name, uint32_t *stack_frame)
{
  char buf[160];
  int len;
  volatile FaultRecord_t *rec = FAULT_RECORD;

  rec->magic = FAULT_RECORD_MAGIC;
  memset((void *)rec->name, 0, sizeof(rec->name));
  strncpy((char *)rec->name, name, sizeof(rec->name) - 1U);
  rec->pc = stack_frame[6];
  rec->lr = stack_frame[5];
  rec->cfsr = SCB->CFSR;
  rec->hfsr = SCB->HFSR;
  rec->mmfar = SCB->MMFAR;
  rec->bfar = SCB->BFAR;

  len = snprintf(buf, sizeof(buf),
                 "\r\nFAULT[%s] pc=0x%08lX lr=0x%08lX cfsr=0x%08lX hfsr=0x%08lX mmfar=0x%08lX bfar=0x%08lX\r\n",
                 name,
                 (unsigned long)stack_frame[6],
                 (unsigned long)stack_frame[5],
                 (unsigned long)SCB->CFSR,
                 (unsigned long)SCB->HFSR,
                 (unsigned long)SCB->MMFAR,
                 (unsigned long)SCB->BFAR);

  if (len > 0)
  {
    HAL_UART_Transmit(&huart6, (uint8_t *)buf, (uint16_t)len, 100U);
  }

  /* No IWDG in this build, so this hang is not guaranteed to end in a reset -
   * force the motors to idle before we spin, otherwise they stay at whatever
   * PWM they were at the instant of the fault. */
  Motors_ForceIdleRegistersOnly();

  /* Best-effort: survives even the full power cycle a truly hung, watchdog-less
   * board can only be stopped with, which erases the RAM_D3 copy above. */
  Fault_WriteFlashRecord(name, stack_frame);

  while (1)
  {
    /* Intentionally hang: persisted record above will be reported/logged at next boot (once something resets the board). */
  }
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
__attribute__((naked)) void HardFault_Handler(void)
{
  __asm volatile (
    "tst lr, #4         \n"
    "ite eq             \n"
    "mrseq r0, msp      \n"
    "mrsne r0, psp      \n"
    "b HardFault_HandlerC\n"
  );
}

void HardFault_HandlerC(uint32_t *stack_frame)
{
  Fault_ReportAndHalt("HARDFAULT", stack_frame);
}

/**
  * @brief This function handles Memory management fault.
  */
__attribute__((naked)) void MemManage_Handler(void)
{
  __asm volatile (
    "tst lr, #4         \n"
    "ite eq             \n"
    "mrseq r0, msp      \n"
    "mrsne r0, psp      \n"
    "b MemManage_HandlerC\n"
  );
}

void MemManage_HandlerC(uint32_t *stack_frame)
{
  Fault_ReportAndHalt("MEMFAULT", stack_frame);
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
__attribute__((naked)) void BusFault_Handler(void)
{
  __asm volatile (
    "tst lr, #4         \n"
    "ite eq             \n"
    "mrseq r0, msp      \n"
    "mrsne r0, psp      \n"
    "b BusFault_HandlerC\n"
  );
}

void BusFault_HandlerC(uint32_t *stack_frame)
{
  Fault_ReportAndHalt("BUSFAULT", stack_frame);
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
__attribute__((naked)) void UsageFault_Handler(void)
{
  __asm volatile (
    "tst lr, #4         \n"
    "ite eq             \n"
    "mrseq r0, msp      \n"
    "mrsne r0, psp      \n"
    "b UsageFault_HandlerC\n"
  );
}

void UsageFault_HandlerC(uint32_t *stack_frame)
{
  Fault_ReportAndHalt("USAGEFAULT", stack_frame);
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USB OTG FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
