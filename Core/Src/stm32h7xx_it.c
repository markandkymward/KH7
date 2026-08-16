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
