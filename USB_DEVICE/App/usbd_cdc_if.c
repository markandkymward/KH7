/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
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
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "app.h"
#include "communications.h"
#include "telemetry.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
static char g_usb_cmd_line[192];
static uint8_t g_usb_cmd_len = 0U;
static const char g_escpt_escape_seq[] = "+++ESCPTOFF+++";
static uint8_t g_escpt_escape_match = 0U;

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
void CDC_ProcessCommandLine(const char *line);

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  uint32_t i;
  char ch;

  if (Communications_IsEscPassthroughEnabled() != 0U)
  {
    for (i = 0U; i < *Len; i++)
    {
      uint8_t b = Buf[i];
      if (b == (uint8_t)g_escpt_escape_seq[g_escpt_escape_match])
      {
        g_escpt_escape_match++;
        if (g_escpt_escape_match >= (sizeof(g_escpt_escape_seq) - 1U))
        {
          Communications_EscPassthroughSetEnabled(0U);
          g_escpt_escape_match = 0U;
          printf("ESCPT[OFF]\r\n");
        }
      }
      else
      {
        g_escpt_escape_match = (b == (uint8_t)g_escpt_escape_seq[0]) ? 1U : 0U;
      }
    }

    Communications_EscPassthroughFromUsb(Buf, (uint16_t)(*Len));
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return (USBD_OK);
  }

  for (i = 0U; i < *Len; i++)
  {
    ch = (char)Buf[i];

    if (ch == '\r')
    {
      continue;
    }

    if (ch == '\n')
    {
      if (g_usb_cmd_len > 0U)
      {
        g_usb_cmd_line[g_usb_cmd_len] = '\0';
        CDC_ProcessCommandLine(g_usb_cmd_line);
        g_usb_cmd_len = 0U;
      }
      continue;
    }

    if (g_usb_cmd_len < (sizeof(g_usb_cmd_line) - 1U))
    {
      g_usb_cmd_line[g_usb_cmd_len++] = ch;
    }
    else
    {
      g_usb_cmd_len = 0U;
    }
  }

  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

void CDC_ProcessCommandLine(const char *line)
{
  const char *prefix;
  char *endptr;
  char *cursor;
  char *token_end;
  uint32_t motor_index;
  uint32_t pulse_us;
  App_RatePidGains_t gains;
  App_AttitudeGains_t att_gains;

  if ((line == NULL) || (line[0] == '\0'))
  {
    return;
  }

  if (strcmp(line, "ESCPT ON") == 0)
  {
    g_usb_cmd_len = 0U;
    g_escpt_escape_match = 0U;
    Communications_EscPassthroughSetEnabled(1U);
    printf("ESCPT[ON UART7 115200] SEND +++ESCPTOFF+++ TO EXIT\r\n");
    return;
  }

  if (strcmp(line, "ESCPT OFF") == 0)
  {
    Communications_EscPassthroughSetEnabled(0U);
    g_escpt_escape_match = 0U;
    printf("ESCPT[OFF]\r\n");
    return;
  }

  if (strcmp(line, "PID GET") == 0)
  {
    App_GetRatePidGains(&gains);
    Telemetry_PrintRatePid(&gains, "get");
    return;
  }

  if (strcmp(line, "ATT GET") == 0)
  {
    App_GetAttitudeGains(&att_gains);
    printf("ATT[src=get]=[ROLL_KP %.4f PITCH_KP %.4f MAX_ANG %.2f]\r\n",
           (double)att_gains.roll_kp,
           (double)att_gains.pitch_kp,
           (double)att_gains.max_angle_deg);
    return;
  }

  if (strcmp(line, "ATT DEFAULT") == 0)
  {
    App_RequestAttitudeDefaults();
    printf("ATT_DEFAULT[QUEUED]\r\n");
    return;
  }

  if (strcmp(line, "PID DEFAULT") == 0)
  {
    App_RequestRatePidDefaults();
    printf("PID_DEFAULT[QUEUED]\r\n");
    return;
  }

  if (strcmp(line, "PID SAVE") == 0)
  {
    App_RequestRatePidSave();
    printf("PID_SAVE[QUEUED]\r\n");
    return;
  }

  if (strcmp(line, "PID LOAD") == 0)
  {
    App_RequestRatePidLoad();
    printf("PID_LOAD[QUEUED]\r\n");
    return;
  }

  if (strcmp(line, "PID DEBUG") == 0)
  {
    App_PrintPidDebug();
    return;
  }

  if (strcmp(line, "GLOG DUMP") == 0)
  {
    App_RequestGyroLogDump();
    printf("GLOG_DUMP[QUEUED]\r\n");
    return;
  }

  if (strcmp(line, "SD INIT") == 0)
  {
    App_RequestSdInit();
    printf("SD_INIT[QUEUED]\r\n");
    return;
  }

  if (strcmp(line, "SD STATUS") == 0)
  {
    App_RequestSdStatus();
    return;
  }

  prefix = "SD RBLOCK ";
  if (strncmp(line, prefix, strlen(prefix)) == 0)
  {
    uint32_t block = (uint32_t)strtoul(line + strlen(prefix), NULL, 10);
    App_RequestSdReadBlock(block);
    return;
  }

  prefix = "SD WBLOCK ";
  if (strncmp(line, prefix, strlen(prefix)) == 0)
  {
    uint32_t block = (uint32_t)strtoul(line + strlen(prefix), NULL, 10);
    App_RequestSdWriteBlock(block);
    return;
  }

  prefix = "PID SET ";
  if (strncmp(line, prefix, strlen(prefix)) == 0)
  {
    cursor = (char *)(line + strlen(prefix));

    gains.roll.kp = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != ' '))
    {
      printf("PID_SET[FAIL field=roll.kp]\r\n");
      return;
    }
    cursor = token_end + 1;

    gains.roll.ki = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != ' '))
    {
      printf("PID_SET[FAIL field=roll.ki]\r\n");
      return;
    }
    cursor = token_end + 1;

    gains.roll.kd = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != ' '))
    {
      printf("PID_SET[FAIL field=roll.kd]\r\n");
      return;
    }
    cursor = token_end + 1;

    gains.roll.kff = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != ' '))
    {
      printf("PID_SET[FAIL field=roll.kff]\r\n");
      return;
    }
    cursor = token_end + 1;

    gains.pitch.kp = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != ' '))
    {
      printf("PID_SET[FAIL field=pitch.kp]\r\n");
      return;
    }
    cursor = token_end + 1;

    gains.pitch.ki = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != ' '))
    {
      printf("PID_SET[FAIL field=pitch.ki]\r\n");
      return;
    }
    cursor = token_end + 1;

    gains.pitch.kd = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != ' '))
    {
      printf("PID_SET[FAIL field=pitch.kd]\r\n");
      return;
    }
    cursor = token_end + 1;

    gains.pitch.kff = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != ' '))
    {
      printf("PID_SET[FAIL field=pitch.kff]\r\n");
      return;
    }
    cursor = token_end + 1;

    gains.yaw.kp = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != ' '))
    {
      printf("PID_SET[FAIL field=yaw.kp]\r\n");
      return;
    }
    cursor = token_end + 1;

    gains.yaw.ki = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != ' '))
    {
      printf("PID_SET[FAIL field=yaw.ki]\r\n");
      return;
    }
    cursor = token_end + 1;

    gains.yaw.kd = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != ' '))
    {
      printf("PID_SET[FAIL field=yaw.kd]\r\n");
      return;
    }
    cursor = token_end + 1;

    gains.yaw.kff = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != '\0'))
    {
      printf("PID_SET[FAIL field=yaw.kff trailing=%d]\r\n", (int)(*token_end));
      return;
    }

    if (App_RequestRatePidSetAndSave(&gains) == 0U)
    {
      printf("PID_SET[FAIL range]\r\n");
      return;
    }
    printf("PID_SET[QUEUED]\r\n");
    return;
  }

  prefix = "ATT SET ";
  if (strncmp(line, prefix, strlen(prefix)) == 0)
  {
    cursor = (char *)(line + strlen(prefix));

    att_gains.roll_kp = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != ' '))
    {
      printf("ATT_SET[FAIL]\r\n");
      return;
    }
    cursor = token_end + 1;

    att_gains.pitch_kp = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != ' '))
    {
      printf("ATT_SET[FAIL]\r\n");
      return;
    }
    cursor = token_end + 1;

    att_gains.max_angle_deg = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != '\0'))
    {
      printf("ATT_SET[FAIL]\r\n");
      return;
    }

    if (App_RequestAttitudeSetAndSave(&att_gains) == 0U)
    {
      printf("ATT_SET[FAIL]\r\n");
      return;
    }

    printf("ATT_SET[QUEUED]\r\n");
    return;
  }

  if (strcmp(line, "BOOT LOG") == 0)
  {
    const char *boot_log = App_GetBootLog();
    if ((boot_log != NULL) && (boot_log[0] != '\0'))
    {
      printf("--- BOOT LOG START ---\r\n%s--- BOOT LOG END ---\r\n", boot_log);
    }
    else
    {
      printf("--- BOOT LOG EMPTY ---\r\n");
    }
    return;
  }

  if ((strcmp(line, "MTEST OFF") == 0) || (strcmp(line, "MTEST 0 0") == 0))
  {
    App_SetUsbMotorTest(0U, 1U, 1100U);
    return;
  }

  prefix = "MTEST ";
  if (strncmp(line, prefix, strlen(prefix)) != 0)
  {
    return;
  }

  motor_index = strtoul(line + strlen(prefix), &endptr, 10);
  if ((endptr == NULL) || (*endptr != ' '))
  {
    return;
  }

  pulse_us = strtoul(endptr + 1, NULL, 10);

  if ((motor_index < 1U) || (motor_index > 4U))
  {
    return;
  }

  if (pulse_us < 988U)
  {
    pulse_us = 988U;
  }
  else if (pulse_us > 2012U)
  {
    pulse_us = 2012U;
  }

  App_SetUsbMotorTest(1U, (uint8_t)motor_index, (uint16_t)pulse_us);
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
