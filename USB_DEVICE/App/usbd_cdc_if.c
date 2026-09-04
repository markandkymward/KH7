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
#include "main.h"
#include "app.h"
#include "communications.h"
#include "telemetry.h"
#include "fault_record.h"
#include "mag.h"

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

  if (strcmp(line, "ALTHOLD KP GET") == 0)
  {
    printf("ALTHOLD_KP[src=get value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetAltholdAltHoldKp(),
           (double)APP_ALTHOLD_ALT_HOLD_KP_MIN,
           (double)APP_ALTHOLD_ALT_HOLD_KP_MAX);
    return;
  }

  prefix = "ALTHOLD KP SET ";
  if (strncmp(line, prefix, strlen(prefix)) == 0)
  {
    float kp;

    cursor = (char *)(line + strlen(prefix));
    kp = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != '\0'))
    {
      printf("ALTHOLD_KP_SET[FAIL reason=parse]\r\n");
      return;
    }

    if (App_SetAltholdAltHoldKp(kp) == 0U)
    {
      printf("ALTHOLD_KP_SET[FAIL reason=range min=%.4f max=%.4f]\r\n",
             (double)APP_ALTHOLD_ALT_HOLD_KP_MIN, (double)APP_ALTHOLD_ALT_HOLD_KP_MAX);
      return;
    }

    printf("ALTHOLD_KP[src=set value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetAltholdAltHoldKp(),
           (double)APP_ALTHOLD_ALT_HOLD_KP_MIN,
           (double)APP_ALTHOLD_ALT_HOLD_KP_MAX);
    return;
  }

  if (strcmp(line, "ALTHOLD MAXCLIMB GET") == 0)
  {
    printf("ALTHOLD_MAXCLIMB[src=get value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetAltholdMaxClimbMps(),
           (double)APP_ALTHOLD_MAX_CLIMB_MIN,
           (double)APP_ALTHOLD_MAX_CLIMB_MAX);
    return;
  }

  prefix = "ALTHOLD MAXCLIMB SET ";
  if (strncmp(line, prefix, strlen(prefix)) == 0)
  {
    float max_climb_mps;

    cursor = (char *)(line + strlen(prefix));
    max_climb_mps = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != '\0'))
    {
      printf("ALTHOLD_MAXCLIMB_SET[FAIL reason=parse]\r\n");
      return;
    }

    if (App_SetAltholdMaxClimbMps(max_climb_mps) == 0U)
    {
      printf("ALTHOLD_MAXCLIMB_SET[FAIL reason=range min=%.4f max=%.4f]\r\n",
             (double)APP_ALTHOLD_MAX_CLIMB_MIN, (double)APP_ALTHOLD_MAX_CLIMB_MAX);
      return;
    }

    printf("ALTHOLD_MAXCLIMB[src=set value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetAltholdMaxClimbMps(),
           (double)APP_ALTHOLD_MAX_CLIMB_MIN,
           (double)APP_ALTHOLD_MAX_CLIMB_MAX);
    return;
  }

  if (strcmp(line, "ALTHOLD POSKI GET") == 0)
  {
    printf("ALTHOLD_POSKI[src=get value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetAltholdPosKi(),
           (double)APP_ALTHOLD_POS_KI_MIN,
           (double)APP_ALTHOLD_POS_KI_MAX);
    return;
  }

  prefix = "ALTHOLD POSKI SET ";
  if (strncmp(line, prefix, strlen(prefix)) == 0)
  {
    float ki;

    cursor = (char *)(line + strlen(prefix));
    ki = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != '\0'))
    {
      printf("ALTHOLD_POSKI_SET[FAIL reason=parse]\r\n");
      return;
    }

    if (App_SetAltholdPosKi(ki) == 0U)
    {
      printf("ALTHOLD_POSKI_SET[FAIL reason=range min=%.4f max=%.4f]\r\n",
             (double)APP_ALTHOLD_POS_KI_MIN, (double)APP_ALTHOLD_POS_KI_MAX);
      return;
    }

    printf("ALTHOLD_POSKI[src=set value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetAltholdPosKi(),
           (double)APP_ALTHOLD_POS_KI_MIN,
           (double)APP_ALTHOLD_POS_KI_MAX);
    return;
  }

  if (strcmp(line, "NAVPOS KP GET") == 0)
  {
    printf("NAVPOS_KP[src=get value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetNavPosKp(),
           (double)APP_NAVPOS_KP_MIN,
           (double)APP_NAVPOS_KP_MAX);
    return;
  }

  prefix = "NAVPOS KP SET ";
  if (strncmp(line, prefix, strlen(prefix)) == 0)
  {
    float kp;

    cursor = (char *)(line + strlen(prefix));
    kp = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != '\0'))
    {
      printf("NAVPOS_KP_SET[FAIL reason=parse]\r\n");
      return;
    }

    if (App_SetNavPosKp(kp) == 0U)
    {
      printf("NAVPOS_KP_SET[FAIL reason=range min=%.4f max=%.4f]\r\n",
             (double)APP_NAVPOS_KP_MIN, (double)APP_NAVPOS_KP_MAX);
      return;
    }

    printf("NAVPOS_KP[src=set value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetNavPosKp(),
           (double)APP_NAVPOS_KP_MIN,
           (double)APP_NAVPOS_KP_MAX);
    return;
  }

  if (strcmp(line, "NAVPOS KI GET") == 0)
  {
    printf("NAVPOS_KI[src=get value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetNavPosKi(),
           (double)APP_NAVPOS_KI_MIN,
           (double)APP_NAVPOS_KI_MAX);
    return;
  }

  prefix = "NAVPOS KI SET ";
  if (strncmp(line, prefix, strlen(prefix)) == 0)
  {
    float ki;

    cursor = (char *)(line + strlen(prefix));
    ki = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != '\0'))
    {
      printf("NAVPOS_KI_SET[FAIL reason=parse]\r\n");
      return;
    }

    if (App_SetNavPosKi(ki) == 0U)
    {
      printf("NAVPOS_KI_SET[FAIL reason=range min=%.4f max=%.4f]\r\n",
             (double)APP_NAVPOS_KI_MIN, (double)APP_NAVPOS_KI_MAX);
      return;
    }

    printf("NAVPOS_KI[src=set value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetNavPosKi(),
           (double)APP_NAVPOS_KI_MIN,
           (double)APP_NAVPOS_KI_MAX);
    return;
  }

  if (strcmp(line, "ALTHOLD VZKP GET") == 0)
  {
    printf("ALTHOLD_VZKP[src=get value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetAltholdVzKp(),
           (double)APP_ALTHOLD_VZ_KP_MIN,
           (double)APP_ALTHOLD_VZ_KP_MAX);
    return;
  }

  prefix = "ALTHOLD VZKP SET ";
  if (strncmp(line, prefix, strlen(prefix)) == 0)
  {
    float vzkp;

    cursor = (char *)(line + strlen(prefix));
    vzkp = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != '\0'))
    {
      printf("ALTHOLD_VZKP_SET[FAIL reason=parse]\r\n");
      return;
    }

    if (App_SetAltholdVzKp(vzkp) == 0U)
    {
      printf("ALTHOLD_VZKP_SET[FAIL reason=range min=%.4f max=%.4f]\r\n",
             (double)APP_ALTHOLD_VZ_KP_MIN, (double)APP_ALTHOLD_VZ_KP_MAX);
      return;
    }

    printf("ALTHOLD_VZKP[src=set value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetAltholdVzKp(),
           (double)APP_ALTHOLD_VZ_KP_MIN,
           (double)APP_ALTHOLD_VZ_KP_MAX);
    return;
  }

  if (strcmp(line, "ALTHOLD VZKI GET") == 0)
  {
    printf("ALTHOLD_VZKI[src=get value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetAltholdVzKi(),
           (double)APP_ALTHOLD_VZ_KI_MIN,
           (double)APP_ALTHOLD_VZ_KI_MAX);
    return;
  }

  prefix = "ALTHOLD VZKI SET ";
  if (strncmp(line, prefix, strlen(prefix)) == 0)
  {
    float vzki;

    cursor = (char *)(line + strlen(prefix));
    vzki = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != '\0'))
    {
      printf("ALTHOLD_VZKI_SET[FAIL reason=parse]\r\n");
      return;
    }

    if (App_SetAltholdVzKi(vzki) == 0U)
    {
      printf("ALTHOLD_VZKI_SET[FAIL reason=range min=%.4f max=%.4f]\r\n",
             (double)APP_ALTHOLD_VZ_KI_MIN, (double)APP_ALTHOLD_VZ_KI_MAX);
      return;
    }

    printf("ALTHOLD_VZKI[src=set value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetAltholdVzKi(),
           (double)APP_ALTHOLD_VZ_KI_MIN,
           (double)APP_ALTHOLD_VZ_KI_MAX);
    return;
  }

  if (strcmp(line, "ALTHOLD DAMPGAIN GET") == 0)
  {
    printf("ALTHOLD_DAMPGAIN[src=get value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetBaroVzDampGain(),
           (double)APP_BARO_VZ_DAMP_GAIN_MIN,
           (double)APP_BARO_VZ_DAMP_GAIN_MAX);
    return;
  }

  prefix = "ALTHOLD DAMPGAIN SET ";
  if (strncmp(line, prefix, strlen(prefix)) == 0)
  {
    float damp_gain;

    cursor = (char *)(line + strlen(prefix));
    damp_gain = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != '\0'))
    {
      printf("ALTHOLD_DAMPGAIN_SET[FAIL reason=parse]\r\n");
      return;
    }

    if (App_SetBaroVzDampGain(damp_gain) == 0U)
    {
      printf("ALTHOLD_DAMPGAIN_SET[FAIL reason=range min=%.4f max=%.4f]\r\n",
             (double)APP_BARO_VZ_DAMP_GAIN_MIN, (double)APP_BARO_VZ_DAMP_GAIN_MAX);
      return;
    }

    printf("ALTHOLD_DAMPGAIN[src=set value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetBaroVzDampGain(),
           (double)APP_BARO_VZ_DAMP_GAIN_MIN,
           (double)APP_BARO_VZ_DAMP_GAIN_MAX);
    return;
  }

  if (strcmp(line, "ALTHOLD DAMPLIMIT GET") == 0)
  {
    printf("ALTHOLD_DAMPLIMIT[src=get value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetBaroVzDampLimit(),
           (double)APP_BARO_VZ_DAMP_LIMIT_MIN,
           (double)APP_BARO_VZ_DAMP_LIMIT_MAX);
    return;
  }

  prefix = "ALTHOLD DAMPLIMIT SET ";
  if (strncmp(line, prefix, strlen(prefix)) == 0)
  {
    float damp_limit;

    cursor = (char *)(line + strlen(prefix));
    damp_limit = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != '\0'))
    {
      printf("ALTHOLD_DAMPLIMIT_SET[FAIL reason=parse]\r\n");
      return;
    }

    if (App_SetBaroVzDampLimit(damp_limit) == 0U)
    {
      printf("ALTHOLD_DAMPLIMIT_SET[FAIL reason=range min=%.4f max=%.4f]\r\n",
             (double)APP_BARO_VZ_DAMP_LIMIT_MIN, (double)APP_BARO_VZ_DAMP_LIMIT_MAX);
      return;
    }

    printf("ALTHOLD_DAMPLIMIT[src=set value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetBaroVzDampLimit(),
           (double)APP_BARO_VZ_DAMP_LIMIT_MIN,
           (double)APP_BARO_VZ_DAMP_LIMIT_MAX);
    return;
  }

  if (strcmp(line, "ALTHOLD SAVE") == 0)
  {
    if (App_SaveAltholdSettings() == 0U)
    {
      printf("ALTHOLD_SAVE[FAIL]\r\n");
      return;
    }
    printf("ALTHOLD_SAVE[OK]\r\n");
    return;
  }

  if (strcmp(line, "ALTHOLD LOAD") == 0)
  {
    if (App_LoadAltholdSettings() == 0U)
    {
      printf("ALTHOLD_LOAD[FAIL_OR_NONE_SAVED]\r\n");
      return;
    }
    printf("ALTHOLD_LOAD[OK]\r\n");
    printf("ALTHOLD_KP[src=get value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetAltholdAltHoldKp(),
           (double)APP_ALTHOLD_ALT_HOLD_KP_MIN,
           (double)APP_ALTHOLD_ALT_HOLD_KP_MAX);
    printf("ALTHOLD_MAXCLIMB[src=get value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetAltholdMaxClimbMps(),
           (double)APP_ALTHOLD_MAX_CLIMB_MIN,
           (double)APP_ALTHOLD_MAX_CLIMB_MAX);
    printf("ALTHOLD_POSKI[src=get value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetAltholdPosKi(),
           (double)APP_ALTHOLD_POS_KI_MIN,
           (double)APP_ALTHOLD_POS_KI_MAX);
    printf("ALTHOLD_VZKP[src=get value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetAltholdVzKp(),
           (double)APP_ALTHOLD_VZ_KP_MIN,
           (double)APP_ALTHOLD_VZ_KP_MAX);
    printf("ALTHOLD_VZKI[src=get value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetAltholdVzKi(),
           (double)APP_ALTHOLD_VZ_KI_MIN,
           (double)APP_ALTHOLD_VZ_KI_MAX);
    printf("ALTHOLD_DAMPGAIN[src=get value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetBaroVzDampGain(),
           (double)APP_BARO_VZ_DAMP_GAIN_MIN,
           (double)APP_BARO_VZ_DAMP_GAIN_MAX);
    printf("ALTHOLD_DAMPLIMIT[src=get value=%.4f min=%.4f max=%.4f]\r\n",
           (double)App_GetBaroVzDampLimit(),
           (double)APP_BARO_VZ_DAMP_LIMIT_MIN,
           (double)APP_BARO_VZ_DAMP_LIMIT_MAX);
    return;
  }

  if (strcmp(line, "ATT DEFAULT") == 0)
  {
    App_RequestAttitudeDefaults();
    printf("ATT_DEFAULT[QUEUED]\r\n");
    return;
  }

  if (strcmp(line, "ATT ZERO") == 0)
  {
    App_RequestAttitudeZero();
    printf("ATT_ZERO[QUEUED]\r\n");
    return;
  }

  if (strcmp(line, "TELARM STATUS") == 0)
  {
    /* Read-only: RC channel 7 (hi=on, lo=off) is the sole control for this flag now -
     * no command can set it, only query the current confirmed state. */
    App_PrintArmedTelemetryStatus();
    return;
  }

  if (strcmp(line, "MAG CAL START") == 0)
  {
    App_RequestMagCalStart();
    printf("MAG_CAL[QUEUED start]\r\n");
    return;
  }

  if (strcmp(line, "MAG CAL STOP") == 0)
  {
    App_RequestMagCalStop();
    printf("MAG_CAL[QUEUED stop]\r\n");
    return;
  }

  if (strcmp(line, "MAG CAL STATUS") == 0)
  {
    printf("MAG_CAL_STATUS[calibrated=%u cx=%.4f cy=%.4f cz=%.4f "
           "wxx=%.4f wyy=%.4f wzz=%.4f wxy=%.4f wxz=%.4f wyz=%.4f]\r\n",
           (unsigned int)Mag_IsCalibrated(),
           (double)Mag_GetCalCenterX(), (double)Mag_GetCalCenterY(), (double)Mag_GetCalCenterZ(),
           (double)Mag_GetCalMatrixXX(), (double)Mag_GetCalMatrixYY(), (double)Mag_GetCalMatrixZZ(),
           (double)Mag_GetCalMatrixXY(), (double)Mag_GetCalMatrixXZ(), (double)Mag_GetCalMatrixYZ());
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

  if (strcmp(line, "SDLOG STATUS") == 0)
  {
    App_RequestSdLogStatus();
    return;
  }

  if (strcmp(line, "SDLOG DUMP") == 0)
  {
    App_RequestSdLogDump();
    return;
  }

  if (strcmp(line, "SDLOG DUMP LAST") == 0)
  {
    App_RequestSdLogDumpLast();
    return;
  }

  if (strcmp(line, "GPS SCAN") == 0)
  {
    App_RequestGpsScan();
    printf("GPS_SCAN[QUEUED]\r\n");
    return;
  }

  if (strcmp(line, "GPS FACTORY RESET") == 0)
  {
    App_RequestGpsFactoryReset();
    printf("GPS_FACTORY_RESET[QUEUED]\r\n");
    return;
  }

  if (strcmp(line, "I2C SCAN") == 0)
  {
    App_RequestI2c1Scan();
    printf("I2C1_SCAN[QUEUED]\r\n");
    return;
  }

  if (strcmp(line, "RESET STATUS") == 0)
  {
    volatile ResetInfo_t *reset_info = RESET_INFO;

    if (reset_info->magic != RESET_INFO_MAGIC)
    {
      printf("RESET_STATUS[none]\r\n");
    }
    else
    {
      uint32_t flags = reset_info->reset_cause_flags;
      printf("RESET_STATUS[boot_count=%lu cause=%s%s%s%s%s%s raw=0x%08lX]\r\n",
             (unsigned long)reset_info->boot_count,
             (flags & RCC_RSR_IWDG1RSTF) ? "IWDG " : "",
             (flags & RCC_RSR_WWDG1RSTF) ? "WWDG " : "",
             (flags & RCC_RSR_BORRSTF) ? "BOR " : "",
             (flags & RCC_RSR_PINRSTF) ? "PIN " : "",
             (flags & RCC_RSR_PORRSTF) ? "POR " : "",
             (flags & RCC_RSR_SFTRSTF) ? "SOFT " : "",
             (unsigned long)flags);
    }
    return;
  }

  prefix = "SDLOG DUMP FROM ";
  if (strncmp(line, prefix, strlen(prefix)) == 0)
  {
    uint32_t block = (uint32_t)strtoul(line + strlen(prefix), NULL, 10);
    App_RequestSdLogDumpFrom(block);
    return;
  }

  if (strcmp(line, "SDLOG ERASE") == 0)
  {
    App_RequestSdLogErase();
    printf("SDLOG_ERASE[QUEUED]\r\n");
    return;
  }

  /* Unsolicited, sent by the ESP32 bridge (see esp32_s3_uart6_wifi_bridge.ino) - NOT a
   * ground-station-typed command. Deliberately silent (no printf response): at this
   * rate a response line would roughly double UART6 TX traffic for zero benefit, since
   * nothing consumes it. See App_SetRangefinderCm()'s comment for why this exists (SD-log
   * ground truth on the FC's own time_ms axis instead of an error-prone post-hoc clock
   * alignment between two independently-timed capture streams).
   *
   * Format (2026-08-25, replacing the simpler "RANGE <cm>"/"LUNA <cm>" lines):
   * "SENSOR <id> <valid> <range_cm> <confidence> <ts_us>" - <range_cm> is the raw SLANT
   * range, deliberately not tilt-compensated on the ESP32 side (attitude is estimated
   * here on the FC, so slant-to-vertical projection belongs here too, using the fixed
   * mounting descriptor from SENSOR_CFG below plus a live attitude estimate - not yet
   * implemented, this just stores what's needed for it). A malformed line is silently
   * ignored (same "no response" convention as everything else in this block) rather
   * than reported, since a single dropped sensor sample isn't worth the traffic. */
  prefix = "SENSOR ";
  if (strncmp(line, prefix, strlen(prefix)) == 0)
  {
    uint8_t is_sonar;
    long valid_field;
    float range_cm;
    float confidence;
    unsigned long sensor_ts_us;

    cursor = (char *)(line + strlen(prefix));
    if (strncmp(cursor, "SONAR ", 6) == 0)
    {
      is_sonar = 1U;
      cursor += 6;
    }
    else if (strncmp(cursor, "LUNA ", 5) == 0)
    {
      is_sonar = 0U;
      cursor += 5;
    }
    else
    {
      return;
    }

    valid_field = strtol(cursor, &token_end, 10);
    if ((token_end == cursor) || (*token_end != ' ')) { return; }
    cursor = token_end + 1;

    range_cm = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != ' ')) { return; }
    cursor = token_end + 1;

    confidence = strtof(cursor, &token_end);
    if ((token_end == cursor) || (*token_end != ' ')) { return; }
    cursor = token_end + 1;

    sensor_ts_us = strtoul(cursor, &token_end, 10);
    if (token_end == cursor) { return; }

    /* range_cm is passed through UNCOLLAPSED here (2026-08-29) - App_SetRangefinderCm()/
     * App_SetLunaCm() each decide separately how to use an invalid reading (still
     * collapse to 0.0f for their own "unavailable" state, but forward the true value
     * to VertEkf_UpdateRange() alongside the explicit valid flag) - see those
     * functions' comments for why collapsing it here made a genuine 0cm reading
     * indistinguishable from an actually-invalid one. */
    if (is_sonar != 0U)
    {
      App_SetRangefinderCm(range_cm, confidence, (uint32_t)sensor_ts_us, (uint8_t)(valid_field != 0));
    }
    else
    {
      App_SetLunaCm(range_cm, confidence, (uint32_t)sensor_ts_us, (uint8_t)(valid_field != 0));
    }
    return;
  }

  /* Fixed mounting/orientation descriptor, sent once at ESP32 boot and re-sent
   * periodically - see esp32_s3_uart6_wifi_bridge.ino's RANGE_MOUNT_AXIS comment.
   * Format: "SENSOR_CFG <id> <axis> <offset_deg>". */
  prefix = "SENSOR_CFG ";
  if (strncmp(line, prefix, strlen(prefix)) == 0)
  {
    uint8_t is_sonar;
    char axis_buf[8];
    const char *axis_start;
    size_t axis_len;
    float offset_deg;

    cursor = (char *)(line + strlen(prefix));
    if (strncmp(cursor, "SONAR ", 6) == 0)
    {
      is_sonar = 1U;
      cursor += 6;
    }
    else if (strncmp(cursor, "LUNA ", 5) == 0)
    {
      is_sonar = 0U;
      cursor += 5;
    }
    else
    {
      return;
    }

    axis_start = cursor;
    while ((*cursor != '\0') && (*cursor != ' ')) { cursor++; }
    axis_len = (size_t)(cursor - axis_start);
    if ((axis_len == 0U) || (axis_len >= sizeof(axis_buf)) || (*cursor != ' ')) { return; }
    (void)memcpy(axis_buf, axis_start, axis_len);
    axis_buf[axis_len] = '\0';
    cursor++;

    offset_deg = strtof(cursor, &token_end);
    if (token_end == cursor) { return; }

    if (is_sonar != 0U)
    {
      App_SetRangefinderMountDescriptor(axis_buf, offset_deg);
    }
    else
    {
      App_SetLunaMountDescriptor(axis_buf, offset_deg);
    }
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
