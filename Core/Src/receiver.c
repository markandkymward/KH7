#include "receiver.h"
#include "communications.h"
#include "gps.h"
#include "app.h"

#include "main.h"

#include <string.h>
#include <stdio.h>

#define CRSF_PAYLOAD_MAX_SIZE          62U
#define CRSF_FRAME_MAX_SIZE            (CRSF_PAYLOAD_MAX_SIZE + 2U)
#define CRSF_FRAME_TYPE_RC_CHANNELS    0x16U
#define CRSF_RC_PAYLOAD_SIZE           22U
#define CRSF_LINK_TIMEOUT_MS           250U
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8U

typedef struct
{
  uint8_t frame[CRSF_FRAME_MAX_SIZE + 2U];
  uint8_t index;
  uint8_t expected_size;
} receiver_parser_t;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;

static volatile receiver_state_t g_receiver_state;
static receiver_parser_t g_parser;
static uint8_t g_rx_byte;
static uint8_t g_uart6_bridge_byte;

static uint8_t Receiver_Crc8(const uint8_t *data, uint8_t length)
{
  uint8_t crc = 0U;
  uint8_t i;

  while (length-- > 0U)
  {
    crc ^= *data++;
    for (i = 0U; i < 8U; i++)
    {
      if ((crc & 0x80U) != 0U)
      {
        crc = (uint8_t)((crc << 1) ^ 0xD5U);
      }
      else
      {
        crc <<= 1;
      }
    }
  }

  return crc;
}

static void Receiver_ResetParser(void)
{
  g_parser.index = 0U;
  g_parser.expected_size = 0U;
}

static void Receiver_DecodeChannels(const uint8_t *payload)
{
  g_receiver_state.channels[0] = (uint16_t)((payload[0] | (payload[1] << 8U)) & 0x07FFU);
  g_receiver_state.channels[1] = (uint16_t)(((payload[1] >> 3U) | (payload[2] << 5U)) & 0x07FFU);
  g_receiver_state.channels[2] = (uint16_t)(((payload[2] >> 6U) | (payload[3] << 2U) | (payload[4] << 10U)) & 0x07FFU);
  g_receiver_state.channels[3] = (uint16_t)(((payload[4] >> 1U) | (payload[5] << 7U)) & 0x07FFU);
  g_receiver_state.channels[4] = (uint16_t)(((payload[5] >> 4U) | (payload[6] << 4U)) & 0x07FFU);
  g_receiver_state.channels[5] = (uint16_t)(((payload[6] >> 7U) | (payload[7] << 1U) | (payload[8] << 9U)) & 0x07FFU);
  g_receiver_state.channels[6] = (uint16_t)(((payload[8] >> 2U) | (payload[9] << 6U)) & 0x07FFU);
  g_receiver_state.channels[7] = (uint16_t)(((payload[9] >> 5U) | (payload[10] << 3U)) & 0x07FFU);
  g_receiver_state.channels[8] = (uint16_t)((payload[11] | (payload[12] << 8U)) & 0x07FFU);
  g_receiver_state.channels[9] = (uint16_t)(((payload[12] >> 3U) | (payload[13] << 5U)) & 0x07FFU);
  g_receiver_state.channels[10] = (uint16_t)(((payload[13] >> 6U) | (payload[14] << 2U) | (payload[15] << 10U)) & 0x07FFU);
  g_receiver_state.channels[11] = (uint16_t)(((payload[15] >> 1U) | (payload[16] << 7U)) & 0x07FFU);
  g_receiver_state.channels[12] = (uint16_t)(((payload[16] >> 4U) | (payload[17] << 4U)) & 0x07FFU);
  g_receiver_state.channels[13] = (uint16_t)(((payload[17] >> 7U) | (payload[18] << 1U) | (payload[19] << 9U)) & 0x07FFU);
  g_receiver_state.channels[14] = (uint16_t)(((payload[19] >> 2U) | (payload[20] << 6U)) & 0x07FFU);
  g_receiver_state.channels[15] = (uint16_t)(((payload[20] >> 5U) | (payload[21] << 3U)) & 0x07FFU);
}

static void Receiver_ProcessFrame(void)
{
  uint8_t frame_size;
  uint8_t frame_type;
  uint8_t payload_size;
  uint8_t crc_expected;
  uint8_t crc_computed;

  frame_size = g_parser.frame[1];
  if ((frame_size < 2U) || (frame_size > CRSF_FRAME_MAX_SIZE))
  {
    return;
  }

  frame_type = g_parser.frame[2];
  payload_size = (uint8_t)(frame_size - 2U);
  crc_expected = g_parser.frame[frame_size + 1U];
  crc_computed = Receiver_Crc8(&g_parser.frame[2], (uint8_t)(frame_size - 1U));

  if (crc_expected != crc_computed)
  {
    return;
  }

  if ((frame_type == CRSF_FRAME_TYPE_RC_CHANNELS) && (payload_size == CRSF_RC_PAYLOAD_SIZE))
  {
    Receiver_DecodeChannels(&g_parser.frame[3]);
    g_receiver_state.link_active = 1U;
    g_receiver_state.frame_received = 1U;
    g_receiver_state.frame_count++;
    g_receiver_state.last_frame_ms = HAL_GetTick();
  }
}

static void Receiver_HandleByte(uint8_t byte)
{
  if (g_parser.index == 0U)
  {
    /* A dropped/corrupted byte anywhere upstream can desync this parser -
     * without checking for the real frame-start address here, a single lost
     * byte can leave it misaligned for multiple subsequent frames (each one
     * only self-corrects by chance + a CRC failure), showing up as a
     * "spotty" link that isn't actually an RF problem. Discard anything that
     * isn't a real frame start and keep scanning for it. */
    if (byte != CRSF_ADDRESS_FLIGHT_CONTROLLER)
    {
      return;
    }
    g_parser.frame[g_parser.index++] = byte;
    return;
  }

  if (g_parser.index == 1U)
  {
    if ((byte < 2U) || (byte > CRSF_FRAME_MAX_SIZE))
    {
      Receiver_ResetParser();
      return;
    }

    g_parser.frame[g_parser.index++] = byte;
    g_parser.expected_size = (uint8_t)(byte + 2U);
    return;
  }

  g_parser.frame[g_parser.index++] = byte;

  if (g_parser.index >= g_parser.expected_size)
  {
    Receiver_ProcessFrame();
    Receiver_ResetParser();
  }
}

void Receiver_Init(void)
{
  uint8_t channel_index;
  HAL_StatusTypeDef status4, status6;
  char msg[128];

  memset((void *)&g_receiver_state, 0, sizeof(g_receiver_state));
  for (channel_index = 0U; channel_index < RECEIVER_CHANNEL_COUNT; channel_index++)
  {
    g_receiver_state.channels[channel_index] = 992U;
  }

  Receiver_ResetParser();
  printf("[Receiver_Init] Starting UART4/UART6 RX interrupts\r\n");
  App_AppendBootLog("[Receiver_Init] Starting UART4/UART6 RX interrupts\r\n");
  
  status4 = HAL_UART_Receive_IT(&huart4, &g_rx_byte, 1U);
  snprintf(msg, sizeof(msg), "[Receiver_Init] UART4 Receive_IT status = %d (0=OK)\r\n", (int)status4);
  printf("%s", msg);
  App_AppendBootLog(msg);
  
  status6 = HAL_UART_Receive_IT(&huart6, &g_uart6_bridge_byte, 1U);
  snprintf(msg, sizeof(msg), "[Receiver_Init] UART6 Receive_IT status = %d (0=OK)\r\n", (int)status6);
  printf("%s", msg);
  App_AppendBootLog(msg);
}

void Receiver_Update(uint32_t now_ms)
{
  if ((g_receiver_state.link_active != 0U) &&
      ((now_ms - g_receiver_state.last_frame_ms) > CRSF_LINK_TIMEOUT_MS))
  {
    g_receiver_state.link_active = 0U;
  }
}

void Receiver_GetState(receiver_state_t *state)
{
  if (state == NULL)
  {
    return;
  }

  __disable_irq();
  memcpy(state, (const void *)&g_receiver_state, sizeof(*state));
  g_receiver_state.frame_received = 0U;
  __enable_irq();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
    Receiver_HandleByte(g_rx_byte);
    (void)HAL_UART_Receive_IT(&huart4, &g_rx_byte, 1U);
  }
  else if (huart->Instance == USART6)
  {
    Communications_HandleUart6Byte(g_uart6_bridge_byte);
    (void)HAL_UART_Receive_IT(huart, &g_uart6_bridge_byte, 1U);
  }
  else if (huart->Instance == USART3)
  {
    GPS_UartRxCpltCallback();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
    Receiver_ResetParser();
    (void)HAL_UART_AbortReceive(huart);
    (void)HAL_UART_Receive_IT(&huart4, &g_rx_byte, 1U);
  }
  else if (huart->Instance == USART6)
  {
    (void)HAL_UART_AbortReceive(huart);
    (void)HAL_UART_Receive_IT(huart, &g_uart6_bridge_byte, 1U);
  }
  else if (huart->Instance == USART3)
  {
    GPS_UartErrorCallback();
  }
}
