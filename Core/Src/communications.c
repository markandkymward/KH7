#include "communications.h"

#include "main.h"
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
extern UART_HandleTypeDef huart6;

#define COMM_USB_TX_BUFFER_SIZE   192U
#define COMM_USB_TX_TIMEOUT_MS    20U

static uint8_t g_usb_tx_buffer[COMM_USB_TX_BUFFER_SIZE];
static uint16_t g_usb_tx_len = 0U;

static uint8_t Communications_IsUsbTxBusy(void)
{
  USBD_CDC_HandleTypeDef *hcdc;

  if (hUsbDeviceFS.pClassData == NULL)
  {
    return 1U;
  }

  hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  return (uint8_t)(hcdc->TxState != 0U);
}

static void Communications_FlushUsbTxBuffer(void)
{
  uint32_t start_ms;

  if (g_usb_tx_len == 0U)
  {
    return;
  }

  (void)HAL_UART_Transmit(&huart6, g_usb_tx_buffer, g_usb_tx_len, 5U);

  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
    g_usb_tx_len = 0U;
    return;
  }

  start_ms = HAL_GetTick();
  while (CDC_Transmit_FS(g_usb_tx_buffer, g_usb_tx_len) == USBD_BUSY)
  {
    if ((HAL_GetTick() - start_ms) >= COMM_USB_TX_TIMEOUT_MS)
    {
      g_usb_tx_len = 0U;
      return;
    }
  }

  while (Communications_IsUsbTxBusy() != 0U)
  {
    if ((HAL_GetTick() - start_ms) >= COMM_USB_TX_TIMEOUT_MS)
    {
      break;
    }
  }

  g_usb_tx_len = 0U;
}

int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;

  g_usb_tx_buffer[g_usb_tx_len++] = c;

  if ((c == '\n') || (g_usb_tx_len >= COMM_USB_TX_BUFFER_SIZE))
  {
    Communications_FlushUsbTxBuffer();
  }

  return ch;
}
