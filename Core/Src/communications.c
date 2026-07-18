#include "communications.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

int __io_putchar(int ch)
{
  uint8_t c;

  c = (uint8_t)ch;

  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
    return ch;
  }

  (void)CDC_Transmit_FS(&c, 1U);

  return ch;
}
