#include "communications.h"

#include "main.h"
#include "app.h"
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#include <stdio.h>
#include <ctype.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart7;

#define COMM_USB_TX_BUFFER_SIZE   192U
#define COMM_USB_TX_TIMEOUT_MS    20U
#define COMM_UART6_CMD_BUF_SIZE   192U
#define COMM_UART6_TX_TIMEOUT_MS  25U
#define COMM_LOG_TO_UART6         1U
#define COMM_ESCPT_UART_BUDGET    64U
#define COMM_ESCPT_USB_CHUNK      64U
#define COMM_ESCPT_QUEUE_SIZE     512U
#define COMM_UART6_TX_QUEUE_SIZE  2048U

static uint8_t g_usb_tx_buffer[COMM_USB_TX_BUFFER_SIZE];
static uint16_t g_usb_tx_len = 0U;
static uint8_t g_uart6_tx_queue[COMM_UART6_TX_QUEUE_SIZE];
static volatile uint16_t g_uart6_tx_queue_head = 0U;
static volatile uint16_t g_uart6_tx_queue_tail = 0U;
static volatile uint8_t g_uart6_tx_busy = 0U;
static uint8_t g_uart6_tx_byte = 0U;

static char g_uart6_cmd_line[COMM_UART6_CMD_BUF_SIZE];
static uint8_t g_uart6_cmd_len = 0U;
static char g_uart6_pending_cmd_line[COMM_UART6_CMD_BUF_SIZE];
static uint8_t g_uart6_pending_cmd_ready = 0U;
static volatile uint32_t g_uart6_rx_bytes = 0U;
static volatile uint32_t g_uart6_rx_lines = 0U;
static volatile uint32_t g_uart6_last_byte = 0U;
static volatile uint32_t g_uart6_nonprint_bytes = 0U;
static volatile uint8_t g_esc_passthrough_enabled = 0U;
static uint8_t g_escpt_usb_to_uart[COMM_ESCPT_QUEUE_SIZE];
static volatile uint16_t g_escpt_usb_to_uart_head = 0U;
static volatile uint16_t g_escpt_usb_to_uart_tail = 0U;
static uint8_t g_escpt_uart_to_usb[COMM_ESCPT_QUEUE_SIZE];
static volatile uint16_t g_escpt_uart_to_usb_head = 0U;
static volatile uint16_t g_escpt_uart_to_usb_tail = 0U;

static uint16_t Communications_QueueNext(uint16_t index, uint16_t size)
{
  return (uint16_t)((index + 1U) % size);
}

/* Every caller of these two functions shares one of a handful of ring buffers
 * (UART6 TX queue, ESC-passthrough queues) between main-loop code and an ISR
 * (HAL_UART_TxCpltCallback re-kicks the UART6 TX queue after every byte). The
 * read-modify-write of head/tail must be atomic w.r.t. that ISR: without the
 * critical section here, a push/pop on one side that gets preempted mid-update
 * by a pop/push on the other can desync the indices - not a crash, but a
 * ring buffer that starts draining from a stale offset, i.e. it silently
 * starts transmitting whatever old bytes are still sitting at that offset
 * from a previous, already-sent message instead of the current one. Found
 * 2026-08-15: exactly this desync was splicing old telemetry-line bytes into
 * the middle of SDLOG DUMP hex blocks under the sustained back-to-back TX
 * traffic a multi-thousand-byte dump produces (never reproduced over USB,
 * which doesn't go through this queue at all - see App_ServiceSdLog()). */
static uint8_t Communications_QueuePush(volatile uint16_t *head,
                                        volatile uint16_t *tail,
                                        uint8_t *buffer,
                                        uint16_t size,
                                        uint8_t value)
{
  uint16_t next;
  uint8_t pushed;
  uint32_t primask = __get_PRIMASK();

  /* Save/restore PRIMASK rather than a bare disable/enable pair, so this stays
   * correct when called from inside Communications_Uart6TxKick()'s own critical
   * section - a bare __enable_irq() here would re-enable interrupts in the
   * middle of that caller's section instead of just restoring its own. */
  __disable_irq();
  next = Communications_QueueNext(*head, size);
  if (next == *tail)
  {
    pushed = 0U;
  }
  else
  {
    buffer[*head] = value;
    *head = next;
    pushed = 1U;
  }
  __set_PRIMASK(primask);

  return pushed;
}

static uint8_t Communications_QueuePop(volatile uint16_t *head,
                                       volatile uint16_t *tail,
                                       uint8_t *buffer,
                                       uint16_t size,
                                       uint8_t *value)
{
  uint8_t popped;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if (*tail == *head)
  {
    popped = 0U;
  }
  else
  {
    *value = buffer[*tail];
    *tail = Communications_QueueNext(*tail, size);
    popped = 1U;
  }
  __set_PRIMASK(primask);

  return popped;
}

static uint8_t Communications_IsUart6AutoExecutableCommand(const char *line, size_t len)
{
  char normalized[COMM_UART6_CMD_BUF_SIZE];
  size_t start;
  size_t end;
  size_t out_len;
  size_t i;

  if ((line == NULL) || (len == 0U) || (len >= sizeof(normalized)))
  {
    return 0U;
  }

  start = 0U;
  while ((start < len) && isspace((unsigned char)line[start]))
  {
    start++;
  }

  end = len;
  while ((end > start) && isspace((unsigned char)line[end - 1U]))
  {
    end--;
  }

  out_len = end - start;
  if ((out_len == 0U) || (out_len >= sizeof(normalized)))
  {
    return 0U;
  }

  for (i = 0U; i < out_len; i++)
  {
    normalized[i] = (char)toupper((unsigned char)line[start + i]);
  }
  normalized[out_len] = '\0';

  if ((strcmp(normalized, "PID GET") == 0) ||
      (strcmp(normalized, "ATT GET") == 0) ||
      (strcmp(normalized, "ATT DEFAULT") == 0) ||
      (strcmp(normalized, "ATT ZERO") == 0) ||
      (strcmp(normalized, "MAG CAL START") == 0) ||
      (strcmp(normalized, "MAG CAL STOP") == 0) ||
      (strcmp(normalized, "SDLOG ERASE") == 0) ||
      (strcmp(normalized, "PID DEFAULT") == 0) ||
      (strcmp(normalized, "PID SAVE") == 0) ||
      (strcmp(normalized, "PID LOAD") == 0) ||
      (strcmp(normalized, "PID DEBUG") == 0) ||
      (strcmp(normalized, "BOOT LOG") == 0) ||
      (strcmp(normalized, "MTEST OFF") == 0) ||
      (strcmp(normalized, "MTEST 0 0") == 0))
  {
    return 1U;
  }

  return 0U;
}

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

uint16_t Communications_Uart6TxQueueFreeBytes(void)
{
  uint16_t head;
  uint16_t tail;
  uint16_t used;
  uint32_t primask = __get_PRIMASK();

  /* head/tail are each individually atomic on this core, but read them as a
   * consistent pair anyway - the ISR can advance tail between the two reads
   * otherwise, which would only ever make the result too pessimistic here
   * (not unsafe), but there's no reason to allow that when it's this cheap. */
  __disable_irq();
  head = g_uart6_tx_queue_head;
  tail = g_uart6_tx_queue_tail;
  __set_PRIMASK(primask);

  used = (uint16_t)((head + COMM_UART6_TX_QUEUE_SIZE - tail) % COMM_UART6_TX_QUEUE_SIZE);
  /* Communications_QueueNext()'s "next == tail means full" check means one slot
   * is never usable - capacity is size-1, not size. */
  return (uint16_t)((COMM_UART6_TX_QUEUE_SIZE - 1U) - used);
}

static void Communications_Uart6TxKick(void)
{
  /* Called from both the main loop (after every push) and the TX-complete ISR
   * (after every byte). Only the busy check-and-set needs to be atomic against
   * the ISR's own kick - that's what stops two calls from both starting a
   * transmit. HAL_UART_Transmit_IT() itself deliberately runs with interrupts
   * enabled: wrapping it too (tried 2026-08-15) adds disable/enable overhead on
   * every single transmitted byte, which throttled the ISR-driven drain badly
   * under a SD log dump's sustained burst - most blocks after the first came
   * back silently truncated (ring buffer's existing "drop when full" behavior
   * triggering on nearly every block, not the corruption this was meant to fix).
   * Communications_QueuePop()/Push() are independently atomic (see above), which
   * is what actually has to be true for correctness; a rare preemption here in
   * the gap before HAL_UART_Transmit_IT starts costs at worst one dropped byte
   * from an already best-effort, drop-when-overloaded queue - not corruption. */
  __disable_irq();
  if (g_uart6_tx_busy != 0U)
  {
    __enable_irq();
    return;
  }
  g_uart6_tx_busy = 1U;
  __enable_irq();

  if (Communications_QueuePop(&g_uart6_tx_queue_head, &g_uart6_tx_queue_tail,
                              g_uart6_tx_queue, COMM_UART6_TX_QUEUE_SIZE, &g_uart6_tx_byte) == 0U)
  {
    g_uart6_tx_busy = 0U;
    return;
  }

  if (HAL_UART_Transmit_IT(&huart6, &g_uart6_tx_byte, 1U) != HAL_OK)
  {
    g_uart6_tx_busy = 0U;
  }
}

static void Communications_FlushUsbTxBuffer(void)
{
  if (g_usb_tx_len == 0U)
  {
    return;
  }

  /* Try USB CDC if configured and idle, but never block the control loop. */
  if ((g_esc_passthrough_enabled == 0U) && (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED))
  {
    if (Communications_IsUsbTxBusy() == 0U)
    {
      (void)CDC_Transmit_FS(g_usb_tx_buffer, g_usb_tx_len);
    }
  }

#if COMM_LOG_TO_UART6
  /* Never block the control loop on UART6: a synchronous HAL_UART_Transmit here used to
   * cost up to COMM_UART6_TX_TIMEOUT_MS per printed line. Queue every byte into a ring
   * buffer instead of a single in-flight slot - a telemetry burst is many back-to-back
   * printf lines, and a single-slot "drop if busy" design silently lost everything after
   * the first line. The ring drains automatically via HAL_UART_TxCpltCallback chaining one
   * byte at a time, and only drops the newest bytes if it's genuinely full (sustained
   * overload), never blocking and never truncating a burst down to just its first line. */
  {
    uint16_t i;

    for (i = 0U; i < g_usb_tx_len; i++)
    {
      if (Communications_QueuePush(&g_uart6_tx_queue_head, &g_uart6_tx_queue_tail,
                                   g_uart6_tx_queue, COMM_UART6_TX_QUEUE_SIZE, g_usb_tx_buffer[i]) == 0U)
      {
        break; /* ring full - drop the remainder of this chunk rather than blocking */
      }
    }
  }
  Communications_Uart6TxKick();
#endif

  g_usb_tx_len = 0U;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    g_uart6_tx_busy = 0U;
    Communications_Uart6TxKick();
  }
}

void Communications_Init(void)
{
  /* UART6 receive is started by Receiver_Init via HAL_UART_Receive_IT */
}

void Communications_EscPassthroughSetEnabled(uint8_t enabled)
{
  __disable_irq();
  g_esc_passthrough_enabled = (enabled != 0U) ? 1U : 0U;
  g_escpt_usb_to_uart_head = 0U;
  g_escpt_usb_to_uart_tail = 0U;
  g_escpt_uart_to_usb_head = 0U;
  g_escpt_uart_to_usb_tail = 0U;
  __enable_irq();
}

uint8_t Communications_IsEscPassthroughEnabled(void)
{
  return g_esc_passthrough_enabled;
}

void Communications_EscPassthroughFromUsb(const uint8_t *data, uint16_t len)
{
  uint16_t i;

  if ((g_esc_passthrough_enabled == 0U) || (data == NULL) || (len == 0U))
  {
    return;
  }

  __disable_irq();
  for (i = 0U; i < len; i++)
  {
    (void)Communications_QueuePush(&g_escpt_usb_to_uart_head,
                                   &g_escpt_usb_to_uart_tail,
                                   g_escpt_usb_to_uart,
                                   COMM_ESCPT_QUEUE_SIZE,
                                   data[i]);
  }
  __enable_irq();
}

void Communications_ServiceEscPassthrough(void)
{
  uint8_t byte;
  uint8_t tx_buf[COMM_ESCPT_USB_CHUNK];
  uint16_t tx_len;
  uint16_t i;

  if (g_esc_passthrough_enabled == 0U)
  {
    return;
  }

  for (i = 0U; i < COMM_ESCPT_UART_BUDGET; i++)
  {
    if (HAL_UART_Receive(&huart7, &byte, 1U, 0U) != HAL_OK)
    {
      break;
    }

    __disable_irq();
    (void)Communications_QueuePush(&g_escpt_uart_to_usb_head,
                                   &g_escpt_uart_to_usb_tail,
                                   g_escpt_uart_to_usb,
                                   COMM_ESCPT_QUEUE_SIZE,
                                   byte);
    __enable_irq();
  }

  for (i = 0U; i < COMM_ESCPT_UART_BUDGET; i++)
  {
    __disable_irq();
    if (Communications_QueuePop(&g_escpt_usb_to_uart_head,
                                &g_escpt_usb_to_uart_tail,
                                g_escpt_usb_to_uart,
                                COMM_ESCPT_QUEUE_SIZE,
                                &byte) == 0U)
    {
      __enable_irq();
      break;
    }
    __enable_irq();

    if (HAL_UART_Transmit(&huart7, &byte, 1U, 1U) != HAL_OK)
    {
      __disable_irq();
      (void)Communications_QueuePush(&g_escpt_usb_to_uart_head,
                                     &g_escpt_usb_to_uart_tail,
                                     g_escpt_usb_to_uart,
                                     COMM_ESCPT_QUEUE_SIZE,
                                     byte);
      __enable_irq();
      break;
    }
  }

  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
    return;
  }

  if (Communications_IsUsbTxBusy() != 0U)
  {
    return;
  }

  tx_len = 0U;
  __disable_irq();
  while ((tx_len < COMM_ESCPT_USB_CHUNK) &&
         (Communications_QueuePop(&g_escpt_uart_to_usb_head,
                                  &g_escpt_uart_to_usb_tail,
                                  g_escpt_uart_to_usb,
                                  COMM_ESCPT_QUEUE_SIZE,
                                  &byte) != 0U))
  {
    tx_buf[tx_len++] = byte;
  }
  __enable_irq();

  if (tx_len > 0U)
  {
    (void)CDC_Transmit_FS(tx_buf, tx_len);
  }
}

void Communications_HandleUart6Byte(uint8_t byte)
{
  char ch = (char)byte;

  g_uart6_rx_bytes++;
  g_uart6_last_byte = (uint32_t)byte;
  if ((byte < 32U) || (byte > 126U))
  {
    if ((byte != '\r') && (byte != '\n') && (byte != '\t'))
    {
      g_uart6_nonprint_bytes++;
    }
  }

  if ((ch == '\r') || (ch == '\n'))
  {
    if (g_uart6_cmd_len > 0U)
    {
      size_t n;
      g_uart6_cmd_line[g_uart6_cmd_len] = '\0';

      n = strlen(g_uart6_cmd_line);
      if (n >= sizeof(g_uart6_pending_cmd_line))
      {
        n = sizeof(g_uart6_pending_cmd_line) - 1U;
      }

      memcpy(g_uart6_pending_cmd_line, g_uart6_cmd_line, n);
      g_uart6_pending_cmd_line[n] = '\0';
      g_uart6_pending_cmd_ready = 1U;
      g_uart6_rx_lines++;
      g_uart6_cmd_len = 0U;
    }
    return;
  }

  if (g_uart6_cmd_len < (COMM_UART6_CMD_BUF_SIZE - 1U))
  {
    g_uart6_cmd_line[g_uart6_cmd_len++] = ch;
  }
  else
  {
    g_uart6_cmd_len = 0U;
  }

  /* Match USB behavior: execute fixed commands even if sender omits CR/LF. */
  if ((g_uart6_cmd_len > 0U) &&
      (Communications_IsUart6AutoExecutableCommand(g_uart6_cmd_line, g_uart6_cmd_len) != 0U))
  {
    size_t n;

    g_uart6_cmd_line[g_uart6_cmd_len] = '\0';
    n = strlen(g_uart6_cmd_line);
    if (n >= sizeof(g_uart6_pending_cmd_line))
    {
      n = sizeof(g_uart6_pending_cmd_line) - 1U;
    }

    memcpy(g_uart6_pending_cmd_line, g_uart6_cmd_line, n);
    g_uart6_pending_cmd_line[n] = '\0';
    g_uart6_pending_cmd_ready = 1U;
    g_uart6_rx_lines++;
    g_uart6_cmd_len = 0U;
  }
}

void Communications_GetUart6Debug(uint32_t *rx_bytes,
                                  uint32_t *rx_lines,
                                  uint32_t *pending_ready,
                                  uint32_t *last_byte,
                                  uint32_t *cmd_len,
                                  uint32_t *nonprint_bytes)
{
  __disable_irq();
  if (rx_bytes != NULL)
  {
    *rx_bytes = g_uart6_rx_bytes;
  }
  if (rx_lines != NULL)
  {
    *rx_lines = g_uart6_rx_lines;
  }
  if (pending_ready != NULL)
  {
    *pending_ready = (uint32_t)g_uart6_pending_cmd_ready;
  }
  if (last_byte != NULL)
  {
    *last_byte = g_uart6_last_byte;
  }
  if (cmd_len != NULL)
  {
    *cmd_len = (uint32_t)g_uart6_cmd_len;
  }
  if (nonprint_bytes != NULL)
  {
    *nonprint_bytes = g_uart6_nonprint_bytes;
  }
  __enable_irq();
}

void Communications_ServiceUart6Commands(void)
{
  char local_cmd[COMM_UART6_CMD_BUF_SIZE];
  char pid_line[192];
  int pid_line_len;
  App_RatePidGains_t gains;
  uint8_t has_cmd;

  __disable_irq();
  has_cmd = g_uart6_pending_cmd_ready;
  if (has_cmd != 0U)
  {
    memcpy(local_cmd, g_uart6_pending_cmd_line, sizeof(local_cmd));
    g_uart6_pending_cmd_ready = 0U;
  }
  __enable_irq();

  if (has_cmd == 0U)
  {
    return;
  }

  local_cmd[sizeof(local_cmd) - 1U] = '\0';

  /* Fallback for Wi-Fi PID sync/read path: send a direct UART6 PID line even if
   * the shared printf mirroring path drops command responses under load. */
  if (strcmp(local_cmd, "PID GET") == 0)
  {
    App_GetRatePidGains(&gains);
    pid_line_len = snprintf(pid_line,
                            sizeof(pid_line),
                            "PID[src=get]=[R %.4f %.4f %.4f %.4f P %.4f %.4f %.4f %.4f Y %.4f %.4f %.4f %.4f]\r\n",
                            (double)gains.roll.kp,
                            (double)gains.roll.ki,
                            (double)gains.roll.kd,
                            (double)gains.roll.kff,
                            (double)gains.pitch.kp,
                            (double)gains.pitch.ki,
                            (double)gains.pitch.kd,
                            (double)gains.pitch.kff,
                            (double)gains.yaw.kp,
                            (double)gains.yaw.ki,
                            (double)gains.yaw.kd,
                            (double)gains.yaw.kff);
    if (pid_line_len > 0)
    {
      uint16_t tx_len = (uint16_t)((pid_line_len < (int)sizeof(pid_line)) ? pid_line_len : (int)sizeof(pid_line));
      (void)HAL_UART_Transmit(&huart6, (uint8_t *)pid_line, tx_len, COMM_UART6_TX_TIMEOUT_MS);
    }
  }

  CDC_ProcessCommandLine(local_cmd);
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
