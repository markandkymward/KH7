/* SD card block driver, SPI mode (regular Kakute H7: SD on SPI1, CS=PA4, DETECT=PA3).
 * Implements the standard SD-over-SPI bring-up (CMD0/CMD8/ACMD41/CMD58) plus
 * single block read/write (CMD17/CMD24). Byte-polled transfers, no DMA - simplicity
 * and correctness over throughput, since logging bandwidth needs here are modest. */
#include "sdcard.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;

#define SD_CS_Pin        GPIO_PIN_4
#define SD_CS_GPIO_Port  GPIOA
#define SD_DETECT_Pin       GPIO_PIN_3
#define SD_DETECT_GPIO_Port GPIOA

#define SD_SPI_TIMEOUT_MS    100U
#define SD_CMD_TIMEOUT_MS    200U
#define SD_WRITE_TIMEOUT_MS  500U
#define SD_ACMD41_TIMEOUT_MS 1000U

/* SPI123 kernel clock is 80MHz (see SPI1.CalculateBaudRate in KH7.ioc). */
#define SD_SPI_PRESCALER_INIT SPI_BAUDRATEPRESCALER_256 /* ~312.5kHz, within SD init spec */
#define SD_SPI_PRESCALER_FAST SPI_BAUDRATEPRESCALER_32  /* ~2.5MHz operating speed */

static uint8_t g_sd_initialized = 0U;
static uint8_t g_sd_high_capacity = 0U;
static uint32_t g_sd_write_busy_start_ms = 0U;

static void SD_GpioInit(void)
{
  GPIO_InitTypeDef gi = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  gi.Pin = SD_CS_Pin;
  gi.Mode = GPIO_MODE_OUTPUT_PP;
  gi.Pull = GPIO_NOPULL;
  gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &gi);
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  gi.Pin = SD_DETECT_Pin;
  gi.Mode = GPIO_MODE_INPUT;
  gi.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SD_DETECT_GPIO_Port, &gi);
}

static void SD_SpiReconfigure(uint32_t prescaler)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = prescaler;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0U;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  (void)HAL_SPI_Init(&hspi1);
}

/* This is the most vibration-exposed SPI peripheral in the firmware (writes
 * every ~30ms during flight, right where mechanical stress is highest), and
 * shares the same STM32 HAL risk already found and fixed on hspi4/hi2c1: a
 * timed-out transfer can leave the peripheral's internal state stuck at BUSY,
 * so every subsequent call fails immediately without ever touching the bus -
 * silently killing SD logging for the rest of the boot with zero effect on
 * flight control (motors/receiver/disarm don't depend on write success).
 * That matches the single most consistent symptom across this investigation:
 * every recovered flight's SD log stops mid-armed partway through, regardless
 * of watchdog state. A DeInit/Init on any failure clears that stuck state so
 * a transient glitch self-heals instead of permanently wedging (2026-08-21). */
static void SD_SpiRecover(void)
{
  (void)HAL_SPI_DeInit(&hspi1);
  SD_SpiReconfigure(hspi1.Init.BaudRatePrescaler);
}

static HAL_StatusTypeDef SD_SpiTxRx(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
  uint8_t tx_fill[32];
  uint8_t rx_discard[32];
  uint16_t offset = 0U;

  memset(tx_fill, 0xFF, sizeof(tx_fill));

  while (offset < len)
  {
    uint16_t chunk_len = (uint16_t)(len - offset);
    HAL_StatusTypeDef status;

    if (chunk_len > sizeof(tx_fill))
    {
      chunk_len = sizeof(tx_fill);
    }

    status = HAL_SPI_TransmitReceive(&hspi1,
                                     (uint8_t *)((tx != NULL) ? &tx[offset] : tx_fill),
                                     (rx != NULL) ? &rx[offset] : rx_discard,
                                     chunk_len,
                                     SD_SPI_TIMEOUT_MS);
    if (status != HAL_OK)
    {
      SD_SpiRecover();
      return status;
    }
    offset = (uint16_t)(offset + chunk_len);
  }

  return HAL_OK;
}

static void SD_Deselect(void)
{
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
  SD_SpiTxRx(NULL, NULL, 1U);
}

/* Sends a command frame and polls for R1 in a single continuous SPI transfer
 * (avoids disabling/re-enabling the SPI peripheral between bytes, which can
 * inject glitches on the H7's SPI block during back-to-back single-byte
 * transfers). If extra_len>0, also returns the extra_len bytes immediately
 * following R1 (for R7/R3 responses like CMD8/CMD58), captured in the same
 * transfer so the bit position stays consistent regardless of which poll
 * iteration R1 was found at. */
static uint8_t SD_SendCmdEx(uint8_t cmd, uint32_t arg, uint8_t crc, uint8_t *extra, uint8_t extra_len)
{
  uint8_t tx[20];
  uint8_t rx[20] = {0};
  uint8_t r1 = 0xFFU;
  uint8_t total = (uint8_t)(16U + extra_len);
  uint8_t found_at = 0xFFU;
  uint8_t i;

  tx[0] = (uint8_t)(0x40U | cmd);
  tx[1] = (uint8_t)(arg >> 24);
  tx[2] = (uint8_t)(arg >> 16);
  tx[3] = (uint8_t)(arg >> 8);
  tx[4] = (uint8_t)(arg);
  tx[5] = crc;
  for (i = 6U; i < total; i++)
  {
    tx[i] = 0xFFU;
  }

  (void)HAL_SPI_TransmitReceive(&hspi1, tx, rx, total, SD_SPI_TIMEOUT_MS);

  for (i = 6U; i < 16U; i++)
  {
    if ((rx[i] & 0x80U) == 0U)
    {
      r1 = rx[i];
      found_at = i;
      break;
    }
  }

  if ((extra != NULL) && (extra_len > 0U))
  {
    for (i = 0U; i < extra_len; i++)
    {
      extra[i] = (found_at != 0xFFU) ? rx[found_at + 1U + i] : 0xFFU;
    }
  }

  return r1;
}

static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
  return SD_SendCmdEx(cmd, arg, crc, NULL, 0U);
}

SD_Status SD_Init(void)
{
  uint8_t r1;
  uint8_t resp[4];
  uint32_t start_ms;
  uint8_t legacy = 0U; /* SDSC v1.x card (no CMD8 support) - byte-addressed, no HCS bit in ACMD41 */

  g_sd_initialized = 0U;
  g_sd_high_capacity = 0U;

  SD_GpioInit();
  SD_SpiReconfigure(SD_SPI_PRESCALER_INIT);

  HAL_Delay(10U);
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
  SD_SpiTxRx(NULL, NULL, 10U); /* >=74 clocks with CS high before first command */

  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  {
    /* CMD0 right after power-up is unreliable on some cards while the bus settles; retry a few times. */
    uint8_t cmd0_tries;
    for (cmd0_tries = 0U; cmd0_tries < 5U; cmd0_tries++)
    {
      r1 = SD_SendCmd(0U, 0x00000000UL, 0x95U);
      if (r1 == 0x01U)
      {
        break;
      }
      SD_Deselect();
      HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
      HAL_Delay(2U);
    }
  }
  if (r1 != 0x01U)
  {
    SD_Deselect();
    return SD_ERR_NO_CARD;
  }

  r1 = SD_SendCmdEx(8U, 0x000001AAUL, 0x87U, resp, 4U);
  if ((r1 & 0x04U) != 0U)
  {
    /* Illegal command: card predates CMD8 (SD spec < v2.00) - fall back to legacy init. */
    legacy = 1U;
  }
  else if ((r1 != 0x01U) || (resp[2] != 0x01U) || (resp[3] != 0xAAU))
  {
    SD_Deselect();
    return SD_ERR_CMD;
  }

  start_ms = HAL_GetTick();
  do
  {
    SD_Deselect();
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
    (void)SD_SendCmd(55U, 0U, 0x65U);
    r1 = SD_SendCmd(41U, legacy ? 0x00000000UL : 0x40000000UL, 0x77U);
    if (r1 == 0x00U)
    {
      break;
    }
    HAL_Delay(2U);
  } while ((HAL_GetTick() - start_ms) < SD_ACMD41_TIMEOUT_MS);

  if (r1 != 0x00U)
  {
    SD_Deselect();
    return SD_ERR_TIMEOUT;
  }

  SD_Deselect();
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
  r1 = SD_SendCmdEx(58U, 0U, 0x01U, resp, 4U);
  if (r1 != 0x00U)
  {
    SD_Deselect();
    return SD_ERR_CMD;
  }
  g_sd_high_capacity = (!legacy && ((resp[0] & 0x40U) != 0U)) ? 1U : 0U;

  SD_Deselect();
  SD_SpiReconfigure(SD_SPI_PRESCALER_FAST);

  g_sd_initialized = 1U;
  return SD_OK;
}

SD_Status SD_ReadBlock(uint32_t block_addr, uint8_t *buf512)
{
  uint8_t r1;
  uint8_t token = 0xFFU;
  uint32_t addr;
  uint32_t start_ms;
  uint8_t crc[2];

  if (g_sd_initialized == 0U)
  {
    return SD_ERR_NOT_INIT;
  }

  addr = (g_sd_high_capacity != 0U) ? block_addr : (block_addr * SD_BLOCK_SIZE);

  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
  r1 = SD_SendCmd(17U, addr, 0x01U);
  if (r1 != 0x00U)
  {
    SD_Deselect();
    return SD_ERR_CMD;
  }

  start_ms = HAL_GetTick();
  do
  {
    SD_SpiTxRx(NULL, &token, 1U);
    if (token == 0xFEU)
    {
      break;
    }
  } while ((HAL_GetTick() - start_ms) < SD_CMD_TIMEOUT_MS);

  if (token != 0xFEU)
  {
    SD_Deselect();
    return SD_ERR_TIMEOUT;
  }

  SD_SpiTxRx(NULL, buf512, (uint16_t)SD_BLOCK_SIZE);
  SD_SpiTxRx(NULL, crc, 2U);

  SD_Deselect();
  return SD_OK;
}

SD_Status SD_WriteBlockBegin(uint32_t block_addr, const uint8_t *buf512)
{
  uint8_t r1;
  uint32_t addr;
  uint8_t token = 0xFEU;
  uint8_t crc[2] = { 0xFFU, 0xFFU };
  uint8_t resp = 0xFFU;

  if (g_sd_initialized == 0U)
  {
    return SD_ERR_NOT_INIT;
  }

  addr = (g_sd_high_capacity != 0U) ? block_addr : (block_addr * SD_BLOCK_SIZE);

  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
  r1 = SD_SendCmd(24U, addr, 0x01U);
  if (r1 != 0x00U)
  {
    SD_Deselect();
    return SD_ERR_CMD;
  }

  if ((SD_SpiTxRx(&token, NULL, 1U) != HAL_OK) ||
      (SD_SpiTxRx(buf512, NULL, (uint16_t)SD_BLOCK_SIZE) != HAL_OK) ||
      (SD_SpiTxRx(crc, NULL, 2U) != HAL_OK))
  {
    SD_Deselect();
    return SD_ERR_TIMEOUT;
  }

  if (SD_SpiTxRx(NULL, &resp, 1U) != HAL_OK)
  {
    SD_Deselect();
    return SD_ERR_TIMEOUT;
  }
  if ((resp & 0x1FU) != 0x05U)
  {
    SD_Deselect();
    return SD_ERR_CMD;
  }

  /* CS stays asserted here - caller must poll SD_WriteBlockPoll() until done. */
  g_sd_write_busy_start_ms = HAL_GetTick();
  return SD_OK;
}

int8_t SD_WriteBlockPoll(void)
{
  uint8_t busy = 0x00U;

  SD_SpiTxRx(NULL, &busy, 1U);
  if (busy == 0xFFU)
  {
    SD_Deselect();
    return 0;
  }
  if ((HAL_GetTick() - g_sd_write_busy_start_ms) >= SD_WRITE_TIMEOUT_MS)
  {
    SD_Deselect();
    return 1;
  }
  return -1;
}

SD_Status SD_WriteBlock(uint32_t block_addr, const uint8_t *buf512)
{
  SD_Status st;
  int8_t poll;

  st = SD_WriteBlockBegin(block_addr, buf512);
  if (st != SD_OK)
  {
    return st;
  }

  do
  {
    poll = SD_WriteBlockPoll();
  } while (poll < 0);

  return (poll == 0) ? SD_OK : SD_ERR_TIMEOUT;
}

uint8_t SD_IsInitialized(void)
{
  return g_sd_initialized;
}

uint8_t SD_IsHighCapacity(void)
{
  return g_sd_high_capacity;
}
