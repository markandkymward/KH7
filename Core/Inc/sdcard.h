#ifndef SDCARD_H
#define SDCARD_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  SD_OK = 0,
  SD_ERR_NO_CARD,
  SD_ERR_TIMEOUT,
  SD_ERR_CMD,
  SD_ERR_NOT_INIT
} SD_Status;

#define SD_BLOCK_SIZE 512U

SD_Status SD_Init(void);
SD_Status SD_ReadBlock(uint32_t block_addr, uint8_t *buf512);
SD_Status SD_WriteBlock(uint32_t block_addr, const uint8_t *buf512);
uint8_t SD_IsInitialized(void);
uint8_t SD_IsHighCapacity(void);

#ifdef __cplusplus
}
#endif

#endif
