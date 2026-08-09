#ifndef GPS_H
#define GPS_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Matek M10Q-5883 (u-blox SAM-M10Q) on USART3, 9600 baud 8N1, reconfigured to
 * UBX-only output (NAV-PVT). Module has no dataflash - GPS_Init() must be
 * re-run every boot, it is not a one-time factory setting. */
HAL_StatusTypeDef GPS_Init(void);
uint8_t GPS_IsConfigured(void);
/* Per-stage ACK result from the most recent GPS_Init() attempt, for diagnostics. */
uint8_t GPS_GetLastPrtAcked(void);
uint8_t GPS_GetLastMsgAcked(void);
/* Configured AND a valid UBX frame has been seen recently (live comms watchdog). */
uint8_t GPS_IsHealthy(void);
uint8_t GPS_HasFix(void);
uint8_t GPS_GetFixType(void);
uint8_t GPS_GetNumSatellites(void);
float GPS_GetLatitudeDeg(void);
float GPS_GetLongitudeDeg(void);
float GPS_GetAltitudeM(void);
uint32_t GPS_GetLastFixAgeMs(void);

/* Called from HAL_UART_RxCpltCallback/HAL_UART_ErrorCallback for USART3. */
void GPS_UartRxCpltCallback(void);
void GPS_UartErrorCallback(void);

#ifdef __cplusplus
}
#endif

#endif
