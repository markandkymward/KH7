#ifndef GPS_H
#define GPS_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Matek M10Q-5883 (u-blox SAM-M10Q) on USART3, 9600 baud 8N1, reconfigured to
 * UBX-only output (NAV-PVT). Module has no dataflash - GPS_Init() must be
 * re-run every boot, it is not a one-time factory setting.
 * GPS_Init() also sends UBX-CFG-RATE to raise the nav solution rate from the
 * module's 1Hz default to GPS_NAV_RATE_HZ (see gps.c) - the velocity-brake
 * controller needs fresher-than-1Hz GPS velocity to be usable. */
HAL_StatusTypeDef GPS_Init(void);
uint8_t GPS_IsConfigured(void);
/* Per-stage ACK result from the most recent GPS_Init() attempt, for diagnostics. */
uint8_t GPS_GetLastPrtAcked(void);
uint8_t GPS_GetLastMsgAcked(void);
uint8_t GPS_GetLastRateAcked(void);
/* Configured AND a valid UBX frame has been seen recently (live comms watchdog). */
uint8_t GPS_IsHealthy(void);
uint8_t GPS_HasFix(void);
uint8_t GPS_GetFixType(void);
uint8_t GPS_GetNumSatellites(void);
float GPS_GetLatitudeDeg(void);
float GPS_GetLongitudeDeg(void);
float GPS_GetAltitudeM(void);
uint32_t GPS_GetLastFixAgeMs(void);

/* Horizontal/vertical/speed accuracy estimates reported by the receiver itself
 * (UBX NAV-PVT hAcc/vAcc/sAcc), meters and m/s. Always "available" on u-blox
 * (no separate valid flag in the message) but can be large/meaningless with a
 * poor fix - callers should gate on fix type/sat count too, not accuracy alone. */
float GPS_GetHorizontalAccuracyM(void);
float GPS_GetVerticalAccuracyM(void);
float GPS_GetSpeedAccuracyMps(void);
float GPS_GetGroundSpeedMps(void);
/* Course over ground, degrees, 0=north/360, clockwise-positive (compass convention),
 * per UBX NAV-PVT headMot. Only meaningful above a few m/s of ground speed. */
float GPS_GetCourseDeg(void);
/* Native NED velocity from UBX NAV-PVT velN/velE/velD, m/s. This is the receiver's
 * own Doppler-derived velocity solution - preferred over differentiating position. */
float GPS_GetVelNorthMps(void);
float GPS_GetVelEastMps(void);
float GPS_GetVelDownMps(void);
/* GPS receiver's own time-of-week for the most recent NAV-PVT, milliseconds
 * (UBX iTOW). Wraps weekly - only use for detecting duplicate/repeated
 * solutions and measuring inter-sample deltas, never as an absolute clock. */
uint32_t GPS_GetLastITowMs(void);
/* Host (HAL_GetTick()) timestamp of the most recent NAV-PVT decode. */
uint32_t GPS_GetLastPvtHostMs(void);
/* Increments once per NAV-PVT payload successfully decoded - lets callers detect
 * "a new GPS sample arrived" without relying on value comparison (position/velocity
 * can legitimately repeat, e.g. a stationary receiver). */
uint32_t GPS_GetPvtUpdateCount(void);

/* Called from HAL_UART_RxCpltCallback/HAL_UART_ErrorCallback for USART3. */
void GPS_UartRxCpltCallback(void);
void GPS_UartErrorCallback(void);

#ifdef __cplusplus
}
#endif

#endif
