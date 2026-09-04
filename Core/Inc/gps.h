#ifndef GPS_H
#define GPS_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* HGLRC M100 Pro (u-blox M10-class chipset) on USART3, 115200 baud 8N1 (module's
 * factory default). Module lineage this project has been through: Matek
 * M10Q-5883 at 9600 baud -> SoloGood M10-180C at 115200 (2026-08-19, module
 * failed) -> HGLRC M100 Pro at 115200 (2026-08-21, current - see GPS_Init()'s
 * CFG-PRT comment in gps.c for a quirk specific to this exact module). Same
 * wiring/pins across all three swaps. UBX+NMEA in / UBX-only out. Module has no
 * dataflash - GPS_Init() must be re-run every boot, it is not a one-time
 * factory setting.
 * GPS_Init() also sends UBX-CFG-RATE to raise the nav solution rate from the
 * module's 1Hz default to GPS_NAV_RATE_HZ (see gps.c) - the velocity-brake
 * controller needs fresher-than-1Hz GPS velocity to be usable. */
HAL_StatusTypeDef GPS_Init(void);
/* Manual bench diagnostic - probes common baud rates for a valid checksummed
 * UBX or NMEA frame and leaves huart3 set to whichever one worked (see
 * gps.c). Blocks for ~3s - only call while disarmed. */
void GPS_ScanBaud(void);
/* Manual bench diagnostic - broadcasts a UBX-CFG-CFG clear-to-default +
 * UBX-CFG-RST cold-start across the same baud candidates GPS_ScanBaud() uses
 * (see gps.c). Cannot confirm the module received it without a working RX
 * path - follow up with GPS_ScanBaud() to check for a response. */
void GPS_FactoryReset(void);
uint8_t GPS_IsConfigured(void);
/* Per-stage ACK result from the most recent GPS_Init() attempt, for diagnostics. */
uint8_t GPS_GetLastPrtAcked(void);
uint8_t GPS_GetLastMsgAcked(void);
uint8_t GPS_GetLastRateAcked(void);
/* ACK result for UBX-CFG-NAV5 (dynamic platform model set to Airborne <1g,
 * 2026-09-04) - see GPS_Init()'s comment for why this was added: the module's
 * factory-default "Portable" dynamic model applies static-hold position
 * clamping at low speed, which was found (via cross-session flight telemetry
 * analysis) to freeze the reported GPS position for over half of all flight
 * time with no correlation to motor power, right when position-hold most needs
 * honest position feedback. */
uint8_t GPS_GetLastNav5Acked(void);
/* Configured AND a valid UBX frame has been seen recently (live comms watchdog). */
uint8_t GPS_IsHealthy(void);
uint8_t GPS_HasFix(void);
uint8_t GPS_GetFixType(void);
uint8_t GPS_GetNumSatellites(void);
float GPS_GetLatitudeDeg(void);
float GPS_GetLongitudeDeg(void);
/* Height above MEAN SEA LEVEL (UBX NAV-PVT hMSL, offset 36), NOT height above the
 * WGS84 ellipsoid despite this getter's name predating that distinction being
 * documented - "altitude" in this project has always meant hMSL in practice (baro
 * altitude is also MSL-referenced, so this was the consistent choice, just not the
 * geodetically precise one the name implies). See GPS_GetAltitudeEllipsoidM() below
 * for the true ellipsoid height (hMSL and ellipsoid height differ by tens of meters
 * depending on location - do not mix the two in the same calculation). */
float GPS_GetAltitudeM(void);
/* True height above the WGS84 ellipsoid (UBX NAV-PVT height, offset 32) - added
 * 2026-08-30 alongside the comment fix above. Not used by anything in this project
 * yet (baro is the real altitude reference project-wide); exposed because the byte
 * offset sits immediately before hMSL in the same already-parsed message. */
float GPS_GetAltitudeEllipsoidM(void);
uint32_t GPS_GetLastFixAgeMs(void);

/* Horizontal/vertical/speed accuracy estimates reported by the receiver itself
 * (UBX NAV-PVT hAcc/vAcc/sAcc), meters and m/s. Always "available" on u-blox
 * (no separate valid flag in the message) but can be large/meaningless with a
 * poor fix - callers should gate on fix type/sat count too, not accuracy alone. */
float GPS_GetHorizontalAccuracyM(void);
float GPS_GetVerticalAccuracyM(void);
float GPS_GetSpeedAccuracyMps(void);
/* Receiver's own fix-confidence flag (UBX NAV-PVT flags bit 0, gnssFixOK) - added
 * 2026-08-30. More authoritative than fixType alone: fixType can report a 3D fix
 * during a marginal/transient solution the receiver itself doesn't yet trust.
 * Callers wanting a strict "is this fix genuinely good" check should use this
 * ANDed with fixType/GPS_HasFix(), not either alone. */
uint8_t GPS_GetGnssFixOk(void);
/* Position dilution of precision (UBX NAV-PVT pDOP, offset 76, scale 0.01) - added
 * 2026-08-30. A pure satellite-geometry quality metric, independent of the
 * receiver's own hAcc error estimate - standard practice is to gate on both
 * together, since hAcc can be overconfident in poor geometry. Lower is better;
 * u-blox's own guidance treats <2.5 as excellent, <5 as good, >10 as poor. Defaults
 * to 99.99 (worst-case-looking) until a real PVT has been decoded. */
float GPS_GetPdop(void);
float GPS_GetGroundSpeedMps(void);
/* Course over ground, degrees, 0=north/360, clockwise-positive (compass convention),
 * per UBX NAV-PVT headMot. Only meaningful above a few m/s of ground speed. */
float GPS_GetCourseDeg(void);
/* Accuracy estimate for GPS_GetCourseDeg() above (UBX NAV-PVT headAcc, offset 72,
 * same 1e-5 deg scaling as headMot) - added 2026-08-30 specifically so GPS course
 * can be gated for trust the same way every other value/accuracy pair in this file
 * already is (hAcc, vAcc, sAcc). Opens the door to using GPS course-over-ground as
 * an independent cross-check against the compass/AHRS yaw estimate once moving -
 * not wired into anything yet, just exposed. */
float GPS_GetHeadingAccuracyDeg(void);
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
