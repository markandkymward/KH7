#ifndef NAV_H
#define NAV_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * GPS navigation foundation - Phase 1 (see project roadmap: health/estimation,
 * then velocity brake, then position hold, then RTH, then follow-me).
 *
 * THIS FILE ONLY IMPLEMENTS: GPS health/validity, a local NED reference,
 * north/east position, and filtered north/east velocity. It does NOT
 * implement position hold, return-to-home, or waypoint navigation.
 *
 * Coordinate conventions (documented, not left implicit):
 * - Local tangent-plane frame: north_m/east_m are meters relative to a single
 *   latched reference lat/lon, i.e. a local NED frame with reference altitude
 *   ignored for the horizontal math (small-area flat-Earth approximation -
 *   see Nav_LatLonToLocalNE() for the exact formula and its valid range).
 * - Yaw convention: this module expects yaw_deg in the SAME convention
 *   Attitude_GetBoardAnglesDeg() already produces (clockwise-positive when
 *   viewed from above, consistent with the existing compass-heading-nudge
 *   feature in app.c which compares Mag_GetHeadingDeg() - also clockwise-from-
 *   north - against yaw_deg). Nav_RotateBodyToNed()/Nav_RotateNedToBody() are
 *   the ONLY two places this assumption is encoded (see APP-side bench test:
 *   rotating the aircraft in yaw must brake in the correct direction).
 * - Body frame: FRD (+X forward, +Y right), matching the rest of this
 *   codebase's roll/pitch conventions.
 */

typedef enum
{
  NAV_INVALID_REASON_NONE = 0,
  NAV_INVALID_REASON_NO_3D_FIX,
  NAV_INVALID_REASON_GPS_STALE,
  NAV_INVALID_REASON_BAD_UPDATE_INTERVAL,
  NAV_INVALID_REASON_POOR_ACCURACY,
  NAV_INVALID_REASON_VELOCITY_INVALID,
  NAV_INVALID_REASON_POSITION_JUMP,
  NAV_INVALID_REASON_VELOCITY_JUMP,
  NAV_INVALID_REASON_REACQUIRING,
  NAV_INVALID_REASON_NO_REFERENCE,
  NAV_INVALID_REASON_NONFINITE,
} Nav_InvalidReason_t;

typedef struct
{
  /* Raw receiver fields (mirrors gps.h getters, captured together per Nav_Update() call). */
  uint8_t fix_type;
  uint8_t num_sv;
  float lat_deg;
  float lon_deg;
  float alt_m;
  float h_acc_m;
  float v_acc_m;
  float s_acc_mps;
  float ground_speed_mps;
  float course_deg;
  float vel_n_mps;
  float vel_e_mps;
  float vel_d_mps;
  uint32_t gps_itow_ms;
  uint32_t host_rx_ms;
  uint32_t age_ms;
  uint32_t update_period_ms;

  /* Health/quality bookkeeping. */
  uint32_t consecutive_valid;
  uint32_t consecutive_invalid;
  uint32_t duplicate_count;
  uint32_t rejected_count;
  uint32_t dropout_count;

  /* Local NED reference/position. */
  uint8_t reference_valid;
  float ref_lat_deg;
  float ref_lon_deg;
  float ref_alt_m;
  float north_m;
  float east_m;

  /* Velocity (raw = receiver's native NED velocity; filtered = LPF of raw). */
  float raw_north_vel_mps;
  float raw_east_vel_mps;
  float filtered_north_vel_mps;
  float filtered_east_vel_mps;

  /* Composite result. */
  uint8_t valid;
  Nav_InvalidReason_t invalid_reason;
  uint8_t new_sample; /* true only on the iteration a new GPS sample was accepted */
} Nav_State_t;

/* All thresholds are configurable/named per the safety requirements - see nav.c
 * for defaults and rationale. Exposed here so app.c/telemetry/tests can reference
 * the same names rather than duplicating magic numbers. */
#define NAV_MAX_MESSAGE_AGE_MS         700U
#define NAV_MIN_UPDATE_INTERVAL_MS     50U
#define NAV_MAX_UPDATE_INTERVAL_MS     1500U
#define NAV_MAX_HORIZONTAL_ACC_M       5.0f
#define NAV_MAX_PLAUSIBLE_VEL_MPS      25.0f
#define NAV_MAX_POSITION_JUMP_M        15.0f
#define NAV_MAX_VELOCITY_JUMP_MPS      8.0f
#define NAV_MIN_CONSECUTIVE_VALID      5U
#define NAV_MIN_FIX_TYPE               3U /* 3D fix */
#define NAV_VELOCITY_LPF_HZ            0.5f

void Nav_Init(void);
/* Called every App_Update() iteration. armed/disarmed only affects automatic
 * reference (re)latching, never gates health tracking itself. */
void Nav_Update(uint32_t now_ms, uint8_t motors_armed);
void Nav_GetState(Nav_State_t *out);
uint8_t Nav_IsValid(void);
uint8_t Nav_IsReferenceValid(void);
/* Explicitly latch the local reference at the current position (used on
 * navigation-mode engagement per the safety spec - does not require disarmed). */
void Nav_LatchReference(void);
/* Clears reference_valid so the next disarmed period with a good GPS fix
 * auto-latches a fresh reference (see Nav_Update()'s auto-latch block). Call
 * this on arm->disarm transitions - otherwise a reference latched once per
 * boot session never gets a chance to recover from a bad/borderline latch,
 * and NAV_VELOCITY_BRAKE's arm gate can stay blocked until power-cycled. */
void Nav_ResetReference(void);
const char *Nav_InvalidReasonName(Nav_InvalidReason_t reason);

/* ---- Named, independently-testable transformation functions (see tools/test_nav_math.py) ---- */

/* Local tangent-plane approximation: valid for short operating distances (a few
 * hundred meters to a few km) around the reference point - error grows with
 * distance and with |reference latitude| approaching the poles. Not valid for
 * long-range navigation. */
void Nav_LatLonToLocalNE(float ref_lat_deg, float ref_lon_deg,
                         float lat_deg, float lon_deg,
                         float *north_m, float *east_m);

/* Rotates a body-frame (forward, right) vector into the local NED (north, east)
 * frame using yaw_deg (clockwise-positive-from-above, see file header). */
void Nav_RotateBodyToNed(float fwd, float right, float yaw_deg,
                         float *north, float *east);

/* Inverse of Nav_RotateBodyToNed(): rotates a NED (north, east) vector into
 * body-frame (forward, right) using the same yaw convention. */
void Nav_RotateNedToBody(float north, float east, float yaw_deg,
                         float *fwd, float *right);

/* Small-angle conversion: accel_mps2 (in the direction that would need MORE
 * forward/right tilt to produce, e.g. forward accel -> pitch-style angle) to a
 * tilt angle in degrees using atan2(accel, g), clamped to +-max_angle_deg. */
float Nav_AccelToAngleDeg(float accel_mps2, float max_angle_deg);

#ifdef __cplusplus
}
#endif

#endif
