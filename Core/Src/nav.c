#include "nav.h"

#include "gps.h"

#include <math.h>
#include <string.h>

#define NAV_DEG_PER_RAD   57.29577951f
#define NAV_RAD_PER_DEG   0.0174532925f
#define NAV_EARTH_RADIUS_M 6371000.0f
#define NAV_STANDARD_GRAVITY_MPS2 9.80665f

static Nav_State_t g_nav;
static uint32_t g_last_seen_update_count = 0U;
static uint8_t g_have_seen_update = 0U;
static uint32_t g_prev_host_rx_ms = 0U;
static uint32_t g_prev_itow_ms = 0U;
static float g_prev_north_m = 0.0f;
static float g_prev_east_m = 0.0f;
static uint8_t g_have_prev_position = 0U;
static uint8_t g_was_valid_prev = 0U;

static uint8_t Nav_IsFiniteF(float v)
{
  return (uint8_t)(isfinite(v) ? 1U : 0U);
}

void Nav_LatLonToLocalNE(float ref_lat_deg, float ref_lon_deg,
                         float lat_deg, float lon_deg,
                         float *north_m, float *east_m)
{
  float dlat_rad = (lat_deg - ref_lat_deg) * NAV_RAD_PER_DEG;
  float dlon_rad = (lon_deg - ref_lon_deg) * NAV_RAD_PER_DEG;
  float ref_lat_rad = ref_lat_deg * NAV_RAD_PER_DEG;

  if (north_m != NULL)
  {
    *north_m = dlat_rad * NAV_EARTH_RADIUS_M;
  }
  if (east_m != NULL)
  {
    *east_m = dlon_rad * NAV_EARTH_RADIUS_M * cosf(ref_lat_rad);
  }
}

void Nav_RotateBodyToNed(float fwd, float right, float yaw_deg,
                         float *north, float *east)
{
  float yaw_rad = yaw_deg * NAV_RAD_PER_DEG;
  float c = cosf(yaw_rad);
  float s = sinf(yaw_rad);

  if (north != NULL)
  {
    *north = (fwd * c) - (right * s);
  }
  if (east != NULL)
  {
    *east = (fwd * s) + (right * c);
  }
}

void Nav_RotateNedToBody(float north, float east, float yaw_deg,
                         float *fwd, float *right)
{
  float yaw_rad = yaw_deg * NAV_RAD_PER_DEG;
  float c = cosf(yaw_rad);
  float s = sinf(yaw_rad);

  if (fwd != NULL)
  {
    *fwd = (north * c) + (east * s);
  }
  if (right != NULL)
  {
    *right = (-north * s) + (east * c);
  }
}

float Nav_AccelToAngleDeg(float accel_mps2, float max_angle_deg)
{
  float angle_deg;

  if (Nav_IsFiniteF(accel_mps2) == 0U)
  {
    return 0.0f;
  }

  angle_deg = atan2f(accel_mps2, NAV_STANDARD_GRAVITY_MPS2) * NAV_DEG_PER_RAD;

  if (angle_deg > max_angle_deg)
  {
    angle_deg = max_angle_deg;
  }
  else if (angle_deg < -max_angle_deg)
  {
    angle_deg = -max_angle_deg;
  }

  return angle_deg;
}

const char *Nav_InvalidReasonName(Nav_InvalidReason_t reason)
{
  switch (reason)
  {
    case NAV_INVALID_REASON_NO_3D_FIX:        return "NO_3D_FIX";
    case NAV_INVALID_REASON_GPS_STALE:        return "GPS_STALE";
    case NAV_INVALID_REASON_BAD_UPDATE_INTERVAL: return "BAD_UPDATE_INTERVAL";
    case NAV_INVALID_REASON_POOR_ACCURACY:    return "POOR_ACCURACY";
    case NAV_INVALID_REASON_VELOCITY_INVALID: return "VELOCITY_INVALID";
    case NAV_INVALID_REASON_POSITION_JUMP:    return "POSITION_JUMP";
    case NAV_INVALID_REASON_VELOCITY_JUMP:    return "VELOCITY_JUMP";
    case NAV_INVALID_REASON_REACQUIRING:      return "REACQUIRING";
    case NAV_INVALID_REASON_NO_REFERENCE:     return "NO_REFERENCE";
    case NAV_INVALID_REASON_NONFINITE:        return "NONFINITE";
    case NAV_INVALID_REASON_NONE:
    default:
      return "NONE";
  }
}

void Nav_Init(void)
{
  memset(&g_nav, 0, sizeof(g_nav));
  g_nav.invalid_reason = NAV_INVALID_REASON_NO_3D_FIX;
  g_last_seen_update_count = 0U;
  g_have_seen_update = 0U;
  g_prev_host_rx_ms = 0U;
  g_prev_itow_ms = 0U;
  g_have_prev_position = 0U;
  g_was_valid_prev = 0U;
}

void Nav_LatchReference(void)
{
  if (GPS_GetFixType() < NAV_MIN_FIX_TYPE)
  {
    return;
  }

  g_nav.ref_lat_deg = GPS_GetLatitudeDeg();
  g_nav.ref_lon_deg = GPS_GetLongitudeDeg();
  g_nav.ref_alt_m = GPS_GetAltitudeM();
  g_nav.reference_valid = 1U;
  g_nav.north_m = 0.0f;
  g_nav.east_m = 0.0f;
  g_nav.filtered_north_m = 0.0f;
  g_nav.filtered_east_m = 0.0f;
  g_have_prev_position = 0U;
}

/* Central "is this candidate sample sane" gate - a single function per the
 * safety spec, used both to accept/reject a fresh GPS sample and (separately,
 * below) to decide overall Nav_IsValid() authority. */
static uint8_t Nav_SampleLooksValid(uint8_t fix_type, float lat_deg, float lon_deg,
                                    float h_acc_m, float vel_n_mps, float vel_e_mps,
                                    Nav_InvalidReason_t *reason_out)
{
  float vel_mag_sq;

  if ((Nav_IsFiniteF(lat_deg) == 0U) || (Nav_IsFiniteF(lon_deg) == 0U) ||
      (Nav_IsFiniteF(vel_n_mps) == 0U) || (Nav_IsFiniteF(vel_e_mps) == 0U) ||
      (Nav_IsFiniteF(h_acc_m) == 0U))
  {
    *reason_out = NAV_INVALID_REASON_NONFINITE;
    return 0U;
  }

  if (fix_type < NAV_MIN_FIX_TYPE)
  {
    *reason_out = NAV_INVALID_REASON_NO_3D_FIX;
    return 0U;
  }

  vel_mag_sq = (vel_n_mps * vel_n_mps) + (vel_e_mps * vel_e_mps);
  if (vel_mag_sq > (NAV_MAX_PLAUSIBLE_VEL_MPS * NAV_MAX_PLAUSIBLE_VEL_MPS))
  {
    *reason_out = NAV_INVALID_REASON_VELOCITY_INVALID;
    return 0U;
  }

  return 1U;
}

void Nav_Update(uint32_t now_ms, uint8_t motors_armed)
{
  uint32_t update_count = GPS_GetPvtUpdateCount();
  uint8_t fix_type = GPS_GetFixType();
  float lat_deg = GPS_GetLatitudeDeg();
  float lon_deg = GPS_GetLongitudeDeg();
  float alt_m = GPS_GetAltitudeM();
  float h_acc_m = GPS_GetHorizontalAccuracyM();
  float v_acc_m = GPS_GetVerticalAccuracyM();
  float s_acc_mps = GPS_GetSpeedAccuracyMps();
  float ground_speed_mps = GPS_GetGroundSpeedMps();
  float course_deg = GPS_GetCourseDeg();
  float vel_n_mps = GPS_GetVelNorthMps();
  float vel_e_mps = GPS_GetVelEastMps();
  float vel_d_mps = GPS_GetVelDownMps();
  uint32_t itow_ms = GPS_GetLastITowMs();
  uint32_t host_rx_ms = GPS_GetLastPvtHostMs();
  Nav_InvalidReason_t sample_reject_reason = NAV_INVALID_REASON_NONE;

  g_nav.fix_type = fix_type;
  g_nav.num_sv = GPS_GetNumSatellites();
  g_nav.lat_deg = lat_deg;
  g_nav.lon_deg = lon_deg;
  g_nav.alt_m = alt_m;
  g_nav.h_acc_m = h_acc_m;
  g_nav.v_acc_m = v_acc_m;
  g_nav.s_acc_mps = s_acc_mps;
  g_nav.ground_speed_mps = ground_speed_mps;
  g_nav.course_deg = course_deg;
  g_nav.gps_itow_ms = itow_ms;
  g_nav.host_rx_ms = host_rx_ms;
  g_nav.new_sample = 0U;

  if (g_have_seen_update == 0U)
  {
    g_last_seen_update_count = update_count;
    g_have_seen_update = 1U;
    g_prev_host_rx_ms = host_rx_ms;
    g_prev_itow_ms = itow_ms;
  }
  else if (update_count != g_last_seen_update_count)
  {
    uint8_t is_duplicate = (uint8_t)((itow_ms == g_prev_itow_ms) ? 1U : 0U);
    uint32_t update_period_ms = host_rx_ms - g_prev_host_rx_ms;

    g_last_seen_update_count = update_count;

    if (is_duplicate != 0U)
    {
      g_nav.duplicate_count++;
    }
    else
    {
      uint8_t sample_ok = Nav_SampleLooksValid(fix_type, lat_deg, lon_deg, h_acc_m,
                                               vel_n_mps, vel_e_mps, &sample_reject_reason);
      uint8_t interval_ok = (uint8_t)((update_period_ms >= NAV_MIN_UPDATE_INTERVAL_MS) &&
                                      (update_period_ms <= NAV_MAX_UPDATE_INTERVAL_MS));

      if (sample_ok != 0U && interval_ok == 0U)
      {
        sample_ok = 0U;
        sample_reject_reason = NAV_INVALID_REASON_BAD_UPDATE_INTERVAL;
      }

      if (sample_ok != 0U && g_nav.reference_valid != 0U)
      {
        float north_m;
        float east_m;

        Nav_LatLonToLocalNE(g_nav.ref_lat_deg, g_nav.ref_lon_deg, lat_deg, lon_deg,
                            &north_m, &east_m);

        if (g_have_prev_position != 0U)
        {
          float dn = north_m - g_prev_north_m;
          float de = east_m - g_prev_east_m;

          if (((dn * dn) + (de * de)) > (NAV_MAX_POSITION_JUMP_M * NAV_MAX_POSITION_JUMP_M))
          {
            sample_ok = 0U;
            sample_reject_reason = NAV_INVALID_REASON_POSITION_JUMP;
          }
        }

        if (sample_ok != 0U)
        {
          float dvn = vel_n_mps - g_nav.raw_north_vel_mps;
          float dve = vel_e_mps - g_nav.raw_east_vel_mps;

          if (((dvn * dvn) + (dve * dve)) > (NAV_MAX_VELOCITY_JUMP_MPS * NAV_MAX_VELOCITY_JUMP_MPS))
          {
            sample_ok = 0U;
            sample_reject_reason = NAV_INVALID_REASON_VELOCITY_JUMP;
          }
        }

        if (sample_ok != 0U)
        {
          float dt_s = ((float)update_period_ms) * 0.001f;

          g_nav.north_m = north_m;
          g_nav.east_m = east_m;
          g_prev_north_m = north_m;
          g_prev_east_m = east_m;
          g_have_prev_position = 1U;

          g_nav.raw_north_vel_mps = vel_n_mps;
          g_nav.raw_east_vel_mps = vel_e_mps;
          g_nav.vel_n_mps = vel_n_mps;
          g_nav.vel_e_mps = vel_e_mps;
          g_nav.vel_d_mps = vel_d_mps;

          if (g_nav.consecutive_valid == 0U)
          {
            /* First sample after init/reacquisition: seed the filters directly,
             * no transient from whatever stale value was there before. */
            g_nav.filtered_north_vel_mps = vel_n_mps;
            g_nav.filtered_east_vel_mps = vel_e_mps;
            g_nav.filtered_north_m = north_m;
            g_nav.filtered_east_m = east_m;
          }
          else
          {
            float vel_tau_s = 1.0f / (2.0f * 3.14159265f * NAV_VELOCITY_LPF_HZ);
            float vel_alpha = 1.0f - expf(-dt_s / vel_tau_s);
            float pos_tau_s = 1.0f / (2.0f * 3.14159265f * NAV_POSITION_LPF_HZ);
            float pos_alpha = 1.0f - expf(-dt_s / pos_tau_s);

            g_nav.filtered_north_vel_mps += vel_alpha * (vel_n_mps - g_nav.filtered_north_vel_mps);
            g_nav.filtered_east_vel_mps += vel_alpha * (vel_e_mps - g_nav.filtered_east_vel_mps);
            g_nav.filtered_north_m += pos_alpha * (north_m - g_nav.filtered_north_m);
            g_nav.filtered_east_m += pos_alpha * (east_m - g_nav.filtered_east_m);
          }
        }
      }
      else if (sample_ok != 0U)
      {
        /* Reference not latched yet - still track raw velocity/health, just no
         * north_m/east_m position yet (stays at whatever it last was, 0 if never
         * referenced). */
        g_nav.raw_north_vel_mps = vel_n_mps;
        g_nav.raw_east_vel_mps = vel_e_mps;
        g_nav.vel_n_mps = vel_n_mps;
        g_nav.vel_e_mps = vel_e_mps;
        g_nav.vel_d_mps = vel_d_mps;

        if (g_nav.consecutive_valid == 0U)
        {
          g_nav.filtered_north_vel_mps = vel_n_mps;
          g_nav.filtered_east_vel_mps = vel_e_mps;
        }
        sample_reject_reason = NAV_INVALID_REASON_NO_REFERENCE;
      }

      if (sample_ok != 0U)
      {
        g_nav.consecutive_valid++;
        g_nav.consecutive_invalid = 0U;
        g_nav.new_sample = 1U;
      }
      else
      {
        g_nav.rejected_count++;
        g_nav.consecutive_invalid++;
        g_nav.consecutive_valid = 0U;
      }
    }

    g_prev_host_rx_ms = host_rx_ms;
    g_prev_itow_ms = itow_ms;
    g_nav.update_period_ms = update_period_ms;
  }

  /* GPS reception is interrupt-driven and can update host_rx_ms after App_Update()
   * sampled its loop timestamp. Refresh the clock after capturing the GPS fields
   * so subtracting a newer ISR timestamp cannot underflow into a false stale age. */
  now_ms = HAL_GetTick();

  /* Age/staleness grows every iteration regardless of whether a new sample arrived. */
  g_nav.age_ms = (g_have_seen_update != 0U) ? (now_ms - host_rx_ms) : 0xFFFFFFFFU;

  /* Auto-latch the reference whenever GPS is already solid enough, regardless of
   * arm state - originally disarmed-only (mirroring the gyro-bias/attitude-zero
   * "settle on the ground" pattern), but that created a real deadlock (found
   * 2026-08-30 from an actual flight): NAV_VELOCITY_BRAKE's own engagement path
   * only calls Nav_LatchReference() from INSIDE its "nav_engage_ok" branch, and
   * nav_engage_ok itself requires reference_valid - so if the reference hadn't
   * latched yet by the moment the pilot armed (GPS just a little slow), NOTHING
   * could ever latch it for the rest of that arm cycle: not the disarmed-only
   * path (already armed), not NAVBRAKE's own call (gated behind the very
   * reference it would set). A real flight hit exactly this - GPS never went
   * valid the entire ~56s flight (bounced between POOR_ACCURACY/NO_REFERENCE/
   * GPS_STALE) and only cleared a moment after disarming, leaving both NAVBRAKE
   * and position-hold completely unavailable that whole flight. Removing the
   * arm-state restriction lets the reference latch mid-flight the moment GPS
   * genuinely becomes solid, exactly like it already would if the pilot had
   * disarmed and rearmed - same accuracy/fix-quality bar, just not blocked on
   * arm state. Rebasing north_m/east_m to 0 mid-flight is safe: NAVBRAKE's
   * velocity brake never depended on absolute position, and position-hold
   * (which does) only ever holds wherever it happens to latch a target anyway. */
  if ((g_nav.reference_valid == 0U) &&
      (fix_type >= NAV_MIN_FIX_TYPE) && (h_acc_m <= NAV_MAX_HORIZONTAL_ACC_M) &&
      (g_nav.consecutive_valid >= NAV_MIN_CONSECUTIVE_VALID))
  {
    Nav_LatchReference();
  }

  /* ---- Single central validity decision (App_Update/telemetry/tests all use this same result). ---- */
  if ((Nav_IsFiniteF(lat_deg) == 0U) || (Nav_IsFiniteF(lon_deg) == 0U) ||
      (Nav_IsFiniteF(g_nav.filtered_north_vel_mps) == 0U) || (Nav_IsFiniteF(g_nav.filtered_east_vel_mps) == 0U))
  {
    g_nav.valid = 0U;
    g_nav.invalid_reason = NAV_INVALID_REASON_NONFINITE;
  }
  else if (fix_type < NAV_MIN_FIX_TYPE)
  {
    g_nav.valid = 0U;
    g_nav.invalid_reason = NAV_INVALID_REASON_NO_3D_FIX;
  }
  else if (g_nav.age_ms > NAV_MAX_MESSAGE_AGE_MS)
  {
    g_nav.valid = 0U;
    g_nav.invalid_reason = NAV_INVALID_REASON_GPS_STALE;
    /* Only force a full reacquisition (consecutive_valid=0, up to
     * NAV_MIN_CONSECUTIVE_VALID more samples of REACQUIRING once fresh data
     * resumes) once staleness has genuinely persisted past NAV_STALE_RESET_MS,
     * not on every single brief gap past NAV_MAX_MESSAGE_AGE_MS (found
     * 2026-08-30: a 120s stationary bench capture showed GPS 100% valid the
     * whole time, but real flights showed frequent brief GPS_STALE/REACQUIRING
     * cycling - motor-current interference causing isolated, self-recovering
     * sample gaps, not a real/sustained GPS problem). g_nav.valid still drops
     * immediately at NAV_MAX_MESSAGE_AGE_MS either way - this only avoids
     * piling on an extra ~1s reacquisition penalty for a gap that was already
     * over by the very next sample. */
    if (g_nav.age_ms > NAV_STALE_RESET_MS)
    {
      g_nav.consecutive_valid = 0U;
    }
  }
  else if (h_acc_m > NAV_MAX_HORIZONTAL_ACC_M)
  {
    g_nav.valid = 0U;
    g_nav.invalid_reason = NAV_INVALID_REASON_POOR_ACCURACY;
  }
  else if (g_nav.reference_valid == 0U)
  {
    g_nav.valid = 0U;
    g_nav.invalid_reason = NAV_INVALID_REASON_NO_REFERENCE;
  }
  else if (sample_reject_reason != NAV_INVALID_REASON_NONE)
  {
    /* Most specific/actionable reason first - a jump/interval rejection on
     * this exact iteration is more useful for logging than the generic
     * "reacquiring" state it also causes via consecutive_valid reset below. */
    g_nav.valid = 0U;
    g_nav.invalid_reason = sample_reject_reason;
  }
  else if (g_nav.consecutive_valid < NAV_MIN_CONSECUTIVE_VALID)
  {
    g_nav.valid = 0U;
    g_nav.invalid_reason = NAV_INVALID_REASON_REACQUIRING;
  }
  else
  {
    g_nav.valid = 1U;
    g_nav.invalid_reason = NAV_INVALID_REASON_NONE;
  }

  /* Edge-triggered: count exactly one dropout per valid->invalid transition,
   * regardless of which check tripped it. */
  if ((g_was_valid_prev != 0U) && (g_nav.valid == 0U))
  {
    g_nav.dropout_count++;
  }
  g_was_valid_prev = g_nav.valid;
}

void Nav_GetState(Nav_State_t *out)
{
  if (out != NULL)
  {
    *out = g_nav;
  }
}

uint8_t Nav_IsValid(void)
{
  return g_nav.valid;
}

uint8_t Nav_IsReferenceValid(void)
{
  return g_nav.reference_valid;
}

void Nav_ResetReference(void)
{
  g_nav.reference_valid = 0U;
}
