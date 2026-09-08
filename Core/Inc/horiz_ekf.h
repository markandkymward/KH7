#ifndef HORIZ_EKF_H
#define HORIZ_EKF_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void HorizEkf_Init(void);
/* Call from Nav_LatchReference() (nav.c), at the exact same moment the local
 * NED origin re-zeros - the position states below are only ever meaningful
 * relative to that origin, so they must reset in lockstep with it or every
 * consumer of HorizEkf_GetNorthM()/GetEastM() would silently read stale
 * distances measured from the OLD origin. Resets velocity/accel-bias too,
 * same full-reset-on-a-rare-event philosophy as VertEkf_Reset() on arm - this
 * only fires once per genuinely fresh mode engagement now (see
 * navpos_needs_latch in app.c), not on every GPS blip, so treating it as a
 * safe "start fresh" moment is consistent with that fix. */
void HorizEkf_Reset(void);

/* Predict step - call at IMU/control-loop rate, right alongside
 * VertEkf_Predict() for the same sample (same two IMU-processing branches in
 * app.c that already call that).
 * accel_fwd_mps2/accel_right_mps2: Attitude_GetHorizontalAccelBodyYaw()'s
 * output for this same accelerometer sample - tilt-compensated specific
 * force in the vehicle's OWN current-heading frame (forward/right), gravity
 * already removed by construction (it has no component in this frame). This
 * function does the one remaining rotation itself, from that body-yaw frame
 * into true NED, using yaw_deg - the SAME already-mag-corrected heading
 * Nav_RotateBodyToNed() uses everywhere else in this codebase, so this
 * estimator's north/east frame can never silently disagree with NAVPOSHOLD's
 * own stick-to-NED mapping. */
void HorizEkf_Predict(float accel_fwd_mps2, float accel_right_mps2, float yaw_deg, float dt_s);

/* GPS measurement update - call once per fresh, ALREADY-VALIDATED GPS sample
 * (from Nav_Update(), after its existing fix-type/jump/interval checks pass -
 * this file trusts the caller to have already rejected outliers the way
 * VertEkf_UpdateBaro()/UpdateRange() trust their own callers to hand them a
 * sane raw reading).
 * north_m/east_m: local-NED position for this sample (Nav_LatLonToLocalNE()'s
 * output - same reference frame this file's own position states use).
 * vel_n_mps/vel_e_mps: the receiver's native NED velocity for this sample.
 * h_acc_m: GPS_GetHorizontalAccuracyM() - scales this update's position R.
 * s_acc_mps: GPS_GetSpeedAccuracyMps() - scales this update's velocity R;
 * falls back to a fixed default if <= 0 (some receivers/messages don't
 * populate a real speed accuracy). */
void HorizEkf_UpdateGps(float north_m, float east_m, float vel_n_mps, float vel_e_mps,
                         float h_acc_m, float s_acc_mps);

float HorizEkf_GetNorthM(void);
float HorizEkf_GetEastM(void);
float HorizEkf_GetNorthVelMps(void);
float HorizEkf_GetEastVelMps(void);
float HorizEkf_GetAccelBiasNorthMps2(void);
float HorizEkf_GetAccelBiasEastMps2(void);

#ifdef __cplusplus
}
#endif

#endif
