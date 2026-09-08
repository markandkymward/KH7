#ifndef ATTITUDE_H
#define ATTITUDE_H

#ifdef __cplusplus
extern "C" {
#endif

void Attitude_Init(void);
void Attitude_UpdateIMU(float gx_rad_s,
                        float gy_rad_s,
                        float gz_rad_s,
                        float ax_g,
                        float ay_g,
                        float az_g,
                        float dt_s);
void Attitude_GetBoardAnglesDeg(float *pitch_deg,
                                float *roll_deg,
                                float *yaw_deg);
float Attitude_WrapAngle180(float angle_deg);
/* Net (gravity-removed) earth-frame vertical acceleration in m/s^2, positive
 * = accelerating upward - see the doc comment at its definition in
 * attitude.c. Uses the CURRENT AHRS attitude estimate internally, so call
 * this after Attitude_UpdateIMU() for the same sample. */
float Attitude_GetVerticalAccelMps2(float ax_g, float ay_g, float az_g);
/* World-up (earth +Z) unit vector expressed in body-frame coordinates - see the
 * doc comment at its definition in attitude.c for what it's for (dot it with
 * any body-frame vector to get that vector's world-up component). */
void Attitude_GetWorldUpInBodyFrame(float *gx, float *gy, float *gz);
/* Tilt-compensated horizontal specific force, in m/s^2, expressed in the
 * vehicle's OWN current-heading frame (forward/right) - NOT true north/east
 * (this deliberately does not touch yaw at all; see the doc comment at its
 * definition in attitude.c for why, and Nav_RotateBodyToNed() for the
 * separate step that rotates this into true NED using the already-mag-
 * corrected yaw_deg). Feeds HorizEkf_Predict(). */
void Attitude_GetHorizontalAccelBodyYaw(float ax_g, float ay_g, float az_g,
                                        float *accel_fwd_mps2, float *accel_right_mps2);

#ifdef __cplusplus
}
#endif

#endif
