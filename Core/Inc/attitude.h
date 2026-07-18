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

#ifdef __cplusplus
}
#endif

#endif
