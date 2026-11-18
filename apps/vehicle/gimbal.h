/*
 * 云台 yaw 目标跟踪 PID。
 */

#ifndef STCNAV_VEHICLE_GIMBAL_H
#define STCNAV_VEHICLE_GIMBAL_H

void gimbal_reset(void);
void gimbal_set_target(float yaw);
float gimbal_get_target(void);
void gimbal_set_gains(float kp, float ki, float kd);
void gimbal_get_gains(float *kp, float *ki, float *kd);
void gimbal_set_deadband(float deg);
float gimbal_get_deadband(void);
signed short gimbal_update(signed short right_h, float gimbal_yaw,
                           float gimbal_rate, float dt);

#endif
