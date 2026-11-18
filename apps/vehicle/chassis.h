/*
 * 四全向轮 X 型运动学与场地坐标系平移。
 */

#ifndef STCNAV_VEHICLE_CHASSIS_H
#define STCNAV_VEHICLE_CHASSIS_H

void chassis_reset_spin(void);
void chassis_stop(void);
void chassis_update(signed short left_v, signed short left_h,
                    float chassis_yaw, float gimbal_yaw,
                    float chassis_rate, unsigned char spin_mode,
                    float dt, signed short wheel_out[4]);

#endif
