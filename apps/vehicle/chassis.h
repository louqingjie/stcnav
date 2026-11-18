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

/*
 * V2 上位机速度命令（base 系：+x 前、+y 左；wz 绕 +z 右手）。
 * vx/vy 单位 m/s，wz 单位 rad/s；不参与小陀螺 PI。
 */
void chassis_update_velocity(float vx_mps, float vy_mps, float wz_rad_s,
                             float dt, signed short wheel_out[4]);

#endif
