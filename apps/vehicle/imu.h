/*
 * 双 MPU6050 yaw 读取与陀螺零偏校准。
 * 底盘 IMU: 0x68，云台 IMU: 0x69。
 */

#ifndef STCNAV_VEHICLE_IMU_H
#define STCNAV_VEHICLE_IMU_H

unsigned char imu_init(void);
unsigned char imu_update(float dt);
float imu_get_chassis_yaw(void);
float imu_get_gimbal_yaw(void);
float imu_get_chassis_rate(void);
float imu_get_gimbal_rate(void);
unsigned char imu_fault(void);

#endif
