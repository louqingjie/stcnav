/*
 * 双 MPU6050 读取：底盘姿态 + 云台单轴 yaw。
 *
 * 底盘 IMU (0x68)：14 字节 burst（accel + temp + gyro）。
 *   - 倾角：加速度计重力方向 -> 最短弧四元数（无持续 Mahony 修正，节省 flash）；
 *   - 航向：陀螺 Z（去零偏）积分；四元数 = 航向四元数 (x) 倾角四元数。
 *   参考系为上电时 yaw=0、重力对齐的本地系；无磁力计，yaw 会缓慢漂移。
 * 云台 IMU (0x69)：只读 GYRO_ZOUT，积分得到相对 yaw。
 *
 * 启动时两个传感器各采样 200 次计算陀螺 Z 零偏，期间保持静止。
 * 控制用接口仍是 chassis/gimbal yaw/rate（度、度每秒）。
 */

#ifndef STCNAV_VEHICLE_IMU_H
#define STCNAV_VEHICLE_IMU_H

#if defined(__SDCC_mcs251)
#define IMU_XDATA __xdata
#else
#define IMU_XDATA
#endif

/*
 * 供 V2 协议 0x88/0x85/0x82 使用的姿态快照。
 * 单位：quaternion 无量纲；gyro 为 rad/s（原始测量，未做 x/y 零偏补偿）；
 * accel 为 m/s^2（specific force，保留重力响应，静止且 +z 朝上时约 +9.81）。
 */
typedef struct
{
    unsigned long chassis_sample_sequence;
    unsigned long gimbal_sample_sequence;
    unsigned long sample_time_ms;
    unsigned int epoch;
    unsigned char valid;
} imu_state_t;

extern imu_state_t IMU_XDATA imu_snapshot;

unsigned char imu_init(void);
unsigned char imu_update(float dt);
float imu_get_chassis_yaw(void);
float imu_get_gimbal_yaw(void);
float imu_get_chassis_rate(void);
float imu_get_gimbal_rate(void);

#endif
