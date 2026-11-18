/*
 * 整车参数：引脚、遥控通道、IMU 地址、控制周期、速度/自转/云台 PID、方向符号。
 * 所有需要上板调整的量集中在这里。
 */

#ifndef STCNAV_VEHICLE_CONFIG_H
#define STCNAV_VEHICLE_CONFIG_H

#ifndef VEHICLE_DEBUG
#define VEHICLE_DEBUG 0
#endif

/* ---------- NRF24L01 引脚（PIE-Block 接线） ---------- */
/* CE=P3.4, CSN=P2.2, SCK=P2.5, MOSI=P2.3, MISO=P2.4 */
#define NRF_CE_MASK 0x10
#define NRF_CSN_MASK 0x04
#define NRF_SCK_MASK 0x20
#define NRF_MOSI_MASK 0x08
#define NRF_MISO_MASK 0x10

/* ---------- 遥控 ---------- */
#define REMOTE_CHANNEL 3
#define REMOTE_PACKET_LEN 12
#define REMOTE_TIMEOUT_MS 100
#define REMOTE_KEY_E 1

/* ---------- IMU ---------- */
#define IMU_CHASSIS_ADDR 0x68
#define IMU_GIMBAL_ADDR 0x69
#define MPU6050_REG_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_LSB_PER_DPS 131.0f

/* ---------- 控制周期 ---------- */
#define CONTROL_PERIOD_MS 10
#define DUTY_MAX 10000

/* ---------- 平移 ---------- */
#define JOYSTICK_MAX 2047.0f
#define JOYSTICK_DEADBAND 50
#define MAX_TRANSLATE_DUTY 6000.0f

/* ---------- 自转 ---------- */
#define ROT_GAIN_DUTY_PER_DPS 20.0f
#define SPIN_RATE_DPS 180.0f
#define SPIN_KP 0.5f
#define SPIN_KI 0.05f
#define SPIN_INTEGRAL_LIMIT 5000.0f
#define CHASSIS_YAW_DAMP 0.0f

/* ---------- 云台 ---------- */
#define GIMBAL_MAX_RATE_DPS 120.0f
#define GIMBAL_KP 30.0f
#define GIMBAL_KI 0.0f
#define GIMBAL_KD 2.0f
#define GIMBAL_DEADBAND_DEG 1.0f
#define GIMBAL_INTEGRAL_LIMIT 2000.0f
#define GIMBAL_RATE_FILTER_ALPHA 0.5f

/* ---------- 输出斜率 ---------- */
#define MAX_DUTY_STEP 300
#define GIMBAL_MAX_DUTY_STEP 600

/* ---------- 方向符号（上板标定） ---------- */
#define LEFT_V_SIGN 1.0f
#define LEFT_H_SIGN -1.0f
#define RIGHT_H_SIGN -1.0f
#define WHEEL_SIGN_FL 1.0f
#define WHEEL_SIGN_RL 1.0f
#define WHEEL_SIGN_RR 1.0f
#define WHEEL_SIGN_FR 1.0f
#define GIMBAL_DIR_SIGN 1.0f

#endif
