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
#define MPU6050_ACCEL_LSB_PER_G 16384.0f
#define GRAVITY_MPS2 9.80665f

/* 六轴姿态互补/Mahony 滤波参数（100 Hz 控制周期）。 */
#define IMU_FILTER_KP 1.0f
#define IMU_FILTER_KI 0.02f
#define IMU_FILTER_INTEGRAL_LIMIT 0.5f

/* ---------- V2 上位机链路（UART2 P1.0/P1.1） ---------- */
/* 与 UART1 的拓展板/调参共用 Timer2，因此必须保持 230400。 */
#define V2_UART_BAUD 230400UL
#define V2_STATE_PERIOD_MS 20       /* 0x82/0x85/0x88 -> 50 Hz */
#define V2_HEARTBEAT_PERIOD_MS 100  /* 0x03 -> 10 Hz */
#define V2_CMD_TIMEOUT_MS 200       /* 0x01 命令新鲜度 */
#define V2_HOST_TIMEOUT_MS 500      /* 上位机心跳超时 -> 撤销 V2 运动授权 */
#define V2_MAX_LINEAR_MPS 2.0f
#define V2_MAX_WZ_RPS 6.2831853f
#define V2_ENABLE_MOTION 1
/* bit0 chassis command | bit2 gimbal | bit5 heading | bit8 attitude */
#define V2_CAPABILITIES 0x00000125UL
/* app 区最后 512 B 的保留扇区，用于上电 boot_id 计数。 */
#define V2_BOOT_SECTOR_ADDR 0xFEFE00UL

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
