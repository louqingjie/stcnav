/*
 * 双 MPU6050 读取：底盘姿态 + 云台单轴 yaw。
 *
 * 底盘姿态（无持续 Mahony 修正，控制 flash 占用）：
 *   1. 加速度计低通后归一化，用最短弧四元数把重力方向对齐到参考系 +z；
 *   2. 陀螺（z 去零偏）在参考系 z 轴上的投影积分出 yaw；
 *   3. 四元数 = 航向四元数 (x) 倾角四元数。
 * 参考系从上电时开始（yaw=0，重力对齐）；无磁力计，yaw 缓慢漂移，
 * 倾角对线加速度敏感（用 0.8~1.2 g 模长门限屏蔽）。
 *
 * 云台：只读 GYRO_ZOUT，积分得到相对 yaw。
 * 两个传感器启动时各采样 200 次计算陀螺 Z 零偏，期间保持静止。
 */

#include "imu.h"
#include "config.h"
#include "common.h"
#include "stc32g_i2c.h"
#include "timebase.h"

/*
 * IMU_PUBLISH_V2=1 时把姿态/角速度/加速度直接写入 v2_codec 的 TX 消息
 * （否则 tune 类 app 不必链接 codec；本文件在 tune app 中仍可单独编译）。
 */
#ifndef IMU_PUBLISH_V2
#define IMU_PUBLISH_V2 0
#endif
#if IMU_PUBLISH_V2
#include "v2_codec.h"
#endif

#define MPU6050_REG_WHO_AM_I 0x75
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_CONFIG 0x1A
#define MPU6050_REG_SMPLRT_DIV 0x19
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_GYRO_ZOUT_H 0x47
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_WHO_AM_I_VALUE 0x68

#define CALIBRATION_SAMPLES 200
#define CALIBRATION_INTERVAL_MS 5

#define DEG_TO_RAD 0.0174532925f
#define RAD_TO_DEG 57.2957795f

#define ACCEL_LP_ALPHA 0.35f
#define ACCEL_GATE_MIN 0.64f /* (0.8g)^2 */
#define ACCEL_GATE_MAX 1.44f /* (1.2g)^2 */
#define YAW_QUAT_RENORM_INTERVAL 32u

#define INV_SQRT_MAGIC 0x5F3759DFUL

imu_state_t IMU_XDATA imu_snapshot;

static float chassis_yaw;
static float gimbal_yaw;
static float chassis_rate;
static float gimbal_rate;

static float chassis_bias_z;
static float gimbal_bias_z;

static unsigned int epoch;

static unsigned char raw_sample[14];

/* 最近一次采样（IMU 系，原始量）。 */
static float accel_x_g;
static float accel_y_g;
static float accel_z_g;
static float gyro_x_rps;
static float gyro_y_rps;
static float gyro_z_rps;
static float gyro_z_dps;

/* 姿态滤波状态：单位重力方向（IMU 系）与倾角四元数 (x,y,0,w)。 */
static float unit_x;
static float unit_y;
static __data float unit_z;
static __data float tilt_x;
static __data float tilt_y;
static float tilt_w;

/* 航向半角 sin/cos 与低通后的加速度。 */
static __data float yaw_sin_half;
static __data float yaw_cos_half;
static float accel_lp_x;
static float accel_lp_y;
static float accel_lp_z;

/* 计算中间量（放 XRAM，避免硬件栈溢出，也避免重复压栈）。 */
static float accel_lp_tmp;
static float norm2;
static float inverse_norm;
static float gravity_x;
static float gravity_y;
static float gravity_z;
static float yaw_rate_ref;
static float half_angle;
static float sin_step;
static float cos_step;
static float new_sin;
static float new_cos;
static float yaw_deg;
static unsigned char renorm_counter;
static unsigned char filter_started;

static float wrap180(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

/* 1/sqrt 近似：IEEE-754 位技巧 + 两次牛顿迭代。 */
static float inv_sqrt(float value)
{
    union
    {
        float f;
        unsigned long u;
    } bits;
    float approx;

    bits.f = value;
    bits.u = INV_SQRT_MAGIC - (bits.u >> 1);
    approx = bits.f;
    approx = approx * (1.5f - 0.5f * value * approx * approx);
    approx = approx * (1.5f - 0.5f * value * approx * approx);
    return approx;
}

static signed short read_be16(const unsigned char *buffer)
{
    return (signed short)(((unsigned int)buffer[0] << 8) | (unsigned int)buffer[1]);
}

static unsigned char mpu_write_reg(unsigned char address, unsigned char reg,
                                   unsigned char value)
{
    return i2c_write_reg(address, reg, value);
}

static unsigned char mpu_read_reg(unsigned char address, unsigned char reg,
                                  unsigned char *value)
{
    return i2c_read_bytes(address, reg, value, 1);
}

static unsigned char mpu_init_one(unsigned char address)
{
    unsigned char who;

    if (mpu_read_reg(address, MPU6050_REG_WHO_AM_I, &who) != 0)
        return 0;
    if (who != MPU6050_WHO_AM_I_VALUE)
        return 0;

    if (mpu_write_reg(address, MPU6050_REG_PWR_MGMT_1, 0x80) != 0)
        return 0;
    delay_ms(100);

    if (mpu_write_reg(address, MPU6050_REG_PWR_MGMT_1, 0x01) != 0)
        return 0;
    delay_ms(10);

    if (mpu_write_reg(address, MPU6050_REG_CONFIG, 0x03) != 0)
        return 0;
    if (mpu_write_reg(address, MPU6050_REG_SMPLRT_DIV, 0x04) != 0)
        return 0;
    if (mpu_write_reg(address, MPU6050_REG_GYRO_CONFIG, 0x00) != 0)
        return 0;
    if (mpu_write_reg(address, MPU6050_REG_ACCEL_CONFIG, 0x00) != 0)
        return 0;

    delay_ms(50);
    return 1;
}

/* 读底盘 IMU 14 字节：accel(6) + temp(2) + gyro(6)。 */
static unsigned char read_chassis_sample(void)
{
    if (i2c_read_bytes(IMU_CHASSIS_ADDR, MPU6050_REG_ACCEL_XOUT_H, raw_sample, 14) != 0)
        return 0;

    accel_x_g = (float)read_be16(raw_sample) / MPU6050_ACCEL_LSB_PER_G;
    accel_y_g = (float)read_be16(raw_sample + 2) / MPU6050_ACCEL_LSB_PER_G;
    accel_z_g = (float)read_be16(raw_sample + 4) / MPU6050_ACCEL_LSB_PER_G;

    gyro_x_rps = (float)read_be16(raw_sample + 8) / MPU6050_GYRO_LSB_PER_DPS * DEG_TO_RAD;
    gyro_y_rps = (float)read_be16(raw_sample + 10) / MPU6050_GYRO_LSB_PER_DPS * DEG_TO_RAD;
    gyro_z_dps = (float)read_be16(raw_sample + 12) / MPU6050_GYRO_LSB_PER_DPS -
                 chassis_bias_z;
    gyro_z_rps = gyro_z_dps * DEG_TO_RAD;
    return 1;
}

static unsigned char read_gimbal_sample(void)
{
    unsigned char buffer[2];
    signed short raw;

    if (i2c_read_bytes(IMU_GIMBAL_ADDR, MPU6050_REG_GYRO_ZOUT_H, buffer, 2) != 0)
        return 0;

    raw = (signed short)(((unsigned int)buffer[0] << 8) | (unsigned int)buffer[1]);
    gimbal_rate = (float)raw / MPU6050_GYRO_LSB_PER_DPS - gimbal_bias_z;
    return 1;
}

static unsigned char mpu_calibrate_gyro_z(unsigned char address, float *bias)
{
    unsigned char buffer[2];
    unsigned int sample;
    unsigned int count = 0;
    long sum = 0;
    signed short raw;

    for (sample = 0; sample < CALIBRATION_SAMPLES; sample++)
    {
        if (i2c_read_bytes(address, MPU6050_REG_GYRO_ZOUT_H, buffer, 2) == 0)
        {
            raw = (signed short)(((unsigned int)buffer[0] << 8) | (unsigned int)buffer[1]);
            sum += raw;
            count++;
        }
        delay_ms(CALIBRATION_INTERVAL_MS);
    }

    if (count == 0)
        return 0;

    *bias = ((float)sum / (float)count) / MPU6050_GYRO_LSB_PER_DPS;
    return 1;
}

static void attitude_reset(void)
{
    unit_x = 0.0f;
    unit_y = 0.0f;
    unit_z = 1.0f;
    tilt_x = 0.0f;
    tilt_y = 0.0f;
    tilt_w = 1.0f;
    yaw_sin_half = 0.0f;
    yaw_cos_half = 1.0f;
    accel_lp_x = 0.0f;
    accel_lp_y = 0.0f;
    accel_lp_z = 0.0f;
    yaw_deg = 0.0f;
    renorm_counter = 0;
    filter_started = 0;
}

/*
 * 倾角（加速度计）+ 航向（陀螺 z）合成四元数。
 * q = R_reference_from_body，表示把 IMU 系向量旋转到参考系。
 */
static void attitude_update(float dt)
{
    /* 加速度低通（首帧直接初始化，避免收敛过程）。 */
    if (!filter_started)
    {
        accel_lp_x = accel_x_g;
        accel_lp_y = accel_y_g;
        accel_lp_z = accel_z_g;
        filter_started = 1;
    }
    else
    {
        accel_lp_tmp = accel_x_g - accel_lp_x;
        accel_lp_x += ACCEL_LP_ALPHA * accel_lp_tmp;
        accel_lp_tmp = accel_y_g - accel_lp_y;
        accel_lp_y += ACCEL_LP_ALPHA * accel_lp_tmp;
        accel_lp_tmp = accel_z_g - accel_lp_z;
        accel_lp_z += ACCEL_LP_ALPHA * accel_lp_tmp;
    }

    norm2 = accel_lp_x * accel_lp_x + accel_lp_y * accel_lp_y +
            accel_lp_z * accel_lp_z;

    if (norm2 > ACCEL_GATE_MIN && norm2 < ACCEL_GATE_MAX)
    {
        inverse_norm = inv_sqrt(norm2);
        unit_x = accel_lp_x * inverse_norm;
        unit_y = accel_lp_y * inverse_norm;
        unit_z = accel_lp_z * inverse_norm;

        gravity_z = 1.0f + unit_z;
        if (gravity_z < 1.0e-4f)
        {
            /* 倒置：绕 x 轴 180°。 */
            tilt_x = 1.0f;
            tilt_y = 0.0f;
            tilt_w = 0.0f;
        }
        else
        {
            /* 最短弧四元数 normalize((ay, -ax, 0, 1+az))。 */
            inverse_norm = inv_sqrt(2.0f * gravity_z);
            tilt_x = unit_y * inverse_norm;
            tilt_y = -unit_x * inverse_norm;
            tilt_w = gravity_z * inverse_norm;
        }
    }

    /* 参考系 z 轴的机体角速度 = 单位重力方向 · 机体角速度。 */
    yaw_rate_ref = unit_x * gyro_x_rps + unit_y * gyro_y_rps +
                   unit_z * gyro_z_rps;
    yaw_deg = wrap180(yaw_deg + yaw_rate_ref * RAD_TO_DEG * dt);

    /* 小角度增量更新 sin/cos(yaw/2)。 */
    half_angle = 0.5f * yaw_rate_ref * dt;
    sin_step = half_angle;
    cos_step = 1.0f - 0.5f * half_angle * half_angle;
    new_sin = yaw_sin_half * cos_step + yaw_cos_half * sin_step;
    new_cos = yaw_cos_half * cos_step - yaw_sin_half * sin_step;

    renorm_counter++;
    if (renorm_counter >= YAW_QUAT_RENORM_INTERVAL)
    {
        renorm_counter = 0;
        norm2 = new_sin * new_sin + new_cos * new_cos;
        inverse_norm = inv_sqrt(norm2);
        new_sin *= inverse_norm;
        new_cos *= inverse_norm;
    }
    yaw_sin_half = new_sin;
    yaw_cos_half = new_cos;

    /* q = q_yaw (x) q_tilt，展开后只需 6 次乘法（tilt_z = 0）。 */
    gravity_x = yaw_cos_half * tilt_w;
    gravity_y = yaw_cos_half * tilt_x + yaw_sin_half * tilt_y;
    gravity_z = yaw_cos_half * tilt_y - yaw_sin_half * tilt_x;
    tilt_w = yaw_sin_half * tilt_w;

    /*
     * 发布前把四元数归一化：V2 编码器要求 orientation_valid 帧满足
     * |norm-1| <= 1e-3，否则整帧丢弃（实测不改时会丢 20~50% 的 0x88 帧，
     * 频率掉到 20~40 Hz）。yaw 增量与倾角四元数相乘后可能出现范数漂移，
     * 这里每帧做一次 Newton 逆平方根归一化。
     */
    norm2 = gravity_y * gravity_y + gravity_z * gravity_z +
            tilt_w * tilt_w + gravity_x * gravity_x;
    if (norm2 > 1.0e-6f && norm2 < 4.0f)
    {
        inverse_norm = inv_sqrt(norm2);
        gravity_y *= inverse_norm;
        gravity_z *= inverse_norm;
        tilt_w *= inverse_norm;
        gravity_x *= inverse_norm;
    }

#if IMU_PUBLISH_V2
    v2_tx_attitude.orientation_x = gravity_y;
    v2_tx_attitude.orientation_y = gravity_z;
    v2_tx_attitude.orientation_z = tilt_w;
    v2_tx_attitude.orientation_w = gravity_x;
#endif
}

unsigned char imu_init(void)
{
    if (!mpu_init_one(IMU_CHASSIS_ADDR))
        return 0;
    if (!mpu_init_one(IMU_GIMBAL_ADDR))
        return 0;

    if (!mpu_calibrate_gyro_z(IMU_CHASSIS_ADDR, &chassis_bias_z))
        return 0;
    if (!mpu_calibrate_gyro_z(IMU_GIMBAL_ADDR, &gimbal_bias_z))
        return 0;

    chassis_yaw = 0.0f;
    gimbal_yaw = 0.0f;
    chassis_rate = 0.0f;
    gimbal_rate = 0.0f;
    attitude_reset();

    epoch++;
    imu_snapshot.epoch = epoch;
    imu_snapshot.valid = 0;
#if IMU_PUBLISH_V2
    v2_tx_attitude.orientation_x = 0.0f;
    v2_tx_attitude.orientation_y = 0.0f;
    v2_tx_attitude.orientation_z = 0.0f;
    v2_tx_attitude.orientation_w = 1.0f;
    v2_tx_attitude.angular_velocity_x_rad_s = 0.0f;
    v2_tx_attitude.angular_velocity_y_rad_s = 0.0f;
    v2_tx_attitude.angular_velocity_z_rad_s = 0.0f;
    v2_tx_attitude.linear_acceleration_x_mps2 = 0.0f;
    v2_tx_attitude.linear_acceleration_y_mps2 = 0.0f;
    v2_tx_attitude.linear_acceleration_z_mps2 = GRAVITY_MPS2;
#endif

    return 1;
}

unsigned char imu_update(float dt)
{
    if (dt <= 0.0f)
        dt = 0.01f;
    if (dt > 0.2f)
        dt = 0.2f;

    if (!read_chassis_sample())
    {
        imu_snapshot.valid = 0;
        return 0;
    }
    if (!read_gimbal_sample())
    {
        imu_snapshot.valid = 0;
        return 0;
    }

    attitude_update(dt);

    chassis_rate = gyro_z_dps;
    chassis_yaw = yaw_deg;
    gimbal_yaw = wrap180(gimbal_yaw + gimbal_rate * dt);

#if IMU_PUBLISH_V2
    v2_tx_attitude.angular_velocity_x_rad_s = gyro_x_rps;
    v2_tx_attitude.angular_velocity_y_rad_s = gyro_y_rps;
    v2_tx_attitude.angular_velocity_z_rad_s = gyro_z_rps;
    v2_tx_attitude.linear_acceleration_x_mps2 = accel_x_g * GRAVITY_MPS2;
    v2_tx_attitude.linear_acceleration_y_mps2 = accel_y_g * GRAVITY_MPS2;
    v2_tx_attitude.linear_acceleration_z_mps2 = accel_z_g * GRAVITY_MPS2;
#endif
    imu_snapshot.chassis_sample_sequence++;
    imu_snapshot.gimbal_sample_sequence++;
    imu_snapshot.sample_time_ms = millis32();
    imu_snapshot.valid = 1;

    return 1;
}

float imu_get_chassis_yaw(void)
{
    return chassis_yaw;
}

float imu_get_gimbal_yaw(void)
{
    return gimbal_yaw;
}

float imu_get_chassis_rate(void)
{
    return chassis_rate;
}

float imu_get_gimbal_rate(void)
{
    return gimbal_rate;
}
