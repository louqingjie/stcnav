/*
 * 双 MPU6050 yaw 读取。
 *
 * 只读 GYRO_ZOUT (0x47/0x48)，积分得到 yaw。
 * 启动时两个传感器各采样 200 次计算陀螺 Z 零偏，期间保持静止。
 */

#include "imu.h"
#include "config.h"
#include "common.h"
#include "stc32g_i2c.h"
#include "timebase.h"

#define MPU6050_REG_WHO_AM_I 0x75
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_CONFIG 0x1A
#define MPU6050_REG_SMPLRT_DIV 0x19
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_WHO_AM_I_VALUE 0x68

#define CALIBRATION_SAMPLES 200
#define CALIBRATION_INTERVAL_MS 5

static float chassis_yaw;
static float gimbal_yaw;
static float chassis_rate;
static float gimbal_rate;
static float chassis_bias;
static float gimbal_bias;
static unsigned char fault;

static float wrap180(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

static unsigned char mpu_write_reg(unsigned char address, unsigned char reg, unsigned char value)
{
    return i2c_write_reg(address, reg, value);
}

static unsigned char mpu_read_reg(unsigned char address, unsigned char reg, unsigned char *value)
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

static unsigned char mpu_read_rate(unsigned char address, float bias, float *rate)
{
    unsigned char buffer[2];
    signed short raw;

    if (i2c_read_bytes(address, MPU6050_REG_GYRO_ZOUT_H, buffer, 2) != 0)
        return 0;

    raw = (signed short)(((unsigned int)buffer[0] << 8) | (unsigned int)buffer[1]);
    *rate = (float)raw / MPU6050_GYRO_LSB_PER_DPS - bias;
    return 1;
}

static unsigned char mpu_calibrate_z(unsigned char address, float *bias)
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

unsigned char imu_init(void)
{
    if (!mpu_init_one(IMU_CHASSIS_ADDR))
        return 0;
    if (!mpu_init_one(IMU_GIMBAL_ADDR))
        return 0;

    if (!mpu_calibrate_z(IMU_CHASSIS_ADDR, &chassis_bias))
        return 0;
    if (!mpu_calibrate_z(IMU_GIMBAL_ADDR, &gimbal_bias))
        return 0;

    chassis_yaw = 0.0f;
    gimbal_yaw = 0.0f;
    chassis_rate = 0.0f;
    gimbal_rate = 0.0f;
    fault = 0;

    return 1;
}

unsigned char imu_update(float dt)
{
    if (!mpu_read_rate(IMU_CHASSIS_ADDR, chassis_bias, &chassis_rate))
    {
        fault = 1;
        return 0;
    }
    if (!mpu_read_rate(IMU_GIMBAL_ADDR, gimbal_bias, &gimbal_rate))
    {
        fault = 1;
        return 0;
    }

    chassis_yaw = wrap180(chassis_yaw + chassis_rate * dt);
    gimbal_yaw = wrap180(gimbal_yaw + gimbal_rate * dt);

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

unsigned char imu_fault(void)
{
    return fault;
}
