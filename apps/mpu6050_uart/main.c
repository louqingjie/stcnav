/*
 * MPU6050 -> VOFA+（FireWater 协议）姿态输出。
 *
 * 默认单 IMU；编译时定义 MPU6050_COUNT=2 可同时读取两个 IMU：
 *   IMU0: AD0 -> GND  (0x68)
 *   IMU1: AD0 -> 3.3V (0x69)
 * 两个模块共用 SCL=P1.5、SDA=P1.4，模块都用 3.3V 供电。
 * AD0 不能悬空，否则地址不确定；悬空的那个请补接到 GND。
 *
 * UART1 引脚组由 MPU6050_UART_GROUP 选择：
 *   0 -> P3.0(RxD)/P3.1(TxD)（默认）
 *   3 -> P4.3(RxD)/P4.4(TxD)
 *
 * 数据格式（单 IMU）：roll,pitch,yaw\r\n
 * 数据格式（双 IMU）：roll0,pitch0,yaw0,roll1,pitch1,yaw1\r\n
 * 单位度，20 Hz。
 *
 * 角度约定：Roll 绕 X 轴，Pitch 绕 Y 轴，Yaw 绕 Z 轴（右手方向）。
 * VOFA+：协议 FireWater；IMU0 的立方体用第 1/2/3 列，
 * IMU1 的立方体用第 4/5/6 列。
 *
 * 姿态由加速度计 + 陀螺仪互补滤波得到；启动时自动校准陀螺零偏，
 * 校准期间保持传感器静止。Yaw 无磁力计参考，会缓慢漂移。
 */

#include <math.h>

#include "common.h"
#include "stc32g_i2c.h"
#include "stc32g_timer.h"
#include "stc32g_uart.h"

#ifndef MPU6050_COUNT
#define MPU6050_COUNT 1
#endif

#ifndef MPU6050_UART_GROUP
#define MPU6050_UART_GROUP 0
#endif

#define MPU6050_MAX_COUNT 2

#define MPU6050_WHO_AM_I_VALUE 0x68

#define MPU6050_REG_SMPLRT_DIV 0x19
#define MPU6050_REG_CONFIG 0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_WHO_AM_I 0x75

#define MPU6050_ACCEL_LSB_PER_G 16384.0f
#define MPU6050_GYRO_LSB_PER_DPS 131.0f
#define RAD_TO_DEG 57.2957795f

#define TIMER0_RELOAD ((unsigned int)(65536UL - (FOSC / 1000UL)))

#define UPDATE_MS 50
#define PRINT_MS 50
#define CALIBRATION_SAMPLES 200
#define COMP_ALPHA 0.98f

/* AD0=GND -> 0x68；AD0=3.3V -> 0x69。 */
static const unsigned char mpu_address[MPU6050_MAX_COUNT] = {0x68, 0x69};

static __data volatile unsigned int ms_ticks;

static unsigned char mpu_raw[14];

static float roll_deg[MPU6050_MAX_COUNT];
static float pitch_deg[MPU6050_MAX_COUNT];
static float yaw_deg[MPU6050_MAX_COUNT];

static float gyro_bias_x[MPU6050_MAX_COUNT];
static float gyro_bias_y[MPU6050_MAX_COUNT];
static float gyro_bias_z[MPU6050_MAX_COUNT];

void timer0_isr(void) __interrupt (1)
{
    ms_ticks++;
}

static unsigned int millis(void)
{
    unsigned int value;

    ET0 = 0;
    value = ms_ticks;
    ET0 = 1;
    return value;
}

static void delay_ms(unsigned int ms)
{
    unsigned int start = millis();

    while ((unsigned int)(millis() - start) < ms)
    {
    }
}

static signed short mpu_read_be16(const unsigned char *buffer)
{
    return (signed short)(((unsigned int)buffer[0] << 8) | (unsigned int)buffer[1]);
}

static void uart_write_long(long value)
{
    char digits[10];
    unsigned char count = 0;
    unsigned long magnitude;

    if (value < 0)
    {
        uart1_write_byte('-');
        magnitude = (unsigned long)(-value);
    }
    else
    {
        magnitude = (unsigned long)value;
    }

    do
    {
        digits[count++] = (char)('0' + (unsigned char)(magnitude % 10UL));
        magnitude /= 10UL;
    } while (magnitude != 0);

    while (count != 0)
        uart1_write_byte((unsigned char)digits[--count]);
}

static void uart_write_fixed100(long value)
{
    unsigned char fraction;

    if (value < 0)
    {
        uart1_write_byte('-');
        value = -value;
    }

    uart_write_long(value / 100L);
    uart1_write_byte('.');

    fraction = (unsigned char)(value % 100L);
    if (fraction < 10)
        uart1_write_byte('0');
    uart1_write_byte((unsigned char)('0' + fraction / 10));
    uart1_write_byte((unsigned char)('0' + fraction % 10));
}

static void uart_write_hex8(unsigned char value)
{
    const char *digits = "0123456789ABCDEF";

    uart1_write_byte((unsigned char)digits[value >> 4]);
    uart1_write_byte((unsigned char)digits[value & 0x0F]);
}

static unsigned char mpu6050_init(unsigned char address, unsigned char *who)
{
    *who = 0;

    if (i2c_read_bytes(address, MPU6050_REG_WHO_AM_I, who, 1) != 0)
        return 0;
    if (*who != MPU6050_WHO_AM_I_VALUE)
        return 0;

    /* 复位后选择 X 轴陀螺 PLL 作为时钟源。 */
    if (i2c_write_reg(address, MPU6050_REG_PWR_MGMT_1, 0x80) != 0)
        return 0;
    delay_ms(100);
    if (i2c_write_reg(address, MPU6050_REG_PWR_MGMT_1, 0x01) != 0)
        return 0;
    delay_ms(10);

    /* DLPF 44 Hz，采样率 1 kHz/(1+4)=200 Hz，±2g，±250 dps。 */
    if (i2c_write_reg(address, MPU6050_REG_CONFIG, 0x03) != 0)
        return 0;
    if (i2c_write_reg(address, MPU6050_REG_SMPLRT_DIV, 0x04) != 0)
        return 0;
    if (i2c_write_reg(address, MPU6050_REG_GYRO_CONFIG, 0x00) != 0)
        return 0;
    if (i2c_write_reg(address, MPU6050_REG_ACCEL_CONFIG, 0x00) != 0)
        return 0;

    delay_ms(50);
    return 1;
}

static unsigned char mpu6050_read(unsigned char address, unsigned char *raw)
{
    return i2c_read_bytes(address, MPU6050_REG_ACCEL_XOUT_H, raw, 14);
}

static void accel_angles(const unsigned char *raw, float *roll, float *pitch)
{
    float ax = (float)mpu_read_be16(&raw[0]) / MPU6050_ACCEL_LSB_PER_G;
    float ay = (float)mpu_read_be16(&raw[2]) / MPU6050_ACCEL_LSB_PER_G;
    float az = (float)mpu_read_be16(&raw[4]) / MPU6050_ACCEL_LSB_PER_G;

    *roll = atan2f(ay, az) * RAD_TO_DEG;
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

static float wrap_180(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

static unsigned char mpu6050_calibrate_gyro(unsigned char address, unsigned char index)
{
    unsigned int sample;
    unsigned int count = 0;
    long sum_x = 0;
    long sum_y = 0;
    long sum_z = 0;

    for (sample = 0; sample < CALIBRATION_SAMPLES; sample++)
    {
        if (mpu6050_read(address, mpu_raw) == 0)
        {
            sum_x += mpu_read_be16(&mpu_raw[8]);
            sum_y += mpu_read_be16(&mpu_raw[10]);
            sum_z += mpu_read_be16(&mpu_raw[12]);
            count++;
        }

        /* 读取本身约 4 ms，再等 1 ms，接近传感器 200 Hz 更新率。 */
        delay_ms(1);
    }

    if (count == 0)
        return 0;

    gyro_bias_x[index] = ((float)sum_x / (float)count) / MPU6050_GYRO_LSB_PER_DPS;
    gyro_bias_y[index] = ((float)sum_y / (float)count) / MPU6050_GYRO_LSB_PER_DPS;
    gyro_bias_z[index] = ((float)sum_z / (float)count) / MPU6050_GYRO_LSB_PER_DPS;

    return 1;
}

static void orientation_update(unsigned char index, const unsigned char *raw, float dt)
{
    float ax = (float)mpu_read_be16(&raw[0]) / MPU6050_ACCEL_LSB_PER_G;
    float ay = (float)mpu_read_be16(&raw[2]) / MPU6050_ACCEL_LSB_PER_G;
    float az = (float)mpu_read_be16(&raw[4]) / MPU6050_ACCEL_LSB_PER_G;
    float gx = (float)mpu_read_be16(&raw[8]) / MPU6050_GYRO_LSB_PER_DPS - gyro_bias_x[index];
    float gy = (float)mpu_read_be16(&raw[10]) / MPU6050_GYRO_LSB_PER_DPS - gyro_bias_y[index];
    float gz = (float)mpu_read_be16(&raw[12]) / MPU6050_GYRO_LSB_PER_DPS - gyro_bias_z[index];
    float roll_acc = atan2f(ay, az) * RAD_TO_DEG;
    float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

    roll_deg[index] = COMP_ALPHA * (roll_deg[index] + gx * dt) + (1.0f - COMP_ALPHA) * roll_acc;
    pitch_deg[index] = COMP_ALPHA * (pitch_deg[index] + gy * dt) + (1.0f - COMP_ALPHA) * pitch_acc;
    yaw_deg[index] += gz * dt;

    roll_deg[index] = wrap_180(roll_deg[index]);
    pitch_deg[index] = wrap_180(pitch_deg[index]);
    yaw_deg[index] = wrap_180(yaw_deg[index]);
}

static long angle_to_x100(float angle)
{
    if (angle >= 0.0f)
        return (long)(angle * 100.0f + 0.5f);
    return (long)(angle * 100.0f - 0.5f);
}

static void print_vofa(void)
{
    unsigned char index;

    for (index = 0; index < MPU6050_COUNT; index++)
    {
        if (index != 0)
            uart1_write_byte(',');

        uart_write_fixed100(angle_to_x100(roll_deg[index]));
        uart1_write_byte(',');
        uart_write_fixed100(angle_to_x100(pitch_deg[index]));
        uart1_write_byte(',');
        uart_write_fixed100(angle_to_x100(yaw_deg[index]));
    }
    uart1_write("\r\n");
}

void main(void)
{
    unsigned char who;
    unsigned char index;
    unsigned char ok;
    unsigned int last_update;
    unsigned int last_print;
    unsigned int now;
    unsigned int elapsed;
    float dt;

    board_init();
    uart1_init(115200UL, MPU6050_UART_GROUP);
    i2c_init();

    /* 1 ms 系统节拍，用于采样调度和陀螺积分。 */
    timer0_init(TIMER0_RELOAD);
    ms_ticks = 0;
    ET0 = 1;
    EA = 1;
    timer0_start();

    uart1_write("MPU6050 VOFA+ TEST\r\n");

    for (;;)
    {
        ok = 1;
        for (index = 0; index < MPU6050_COUNT; index++)
        {
            if (!mpu6050_init(mpu_address[index], &who))
            {
                uart1_write("IMU");
                uart1_write_byte((unsigned char)('0' + index));
                uart1_write(" NOT FOUND - retry\r\n");
                ok = 0;
                break;
            }

            uart1_write("IMU");
            uart1_write_byte((unsigned char)('0' + index));
            uart1_write(" OK - WHO_AM_I=0x");
            uart_write_hex8(who);
            uart1_write("\r\n");
        }
        if (!ok)
        {
            delay_ms(1000);
            continue;
        }

        uart1_write("CALIBRATING GYRO - keep still\r\n");
        delay_ms(500);

        for (index = 0; index < MPU6050_COUNT; index++)
        {
            if (!mpu6050_calibrate_gyro(mpu_address[index], index))
            {
                uart1_write("CALIBRATION FAIL\r\n");
                ok = 0;
                break;
            }
        }
        if (!ok)
        {
            delay_ms(1000);
            continue;
        }

        for (index = 0; index < MPU6050_COUNT; index++)
        {
            if (mpu6050_read(mpu_address[index], mpu_raw) != 0)
            {
                uart1_write("READ FAIL - reinit\r\n");
                ok = 0;
                break;
            }
            accel_angles(mpu_raw, &roll_deg[index], &pitch_deg[index]);
            yaw_deg[index] = 0.0f;
        }
        if (!ok)
        {
            delay_ms(500);
            continue;
        }

        last_update = millis();
        last_print = last_update;

        uart1_write("READY\r\n");

        for (;;)
        {
            now = millis();

            if ((unsigned int)(now - last_update) >= UPDATE_MS)
            {
                elapsed = (unsigned int)(now - last_update);
                last_update = now;

                dt = (float)elapsed * 0.001f;
                if (dt > 0.5f)
                    dt = 0.5f;

                ok = 1;
                for (index = 0; index < MPU6050_COUNT; index++)
                {
                    if (mpu6050_read(mpu_address[index], mpu_raw) != 0)
                    {
                        ok = 0;
                        break;
                    }
                    orientation_update(index, mpu_raw, dt);
                }
                if (!ok)
                {
                    uart1_write("READ FAIL - reinit\r\n");
                    delay_ms(500);
                    break;
                }
            }

            if ((unsigned int)(now - last_print) >= PRINT_MS)
            {
                last_print = now;
                print_vofa();
            }
        }
    }
}
