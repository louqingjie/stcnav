/*
 * 整车主程序：遥控 + 双 IMU + 四全向轮 + 云台 + RM2027 V2 上位机链路。
 *
 * 控制周期 10 ms（35 MHz 主频）。
 * 正常构建（vehicle）：UART1 P3.0/P3.1 230400 控制拓展板；
 *                      UART2 P1.0/P1.1 230400 与上位机通信（V2 协议）。
 * VEHICLE_DEBUG：UART1 P4.3/P4.4 230400 输出调试 CSV 与 V2 golden 自检；
 *                 不控制拓展板，也不运行 V2 链路（UART2 留给现场调试）。
 *
 * 运动所有权（normal 构建）：NRF 遥控 > V2 上位机命令；两者都无有效命令时停车。
 */

#include "config.h"
#include "remote.h"
#include "expansion.h"
#include "imu.h"
#include "chassis.h"
#include "gimbal.h"
#include "timebase.h"
#include "common.h"
#include "stc32g_timer.h"
#include "stc32g_uart.h"
#include "stc32g_i2c.h"
#if VEHICLE_DEBUG
#include "v2_selftest.h"
#define V2_SERVICE_WAIT(ms) delay_ms(ms)
#define V2_POLL(now, ok) ((void)(now), (void)(ok))
#define V2_SET_READY(value) ((void)(value))
#else
#include "v2_link.h"
#include "stc32g_uart2.h"
#define V2_SERVICE_WAIT(ms) service_wait(ms)
#define V2_POLL(now, ok) v2_link_poll((now), (ok))
#define V2_SET_READY(value) v2_link_set_ready(value)
#endif

#define TIMER0_RELOAD ((unsigned int)(65536UL - (FOSC / 1000UL)))

#define OWNER_NONE 0
#define OWNER_REMOTE 1
#define OWNER_V2 2

static __data volatile unsigned int ms_ticks;

static unsigned char motor_dir[8];
static unsigned short motor_duty[8];
static signed short wheel_duty[4];

void timer0_isr(void) __interrupt (1)
{
    ms_ticks++;
}

/* UART2 中断服务程序位于 v2_link.c（向量 8，使能位 IE2.1/ES2）。 */

unsigned int millis(void)
{
    unsigned int value;

    ET0 = 0;
    value = ms_ticks;
    ET0 = 1;
    return value;
}

void delay_ms(unsigned int ms)
{
    unsigned int start = millis();

    while ((unsigned int)(millis() - start) < ms)
    {
    }
}

#if !VEHICLE_DEBUG
/* 启动/故障等待期间保持 V2 心跳。 */
static void service_wait(unsigned int wait_ms)
{
    unsigned int start = millis();

    while ((unsigned int)(millis() - start) < wait_ms)
        v2_link_poll(millis(), 0);
}
#endif

#if VEHICLE_DEBUG
static void debug_write_long(long value)
{
    static char digits[10];
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

static long angle_to_d10(float angle)
{
    if (angle >= 0.0f)
        return (long)(angle * 10.0f + 0.5f);
    return (long)(angle * 10.0f - 0.5f);
}

static void debug_print(signed short gimbal_duty)
{
    uart1_write_byte(g_remote.connected ? '1' : '0');
    uart1_write_byte(',');
    debug_write_long(g_remote.left_v);
    uart1_write_byte(',');
    debug_write_long(g_remote.left_h);
    uart1_write_byte(',');
    debug_write_long(g_remote.right_v);
    uart1_write_byte(',');
    debug_write_long(g_remote.right_h);
    uart1_write_byte(',');
    debug_write_long((long)g_remote.keys);
    uart1_write_byte(',');
    debug_write_long(angle_to_d10(imu_get_chassis_yaw()));
    uart1_write_byte(',');
    debug_write_long(angle_to_d10(imu_get_gimbal_yaw()));
    uart1_write_byte(',');
    debug_write_long(angle_to_d10(gimbal_get_target()));
    uart1_write_byte(',');
    debug_write_long(wheel_duty[0]);
    uart1_write_byte(',');
    debug_write_long(wheel_duty[1]);
    uart1_write_byte(',');
    debug_write_long(wheel_duty[2]);
    uart1_write_byte(',');
    debug_write_long(wheel_duty[3]);
    uart1_write_byte(',');
    debug_write_long(gimbal_duty);
    uart1_write("\r\n");
}
#endif

void main(void)
{
    unsigned int last_control;
    unsigned int now;
    unsigned int elapsed;
    float dt;
    unsigned char spin_mode = 0;
    unsigned char last_e = 0;
    unsigned char imu_ok = 0;
    unsigned char index;
    unsigned char owner = OWNER_NONE;
    unsigned char last_owner = OWNER_NONE;
    signed short gimbal_duty = 0;
#if !VEHICLE_DEBUG
    float vx;
    float vy;
    float wz;
    unsigned long boot_id;
#else
    unsigned int last_print;
#endif

    board_init();

#if !VEHICLE_DEBUG
    /* boot_id 持久化在 app 区最后 512 B 扇区（STC-ISP 需配置 EEPROM 区）。 */
    boot_id = v2_boot_id_load();
#endif

    timer0_init(TIMER0_RELOAD);
    ms_ticks = 0;
    ET0 = 1;
    EA = 1;
    timer0_start();

#if VEHICLE_DEBUG
    uart1_init(V2_UART_BAUD, 3);
    uart1_write("DEBUG BOOT\r\n");
    if (v2_selftest() == 0u)
        uart1_write("V2SELFTEST PASS\r\n");
    else
        uart1_write("V2SELFTEST FAIL\r\n");
#else
    expansion_init();
    /* V2 链路尽早启动：上位机立即看到 boot_id/capability（ready=0）。 */
    v2_link_init(boot_id, millis());
#endif

    i2c_init();

#if VEHICLE_DEBUG
    uart1_write("IMU INIT...\r\n");
#endif
    while (!imu_init())
    {
#if VEHICLE_DEBUG
        uart1_write("IMU FAIL\r\n");
#else
        expansion_stop();
#endif
        V2_SERVICE_WAIT(500);
    }
    V2_SET_READY(1);
#if VEHICLE_DEBUG
    uart1_write("IMU OK\r\n");
    uart1_write("NRF INIT...\r\n");
#endif

    while (!remote_init())
    {
#if VEHICLE_DEBUG
        uart1_write("NRF FAIL\r\n");
#else
        expansion_stop();
#endif
        V2_SERVICE_WAIT(500);
    }
#if VEHICLE_DEBUG
    uart1_write("NRF OK\r\n");
#endif

    gimbal_set_target(0.0f);
    last_control = millis();
#if VEHICLE_DEBUG
    last_print = last_control;
    uart1_write("conn,lv,lh,rv,rh,key,chy,gya,gtg,w0,w1,w2,w3,gd\r\n");
#endif

    for (;;)
    {
        now = millis();

        V2_POLL(now, imu_ok);

        if ((unsigned int)(now - last_control) < CONTROL_PERIOD_MS)
            continue;

        elapsed = (unsigned int)(now - last_control);
        last_control = now;
        dt = (float)elapsed * 0.001f;
        if (dt > 0.5f)
            dt = 0.5f;

        remote_poll(now);
        imu_ok = imu_update(dt);

        if (!imu_ok)
        {
            /* IMU 故障：先停车、撤销 ready，再重新初始化，期间保持心跳。 */
            V2_SET_READY(0);
#if !VEHICLE_DEBUG
            expansion_stop();
#endif
            chassis_stop();
            spin_mode = 0;
            gimbal_duty = 0;
            for (index = 0; index < 4; index++)
                wheel_duty[index] = 0;

            V2_SERVICE_WAIT(500);
            while (!imu_init())
            {
#if !VEHICLE_DEBUG
                expansion_stop();
#endif
                V2_SERVICE_WAIT(500);
            }
            V2_SET_READY(1);

            gimbal_set_target(imu_get_gimbal_yaw());
            gimbal_reset();
            last_e = remote_key(REMOTE_KEY_E);
            owner = OWNER_NONE;
            last_owner = OWNER_NONE;
            last_control = millis();
            continue;
        }

        if (remote_connected())
        {
            owner = OWNER_REMOTE;

            if (last_owner != OWNER_REMOTE)
            {
                gimbal_set_target(imu_get_gimbal_yaw());
                gimbal_reset();
                chassis_reset_spin();
                last_e = remote_key(REMOTE_KEY_E);
            }

            if (remote_key(REMOTE_KEY_E) && !last_e)
            {
                spin_mode = !spin_mode;
                chassis_reset_spin();
            }
            last_e = remote_key(REMOTE_KEY_E);

            gimbal_duty = gimbal_update(g_remote.right_h,
                                        imu_get_gimbal_yaw(),
                                        imu_get_gimbal_rate(), dt);

            chassis_update(g_remote.left_v, g_remote.left_h,
                           imu_get_chassis_yaw(), imu_get_gimbal_yaw(),
                           imu_get_chassis_rate(), spin_mode, dt, wheel_duty);
        }
#if !VEHICLE_DEBUG
        else if (v2_motion_take(now, imu_ok, &vx, &vy, &wz))
        {
            owner = OWNER_V2;

            if (last_owner != OWNER_V2)
            {
                gimbal_set_target(imu_get_gimbal_yaw());
                gimbal_reset();
                chassis_reset_spin();
                spin_mode = 0;
            }

            /* V2 没有云台命令：保持当前云台目标，PID 继续工作。 */
            gimbal_duty = gimbal_update(0, imu_get_gimbal_yaw(),
                                        imu_get_gimbal_rate(), dt);
            chassis_update_velocity(vx, vy, wz, dt, wheel_duty);
        }
#endif
        else
        {
            owner = OWNER_NONE;

            if (last_owner != OWNER_NONE)
            {
#if !VEHICLE_DEBUG
                expansion_stop();
#endif
                chassis_stop();
                spin_mode = 0;
                gimbal_duty = 0;
                for (index = 0; index < 4; index++)
                    wheel_duty[index] = 0;
            }
        }
        last_owner = owner;

        if (!remote_connected())
        {
            g_remote.left_v = 0;
            g_remote.left_h = 0;
            g_remote.right_v = 0;
            g_remote.right_h = 0;
            g_remote.keys = 0;
        }

#if !VEHICLE_DEBUG
        if (owner != OWNER_NONE)
        {
            motor_duty[0] = (unsigned short)(gimbal_duty >= 0 ? gimbal_duty : -gimbal_duty);
            motor_dir[0] = (gimbal_duty >= 0) ? 1 : 0;
            motor_duty[1] = 0;
            motor_dir[1] = 1;
            motor_duty[2] = 0;
            motor_dir[2] = 1;
            motor_duty[3] = 0;
            motor_dir[3] = 1;

            for (index = 0; index < 4; index++)
            {
                motor_duty[4 + index] = (unsigned short)(wheel_duty[index] >= 0 ?
                                                         wheel_duty[index] : -wheel_duty[index]);
                motor_dir[4 + index] = (wheel_duty[index] >= 0) ? 1 : 0;
            }

            expansion_apply(motor_dir, motor_duty);
        }
#endif

#if VEHICLE_DEBUG
        if ((unsigned int)(now - last_print) >= 50)
        {
            last_print = now;
            debug_print(gimbal_duty);
        }
#endif
    }
}
