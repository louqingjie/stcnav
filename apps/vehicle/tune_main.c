/*
 * 云台串口调参固件。
 *
 * UART1 P3.0/P3.1 230400：拓展板（四轮占空比 0，只驱动 P60 云台）。
 * UART2 P1.0(RX)/P1.1(TX) 230400：遥测 + 调参命令。
 *
 * 遥测（50 Hz）：
 *   T,ms,yaw_x10,rate_x10,target_x10,error_x10,duty,dir,kp_x10,ki_x100,kd_x10,mode
 *
 * 命令（ASCII，回车/换行结束）：
 *   P<kp>  I<ki>  D<kd>  B<deadband_deg>  S<target_deg>
 *   E0/E1  关/开闭环      O<duty>  开环占空比（-10000~10000）
 *   M0     停止输出        R       复位积分并把目标设为当前角度
 *   ?      打印参数        K       心跳（2 s 内无命令自动停机）
 */

#include "config.h"
#include "expansion.h"
#include "gimbal.h"
#include "imu.h"
#include "timebase.h"
#include "common.h"
#include "stc32g_timer.h"
#include "stc32g_uart2.h"
#include "stc32g_i2c.h"

#define TIMER0_RELOAD ((unsigned int)(65536UL - (FOSC / 1000UL)))
#define TUNE_PERIOD_MS 20
#define TUNE_TIMEOUT_MS 2000

static __data volatile unsigned int ms_ticks;

static unsigned char motor_dir[8];
static unsigned short motor_duty[8];
static unsigned char mode; /* 0=停止, 1=闭环, 2=开环 */
static signed short open_duty;
static signed short current_duty;
static unsigned char current_dir;

static char cmd_buf[32];
static unsigned char cmd_len;
static unsigned int last_cmd_ms;

void timer0_isr(void) __interrupt (1)
{
    ms_ticks++;
}

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

static float wrap180(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

static long angle_x10(float angle)
{
    if (angle >= 0.0f)
        return (long)(angle * 10.0f + 0.5f);
    return (long)(angle * 10.0f - 0.5f);
}

static void write_long(long value)
{
    static char digits[10];
    unsigned char count = 0;
    unsigned long magnitude;

    if (value < 0)
    {
        uart2_write_byte('-');
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
        uart2_write_byte((unsigned char)digits[--count]);
}

static float parse_number(const char *text)
{
    float value = 0.0f;
    float scale = 0.1f;
    unsigned char negative = 0;
    unsigned char decimal = 0;

    while (*text == ' ')
        text++;

    if (*text == '-')
    {
        negative = 1;
        text++;
    }
    else if (*text == '+')
    {
        text++;
    }

    while (*text)
    {
        if (*text >= '0' && *text <= '9')
        {
            if (!decimal)
            {
                value = value * 10.0f + (float)(*text - '0');
            }
            else
            {
                value += (float)(*text - '0') * scale;
                scale *= 0.1f;
            }
        }
        else if (*text == '.')
        {
            decimal = 1;
        }
        else
        {
            break;
        }
        text++;
    }

    return negative ? -value : value;
}

static void print_params(void)
{
    float kp;
    float ki;
    float kd;

    gimbal_get_gains(&kp, &ki, &kd);

    uart2_write("P,");
    write_long((long)(kp * 10.0f + 0.5f));
    uart2_write_byte(',');
    write_long((long)(ki * 100.0f + 0.5f));
    uart2_write_byte(',');
    write_long((long)(kd * 10.0f + 0.5f));
    uart2_write_byte(',');
    write_long(angle_x10(gimbal_get_deadband()));
    uart2_write_byte(',');
    write_long((long)mode);
    uart2_write("\r\n");
}

static void handle_command(const char *cmd)
{
    char code;
    float value;
    float kp;
    float ki;
    float kd;

    code = cmd[0];
    if (code == '\0')
        return;

    if (code == 'K')
        return;

    if (code == '?')
    {
        print_params();
        return;
    }

    value = parse_number(cmd + 1);

    if (code == 'P')
    {
        gimbal_get_gains(&kp, &ki, &kd);
        gimbal_set_gains(value, ki, kd);
    }
    else if (code == 'I')
    {
        gimbal_get_gains(&kp, &ki, &kd);
        gimbal_set_gains(kp, value, kd);
    }
    else if (code == 'D')
    {
        gimbal_get_gains(&kp, &ki, &kd);
        gimbal_set_gains(kp, ki, value);
    }
    else if (code == 'B')
    {
        gimbal_set_deadband(value);
    }
    else if (code == 'S')
    {
        gimbal_set_target(value);
    }
    else if (code == 'E')
    {
        mode = (value >= 1.0f) ? 1 : 0;
        if (mode == 0)
            open_duty = 0;
    }
    else if (code == 'O')
    {
        if (value > 10000.0f)
            value = 10000.0f;
        if (value < -10000.0f)
            value = -10000.0f;
        open_duty = (signed short)value;
        mode = 2;
    }
    else if (code == 'M')
    {
        mode = 0;
        open_duty = 0;
    }
    else if (code == 'R')
    {
        gimbal_reset();
        gimbal_set_target(imu_get_gimbal_yaw());
        mode = 0;
        open_duty = 0;
    }
    else
    {
        uart2_write("ERR\r\n");
        return;
    }

    print_params();
}

static void poll_commands(void)
{
    unsigned char data;

    while (uart2_rx_ready())
    {
        data = uart2_read_byte();
        last_cmd_ms = millis();

        if (data == '\r' || data == '\n')
        {
            if (cmd_len != 0)
            {
                cmd_buf[cmd_len] = '\0';
                handle_command(cmd_buf);
                cmd_len = 0;
            }
        }
        else if (cmd_len < (unsigned char)(sizeof(cmd_buf) - 1))
        {
            cmd_buf[cmd_len++] = (char)data;
        }
    }
}

static void print_telemetry(unsigned int now, float yaw, float rate)
{
    float kp;
    float ki;
    float kd;

    gimbal_get_gains(&kp, &ki, &kd);

    uart2_write("T,");
    write_long((long)now);
    uart2_write_byte(',');
    write_long(angle_x10(yaw));
    uart2_write_byte(',');
    write_long(angle_x10(rate));
    uart2_write_byte(',');
    write_long(angle_x10(gimbal_get_target()));
    uart2_write_byte(',');
    write_long(angle_x10(wrap180(gimbal_get_target() - yaw)));
    uart2_write_byte(',');
    write_long((long)current_duty);
    uart2_write_byte(',');
    uart2_write_byte(current_dir ? '1' : '0');
    uart2_write_byte(',');
    write_long((long)(kp * 10.0f + 0.5f));
    uart2_write_byte(',');
    write_long((long)(ki * 100.0f + 0.5f));
    uart2_write_byte(',');
    write_long((long)(kd * 10.0f + 0.5f));
    uart2_write_byte(',');
    write_long((long)mode);
    uart2_write("\r\n");
}

void main(void)
{
    unsigned int now;
    unsigned int last_control;
    unsigned int elapsed;
    float dt;
    float yaw;
    float rate;
    signed short duty;
    unsigned char index;

    timer0_init(TIMER0_RELOAD);
    EA = 1;
    timer0_start();

    expansion_init();
    uart2_init(230400UL);
    uart2_write("TUNE BOOT\r\n");

    i2c_init();
    while (!imu_init())
    {
        uart2_write("IMU FAIL\r\n");
        expansion_stop();
        delay_ms(500);
    }
    uart2_write("IMU OK\r\n");

    gimbal_reset();
    gimbal_set_target(0.0f);
    gimbal_set_gains(GIMBAL_KP, GIMBAL_KI, GIMBAL_KD);
    gimbal_set_deadband(GIMBAL_DEADBAND_DEG);

    uart2_write("T,ms,yaw_x10,rate_x10,target_x10,error_x10,duty,dir,kp_x10,ki_x100,kd_x10,mode\r\n");
    uart2_write("READY\r\n");

    mode = 0;
    open_duty = 0;
    current_duty = 0;
    current_dir = 1;
    last_cmd_ms = millis();
    last_control = last_cmd_ms;

    for (;;)
    {
        poll_commands();

        now = millis();
        if ((unsigned int)(now - last_control) < TUNE_PERIOD_MS)
            continue;

        elapsed = (unsigned int)(now - last_control);
        last_control = now;
        dt = (float)elapsed * 0.001f;
        if (dt > 0.2f)
            dt = 0.2f;

        if (!imu_update(dt))
            mode = 0;

        yaw = imu_get_gimbal_yaw();
        rate = imu_get_gimbal_rate();

        if (mode == 1)
            duty = gimbal_update(0, yaw, rate, dt);
        else if (mode == 2)
            duty = open_duty;
        else
            duty = 0;

        if (mode != 0 && (unsigned int)(now - last_cmd_ms) > TUNE_TIMEOUT_MS)
        {
            mode = 0;
            open_duty = 0;
            duty = 0;
            uart2_write("T,TIMEOUT\r\n");
        }

        for (index = 0; index < 8; index++)
        {
            motor_dir[index] = 1;
            motor_duty[index] = 0;
        }

        if (duty >= 0)
        {
            current_dir = 1;
            current_duty = duty;
        }
        else
        {
            current_dir = 0;
            current_duty = duty;
        }
        motor_dir[0] = current_dir;
        motor_duty[0] = (unsigned short)(duty >= 0 ? duty : -duty);

        expansion_apply(motor_dir, motor_duty);
        print_telemetry(now, yaw, rate);
    }
}
