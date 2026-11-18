/*
 * 云台 USB HID 调参固件。
 *
 * 复用 stcnav 的通用 HID（VID 34BF / PID FF01，EP1 64 字节）：
 *   PC -> 设备：64 字节报告，内容为 ASCII 命令（补 0）
 *   设备 -> PC：64 字节报告，内容为 ASCII 遥测/状态（补 0）
 *
 * 遥测（50 Hz）：
 *   T,ms,yaw_x10,rate_x10,target_x10,duty,dir,mode
 *
 * 命令：
 *   P<kp> I<ki> D<kd> B<deadband> S<target>
 *   E0/E1  O<duty>  M0  R  ?  K
 */

#include "config.h"
#include "expansion.h"
#include "gimbal.h"
#include "imu.h"
#include "timebase.h"
#include "common.h"
#include "stc32g_timer.h"
#include "stc32g_usb_hid.h"
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

static unsigned char hid_tx[USB_HID_REPORT_SIZE];
static unsigned char hid_len;

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

static void hid_flush(void)
{
    if (hid_len != 0)
    {
        usb_hid_write(hid_tx, hid_len);
        hid_len = 0;
    }
}

static void hid_putc(char data)
{
    if (hid_len >= (unsigned char)(USB_HID_REPORT_SIZE - 1))
        hid_flush();
    hid_tx[hid_len++] = (unsigned char)data;
    if (data == '\n')
        hid_flush();
}

static void hid_puts(const char *text)
{
    while (*text)
        hid_putc(*text++);
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

static void hid_put_long(long value)
{
    static char digits[10];
    unsigned char count = 0;
    unsigned long magnitude;

    if (value < 0)
    {
        hid_putc('-');
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
        hid_putc(digits[--count]);
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

    hid_puts("P,");
    hid_put_long((long)(kp * 10.0f + 0.5f));
    hid_putc(',');
    hid_put_long((long)(ki * 100.0f + 0.5f));
    hid_putc(',');
    hid_put_long((long)(kd * 10.0f + 0.5f));
    hid_putc(',');
    hid_put_long(angle_x10(gimbal_get_deadband()));
    hid_putc(',');
    hid_put_long((long)mode);
    hid_puts("\r\n");
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
        hid_puts("ERR\r\n");
        return;
    }

    print_params();
}

static void poll_commands(void)
{
    static unsigned char rx[USB_HID_REPORT_SIZE];
    unsigned char length;
    unsigned char index;
    char data;

    length = usb_hid_read(rx);
    if (length == 0)
        return;

    last_cmd_ms = millis();

    for (index = 0; index < length; index++)
    {
        data = (char)rx[index];
        if (data == '\0')
            break;

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
            cmd_buf[cmd_len++] = data;
        }
    }
}

static void print_telemetry(unsigned int now, float yaw, float rate)
{
    hid_puts("T,");
    hid_put_long((long)now);
    hid_putc(',');
    hid_put_long(angle_x10(yaw));
    hid_putc(',');
    hid_put_long(angle_x10(rate));
    hid_putc(',');
    hid_put_long(angle_x10(gimbal_get_target()));
    hid_putc(',');
    hid_put_long((long)current_duty);
    hid_putc(',');
    hid_putc(current_dir ? '1' : '0');
    hid_putc(',');
    hid_put_long((long)mode);
    hid_puts("\r\n");
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
    usb_hid_init();

    i2c_init();
    while (!imu_init())
    {
        hid_puts("IMU FAIL\r\n");
        expansion_stop();
        delay_ms(500);
    }
    hid_puts("IMU OK\r\n");

    gimbal_reset();
    gimbal_set_target(0.0f);
    gimbal_set_gains(GIMBAL_KP, GIMBAL_KI, GIMBAL_KD);
    gimbal_set_deadband(GIMBAL_DEADBAND_DEG);

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
            hid_puts("T,TIMEOUT\r\n");
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
