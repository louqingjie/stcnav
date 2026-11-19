/*
 * UART2（P1.0 = RxD2 / P1.1 = TxD2）收发自检固件，配合 tools/uart2_probe.py。
 *
 * 行为：
 *   1. 上电打印 "U2 READY baud=<baud> fosc=<FOSC>"，验证 MCU -> PC 方向；
 *   2. 空闲 1 s 打印 "U2 HB <n> rx=<n>" 心跳，持续验证 TX；
 *   3. 收到任意字节立即原样回显（透明，不插入任何字节），验证 PC -> MCU 方向；
 *      有接收活动时暂停心跳，保证回显字节流连续。
 *
 * 接线（3.3 V TTL，必须共地）：
 *   USB-TTL TXD -> P1.0（MCU RX）
 *   USB-TTL RXD -> P1.1（MCU TX）
 *   USB-TTL GND <-> MCU GND
 *
 * 注意：不要额外把 P1.0 与 P1.1 短接成自环；回显模式下会形成无限回环。
 */

#include "common.h"
#include "stc32g_timer.h"
#include "stc32g_uart2.h"

/* 115200 最宽容；验证 V2 实际速率时用 -DUART2_ECHO_BAUD=230400UL 重新编译。 */
#ifndef UART2_ECHO_BAUD
#define UART2_ECHO_BAUD 115200UL
#endif

#define TIMER0_RELOAD ((unsigned int)(65536UL - (FOSC / 1000UL)))
#define IDLE_HEARTBEAT_MS 1000u

static __data volatile unsigned int ms_ticks;
static unsigned int rx_total;
static unsigned int heartbeat_total;
static unsigned int last_activity_ms;

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

static void write_ulong(unsigned long value)
{
    static char digits[10];
    unsigned char count = 0;

    do
    {
        digits[count] = (char)('0' + (unsigned char)(value % 10UL));
        value /= 10UL;
        count++;
    } while (value != 0UL);

    while (count != 0u)
        uart2_write_byte((unsigned char)digits[--count]);
}

static void send_banner(void)
{
    uart2_write("\r\nU2 READY baud=");
    write_ulong(UART2_ECHO_BAUD);
    uart2_write(" fosc=");
    write_ulong((unsigned long)FOSC);
    uart2_write("\r\n");
}

static void send_heartbeat(unsigned int uptime_ms)
{
    uart2_write("\r\nU2 HB ");
    write_ulong((unsigned long)heartbeat_total);
    uart2_write(" rx=");
    write_ulong((unsigned long)rx_total);
    uart2_write(" uptime=");
    write_ulong((unsigned long)uptime_ms);
    uart2_write("\r\n");
}

void main(void)
{
    unsigned char value;
    unsigned int now;

    board_init();

    timer0_init(TIMER0_RELOAD);
    ms_ticks = 0;
    ET0 = 1;
    EA = 1;
    timer0_start();

    uart2_init(UART2_ECHO_BAUD);

    send_banner();
    last_activity_ms = millis();

    for (;;)
    {
        if (uart2_rx_ready())
        {
            value = uart2_read_byte();
            uart2_write_byte(value); /* 原样回显：字节流必须与主机发送完全一致 */
            rx_total++;
            last_activity_ms = millis();
            continue;
        }

        now = millis();
        if ((unsigned int)(now - last_activity_ms) >= IDLE_HEARTBEAT_MS)
        {
            heartbeat_total++;
            send_heartbeat(now);
            last_activity_ms = millis();
        }
    }
}
