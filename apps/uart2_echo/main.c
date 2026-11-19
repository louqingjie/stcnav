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
#define IDLE_REPORT_MS 300u

static __data volatile unsigned int ms_ticks;
static unsigned int rx_total;
static unsigned int report_total;
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

/*
 * 状态行同时兼作上电横幅与心跳：
 *   U2 READY baud=<baud> fosc=<FOSC> hb=<n> rx=<n>
 * 主机收到它即可判定 P1.1 -> PC 方向正常。
 */
static void send_status(void)
{
    uart2_write("\r\nU2 READY baud=");
    write_ulong(UART2_ECHO_BAUD);
    uart2_write(" fosc=");
    write_ulong((unsigned long)FOSC);
    uart2_write(" hb=");
    write_ulong((unsigned long)report_total);
    uart2_write(" rx=");
    write_ulong((unsigned long)rx_total);
    uart2_write("\r\n");
}

void main(void)
{
    unsigned char value;
    unsigned int now;

    board_init();
    uart2_init(UART2_ECHO_BAUD);

    /*
     * 先连发 3 次状态行，再打开 Timer0 中断：
     * 即使中断向量区没烧好、程序在 1 ms 后跑飞，也能证明 P1.1 有输出。
     */
    send_status();
    ms_delay(20);
    send_status();
    ms_delay(20);
    send_status();

    timer0_init(TIMER0_RELOAD);
    ms_ticks = 0;
    ET0 = 1;
    EA = 1;
    timer0_start();

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
        if ((unsigned int)(now - last_activity_ms) >= IDLE_REPORT_MS)
        {
            report_total++;
            send_status();
            last_activity_ms = millis();
        }
    }
}
