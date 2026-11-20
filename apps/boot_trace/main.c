/*
 * 启动进度探针（放在 0xFE0400，与 vehicle 相同布局）。
 *
 * 用途：vehicle 固件上电无任何输出，用它逐段打印，定位是"0xFE0400 区没被烧录/
 * 不运行"，还是"UART2 初始化""IAP（boot_id）""Timer0 中断"某一步卡死。
 *
 * 输出（UART2 P1.0/P1.1，115200）：
 *   BT 1 <uart2_init 完成>
 *   BT 2 IAP READ ... -> 读 0xFE0000 的值
 *   BT 3 IAP WRITE ... -> 擦除+写 0xFE0000 的结果
 *   BT 4 TIMER0 -> Timer0 中断已开
 *   BT 5 H<n> -> 每 500 ms 一行（证明主循环还活着）
 */

#include "common.h"
#include "stc32g_timer.h"
#include "stc32g_uart2.h"
#include "stc32g_iap.h"

#define TIMER0_RELOAD ((unsigned int)(65536UL - (FOSC / 1000UL)))
#define PROBE_SECTOR 0xFE0000UL

static __data volatile unsigned int ms_ticks;

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

static void write_hex8(unsigned char value)
{
    static const char hex[] = "0123456789ABCDEF";

    uart2_write_byte((unsigned char)hex[value >> 4]);
    uart2_write_byte((unsigned char)hex[value & 0x0Fu]);
}

static void write_uint(unsigned int value)
{
    static char digits[5];
    unsigned char count = 0;

    do
    {
        digits[count] = (char)('0' + (unsigned char)(value % 10u));
        value /= 10u;
        count++;
    } while (value != 0u);

    while (count != 0u)
        uart2_write_byte((unsigned char)digits[--count]);
}

void main(void)
{
    unsigned char value;
    unsigned char ok;
    unsigned int last;
    unsigned int count = 0;

    board_init();
    uart2_init(115200UL);
    uart2_write("\r\nBT 1 UART2 OK\r\n");

    uart2_write("BT 2 IAP READ ...\r\n");
    iap_begin();
    value = iap_read_byte(PROBE_SECTOR);
    ok = (unsigned char)(iap_failed() ? 0u : 1u);
    iap_end();
    uart2_write("BT 2 IAP READ done: value=0x");
    write_hex8(value);
    uart2_write(ok ? " ok\r\n" : " FAIL\r\n");

    uart2_write("BT 3 IAP WRITE ...\r\n");
    iap_begin();
    ok = 1u;
    if (iap_erase_sector(PROBE_SECTOR))
        ok = 0u;
    if (ok && iap_write_byte(PROBE_SECTOR, 0x5Au))
        ok = 0u;
    if (iap_failed())
        ok = 0u;
    iap_end();
    uart2_write(ok ? "BT 3 IAP WRITE ok\r\n" : "BT 3 IAP WRITE FAIL\r\n");

    timer0_init(TIMER0_RELOAD);
    ms_ticks = 0;
    ET0 = 1;
    EA = 1;
    timer0_start();
    uart2_write("BT 4 TIMER0 OK\r\n");

    last = millis();
    for (;;)
    {
        unsigned int now = millis();

        if ((unsigned int)(now - last) >= 500u)
        {
            last = now;
            uart2_write("BT 5 H");
            write_uint(count);
            uart2_write("\r\n");
            count++;
        }
    }
}
