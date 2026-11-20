/*
 * 启动/外设进度探针（放在 0xFE0400，与 vehicle 相同布局），循环输出：
 *
 *   ---- 轮询 UART2，115200 ----
 *   BT 2 IAP=0xXX      读 EEPROM 首扇区（0xFE0000）的值
 *   BT 3 IAPWR ok      擦除 + 写 0xFE0000 的结果
 *   BT 4 T<n>          Timer0 中断累计秒数（证明 Timer0 ISR 在跑）
 *   BT 5 POLL ok       轮询方式 UART2 发送正常
 *   ---- 中断异步 UART2，230400 ----
 *   BT 7 ASYNC ok      中断 + 环形缓冲方式 UART2 发送正常
 *
 * 这样无论何时开始监听，两个波特率下都能分别看到对应标记：
 *   115200 看到 BT 2/3/4/5  => 基址 0xFE0400 烧录正常、IAP 正常、Timer0 正常、轮询发送正常
 *   230400 看到 BT 7        => vehicle 使用的异步 UART2 发送路径正常
 *
 * 上电时还会各打印一次 BT 1 UART2 / BT 6 UART1（UART1 在 P3.0/P3.1，本机不可观测）。
 */

#include "common.h"
#include "stc32g_timer.h"
#include "stc32g_uart.h"
#include "stc32g_uart2.h"
#include "stc32g_iap.h"

#define TIMER0_RELOAD ((unsigned int)(65536UL - (FOSC / 1000UL)))
#define PROBE_SECTOR 0xFE0000UL

static __data volatile unsigned int ms_ticks;

/* UART2 中断服务程序（向量 8）：异步模式使用。 */
void uart2_isr(void) __interrupt (8)
{
    uart2_irq_handler();
}

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
    static const unsigned char async_msg[] = "BT 7 ASYNC ok\r\n";
    unsigned char value;
    unsigned char failed;

    board_init();

    /* 上电一次性标记：轮询 UART2 */
    uart2_init(115200UL);
    uart2_write("\r\nBT 1 UART2 115200\r\n");

    /* 上电一次性标记：UART1（P3.0/P3.1，本机不可观测，只验证不卡死） */
    uart1_init(230400UL, 0);
    uart1_write("X");
    uart2_write("BT 6 UART1 ok\r\n");

    /* Timer0 1 ms 中断 */
    timer0_init(TIMER0_RELOAD);
    ms_ticks = 0;
    ET0 = 1;
    EA = 1;
    timer0_start();

    for (;;)
    {
        unsigned int seconds;

        /* ---------------- 轮询模式 115200 ---------------- */
        IE2 = (unsigned char)(IE2 & (unsigned char)~0x01); /* 关 UART2 中断 */
        uart2_init(115200UL);

        /* IAP：读 EEPROM 首扇区 */
        uart2_write("BT 2 IAP=0x");
        iap_begin();
        value = iap_read_byte(PROBE_SECTOR);
        failed = iap_failed();
        iap_end();
        write_hex8(value);
        uart2_write(failed ? " FAIL\r\n" : "\r\n");

        /* IAP：擦除 + 写入 */
        iap_begin();
        failed = 0u;
        if (iap_erase_sector(PROBE_SECTOR))
            failed = 1u;
        if (!failed && iap_write_byte(PROBE_SECTOR, 0x5Au))
            failed = 1u;
        if (iap_failed())
            failed = 1u;
        iap_end();
        uart2_write(failed ? "BT 3 IAPWR FAIL\r\n" : "BT 3 IAPWR ok\r\n");

        /* Timer0 是否在跑 */
        seconds = (unsigned int)(millis() / 1000u);
        uart2_write("BT 4 T");
        write_uint(seconds);
        uart2_write("\r\n");

        uart2_write("BT 5 POLL ok\r\n");
        ms_delay(400);

        /* ---------------- 中断异步模式 230400 ---------------- */
        uart2_init_async(230400UL);
        uart2_tx_push(async_msg, (unsigned int)(sizeof(async_msg) - 1u));
        ms_delay(400);
    }
}
