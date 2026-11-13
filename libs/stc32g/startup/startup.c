/*
 * 启动代码：复位后的最小初始化。
 *
 * SDCC 的 crt 负责设置 SPX、复制 .data、清零 .bss；这里只做板级配置。
 */

#include "common.h"

/* SDCC 在初始化全局变量之前调用该钩子；返回 0 表示继续默认初始化。 */
unsigned char __sdcc_external_startup(void)
{
    /* 把代码取指和 XRAM 总线都设为最快。 */
    WTST = 0x00;
    CKCON = 0x00;
    return 0;
}

/* 系统时钟由 ISP 下载时选择的 IRC 频率决定，运行期不修改 IRTRIM/LIRTRIM。 */
void system_clock_set(void)
{
}

void board_init(void)
{
    system_clock_set();

    /* 全部端口先恢复为准双向，具体引脚在各自驱动里再配置。 */
    P0M1 = 0x00;
    P0M0 = 0x00;
    P1M1 = 0x00;
    P1M0 = 0x00;
    P2M1 = 0x00;
    P2M0 = 0x00;
    P3M1 = 0x00;
    P3M0 = 0x00;
    P4M1 = 0x00;
    P4M0 = 0x00;
    P5M1 = 0x00;
    P5M0 = 0x00;
    P6M1 = 0x00;
    P6M0 = 0x00;
    P7M1 = 0x00;
    P7M0 = 0x00;

    /* UART1 使用默认引脚 P3.0/P3.1。 */
    P_SW1 &= 0x3F;
}

void disable_global_irq(void)
{
    EA = 0;
}

void enable_global_irq(void)
{
    EA = 1;
}

/*
 * 微秒延时：每轮 20 个 NOP，加循环开销后约 1 µs（35 MHz，1T）。
 * 原 volatile 循环在 MCS251 上实测慢约 4.7 倍，改为固定 NOP 校准。
 */
void us_delay(unsigned int us)
{
    while (us--)
    {
        __asm
            nop
            nop
            nop
            nop
            nop
            nop
            nop
            nop
            nop
            nop
            nop
            nop
            nop
            nop
            nop
            nop
            nop
            nop
            nop
            nop
        __endasm;
    }
}

void ms_delay(unsigned int ms)
{
    while (ms--)
    {
        us_delay(1000);
    }
}

void _nop_(void)
{
    __asm
        nop
    __endasm;
}
