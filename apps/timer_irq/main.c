/*
 * M2 验收：Timer0 1 kHz 计数。
 * 每累计 1000 次中断打印一次 PASS；用秒表观察应为 1 秒一次。
 */

#include "common.h"
#include "stc32g_timer.h"
#include "stc32g_uart.h"

#define TIMER0_RELOAD ((unsigned int)(65536UL - (FOSC / 1000UL)))

static volatile unsigned long ticks;

void timer0_isr(void) __interrupt (1)
{
    ticks++;
}

void main(void)
{
    unsigned long last = 0;

    board_init();
    stc32g_uart1_init(115200UL, 0);
    stc32g_timer0_init(TIMER0_RELOAD);

    ET0 = 1;
    EA = 1;
    stc32g_timer0_start();

    while (1)
    {
        if ((unsigned long)(ticks - last) >= 1000UL)
        {
            last += 1000UL;
            stc32g_uart1_write("PASS\r\n");
        }
    }
}
