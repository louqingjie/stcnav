/*
 * DPXL 复现：主循环用 24 位 __code 指针读常量表，Timer0 ISR 写 XRAM。
 * 若编译器 ISR 不保存 DPXL，常量表读取会被破坏。
 */

#include "common.h"
#include "stc32g_timer.h"
#include "stc32g_uart.h"

#define TABLE_SIZE 16
#define TIMER0_RELOAD ((unsigned int)(65536UL - (FOSC / 1000UL)))

static const unsigned char __code code_table[TABLE_SIZE] = {
    0x5A, 0xA5, 0x3C, 0xC3, 0x0F, 0xF0, 0x69, 0x96,
    0x12, 0x21, 0x7E, 0xE7, 0x81, 0x18, 0x2D, 0xD2
};

static volatile __xdata unsigned char xram_sink;
static volatile unsigned char failures;

void timer0_isr(void) __interrupt (1)
{
    xram_sink++;
}

void main(void)
{
    unsigned char index;
    const unsigned char __code *pointer;
    unsigned char value;

    board_init();
    uart1_init(115200UL, 0);
    timer0_init(TIMER0_RELOAD);

    ET0 = 1;
    EA = 1;
    timer0_start();

    while (1)
    {
        pointer = code_table;
        for (index = 0; index < TABLE_SIZE; index++)
        {
            value = *pointer++;
            if (value != code_table[index])
            {
                failures = 1;
                break;
            }
        }

        if (failures)
        {
            uart1_write("FAIL\r\n");
            failures = 0;
        }
        else
        {
            uart1_write("PASS\r\n");
        }
    }
}
