/*
 * UART2 最小驱动。
 *
 * 固定使用 P1.0(RxD2) / P1.1(TxD2)，P_SW2.0 = 0。
 * 波特率与 UART1 共用 Timer2，因此两者必须使用同一波特率。
 */

#include "stc32g_uart2.h"
#include "common.h"

void uart2_init(unsigned long baud)
{
    unsigned int reload;

    if (baud == 0)
        return;

    reload = (unsigned int)(65536UL - ((FOSC / baud + 2UL) / 4UL));

    /* P1.0 高阻输入（RxD2），P1.1 推挽输出（TxD2）。 */
    P1M1 = (unsigned char)((P1M1 & (unsigned char)~0x02) | 0x01);
    P1M0 = (unsigned char)((P1M0 & (unsigned char)~0x01) | 0x02);
    P_SW2 = (unsigned char)(P_SW2 & (unsigned char)~0x01);

    T2L = (unsigned char)(reload & 0xFF);
    T2H = (unsigned char)(reload >> 8);
    AUXR = (unsigned char)(AUXR | 0x14); /* Timer2 1T、启动 */

    S2CON = 0x10; /* 模式 0（8 位 UART），允许接收 */
    S2TI = 0;
    S2RI = 0;
}

void uart2_write_byte(unsigned char data)
{
    S2BUF = data;
    while (!S2TI)
    {
    }
    S2TI = 0;
}

void uart2_write(const char *text)
{
    while (*text)
    {
        uart2_write_byte((unsigned char)*text);
        text++;
    }
}

unsigned char uart2_rx_ready(void)
{
    return S2RI ? 1 : 0;
}

unsigned char uart2_read_byte(void)
{
    unsigned char data = S2BUF;

    S2RI = 0;
    return data;
}
