/*
 * UART1 最小驱动。
 *
 * 波特率：Timer2 的 16 位自动重装载模式，1T。
 * 手册公式：reload = 65536 - (FOSC / baud + 2) / 4
 */

#include "stc32g_uart.h"
#include "common.h"

static void uart1_config_pins(unsigned char group)
{
    switch (group & 0x03)
    {
    case 0: /* P3.0 RxD, P3.1 TxD */
        P3M1 = (unsigned char)((P3M1 & (unsigned char)~0x03) | 0x01);
        P3M0 = (unsigned char)((P3M0 & (unsigned char)~0x03) | 0x02);
        break;
    case 1: /* P3.6 RxD, P3.7 TxD */
        P3M1 = (unsigned char)((P3M1 & (unsigned char)~0xC0) | 0x40);
        P3M0 = (unsigned char)((P3M0 & (unsigned char)~0xC0) | 0x80);
        break;
    case 2: /* P1.6 RxD, P1.7 TxD */
        P1M1 = (unsigned char)((P1M1 & (unsigned char)~0xC0) | 0x40);
        P1M0 = (unsigned char)((P1M0 & (unsigned char)~0xC0) | 0x80);
        break;
    default: /* P4.3 RxD, P4.4 TxD */
        P4M1 = (unsigned char)((P4M1 & (unsigned char)~0x18) | 0x08);
        P4M0 = (unsigned char)((P4M0 & (unsigned char)~0x18) | 0x10);
        break;
    }

    P_SW1 = (unsigned char)((P_SW1 & 0x3F) | ((group & 0x03) << 6));
}

void uart1_init(unsigned long baud, unsigned char pin_group)
{
    unsigned int reload;

    if (baud == 0)
        return;

    reload = (unsigned int)(65536UL - ((FOSC / baud + 2UL) / 4UL));

    uart1_config_pins(pin_group);

    SCON = 0x50; /* 模式 1，允许接收 */
    T2L = (unsigned char)(reload & 0xFF);
    T2H = (unsigned char)(reload >> 8);

    /* Timer2：1T、定时模式、启动；串口 1 选择 Timer2 作为波特率发生器。 */
    AUXR = (unsigned char)((AUXR & (unsigned char)~0x08) | 0x04 | 0x10 | 0x01);

    TI = 0;
    RI = 0;
}

void uart1_write_byte(unsigned char data)
{
    unsigned int spin = 0;

    SBUF = data;
    while (!TI)
    {
        /* 超时保护：正常单字节 43 us 量级；TI 异常时不至于死等（看门狗兜底）。 */
        if (++spin >= 8000u)
            break;
    }
    TI = 0;
}

void uart1_write(const char *text)
{
    while (*text)
    {
        uart1_write_byte((unsigned char)*text);
        text++;
    }
}
