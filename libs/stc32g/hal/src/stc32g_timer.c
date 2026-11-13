/*
 * Timer0 最小驱动。
 * reload = 65536 - FOSC / tick_rate（1T 模式）。
 */

#include "stc32g_timer.h"
#include "common.h"

void stc32g_timer0_init(unsigned int reload)
{
    TR0 = 0;
    TMOD = (unsigned char)(TMOD & 0xF0); /* Timer0 模式 0：16 位自动重载 */
    AUXR |= 0x80;                        /* T0x12：1T */
    TH0 = (unsigned char)(reload >> 8);
    TL0 = (unsigned char)(reload & 0xFF);
}

void stc32g_timer0_start(void)
{
    TR0 = 1;
}

void stc32g_timer0_stop(void)
{
    TR0 = 0;
}
