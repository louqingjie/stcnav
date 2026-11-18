/*
 * 32 位毫秒 uptime。
 *
 * millis() 由各 app 自己的 Timer0 中断提供（main.c / tune_main.c），
 * 本模块只做 16 位到 32 位的高位扩展。必须在 65.5 s 内至少调用一次。
 */

#include "timebase.h"

static unsigned long ms_ticks_high;
static unsigned int ms_ticks_last;

unsigned long millis32(void)
{
    unsigned int now = millis();

    if (now < ms_ticks_last)
        ms_ticks_high += 0x10000UL;
    ms_ticks_last = now;
    return ms_ticks_high | (unsigned long)now;
}
