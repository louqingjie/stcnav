/*
 * 看门狗驱动。寄存器定义见 stc32g_wdt.h 与 STC32G 手册 9.1.1。
 */

#include "stc32g_wdt.h"
#include "common.h"

/* STC32Gxx.h 未收录 WDT_CONTR（地址 0xC1），按手册 9.1.1 在这里补充声明。 */
__sfr __at (0xC1) WDT_CONTR;

void wdt_init(unsigned char ps)
{
    /* EN_WDT=1、CLR_WDT=1（清零计数）、IDL_WDT=0（IDLE 时停止）、分频 ps。 */
    WDT_CONTR = (unsigned char)(0x30u | (unsigned char)(ps & 0x07u));
}

void wdt_feed(void)
{
    /* 保持使能与分频不变，仅写 CLR_WDT 清零计数器。 */
    WDT_CONTR = (unsigned char)(0x30u | (unsigned char)(WDT_CONTR & 0x07u));
}
