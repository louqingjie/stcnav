/*
 * 看门狗驱动。寄存器定义与溢出时间公式见 stc32g_wdt.h 与 STC32G 手册 9.1.1。
 */

#include "stc32g_wdt.h"
#include "common.h"

/* STC32Gxx.h 未收录 WDT_CONTR（地址 0xC1），按手册 9.1.1 在这里补充声明。 */
__sfr __at (0xC1) WDT_CONTR;

/* WDT_PS 保存值（0..7）；不依赖 WDT_CONTR 读回。 */
static unsigned char wdt_ps_value;

void wdt_init(unsigned char ps)
{
    wdt_ps_value = (unsigned char)(ps & 0x07u);
    /* EN_WDT=1、CLR_WDT=1（清零计数）、IDL_WDT=0、WDT_PS=ps、WDT_FLAG=0。 */
    WDT_CONTR = (unsigned char)(0x30u | wdt_ps_value);
}

void wdt_feed(void)
{
    /* 整字节写：EN_WDT=1、CLR_WDT=1、分频不变。 */
    WDT_CONTR = (unsigned char)(0x30u | wdt_ps_value);
}
