/*
 * 看门狗驱动。寄存器定义见 stc32g_wdt.h 与 STC32G 手册 9.1.1。
 *
 * 注意：不要用“读回 WDT_CONTR 再掩码保留分频位”的方式喂狗——实测该寄存器
 * 读回值不可靠（WDT_PS 位读回为 0），会把分频改回 0，超时从 1.44 s 缩到
 * 约 22 ms，表现为上电后 ~120 ms 一次复位循环。这里用静态变量保存分频值。
 */

#include "stc32g_wdt.h"
#include "common.h"

/* STC32Gxx.h 未收录 WDT_CONTR（地址 0xC1），按手册 9.1.1 在这里补充声明。 */
__sfr __at (0xC1) WDT_CONTR;

/* WDT_PS 保存值（0..7）。 */
static unsigned char wdt_ps_value;

void wdt_init(unsigned char ps)
{
    wdt_ps_value = (unsigned char)(ps & 0x07u);
    /* EN_WDT=1、CLR_WDT=1（清零计数）、IDL_WDT=0（IDLE 时停止）、分频 ps。 */
    WDT_CONTR = (unsigned char)(0x30u | wdt_ps_value);
}

void wdt_feed(void)
{
    /* 整字节写：EN_WDT=1、CLR_WDT=1、分频不变。 */
    WDT_CONTR = (unsigned char)(0x30u | wdt_ps_value);
}
