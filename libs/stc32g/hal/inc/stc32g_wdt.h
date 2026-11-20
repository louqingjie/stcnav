/*
 * 看门狗（独立 WDT）。
 *
 * 依据：STC32G 手册 9.1.1「看门狗控制寄存器 WDT_CONTR」（地址 0xC1）
 *    B7 WDT_FLAG（溢出标志，软件写 0 清）
 *    B6 -
 *    B5 EN_WDT（1=启动）
 *    B4 CLR_WDT（1=清零计数器，硬件自动复位该位）
 *    B3 IDL_WDT（1=IDLE 模式下继续计数）
 *    B2:B0 WDT_PS[2:0]（分频 2/4/8/.../256）
 * 溢出时间 = 12 × 32768 × 2^(WDT_PS+1) / SYSclk。
 *
 * 与官方 STC32G-SOFTWARE-LIB 的 STC32G_WDT.h/.c（EN_WDT/CLR_WDT/IDL_WDT/
 * WDT_PS_Set）一致，只是这里直接整字节写，避免位操作开销。
 */

#ifndef STCNAV_STC32G_WDT_H
#define STCNAV_STC32G_WDT_H

/*
 * ps = 0..7，35 MHz 下溢出时间约：0=22.5ms 1=45ms 2=90ms 3=180ms
 * 4=360ms 5=719ms 6=1.44s 7=2.87s。
 */
void wdt_init(unsigned char ps);

/* 喂狗：清零计数器并保持使能。 */
void wdt_feed(void);

#endif
