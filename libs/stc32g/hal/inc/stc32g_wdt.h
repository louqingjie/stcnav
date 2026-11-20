/*
 * 看门狗（WDT）。
 *
 * 依据：STC32G 系列技术手册 9.1.1「看门狗控制寄存器（WDT_CONTR）」（第 507 页）
 *   WDT_CONTR 地址 C1H，复位值 0x00
 *   B7 WDT_FLAG（溢出标志，软件写 0 清）
 *   B6 -（保留）
 *   B5 EN_WDT（1=启动看门狗）
 *   B4 CLR_WDT（1=清零看门狗计数器，硬件自动复位该位）
 *   B3 IDL_WDT（1=IDLE 模式继续计数；本工程 0）
 *   B2:B0 WDT_PS[2:0]（分频 2/4/8/16/32/64/128/256）
 * 溢出时间 = 12 × 32768 × 2^(WDT_PS+1) / SYSclk（手册第 507 页公式）。
 *
 * 注意（实测坑，见 apps/vehicle/docs/stm_v2_implementation.md §11.1）：
 * 不要用「读回 WDT_CONTR 再掩码保留 WDT_PS」的方式喂狗——该寄存器读回值
 * 不可靠（WDT_PS 位读回为 0），会把分频写回 0，超时从 1.44 s 变成约 22 ms。
 * 这里用静态变量保存分频值，每次整字节写。
 */

#ifndef STCNAV_STC32G_WDT_H
#define STCNAV_STC32G_WDT_H

/*
 * ps = 0..7；35 MHz 下溢出时间约：
 * 0=22.5ms 1=45ms 2=90ms 3=180ms 4=360ms 5=719ms 6=1.44s 7=2.87s。
 */
void wdt_init(unsigned char ps);

/* 喂狗：EN_WDT=1、CLR_WDT=1、分频不变。 */
void wdt_feed(void);

#endif
