/*
 * 板级公共定义：主频、初始化、延时、全局中断开关。
 *
 * FOSC 必须与 STC-ISP 下载时选择的 IRC 频率一致；本项目按 35 MHz 配置。
 */

#ifndef STCNAV_COMMON_H
#define STCNAV_COMMON_H

#include "STC32Gxx.h"

#ifndef FOSC
#define FOSC 35000000UL
#endif

void board_init(void);
void system_clock_set(void);
void ms_delay(unsigned int ms);
void us_delay(unsigned int us);
void disable_global_irq(void);
void enable_global_irq(void);

#endif
