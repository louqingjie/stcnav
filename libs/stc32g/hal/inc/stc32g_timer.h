/*
 * Timer0 最小驱动：模式 0（16 位自动重载），1T。
 * 中断允许与中断服务程序由调用方负责。
 */

#ifndef STCNAV_STC32G_TIMER_H
#define STCNAV_STC32G_TIMER_H

void timer0_init(unsigned int reload);
void timer0_start(void);
void timer0_stop(void);

#endif
