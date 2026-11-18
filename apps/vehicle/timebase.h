/*
 * 1 ms 节拍、延时与 32 位 uptime。实现位于 main.c。
 */

#ifndef STCNAV_VEHICLE_TIMEBASE_H
#define STCNAV_VEHICLE_TIMEBASE_H

unsigned int millis(void);
void delay_ms(unsigned int ms);

/*
 * 32 位毫秒 uptime：把 16 位 millis() 的高位软件扩展。
 * 必须在 65.5 s 内至少调用一次（主循环/IMU 更新每 10 ms 调用）。
 */
unsigned long millis32(void);

#endif
