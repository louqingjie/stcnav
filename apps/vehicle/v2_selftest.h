/*
 * V2 协议 golden vector / 流解码自检。
 *
 * 返回 0 表示全部通过；非 0 为失败步骤编号（便于串口打印定位）。
 * 该文件不依赖 MCU 头文件，可在宿主机 gcc 下运行。
 */

#ifndef STCNAV_VEHICLE_V2_SELFTEST_H
#define STCNAV_VEHICLE_V2_SELFTEST_H

unsigned char v2_selftest(void);

#endif
