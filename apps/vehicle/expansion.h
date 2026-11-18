/*
 * PIE-Block 拓展板 21 字节协议。
 */

#ifndef STCNAV_VEHICLE_EXPANSION_H
#define STCNAV_VEHICLE_EXPANSION_H

void expansion_init(void);
void expansion_apply(const unsigned char dir[8], const unsigned short duty[8]);
void expansion_stop(void);

#endif
