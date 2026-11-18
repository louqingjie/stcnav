/*
 * PIE-Block 遥控接收：NRF24L01 软件 SPI + 12 字节遥控协议。
 */

#ifndef STCNAV_VEHICLE_REMOTE_H
#define STCNAV_VEHICLE_REMOTE_H

typedef struct
{
    signed short left_v;
    signed short left_h;
    signed short right_v;
    signed short right_h;
    unsigned int keys;
    unsigned char connected;
} remote_state_t;

extern remote_state_t g_remote;

unsigned char remote_init(void);
unsigned char remote_poll(unsigned int now_ms);
unsigned char remote_connected(void);
unsigned char remote_key(unsigned char bit);

#endif
