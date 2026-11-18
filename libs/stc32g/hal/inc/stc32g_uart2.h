/*
 * UART2 最小驱动：轮询收发，Timer2 作为波特率发生器。
 * 固定引脚 P1.0(RxD2) / P1.1(TxD2)，230400 8N1。
 */

#ifndef STCNAV_STC32G_UART2_H
#define STCNAV_STC32G_UART2_H

void uart2_init(unsigned long baud);
void uart2_write_byte(unsigned char data);
void uart2_write(const char *text);
unsigned char uart2_rx_ready(void);
unsigned char uart2_read_byte(void);

#endif
