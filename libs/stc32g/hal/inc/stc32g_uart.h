/*
 * UART1 最小驱动：轮询收发，Timer2 作为波特率发生器。
 * 默认引脚 P3.0(RxD)/P3.1(TxD)。
 */

#ifndef STCNAV_STC32G_UART_H
#define STCNAV_STC32G_UART_H

/* pin_group: 0=P3.0/P3.1, 1=P3.6/P3.7, 2=P1.6/P1.7, 3=P4.3/P4.4 */
void uart1_init(unsigned long baud, unsigned char pin_group);
void uart1_write_byte(unsigned char data);
void uart1_write(const char *text);

#endif
