/*
 * UART2 驱动：轮询模式（调参）与中断异步模式（V2 协议链路）。
 * 固定引脚 P1.0(RxD2) / P1.1(TxD2)，P_SW2.0 = 0。
 *
 * 波特率由 Timer2 产生；UART1 也使用 Timer2 作为波特率发生器，
 * 因此 UART1 与 UART2 必须使用同一波特率（本项目 230400）。
 */

#ifndef STCNAV_STC32G_UART2_H
#define STCNAV_STC32G_UART2_H

/* ---------- 轮询模式（vehicle_tune） ---------- */
void uart2_init(unsigned long baud);
void uart2_write_byte(unsigned char data);
void uart2_write(const char *text);
unsigned char uart2_rx_ready(void);
unsigned char uart2_read_byte(void);

/*
 * ---------- 中断异步模式（vehicle V2 协议链路） ----------
 *
 * uart2_init_async() 会打开 UART2 中断使能（IE2.0/ES2，向量 8）。
 * vehicle 构建在 v2_link.c 中定义中断服务程序：
 *     void uart2_isr(void) __interrupt (8) { uart2_irq_handler(); }
 * 并保证 EA = 1。
 *
 * 异步模式与轮询模式不要混用：uart2_write_byte / uart2_read_byte
 * 只供纯轮询应用使用。
 */
void uart2_init_async(unsigned long baud);
void uart2_irq_handler(void);

/* 主循环侧接口：RX 取一字节（返回 1 表示取到）；TX 整帧入队（0=空间不足）。 */
unsigned int uart2_rx_pop(unsigned char *data);
unsigned char uart2_tx_push(const unsigned char *data, unsigned int length);

/*
 * TX 停滞看门狗（中断异步模式下务必在主循环里周期调用，建议每轮一次）：
 * 连续若干 ms 没有装填新字节时丢弃当前半帧并复位 TX 环，返回 1。
 * 用于兜底 S2TI 被意外覆盖导致的"TX 永久停在半帧"（现象：静默、不复位）。
 */
unsigned char uart2_tx_service(unsigned int now_ms);

unsigned int uart2_tx_overflow_count(void);
unsigned int uart2_tx_stall_count(void);

#endif