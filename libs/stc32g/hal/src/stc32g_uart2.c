/*
 * UART2 驱动：轮询 + 中断环形缓冲（256 字节环，8 位索引）。
 *
 * 固定使用 P1.0(RxD2) / P1.1(TxD2)，P_SW2.0 = 0。
 * 波特率与 UART1 共用 Timer2，因此两者必须使用同一波特率。
 *
 * 中断说明（STC32G 手册）：
 *   - UART2 中断向量号 8；
 *   - 中断使能 IE2.1（ES2）；
 *   - S2CON.0 = S2RI，S2CON.1 = S2TI。
 *
 * 环形缓冲索引只在主循环（关 UART2 中断的短临界区）与 UART2 ISR 之间共享。
 * 230400 下单字节省时间约 43 us，临界区 < 2 us，不会丢字节。
 */

#include "stc32g_uart2.h"
#include "common.h"

#define UART2_RING_SIZE 256u
#define UART2_RING_MASK 0xFFu

/* IE2.1 = ES2（UART2 中断使能）。 */
#define UART2_IRQ_OFF() (IE2 = (unsigned char)(IE2 & (unsigned char)~0x02))
#define UART2_IRQ_ON() (IE2 = (unsigned char)(IE2 | 0x02))

static __xdata unsigned char rx_ring[UART2_RING_SIZE];
static __xdata unsigned char tx_ring[UART2_RING_SIZE];

static volatile unsigned char rx_head;
static volatile unsigned char rx_tail;
static volatile unsigned char rx_count;
static volatile unsigned char tx_head;
static volatile unsigned char tx_tail;
static volatile unsigned char tx_count;
static volatile unsigned char tx_active;

static unsigned int rx_overflow;
static unsigned int tx_overflow;

void uart2_init(unsigned long baud)
{
    unsigned int reload;

    if (baud == 0)
        return;

    reload = (unsigned int)(65536UL - ((FOSC / baud + 2UL) / 4UL));

    /* P1.0 高阻输入（RxD2），P1.1 推挽输出（TxD2）。 */
    P1M1 = (unsigned char)((P1M1 & (unsigned char)~0x02) | 0x01);
    P1M0 = (unsigned char)((P1M0 & (unsigned char)~0x01) | 0x02);
    P_SW2 = (unsigned char)(P_SW2 & (unsigned char)~0x01);

    T2L = (unsigned char)(reload & 0xFF);
    T2H = (unsigned char)(reload >> 8);
    AUXR = (unsigned char)(AUXR | 0x14); /* Timer2 1T、启动 */

    S2CON = 0x10; /* 模式 0（8 位 UART），允许接收 */
    S2TI = 0;
    S2RI = 0;
}

void uart2_write_byte(unsigned char data)
{
    S2BUF = data;
    while (!S2TI)
    {
    }
    S2TI = 0;
}

void uart2_write(const char *text)
{
    while (*text)
    {
        uart2_write_byte((unsigned char)*text);
        text++;
    }
}

unsigned char uart2_rx_ready(void)
{
    return S2RI ? 1 : 0;
}

unsigned char uart2_read_byte(void)
{
    unsigned char data = S2BUF;

    S2RI = 0;
    return data;
}

void uart2_init_async(unsigned long baud)
{
    uart2_init(baud);

    rx_head = 0;
    rx_tail = 0;
    rx_count = 0;
    tx_head = 0;
    tx_tail = 0;
    tx_count = 0;
    tx_active = 0;
    rx_overflow = 0;
    tx_overflow = 0;

    S2RI = 0;
    S2TI = 0;
    UART2_IRQ_ON();
}

void uart2_irq_handler(void)
{
    unsigned char data;

    if (S2RI)
    {
        data = S2BUF;
        S2RI = 0;

        if (rx_count < (UART2_RING_SIZE - 1u))
        {
            rx_ring[rx_head] = data;
            rx_head = (unsigned char)((rx_head + 1u) & UART2_RING_MASK);
            rx_count++;
        }
        else
        {
            rx_overflow++;
        }
    }

    if (S2TI)
    {
        S2TI = 0;

        if (tx_count != 0u)
        {
            S2BUF = tx_ring[tx_tail];
            tx_tail = (unsigned char)((tx_tail + 1u) & UART2_RING_MASK);
            tx_count--;
        }
        else
        {
            tx_active = 0;
        }
    }
}

unsigned int uart2_rx_pop(unsigned char *data)
{
    unsigned int got = 0;

    UART2_IRQ_OFF();
    if (rx_count != 0u)
    {
        *data = rx_ring[rx_tail];
        rx_tail = (unsigned char)((rx_tail + 1u) & UART2_RING_MASK);
        rx_count--;
        got = 1;
    }
    UART2_IRQ_ON();

    return got;
}

unsigned char uart2_tx_push(const unsigned char *data, unsigned int length)
{
    unsigned int index;

    if (length == 0u || length > UART2_RING_SIZE)
        return 0;

    UART2_IRQ_OFF();
    if ((unsigned int)tx_count + length > (UART2_RING_SIZE - 1u))
    {
        UART2_IRQ_ON();
        tx_overflow++;
        return 0;
    }

    for (index = 0; index < length; index++)
    {
        tx_ring[tx_head] = data[index];
        tx_head = (unsigned char)((tx_head + 1u) & UART2_RING_MASK);
    }
    tx_count = (unsigned char)(tx_count + length);

    if (!tx_active)
    {
        tx_active = 1;
        S2BUF = tx_ring[tx_tail];
        tx_tail = (unsigned char)((tx_tail + 1u) & UART2_RING_MASK);
        tx_count--;
    }
    UART2_IRQ_ON();

    return 1;
}

unsigned int uart2_tx_overflow_count(void)
{
    return tx_overflow;
}
