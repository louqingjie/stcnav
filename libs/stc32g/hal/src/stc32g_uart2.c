/*
 * UART2 驱动：轮询 + 中断环形缓冲（256 字节环，8 位索引）。
 *
 * 固定使用 P1.0(RxD2) / P1.1(TxD2)，P_SW2.0 = 0。
 * 波特率与 UART1 共用 Timer2，因此两者必须使用同一波特率。
 *
 * 中断说明（STC32G 手册）：
 *   - UART2 中断向量号 8；
 *   - 中断使能 IE2.0（ES2）；IE2.1 是 ESPI（SPI），不要写错；
 *   - S2CON.0 = S2RI，S2CON.1 = S2TI。S2CON(0x9A) 不可位寻址，只能用
 *     MCS-251 的 SFR 位清指令或字节掩码访问。写成 __sbit __at (0x9A/0x9B)
 *     会别名到 SCON(0x98).2/3 = RB8/TB8（UART1）。
 *
 * 环形缓冲索引只在主循环（关 UART2 中断的短临界区）与 UART2 ISR 之间共享。
 * 230400 下单字节省时间约 43 us，临界区 < 2 us，不会丢字节。
 *
 * 标志清零方式（现场"跑十几秒后静默"的根因，勿改回整字节掩码）：
 *   手册第 1397/1398 页的接收例程与 STC32G-SOFTWARE-LIB 都是逐位清
 *   S2RI/S2TI（Keil C251 生成 MCS-251 的 A9 位操作指令）。
 *   若写成 S2CON = S2CON & ~S2RI，编译结果是整字节读-改-写（anl _S2CON,#0xFE）：
 *   在该指令读与写之间，UART 可能刚好发送完成置位 S2TI，写回会把 S2TI 覆盖成 0。
 *   一旦丢掉 S2TI，tx_active 永远停在 1、再没人装填 S2BUF，TX 永久停在半帧，
 *   芯片不复位也不再发帧（只能断电），与现场症状完全一致。
 *   因此这里用 s2_clear_ri()/s2_clear_ti() 的位清指令，并先处理发送再处理接收；
 *   另外 uart2_tx_service() 在主循环里做 TX 停滞看门狗兜底。
 */

#include "stc32g_uart2.h"
#include "common.h"

#define UART2_RING_SIZE 256u
#define UART2_RING_MASK 0xFFu

/* TX 链停滞判定：单字节最长 43 us、最长 65 B 帧约 2.8 ms，且每字节都会刷新
 * tx_progress，所以 3 ms 无进度即可判定 S2TI 丢失/发送链断了。 */
#define UART2_TX_STALL_MS 3u

/* 轮询发送等待上限（约 1~2 ms；正常单字节 43 us 量级）。 */
#define UART2_TX_SPIN_LIMIT 8000u

/* IE2.0 = ES2（UART2 中断使能）。 */
#define UART2_IRQ_OFF() (IE2 = (unsigned char)(IE2 & (unsigned char)~0x01))
#define UART2_IRQ_ON() (IE2 = (unsigned char)(IE2 | 0x01))

static __xdata unsigned char rx_ring[UART2_RING_SIZE];
static __xdata unsigned char tx_ring[UART2_RING_SIZE];

static volatile unsigned char rx_head;
static volatile unsigned char rx_tail;
static volatile unsigned char rx_count;
static volatile unsigned char tx_head;
static volatile unsigned char tx_tail;
static volatile unsigned char tx_count;
static volatile unsigned char tx_active;
/* ISR 每向 S2BUF 装一个字节 +1，供 uart2_tx_service() 判定 TX 是否有进度。 */
static volatile unsigned char tx_progress;

static unsigned int rx_overflow;
static unsigned int tx_overflow;
static unsigned int tx_stalls;

/*
 * MCS-251 SFR 位清指令（A9 前缀），与 Keil C251 / STC32G-SOFTWARE-LIB 的
 * CLR S2RI / CLR S2TI 等价；本仓库 sdas251 支持 `clr 0x9A.0`（编码 A9 C0 9A）。
 * 不能写成 __sbit（见文件头注释），也不能用整字节读-改-写。
 */
static void s2_clear_ri(void)
{
    __asm
        clr 0x9A.0
    __endasm;
}

static void s2_clear_ti(void)
{
    __asm
        clr 0x9A.1
    __endasm;
}

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

    /*
     * 串口2 模式 1：8 位数据、可变波特率，并允许接收（S2REN=1）。
     *
     * STC32G 的 S2CON 是 4 模式结构（S2SM0=S2CON^7、S2SM1=S2CON^6，见官方
     * STC32G.H 与 STC32G-SOFTWARE-LIB 中 UART_8bit_BRTx=(1<<6)）：
     *   00=模式0 同步移位，01=模式1 8位UART，10=模式2 9位固定，11=模式3 9位可变。
     * 写成 0x10（S2SM1=0）会落到模式0 同步移位，P1.1 输出的是移位时钟而不是
     * UART 帧，CH340/上位机只能收到噪声。官方库初始化后的 S2CON 即 0x50。
     * 写整个寄存器同时清 S2RI/S2TI。
     */
    S2CON = 0x50;
}

void uart2_write_byte(unsigned char data)
{
    /* 轮询模式专用；加超时，避免 TI 异常时死等（调用方由看门狗兜底）。 */
    unsigned int spin = 0;

    S2BUF = data;
    while (!(S2CON & S2CON_S2TI))
    {
        if (++spin >= UART2_TX_SPIN_LIMIT)
            break;
    }
    s2_clear_ti();
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
    return (S2CON & S2CON_S2RI) ? 1 : 0;
}

unsigned char uart2_read_byte(void)
{
    unsigned char data = S2BUF;

    s2_clear_ri();
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
    tx_progress = 0;
    rx_overflow = 0;
    tx_overflow = 0;
    tx_stalls = 0;

    s2_clear_ri();
    s2_clear_ti();
    UART2_IRQ_ON();
}

/* 从 TX 环取下一字节装进 S2BUF；无数据则关闭 tx_active。仅主/ISR 单方调用。 */
static void uart2_tx_load_next(void)
{
    if (tx_count != 0u)
    {
        S2BUF = tx_ring[tx_tail];
        tx_tail = (unsigned char)((tx_tail + 1u) & UART2_RING_MASK);
        tx_count--;
        tx_progress++;
    }
    else
    {
        tx_active = 0;
    }
}

void uart2_irq_handler(void)
{
    unsigned char flags;
    unsigned char data;

    /* 一次快照；先发送后接收（发送分支清 TI 后至少 43 us 才会再置位）。 */
    flags = S2CON;

    if (flags & S2CON_S2TI)
    {
        s2_clear_ti();
        uart2_tx_load_next();
    }

    if (flags & S2CON_S2RI)
    {
        data = S2BUF;
        s2_clear_ri();

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

    /*
     * 兜底：若接收处理期间恰好有新的发送完成（位清指令不影响 S2TI 时），
     * 这里再服务一次，避免漏掉 TI 造成 TX 停摆。
     */
    if (!(flags & S2CON_S2TI) && (S2CON & S2CON_S2TI))
    {
        s2_clear_ti();
        uart2_tx_load_next();
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

/*
 * TX 停滞看门狗：uart2_irq_handler() 的位清指令已把 S2TI 被覆盖的概率压到
 * 最低，但只要还有"接收处理期间恰好发送完成"的时序交集，就不能排除 S2TI
 * 丢失。这里在主循环里按进度判定：连续 UART2_TX_STALL_MS 没有装填新字节，
 * 就丢弃当前半帧并复位 TX 环（上位机按 52 4D 重同步），让链路自动恢复，
 * 而不是永久静默到断电。返回 1 表示本次做了一次恢复。
 */
unsigned char uart2_tx_service(unsigned int now_ms)
{
    static unsigned char last_progress;
    static unsigned int last_progress_ms;

    if (!tx_active)
    {
        last_progress = tx_progress;
        last_progress_ms = now_ms;
        return 0;
    }

    if (tx_progress != last_progress)
    {
        last_progress = tx_progress;
        last_progress_ms = now_ms;
        return 0;
    }

    if ((unsigned int)(now_ms - last_progress_ms) < UART2_TX_STALL_MS)
        return 0;

    /* 已停滞 3 ms：此时最后一只字节早已发完，UART 空闲，可以安全复位环。 */
    UART2_IRQ_OFF();
    tx_head = 0;
    tx_tail = 0;
    tx_count = 0;
    tx_active = 0;
    tx_stalls++;
    last_progress = tx_progress;
    last_progress_ms = now_ms;
    UART2_IRQ_ON();

    return 1;
}

unsigned int uart2_tx_overflow_count(void)
{
    return tx_overflow;
}

unsigned int uart2_tx_stall_count(void)
{
    return tx_stalls;
}
