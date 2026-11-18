/*
 * PIE-Block 遥控接收。
 *
 * NRF24L01 使用软件 SPI：
 *   CE=P3.4, CSN=P2.2, SCK=P2.5, MOSI=P2.3, MISO=P2.4
 * 接收包 12 字节：
 *   [0]=0x0B, [1..8]=4 个 int16 小端摇杆, [9..10]=按键位图, [11]=校验和
 */

#include "remote.h"
#include "config.h"
#include "common.h"
#include "stc32g_uart.h"

#define NRF_READ_REG 0x00
#define NRF_WRITE_REG 0x20
#define RD_RX_PLOAD 0x61
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define NRF_NOP 0xFF

#define NRF_CONFIG 0x00
#define NRF_EN_AA 0x01
#define NRF_EN_RXADDR 0x02
#define NRF_SETUP_AW 0x03
#define NRF_SETUP_RETR 0x04
#define NRF_RF_CH 0x05
#define NRF_RF_SETUP 0x06
#define NRF_STATUS 0x07
#define NRF_RX_ADDR_P0 0x0A
#define NRF_TX_ADDR 0x10
#define NRF_RX_PW_P0 0x11

#define NRF_RX_DR 0x40
#define NRF_TX_DS 0x20
#define NRF_MAX_RT 0x10

remote_state_t g_remote;

static unsigned int last_packet_ms;
static unsigned char have_packet;
static unsigned char rx_data[REMOTE_PACKET_LEN];

static const unsigned char nrf_address[5] = {'R', 'C', 'T', 'L', REMOTE_CHANNEL};
static const unsigned char nrf_link_pattern[5] = {6, 6, 6, 6, 6};
static unsigned char nrf_link_readback[5];

#define NRF_CE_HIGH() (P3 |= NRF_CE_MASK)
#define NRF_CE_LOW() (P3 &= (unsigned char)~NRF_CE_MASK)
#define NRF_CSN_HIGH() (P2 |= NRF_CSN_MASK)
#define NRF_CSN_LOW() (P2 &= (unsigned char)~NRF_CSN_MASK)
#define NRF_SCK_HIGH() (P2 |= NRF_SCK_MASK)
#define NRF_SCK_LOW() (P2 &= (unsigned char)~NRF_SCK_MASK)
#define NRF_MOSI_HIGH() (P2 |= NRF_MOSI_MASK)
#define NRF_MOSI_LOW() (P2 &= (unsigned char)~NRF_MOSI_MASK)
#define NRF_MISO_READ() ((P2 & NRF_MISO_MASK) ? 1 : 0)

static unsigned char spi_transfer(unsigned char value)
{
    unsigned char index;
    unsigned char result = 0;

    for (index = 0; index < 8; index++)
    {
        if (value & 0x80)
            NRF_MOSI_HIGH();
        else
            NRF_MOSI_LOW();

        us_delay(1);
        NRF_SCK_HIGH();
        us_delay(1);

        result <<= 1;
        if (NRF_MISO_READ())
            result |= 1;

        NRF_SCK_LOW();
        us_delay(1);
        value <<= 1;
    }

    return result;
}

static void nrf_write_reg(unsigned char reg, unsigned char value)
{
    NRF_CSN_LOW();
    spi_transfer(reg);
    spi_transfer(value);
    NRF_CSN_HIGH();
}

static unsigned char nrf_read_reg(unsigned char reg)
{
    unsigned char value;

    NRF_CSN_LOW();
    spi_transfer(reg);
    value = spi_transfer(NRF_NOP);
    NRF_CSN_HIGH();

    return value;
}

static void nrf_write_buf(unsigned char reg, const unsigned char *buffer, unsigned char length)
{
    unsigned char index;

    NRF_CSN_LOW();
    spi_transfer(reg);
    for (index = 0; index < length; index++)
        spi_transfer(buffer[index]);
    NRF_CSN_HIGH();
}

static void nrf_read_buf(unsigned char reg, unsigned char *buffer, unsigned char length)
{
    unsigned char index;

    NRF_CSN_LOW();
    spi_transfer(reg);
    for (index = 0; index < length; index++)
        buffer[index] = spi_transfer(NRF_NOP);
    NRF_CSN_HIGH();
}

#if VEHICLE_DEBUG
static unsigned char debug_counter;

static void debug_hex8(unsigned char value)
{
    const char *digits = "0123456789ABCDEF";
    uart1_write_byte((unsigned char)digits[value >> 4]);
    uart1_write_byte((unsigned char)digits[value & 0x0F]);
}

static void remote_debug_regs(void)
{
    unsigned char address[5];
    unsigned char index;

    uart1_write("NRF CFG=");
    debug_hex8(nrf_read_reg(NRF_CONFIG));
    uart1_write(" CH=");
    debug_hex8(nrf_read_reg(NRF_RF_CH));
    uart1_write(" AA=");
    debug_hex8(nrf_read_reg(NRF_EN_AA));
    uart1_write(" RXEN=");
    debug_hex8(nrf_read_reg(NRF_EN_RXADDR));
    uart1_write(" PW=");
    debug_hex8(nrf_read_reg(NRF_RX_PW_P0));
    uart1_write(" ADDR=");
    nrf_read_buf(NRF_RX_ADDR_P0, address, 5);
    for (index = 0; index < 5; index++)
    {
        debug_hex8(address[index]);
        uart1_write_byte(' ');
    }
    uart1_write("\r\n");
}
#endif

static unsigned char nrf_init(void)
{
    unsigned char i;

    /* P2.2/P2.3/P2.5 推挽，P2.4 高阻输入；P3.4 推挽。 */
    P2M1 &= (unsigned char)~0x2C;
    P2M0 |= 0x2C;
    P2M1 |= 0x10;
    P2M0 &= (unsigned char)~0x10;
    P2M1 |= 0x40;
    P2M0 &= (unsigned char)~0x40;
    P3M1 &= (unsigned char)~0x10;
    P3M0 |= 0x10;

    NRF_CE_LOW();
    NRF_CSN_HIGH();
    NRF_SCK_LOW();
    NRF_MOSI_LOW();

    nrf_write_reg(NRF_WRITE_REG + NRF_SETUP_AW, 0x03);
    nrf_write_reg(NRF_WRITE_REG + NRF_RF_CH, REMOTE_CHANNEL);
    nrf_write_reg(NRF_WRITE_REG + NRF_RF_SETUP, 0x06);
    nrf_write_reg(NRF_WRITE_REG + NRF_EN_AA, 0x01);
    nrf_write_reg(NRF_WRITE_REG + NRF_EN_RXADDR, 0x01);
    nrf_write_buf(NRF_WRITE_REG + NRF_RX_ADDR_P0, nrf_address, 5);
    nrf_write_reg(NRF_WRITE_REG + NRF_RX_PW_P0, REMOTE_PACKET_LEN);
    nrf_write_reg(NRF_WRITE_REG + NRF_SETUP_RETR, 0x05);
    nrf_write_reg(FLUSH_RX, NRF_NOP);
    nrf_write_reg(FLUSH_TX, NRF_NOP);
    nrf_write_reg(NRF_WRITE_REG + NRF_CONFIG, 0x0F);
    nrf_write_reg(NRF_WRITE_REG + NRF_STATUS, 0xFF);
    nrf_write_reg(FLUSH_RX, NRF_NOP);

    ms_delay(2);

    NRF_CE_LOW();
    /* 写/读 TX_ADDR 做 SPI 链路自检。 */
    nrf_write_buf(NRF_WRITE_REG + NRF_TX_ADDR, nrf_link_pattern, 5);
    nrf_read_buf(NRF_TX_ADDR, nrf_link_readback, 5);
    for (i = 0; i < 5; i++)
    {
        if (nrf_link_readback[i] != nrf_link_pattern[i])
            return 0;
    }

    /* 恢复 TX_ADDR，保持和 RX_ADDR_P0 一致。 */
    nrf_write_buf(NRF_WRITE_REG + NRF_TX_ADDR, nrf_address, 5);
    NRF_CE_HIGH();

    return 1;
}

unsigned char remote_init(void)
{
    g_remote.left_v = 0;
    g_remote.left_h = 0;
    g_remote.right_v = 0;
    g_remote.right_h = 0;
    g_remote.keys = 0;
    g_remote.connected = 0;
    last_packet_ms = 0;
    have_packet = 0;

    if (!nrf_init())
        return 0;

#if VEHICLE_DEBUG
    remote_debug_regs();
#endif
    return 1;
}

unsigned char remote_poll(unsigned int now_ms)
{
    unsigned char status;
    unsigned char index;
    unsigned char checksum = 0;
    unsigned char received = 0;

    status = nrf_read_reg(NRF_STATUS);
    if (status != 0)
        nrf_write_reg(NRF_WRITE_REG + NRF_STATUS, status);

    if (status & NRF_RX_DR)
    {
        NRF_CE_LOW();
        nrf_read_buf(RD_RX_PLOAD, rx_data, REMOTE_PACKET_LEN);
        NRF_CE_HIGH();

#if VEHICLE_DEBUG
        uart1_write("RX ");
        for (index = 0; index < REMOTE_PACKET_LEN; index++)
        {
            debug_hex8(rx_data[index]);
            uart1_write_byte(' ');
        }
        uart1_write("\r\n");
#endif

        if (rx_data[0] == 0x0B)
        {
            for (index = 1; index < 11; index++)
                checksum += rx_data[index];

            if (checksum == rx_data[11])
            {
                g_remote.left_v = (signed short)((unsigned short)rx_data[1] |
                                                 ((unsigned short)rx_data[2] << 8));
                g_remote.left_h = (signed short)((unsigned short)rx_data[3] |
                                                 ((unsigned short)rx_data[4] << 8));
                g_remote.right_v = (signed short)((unsigned short)rx_data[5] |
                                                  ((unsigned short)rx_data[6] << 8));
                g_remote.right_h = (signed short)((unsigned short)rx_data[7] |
                                                  ((unsigned short)rx_data[8] << 8));
                g_remote.keys = (unsigned int)rx_data[9] | ((unsigned int)rx_data[10] << 8);

                last_packet_ms = now_ms;
                have_packet = 1;
                received = 1;
            }
        }
    }

    if (have_packet && ((unsigned int)(now_ms - last_packet_ms) < REMOTE_TIMEOUT_MS))
        g_remote.connected = 1;
    else
        g_remote.connected = 0;

#if VEHICLE_DEBUG
    if (++debug_counter >= 100)
    {
        debug_counter = 0;
        uart1_write("NRF ST=");
        debug_hex8(status);
        uart1_write(" FIFO=");
        debug_hex8(nrf_read_reg(0x17));
        uart1_write("\r\n");
    }
#endif

    return received;
}

unsigned char remote_connected(void)
{
    return g_remote.connected;
}

unsigned char remote_key(unsigned char bit)
{
    if (bit > 15)
        return 0;
    return (unsigned char)((g_remote.keys >> bit) & 0x01);
}
