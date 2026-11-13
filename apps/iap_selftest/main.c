/*
 * IAP + USB HID 自检（VID:PID 34BF:FF03）。
 *
 * 上电只初始化 USB，不碰 IAP；主机发命令后才执行：
 *   PING        确认 USB EP1 OUT/IN 正常
 *   READ_CODE   用 __code 指针和 IAP 各读 0xFE4000 / 0xFF0000 的 16 字节并比较
 *   TEST_SECTOR 擦除/写入/回读 0xFEF000 的 512 字节
 *   RESET_ISP   写 IAP_CONTR=0x60，回 ROM ISP
 *
 * 0xFEF000 属于 App 区末尾的安全扇区，自检程序本身只有几 KB，不会碰到。
 */

#include "common.h"
#include "stc32g_gpio.h"
#include "stc32g_iap.h"
#include "stc32g_uart.h"
#include "stc32g_usb_hid.h"

#define PROTO_VERSION 0x03

#define CMD_PING 0x01
#define CMD_READ_CODE 0x02
#define CMD_TEST_SECTOR 0x03
#define CMD_RESET_ISP 0x04

#define APP_PROBE 0xFE4000UL
#define VEC_PROBE 0xFF0000UL
#define PROBE_SIZE 16
#define TEST_SECTOR 0xFEF000UL
#define TEST_SIZE 512

static __xdata unsigned char rx[STC32G_USB_HID_REPORT_SIZE];
static __xdata unsigned char tx[STC32G_USB_HID_REPORT_SIZE];

static unsigned char read_result;   /* bit0=App 区, bit1=向量区 */
static unsigned char read_bad_app;
static unsigned char read_bad_vec;
static unsigned char iap_error;

static unsigned char sector_ok;
static unsigned int sector_bad_index;
static unsigned char sector_expected;
static unsigned char sector_actual;

static char hex_text[3];

static void uart_hex(unsigned char value)
{
    const char *digits = "0123456789ABCDEF";

    hex_text[0] = digits[value >> 4];
    hex_text[1] = digits[value & 0x0F];
    hex_text[2] = 0;
    stc32g_uart1_write(hex_text);
}

static void respond(unsigned char command, unsigned char status,
                    const unsigned char *data, unsigned char length)
{
    unsigned char index;

    for (index = 0; index < STC32G_USB_HID_REPORT_SIZE; index++)
        tx[index] = 0;
    tx[0] = 'S';
    tx[1] = 'T';
    tx[2] = 'C';
    tx[3] = 'N';
    tx[4] = 'A';
    tx[5] = 'V';
    tx[6] = PROTO_VERSION;
    tx[7] = command;
    tx[8] = status;
    for (index = 0; index < length && (unsigned char)(9 + index) < STC32G_USB_HID_REPORT_SIZE; index++)
        tx[9 + index] = data[index];

    stc32g_usb_hid_write(tx, STC32G_USB_HID_REPORT_SIZE);
}

static void test_read_code(void)
{
    const unsigned char __code *code =
        (const unsigned char __code *)APP_PROBE;
    const unsigned char __code *vector =
        (const unsigned char __code *)VEC_PROBE;
    unsigned char index;
    unsigned char expected;
    unsigned char actual;

    read_result = 0;
    read_bad_app = 0xFF;
    read_bad_vec = 0xFF;
    iap_error = 0;

    stc32g_iap_begin();
    for (index = 0; index < PROBE_SIZE; index++)
    {
        expected = code[index];
        actual = stc32g_iap_read_byte(APP_PROBE + index);
        if (expected != actual)
        {
            read_bad_app = index;
            break;
        }
    }
    if (index == PROBE_SIZE)
        read_result |= 0x01;

    for (index = 0; index < PROBE_SIZE; index++)
    {
        expected = vector[index];
        actual = stc32g_iap_read_byte(VEC_PROBE + index);
        if (expected != actual)
        {
            read_bad_vec = index;
            break;
        }
    }
    if (index == PROBE_SIZE)
        read_result |= 0x02;

    if (stc32g_iap_failed())
    {
        iap_error = 1;
        read_result = 0;
    }
    stc32g_iap_end();
}

static void test_sector(void)
{
    unsigned int index;
    unsigned char expected;
    unsigned char actual;

    sector_ok = 0;
    sector_bad_index = 0;
    sector_expected = 0;
    sector_actual = 0;
    iap_error = 0;

    stc32g_iap_begin();

    if (stc32g_iap_erase_sector(TEST_SECTOR))
    {
        iap_error = 1;
        stc32g_iap_end();
        return;
    }

    for (index = 0; index < TEST_SIZE; index++)
    {
        expected = (unsigned char)(index ^ 0x5A);
        if (stc32g_iap_write_byte(TEST_SECTOR + index, expected))
        {
            iap_error = 1;
            stc32g_iap_end();
            return;
        }
    }

    for (index = 0; index < TEST_SIZE; index++)
    {
        expected = (unsigned char)(index ^ 0x5A);
        actual = stc32g_iap_read_byte(TEST_SECTOR + index);
        if (expected != actual)
        {
            sector_bad_index = index;
            sector_expected = expected;
            sector_actual = actual;
            stc32g_iap_end();
            return;
        }
    }

    if (stc32g_iap_failed())
    {
        iap_error = 1;
        stc32g_iap_end();
        return;
    }

    sector_ok = 1;
    stc32g_iap_end();
}

static void reply_read_code(void)
{
    unsigned char data[20];
    unsigned char index;
    const unsigned char __code *code =
        (const unsigned char __code *)APP_PROBE;
    const unsigned char __code *vector =
        (const unsigned char __code *)VEC_PROBE;

    data[0] = read_result;
    data[1] = read_bad_app;
    data[2] = read_bad_vec;
    data[3] = iap_error;
    for (index = 0; index < 8; index++)
    {
        data[4 + index] = code[index];
        data[12 + index] = vector[index];
    }

    respond(CMD_READ_CODE, 0, data, 20);
}

static void reply_sector(void)
{
    unsigned char data[7];

    data[0] = sector_ok;
    data[1] = iap_error;
    data[2] = (unsigned char)sector_bad_index;
    data[3] = (unsigned char)(sector_bad_index >> 8);
    data[4] = sector_expected;
    data[5] = sector_actual;

    respond(CMD_TEST_SECTOR, 0, data, 6);
}

static void handle_report(const unsigned char *report, unsigned char length)
{
    unsigned char index;

    if (length < 8)
        return;
    for (index = 0; index < 6; index++)
    {
        if (report[index] != "STCNAV"[index])
            return;
    }
    if (report[6] != PROTO_VERSION)
        return;

    switch (report[7])
    {
    case CMD_PING:
        stc32g_gpio_write(2, 0, 1);
        respond(CMD_PING, 0, 0, 0);
        stc32g_gpio_write(2, 0, 0);
        break;
    case CMD_READ_CODE:
        test_read_code();
        reply_read_code();
        stc32g_uart1_write("READ_CODE app=");
        uart_hex(read_result & 0x01);
        stc32g_uart1_write(" vec=");
        uart_hex((read_result >> 1) & 0x01);
        stc32g_uart1_write(" bad_app=");
        uart_hex(read_bad_app);
        stc32g_uart1_write(" bad_vec=");
        uart_hex(read_bad_vec);
        stc32g_uart1_write(" iap_err=");
        uart_hex(iap_error);
        stc32g_uart1_write("\r\n");
        break;
    case CMD_TEST_SECTOR:
        test_sector();
        reply_sector();
        stc32g_uart1_write("TEST_SECTOR ok=");
        uart_hex(sector_ok);
        stc32g_uart1_write(" iap_err=");
        uart_hex(iap_error);
        stc32g_uart1_write(" bad_idx=");
        uart_hex((unsigned char)(sector_bad_index >> 8));
        uart_hex((unsigned char)sector_bad_index);
        stc32g_uart1_write(" exp=");
        uart_hex(sector_expected);
        stc32g_uart1_write(" act=");
        uart_hex(sector_actual);
        stc32g_uart1_write("\r\n");
        break;
    case CMD_RESET_ISP:
        respond(CMD_RESET_ISP, 0, 0, 0);
        ms_delay(50);
        stc32g_iap_reset_to_isp();
        break;
    default:
        respond(report[7], 1, 0, 0);
        break;
    }
}

void app_main(void)
{
    unsigned char length;

    board_init();
    stc32g_uart1_init(115200UL, 1);
    stc32g_gpio_set_mode(2, 0, STC32G_GPIO_MODE_PUSH_PULL);
    stc32g_uart1_write("IAP SELFTEST READY\r\n");
    stc32g_usb_hid_init();

    for (;;)
    {
        length = stc32g_usb_hid_read(rx);
        if (length)
            handle_report(rx, length);
    }
}

void main(void) __naked
{
    __asm
        mov spx,#0x0dff
        ecall _app_main
        eret
    __endasm;
}
