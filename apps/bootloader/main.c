/*
 * stcnav HID Bootloader（VID:PID 34BF:FF02）。
 *
 * Flash 布局：
 *   0xFE0000-0xFE3DFF  Bootloader 代码（本程序，16 KB 区）
 *   0xFE3E00-0xFE3FFF  元数据扇区（App 长度/CRC/有效标志）
 *   0xFE4000-0xFEFFFF  App 代码（48 KB）
 *   0xFF0000-0xFF0FFF  中断向量 + 复位跳转
 *
 * 主机协议（64 字节 HID 报告）：
 *   0..5  "STCNAV"
 *   6     0x02
 *   7     命令
 *   8..   参数
 * 响应第 8 字节为状态码，9.. 为数据。
 *
 * App 更新完成后把 0xFF0003 的复位跳转补成 0xFE0000，这样任何复位
 * 都会先进入 Bootloader；App 自身仍可直接通过 ROM ISP 烧录运行。
 */

#include "common.h"
#include "stc32g_iap.h"
#include "stc32g_usb_hid.h"

#define APP_START 0xFE4000UL
#define APP_END 0xFEFFFFUL
#define APP_SIZE (APP_END - APP_START + 1UL)
#define VEC_START 0xFF0000UL
#define VEC_END 0xFF0FFFUL
#define VEC_SIZE (VEC_END - VEC_START + 1UL)
#define META_ADDR 0xFE3E00UL
#define META_MAGIC 0x544F4F42UL /* "BOOT" */

#define PROTO_VERSION 0x02

#define CMD_INFO 0x01
#define CMD_ERASE 0x02
#define CMD_BEGIN 0x03
#define CMD_WRITE 0x04
#define CMD_VERIFY 0x05
#define CMD_FINALIZE 0x06
#define CMD_RUN 0x07
#define CMD_ISP 0x08
#define CMD_READ 0x09
#define CMD_PING 0x0A
#define CMD_PROBE 0x0B

#define STATUS_OK 0
#define STATUS_FAIL 1
#define STATUS_BAD_COMMAND 2
#define STATUS_BAD_ADDRESS 3
#define STATUS_BAD_LENGTH 4
#define STATUS_IAP_ERROR 5
#define STATUS_CRC_ERROR 6
#define STATUS_NO_APP 7
#define STATUS_NOT_VERIFIED 8

static const unsigned char proto_magic[6] = {'S', 'T', 'C', 'N', 'A', 'V'};

static __xdata unsigned char rx[USB_HID_REPORT_SIZE];
static __xdata unsigned char tx[USB_HID_REPORT_SIZE];

static unsigned long app_length;
static unsigned long app_crc;
static unsigned long vector_length;
static unsigned long vector_crc;
static unsigned char verified;

static unsigned long get_u24(const unsigned char *data)
{
    return (unsigned long)data[0] |
           ((unsigned long)data[1] << 8) |
           ((unsigned long)data[2] << 16);
}

static unsigned long get_u32(const unsigned char *data)
{
    return (unsigned long)data[0] |
           ((unsigned long)data[1] << 8) |
           ((unsigned long)data[2] << 16) |
           ((unsigned long)data[3] << 24);
}

static unsigned int get_u16(const unsigned char *data)
{
    return (unsigned int)data[0] | ((unsigned int)data[1] << 8);
}

static void put_u32(unsigned char *data, unsigned long value)
{
    data[0] = (unsigned char)value;
    data[1] = (unsigned char)(value >> 8);
    data[2] = (unsigned char)(value >> 16);
    data[3] = (unsigned char)(value >> 24);
}

static unsigned long flash_read32(unsigned long address)
{
    unsigned long value = 0;
    unsigned char index;

    for (index = 0; index < 4; index++)
        value |= (unsigned long)iap_read_byte(address + index) << (index * 8);
    return value;
}

static unsigned char flash_write32(unsigned long address, unsigned long value)
{
    unsigned char index;

    for (index = 0; index < 4; index++)
    {
        if (iap_write_byte(address + index,
                                  (unsigned char)(value >> (index * 8))))
            return 1;
    }
    return 0;
}

/* 直接按代码空间读元数据，启动阶段不依赖 IAP。 */
static unsigned long code_read32(unsigned long address)
{
    const unsigned char __code *pointer =
        (const unsigned char __code *)address;
    unsigned long value = 0;
    unsigned char index;

    for (index = 0; index < 4; index++)
        value |= (unsigned long)pointer[index] << (index * 8);
    return value;
}

static unsigned char app_valid(void)
{
    unsigned long magic = code_read32(META_ADDR);
    unsigned long length = code_read32(META_ADDR + 4);

    return (unsigned char)(magic == META_MAGIC &&
                           length > 0 &&
                           length <= APP_SIZE);
}

static unsigned char meta_write(void)
{
    unsigned char failed;

    iap_begin();
    failed = iap_erase_sector(META_ADDR);
    if (!failed)
        failed = flash_write32(META_ADDR, META_MAGIC);
    if (!failed)
        failed = flash_write32(META_ADDR + 4, app_length);
    if (!failed)
        failed = flash_write32(META_ADDR + 8, app_crc);
    if (!failed)
        failed = flash_write32(META_ADDR + 12, vector_crc);
    if (!failed)
    {
        unsigned int short_length = (unsigned int)vector_length;

        if (iap_write_byte(META_ADDR + 16, (unsigned char)short_length) ||
            iap_write_byte(META_ADDR + 17, (unsigned char)(short_length >> 8)))
            failed = 1;
    }
    if (!failed)
    {
        if (flash_read32(META_ADDR) != META_MAGIC ||
            flash_read32(META_ADDR + 4) != app_length ||
            flash_read32(META_ADDR + 8) != app_crc)
            failed = 1;
    }
    iap_end();

    return failed;
}

static unsigned char erase_region(unsigned long start, unsigned long size)
{
    unsigned long address;

    iap_begin();
    for (address = start; address < start + size; address += IAP_SECTOR_SIZE)
    {
        if (iap_erase_sector(address))
        {
            iap_end();
            return 1;
        }
    }
    iap_end();
    return 0;
}

static unsigned char write_bytes(unsigned long address, const unsigned char *data,
                                 unsigned char length)
{
    unsigned char index;

    iap_begin();
    for (index = 0; index < length; index++)
    {
        if (iap_write_byte(address + index, data[index]))
        {
            iap_end();
            return 1;
        }
    }
    iap_end();
    return 0;
}

static unsigned long crc32_update(unsigned long crc, unsigned char value)
{
    unsigned char bit;

    crc ^= value;
    for (bit = 0; bit < 8; bit++)
    {
        if (crc & 1)
            crc = (crc >> 1) ^ 0xEDB88320UL;
        else
            crc >>= 1;
    }
    return crc;
}

static unsigned char flash_crc32(unsigned long address, unsigned long length,
                                 unsigned long *result)
{
    unsigned long crc = 0xFFFFFFFFUL;

    iap_begin();
    while (length--)
        crc = crc32_update(crc, iap_read_byte(address++));
    if (iap_failed())
    {
        iap_end();
        return 1;
    }
    iap_end();

    *result = crc ^ 0xFFFFFFFFUL;
    return 0;
}

static void respond(unsigned char command, unsigned char status,
                    const unsigned char *data, unsigned char length)
{
    unsigned char index;

    for (index = 0; index < USB_HID_REPORT_SIZE; index++)
        tx[index] = 0;
    for (index = 0; index < 6; index++)
        tx[index] = proto_magic[index];
    tx[6] = PROTO_VERSION;
    tx[7] = command;
    tx[8] = status;
    for (index = 0; index < length && (unsigned char)(9 + index) < USB_HID_REPORT_SIZE; index++)
        tx[9 + index] = data[index];

    usb_hid_write(tx, USB_HID_REPORT_SIZE);
}

static void handle_info(void)
{
    unsigned char info[18];
    unsigned long magic = code_read32(META_ADDR);
    unsigned long length = code_read32(META_ADDR + 4);
    unsigned long crc = code_read32(META_ADDR + 8);

    put_u32(info + 0, APP_START);
    put_u32(info + 4, APP_SIZE);
    put_u32(info + 8, length);
    put_u32(info + 12, crc);
    info[16] = (unsigned char)(magic == META_MAGIC && length > 0 && length <= APP_SIZE);
    info[17] = PROTO_VERSION;

    respond(CMD_INFO, STATUS_OK, info, 18);
}

static void handle_probe(void)
{
    unsigned char data[3];

    /* IAP 映射探测：0xFE0000 是本程序首字节，0xFF0000 应为 0x02。 */
    iap_begin();
    data[0] = iap_read_byte(0xFE0000UL);
    data[1] = iap_read_byte(0xFF0000UL);
    data[2] = iap_failed();
    iap_end();

    respond(CMD_PROBE, STATUS_OK, data, 3);
}

static void handle_erase(void)
{
    if (erase_region(META_ADDR, IAP_SECTOR_SIZE) ||
        erase_region(APP_START, APP_SIZE) ||
        erase_region(VEC_START, VEC_SIZE))
    {
        respond(CMD_ERASE, STATUS_IAP_ERROR, 0, 0);
        return;
    }

    verified = 0;
    respond(CMD_ERASE, STATUS_OK, 0, 0);
}

static void handle_begin(const unsigned char *report, unsigned char length)
{
    if (length < 22)
    {
        respond(CMD_BEGIN, STATUS_BAD_LENGTH, 0, 0);
        return;
    }

    app_length = get_u32(report + 8);
    vector_length = get_u16(report + 12);
    app_crc = get_u32(report + 14);
    vector_crc = get_u32(report + 18);

    if (app_length == 0 || app_length > APP_SIZE ||
        vector_length < 4 || vector_length > VEC_SIZE)
    {
        respond(CMD_BEGIN, STATUS_BAD_LENGTH, 0, 0);
        return;
    }

    verified = 0;
    respond(CMD_BEGIN, STATUS_OK, 0, 0);
}

static unsigned char address_in_range(unsigned long address, unsigned char length)
{
    unsigned long end = address + length;

    if (address >= APP_START && end <= APP_START + app_length)
        return 1;
    if (address >= VEC_START && end <= VEC_START + vector_length)
        return 1;
    return 0;
}

static void handle_write(const unsigned char *report, unsigned char length)
{
    unsigned long address;
    unsigned char count;

    if (length < 12)
    {
        respond(CMD_WRITE, STATUS_BAD_LENGTH, 0, 0);
        return;
    }

    address = get_u24(report + 8);
    count = report[11];
    if (count == 0 || (unsigned char)(12 + count) > length)
    {
        respond(CMD_WRITE, STATUS_BAD_LENGTH, 0, 0);
        return;
    }
    if (!address_in_range(address, count))
    {
        respond(CMD_WRITE, STATUS_BAD_ADDRESS, 0, 0);
        return;
    }

    if (write_bytes(address, report + 12, count))
    {
        respond(CMD_WRITE, STATUS_IAP_ERROR, 0, 0);
        return;
    }

    verified = 0;
    respond(CMD_WRITE, STATUS_OK, 0, 0);
}

static void handle_verify(void)
{
    unsigned long actual;

    if (app_length == 0 || vector_length == 0)
    {
        respond(CMD_VERIFY, STATUS_BAD_LENGTH, 0, 0);
        return;
    }

    if (flash_crc32(APP_START, app_length, &actual) || actual != app_crc)
    {
        verified = 0;
        respond(CMD_VERIFY, STATUS_CRC_ERROR, 0, 0);
        return;
    }

    if (flash_crc32(VEC_START, vector_length, &actual) || actual != vector_crc)
    {
        verified = 0;
        respond(CMD_VERIFY, STATUS_CRC_ERROR, 0, 0);
        return;
    }

    verified = 1;
    respond(CMD_VERIFY, STATUS_OK, 0, 0);
}

static void handle_finalize(void)
{
    if (!verified)
    {
        respond(CMD_FINALIZE, STATUS_NOT_VERIFIED, 0, 0);
        return;
    }

    if (meta_write())
    {
        respond(CMD_FINALIZE, STATUS_IAP_ERROR, 0, 0);
        return;
    }

    respond(CMD_FINALIZE, STATUS_OK, 0, 0);
}

static void jump_to_app(void) __naked
{
    __asm
        mov spx,#0x0dff
        .db 0x8a, 0xfe, 0x40, 0x00
    __endasm;
}

static void handle_run(void)
{
    if (!app_valid())
    {
        respond(CMD_RUN, STATUS_NO_APP, 0, 0);
        return;
    }

    respond(CMD_RUN, STATUS_OK, 0, 0);
    ms_delay(50);
    usb_hid_disconnect();
    ms_delay(50);
    jump_to_app();
}

static void handle_isp(void)
{
    respond(CMD_ISP, STATUS_OK, 0, 0);
    ms_delay(50);
    iap_reset_to_isp();
}

static __xdata unsigned char read_data[55];

static void handle_read(const unsigned char *report, unsigned char length)
{
    unsigned long address;
    unsigned char count;
    unsigned char index;

    if (length < 12)
    {
        respond(CMD_READ, STATUS_BAD_LENGTH, 0, 0);
        return;
    }

    address = get_u24(report + 8);
    count = report[11];
    if (count == 0 || count > sizeof(read_data))
    {
        respond(CMD_READ, STATUS_BAD_LENGTH, 0, 0);
        return;
    }
    if (address < 0xFE0000UL || address + count > 0x1000000UL)
    {
        respond(CMD_READ, STATUS_BAD_ADDRESS, 0, 0);
        return;
    }

    iap_begin();
    for (index = 0; index < count; index++)
        read_data[index] = iap_read_byte(address + index);
    if (iap_failed())
    {
        iap_end();
        respond(CMD_READ, STATUS_IAP_ERROR, 0, 0);
        return;
    }
    iap_end();

    respond(CMD_READ, STATUS_OK, read_data, count);
}

static unsigned char handle_report(const unsigned char *report, unsigned char length)
{
    unsigned char index;

    if (length < 8)
        return 0;
    for (index = 0; index < 6; index++)
    {
        if (report[index] != proto_magic[index])
            return 0;
    }
    if (report[6] != PROTO_VERSION)
        return 0;

    switch (report[7])
    {
    case CMD_INFO:
        handle_info();
        break;
    case CMD_ERASE:
        handle_erase();
        break;
    case CMD_BEGIN:
        handle_begin(report, length);
        break;
    case CMD_WRITE:
        handle_write(report, length);
        break;
    case CMD_VERIFY:
        handle_verify();
        break;
    case CMD_FINALIZE:
        handle_finalize();
        break;
    case CMD_RUN:
        handle_run();
        break;
    case CMD_ISP:
        handle_isp();
        break;
    case CMD_READ:
        handle_read(report, length);
        break;
    case CMD_PING:
        respond(CMD_PING, STATUS_OK, 0, 0);
        break;
    case CMD_PROBE:
        handle_probe();
        break;
    default:
        respond(report[7], STATUS_BAD_COMMAND, 0, 0);
        break;
    }

    return 1;
}

void app_main(void)
{
    unsigned char length;
    unsigned char index;
    unsigned char stay = 0;

    board_init();
    usb_hid_init();

    if (iap_take_boot_request())
        stay = 1;
    else if (!app_valid())
        stay = 1;

    if (!stay)
    {
        /* 给主机 3 秒窗口连接 FF02；收到任何协议命令就留在 Bootloader。 */
        for (index = 0; index < 300; index++)
        {
            length = usb_hid_read(rx);
            if (length && handle_report(rx, length))
            {
                stay = 1;
                break;
            }
            ms_delay(10);
        }
    }

    if (!stay)
    {
        usb_hid_disconnect();
        ms_delay(20);
        jump_to_app();
    }

    for (;;)
    {
        length = usb_hid_read(rx);
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
