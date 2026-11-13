/*
 * IAP 驱动：读、写、擦除 512 字节扇区。
 *
 * 依据 STC32G 手册第 20 章：
 *   IAP_CMD=1 读字节，=2 写字节，=3 擦除 512 字节扇区
 *   地址 = 物理地址 - 0xFE0000（IAP_ADDRE/ADDRH/ADDRL）
 *   触发：先写 0x5A，再写 0xA5；CPU 会 IDLE 到操作完成
 *   IAP_CONTR.4 是 CMD_FAIL，需要写 0 清除
 */

#include "stc32g_iap.h"
#include "STC32Gxx.h"
#include "intrins.h"
#include "common.h"

static unsigned char iap_error;

static void iap_set_address(unsigned long address)
{
    unsigned long offset = address - STC32G_IAP_BASE;

    IAP_ADDRE = (unsigned char)(offset >> 16);
    IAP_ADDRH = (unsigned char)(offset >> 8);
    IAP_ADDRL = (unsigned char)offset;
}

/* 触发一次操作并返回 0=成功、1=失败。失败后顺手清 CMD_FAIL。 */
static unsigned char iap_execute(void)
{
    IAP_TRIG = 0x5A;
    IAP_TRIG = 0xA5;
    _nop_();
    _nop_();
    _nop_();
    _nop_();

    if (IAP_CONTR & 0x10)
    {
        iap_error = 1;
        IAP_CONTR = 0x80; /* 保持 IAPEN，清 CMD_FAIL */
        return 1;
    }
    return 0;
}

void stc32g_iap_begin(void)
{
    P_SW2 |= 0x80; /* EAXFR */
    IAP_TPS = (unsigned char)((FOSC + 500000UL) / 1000000UL);
    IAP_CONTR = 0x80;
    iap_error = 0;
}

unsigned char stc32g_iap_failed(void)
{
    return iap_error;
}

void stc32g_iap_end(void)
{
    IAP_CONTR = 0x00;
    IAP_CMD = 0x00;
    IAP_TRIG = 0x00;
    IAP_ADDRE = 0x00;
    IAP_ADDRH = 0x00;
    IAP_ADDRL = 0x00;
}

unsigned char stc32g_iap_read_byte(unsigned long address)
{
    IAP_CMD = 1;
    iap_set_address(address);
    if (iap_execute())
        return 0xFF;
    return IAP_DATA;
}

unsigned char stc32g_iap_write_byte(unsigned long address, unsigned char value)
{
    IAP_CMD = 2;
    iap_set_address(address);
    IAP_DATA = value;
    return iap_execute();
}

unsigned char stc32g_iap_erase_sector(unsigned long address)
{
    IAP_CMD = 3;
    iap_set_address(address);
    return iap_execute();
}

void stc32g_iap_reset_to_isp(void)
{
    IAP_CONTR = 0x60;
    for (;;)
    {
    }
}

#define STC32G_BOOT_MAGIC 0x544F4F42UL /* "BOOT" */

__xdata __at (0x011FF0) volatile unsigned long boot_magic;

void stc32g_iap_request_bootloader(void)
{
    boot_magic = STC32G_BOOT_MAGIC;
    IAP_CONTR = 0x20; /* SWBS=0，软复位后从用户程序区运行 */
    for (;;)
    {
    }
}

unsigned char stc32g_iap_take_boot_request(void)
{
    if (boot_magic != STC32G_BOOT_MAGIC)
        return 0;
    boot_magic = 0;
    return 1;
}
