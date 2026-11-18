/*
 * HID 命令触发的不停电下载示例。
 *
 * 应用枚举为 34BF:FF01 的 HID 设备；主机发送 64 字节报告
 * "STCNAV" + 0x01 后，应用写 IAP_CONTR=0x60 软复位到系统 ISP 区，
 * 随后 ROM 在 P3.0/P3.1 上以 34BF:1001 枚举，等待 flash.py 烧录。
 *
 * UART1 改到 P3.6/P3.7，避免与 USB D-/D+ 冲突。
 */

#include "common.h"
#include "stc32g_uart.h"
#include "stc32g_usb_hid.h"
#include "stc32g_iap.h"

void app_main(void)
{
    board_init();
    uart1_init(115200UL, 1);
    uart1_write("USB HID ISP READY\r\n");

    usb_hid_init();

    for (;;)
    {
        unsigned char command = usb_hid_poll();

        if (command == 1)
        {
            uart1_write("RESET TO ISP\r\n");
            ms_delay(10);
            iap_reset_to_isp();
        }
        else if (command == 2)
        {
            uart1_write("RESET TO BOOTLOADER\r\n");
            ms_delay(10);
            iap_request_bootloader();
        }
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
