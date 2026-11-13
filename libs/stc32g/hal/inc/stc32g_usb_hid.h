/*
 * 最小 USB HID 设备（轮询方式）。
 *
 * 默认 VID:PID = 34BF:FF01（应用）；Bootloader 编译时用
 * -DSTC32G_USB_HID_PID=0xFF02 覆盖 PID。
 *
 * 主机 -> 设备：EP1 OUT，64 字节报告（hidapi 需带 report ID 0）。
 * 设备 -> 主机：EP1 IN，64 字节报告。
 */

#ifndef STCNAV_STC32G_USB_HID_H
#define STCNAV_STC32G_USB_HID_H

#define STC32G_USB_HID_REPORT_SIZE 64

#ifndef STC32G_USB_HID_PID
#define STC32G_USB_HID_PID 0xFF01
#endif

#ifndef STC32G_USB_HID_PRODUCT
#define STC32G_USB_HID_PRODUCT "stcnav HID"
#endif

void stc32g_usb_hid_init(void);

/* 处理 USB 事件并取走一个 EP1 OUT 报告；返回长度，0 表示没有新报告。 */
unsigned char stc32g_usb_hid_read(unsigned char *buffer);

/* 通过 EP1 IN 发送一个 64 字节报告（不足补 0）。 */
void stc32g_usb_hid_write(const unsigned char *buffer, unsigned char length);

/* 兼容旧应用：返回 1=收到 "STCNAV"+0x01，2=收到 "STCNAV"+0x02。 */
unsigned char stc32g_usb_hid_poll(void);

/* 关闭 USB 上拉，让主机看到断开；应用重新初始化后会再次枚举。 */
void stc32g_usb_hid_disconnect(void);

#endif
