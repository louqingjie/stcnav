/*
 * STC32G 最小 USB HID 设备栈（轮询，不使用 USB 中断）。
 *
 * 只实现枚举所需的标准请求 + HID report descriptor + EP1 OUT 命令。
 * 主机发送 64 字节输出报告，内容以 "STCNAV" 开头且第 7 字节为 0x01 时，
 * 本模块返回 1，由应用写 IAP_CONTR=0x60 软复位到系统 ISP 区。
 *
 * 依据：STC32G 技术参考手册 27.1/27.2/5.16。
 */

#include "stc32g_usb_hid.h"
#include "STC32Gxx.h"

/* USB 专用 48MHz IRC（XFR，需要 EAXFR=1）。 */
__xdata __at (0x7EFE07) volatile unsigned char IRC48MCR;

/* USB 间接寄存器索引 */
#define USB_FADDR 0x00
#define USB_POWER 0x01
#define USB_INTRIN1 0x02
#define USB_INTROUT1 0x04
#define USB_INTRUSB 0x06
#define USB_INTRIN1E 0x07
#define USB_INTROUT1E 0x09
#define USB_INTRUSBE 0x0B
#define USB_INDEX 0x0E
#define USB_INMAXP 0x10
#define USB_CSR0 0x11
#define USB_INCSR1 0x11
#define USB_INCSR2 0x12
#define USB_OUTMAXP 0x13
#define USB_OUTCSR1 0x14
#define USB_OUTCSR2 0x15
#define USB_COUNT0 0x16
#define USB_OUTCOUNT1 0x16
#define USB_OUTCOUNT2 0x17
#define USB_FIFO0 0x20

#define USB_VID_L 0xBF
#define USB_VID_H 0x34
#define USB_PID_L (STC32G_USB_HID_PID & 0xFF)
#define USB_PID_H ((STC32G_USB_HID_PID >> 8) & 0xFF)

static const unsigned char device_descriptor[18] = {
    18, 0x01,
    0x00, 0x02,
    0x00, 0x00, 0x00,
    0x40,
    USB_VID_L, USB_VID_H,
    USB_PID_L, USB_PID_H,
    0x00, 0x01,
    1, 2, 0,
    1
};

#define REPORT_DESCRIPTOR_SIZE 27
static const unsigned char report_descriptor[REPORT_DESCRIPTOR_SIZE] = {
    0x06, 0x00, 0xFF, /* Usage Page (Vendor Defined) */
    0x09, 0x01,       /* Usage */
    0xA1, 0x01,       /* Collection (Application) */
    0x09, 0x02,       /* Usage (Output) */
    0x15, 0x00,       /* Logical Minimum 0 */
    0x26, 0xFF, 0x00, /* Logical Maximum 255 */
    0x75, 0x08,       /* Report Size 8 */
    0x95, 0x40,       /* Report Count 64 */
    0x91, 0x02,       /* Output (Data,Var,Abs) */
    0x09, 0x03,       /* Usage (Input) */
    0x95, 0x40,       /* Report Count 64 */
    0x81, 0x02,       /* Input (Data,Var,Abs) */
    0xC0              /* End Collection */
};

static const unsigned char config_descriptor[41] = {
    9, 0x02, 41, 0, 1, 1, 0, 0x80, 50,
    9, 0x04, 0, 0, 2, 3, 0, 0, 0,
    9, 0x21, 0x11, 0x01, 0, 1, 0x22, REPORT_DESCRIPTOR_SIZE, 0,
    7, 0x05, 0x81, 0x03, 64, 0, 1,
    7, 0x05, 0x01, 0x03, 64, 0, 1
};

static unsigned char usb_ready;
static unsigned char ep0_out_pending;
static unsigned char ep0_command;
static unsigned char ep0_report[64];
static unsigned char string_buffer[34];

/* EP1 OUT 收到的报告，供 read() 取走。放在 XDATA，避免占用 EDATA 栈。 */
static __xdata unsigned char ep1_out[STC32G_USB_HID_REPORT_SIZE];
static unsigned char ep1_out_length;
static unsigned char ep1_out_ready;

static unsigned char usb_command_code(const unsigned char *buffer, unsigned char length)
{
    if (length >= 7 &&
        buffer[0] == 'S' && buffer[1] == 'T' && buffer[2] == 'C' &&
        buffer[3] == 'N' && buffer[4] == 'A' && buffer[5] == 'V')
        return buffer[6];
    return 0;
}

static void usb_write(unsigned char index, unsigned char value)
{
    USBADR = (unsigned char)(index & 0x3F);
    USBDAT = value;
}

static unsigned char usb_read(unsigned char index)
{
    USBADR = (unsigned char)((index & 0x3F) | 0x80);
    while (USBADR & 0x80)
    {
    }
    return USBDAT;
}

static void usb_fifo_read(unsigned char endpoint, unsigned char *buffer, unsigned char length)
{
    unsigned char index;

    USBADR = (unsigned char)((USB_FIFO0 + endpoint) | 0xC0);
    while (USBADR & 0x80)
    {
    }
    for (index = 0; index < length; index++)
        buffer[index] = USBDAT;
}

static void usb_fifo_write(unsigned char endpoint, const unsigned char *buffer, unsigned char length)
{
    unsigned char index;

    USBADR = (unsigned char)(USB_FIFO0 + endpoint);
    for (index = 0; index < length; index++)
        USBDAT = buffer[index];
}

static void usb_fifo_write_padded(unsigned char endpoint, const unsigned char *buffer, unsigned char length)
{
    unsigned char index;

    USBADR = (unsigned char)(USB_FIFO0 + endpoint);
    for (index = 0; index < length; index++)
        USBDAT = buffer[index];
    for (; index < STC32G_USB_HID_REPORT_SIZE; index++)
        USBDAT = 0;
}

static void usb_configure_endpoints(void)
{
    usb_write(USB_INDEX, 0);
    usb_write(USB_INMAXP, 8); /* 64 字节 */
    usb_write(USB_CSR0, 0x00);

    usb_write(USB_INDEX, 1);
    usb_write(USB_INMAXP, 8);
    usb_write(USB_INCSR2, 0x20); /* IN，批量/中断 */
    usb_write(USB_OUTMAXP, 8);
    usb_write(USB_OUTCSR2, 0x00); /* OUT */
    usb_write(USB_INCSR1, 0x80);  /* CLRDT */
    usb_write(USB_OUTCSR1, 0x80); /* CLRDT */

    (void)usb_read(USB_INTRIN1);
    (void)usb_read(USB_INTROUT1);
    (void)usb_read(USB_INTRUSB);
}

static void usb_ep0_send(const unsigned char *data, unsigned char length)
{
    usb_write(USB_INDEX, 0);
    if (length)
        usb_fifo_write(0, data, length);
    usb_write(USB_CSR0, (unsigned char)(0x02 | (length < 64 ? 0x08 : 0x00)));
}

static void usb_ep0_send_descriptor(
    const unsigned char *data,
    unsigned char length,
    const unsigned char *setup)
{
    unsigned int requested = (unsigned int)setup[7] |
                             ((unsigned int)setup[8] << 8);
    if (requested < length)
        length = (unsigned char)requested;
    usb_ep0_send(data, length);
}

static void usb_ep0_stall(void)
{
    usb_write(USB_INDEX, 0);
    usb_write(USB_CSR0, 0x20);
}

static unsigned char usb_string_descriptor(unsigned char index, unsigned char *buffer)
{
    const char *text;
    unsigned char length;
    unsigned char position;

    if (index == 0)
    {
        buffer[0] = 4;
        buffer[1] = 0x03;
        buffer[2] = 0x09;
        buffer[3] = 0x04;
        return 4;
    }

    text = (index == 1) ? "stcnav" : STC32G_USB_HID_PRODUCT;
    length = 0;
    while (text[length])
        length++;

    buffer[0] = (unsigned char)(2 + length * 2);
    buffer[1] = 0x03;
    for (position = 0; position < length; position++)
    {
        buffer[2 + position * 2] = (unsigned char)text[position];
        buffer[3 + position * 2] = 0x00;
    }
    return (unsigned char)(2 + length * 2);
}

static void usb_handle_setup(const unsigned char *setup)
{
    unsigned char request_type = setup[0];
    unsigned char request = setup[1];
    unsigned char descriptor_type = setup[3];
    unsigned char descriptor_index = setup[2];
    unsigned char length;

    if ((request_type & 0x60) != 0x00 && (request_type & 0x60) != 0x20)
    {
        usb_ep0_stall();
        return;
    }

    switch (request)
    {
    case 0x06: /* GET_DESCRIPTOR */
        switch (descriptor_type)
        {
        case 0x01:
            usb_ep0_send_descriptor(
                device_descriptor, sizeof(device_descriptor), setup);
            return;
        case 0x02:
            usb_ep0_send_descriptor(
                config_descriptor, sizeof(config_descriptor), setup);
            return;
        case 0x03:
            length = usb_string_descriptor(descriptor_index, string_buffer);
            usb_ep0_send_descriptor(string_buffer, length, setup);
            return;
        case 0x22:
            usb_ep0_send_descriptor(
                report_descriptor, REPORT_DESCRIPTOR_SIZE, setup);
            return;
        default:
            usb_ep0_stall();
            return;
        }

    case 0x05: /* SET_ADDRESS */
        usb_write(USB_FADDR, (unsigned char)(setup[2] & 0x7F));
        usb_ep0_send(0, 0);
        return;

    case 0x09:
        if ((request_type & 0x60) == 0x20)
        {
            /* HID SET_REPORT：等待下一个 OUT 数据包。 */
            ep0_out_pending = 1;
            return;
        }
        /* SET_CONFIGURATION */
        usb_configure_endpoints();
        usb_ep0_send(0, 0);
        return;

    case 0x08: /* GET_CONFIGURATION */
        string_buffer[0] = 1;
        usb_ep0_send(string_buffer, 1);
        return;

    case 0x00: /* GET_STATUS */
        string_buffer[0] = 0;
        string_buffer[1] = 0;
        usb_ep0_send(string_buffer, 2);
        return;

    case 0x0A: /* GET_INTERFACE / SET_IDLE */
        if ((request_type & 0x80) != 0)
        {
            string_buffer[0] = 0;
            usb_ep0_send(string_buffer, 1);
        }
        else
        {
            usb_ep0_send(0, 0);
        }
        return;

    case 0x0B: /* SET_INTERFACE / SET_PROTOCOL */
        usb_ep0_send(0, 0);
        return;

    case 0x01: /* GET_REPORT */
        string_buffer[0] = 0;
        usb_ep0_send(string_buffer, 1);
        return;

    default:
        usb_ep0_stall();
        return;
    }
}

static void usb_handle_ep0(void)
{
    unsigned char setup[8];
    unsigned char count;

    usb_write(USB_INDEX, 0);
    count = (unsigned char)(usb_read(USB_COUNT0) & 0x7F);
    if (count == 0)
    {
        usb_write(USB_CSR0, 0x40); /* SOPRDY */
        return;
    }

    if (ep0_out_pending)
    {
        if (count > sizeof(ep0_report))
            count = sizeof(ep0_report);
        usb_fifo_read(0, ep0_report, count);
        usb_write(USB_CSR0, 0x40); /* SOPRDY */
        ep0_out_pending = 0;
        usb_ep0_send(0, 0);
        ep0_command = usb_command_code(ep0_report, count);
        return;
    }

    if (count > sizeof(setup))
        count = sizeof(setup);
    usb_fifo_read(0, setup, count);
    usb_write(USB_CSR0, 0x40); /* SOPRDY */
    usb_handle_setup(setup);
}

void stc32g_usb_hid_init(void)
{
    unsigned int timeout = 0;
    unsigned int delay;

    P_SW2 |= 0x80; /* EAXFR：允许访问 XFR */

    /* P3.0/P3.1 与 USB D-/D+ 共用，必须设为高阻输入。 */
    P3M1 |= 0x03;
    P3M0 &= (unsigned char)~0x03;

    IRC48MCR = 0x80;
    while (!(IRC48MCR & 0x01) && timeout < 60000)
        timeout++;

    IRCBAND |= 0x80;  /* USB 时钟选择 IRC48M */
    IRCBAND &= ~0x40;

    USBCLK = 0x00;
    USBCON = 0x90; /* ENUSB | PUEN */

    for (delay = 0; delay < 20000; delay++)
    {
    }

    usb_write(USB_FADDR, 0x00);
    usb_configure_endpoints();

    ep1_out_ready = 0;
    ep0_command = 0;
    usb_ready = 1;
}

static void usb_process_events(void)
{
    unsigned char intr_usb;
    unsigned char intr_out;
    unsigned char csr0;
    unsigned char length;

    if (!usb_ready)
        return;

    intr_usb = usb_read(USB_INTRUSB);
    if (intr_usb & 0x04) /* RSTIF */
    {
        usb_write(USB_FADDR, 0x00);
        usb_configure_endpoints();
        ep1_out_ready = 0;
    }

    intr_out = usb_read(USB_INTROUT1);
    if (intr_out & 0x02) /* EP1 OUT */
    {
        usb_write(USB_INDEX, 1);
        length = usb_read(USB_OUTCOUNT1);
        if (length > STC32G_USB_HID_REPORT_SIZE)
            length = STC32G_USB_HID_REPORT_SIZE;
        usb_fifo_read(1, ep1_out, length);
        usb_write(USB_OUTCSR1, 0x00); /* 清 OPRDY */
        ep1_out_length = length;
        ep1_out_ready = 1;
    }

    usb_write(USB_INDEX, 0);
    csr0 = usb_read(USB_CSR0);
    if (csr0 & 0x10) /* SUEND */
        usb_write(USB_CSR0, 0x80);
    if (csr0 & 0x01) /* OPRDY */
        usb_handle_ep0();
}

unsigned char stc32g_usb_hid_read(unsigned char *buffer)
{
    unsigned char index;
    unsigned char length;

    usb_process_events();
    if (!ep1_out_ready)
        return 0;

    length = ep1_out_length;
    for (index = 0; index < length; index++)
        buffer[index] = ep1_out[index];
    ep1_out_ready = 0;
    return length;
}

void stc32g_usb_hid_write(const unsigned char *buffer, unsigned char length)
{
    unsigned char index;
    unsigned char csr;

    if (!usb_ready)
        return;
    if (length > STC32G_USB_HID_REPORT_SIZE)
        length = STC32G_USB_HID_REPORT_SIZE;

    usb_write(USB_INDEX, 1);

    /* 等上一个包发完（FIFONE=0）；主机长时间不读时丢弃旧包。 */
    for (index = 0; index < 200; index++)
    {
        csr = usb_read(USB_INCSR1);
        if (!(csr & 0x02))
            break;
    }
    if (index == 200)
        usb_write(USB_INCSR1, 0x08); /* FLUSH */

    usb_fifo_write_padded(1, buffer, length);
    usb_write(USB_INCSR1, 0x01); /* IPRDY */
}

unsigned char stc32g_usb_hid_poll(void)
{
    unsigned char command;
    unsigned char buffer[STC32G_USB_HID_REPORT_SIZE];

    usb_process_events();

    if (ep0_command)
    {
        command = ep0_command;
        ep0_command = 0;
        return command;
    }

    command = stc32g_usb_hid_read(buffer);
    if (command)
        command = usb_command_code(buffer, command);
    return command;
}

void stc32g_usb_hid_disconnect(void)
{
    USBCON = 0x80; /* 保留 ENUSB，关闭上拉 */
    usb_ready = 0;
}
