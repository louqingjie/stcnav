/*
 * V2 协议自检固件（QEMU / 串口回归用）。
 *
 * UART1 P3.0/P3.1 115200 打印自检结果。QEMU 只依赖 UART1 与普通代码路径，
 * 因此该 app 可在 tools/build.py --qemu 下直接验证：
 *   - SDCC MCS251 大端 ABI 下 float 的小端序列化；
 *   - CRC16/Modbus、帧长度与 0x88 golden vector 的逐字节一致性。
 */

#include "common.h"
#include "stc32g_uart.h"
#include "v2_selftest.h"

static void write_uint(unsigned int value)
{
    static char digits[5];
    unsigned char count = 0;

    do
    {
        digits[count] = (char)('0' + (unsigned char)(value % 10u));
        value /= 10u;
        count++;
    } while (value != 0u);

    while (count != 0u)
    {
        count--;
        uart1_write_byte((unsigned char)digits[count]);
    }
}

void main(void)
{
    unsigned char step;

    board_init();
    uart1_init(115200UL, 0);

    step = v2_selftest();
    if (step == 0u)
    {
        uart1_write("PASS\r\n");
    }
    else
    {
        uart1_write("FAIL ");
        write_uint((unsigned int)step);
        uart1_write("\r\n");
    }

    for (;;)
    {
    }
}
