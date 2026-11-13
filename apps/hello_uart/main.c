/*
 * M0 基线：UART1 115200 打印 PASS。
 */

#include "common.h"
#include "stc32g_uart.h"

void main(void)
{
    board_init();
    stc32g_uart1_init(115200UL, 0);

    stc32g_uart1_write("PASS\r\n");

    while (1)
    {
    }
}
