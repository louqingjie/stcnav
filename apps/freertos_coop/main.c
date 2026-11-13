/*
 * M1 验收：两个任务通过 taskYIELD 交替运行。
 * 主栈切到 EDATA 0x0B00 起，避免链接器默认的 120 字节栈。
 */

#include "FreeRTOS.h"
#include "task.h"
#include "common.h"
#include "stc32g_uart.h"

static unsigned char rounds;

static void task_a(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        stc32g_uart1_write("A");
        rounds++;
        if (rounds >= 10)
        {
            stc32g_uart1_write("\r\nPASS\r\n");
            rounds = 0;
        }
        taskYIELD();
    }
}

static void task_b(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        stc32g_uart1_write("B");
        taskYIELD();
    }
}

void app_main(void)
{
    board_init();
    stc32g_uart1_init(115200UL, 0);

    xTaskCreate(task_a, "A", configMINIMAL_STACK_SIZE, 0, 1, 0);
    xTaskCreate(task_b, "B", configMINIMAL_STACK_SIZE, 0, 1, 0);

    vTaskStartScheduler();

    for (;;)
    {
    }
}

void main(void) __naked
{
    __asm
        mov spx,#0x0aff
        ecall _app_main
        eret
    __endasm;
}
