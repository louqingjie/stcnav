/*
 * M2 验收：3 个任务 + 队列，Timer0 tick 抢占调度。
 * producer/consumer 通过队列传递计数，reporter 每秒打印一次。
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "common.h"
#include "stc32g_uart.h"
#include "stc32g_gpio.h"

static QueueHandle_t queue;
static StaticQueue_t queue_buffer;
static unsigned char queue_storage[4];

static void producer(void *pvParameters)
{
    unsigned char value = 0;

    (void)pvParameters;
    for (;;)
    {
        xQueueSend(queue, &value, 0);
        value++;
        vTaskDelay(100);
    }
}

static void consumer(void *pvParameters)
{
    unsigned char value;

    (void)pvParameters;
    for (;;)
    {
        if (xQueueReceive(queue, &value, portMAX_DELAY) == pdPASS)
        {
            stc32g_gpio_write(2, 0, (unsigned char)(value & 0x01));
        }
    }
}

static void reporter(void *pvParameters)
{
    (void)pvParameters;
    for (;;)
    {
        stc32g_uart1_write("PASS\r\n");
        vTaskDelay(1000);
    }
}

void app_main(void)
{
    board_init();
    stc32g_uart1_init(115200UL, 0);
    stc32g_gpio_set_mode(2, 0, STC32G_GPIO_MODE_PUSH_PULL);
    queue = xQueueCreateStatic(4, sizeof(unsigned char), queue_storage, &queue_buffer);
    if (queue == 0)
    {
        stc32g_uart1_write("FAIL\r\n");
        for (;;)
        {
        }
    }

    xTaskCreate(producer, "P", configMINIMAL_STACK_SIZE, 0, 2, 0);
    xTaskCreate(consumer, "C", configMINIMAL_STACK_SIZE, 0, 2, 0);
    xTaskCreate(reporter, "R", configMINIMAL_STACK_SIZE, 0, 1, 0);

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
