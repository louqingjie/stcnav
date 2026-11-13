/*
 * FreeRTOS 配置基线（计划文档 7.7 节）。
 * configUSE_PREEMPTION 可由构建脚本用 -D 覆盖：M1 协作式取 0，M2 取 1。
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "common.h"

#ifndef configUSE_PREEMPTION
#define configUSE_PREEMPTION 1
#endif

#define configUSE_TIME_SLICING 0
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_TICKLESS_IDLE 0
#define configCPU_CLOCK_HZ FOSC
#define configTICK_RATE_HZ 1000
#define configMAX_PRIORITIES 4
#define configMINIMAL_STACK_SIZE 192
#define configTOTAL_HEAP_SIZE 2304
#define configMAX_TASK_NAME_LEN 8

#define configTICK_TYPE_WIDTH_IN_BITS TICK_TYPE_WIDTH_32_BITS
#define configUSE_MUTEXES 0
#define configUSE_RECURSIVE_MUTEXES 0
#define configUSE_COUNTING_SEMAPHORES 0
#define configUSE_QUEUE_SETS 0
#define configUSE_TASK_NOTIFICATIONS 0
#define configUSE_TIMERS 0
#define configUSE_EVENT_GROUPS 0
#define configUSE_STREAM_BUFFERS 0

#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configSUPPORT_STATIC_ALLOCATION 1
#define configAPPLICATION_ALLOCATED_HEAP 1

#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configUSE_MALLOC_FAILED_HOOK 1
#define configCHECK_FOR_STACK_OVERFLOW 1
#define configUSE_DAEMON_TASK_STARTUP_HOOK 0

#define configQUEUE_REGISTRY_SIZE 0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 0
#define configUSE_TRACE_FACILITY 0
#define configUSE_STATS_FORMATTING_FUNCTIONS 0
#define configGENERATE_RUN_TIME_STATS 0

#define INCLUDE_vTaskPrioritySet 0
#define INCLUDE_uxTaskPriorityGet 0
#define INCLUDE_vTaskDelete 1
#define INCLUDE_vTaskSuspend 0
#define INCLUDE_xTaskDelayUntil 0
#define INCLUDE_vTaskDelay 1
#define INCLUDE_xTaskGetSchedulerState 0
#define INCLUDE_xTaskGetCurrentTaskHandle 0

#define configASSERT(x) \
    do \
    { \
        if (!(x)) \
            vAssertCalled(__FILE__, __LINE__); \
    } while (0)

#endif
