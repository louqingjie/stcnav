/*
 * FreeRTOS MCS-251 移植层类型与宏。
 */

#ifndef PORTMACRO_H
#define PORTMACRO_H

#include "FreeRTOSConfig.h"
#include "intrins.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned char StackType_t;
typedef int BaseType_t;
typedef unsigned int UBaseType_t;
typedef unsigned long TickType_t;

#define portCHAR char
#define portFLOAT float
#define portDOUBLE double
#define portLONG long
#define portSHORT int
#define portSTACK_TYPE StackType_t
#define portBASE_TYPE BaseType_t
#define portUBASE_TYPE UBaseType_t
#define portTICK_TYPE TickType_t

#define portSTACK_GROWTH (1)
#define portBYTE_ALIGNMENT 1
#define portTICK_TYPE_IS_ATOMIC 0
#define portMAX_DELAY ((TickType_t)0xFFFFFFFFUL)
#define portNOP() _nop_()
#define portINLINE static inline
#define portMEMORY_BARRIER()

void vPortEnterCritical(void);
void vPortExitCritical(void);
void vPortYield(void);
UBaseType_t ulPortSetInterruptMaskFromISR(void);
void vPortClearInterruptMaskFromISR(UBaseType_t mask);
void vAssertCalled(const char *file, unsigned long line);

#define portDISABLE_INTERRUPTS() (EA = 0)
#define portENABLE_INTERRUPTS() (EA = 1)
#define portENTER_CRITICAL() vPortEnterCritical()
#define portEXIT_CRITICAL() vPortExitCritical()
#define portSET_INTERRUPT_MASK_FROM_ISR() ulPortSetInterruptMaskFromISR()
#define portCLEAR_INTERRUPT_MASK_FROM_ISR(x) vPortClearInterruptMaskFromISR(x)
#define portYIELD() vPortYield()

/* 本项目未使用 FromISR 队列/信号量切换；tick 中断内部完成切换。 */
#define portEND_SWITCHING_ISR(x) do { (void)(x); } while (0)
#define portYIELD_FROM_ISR(x) portEND_SWITCHING_ISR(x)

#define portTASK_FUNCTION_PROTO(vFunction, pvParameters) \
    void vFunction(void *pvParameters)
#define portTASK_FUNCTION(vFunction, pvParameters) \
    void vFunction(void *pvParameters)

#ifdef __cplusplus
}
#endif

#endif
