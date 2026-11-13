/*
 * FreeRTOS MCS-251 移植层。
 *
 * 设计要点：
 *  - 所有任务切换统一走 4 字节中断帧（INT0 软件触发 / Timer0 tick）。
 *  - 每个任务上下文保存 21 字节：R0-R9、R12-R15、ACC、B、DPL、DPH、DPXL、
 *    PSW1、PSW。R10/R11 与 B/ACC 同址，通过 ACC/B 保存。
 *  - 任务栈放在 EDATA 0x0100 起的 2304 字节堆内；SPX 可直接寻址。
 *  - idle 任务静态栈在 0x0A00；主栈：应用 main 用 naked 包装切到 0x0AFF，
 *    避免默认 120 字节栈。
 */

#include "FreeRTOS.h"
#include "task.h"

/* tasks.c 中的私有全局；类型在这里按“3 字节指针”使用。 */
extern void * volatile pxCurrentTCB;

volatile unsigned int ulPortCurrentSP;
volatile unsigned int ulPortNextSP;
volatile unsigned char ulPortYieldPending;

static volatile UBaseType_t uxCriticalNesting = 0;

/* 堆放在 EDATA 0x0100-0x09FF，避开启动栈 0x08-0x7F 和 idle 栈 0x0A00。 */
#if (configAPPLICATION_ALLOCATED_HEAP == 1)
__xdata __at (0x000100) unsigned char ucHeap[configTOTAL_HEAP_SIZE];
#endif

#if (configSUPPORT_STATIC_ALLOCATION == 1)
static StaticTask_t xIdleTaskTCB;
__xdata __at (0x000A00) static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(
    StaticTask_t **ppxIdleTaskTCBBuffer,
    StackType_t **ppxIdleTaskStackBuffer,
    configSTACK_DEPTH_TYPE *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = xIdleStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
#endif

static unsigned int prvSPFromPointer(volatile StackType_t *pointer)
{
    return (unsigned int)(unsigned long)pointer;
}

static volatile StackType_t *prvPointerFromSP(unsigned int sp)
{
    return (volatile StackType_t *)(unsigned long)sp;
}

static volatile StackType_t **prvTopOfStackSlot(void)
{
    return (volatile StackType_t **)pxCurrentTCB;
}

static void prvSetupTimerInterrupt(void)
{
    unsigned int reload = (unsigned int)(65536UL - (FOSC / configTICK_RATE_HZ));

    TR0 = 0;
    TMOD = (unsigned char)(TMOD & 0xF0); /* Timer0 模式 0：16 位自动重载 */
    AUXR |= 0x80;                        /* T0x12：1T */
    TH0 = (unsigned char)(reload >> 8);
    TL0 = (unsigned char)(reload & 0xFF);
    ET0 = 1;
    TR0 = 1;
}

StackType_t *pxPortInitialiseStack(
    StackType_t *pxTopOfStack,
    TaskFunction_t pxCode,
    void *pvParameters)
{
    /* MCS-251 栈向高地址增长：pxTopOfStack 是栈底，初始帧从这里向上放。 */
    StackType_t *frame = pxTopOfStack;
    unsigned long code = (unsigned long)pxCode;
    unsigned long parameter = (unsigned long)pvParameters;
    unsigned char index;

    /* 4 字节中断帧：PSW1, PC[23:16], PC[7:0], PC[15:8]。 */
    frame[0] = 0x00;
    frame[1] = (StackType_t)((code >> 16) & 0xFF);
    frame[2] = (StackType_t)(code & 0xFF);
    frame[3] = (StackType_t)((code >> 8) & 0xFF);

    /* 14 个寄存器槽：R0-R9、R12-R15。 */
    for (index = 4; index < 18; index++)
        frame[index] = 0x00;

    frame[18] = 0x00;                                   /* ACC */
    frame[19] = (StackType_t)((parameter >> 16) & 0xFF); /* B（参数高字节） */
    frame[20] = (StackType_t)(parameter & 0xFF);         /* DPL */
    frame[21] = (StackType_t)((parameter >> 8) & 0xFF);  /* DPH */
    frame[22] = 0x00;                                   /* DPXL */
    frame[23] = 0x00;                                   /* PSW1 */
    frame[24] = 0x00;                                   /* PSW */

    return frame + 24;
}

/* 由汇编调用：tick 计数并按需切换。 */
void vPortTickHandler(void)
{
    *prvTopOfStackSlot() = prvPointerFromSP(ulPortCurrentSP);

#if (configUSE_PREEMPTION == 1)
    if (xTaskIncrementTick() != pdFALSE)
    {
        ulPortYieldPending = 0;
        vTaskSwitchContext();
    }
    else if (ulPortYieldPending)
    {
        ulPortYieldPending = 0;
        vTaskSwitchContext();
    }
#else
    if (ulPortYieldPending)
    {
        ulPortYieldPending = 0;
        vTaskSwitchContext();
    }
#endif

    ulPortNextSP = prvSPFromPointer(*prvTopOfStackSlot());
}

void vPortRestoreContext(void) __naked
{
    __asm
        mov dptr,#_ulPortNextSP
        mov dpxl,#(_ulPortNextSP >> 16)
        mov a,@dpx
        mov sph,a
        inc dpx
        mov a,@dpx
        mov sp,a

        pop psw
        pop 0xd1
        pop 0x84
        pop dph
        pop dpl
        pop b
        pop acc
        pop r15
        pop r14
        pop r13
        pop r12
        pop r9
        pop r8
        pop r7
        pop r6
        pop r5
        pop r4
        pop r3
        pop r2
        pop r1
        pop r0

        setb ea
        reti
    __endasm;
}

void vPortStartFirstTask(void)
{
    ulPortNextSP = prvSPFromPointer(*prvTopOfStackSlot());
    vPortRestoreContext();
}

/*
 * taskYIELD：置位请求后等待 Timer0 tick 中断完成切换。
 * 所有切换都走 Timer0 的 4 字节中断帧，避免 3 字节调用帧与 4 字节中断帧混用。
 */
void vPortYield(void)
{
    if (!EA)
        return;

    ulPortYieldPending = 1;
    while (ulPortYieldPending)
    {
    }
}

void vPortTickISR(void) __interrupt (1) __naked
{
    __asm
        clr ea

        push r0
        push r1
        push r2
        push r3
        push r4
        push r5
        push r6
        push r7
        push r8
        push r9
        push r12
        push r13
        push r14
        push r15
        push acc
        push b
        push dpl
        push dph
        push 0x84
        push 0xd1
        push psw

        mov dptr,#_ulPortCurrentSP
        mov dpxl,#(_ulPortCurrentSP >> 16)
        mov a,sph
        mov @dpx,a
        inc dpx
        mov a,sp
        mov @dpx,a

        ecall _vPortTickHandler
        ecall _vPortRestoreContext
    __endasm;
}

void vPortEnterCritical(void)
{
    portDISABLE_INTERRUPTS();
    uxCriticalNesting++;
}

void vPortExitCritical(void)
{
    if (uxCriticalNesting > 0)
    {
        uxCriticalNesting--;
        if (uxCriticalNesting == 0)
            portENABLE_INTERRUPTS();
    }
}

UBaseType_t ulPortSetInterruptMaskFromISR(void)
{
    UBaseType_t mask = EA ? 1u : 0u;
    EA = 0;
    return mask;
}

void vPortClearInterruptMaskFromISR(UBaseType_t mask)
{
    if (mask)
        EA = 1;
}

BaseType_t xPortStartScheduler(void)
{
    uxCriticalNesting = 0;
    ulPortYieldPending = 0;

    /* M1 协作式也需要 tick 来承载 taskYIELD 的上下文切换。 */
    prvSetupTimerInterrupt();

    vPortStartFirstTask();
    return pdFALSE;
}

void vPortEndScheduler(void)
{
}

void vAssertCalled(const char *file, unsigned long line)
{
    (void)file;
    (void)line;
    portDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    vAssertCalled(__FILE__, __LINE__);
}

void vApplicationMallocFailedHook(void)
{
    vAssertCalled(__FILE__, __LINE__);
}
