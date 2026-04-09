#include "main.h"
#include "board_test.h"
#include "FreeRTOS.h"
#include "task.h"

extern void xPortSysTickHandler(void);

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    while (1) {
    }
}

void MemManage_Handler(void)
{
    while (1) {
    }
}

void BusFault_Handler(void)
{
    while (1) {
    }
}

void UsageFault_Handler(void)
{
    while (1) {
    }
}

void DebugMon_Handler(void)
{
}

void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

void OTG_FS_IRQHandler(void)
{
    test_usb_irq_handler();
}
