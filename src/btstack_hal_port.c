#include "ap6256_driver.h"
#include "cmsis_os2.h"
#include "main.h"

#include "hal_cpu.h"
#include "hal_time_ms.h"
#include "hal_uart_dma.h"

#include <stdint.h>
#include <string.h>

static void (*s_block_received)(void);
static void (*s_block_sent)(void);
static void (*s_csr_irq_handler)(void);

uint32_t hal_time_ms(void)
{
    return HAL_GetTick();
}

void hal_cpu_disable_irqs(void)
{
    __disable_irq();
}

void hal_cpu_enable_irqs(void)
{
    __enable_irq();
}

void hal_cpu_enable_irqs_and_sleep(void)
{
    __enable_irq();
    osDelay(1U);
}

void hal_uart_dma_init(void)
{
}

void hal_uart_dma_set_block_received(void (*callback)(void))
{
    s_block_received = callback;
}

void hal_uart_dma_set_block_sent(void (*callback)(void))
{
    s_block_sent = callback;
}

int hal_uart_dma_set_baud(uint32_t baud)
{
    if (baud == 0U) {
        return 0;
    }

    if (huart3.Init.BaudRate == baud) {
        return 0;
    }

    if (HAL_UART_DeInit(&huart3) != HAL_OK) {
        return -1;
    }

    huart3.Init.BaudRate = baud;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        return -1;
    }

#if defined(USART_CR1_FIFOEN)
    (void)HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8);
    (void)HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8);
    (void)HAL_UARTEx_DisableFifoMode(&huart3);
#endif

    return 0;
}

void hal_uart_dma_send_block(const uint8_t *buffer, uint16_t length)
{
    if (ap6256_bt_uart_write(buffer, length) == AP6256_STATUS_OK) {
        if (s_block_sent != NULL) {
            s_block_sent();
        }
    }
}

void hal_uart_dma_receive_block(uint8_t *buffer, uint16_t len)
{
    if (ap6256_bt_uart_read(buffer, len, 1000U) == AP6256_STATUS_OK) {
        if (s_block_received != NULL) {
            s_block_received();
        }
    }
}

void hal_uart_dma_set_csr_irq_handler(void (*csr_irq_handler)(void))
{
    s_csr_irq_handler = csr_irq_handler;
    (void)s_csr_irq_handler;
}

void hal_uart_dma_set_sleep(uint8_t sleep)
{
    (void)sleep;
}
