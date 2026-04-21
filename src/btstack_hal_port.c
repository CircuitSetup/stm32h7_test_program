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
static uint8_t *s_rx_buffer;
static uint16_t s_rx_len;
static uint16_t s_rx_offset;
static uint8_t s_rx_active;
static uint8_t s_irq_byte;
static uint8_t s_irq_active;
static volatile uint16_t s_ring_head;
static volatile uint16_t s_ring_tail;
static uint8_t s_rx_ring[512];
static volatile uint32_t s_tx_blocks;
static volatile uint32_t s_tx_bytes;
static volatile uint32_t s_rx_irq_bytes;
static volatile uint32_t s_rx_blocks_complete;
static volatile uint32_t s_rx_errors;
static volatile uint32_t s_rx_overruns;
static uint8_t s_hw_flowcontrol_enabled;

static uint16_t hal_uart_dma_ring_count(void)
{
    uint16_t head = s_ring_head;
    uint16_t tail = s_ring_tail;

    if (head >= tail) {
        return (uint16_t)(head - tail);
    }
    return (uint16_t)(sizeof(s_rx_ring) - tail + head);
}

static void hal_uart_dma_start_irq_receive(void)
{
    if (s_irq_active != 0U) {
        return;
    }

    __HAL_UART_CLEAR_OREFLAG(&huart3);
    __HAL_UART_CLEAR_FEFLAG(&huart3);
    __HAL_UART_CLEAR_NEFLAG(&huart3);
    if (HAL_UART_Receive_IT(&huart3, &s_irq_byte, 1U) == HAL_OK) {
        s_irq_active = 1U;
    }
}

static void hal_uart_dma_ring_push(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_ring_head + 1U) % sizeof(s_rx_ring));

    if (next == s_ring_tail) {
        s_rx_overruns++;
        return;
    }

    s_rx_ring[s_ring_head] = byte;
    s_ring_head = next;
}

static uint8_t hal_uart_dma_ring_pop(uint8_t *byte)
{
    if ((byte == NULL) || (s_ring_tail == s_ring_head)) {
        return 0U;
    }

    *byte = s_rx_ring[s_ring_tail];
    s_ring_tail = (uint16_t)((s_ring_tail + 1U) % sizeof(s_rx_ring));
    return 1U;
}

static uint8_t hal_uart_dma_poll_receive(void)
{
    if ((s_rx_active == 0U) || (s_rx_buffer == NULL) || (s_rx_len == 0U)) {
        return 0U;
    }

    while (s_rx_offset < s_rx_len) {
        if (hal_uart_dma_ring_pop(&s_rx_buffer[s_rx_offset]) == 0U) {
            hal_uart_dma_start_irq_receive();
            return 0U;
        }
        s_rx_offset++;
    }

    s_rx_active = 0U;
    s_rx_buffer = NULL;
    s_rx_len = 0U;
    s_rx_offset = 0U;
    s_rx_blocks_complete++;
    if (s_block_received != NULL) {
        s_block_received();
    }
    hal_uart_dma_start_irq_receive();
    return 1U;
}

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
    if (hal_uart_dma_poll_receive() != 0U) {
        return;
    }
    osDelay(1U);
}

void hal_uart_dma_init(void)
{
    (void)HAL_UART_AbortReceive_IT(&huart3);
    s_rx_buffer = NULL;
    s_rx_len = 0U;
    s_rx_offset = 0U;
    s_rx_active = 0U;
    s_irq_active = 0U;
    s_ring_head = 0U;
    s_ring_tail = 0U;
    s_tx_blocks = 0U;
    s_tx_bytes = 0U;
    s_rx_irq_bytes = 0U;
    s_rx_blocks_complete = 0U;
    s_rx_errors = 0U;
    s_rx_overruns = 0U;
    s_hw_flowcontrol_enabled = 1U;
}

void hal_uart_dma_deinit(void)
{
    (void)HAL_UART_AbortReceive_IT(&huart3);
    s_rx_buffer = NULL;
    s_rx_len = 0U;
    s_rx_offset = 0U;
    s_rx_active = 0U;
    s_irq_active = 0U;
    s_ring_head = 0U;
    s_ring_tail = 0U;
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

    if (HAL_UART_DeInit(&huart3) != HAL_OK) {
        return -1;
    }

    s_irq_active = 0U;
    huart3.Init.BaudRate = baud;
    huart3.Init.HwFlowCtl = (s_hw_flowcontrol_enabled != 0U) ? UART_HWCONTROL_RTS_CTS : UART_HWCONTROL_NONE;
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

int hal_uart_dma_set_flowcontrol(int flowcontrol)
{
    uint8_t enabled = (flowcontrol != 0) ? 1U : 0U;

    if (s_hw_flowcontrol_enabled == enabled) {
        return 0;
    }

    s_hw_flowcontrol_enabled = enabled;
    return hal_uart_dma_set_baud(huart3.Init.BaudRate);
}

void hal_uart_dma_send_block(const uint8_t *buffer, uint16_t length)
{
    s_tx_blocks++;
    s_tx_bytes += length;
    if (ap6256_bt_uart_write(buffer, length) == AP6256_STATUS_OK) {
        if (s_block_sent != NULL) {
            s_block_sent();
        }
    }
}

void hal_uart_dma_receive_block(uint8_t *buffer, uint16_t len)
{
    if ((buffer == NULL) || (len == 0U)) {
        return;
    }

    s_rx_buffer = buffer;
    s_rx_len = len;
    s_rx_offset = 0U;
    s_rx_active = 1U;
    hal_uart_dma_poll_receive();
}

uint8_t hal_uart_dma_poll(void)
{
    return hal_uart_dma_poll_receive();
}

void hal_uart_dma_get_diag(hal_uart_dma_diag_t *diag)
{
    if (diag == NULL) {
        return;
    }

    memset(diag, 0, sizeof(*diag));
    diag->tx_blocks = s_tx_blocks;
    diag->tx_bytes = s_tx_bytes;
    diag->rx_irq_bytes = s_rx_irq_bytes;
    diag->rx_blocks_complete = s_rx_blocks_complete;
    diag->rx_errors = s_rx_errors;
    diag->rx_overruns = s_rx_overruns;
    diag->pending_len = s_rx_len;
    diag->pending_offset = s_rx_offset;
    diag->ring_count = hal_uart_dma_ring_count();
    diag->irq_active = s_irq_active;
    diag->rx_active = s_rx_active;
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != USART3)) {
        return;
    }

    s_irq_active = 0U;
    s_rx_irq_bytes++;
    hal_uart_dma_ring_push(s_irq_byte);
    (void)hal_uart_dma_poll_receive();
    hal_uart_dma_start_irq_receive();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != USART3)) {
        return;
    }

    s_irq_active = 0U;
    s_rx_errors++;
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    __HAL_UART_CLEAR_FEFLAG(&huart3);
    __HAL_UART_CLEAR_NEFLAG(&huart3);
    hal_uart_dma_start_irq_receive();
}
