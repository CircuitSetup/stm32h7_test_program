#include "ap6256_cyw43_port.h"

#include "ap6256_driver.h"
#include "cyw43_configport.h"
#include "main.h"

#include "cmsis_os2.h"
#include "stm32h7xx_ll_sdmmc.h"

#include <stdbool.h>
#include <string.h>

#define AP6256_CYW43_SDIO_CMD_TIMEOUT_MS 120U
#define AP6256_CYW43_SDIO_READY_TIMEOUT_MS 500U
#define AP6256_CYW43_WL_REG_ON_PORT GPIOC
#define AP6256_CYW43_WL_REG_ON_PIN  GPIO_PIN_7
#define AP6256_CYW43_SDIO_DAT1_PORT GPIOC
#define AP6256_CYW43_SDIO_DAT1_PIN  GPIO_PIN_9

static osMutexId_t s_cyw43_mutex;
static volatile uint8_t s_poll_pending;
static uint8_t s_sdio_open;
extern void (*cyw43_poll)(void);

static osMutexId_t ap6256_cyw43_mutex(void)
{
    static const osMutexAttr_t mutex_attr = {
        .name = "cyw43",
        .attr_bits = osMutexRecursive
    };

    if (s_cyw43_mutex != NULL) {
        return s_cyw43_mutex;
    }

    if ((osKernelGetState() == osKernelRunning) || (osKernelGetState() == osKernelReady)) {
        s_cyw43_mutex = osMutexNew(&mutex_attr);
    }

    return s_cyw43_mutex;
}

static void ap6256_cyw43_build_mac(uint8_t mac[6])
{
    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();
    uint32_t uid2 = HAL_GetUIDw2();

    mac[0] = 0x02U;
    mac[1] = (uint8_t)(uid0 & 0xFFU);
    mac[2] = (uint8_t)((uid0 >> 8U) & 0xFFU);
    mac[3] = (uint8_t)((uid0 >> 16U) & 0xFFU);
    mac[4] = (uint8_t)(uid1 & 0xFFU);
    mac[5] = (uint8_t)(uid2 & 0xFFU);
}

static int ap6256_cyw43_status_to_errno(ap6256_status_t st)
{
    switch (st) {
    case AP6256_STATUS_OK:
        return 0;
    case AP6256_STATUS_TIMEOUT:
        return -CYW43_ETIMEDOUT;
    case AP6256_STATUS_BAD_PARAM:
        return -CYW43_EINVAL;
    case AP6256_STATUS_IO_ERROR:
    case AP6256_STATUS_PROTOCOL_ERROR:
    default:
        return -CYW43_EIO;
    }
}

typedef enum {
    AP6256_CYW43_RESP_NONE = 0U,
    AP6256_CYW43_RESP_SHORT = 1U,
    AP6256_CYW43_RESP_LONG = 3U
} ap6256_cyw43_resp_t;

static ap6256_status_t ap6256_cyw43_send_cmd(uint32_t cmd_idx,
                                             uint32_t arg,
                                             ap6256_cyw43_resp_t waitresp,
                                             uint8_t ignore_crc_fail,
                                             uint32_t *response)
{
    uint32_t start = HAL_GetTick();
    uint32_t status;
    uint32_t cmd = SDMMC_CMD_CPSMEN | (cmd_idx & 0x3FU);

    if (waitresp == AP6256_CYW43_RESP_SHORT) {
        cmd |= SDMMC_CMD_WAITRESP_0;
    } else if (waitresp == AP6256_CYW43_RESP_LONG) {
        cmd |= SDMMC_CMD_WAITRESP_0 | SDMMC_CMD_WAITRESP_1;
    }

    SDMMC1->ICR = 0xFFFFFFFFU;
    SDMMC1->ARG = arg;
    SDMMC1->CMD = cmd;

    while (1) {
        status = SDMMC1->STA;

        if (waitresp == AP6256_CYW43_RESP_NONE) {
            if ((status & SDMMC_STA_CMDSENT) != 0U) {
                break;
            }
        } else {
            if ((status & SDMMC_STA_CMDREND) != 0U) {
                break;
            }
            if ((status & SDMMC_STA_CCRCFAIL) != 0U) {
                if (ignore_crc_fail != 0U) {
                    break;
                }
                SDMMC1->ICR = 0xFFFFFFFFU;
                return AP6256_STATUS_PROTOCOL_ERROR;
            }
        }

        if ((status & SDMMC_STA_CTIMEOUT) != 0U) {
            SDMMC1->ICR = 0xFFFFFFFFU;
            return AP6256_STATUS_TIMEOUT;
        }

        if ((HAL_GetTick() - start) > AP6256_CYW43_SDIO_CMD_TIMEOUT_MS) {
            SDMMC1->ICR = 0xFFFFFFFFU;
            return AP6256_STATUS_TIMEOUT;
        }
    }

    if (response != NULL) {
        *response = SDMMC1->RESP1;
    }

    SDMMC1->ICR = 0xFFFFFFFFU;
    return AP6256_STATUS_OK;
}

void ap6256_cyw43_port_init(void)
{
    (void)ap6256_cyw43_mutex();
}

void ap6256_cyw43_port_deinit(void)
{
    cyw43_poll = NULL;
    s_poll_pending = 0U;
}

void ap6256_cyw43_thread_enter(void)
{
    osMutexId_t mutex = ap6256_cyw43_mutex();

    if (mutex != NULL) {
        (void)osMutexAcquire(mutex, osWaitForever);
    }
}

void ap6256_cyw43_thread_exit(void)
{
    osMutexId_t mutex = ap6256_cyw43_mutex();

    if (mutex != NULL) {
        (void)osMutexRelease(mutex);
    }
}

unsigned int cyw43_hal_ticks_us(void)
{
    return (unsigned int)(HAL_GetTick() * 1000U);
}

unsigned int cyw43_hal_ticks_ms(void)
{
    return (unsigned int)HAL_GetTick();
}

void cyw43_delay_us(unsigned int us)
{
    uint32_t start = HAL_GetTick() * 1000U;

    while (((HAL_GetTick() * 1000U) - start) < us) {
    }
}

void cyw43_delay_ms(unsigned int ms)
{
    osDelay(ms);
}

void cyw43_hal_get_mac(int interface, uint8_t mac[6])
{
    (void)interface;
    if (mac == NULL) {
        return;
    }
    ap6256_cyw43_build_mac(mac);
}

void cyw43_hal_pin_config(int pin, int mode, int pull, int alt)
{
    (void)pin;
    (void)mode;
    (void)pull;
    (void)alt;
}

void cyw43_hal_pin_config_irq_falling(int pin, int enable)
{
    GPIO_InitTypeDef gpio;

    if (pin != CYW43_PIN_WL_SDIO_1) {
        return;
    }

    memset(&gpio, 0, sizeof(gpio));
    gpio.Pin = AP6256_CYW43_SDIO_DAT1_PIN;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    if (enable != 0) {
        gpio.Mode = GPIO_MODE_IT_FALLING;
        HAL_GPIO_Init(AP6256_CYW43_SDIO_DAT1_PORT, &gpio);
        HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6U, 0U);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    } else {
        gpio.Mode = GPIO_MODE_INPUT;
        HAL_GPIO_Init(AP6256_CYW43_SDIO_DAT1_PORT, &gpio);
        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    }
}

int cyw43_hal_pin_read(int pin)
{
    switch (pin) {
    case CYW43_PIN_WL_REG_ON:
        return (HAL_GPIO_ReadPin(AP6256_CYW43_WL_REG_ON_PORT, AP6256_CYW43_WL_REG_ON_PIN) == GPIO_PIN_SET) ? 1 : 0;
    case CYW43_PIN_WL_SDIO_1:
        return (HAL_GPIO_ReadPin(AP6256_CYW43_SDIO_DAT1_PORT, AP6256_CYW43_SDIO_DAT1_PIN) == GPIO_PIN_SET) ? 1 : 0;
    default:
        return 0;
    }
}

void cyw43_hal_pin_low(int pin)
{
    if (pin == CYW43_PIN_WL_REG_ON) {
        HAL_GPIO_WritePin(AP6256_CYW43_WL_REG_ON_PORT, AP6256_CYW43_WL_REG_ON_PIN, GPIO_PIN_RESET);
    }
}

void cyw43_hal_pin_high(int pin)
{
    if (pin == CYW43_PIN_WL_REG_ON) {
        HAL_GPIO_WritePin(AP6256_CYW43_WL_REG_ON_PORT, AP6256_CYW43_WL_REG_ON_PIN, GPIO_PIN_SET);
    }
}

void cyw43_schedule_internal_poll_dispatch(void (*func)(void))
{
    cyw43_poll = func;
    s_poll_pending = (func != NULL) ? 1U : 0U;
}

void ap6256_cyw43_port_poll(void)
{
    if (cyw43_poll == NULL) {
        return;
    }

    s_poll_pending = 0U;
    ap6256_cyw43_thread_enter();
    cyw43_poll();
    ap6256_cyw43_thread_exit();
}

uint8_t ap6256_cyw43_port_pending_poll(void)
{
    return s_poll_pending;
}

void cyw43_sdio_init(void)
{
    ap6256_sdio_session_t session;

    memset(&session, 0, sizeof(session));
    if (ap6256_sdio_open(&session, 1U, 0U) == AP6256_STATUS_OK) {
        s_sdio_open = 1U;
    } else {
        s_sdio_open = 0U;
    }
}

void cyw43_sdio_reinit(void)
{
    if (s_sdio_open == 0U) {
        cyw43_sdio_init();
    }
}

void cyw43_sdio_deinit(void)
{
    if (s_sdio_open != 0U) {
        ap6256_sdio_close();
        s_sdio_open = 0U;
    }
}

void cyw43_sdio_set_irq(bool enable)
{
    cyw43_hal_pin_config_irq_falling(CYW43_PIN_WL_SDIO_1, enable ? 1 : 0);
}

void cyw43_sdio_enable_high_speed_4bit(void)
{
    MODIFY_REG(SDMMC1->CLKCR,
               SDMMC_CLKCR_WIDBUS | SDMMC_CLKCR_CLKDIV,
               SDMMC_CLKCR_WIDBUS_0 | 1U);
}

int cyw43_sdio_transfer(uint32_t cmd, uint32_t arg, uint32_t *resp)
{
    ap6256_status_t st;

    switch (cmd) {
    case 0U:
        st = ap6256_cyw43_send_cmd(0U, 0U, AP6256_CYW43_RESP_NONE, 1U, NULL);
        break;
    case 3U:
        st = ap6256_cyw43_send_cmd(3U, 0U, AP6256_CYW43_RESP_SHORT, 1U, resp);
        break;
    case 5U:
        st = ap6256_cyw43_send_cmd(5U, arg, AP6256_CYW43_RESP_SHORT, 1U, resp);
        break;
    case 7U:
        st = ap6256_cyw43_send_cmd(7U, arg, AP6256_CYW43_RESP_SHORT, 1U, NULL);
        break;
    case 52U: {
        uint8_t value = 0U;
        uint8_t function = (uint8_t)((arg >> 28U) & 0x07U);
        uint32_t address = (arg >> 9U) & 0x1FFFFU;

        if ((arg & 0x80000000UL) != 0U) {
            st = ap6256_sdio_cmd52_write_u8(function, address, (uint8_t)(arg & 0xFFU));
            if ((st == AP6256_STATUS_OK) && (resp != NULL)) {
                *resp = (arg & 0xFFU);
            }
        } else {
            st = ap6256_sdio_cmd52_read_u8(function, address, &value);
            if ((st == AP6256_STATUS_OK) && (resp != NULL)) {
                *resp = value;
            }
        }
        break;
    }
    default:
        st = AP6256_STATUS_BAD_PARAM;
        break;
    }

    return ap6256_cyw43_status_to_errno(st);
}

int cyw43_sdio_transfer_cmd53(bool write, uint32_t block_size, uint32_t arg, size_t len, uint8_t *buf)
{
    ap6256_status_t st;
    uint8_t function = (uint8_t)((arg >> 28U) & 0x07U);
    uint8_t block_mode = (uint8_t)((arg >> 27U) & 0x01U);
    uint8_t op_code = (uint8_t)((arg >> 26U) & 0x01U);
    uint32_t address = (arg >> 9U) & 0x1FFFFU;

    (void)block_size;

    if (write) {
        st = ap6256_sdio_cmd53_write(function, address, buf, (uint32_t)len, block_mode, op_code);
    } else {
        st = ap6256_sdio_cmd53_read(function, address, buf, (uint32_t)len, block_mode, op_code);
    }

    return ap6256_cyw43_status_to_errno(st);
}

void EXTI9_5_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(AP6256_CYW43_SDIO_DAT1_PIN) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(AP6256_CYW43_SDIO_DAT1_PIN);
        s_poll_pending = 1U;
    }
}