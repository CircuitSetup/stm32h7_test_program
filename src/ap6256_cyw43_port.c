#include "ap6256_cyw43_port.h"

#include "ap6256_cyw43_compat.h"
#include "ap6256_driver.h"
#include "cyw43_configport.h"
#include "main.h"

#include "cmsis_os2.h"
#include "stm32h7xx_ll_sdmmc.h"

#include <stdbool.h>
#include <string.h>
#include <stdint.h>

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
static volatile uint32_t s_cyw43_sched_calls;
static volatile uintptr_t s_cyw43_last_sched_func;
static volatile uint32_t s_cyw43_sdio_init_calls;
static volatile uint32_t s_cyw43_sdio_reinit_calls;
static volatile int32_t s_cyw43_last_bus_init_ret;
static volatile uint32_t s_cyw43_bus_stage;
static volatile int32_t s_cyw43_poll_header_read_status;
static volatile int32_t s_cyw43_poll_payload_read_status;
static volatile uint16_t s_cyw43_poll_hdr0;
static volatile uint16_t s_cyw43_poll_hdr1;
static volatile uint8_t s_cyw43_last_cmd53_write;
static volatile uint8_t s_cyw43_last_cmd53_function;
static volatile uint8_t s_cyw43_last_cmd53_block_mode;
static volatile uint32_t s_cyw43_last_cmd53_block_size;
static volatile uint32_t s_cyw43_last_cmd53_length;
static volatile int32_t s_cyw43_last_cmd53_status;
static volatile uint16_t s_cyw43_last_cmd53_frame_size;
static volatile uint32_t s_cyw43_last_cmd53_count;
static volatile uint32_t s_cyw43_last_cmd;
static volatile uint32_t s_cyw43_last_cmd_arg;
static volatile int32_t s_cyw43_last_cmd_status;
static volatile uint32_t s_cyw43_last_cmd_response;
static volatile uint32_t s_cyw43_chip_id_raw;
static volatile uint32_t s_cyw43_ram_base_addr;
static volatile uint32_t s_cyw43_ram_size_bytes;
static volatile uint32_t s_cyw43_boot_mode;
static volatile uint32_t s_cyw43_cpu_wrapper_addr;
static volatile uint32_t s_cyw43_ram_wrapper_addr;
static volatile uint32_t s_cyw43_firmware_addr;
static volatile uint32_t s_cyw43_nvram_addr;
static volatile uint32_t s_cyw43_footer_addr;
static volatile uint32_t s_cyw43_cpu_core_id;
static volatile uint32_t s_cyw43_ram_core_id;
static volatile uint32_t s_cyw43_reset_vector_addr;
static volatile uint32_t s_cyw43_reset_vector_value;
static volatile uint32_t s_cyw43_verify_mismatch_addr;
static volatile uint32_t s_cyw43_verify_expected;
static volatile uint32_t s_cyw43_verify_actual;
static volatile uint32_t s_cyw43_nvram_packed_len;
static volatile uint32_t s_cyw43_nvram_padded_len;
static volatile uint32_t s_cyw43_nvram_footer_word;
static volatile uint8_t s_cyw43_nvram_using_reference;
static volatile uint8_t s_cyw43_chip_clock_csr_diag;
static volatile uint32_t s_cyw43_sr_control1_diag;
static volatile uint32_t s_cyw43_wlan_ioctrl_diag;
static volatile uint32_t s_cyw43_wlan_resetctrl_diag;
static volatile uint32_t s_cyw43_socram_ioctrl_diag;
static volatile uint32_t s_cyw43_socram_resetctrl_diag;
static volatile uint8_t s_cyw43_wakeup_ctrl_diag;
static volatile uint8_t s_cyw43_sleep_csr_diag;
static volatile uint8_t s_cyw43_cardcap_diag;
static volatile uint8_t s_cyw43_io_ready_diag;
static volatile uint32_t s_cyw43_checkpoint;
static volatile uint32_t s_cyw43_checkpoint_result;
static volatile uint32_t s_cyw43_last_success_checkpoint;
static volatile uint8_t s_cyw43_checkpoint_function;
static volatile uint32_t s_cyw43_checkpoint_address;
static volatile uint32_t s_cyw43_checkpoint_write_value;
static volatile uint32_t s_cyw43_checkpoint_readback_value;
static volatile int32_t s_cyw43_checkpoint_status;
static volatile uint32_t s_cyw43_profile;
static volatile uint8_t s_cyw43_backplane_is_write;
static volatile uint32_t s_cyw43_backplane_address;
static volatile uint8_t s_cyw43_backplane_width_bytes;
static volatile int32_t s_cyw43_backplane_status;
static volatile uint8_t s_cyw43_reference_nvram_enabled;

static void ap6256_cyw43_port_record_cmd53(uint8_t write,
                                           uint8_t function,
                                           uint8_t block_mode,
                                           uint32_t block_size,
                                           uint32_t len,
                                           int32_t status,
                                           const uint8_t *buf)
{
    s_cyw43_last_cmd53_count++;
    s_cyw43_last_cmd53_write = write;
    s_cyw43_last_cmd53_function = function;
    s_cyw43_last_cmd53_block_mode = block_mode;
    s_cyw43_last_cmd53_block_size = block_size;
    s_cyw43_last_cmd53_length = len;
    s_cyw43_last_cmd53_status = status;
    s_cyw43_last_cmd53_frame_size = 0U;

    if ((write == 0U) && (status == 0) && (buf != NULL) && (len >= 2U)) {
        s_cyw43_last_cmd53_frame_size = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8U);
    }
}

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
    s_cyw43_sched_calls = 0U;
    s_cyw43_last_sched_func = 0U;
    s_cyw43_sdio_init_calls = 0U;
    s_cyw43_sdio_reinit_calls = 0U;
    s_cyw43_last_bus_init_ret = 0;
    s_cyw43_bus_stage = 0U;
    s_cyw43_poll_header_read_status = 0;
    s_cyw43_poll_payload_read_status = 0;
    s_cyw43_poll_hdr0 = 0U;
    s_cyw43_poll_hdr1 = 0U;
    s_cyw43_last_cmd53_write = 0U;
    s_cyw43_last_cmd53_function = 0U;
    s_cyw43_last_cmd53_block_mode = 0U;
    s_cyw43_last_cmd53_block_size = 0U;
    s_cyw43_last_cmd53_length = 0U;
    s_cyw43_last_cmd53_status = 0;
    s_cyw43_last_cmd53_frame_size = 0U;
    s_cyw43_last_cmd53_count = 0U;
    s_cyw43_last_cmd = 0U;
    s_cyw43_last_cmd_arg = 0U;
    s_cyw43_last_cmd_status = 0;
    s_cyw43_last_cmd_response = 0U;
    s_cyw43_chip_id_raw = 0U;
    s_cyw43_ram_base_addr = 0U;
    s_cyw43_ram_size_bytes = 0U;
    s_cyw43_boot_mode = AP6256_CYW43_BOOT_CM3_SOCRAM;
    s_cyw43_cpu_wrapper_addr = 0U;
    s_cyw43_ram_wrapper_addr = 0U;
    s_cyw43_firmware_addr = 0U;
    s_cyw43_nvram_addr = 0U;
    s_cyw43_footer_addr = 0U;
    s_cyw43_cpu_core_id = 0U;
    s_cyw43_ram_core_id = 0U;
    s_cyw43_reset_vector_addr = 0U;
    s_cyw43_reset_vector_value = 0U;
    s_cyw43_verify_mismatch_addr = 0U;
    s_cyw43_verify_expected = 0U;
    s_cyw43_verify_actual = 0U;
    s_cyw43_nvram_packed_len = 0U;
    s_cyw43_nvram_padded_len = 0U;
    s_cyw43_nvram_footer_word = 0U;
    s_cyw43_nvram_using_reference = 0U;
    s_cyw43_chip_clock_csr_diag = 0U;
    s_cyw43_sr_control1_diag = 0U;
    s_cyw43_wlan_ioctrl_diag = 0U;
    s_cyw43_wlan_resetctrl_diag = 0U;
    s_cyw43_socram_ioctrl_diag = 0U;
    s_cyw43_socram_resetctrl_diag = 0U;
    s_cyw43_wakeup_ctrl_diag = 0U;
    s_cyw43_sleep_csr_diag = 0U;
    s_cyw43_cardcap_diag = 0U;
    s_cyw43_io_ready_diag = 0U;
    s_cyw43_checkpoint = 0U;
    s_cyw43_checkpoint_result = AP6256_CYW43_CP_RESULT_PENDING;
    s_cyw43_last_success_checkpoint = 0U;
    s_cyw43_checkpoint_function = 0U;
    s_cyw43_checkpoint_address = 0U;
    s_cyw43_checkpoint_write_value = 0U;
    s_cyw43_checkpoint_readback_value = 0U;
    s_cyw43_checkpoint_status = 0;
    s_cyw43_profile = AP6256_CYW43_PROFILE_BASELINE;
    s_cyw43_backplane_is_write = 0U;
    s_cyw43_backplane_address = 0U;
    s_cyw43_backplane_width_bytes = 0U;
    s_cyw43_backplane_status = 0;
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
    /*
     * CYW43 uses WL_SDIO_1/DAT1 as its in-band interrupt source in SDIO mode.
     * On STM32H7, reconfiguring PC9 from SDMMC alternate-function into EXTI GPIO
     * steals DAT1 away from the 4-bit SDIO bus and can wedge subsequent transfers.
     *
     * Keep DAT1 owned by SDMMC1 at all times and treat interrupt enable/disable
     * as a logical no-op. The runtime already polls from a dedicated FreeRTOS task,
     * and cyw43_cb_read_host_interrupt_pin() can still sample the live DAT1 level.
     */
    (void)pin;
    (void)enable;
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
    s_cyw43_sched_calls++;
    s_cyw43_last_sched_func = (uintptr_t)func;
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
    ap6256_status_t st;

    s_cyw43_sdio_init_calls++;
    st = ap6256_sdio_bus_prepare(1U, 0U);
    s_sdio_open = (st == AP6256_STATUS_OK) ? 1U : 0U;
}

void cyw43_sdio_reinit(void)
{
    s_cyw43_sdio_reinit_calls++;

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
    /*
     * See cyw43_hal_pin_config_irq_falling(): do not hand SDIO DAT1 to EXTI on
     * this board. Leave the pin mux on SDMMC1 and rely on the Wi-Fi poll task.
     */
    (void)enable;
}

void cyw43_sdio_enable_high_speed_4bit(void)
{
    /*
     * Keep the bus conservative during bring-up. The previous divider of 1 was
     * aggressive enough that short register traffic could work while the first
     * sustained firmware-download block transfer timed out.
     */
    MODIFY_REG(SDMMC1->CLKCR,
               SDMMC_CLKCR_WIDBUS | SDMMC_CLKCR_CLKDIV,
               SDMMC_CLKCR_WIDBUS_0 | 8U);
}

int cyw43_sdio_transfer(uint32_t cmd, uint32_t arg, uint32_t *resp)
{
    ap6256_status_t st;
    uint32_t local_resp = 0U;

    switch (cmd) {
    case 0U:
        st = ap6256_cyw43_send_cmd(0U, 0U, AP6256_CYW43_RESP_NONE, 1U, NULL);
        break;
    case 3U:
        st = ap6256_cyw43_send_cmd(3U, 0U, AP6256_CYW43_RESP_SHORT, 1U, &local_resp);
        break;
    case 5U:
        st = ap6256_cyw43_send_cmd(5U, arg, AP6256_CYW43_RESP_SHORT, 1U, &local_resp);
        break;
    case 7U:
        st = ap6256_cyw43_send_cmd(7U, arg, AP6256_CYW43_RESP_SHORT, 1U, NULL);
        if (st == AP6256_STATUS_OK) {
            ap6256_sdio_session_adopt((uint16_t)((arg >> 16U) & 0xFFFFU));
        }
        break;
    case 52U: {
        uint8_t value = 0U;
        uint8_t function = (uint8_t)((arg >> 28U) & 0x07U);
        uint32_t address = (arg >> 9U) & 0x1FFFFU;

        if ((arg & 0x80000000UL) != 0U) {
            st = ap6256_sdio_cmd52_write_u8(function, address, (uint8_t)(arg & 0xFFU));
            if (st == AP6256_STATUS_OK) {
                local_resp = (arg & 0xFFU);
            }
        } else {
            st = ap6256_sdio_cmd52_read_u8(function, address, &value);
            if (st == AP6256_STATUS_OK) {
                local_resp = value;
            }
        }
        break;
    }
    default:
        st = AP6256_STATUS_BAD_PARAM;
        break;
    }

    if (resp != NULL) {
        *resp = local_resp;
    }

    ap6256_cyw43_port_record_last_cmd(cmd, arg, ap6256_cyw43_status_to_errno(st), local_resp);
    return ap6256_cyw43_status_to_errno(st);
}

int cyw43_sdio_transfer_cmd53(bool write, uint32_t block_size, uint32_t arg, size_t len, uint8_t *buf)
{
    ap6256_status_t st;
    int result;
    uint8_t function = (uint8_t)((arg >> 28U) & 0x07U);
    uint8_t block_mode = (uint8_t)((arg >> 27U) & 0x01U);
    uint8_t op_code = (uint8_t)((arg >> 26U) & 0x01U);
    uint32_t address = (arg >> 9U) & 0x1FFFFU;

    if (write) {
        st = ap6256_sdio_cmd53_write_ex(function, address, buf, (uint32_t)len, block_mode, op_code, block_size);
    } else {
        st = ap6256_sdio_cmd53_read_ex(function, address, buf, (uint32_t)len, block_mode, op_code, block_size);
    }

    result = ap6256_cyw43_status_to_errno(st);
    ap6256_cyw43_port_record_cmd53(write ? 1U : 0U,
                                   function,
                                   block_mode,
                                   block_size,
                                   (uint32_t)len,
                                   result,
                                   buf);
    return result;
}

void EXTI9_5_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(AP6256_CYW43_SDIO_DAT1_PIN) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(AP6256_CYW43_SDIO_DAT1_PIN);
        s_poll_pending = 1U;
    }
}

uint32_t ap6256_cyw43_port_sched_calls(void)
{
    return s_cyw43_sched_calls;
}

uintptr_t ap6256_cyw43_port_last_sched_func(void)
{
    return s_cyw43_last_sched_func;
}

uint32_t ap6256_cyw43_port_sdio_init_calls(void)
{
    return s_cyw43_sdio_init_calls;
}

uint32_t ap6256_cyw43_port_sdio_reinit_calls(void)
{
    return s_cyw43_sdio_reinit_calls;
}

void ap6256_cyw43_port_record_last_cmd(uint32_t cmd, uint32_t arg, int32_t status, uint32_t response)
{
    s_cyw43_last_cmd = cmd;
    s_cyw43_last_cmd_arg = arg;
    s_cyw43_last_cmd_status = status;
    s_cyw43_last_cmd_response = response;
}

uint32_t ap6256_cyw43_port_last_cmd(void)
{
    return s_cyw43_last_cmd;
}

uint32_t ap6256_cyw43_port_last_cmd_arg(void)
{
    return s_cyw43_last_cmd_arg;
}

int32_t ap6256_cyw43_port_last_cmd_status(void)
{
    return s_cyw43_last_cmd_status;
}

uint32_t ap6256_cyw43_port_last_cmd_response(void)
{
    return s_cyw43_last_cmd_response;
}

void ap6256_cyw43_port_set_chip_id_raw(uint32_t value)
{
    s_cyw43_chip_id_raw = value;
}

uint32_t ap6256_cyw43_port_chip_id_raw(void)
{
    return s_cyw43_chip_id_raw;
}

void ap6256_cyw43_port_set_ram_size_bytes(uint32_t value)
{
    s_cyw43_ram_size_bytes = value;
}

void ap6256_cyw43_port_set_ram_base_addr(uint32_t value)
{
    s_cyw43_ram_base_addr = value;
}

uint32_t ap6256_cyw43_port_ram_base_addr(void)
{
    return s_cyw43_ram_base_addr;
}

uint32_t ap6256_cyw43_port_ram_size_bytes(void)
{
    return s_cyw43_ram_size_bytes;
}

void ap6256_cyw43_port_set_boot_mode(uint32_t value)
{
    s_cyw43_boot_mode = value;
}

uint32_t ap6256_cyw43_port_boot_mode(void)
{
    return s_cyw43_boot_mode;
}

void ap6256_cyw43_port_set_boot_descriptor(uint32_t cpu_wrapper,
                                           uint32_t ram_wrapper,
                                           uint32_t firmware_addr,
                                           uint32_t nvram_addr,
                                           uint32_t footer_addr)
{
    s_cyw43_cpu_wrapper_addr = cpu_wrapper;
    s_cyw43_ram_wrapper_addr = ram_wrapper;
    s_cyw43_firmware_addr = firmware_addr;
    s_cyw43_nvram_addr = nvram_addr;
    s_cyw43_footer_addr = footer_addr;
}

void ap6256_cyw43_port_set_boot_topology(uint32_t cpu_core_id,
                                         uint32_t ram_core_id,
                                         uint32_t reset_vector_addr,
                                         uint32_t reset_vector_value)
{
    s_cyw43_cpu_core_id = cpu_core_id;
    s_cyw43_ram_core_id = ram_core_id;
    s_cyw43_reset_vector_addr = reset_vector_addr;
    s_cyw43_reset_vector_value = reset_vector_value;
}

uint32_t ap6256_cyw43_port_cpu_wrapper_addr(void)
{
    return s_cyw43_cpu_wrapper_addr;
}

uint32_t ap6256_cyw43_port_ram_wrapper_addr(void)
{
    return s_cyw43_ram_wrapper_addr;
}

uint32_t ap6256_cyw43_port_firmware_addr(void)
{
    return s_cyw43_firmware_addr;
}

uint32_t ap6256_cyw43_port_nvram_addr(void)
{
    return s_cyw43_nvram_addr;
}

uint32_t ap6256_cyw43_port_footer_addr(void)
{
    return s_cyw43_footer_addr;
}

uint32_t ap6256_cyw43_port_cpu_core_id(void)
{
    return s_cyw43_cpu_core_id;
}

uint32_t ap6256_cyw43_port_ram_core_id(void)
{
    return s_cyw43_ram_core_id;
}

uint32_t ap6256_cyw43_port_reset_vector_addr(void)
{
    return s_cyw43_reset_vector_addr;
}

uint32_t ap6256_cyw43_port_reset_vector_value(void)
{
    return s_cyw43_reset_vector_value;
}

void ap6256_cyw43_port_set_verify_mismatch(uint32_t address,
                                           uint32_t expected,
                                           uint32_t actual)
{
    s_cyw43_verify_mismatch_addr = address;
    s_cyw43_verify_expected = expected;
    s_cyw43_verify_actual = actual;
}

uint32_t ap6256_cyw43_port_verify_mismatch_addr(void)
{
    return s_cyw43_verify_mismatch_addr;
}

uint32_t ap6256_cyw43_port_verify_expected(void)
{
    return s_cyw43_verify_expected;
}

uint32_t ap6256_cyw43_port_verify_actual(void)
{
    return s_cyw43_verify_actual;
}

void ap6256_cyw43_port_set_nvram_metrics(uint32_t packed_len,
                                         uint32_t padded_len,
                                         uint32_t footer_word,
                                         uint8_t using_reference)
{
    s_cyw43_nvram_packed_len = packed_len;
    s_cyw43_nvram_padded_len = padded_len;
    s_cyw43_nvram_footer_word = footer_word;
    s_cyw43_nvram_using_reference = using_reference;
}

uint32_t ap6256_cyw43_port_nvram_packed_len(void)
{
    return s_cyw43_nvram_packed_len;
}

uint32_t ap6256_cyw43_port_nvram_padded_len(void)
{
    return s_cyw43_nvram_padded_len;
}

uint32_t ap6256_cyw43_port_nvram_footer_word(void)
{
    return s_cyw43_nvram_footer_word;
}

uint8_t ap6256_cyw43_port_nvram_using_reference(void)
{
    return s_cyw43_nvram_using_reference;
}

void ap6256_cyw43_port_set_core_diag(uint8_t chip_clock_csr,
                                     uint32_t sr_control1,
                                     uint32_t wlan_ioctrl,
                                     uint32_t wlan_resetctrl,
                                     uint32_t socram_ioctrl,
                                     uint32_t socram_resetctrl)
{
    s_cyw43_chip_clock_csr_diag = chip_clock_csr;
    s_cyw43_sr_control1_diag = sr_control1;
    s_cyw43_wlan_ioctrl_diag = wlan_ioctrl;
    s_cyw43_wlan_resetctrl_diag = wlan_resetctrl;
    s_cyw43_socram_ioctrl_diag = socram_ioctrl;
    s_cyw43_socram_resetctrl_diag = socram_resetctrl;
}

uint8_t ap6256_cyw43_port_chip_clock_csr_diag(void)
{
    return s_cyw43_chip_clock_csr_diag;
}

uint32_t ap6256_cyw43_port_sr_control1_diag(void)
{
    return s_cyw43_sr_control1_diag;
}

uint32_t ap6256_cyw43_port_wlan_ioctrl_diag(void)
{
    return s_cyw43_wlan_ioctrl_diag;
}

uint32_t ap6256_cyw43_port_wlan_resetctrl_diag(void)
{
    return s_cyw43_wlan_resetctrl_diag;
}

uint32_t ap6256_cyw43_port_socram_ioctrl_diag(void)
{
    return s_cyw43_socram_ioctrl_diag;
}

uint32_t ap6256_cyw43_port_socram_resetctrl_diag(void)
{
    return s_cyw43_socram_resetctrl_diag;
}

void ap6256_cyw43_port_set_checkpoint(uint32_t checkpoint,
                                      uint32_t checkpoint_result,
                                      uint8_t function,
                                      uint32_t address,
                                      uint32_t write_value,
                                      uint32_t readback_value,
                                      int32_t status)
{
    s_cyw43_checkpoint = checkpoint;
    s_cyw43_checkpoint_result = checkpoint_result;
    if (checkpoint_result == AP6256_CYW43_CP_RESULT_OK) {
        s_cyw43_last_success_checkpoint = checkpoint;
    }
    s_cyw43_checkpoint_function = function;
    s_cyw43_checkpoint_address = address;
    s_cyw43_checkpoint_write_value = write_value;
    s_cyw43_checkpoint_readback_value = readback_value;
    s_cyw43_checkpoint_status = status;
}

uint32_t ap6256_cyw43_port_checkpoint(void)
{
    return s_cyw43_checkpoint;
}

uint32_t ap6256_cyw43_port_checkpoint_result(void)
{
    return s_cyw43_checkpoint_result;
}

uint32_t ap6256_cyw43_port_last_success_checkpoint(void)
{
    return s_cyw43_last_success_checkpoint;
}

uint8_t ap6256_cyw43_port_checkpoint_function(void)
{
    return s_cyw43_checkpoint_function;
}

uint32_t ap6256_cyw43_port_checkpoint_address(void)
{
    return s_cyw43_checkpoint_address;
}

uint32_t ap6256_cyw43_port_checkpoint_write_value(void)
{
    return s_cyw43_checkpoint_write_value;
}

uint32_t ap6256_cyw43_port_checkpoint_readback_value(void)
{
    return s_cyw43_checkpoint_readback_value;
}

int32_t ap6256_cyw43_port_checkpoint_status(void)
{
    return s_cyw43_checkpoint_status;
}

void ap6256_cyw43_port_set_profile(uint32_t profile)
{
    s_cyw43_profile = profile;
}

uint32_t ap6256_cyw43_port_profile(void)
{
    return s_cyw43_profile;
}

void ap6256_cyw43_port_set_stage5_snapshot(uint8_t wakeup_ctrl,
                                           uint8_t sleep_csr,
                                           uint8_t cardcap,
                                           uint8_t io_ready)
{
    s_cyw43_wakeup_ctrl_diag = wakeup_ctrl;
    s_cyw43_sleep_csr_diag = sleep_csr;
    s_cyw43_cardcap_diag = cardcap;
    s_cyw43_io_ready_diag = io_ready;
}

uint8_t ap6256_cyw43_port_wakeup_ctrl_diag(void)
{
    return s_cyw43_wakeup_ctrl_diag;
}

uint8_t ap6256_cyw43_port_sleep_csr_diag(void)
{
    return s_cyw43_sleep_csr_diag;
}

uint8_t ap6256_cyw43_port_cardcap_diag(void)
{
    return s_cyw43_cardcap_diag;
}

uint8_t ap6256_cyw43_port_io_ready_diag(void)
{
    return s_cyw43_io_ready_diag;
}

void ap6256_cyw43_port_record_backplane_access(uint8_t is_write,
                                               uint32_t address,
                                               uint8_t width_bytes,
                                               int32_t status)
{
    s_cyw43_backplane_is_write = is_write;
    s_cyw43_backplane_address = address;
    s_cyw43_backplane_width_bytes = width_bytes;
    s_cyw43_backplane_status = status;
}

uint8_t ap6256_cyw43_port_backplane_is_write(void)
{
    return s_cyw43_backplane_is_write;
}

uint32_t ap6256_cyw43_port_backplane_address(void)
{
    return s_cyw43_backplane_address;
}

uint8_t ap6256_cyw43_port_backplane_width_bytes(void)
{
    return s_cyw43_backplane_width_bytes;
}

int32_t ap6256_cyw43_port_backplane_status(void)
{
    return s_cyw43_backplane_status;
}

void ap6256_cyw43_port_set_reference_nvram_enabled(uint8_t enable)
{
    s_cyw43_reference_nvram_enabled = (enable != 0U) ? 1U : 0U;
}

uint8_t ap6256_cyw43_port_reference_nvram_enabled(void)
{
    return s_cyw43_reference_nvram_enabled;
}

void ap6256_cyw43_port_set_last_bus_init_ret(int32_t value)
{
    s_cyw43_last_bus_init_ret = value;
}

int32_t ap6256_cyw43_port_last_bus_init_ret(void)
{
    return s_cyw43_last_bus_init_ret;
}

void ap6256_cyw43_port_set_bus_stage(uint32_t value)
{
    s_cyw43_bus_stage = value;
}

uint32_t ap6256_cyw43_port_bus_stage(void)
{
    return s_cyw43_bus_stage;
}

void ap6256_cyw43_port_set_poll_diag(int32_t header_read_status,
                                     int32_t payload_read_status,
                                     uint16_t hdr0,
                                     uint16_t hdr1)
{
    s_cyw43_poll_header_read_status = header_read_status;
    s_cyw43_poll_payload_read_status = payload_read_status;
    s_cyw43_poll_hdr0 = hdr0;
    s_cyw43_poll_hdr1 = hdr1;
}

int32_t ap6256_cyw43_port_poll_header_read_status(void)
{
    return s_cyw43_poll_header_read_status;
}

int32_t ap6256_cyw43_port_poll_payload_read_status(void)
{
    return s_cyw43_poll_payload_read_status;
}

uint16_t ap6256_cyw43_port_poll_hdr0(void)
{
    return s_cyw43_poll_hdr0;
}

uint16_t ap6256_cyw43_port_poll_hdr1(void)
{
    return s_cyw43_poll_hdr1;
}

uint8_t ap6256_cyw43_port_last_cmd53_write(void)
{
    return s_cyw43_last_cmd53_write;
}

uint8_t ap6256_cyw43_port_last_cmd53_function(void)
{
    return s_cyw43_last_cmd53_function;
}

uint8_t ap6256_cyw43_port_last_cmd53_block_mode(void)
{
    return s_cyw43_last_cmd53_block_mode;
}

uint32_t ap6256_cyw43_port_last_cmd53_block_size(void)
{
    return s_cyw43_last_cmd53_block_size;
}

uint32_t ap6256_cyw43_port_last_cmd53_length(void)
{
    return s_cyw43_last_cmd53_length;
}

int32_t ap6256_cyw43_port_last_cmd53_status(void)
{
    return s_cyw43_last_cmd53_status;
}

uint16_t ap6256_cyw43_port_last_cmd53_frame_size(void)
{
    return s_cyw43_last_cmd53_frame_size;
}

uint32_t ap6256_cyw43_port_last_cmd53_count(void)
{
    return s_cyw43_last_cmd53_count;
}
