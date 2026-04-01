#include "ap6256_driver.h"

#include "test_uart.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define AP6256_WL_ENABLE_PORT GPIOC
#define AP6256_WL_ENABLE_PIN  GPIO_PIN_7
#define AP6256_BT_ENABLE_PORT GPIOD
#define AP6256_BT_ENABLE_PIN  GPIO_PIN_13

#define AP6256_POWER_SETTLE_MS     120U
#define AP6256_SDIO_READY_TIMEOUT  500U
#define AP6256_SDIO_CMD_TIMEOUT    120U
#define AP6256_BT_EVENT_TIMEOUT    350U
#define AP6256_CMD52_RETRIES       8U

typedef enum {
    SDMMC_RESP_NONE = 0U,
    SDMMC_RESP_SHORT = 1U,
    SDMMC_RESP_LONG = 3U
} sdmmc_resp_t;

static ap6256_status_t ap6256_bt_wait_cmd_complete(uint16_t opcode,
                                                   uint8_t *status_out,
                                                   uint8_t *ret_params,
                                                   uint8_t *ret_len,
                                                   uint32_t timeout_ms,
                                                   uint32_t *frames_seen);
static ap6256_status_t ap6256_bt_send_hci_cmd(const uint8_t *cmd, uint16_t len);

static void ap6256_sdmmc1_set_clock(uint16_t div)
{
    uint32_t clkcr = ((uint32_t)div & SDMMC_CLKCR_CLKDIV);
    SDMMC1->CLKCR = clkcr;
}

static ap6256_status_t ap6256_sdmmc1_send_cmd(uint32_t cmd_idx,
                                              uint32_t arg,
                                              sdmmc_resp_t waitresp,
                                              uint8_t ignore_crc_fail,
                                              uint32_t *response)
{
    uint32_t start = HAL_GetTick();
    uint32_t status;
    uint32_t cmd = SDMMC_CMD_CPSMEN | (cmd_idx & 0x3FU);

    if (waitresp == SDMMC_RESP_SHORT) {
        cmd |= SDMMC_CMD_WAITRESP_0;
    } else if (waitresp == SDMMC_RESP_LONG) {
        cmd |= SDMMC_CMD_WAITRESP_0 | SDMMC_CMD_WAITRESP_1;
    }

    SDMMC1->ICR = 0xFFFFFFFFU;
    SDMMC1->ARG = arg;
    SDMMC1->CMD = cmd;

    while (1) {
        status = SDMMC1->STA;

        if (waitresp == SDMMC_RESP_NONE) {
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

        if ((HAL_GetTick() - start) > AP6256_SDIO_CMD_TIMEOUT) {
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

static void ap6256_sdmmc1_prepare(void)
{
    __HAL_RCC_SDMMC1_CLK_ENABLE();
    __HAL_RCC_SDMMC1_FORCE_RESET();
    __HAL_RCC_SDMMC1_RELEASE_RESET();

    SDMMC1->POWER = 0x3U;
    ap6256_sdmmc1_set_clock(250U);
    HAL_Delay(2U);
}

static ap6256_status_t ap6256_sdio_cmd52_read(uint8_t function, uint32_t address, uint8_t *value_out)
{
    uint32_t arg = 0U;
    uint32_t resp = 0U;
    ap6256_status_t st;

    if (value_out == NULL) {
        return AP6256_STATUS_BAD_PARAM;
    }

    arg |= ((uint32_t)function & 0x07U) << 28U;
    arg |= (address & 0x1FFFFU) << 9U;

    /*
     * On this board/AP6256 path, CMD52 can raise CCRCFAIL while still returning
     * a usable R5 response. Keep behavior aligned with previous proven probe logic.
     */
    st = ap6256_sdmmc1_send_cmd(52U, arg, SDMMC_RESP_SHORT, 1U, &resp);
    if (st != AP6256_STATUS_OK) {
        return st;
    }

    /* R5 response lower byte contains read data. */
    *value_out = (uint8_t)(resp & 0xFFU);
    return AP6256_STATUS_OK;
}

static ap6256_status_t ap6256_sdio_cmd52_read_retry(uint8_t function,
                                                     uint32_t address,
                                                     uint8_t *value_out,
                                                     uint8_t retries)
{
    ap6256_status_t st = AP6256_STATUS_TIMEOUT;
    uint8_t i;

    if (retries == 0U) {
        retries = 1U;
    }

    for (i = 0U; i < retries; ++i) {
        st = ap6256_sdio_cmd52_read(function, address, value_out);
        if (st == AP6256_STATUS_OK) {
            return AP6256_STATUS_OK;
        }
        HAL_Delay(2U);
    }

    return st;
}

static ap6256_status_t ap6256_sdio_cmd52_read_retry_count(uint8_t function,
                                                           uint32_t address,
                                                           uint8_t *value_out,
                                                           uint8_t retries,
                                                           uint8_t *attempts_out)
{
    ap6256_status_t st = AP6256_STATUS_TIMEOUT;
    uint8_t i;

    if (attempts_out != NULL) {
        *attempts_out = 0U;
    }

    if (retries == 0U) {
        retries = 1U;
    }

    for (i = 0U; i < retries; ++i) {
        st = ap6256_sdio_cmd52_read(function, address, value_out);
        if (attempts_out != NULL) {
            *attempts_out = (uint8_t)(i + 1U);
        }
        if (st == AP6256_STATUS_OK) {
            return AP6256_STATUS_OK;
        }
        HAL_Delay(2U);
    }

    return st;
}

static void ap6256_bt_prepare_flow_control(void)
{
    GPIO_InitTypeDef gpio;

    memset(&gpio, 0, sizeof(gpio));

    /*
     * PD12 is USART3_RTS net to module UART_CTS_N.
     * Drive active-low during probe so module is clear-to-send HCI events.
     */
    gpio.Pin = GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &gpio);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

    /* PD11 (module UART_RTS_N) sampled as input while flow control is forced. */
    gpio.Pin = GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &gpio);
}

static void ap6256_bt_restore_flow_control(void)
{
    GPIO_InitTypeDef gpio;

    memset(&gpio, 0, sizeof(gpio));
    gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &gpio);
}

static ap6256_status_t ap6256_bt_hci_probe_once(uint8_t wl_on, ap6256_bt_diag_t *diag)
{
    static const uint8_t cmd_reset[] = { 0x01U, 0x03U, 0x0CU, 0x00U };
    static const uint8_t cmd_read_ver[] = { 0x01U, 0x01U, 0x10U, 0x00U };
    uint8_t rp[32];
    uint8_t rp_len = 0U;
    ap6256_status_t st = AP6256_STATUS_OK;

    if (diag == NULL) {
        return AP6256_STATUS_BAD_PARAM;
    }

    memset(diag, 0, sizeof(*diag));

    ap6256_power_down();
    HAL_Delay(20U);
    ap6256_bt_prepare_flow_control();
    ap6256_set_enables((wl_on != 0U) ? 1U : 0U, 1U);
    diag->wl_reg_on = (wl_on != 0U) ? 1U : 0U;
    diag->bt_reg_on = 1U;
    HAL_Delay(AP6256_POWER_SETTLE_MS);

    test_uart_flush_rx();

    st = ap6256_bt_send_hci_cmd(cmd_reset, sizeof(cmd_reset));
    if (st != AP6256_STATUS_OK) {
        goto exit;
    }

    st = ap6256_bt_wait_cmd_complete(0x0C03U,
                                     &diag->reset_status,
                                     rp,
                                     &rp_len,
                                     AP6256_BT_EVENT_TIMEOUT,
                                     &diag->event_frames_seen);
    if (st != AP6256_STATUS_OK) {
        goto exit;
    }
    diag->reset_event_seen = 1U;

    st = ap6256_bt_send_hci_cmd(cmd_read_ver, sizeof(cmd_read_ver));
    if (st != AP6256_STATUS_OK) {
        goto exit;
    }

    st = ap6256_bt_wait_cmd_complete(0x1001U,
                                     &diag->version_status,
                                     rp,
                                     &rp_len,
                                     AP6256_BT_EVENT_TIMEOUT,
                                     &diag->event_frames_seen);
    if (st != AP6256_STATUS_OK) {
        goto exit;
    }
    diag->version_event_seen = 1U;

    if (rp_len >= 9U) {
        diag->hci_version = rp[1];
        diag->hci_revision = (uint16_t)rp[2] | ((uint16_t)rp[3] << 8U);
        diag->lmp_version = rp[4];
        diag->manufacturer = (uint16_t)rp[5] | ((uint16_t)rp[6] << 8U);
        diag->lmp_subversion = (uint16_t)rp[7] | ((uint16_t)rp[8] << 8U);
    }

exit:
    ap6256_power_down();
    ap6256_bt_restore_flow_control();
    return st;
}

static uint8_t ap6256_uart_read_byte(uint8_t *byte_out, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    if (byte_out == NULL) {
        return 0U;
    }

    while ((HAL_GetTick() - start) < timeout_ms) {
        if (HAL_UART_Receive(&huart3, byte_out, 1U, 10U) == HAL_OK) {
            return 1U;
        }
    }

    return 0U;
}

static ap6256_status_t ap6256_bt_wait_cmd_complete(uint16_t opcode,
                                                   uint8_t *status_out,
                                                   uint8_t *ret_params,
                                                   uint8_t *ret_len,
                                                   uint32_t timeout_ms,
                                                   uint32_t *frames_seen)
{
    uint32_t start = HAL_GetTick();
    uint8_t pkt_type;

    if ((status_out == NULL) || (frames_seen == NULL)) {
        return AP6256_STATUS_BAD_PARAM;
    }

    while ((HAL_GetTick() - start) < timeout_ms) {
        uint8_t event_code;
        uint8_t param_len;
        uint8_t payload[64];
        uint8_t i;

        if (ap6256_uart_read_byte(&pkt_type, 15U) == 0U) {
            continue;
        }

        if (pkt_type != 0x04U) {
            continue;
        }

        if ((ap6256_uart_read_byte(&event_code, 20U) == 0U) ||
            (ap6256_uart_read_byte(&param_len, 20U) == 0U)) {
            continue;
        }

        if (param_len > sizeof(payload)) {
            for (i = 0U; i < param_len; ++i) {
                uint8_t throwaway;
                if (ap6256_uart_read_byte(&throwaway, 10U) == 0U) {
                    break;
                }
            }
            continue;
        }

        for (i = 0U; i < param_len; ++i) {
            if (ap6256_uart_read_byte(&payload[i], 20U) == 0U) {
                break;
            }
        }
        if (i != param_len) {
            continue;
        }

        (*frames_seen)++;

        if ((event_code != 0x0EU) || (param_len < 4U)) {
            continue;
        }

        if (((uint16_t)payload[1] | ((uint16_t)payload[2] << 8U)) != opcode) {
            continue;
        }

        *status_out = payload[3];

        if ((ret_params != NULL) && (ret_len != NULL)) {
            uint8_t rp_len = (param_len > 3U) ? (uint8_t)(param_len - 3U) : 0U;
            uint8_t copy = rp_len;
            if (copy > 32U) {
                copy = 32U;
            }
            if (copy > 0U) {
                memcpy(ret_params, &payload[3], copy);
            }
            *ret_len = rp_len;
        }

        return AP6256_STATUS_OK;
    }

    return AP6256_STATUS_TIMEOUT;
}

static ap6256_status_t ap6256_bt_send_hci_cmd(const uint8_t *cmd, uint16_t len)
{
    if ((cmd == NULL) || (len == 0U)) {
        return AP6256_STATUS_BAD_PARAM;
    }

    if (HAL_UART_Transmit(&huart3, (uint8_t *)cmd, len, 200U) != HAL_OK) {
        return AP6256_STATUS_IO_ERROR;
    }

    return AP6256_STATUS_OK;
}

void ap6256_init(void)
{
    ap6256_power_down();
}

void ap6256_set_enables(uint8_t wl_on, uint8_t bt_on)
{
    HAL_GPIO_WritePin(AP6256_WL_ENABLE_PORT, AP6256_WL_ENABLE_PIN, (wl_on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AP6256_BT_ENABLE_PORT, AP6256_BT_ENABLE_PIN, (bt_on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void ap6256_power_down(void)
{
    ap6256_set_enables(0U, 0U);
}

ap6256_status_t ap6256_wifi_transport_probe(ap6256_wifi_diag_t *diag)
{
    uint32_t resp = 0U;
    uint32_t rca_resp = 0U;
    uint8_t cccr = 0xFFU;
    uint8_t sdio_rev = 0xFFU;
    uint8_t io_enable = 0xFFU;
    uint8_t io_ready = 0xFFU;
    uint8_t cis_low = 0xFFU;
    uint8_t cis_mid = 0xFFU;
    uint8_t cis_high = 0xFFU;
    uint32_t start;
    ap6256_status_t st;

    if (diag == NULL) {
        return AP6256_STATUS_BAD_PARAM;
    }

    memset(diag, 0, sizeof(*diag));
    diag->cccr_rev = 0xFFU;
    diag->sdio_rev = 0xFFU;
    diag->io_enable = 0xFFU;
    diag->io_ready = 0xFFU;
    diag->cis_ptr = 0x00FFFFFFUL;

    ap6256_power_down();
    HAL_Delay(20U);
    ap6256_sdmmc1_prepare();

    /* Sanity check while disabled: CMD5 should usually not return ready response. */
    if (ap6256_sdmmc1_send_cmd(5U, 0U, SDMMC_RESP_SHORT, 1U, &resp) == AP6256_STATUS_OK) {
        diag->disabled_cmd5_response = 1U;
    }

    ap6256_set_enables(1U, 0U);
    diag->wl_reg_on = 1U;
    diag->bt_reg_on = 0U;
    HAL_Delay(AP6256_POWER_SETTLE_MS);
    diag->wl_pin_level = (HAL_GPIO_ReadPin(AP6256_WL_ENABLE_PORT, AP6256_WL_ENABLE_PIN) == GPIO_PIN_SET) ? 1U : 0U;
    diag->bt_pin_level = (HAL_GPIO_ReadPin(AP6256_BT_ENABLE_PORT, AP6256_BT_ENABLE_PIN) == GPIO_PIN_SET) ? 1U : 0U;

    if (diag->wl_pin_level == 0U) {
        ap6256_power_down();
        return AP6256_STATUS_IO_ERROR;
    }

    st = ap6256_sdmmc1_send_cmd(0U, 0U, SDMMC_RESP_NONE, 1U, NULL);
    if (st != AP6256_STATUS_OK) {
        ap6256_power_down();
        return st;
    }

    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < AP6256_SDIO_READY_TIMEOUT) {
        st = ap6256_sdmmc1_send_cmd(5U, 0x00300000U, SDMMC_RESP_SHORT, 1U, &resp);
        if (st != AP6256_STATUS_OK) {
            ap6256_power_down();
            return st;
        }

        diag->cmd5_attempts++;

        if ((resp & 0x80000000U) != 0U) {
            diag->cmd5_ready = 1U;
            diag->ocr = resp;
            break;
        }

        HAL_Delay(5U);
    }

    if (diag->cmd5_ready == 0U) {
        ap6256_power_down();
        return AP6256_STATUS_TIMEOUT;
    }

    /*
     * Enter transfer state before CMD52: some SDIO cards require CMD3/CMD7
     * after CMD5 busy/ready negotiation.
     */
    if (ap6256_sdmmc1_send_cmd(3U, 0U, SDMMC_RESP_SHORT, 1U, &rca_resp) == AP6256_STATUS_OK) {
        diag->cmd3_ok = 1U;
        diag->rca = (uint16_t)((rca_resp >> 16U) & 0xFFFFU);
        if (diag->rca != 0U) {
            if (ap6256_sdmmc1_send_cmd(7U,
                                       ((uint32_t)diag->rca << 16U),
                                       SDMMC_RESP_SHORT,
                                       1U,
                                       NULL) == AP6256_STATUS_OK) {
                diag->cmd7_ok = 1U;
            }
        }
    }

    HAL_Delay(5U);

    /* Required for PASS: at least one valid CCCR read over CMD52. */
    if (ap6256_sdio_cmd52_read_retry_count(0U, 0x00U, &cccr, AP6256_CMD52_RETRIES, &diag->cccr_read_attempts) != AP6256_STATUS_OK) {
        ap6256_power_down();
        return AP6256_STATUS_PROTOCOL_ERROR;
    }
    diag->cccr_read_ok = 1U;

    /* Optional diagnostics: do not fail probe if a non-critical CMD52 read misses. */
    (void)ap6256_sdio_cmd52_read_retry(0U, 0x01U, &sdio_rev, 2U);
    (void)ap6256_sdio_cmd52_read_retry(0U, 0x02U, &io_enable, 2U);
    (void)ap6256_sdio_cmd52_read_retry(0U, 0x03U, &io_ready, 2U);
    (void)ap6256_sdio_cmd52_read_retry(0U, 0x09U, &cis_low, 2U);
    (void)ap6256_sdio_cmd52_read_retry(0U, 0x0AU, &cis_mid, 2U);
    (void)ap6256_sdio_cmd52_read_retry(0U, 0x0BU, &cis_high, 2U);

    diag->cccr_rev = cccr;
    diag->sdio_rev = sdio_rev;
    diag->io_enable = io_enable;
    diag->io_ready = io_ready;
    diag->cis_ptr = ((uint32_t)cis_high << 16U) | ((uint32_t)cis_mid << 8U) | cis_low;

    ap6256_power_down();
    return AP6256_STATUS_OK;
}

ap6256_status_t ap6256_bt_hci_probe(ap6256_bt_diag_t *diag)
{
    ap6256_bt_diag_t attempt;
    ap6256_status_t st;

    if (diag == NULL) {
        return AP6256_STATUS_BAD_PARAM;
    }

    memset(diag, 0, sizeof(*diag));

    /* Primary mode: BT only; fallback: BT + WL asserted for combo-module variants. */
    st = ap6256_bt_hci_probe_once(0U, &attempt);
    *diag = attempt;
    if (st == AP6256_STATUS_OK) {
        return st;
    }

    st = ap6256_bt_hci_probe_once(1U, &attempt);
    if (attempt.event_frames_seen >= diag->event_frames_seen) {
        *diag = attempt;
    }

    return st;
}

const char *ap6256_status_to_string(ap6256_status_t status)
{
    switch (status) {
    case AP6256_STATUS_OK:
        return "ok";
    case AP6256_STATUS_TIMEOUT:
        return "timeout";
    case AP6256_STATUS_IO_ERROR:
        return "io_error";
    case AP6256_STATUS_PROTOCOL_ERROR:
        return "protocol_error";
    case AP6256_STATUS_BAD_PARAM:
        return "bad_param";
    default:
        return "unknown";
    }
}
