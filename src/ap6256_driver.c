#include "ap6256_driver.h"

#include "ap6256_assets.h"
#include "test_uart.h"
#include "stm32h7xx_ll_sdmmc.h"

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
#define AP6256_SDIO_FUNC_READY_TIMEOUT 200U
#define AP6256_SDIO_DATA_TIMEOUT_MS    1000U

#define AP6256_SDIO_CCCR_SDIO_REV  0x01U
#define AP6256_SDIO_CCCR_IO_ENABLE 0x02U
#define AP6256_SDIO_CCCR_IO_READY  0x03U

#define AP6256_SDIO_FBR_STRIDE          0x100U
#define AP6256_SDIO_FBR_BLOCK_SIZE_LOW  0x10U
#define AP6256_SDIO_FBR_BLOCK_SIZE_HIGH 0x11U

#define AP6256_BCM_CONTROL_F1_BLOCK_SIZE    64U
#define AP6256_BCM_CONTROL_F2_BLOCK_SIZE    512U
#define AP6256_BCM_CHIPCOMMON_BASE          0x18000000UL
#define AP6256_BCM_CHIP_CLOCK_CSR           0x1000EU
#define AP6256_BCM_CHIP_CLOCK_FORCE_ALP     0x01U
#define AP6256_BCM_CHIP_CLOCK_ALP_AVAIL     0x40U

#define AP6256_BCM_SBADDRLOW        0x1000AU
#define AP6256_BCM_SBADDRMID        0x1000BU
#define AP6256_BCM_SBADDRHIGH       0x1000CU
#define AP6256_BCM_SBWINDOW_MASK    0xFFFF8000UL
#define AP6256_BCM_SB_OFT_ADDR_MASK 0x00007FFFUL
#define AP6256_BCM_BACKPLANE_32BIT_ACCESS 0x08000UL

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
static ap6256_status_t ap6256_sdio_cmd52_write(uint8_t function, uint32_t address, uint8_t value);
static void ap6256_sdio_session_reset(void);
static ap6256_status_t ap6256_bt_send_hci_cmd(const uint8_t *cmd, uint16_t len);
static uint32_t ap6256_sdmmc_block_size_code(uint32_t block_size);
static ap6256_status_t ap6256_sdio_cmd53_transfer(uint8_t write,
                                                  uint8_t function,
                                                  uint32_t address,
                                                  uint8_t *data,
                                                  uint32_t len,
                                                  uint8_t block_mode,
                                                  uint8_t op_code,
                                                  uint32_t explicit_block_size);

static uint8_t s_ap6256_sdio_open = 0U;
static uint16_t s_ap6256_sdio_rca = 0U;
static uint32_t s_ap6256_sb_window = 0xFFFFFFFFUL;
static uint8_t s_ap6256_bt_open = 0U;

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

static ap6256_status_t ap6256_sdio_cmd52_write(uint8_t function, uint32_t address, uint8_t value)
{
    uint32_t arg = 0U;

    arg |= 0x80000000UL;
    arg |= ((uint32_t)function & 0x07U) << 28U;
    arg |= (address & 0x1FFFFU) << 9U;
    arg |= value;

    return ap6256_sdmmc1_send_cmd(52U, arg, SDMMC_RESP_SHORT, 1U, NULL);
}

static uint32_t ap6256_sdmmc_block_size_code(uint32_t block_size)
{
    switch (block_size) {
    case 1U:   return SDMMC_DATABLOCK_SIZE_1B;
    case 2U:   return SDMMC_DATABLOCK_SIZE_2B;
    case 4U:   return SDMMC_DATABLOCK_SIZE_4B;
    case 8U:   return SDMMC_DATABLOCK_SIZE_8B;
    case 16U:  return SDMMC_DATABLOCK_SIZE_16B;
    case 32U:  return SDMMC_DATABLOCK_SIZE_32B;
    case 64U:  return SDMMC_DATABLOCK_SIZE_64B;
    case 128U: return SDMMC_DATABLOCK_SIZE_128B;
    case 256U: return SDMMC_DATABLOCK_SIZE_256B;
    case 512U: return SDMMC_DATABLOCK_SIZE_512B;
    case 1024U: return SDMMC_DATABLOCK_SIZE_1024B;
    case 2048U: return SDMMC_DATABLOCK_SIZE_2048B;
    case 4096U: return SDMMC_DATABLOCK_SIZE_4096B;
    case 8192U: return SDMMC_DATABLOCK_SIZE_8192B;
    case 16384U: return SDMMC_DATABLOCK_SIZE_16384B;
    default:
        return SDMMC_DATABLOCK_SIZE_1B;
    }
}

static uint8_t ap6256_is_power_of_two_u32(uint32_t value)
{
    return (value != 0U) && ((value & (value - 1U)) == 0U);
}

static uint32_t ap6256_round_up_pow2_u32(uint32_t value)
{
    uint32_t rounded = 1U;

    if (value <= 1U) {
        return 1U;
    }

    while ((rounded < value) && (rounded < 512U)) {
        rounded <<= 1U;
    }

    return (rounded > 512U) ? 512U : rounded;
}

static ap6256_status_t ap6256_sdio_cmd53_transfer(uint8_t write,
                                                  uint8_t function,
                                                  uint32_t address,
                                                  uint8_t *data,
                                                  uint32_t len,
                                                  uint8_t block_mode,
                                                  uint8_t op_code,
                                                  uint32_t explicit_block_size)
{
    SDMMC_DataInitTypeDef data_cfg;
    uint32_t arg = 0U;
    uint32_t count = 0U;
    uint32_t block_size = 1U;
    uint32_t host_block_size = 1U;
    uint32_t start = HAL_GetTick();
    uint32_t offset = 0U;
    uint32_t dataremaining = 0U;

    if ((data == NULL) || (len == 0U) || (s_ap6256_sdio_open == 0U)) {
        return AP6256_STATUS_BAD_PARAM;
    }

    if (block_mode != 0U) {
        if (explicit_block_size != 0U) {
            block_size = explicit_block_size;
        } else if (function == 1U) {
            block_size = AP6256_BCM_CONTROL_F1_BLOCK_SIZE;
        } else if (function == 2U) {
            block_size = AP6256_BCM_CONTROL_F2_BLOCK_SIZE;
        } else {
            block_size = 64U;
        }

        if ((len % block_size) != 0U) {
            return AP6256_STATUS_BAD_PARAM;
        }
        count = len / block_size;
        host_block_size = block_size;
    } else {
        count = (len == 512U) ? 0U : len;
        if (explicit_block_size != 0U) {
            block_size = explicit_block_size;
        } else {
            block_size = len;
        }
        /*
         * In SDIO byte mode the command still carries the requested byte count,
         * but the STM32 SDMMC data path must be configured as 1-byte blocks.
         * ST's HAL SDIO implementation does the same for CMD53 byte-mode I/O.
         */
        host_block_size = 1U;
    }

    memset(&data_cfg, 0, sizeof(data_cfg));
    data_cfg.DataTimeOut = SDMMC_DATATIMEOUT;
    data_cfg.DataLength = len;
    data_cfg.DataBlockSize = ap6256_sdmmc_block_size_code(host_block_size);
    data_cfg.TransferDir = (write != 0U) ? SDMMC_TRANSFER_DIR_TO_CARD : SDMMC_TRANSFER_DIR_TO_SDMMC;
    data_cfg.TransferMode = (block_mode != 0U) ? SDMMC_TRANSFER_MODE_BLOCK : SDMMC_TRANSFER_MODE_SDIO;
    data_cfg.DPSM = SDMMC_DPSM_DISABLE;

    /*
     * CMD53 transfers are SDIO I/O operations, so keep SDIOEN asserted on the
     * data path. ST's HAL preserves this bit across SDIO transfers.
     */
    SDMMC1->DCTRL = SDMMC_DCTRL_SDIOEN;

    SDMMC1->ICR = 0xFFFFFFFFU;
    if (SDMMC_ConfigData(SDMMC1, &data_cfg) != HAL_OK) {
        return AP6256_STATUS_IO_ERROR;
    }
    __SDMMC_CMDTRANS_ENABLE(SDMMC1);

    arg |= ((uint32_t)(write != 0U) << 31U);
    arg |= ((uint32_t)function & 0x07U) << 28U;
    arg |= ((uint32_t)(block_mode != 0U) << 27U);
    arg |= ((uint32_t)(op_code != 0U) << 26U);
    arg |= (address & 0x1FFFFUL) << 9U;
    arg |= (count & 0x1FFU);

    if (SDMMC_SDIO_CmdReadWriteExtended(SDMMC1, arg) != SDMMC_ERROR_NONE) {
        __SDMMC_CMDTRANS_DISABLE(SDMMC1);
        SDMMC1->ICR = 0xFFFFFFFFU;
        return AP6256_STATUS_PROTOCOL_ERROR;
    }

    dataremaining = len;
    while ((HAL_GetTick() - start) < AP6256_SDIO_DATA_TIMEOUT_MS) {
        uint32_t sta = SDMMC1->STA;

        if ((sta & (SDMMC_STA_DTIMEOUT | SDMMC_STA_DCRCFAIL | SDMMC_STA_RXOVERR | SDMMC_STA_TXUNDERR)) != 0U) {
            __SDMMC_CMDTRANS_DISABLE(SDMMC1);
            SDMMC1->ICR = 0xFFFFFFFFU;
            return AP6256_STATUS_IO_ERROR;
        }

        if (write == 0U) {
            if (((sta & SDMMC_STA_RXFIFOHF) != 0U) && (dataremaining >= 32U)) {
                uint32_t reg_count;

                for (reg_count = 0U; reg_count < 8U; ++reg_count) {
                    uint32_t word = SDMMC_ReadFIFO(SDMMC1);
                    data[offset++] = (uint8_t)(word & 0xFFU);
                    data[offset++] = (uint8_t)((word >> 8U) & 0xFFU);
                    data[offset++] = (uint8_t)((word >> 16U) & 0xFFU);
                    data[offset++] = (uint8_t)((word >> 24U) & 0xFFU);
                }
                dataremaining -= 32U;
            } else if (dataremaining < 32U) {
                while ((dataremaining > 0U) && ((SDMMC1->STA & SDMMC_STA_RXFIFOE) == 0U)) {
                    uint32_t word = SDMMC_ReadFIFO(SDMMC1);
                    uint32_t byte_count;

                    for (byte_count = 0U; byte_count < 4U; ++byte_count) {
                        if (dataremaining > 0U) {
                            data[offset++] = (uint8_t)((word >> (byte_count * 8U)) & 0xFFU);
                            dataremaining--;
                        }
                    }
                }
            }
        } else {
            if (((sta & SDMMC_STA_TXFIFOHE) != 0U) && (dataremaining >= 32U)) {
                uint32_t reg_count;

                for (reg_count = 0U; reg_count < 8U; ++reg_count) {
                    uint32_t word = ((uint32_t)data[offset + 0U]) |
                                    ((uint32_t)data[offset + 1U] << 8U) |
                                    ((uint32_t)data[offset + 2U] << 16U) |
                                    ((uint32_t)data[offset + 3U] << 24U);
                    SDMMC1->FIFO = word;
                    offset += 4U;
                }
                dataremaining -= 32U;
            } else if ((dataremaining < 32U) &&
                       ((SDMMC1->STA & (SDMMC_STA_TXFIFOHE | SDMMC_STA_TXFIFOE)) != 0U)) {
                while (dataremaining > 0U) {
                    uint32_t word = 0U;
                    uint32_t byte_count;

                    for (byte_count = 0U; (byte_count < 4U) && (dataremaining > 0U); ++byte_count) {
                        word |= ((uint32_t)data[offset++]) << (byte_count << 3U);
                        dataremaining--;
                    }
                    SDMMC1->FIFO = word;
                }
            }
        }

        if ((sta & SDMMC_STA_DATAEND) != 0U) {
            __SDMMC_CMDTRANS_DISABLE(SDMMC1);
            SDMMC1->ICR = 0xFFFFFFFFU;
            return AP6256_STATUS_OK;
        }
    }

    __SDMMC_CMDTRANS_DISABLE(SDMMC1);
    SDMMC1->ICR = 0xFFFFFFFFU;
    return AP6256_STATUS_TIMEOUT;
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

static void ap6256_sdio_session_reset(void)
{
    s_ap6256_sdio_open = 0U;
    s_ap6256_sdio_rca = 0U;
    s_ap6256_sb_window = 0xFFFFFFFFUL;
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
    if (byte_out == NULL) {
        return 0U;
    }

    return (ap6256_bt_uart_read(byte_out, 1U, timeout_ms) == AP6256_STATUS_OK) ? 1U : 0U;
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
    return ap6256_bt_uart_write(cmd, len);
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
    ap6256_sdio_session_reset();
    s_ap6256_bt_open = 0U;
}

ap6256_status_t ap6256_sdio_bus_prepare(uint8_t wl_on, uint8_t bt_on)
{
    ap6256_power_down();
    HAL_Delay(20U);
    ap6256_sdmmc1_prepare();
    ap6256_set_enables((wl_on != 0U) ? 1U : 0U, (bt_on != 0U) ? 1U : 0U);
    HAL_Delay(AP6256_POWER_SETTLE_MS);

    if ((wl_on != 0U) &&
        (HAL_GPIO_ReadPin(AP6256_WL_ENABLE_PORT, AP6256_WL_ENABLE_PIN) != GPIO_PIN_SET)) {
        ap6256_power_down();
        return AP6256_STATUS_IO_ERROR;
    }

    return AP6256_STATUS_OK;
}

void ap6256_sdio_session_adopt(uint16_t rca)
{
    if (rca == 0U) {
        s_ap6256_sdio_open = 0U;
        s_ap6256_sdio_rca = 0U;
        s_ap6256_sb_window = 0xFFFFFFFFUL;
        return;
    }

    s_ap6256_sdio_open = 1U;
    s_ap6256_sdio_rca = rca;
    s_ap6256_sb_window = 0xFFFFFFFFUL;
}

ap6256_status_t ap6256_sdio_open(ap6256_sdio_session_t *session, uint8_t wl_on, uint8_t bt_on)
{
    uint32_t resp = 0U;
    uint32_t rca_resp = 0U;
    uint32_t start;
    ap6256_status_t st;

    if ((session == NULL) || (wl_on == 0U)) {
        return AP6256_STATUS_BAD_PARAM;
    }

    memset(session, 0, sizeof(*session));

    st = ap6256_sdio_bus_prepare(1U, (bt_on != 0U) ? 1U : 0U);
    if (st != AP6256_STATUS_OK) {
        return st;
    }

    session->wl_reg_on = 1U;
    session->bt_reg_on = (bt_on != 0U) ? 1U : 0U;

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

        if ((resp & 0x80000000UL) != 0U) {
            session->cmd5_ready = 1U;
            session->ocr = resp;
            break;
        }

        HAL_Delay(5U);
    }

    if (session->cmd5_ready == 0U) {
        ap6256_power_down();
        return AP6256_STATUS_TIMEOUT;
    }

    st = ap6256_sdmmc1_send_cmd(3U, 0U, SDMMC_RESP_SHORT, 1U, &rca_resp);
    if (st != AP6256_STATUS_OK) {
        ap6256_power_down();
        return st;
    }

    session->rca = (uint16_t)((rca_resp >> 16U) & 0xFFFFU);
    if (session->rca == 0U) {
        ap6256_power_down();
        return AP6256_STATUS_PROTOCOL_ERROR;
    }

    st = ap6256_sdmmc1_send_cmd(7U,
                                ((uint32_t)session->rca << 16U),
                                SDMMC_RESP_SHORT,
                                1U,
                                NULL);
    if (st != AP6256_STATUS_OK) {
        ap6256_power_down();
        return st;
    }

    session->selected = 1U;
    ap6256_sdio_session_adopt(session->rca);
    return AP6256_STATUS_OK;
}

void ap6256_sdio_close(void)
{
    ap6256_power_down();
}

ap6256_status_t ap6256_sdio_cmd52_read_u8(uint8_t function, uint32_t address, uint8_t *value_out)
{
    return ap6256_sdio_cmd52_read(function, address, value_out);
}

ap6256_status_t ap6256_sdio_cmd52_write_u8(uint8_t function, uint32_t address, uint8_t value)
{
    return ap6256_sdio_cmd52_write(function, address, value);
}

ap6256_status_t ap6256_sdio_cmd53_read(uint8_t function,
                                       uint32_t address,
                                       uint8_t *data,
                                       uint32_t len,
                                       uint8_t block_mode,
                                       uint8_t op_code)
{
    return ap6256_sdio_cmd53_read_ex(function, address, data, len, block_mode, op_code, 0U);
}

ap6256_status_t ap6256_sdio_cmd53_read_ex(uint8_t function,
                                          uint32_t address,
                                          uint8_t *data,
                                          uint32_t len,
                                          uint8_t block_mode,
                                          uint8_t op_code,
                                          uint32_t block_size)
{
    return ap6256_sdio_cmd53_transfer(0U, function, address, data, len, block_mode, op_code, block_size);
}

ap6256_status_t ap6256_sdio_cmd53_write(uint8_t function,
                                        uint32_t address,
                                        const uint8_t *data,
                                        uint32_t len,
                                        uint8_t block_mode,
                                        uint8_t op_code)
{
    return ap6256_sdio_cmd53_write_ex(function, address, data, len, block_mode, op_code, 0U);
}

ap6256_status_t ap6256_sdio_cmd53_write_ex(uint8_t function,
                                           uint32_t address,
                                           const uint8_t *data,
                                           uint32_t len,
                                           uint8_t block_mode,
                                           uint8_t op_code,
                                           uint32_t block_size)
{
    return ap6256_sdio_cmd53_transfer(1U, function, address, (uint8_t *)data, len, block_mode, op_code, block_size);
}

ap6256_status_t ap6256_sdio_enable_function(uint8_t function)
{
    uint8_t io_enable = 0U;
    uint8_t io_ready = 0U;
    uint8_t bit;
    uint32_t start;
    ap6256_status_t st;

    if ((function == 0U) || (function > 7U)) {
        return AP6256_STATUS_BAD_PARAM;
    }

    bit = (uint8_t)(1U << function);

    st = ap6256_sdio_cmd52_read_u8(0U, AP6256_SDIO_CCCR_IO_ENABLE, &io_enable);
    if (st != AP6256_STATUS_OK) {
        return st;
    }

    io_enable |= bit;
    st = ap6256_sdio_cmd52_write_u8(0U, AP6256_SDIO_CCCR_IO_ENABLE, io_enable);
    if (st != AP6256_STATUS_OK) {
        return st;
    }

    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < AP6256_SDIO_FUNC_READY_TIMEOUT) {
        st = ap6256_sdio_cmd52_read_u8(0U, AP6256_SDIO_CCCR_IO_READY, &io_ready);
        if ((st == AP6256_STATUS_OK) && ((io_ready & bit) != 0U)) {
            return AP6256_STATUS_OK;
        }
        HAL_Delay(1U);
    }

    return AP6256_STATUS_TIMEOUT;
}

ap6256_status_t ap6256_sdio_set_block_size(uint8_t function, uint16_t block_size)
{
    uint32_t base;
    ap6256_status_t st;

    if (block_size == 0U) {
        return AP6256_STATUS_BAD_PARAM;
    }

    base = (function == 0U) ? AP6256_SDIO_FBR_BLOCK_SIZE_LOW
                            : (((uint32_t)function * AP6256_SDIO_FBR_STRIDE) + AP6256_SDIO_FBR_BLOCK_SIZE_LOW);

    st = ap6256_sdio_cmd52_write_u8(0U, base, (uint8_t)(block_size & 0xFFU));
    if (st != AP6256_STATUS_OK) {
        return st;
    }

    return ap6256_sdio_cmd52_write_u8(0U, base + 1U, (uint8_t)((block_size >> 8U) & 0xFFU));
}

ap6256_status_t ap6256_bcm_prepare_control_bus(uint8_t *chip_clock_csr_out)
{
    uint8_t chip_clock_csr = 0U;
    uint32_t start;
    ap6256_status_t st;

    st = ap6256_sdio_enable_function(1U);
    if (st != AP6256_STATUS_OK) {
        return st;
    }

    st = ap6256_sdio_set_block_size(1U, AP6256_BCM_CONTROL_F1_BLOCK_SIZE);
    if (st != AP6256_STATUS_OK) {
        return st;
    }

    st = ap6256_sdio_enable_function(2U);
    if (st != AP6256_STATUS_OK) {
        return st;
    }

    st = ap6256_sdio_set_block_size(2U, AP6256_BCM_CONTROL_F2_BLOCK_SIZE);
    if (st != AP6256_STATUS_OK) {
        return st;
    }

    st = ap6256_sdio_cmd52_read_u8(1U, AP6256_BCM_CHIP_CLOCK_CSR, &chip_clock_csr);
    if (st != AP6256_STATUS_OK) {
        return st;
    }

    chip_clock_csr |= AP6256_BCM_CHIP_CLOCK_FORCE_ALP;
    st = ap6256_sdio_cmd52_write_u8(1U, AP6256_BCM_CHIP_CLOCK_CSR, chip_clock_csr);
    if (st != AP6256_STATUS_OK) {
        return st;
    }

    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < AP6256_SDIO_FUNC_READY_TIMEOUT) {
        st = ap6256_sdio_cmd52_read_u8(1U, AP6256_BCM_CHIP_CLOCK_CSR, &chip_clock_csr);
        if ((st == AP6256_STATUS_OK) &&
            ((chip_clock_csr & AP6256_BCM_CHIP_CLOCK_ALP_AVAIL) != 0U)) {
            if (chip_clock_csr_out != NULL) {
                *chip_clock_csr_out = chip_clock_csr;
            }
            return AP6256_STATUS_OK;
        }
        HAL_Delay(1U);
    }

    if (chip_clock_csr_out != NULL) {
        *chip_clock_csr_out = chip_clock_csr;
    }

    return AP6256_STATUS_TIMEOUT;
}

ap6256_status_t ap6256_bcm_set_backplane_window(uint32_t address)
{
    uint32_t bar0;
    uint32_t value;
    uint8_t i;
    ap6256_status_t st;

    bar0 = address & AP6256_BCM_SBWINDOW_MASK;
    if (bar0 == s_ap6256_sb_window) {
        return AP6256_STATUS_OK;
    }

    value = bar0 >> 8U;
    for (i = 0U; i < 3U; ++i, value >>= 8U) {
        st = ap6256_sdio_cmd52_write_u8(1U, AP6256_BCM_SBADDRLOW + i, (uint8_t)(value & 0xFFU));
        if (st != AP6256_STATUS_OK) {
            return st;
        }
    }

    s_ap6256_sb_window = bar0;
    return AP6256_STATUS_OK;
}

ap6256_status_t ap6256_bcm_backplane_read32(uint32_t address, uint32_t *value_out)
{
    uint32_t local;
    uint8_t b0;
    uint8_t b1;
    uint8_t b2;
    uint8_t b3;
    ap6256_status_t st;

    if (value_out == NULL) {
        return AP6256_STATUS_BAD_PARAM;
    }

    st = ap6256_bcm_set_backplane_window(address);
    if (st != AP6256_STATUS_OK) {
        return st;
    }

    /* Use byte-wise F1 accesses here as a low-bandwidth control-path primitive. */
    local = address & AP6256_BCM_SB_OFT_ADDR_MASK;
    st = ap6256_sdio_cmd52_read_u8(1U, local + 0U, &b0);
    if (st != AP6256_STATUS_OK) {
        return st;
    }
    st = ap6256_sdio_cmd52_read_u8(1U, local + 1U, &b1);
    if (st != AP6256_STATUS_OK) {
        return st;
    }
    st = ap6256_sdio_cmd52_read_u8(1U, local + 2U, &b2);
    if (st != AP6256_STATUS_OK) {
        return st;
    }
    st = ap6256_sdio_cmd52_read_u8(1U, local + 3U, &b3);
    if (st != AP6256_STATUS_OK) {
        return st;
    }

    *value_out = (uint32_t)b0 |
                 ((uint32_t)b1 << 8U) |
                 ((uint32_t)b2 << 16U) |
                 ((uint32_t)b3 << 24U);
    return AP6256_STATUS_OK;
}

ap6256_status_t ap6256_bcm_backplane_write32(uint32_t address, uint32_t value)
{
    uint32_t local;
    ap6256_status_t st;

    st = ap6256_bcm_set_backplane_window(address);
    if (st != AP6256_STATUS_OK) {
        return st;
    }

    /* Use byte-wise F1 accesses here as a low-bandwidth control-path primitive. */
    local = address & AP6256_BCM_SB_OFT_ADDR_MASK;
    st = ap6256_sdio_cmd52_write_u8(1U, local + 0U, (uint8_t)(value & 0xFFU));
    if (st != AP6256_STATUS_OK) {
        return st;
    }
    st = ap6256_sdio_cmd52_write_u8(1U, local + 1U, (uint8_t)((value >> 8U) & 0xFFU));
    if (st != AP6256_STATUS_OK) {
        return st;
    }
    st = ap6256_sdio_cmd52_write_u8(1U, local + 2U, (uint8_t)((value >> 16U) & 0xFFU));
    if (st != AP6256_STATUS_OK) {
        return st;
    }

    return ap6256_sdio_cmd52_write_u8(1U, local + 3U, (uint8_t)((value >> 24U) & 0xFFU));
}

ap6256_status_t ap6256_bcm_backplane_read(uint32_t address, uint8_t *data, uint32_t len)
{
    ap6256_status_t st = AP6256_STATUS_OK;

    if ((data == NULL) || (len == 0U)) {
        return AP6256_STATUS_BAD_PARAM;
    }

    while (len > 0U) {
        uint32_t window_remaining = (AP6256_BCM_SB_OFT_ADDR_MASK + 1U) - (address & AP6256_BCM_SB_OFT_ADDR_MASK);
        uint32_t chunk = (len < window_remaining) ? len : window_remaining;
        uint32_t local = address & AP6256_BCM_SB_OFT_ADDR_MASK;
        uint8_t block_mode = 0U;

        st = ap6256_bcm_set_backplane_window(address);
        if (st != AP6256_STATUS_OK) {
            return st;
        }

        if ((chunk >= AP6256_BCM_CONTROL_F1_BLOCK_SIZE) &&
            ((chunk % AP6256_BCM_CONTROL_F1_BLOCK_SIZE) == 0U)) {
            block_mode = 1U;
        }

        st = ap6256_sdio_cmd53_read(1U, local, data, chunk, block_mode, 1U);
        if (st != AP6256_STATUS_OK) {
            return st;
        }

        address += chunk;
        data += chunk;
        len -= chunk;
    }

    return AP6256_STATUS_OK;
}

ap6256_status_t ap6256_bcm_backplane_write(uint32_t address, const uint8_t *data, uint32_t len)
{
    ap6256_status_t st = AP6256_STATUS_OK;

    if ((data == NULL) || (len == 0U)) {
        return AP6256_STATUS_BAD_PARAM;
    }

    while (len > 0U) {
        uint32_t window_remaining = (AP6256_BCM_SB_OFT_ADDR_MASK + 1U) - (address & AP6256_BCM_SB_OFT_ADDR_MASK);
        uint32_t chunk = (len < window_remaining) ? len : window_remaining;
        uint32_t local = address & AP6256_BCM_SB_OFT_ADDR_MASK;
        uint8_t block_mode = 0U;

        st = ap6256_bcm_set_backplane_window(address);
        if (st != AP6256_STATUS_OK) {
            return st;
        }

        if ((chunk >= AP6256_BCM_CONTROL_F1_BLOCK_SIZE) &&
            ((chunk % AP6256_BCM_CONTROL_F1_BLOCK_SIZE) == 0U)) {
            block_mode = 1U;
        }

        st = ap6256_sdio_cmd53_write(1U, local, data, chunk, block_mode, 1U);
        if (st != AP6256_STATUS_OK) {
            return st;
        }

        address += chunk;
        data += chunk;
        len -= chunk;
    }

    return AP6256_STATUS_OK;
}

uint16_t ap6256_bcm_chip_id_from_raw(uint32_t chip_id_raw)
{
    return (uint16_t)(chip_id_raw & 0xFFFFU);
}

ap6256_status_t ap6256_bt_uart_write(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U)) {
        return AP6256_STATUS_BAD_PARAM;
    }

    if (HAL_UART_Transmit(&huart3, (uint8_t *)data, len, 200U) != HAL_OK) {
        return AP6256_STATUS_IO_ERROR;
    }

    return AP6256_STATUS_OK;
}

ap6256_status_t ap6256_bt_uart_read(uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint16_t offset = 0U;

    if ((data == NULL) || (len == 0U)) {
        return AP6256_STATUS_BAD_PARAM;
    }

    while (offset < len) {
        uint32_t elapsed = HAL_GetTick() - start;
        uint32_t remaining = (elapsed >= timeout_ms) ? 0U : (timeout_ms - elapsed);

        if (remaining == 0U) {
            return AP6256_STATUS_TIMEOUT;
        }

        if (HAL_UART_Receive(&huart3, &data[offset], 1U, (remaining > 10U) ? 10U : remaining) == HAL_OK) {
            offset++;
        }
    }

    return AP6256_STATUS_OK;
}

ap6256_status_t ap6256_bt_open(uint8_t wl_on)
{
    ap6256_power_down();
    HAL_Delay(20U);
    ap6256_bt_prepare_flow_control();
    ap6256_set_enables((wl_on != 0U) ? 1U : 0U, 1U);
    HAL_Delay(AP6256_POWER_SETTLE_MS);
    test_uart_flush_rx();
    s_ap6256_bt_open = 1U;
    return AP6256_STATUS_OK;
}

void ap6256_bt_close(void)
{
    ap6256_power_down();
    ap6256_bt_restore_flow_control();
}

ap6256_status_t ap6256_bt_hci_command(uint16_t opcode,
                                      const uint8_t *params,
                                      uint8_t params_len,
                                      uint8_t *ret_params,
                                      uint8_t *ret_len)
{
    uint8_t buffer[260];
    uint8_t status = 0xFFU;
    uint32_t frames_seen = 0U;
    ap6256_status_t st;

    if (params_len > (sizeof(buffer) - 4U)) {
        return AP6256_STATUS_BAD_PARAM;
    }

    buffer[0] = 0x01U;
    buffer[1] = (uint8_t)(opcode & 0xFFU);
    buffer[2] = (uint8_t)((opcode >> 8U) & 0xFFU);
    buffer[3] = params_len;
    if ((params != NULL) && (params_len > 0U)) {
        memcpy(&buffer[4], params, params_len);
    }

    st = ap6256_bt_send_hci_cmd(buffer, (uint16_t)(params_len + 4U));
    if (st != AP6256_STATUS_OK) {
        return st;
    }

    st = ap6256_bt_wait_cmd_complete(opcode,
                                     &status,
                                     ret_params,
                                     ret_len,
                                     AP6256_BT_EVENT_TIMEOUT,
                                     &frames_seen);
    if (st != AP6256_STATUS_OK) {
        return st;
    }

    return (status == 0x00U) ? AP6256_STATUS_OK : AP6256_STATUS_PROTOCOL_ERROR;
}

ap6256_status_t ap6256_bt_patchram_download(void)
{
    const ap6256_embedded_asset_t *asset = ap6256_assets_bt_patchram();
    uint32_t offset = 0U;
    uint8_t ret_params[32];
    uint8_t ret_len = 0U;
    ap6256_status_t st;

    if ((asset == NULL) || (asset->data == NULL) || (asset->size == 0U)) {
        return AP6256_STATUS_BAD_PARAM;
    }

    st = ap6256_bt_hci_command(0xFC2EU, NULL, 0U, ret_params, &ret_len);
    if (st != AP6256_STATUS_OK) {
        return st;
    }

    while (offset < asset->size) {
        uint16_t opcode;
        uint8_t params_len;

        if ((offset + 3U) > asset->size) {
            return AP6256_STATUS_PROTOCOL_ERROR;
        }

        opcode = (uint16_t)asset->data[offset] | ((uint16_t)asset->data[offset + 1U] << 8U);
        params_len = asset->data[offset + 2U];
        offset += 3U;

        if ((offset + params_len) > asset->size) {
            return AP6256_STATUS_PROTOCOL_ERROR;
        }

        st = ap6256_bt_hci_command(opcode,
                                   &asset->data[offset],
                                   params_len,
                                   ret_params,
                                   &ret_len);
        if (st != AP6256_STATUS_OK) {
            return st;
        }

        offset += params_len;
    }

    return AP6256_STATUS_OK;
}

ap6256_status_t ap6256_wifi_transport_probe(ap6256_wifi_diag_t *diag)
{
    ap6256_sdio_session_t session;
    uint8_t cccr = 0xFFU;
    uint8_t sdio_rev = 0xFFU;
    uint8_t io_enable = 0xFFU;
    uint8_t io_ready = 0xFFU;
    uint8_t cis_low = 0xFFU;
    uint8_t cis_mid = 0xFFU;
    uint8_t cis_high = 0xFFU;
    uint8_t chip_clock_csr = 0x00U;
    uint32_t start;
    uint32_t chip_id_raw = 0U;
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
    diag->function1_block_size = AP6256_BCM_CONTROL_F1_BLOCK_SIZE;
    diag->function2_block_size = AP6256_BCM_CONTROL_F2_BLOCK_SIZE;

    ap6256_power_down();
    HAL_Delay(20U);
    ap6256_sdmmc1_prepare();

    /* Sanity check while disabled: CMD5 should usually not return ready response. */
    if (ap6256_sdmmc1_send_cmd(5U, 0U, SDMMC_RESP_SHORT, 1U, &start) == AP6256_STATUS_OK) {
        diag->disabled_cmd5_response = 1U;
    }

    st = ap6256_sdio_open(&session, 1U, 0U);
    if (st != AP6256_STATUS_OK) {
        ap6256_sdio_close();
        return st;
    }

    diag->wl_reg_on = session.wl_reg_on;
    diag->bt_reg_on = session.bt_reg_on;
    diag->wl_pin_level = (HAL_GPIO_ReadPin(AP6256_WL_ENABLE_PORT, AP6256_WL_ENABLE_PIN) == GPIO_PIN_SET) ? 1U : 0U;
    diag->bt_pin_level = (HAL_GPIO_ReadPin(AP6256_BT_ENABLE_PORT, AP6256_BT_ENABLE_PIN) == GPIO_PIN_SET) ? 1U : 0U;
    diag->cmd5_ready = session.cmd5_ready;
    diag->cmd5_attempts = 1U;
    diag->ocr = session.ocr;
    diag->cmd3_ok = 1U;
    diag->cmd7_ok = (session.selected != 0U) ? 1U : 0U;
    diag->rca = session.rca;

    HAL_Delay(5U);

    /* Required for PASS: at least one valid CCCR read over CMD52. */
    if (ap6256_sdio_cmd52_read_retry_count(0U, 0x00U, &cccr, AP6256_CMD52_RETRIES, &diag->cccr_read_attempts) != AP6256_STATUS_OK) {
        ap6256_sdio_close();
        return AP6256_STATUS_PROTOCOL_ERROR;
    }
    diag->cccr_read_ok = 1U;

    /* Optional diagnostics: do not fail probe if a non-critical CMD52 read misses. */
    (void)ap6256_sdio_cmd52_read_retry(0U, AP6256_SDIO_CCCR_SDIO_REV, &sdio_rev, 2U);
    (void)ap6256_sdio_cmd52_read_retry(0U, AP6256_SDIO_CCCR_IO_ENABLE, &io_enable, 2U);
    (void)ap6256_sdio_cmd52_read_retry(0U, AP6256_SDIO_CCCR_IO_READY, &io_ready, 2U);
    (void)ap6256_sdio_cmd52_read_retry(0U, 0x09U, &cis_low, 2U);
    (void)ap6256_sdio_cmd52_read_retry(0U, 0x0AU, &cis_mid, 2U);
    (void)ap6256_sdio_cmd52_read_retry(0U, 0x0BU, &cis_high, 2U);

    diag->cccr_rev = cccr;
    diag->sdio_rev = sdio_rev;
    diag->io_enable = io_enable;
    diag->io_ready = io_ready;
    diag->cis_ptr = ((uint32_t)cis_high << 16U) | ((uint32_t)cis_mid << 8U) | cis_low;

    if (ap6256_bcm_prepare_control_bus(&chip_clock_csr) == AP6256_STATUS_OK) {
        diag->function1_ready = 1U;
        diag->function2_ready = 1U;
        diag->chip_clock_csr = chip_clock_csr;
        if (ap6256_bcm_backplane_read32(AP6256_BCM_CHIPCOMMON_BASE, &chip_id_raw) == AP6256_STATUS_OK) {
            diag->chip_id_raw = chip_id_raw;
        }
    }

    ap6256_sdio_close();
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
