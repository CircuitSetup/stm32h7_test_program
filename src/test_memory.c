#include "board_test.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SRAM_TEST_WORDS 512U
#define FLASH_CRC_LENGTH_BYTES 4096U

static uint32_t s_sram_test_buffer[SRAM_TEST_WORDS];

static uint32_t crc32_update(uint32_t crc, uint8_t data)
{
    uint32_t i;

    crc ^= data;
    for (i = 0U; i < 8U; ++i) {
        if ((crc & 1U) != 0U) {
            crc = (crc >> 1U) ^ 0xEDB88320U;
        } else {
            crc >>= 1U;
        }
    }
    return crc;
}

static uint32_t crc32_compute(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    size_t i;

    for (i = 0U; i < len; ++i) {
        crc = crc32_update(crc, data[i]);
    }

    return ~crc;
}

void test_memory_sram(board_test_result_t *result)
{
    uint32_t i;

    for (i = 0U; i < SRAM_TEST_WORDS; ++i) {
        s_sram_test_buffer[i] = 0xA5A5A5A5U;
    }

    for (i = 0U; i < SRAM_TEST_WORDS; ++i) {
        if (s_sram_test_buffer[i] != 0xA5A5A5A5U) {
            board_test_set_result(result,
                                  "core.sram_march",
                                  "SRAM",
                                  TEST_STATUS_FAIL,
                                  TEST_MODE_AUTOMATIC,
                                  "pattern1 mismatch",
                                  "SRAM march phase 1 failed.",
                                  "Check MCU core rail and memory interface.");
            return;
        }
        s_sram_test_buffer[i] = 0x5A5A5A5AU;
    }

    for (i = 0U; i < SRAM_TEST_WORDS; ++i) {
        if (s_sram_test_buffer[i] != 0x5A5A5A5AU) {
            board_test_set_result(result,
                                  "core.sram_march",
                                  "SRAM",
                                  TEST_STATUS_FAIL,
                                  TEST_MODE_AUTOMATIC,
                                  "pattern2 mismatch",
                                  "SRAM march phase 2 failed.",
                                  "Check MCU core rail and memory interface.");
            return;
        }
        s_sram_test_buffer[i] = 0x00000000U;
    }

    board_test_set_result(result,
                          "core.sram_march",
                          "SRAM",
                          TEST_STATUS_PASS,
                          TEST_MODE_AUTOMATIC,
                          "512 words pattern sweep",
                          "March-style write/read sanity completed on dedicated buffer.",
                          "");
}

void test_memory_flash_crc(board_test_result_t *result)
{
    const uint8_t *flash_ptr = (const uint8_t *)0x08000000U;
    uint32_t crc = crc32_compute(flash_ptr, FLASH_CRC_LENGTH_BYTES);
    char measured[64];

    (void)snprintf(measured, sizeof(measured), "crc32[0x08000000..+4k]=0x%08lX", (unsigned long)crc);

    if ((crc == 0x00000000U) || (crc == 0xFFFFFFFFU)) {
        board_test_set_result(result,
                              "core.flash_crc",
                              "Internal Flash",
                              TEST_STATUS_WARN,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "CRC value is atypical; verify programmed image.",
                              "Reflash test firmware and rerun.");
    } else {
        board_test_set_result(result,
                              "core.flash_crc",
                              "Internal Flash",
                              TEST_STATUS_PASS,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "Flash read and CRC sanity completed.",
                              "");
    }
}

static void sdmmc_set_clock(SDMMC_TypeDef *inst, uint16_t div)
{
    uint32_t clkcr = 0U;

    clkcr |= ((uint32_t)div & SDMMC_CLKCR_CLKDIV);
    inst->CLKCR = clkcr;
}

static HAL_StatusTypeDef sdmmc_send_cmd(SDMMC_TypeDef *inst,
                                        uint32_t cmd_idx,
                                        uint32_t arg,
                                        uint32_t waitresp,
                                        bool ignore_crc_fail,
                                        uint32_t *response)
{
    uint32_t start = HAL_GetTick();
    uint32_t status;
    uint32_t cmd = SDMMC_CMD_CPSMEN | (cmd_idx & 0x3FU);

    if (waitresp == 1U) {
        cmd |= SDMMC_CMD_WAITRESP_0;
    } else if (waitresp == 3U) {
        cmd |= SDMMC_CMD_WAITRESP_0 | SDMMC_CMD_WAITRESP_1;
    }

    inst->ICR = 0xFFFFFFFFU;
    inst->ARG = arg;
    inst->CMD = cmd;

    while (1) {
        status = inst->STA;

        if (waitresp == 0U) {
            if ((status & SDMMC_STA_CMDSENT) != 0U) {
                break;
            }
        } else {
            if ((status & SDMMC_STA_CMDREND) != 0U) {
                break;
            }
            if ((status & SDMMC_STA_CCRCFAIL) != 0U) {
                if (ignore_crc_fail) {
                    break;
                }
                return HAL_ERROR;
            }
        }

        if ((status & SDMMC_STA_CTIMEOUT) != 0U) {
            return HAL_TIMEOUT;
        }

        if ((HAL_GetTick() - start) > 100U) {
            return HAL_TIMEOUT;
        }
    }

    if (response != NULL) {
        *response = inst->RESP1;
    }

    inst->ICR = 0xFFFFFFFFU;
    return HAL_OK;
}

static bool sdmmc2_probe_card(uint32_t *ocr_out)
{
    uint32_t resp = 0U;
    uint32_t i;

    __HAL_RCC_SDMMC2_CLK_ENABLE();

    SDMMC2->POWER = 0x3U;
    sdmmc_set_clock(SDMMC2, 250U);

    if (sdmmc_send_cmd(SDMMC2, 0U, 0U, 0U, true, NULL) != HAL_OK) {
        return false;
    }

    (void)sdmmc_send_cmd(SDMMC2, 8U, 0x1AAU, 1U, false, &resp);

    for (i = 0U; i < 100U; ++i) {
        if (sdmmc_send_cmd(SDMMC2, 55U, 0U, 1U, false, &resp) != HAL_OK) {
            return false;
        }

        if (sdmmc_send_cmd(SDMMC2, 41U, 0x40FF8000U, 1U, true, &resp) != HAL_OK) {
            return false;
        }

        if ((resp & 0x80000000U) != 0U) {
            if (ocr_out != NULL) {
                *ocr_out = resp;
            }
            return true;
        }

        HAL_Delay(10U);
    }

    return false;
}

void test_memory_microsd(board_test_result_t *result)
{
    uint32_t ocr = 0U;
    char measured[64];

    if (sdmmc2_probe_card(&ocr)) {
        (void)snprintf(measured, sizeof(measured), "OCR=0x%08lX", (unsigned long)ocr);
        board_test_set_result(result,
                              "memory.microsd",
                              "microSD socket",
                              TEST_STATUS_PASS,
                              TEST_MODE_INTERACTIVE,
                              measured,
                              "SDMMC2 command sequence succeeded; card detected.",
                              "");
    } else {
        board_test_set_result(result,
                              "memory.microsd",
                              "microSD socket",
                              TEST_STATUS_SKIP,
                              TEST_MODE_INTERACTIVE,
                              "no SD response",
                              "No card response on SDMMC2_CMD/CLK path.",
                              "Insert known-good microSD card and rerun this test.");
    }
}
