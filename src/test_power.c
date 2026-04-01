#include "board_test.h"
#include "test_uart.h"

#include <stdio.h>
#include <string.h>

#ifndef VREFINT_CAL_ADDR
#define VREFINT_CAL_ADDR ((uint16_t *)0x1FF1E860UL)
#endif

#define POWER_ADC_CAL_RETRIES 3U
#define POWER_ADC_READ_RETRIES 2U
#define POWER_ADC_RETRY_DELAY_MS 10U

static HAL_StatusTypeDef adc_read_channel(uint32_t channel, uint32_t *out_value);

static const char *power_ckper_source_name(void)
{
#if defined(__HAL_RCC_GET_CLKP_SOURCE)
    switch (__HAL_RCC_GET_CLKP_SOURCE()) {
    case RCC_CLKPSOURCE_HSE:
        return "HSE";
    case RCC_CLKPSOURCE_HSI:
        return "HSI";
    case RCC_CLKPSOURCE_CSI:
        return "CSI";
    default:
        return "unknown";
    }
#else
    return "unknown";
#endif
}

static HAL_StatusTypeDef adc_read_channel_with_retry(uint32_t channel, uint32_t retries, uint32_t *out_value)
{
    uint32_t attempt;
    HAL_StatusTypeDef status = HAL_ERROR;

    if (out_value == NULL) {
        return HAL_ERROR;
    }

    for (attempt = 0U; attempt < retries; ++attempt) {
        status = adc_read_channel(channel, out_value);
        if (status == HAL_OK) {
            return HAL_OK;
        }
        HAL_Delay(POWER_ADC_RETRY_DELAY_MS);
    }

    return status;
}

static HAL_StatusTypeDef adc_read_channel(uint32_t channel, uint32_t *out_value)
{
    ADC_ChannelConfTypeDef sConfig;

    if (out_value == NULL) {
        return HAL_ERROR;
    }

    memset(&sConfig, 0, sizeof(sConfig));
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_ADC_Start(&hadc3) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_ADC_PollForConversion(&hadc3, 50U) != HAL_OK) {
        (void)HAL_ADC_Stop(&hadc3);
        return HAL_TIMEOUT;
    }

    *out_value = HAL_ADC_GetValue(&hadc3);
    (void)HAL_ADC_Stop(&hadc3);

    return HAL_OK;
}

void test_power_internal_rails(board_test_result_t *result)
{
    uint32_t vref_raw = 0U;
    uint32_t temp_raw = 0U;
    uint32_t vdda_mv = 0U;
    uint32_t adc_clk_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ADC);
    uint32_t cal_attempt;
    HAL_StatusTypeDef cal_status = HAL_ERROR;
    char measured[160];

    test_uart_printf("[ INFO ] power.internal: calibrating ADC3 (clk=%lu Hz, ckper=%s, hse_fallback=%lu)...\r\n",
                     (unsigned long)adc_clk_hz,
                     power_ckper_source_name(),
                     (unsigned long)g_clock_fallback_hsi);
    for (cal_attempt = 1U; cal_attempt <= POWER_ADC_CAL_RETRIES; ++cal_attempt) {
        cal_status = HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
        if (cal_status == HAL_OK) {
            break;
        }
        HAL_Delay(POWER_ADC_RETRY_DELAY_MS);
    }

    if (cal_status != HAL_OK) {
        (void)snprintf(measured,
                       sizeof(measured),
                       "adc_cal=failed,attempts=%lu,adc_clk_hz=%lu,ckper=%s,hse_fallback=%lu",
                       (unsigned long)POWER_ADC_CAL_RETRIES,
                       (unsigned long)adc_clk_hz,
                       power_ckper_source_name(),
                       (unsigned long)g_clock_fallback_hsi);
        board_test_set_result(result,
                              "power.internal",
                              "MCU internal ADC",
                              TEST_STATUS_WARN,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "ADC3 internal calibration failed after retries; non-blocking for this design because metrology is handled by ADE7816 devices.",
                              "Verify VDDA/3V3 analog domain and ADC kernel clock source (CKPER from HSE bypass Y3 or HSI fallback).");
        return;
    }

    test_uart_write_str("[ INFO ] power.internal: reading VREFINT/TEMPSENSOR...\r\n");
    if (adc_read_channel_with_retry(ADC_CHANNEL_VREFINT, POWER_ADC_READ_RETRIES, &vref_raw) != HAL_OK) {
        (void)snprintf(measured,
                       sizeof(measured),
                       "vref=timeout,adc_clk_hz=%lu,ckper=%s,hse_fallback=%lu",
                       (unsigned long)adc_clk_hz,
                       power_ckper_source_name(),
                       (unsigned long)g_clock_fallback_hsi);
        board_test_set_result(result,
                              "power.internal",
                              "MCU internal ADC",
                              TEST_STATUS_WARN,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "Could not read VREFINT after retries; non-blocking for this board function.",
                              "Check ADC3 clocking and internal channel path if you need MCU-internal rail telemetry.");
        return;
    }

    (void)adc_read_channel_with_retry(ADC_CHANNEL_TEMPSENSOR, POWER_ADC_READ_RETRIES, &temp_raw);

    if (vref_raw != 0U) {
        vdda_mv = ((uint32_t)(*VREFINT_CAL_ADDR) * 3300U) / vref_raw;
    }

    (void)snprintf(measured,
                   sizeof(measured),
                   "vref_raw=%lu,temp_raw=%lu,vd_da_mv=%lu,adc_clk_hz=%lu,ckper=%s,hse_fallback=%lu",
                   (unsigned long)vref_raw,
                   (unsigned long)temp_raw,
                   (unsigned long)vdda_mv,
                   (unsigned long)adc_clk_hz,
                   power_ckper_source_name(),
                   (unsigned long)g_clock_fallback_hsi);

    if ((vdda_mv > 2800U) && (vdda_mv < 3600U)) {
        board_test_set_result(result,
                              "power.internal",
                              "MCU internal ADC",
                              TEST_STATUS_PASS,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "Internal rail sanity (VREFINT-derived VDDA) within expected range.",
                              "");
    } else {
        board_test_set_result(result,
                              "power.internal",
                              "MCU internal ADC",
                              TEST_STATUS_WARN,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "VDDA estimate outside nominal 3.3V band.",
                              "Verify 3V3/VDD_MCU rails and ferrite FB2 path.");
    }
}

void test_power_rail_inference(board_test_result_t *result)
{
    board_test_result_t ade;
    board_test_result_t eth;
    board_test_result_t wifi;
    uint8_t verbose_saved;
    char measured[160];

    verbose_saved = board_test_get_verbose();
    board_test_set_verbose(0U);
    test_ade_scan_all(&ade);
    test_ethernet_phy(&eth);
    test_wifi_sdio_presence(&wifi);
    board_test_set_verbose(verbose_saved);

    (void)snprintf(measured,
                   sizeof(measured),
                   "3V3_ADE=%s,VDD_ETH=%s,3V3_WIFI=%s",
                   board_map_status_to_string(ade.status),
                   board_map_status_to_string(eth.status),
                   board_map_status_to_string(wifi.status));

    if ((ade.status == TEST_STATUS_FAIL) || (eth.status == TEST_STATUS_FAIL)) {
        board_test_set_result(result,
                              "power.inference",
                              "Derived rail inference",
                              TEST_STATUS_WARN,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "One or more domain inferences failed from peripheral enumeration.",
                              "Run individual ADE/Ethernet/Wi-Fi tests and inspect power tree segments.");
    } else {
        board_test_set_result(result,
                              "power.inference",
                              "Derived rail inference",
                              TEST_STATUS_PASS,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "Indirect rail inference succeeded for measured domains.",
                              "");
    }
}
