#include "board_test.h"

#include <stdio.h>

void test_clock_tree(board_test_result_t *result)
{
    uint32_t hse_ready = __HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY);
    uint32_t lse_ready = __HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY);
    uint32_t sysclk = HAL_RCC_GetSysClockFreq();
    char measured[128];

    (void)snprintf(measured,
                   sizeof(measured),
                   "SYSCLK=%lu Hz,HSERDY=%lu,LSERDY=%lu,fallback_hsi=%lu",
                   (unsigned long)sysclk,
                   (unsigned long)hse_ready,
                   (unsigned long)lse_ready,
                   (unsigned long)g_clock_fallback_hsi);

    if ((hse_ready != 0U) && (lse_ready != 0U) && (g_clock_fallback_hsi == 0U)) {
        board_test_set_result(result,
                              "core.clock_tree",
                              "STM32H743 clocks",
                              TEST_STATUS_PASS,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "HSE bypass and LSE bypass both report ready.",
                              "");
    } else if (g_clock_fallback_hsi != 0U) {
        board_test_set_result(result,
                              "core.clock_tree",
                              "STM32H743 clocks",
                              TEST_STATUS_WARN,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "System is running from HSI fallback; check Y3 25MHz path and HSE bypass wiring.",
                              "Inspect Y3, R26, and PH0-OSC_IN net (25MHZ_2)." );
    } else {
        board_test_set_result(result,
                              "core.clock_tree",
                              "STM32H743 clocks",
                              TEST_STATUS_WARN,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "One or more external clocks did not assert ready flag.",
                              "Verify Y2 32.768k and Y3 25MHz oscillator rails and routing.");
    }
}

void test_clock_systick(board_test_result_t *result)
{
    uint32_t t0 = HAL_GetTick();
    HAL_Delay(20U);
    uint32_t t1 = HAL_GetTick();
    char measured[64];

    (void)snprintf(measured, sizeof(measured), "tick_delta_ms=%lu", (unsigned long)(t1 - t0));

    if ((t1 > t0) && ((t1 - t0) <= 30U)) {
        board_test_set_result(result,
                              "core.systick",
                              "SysTick",
                              TEST_STATUS_PASS,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "SysTick advanced monotonically within expected window.",
                              "");
    } else {
        board_test_set_result(result,
                              "core.systick",
                              "SysTick",
                              TEST_STATUS_FAIL,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "SysTick did not advance as expected.",
                              "Check system clock config and HAL tick source.");
    }
}
