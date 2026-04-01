#include "board_test.h"

#include <stdio.h>
#include <string.h>

#define RESET_CHECK_MAGIC 0xB007B007UL

static void append_flag(char *dst, size_t len, const char *flag)
{
    if ((dst == NULL) || (flag == NULL) || (len == 0U)) {
        return;
    }

    if (dst[0] != '\0') {
        (void)strncat(dst, "|", len - strlen(dst) - 1U);
    }
    (void)strncat(dst, flag, len - strlen(dst) - 1U);
}

void test_reset_source(board_test_result_t *result)
{
    uint32_t rsr = RCC->RSR;
    char flags[128] = {0};

    if ((rsr & RCC_RSR_PORRSTF) != 0U) {
        append_flag(flags, sizeof(flags), "POR");
    }
    if ((rsr & RCC_RSR_BORRSTF) != 0U) {
        append_flag(flags, sizeof(flags), "BOR");
    }
    if ((rsr & RCC_RSR_PINRSTF) != 0U) {
        append_flag(flags, sizeof(flags), "PIN");
    }
    if ((rsr & RCC_RSR_SFTRSTF) != 0U) {
        append_flag(flags, sizeof(flags), "SW");
    }
    if ((rsr & RCC_RSR_IWDG1RSTF) != 0U) {
        append_flag(flags, sizeof(flags), "IWDG1");
    }
    if ((rsr & RCC_RSR_WWDG1RSTF) != 0U) {
        append_flag(flags, sizeof(flags), "WWDG1");
    }
    if ((rsr & RCC_RSR_LPWRRSTF) != 0U) {
        append_flag(flags, sizeof(flags), "LPWR");
    }

    if (flags[0] == '\0') {
        (void)snprintf(flags, sizeof(flags), "none");
    }

    board_test_set_result(result,
                          "core.reset_source",
                          "Reset controller",
                          TEST_STATUS_PASS,
                          TEST_MODE_AUTOMATIC,
                          flags,
                          "Reset cause flags sampled from RCC->RSR.",
                          "Use 'reboot' or press S1 then rerun to confirm expected reset path.");
}

void test_boot_and_reset_sanity(board_test_result_t *result)
{
    uint32_t rsr = RCC->RSR;
    uint32_t marker = 0U;
    GPIO_PinState man_reset_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
    char measured[96];

    HAL_PWR_EnableBkUpAccess();
    marker = RTC->BKP0R;

    (void)snprintf(measured,
                   sizeof(measured),
                   "BOOT0=strap_low(schematic),!MAN_RESET=%s,RSR=0x%08lX",
                   (man_reset_state == GPIO_PIN_SET) ? "HIGH" : "LOW",
                   (unsigned long)rsr);

    if (marker == RESET_CHECK_MAGIC) {
        RTC->BKP0R = 0U;
        __HAL_RCC_CLEAR_RESET_FLAGS();

        if ((rsr & RCC_RSR_PINRSTF) != 0U) {
            board_test_set_result(result,
                                  "reset.boot_sanity",
                                  "Boot/Reset straps",
                                  TEST_STATUS_PASS,
                                  TEST_MODE_INTERACTIVE,
                                  measured,
                                  "Manual reset path confirmed (PINRST flag observed after armed check).",
                                  "");
        } else {
            board_test_set_result(result,
                                  "reset.boot_sanity",
                                  "Boot/Reset straps",
                                  TEST_STATUS_WARN,
                                  TEST_MODE_INTERACTIVE,
                                  measured,
                                  "Armed reset check completed but PINRST flag was not observed.",
                                  "Repeat test: run 'run reset' then press S1 reset button once.");
        }
        return;
    }

    RTC->BKP0R = RESET_CHECK_MAGIC;
    board_test_set_result(result,
                          "reset.boot_sanity",
                          "Boot/Reset straps",
                          TEST_STATUS_REQUIRES_FIXTURE,
                          TEST_MODE_INTERACTIVE,
                          measured,
                          "BOOT0 is fixed by resistor (R13 to GND) and cannot be toggled in firmware.",
                          "Press S1 reset button once, then run 'run reset' again to confirm clean reboot path.");
}
