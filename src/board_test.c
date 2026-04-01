#include "board_test.h"

#include "ap6256_driver.h"
#include "test_ade7816.h"
#include "test_uart.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#define BOARD_TEST_MAX_RESULTS 128U

typedef struct {
    const board_test_case_t *tc;
    board_test_result_t result;
} board_result_entry_t;

static uint8_t s_verbose = 1U;
static board_result_entry_t s_results[BOARD_TEST_MAX_RESULTS];
static size_t s_result_count = 0U;

static int board_test_str_ieq(const char *a, const char *b)
{
    if ((a == NULL) || (b == NULL)) {
        return 0;
    }

    while ((*a != '\0') && (*b != '\0')) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) {
            return 0;
        }
        ++a;
        ++b;
    }

    return (*a == '\0') && (*b == '\0');
}

static const board_test_case_t s_test_cases[] = {
    { "core.reset_source", "core", "STM32H743", TEST_MODE_AUTOMATIC, test_reset_source },
    { "core.clock_tree", "core", "STM32H743", TEST_MODE_AUTOMATIC, test_clock_tree },
    { "core.systick", "core", "STM32H743", TEST_MODE_AUTOMATIC, test_clock_systick },
    { "core.sram_march", "core", "STM32H743", TEST_MODE_AUTOMATIC, test_memory_sram },
    { "core.flash_crc", "core", "STM32H743", TEST_MODE_AUTOMATIC, test_memory_flash_crc },

    { "reset.boot_sanity", "reset", "Reset/Boot tree", TEST_MODE_INTERACTIVE, test_boot_and_reset_sanity },
    { "uart.console", "uart", "USART3", TEST_MODE_AUTOMATIC, test_uart_console_path },
    { "usb.fs", "usb", "USB-C OTG FS", TEST_MODE_INTERACTIVE, test_usb_fs_device },

    { "power.internal", "power", "Power rails", TEST_MODE_AUTOMATIC, test_power_internal_rails },
    { "power.inference", "power", "Power rails", TEST_MODE_AUTOMATIC, test_power_rail_inference },

    { "gpio.safe_outputs", "gpio", "GPIO", TEST_MODE_AUTOMATIC, test_gpio_walk_outputs },
    { "gpio.safe_inputs", "gpio", "GPIO", TEST_MODE_AUTOMATIC, test_gpio_log_inputs },

    { "eth.phy", "eth", "DP83640", TEST_MODE_AUTOMATIC, test_ethernet_phy },
    { "eth.link", "eth", "DP83640/RJ45", TEST_MODE_INTERACTIVE, test_ethernet_link },

    { "wifi.sdio", "wifi", "AP6256", TEST_MODE_AUTOMATIC, test_wifi_sdio_presence },
    { "bt.hci", "bt", "AP6256", TEST_MODE_AUTOMATIC, test_bt_uart_hci },

    { "ade.scan", "ade", "ADE7816 x8", TEST_MODE_AUTOMATIC, test_ade_scan_all },
    { "ade.irq_idle", "ade", "ADE7816 IRQ", TEST_MODE_SANITY, test_ade_irq_lines },
    { "ade.registers", "ade", "ADE7816 registers", TEST_MODE_AUTOMATIC, test_ade_register_snapshot },

    { "analog.sanity", "analog", "Metering AFE", TEST_MODE_SANITY, test_analog_sanity },
    { "analog.fixture", "analog", "Metering AFE", TEST_MODE_FIXTURE, test_analog_fixture },

    { "memory.microsd", "memory", "microSD socket", TEST_MODE_INTERACTIVE, test_memory_microsd },

    { "human.led_button", "human", "LED/Button", TEST_MODE_SANITY, test_led_button }
};

#define BOARD_TEST_CASE_COUNT (sizeof(s_test_cases) / sizeof(s_test_cases[0]))

static void copy_text(char *dst, size_t dst_len, const char *src)
{
    if ((dst == NULL) || (dst_len == 0U)) {
        return;
    }

    if (src == NULL) {
        dst[0] = '\0';
        return;
    }

    (void)snprintf(dst, dst_len, "%s", src);
}

static const char *field_or_na(const char *text)
{
    if ((text == NULL) || (text[0] == '\0')) {
        return "n/a";
    }

    return text;
}

static void log_case_start(const board_test_case_t *tc, size_t step, size_t total)
{
    if (tc == NULL) {
        return;
    }

    if (total == 0U) {
        total = 1U;
    }

    test_uart_write_str("\r\n--- TEST START ------------------------------------------------\r\n");
    test_uart_printf("Status    : RUNNING\r\n");
    test_uart_printf("Test      : %s\r\n", field_or_na(tc->name));
    test_uart_printf("Group     : %s\r\n", field_or_na(tc->group));
    test_uart_printf("Component : %s\r\n", field_or_na(tc->component));
    test_uart_printf("Mode      : %s\r\n", board_map_mode_to_string(tc->mode));
    test_uart_printf("Progress  : %lu/%lu\r\n",
                     (unsigned long)step,
                     (unsigned long)total);
}

static void log_case_done(const board_test_result_t *result, uint32_t elapsed_ms)
{
    if (result == NULL) {
        return;
    }

    test_uart_printf("Elapsed   : %lu ms\r\n", (unsigned long)elapsed_ms);
    test_uart_printf("Done      : %s (%s)\r\n",
                     field_or_na(result->test_name),
                     board_map_status_to_string(result->status));
    test_uart_write_str("----------------------------------------------------------------\r\n");
}

static void reset_result_store(void)
{
    memset(s_results, 0, sizeof(s_results));
    s_result_count = 0U;
}

static void append_result(const board_test_case_t *tc, const board_test_result_t *result)
{
    if ((result == NULL) || (s_result_count >= BOARD_TEST_MAX_RESULTS)) {
        return;
    }

    s_results[s_result_count].tc = tc;
    s_results[s_result_count].result = *result;
    s_result_count++;
}

static void run_one_case(const board_test_case_t *tc, size_t step, size_t total)
{
    board_test_result_t result;
    uint32_t start_ms;

    if ((tc == NULL) || (tc->run == NULL)) {
        return;
    }

    log_case_start(tc, step, total);
    start_ms = HAL_GetTick();

    board_test_set_result(&result,
                          tc->name,
                          tc->component,
                          TEST_STATUS_FAIL,
                          tc->mode,
                          "",
                          "Test did not run.",
                          "");

    tc->run(&result);
    append_result(tc, &result);
    board_test_log_result(&result);
    log_case_done(&result, HAL_GetTick() - start_ms);
}

void board_test_init(void)
{
    reset_result_store();
#if BOARD_HAS_AP6256
    ap6256_init();
#endif
}

void board_test_set_verbose(uint8_t verbose)
{
    s_verbose = (verbose != 0U) ? 1U : 0U;
}

uint8_t board_test_get_verbose(void)
{
    return s_verbose;
}

void board_test_set_result(board_test_result_t *result,
                           const char *name,
                           const char *component,
                           test_status_t status,
                           test_mode_t mode,
                           const char *measured,
                           const char *detail,
                           const char *operator_action)
{
    if (result == NULL) {
        return;
    }

    memset(result, 0, sizeof(*result));
    copy_text(result->test_name, sizeof(result->test_name), name);
    copy_text(result->component, sizeof(result->component), component);
    copy_text(result->measured_value_text, sizeof(result->measured_value_text), measured);
    copy_text(result->detail, sizeof(result->detail), detail);
    copy_text(result->operator_action_required, sizeof(result->operator_action_required), operator_action);
    result->status = status;
    result->mode = mode;
    result->timestamp_ms = HAL_GetTick();
}

void board_test_log_result(const board_test_result_t *result)
{
    if (result == NULL) {
        return;
    }

    test_uart_write_str("--- TEST RESULT -----------------------------------------------\r\n");
    test_uart_printf("Status    : %s\r\n",
                     board_map_status_to_string(result->status));
    test_uart_printf("Test      : %s\r\n", field_or_na(result->test_name));
    test_uart_printf("Component : %s\r\n", field_or_na(result->component));
    test_uart_printf("Mode      : %s\r\n", board_map_mode_to_string(result->mode));
    test_uart_printf("Timestamp : %lu ms\r\n", (unsigned long)result->timestamp_ms);

    if (s_verbose != 0U) {
        test_uart_printf("Measured  : %s\r\n", field_or_na(result->measured_value_text));
        test_uart_printf("Detail    : %s\r\n", field_or_na(result->detail));
        test_uart_printf("Action    : %s\r\n", field_or_na(result->operator_action_required));
    }

#if BOARD_ENABLE_CSV_LOG
    test_uart_printf("CSV_RESULT,%lu,%s,%s,%s,%s\r\n",
                     (unsigned long)result->timestamp_ms,
                     result->test_name,
                     result->component,
                     board_map_status_to_string(result->status),
                     result->measured_value_text);
#endif
}

void board_test_run_all(void)
{
    size_t i;

    reset_result_store();
    test_uart_write_str("\r\n=== RUN ALL TESTS ===\r\n");

    for (i = 0U; i < BOARD_TEST_CASE_COUNT; ++i) {
        run_one_case(&s_test_cases[i], i + 1U, BOARD_TEST_CASE_COUNT);
    }

    board_test_print_summary();
}

void board_test_run_group(const char *group_name)
{
    size_t i;
    size_t total = 0U;
    size_t matched = 0U;

    if ((group_name == NULL) || (group_name[0] == '\0')) {
        test_uart_write_str("Group name required.\r\n");
        return;
    }

    for (i = 0U; i < BOARD_TEST_CASE_COUNT; ++i) {
        if (board_test_str_ieq(s_test_cases[i].group, group_name) != 0) {
            total++;
        }
    }

    if (total == 0U) {
        test_uart_printf("No tests found for group '%s'.\r\n", group_name);
        return;
    }

    reset_result_store();
    test_uart_printf("\r\n=== RUN GROUP: %s ===\r\n", group_name);

    for (i = 0U; i < BOARD_TEST_CASE_COUNT; ++i) {
        if (board_test_str_ieq(s_test_cases[i].group, group_name) != 0) {
            matched++;
            run_one_case(&s_test_cases[i], matched, total);
        }
    }

    test_uart_printf("\r\n=== GROUP COMPLETE: %s (executed=%lu) ===\r\n",
                     group_name,
                     (unsigned long)total);
    board_test_print_summary();
}

void board_test_run_named(const char *test_name)
{
    size_t i;

    if ((test_name == NULL) || (test_name[0] == '\0')) {
        test_uart_write_str("Test name required.\r\n");
        return;
    }

    for (i = 0U; i < BOARD_TEST_CASE_COUNT; ++i) {
        if (board_test_str_ieq(s_test_cases[i].name, test_name) != 0) {
            run_one_case(&s_test_cases[i], 1U, 1U);
            return;
        }
    }

    test_uart_printf("Unknown test '%s'.\r\n", test_name);
}

void board_test_run_ade_index(uint8_t index)
{
    board_test_result_t result;
    uint32_t start_ms;
    const board_test_case_t tc = { "ade.single", "ade", "ADE7816", TEST_MODE_AUTOMATIC, NULL };

    log_case_start(&tc, 1U, 1U);
    start_ms = HAL_GetTick();

    board_test_set_result(&result,
                          "ade.single",
                          "ADE7816",
                          TEST_STATUS_FAIL,
                          TEST_MODE_AUTOMATIC,
                          "",
                          "ADE single-device test failed.",
                          "");

    test_ade_single_device(index, &result);
    append_result(NULL, &result);
    board_test_log_result(&result);
    log_case_done(&result, HAL_GetTick() - start_ms);
}

void board_test_run_analog_fixture(void)
{
    board_test_result_t result;
    uint32_t start_ms;
    const board_test_case_t tc = { "analog.fixture", "analog", "Metering AFE", TEST_MODE_FIXTURE, NULL };

    log_case_start(&tc, 1U, 1U);
    start_ms = HAL_GetTick();

    board_test_set_result(&result,
                          "analog.fixture",
                          "Metering AFE",
                          TEST_STATUS_REQUIRES_FIXTURE,
                          TEST_MODE_FIXTURE,
                          "",
                          "Fixture test not run.",
                          "Apply low-voltage isolated AC fixture before running.");

    test_analog_fixture_set_explicit(1U);
    test_analog_fixture(&result);
    test_analog_fixture_set_explicit(0U);
    append_result(NULL, &result);
    board_test_log_result(&result);
    log_case_done(&result, HAL_GetTick() - start_ms);
}

void board_test_print_summary(void)
{
    size_t i;
    uint32_t pass = 0U;
    uint32_t fail = 0U;
    uint32_t warn = 0U;
    uint32_t skip = 0U;
    uint32_t fixture = 0U;

    for (i = 0U; i < s_result_count; ++i) {
        switch (s_results[i].result.status) {
        case TEST_STATUS_PASS:
            pass++;
            break;
        case TEST_STATUS_FAIL:
            fail++;
            break;
        case TEST_STATUS_WARN:
            warn++;
            break;
        case TEST_STATUS_SKIP:
            skip++;
            break;
        case TEST_STATUS_REQUIRES_FIXTURE:
            fixture++;
            break;
        default:
            break;
        }
    }

    test_uart_write_str("\r\n=== TEST SUMMARY ===\r\n");
    test_uart_printf("Overall   : %s\r\n", (fail == 0U) ? "PASS" : "FAIL");
    test_uart_printf("Counts    : PASS=%lu FAIL=%lu WARN=%lu SKIP=%lu REQUIRES_FIXTURE=%lu TOTAL=%lu\r\n",
                     (unsigned long)pass,
                     (unsigned long)fail,
                     (unsigned long)warn,
                     (unsigned long)skip,
                     (unsigned long)fixture,
                     (unsigned long)s_result_count);
    test_uart_write_str("Results   :\r\n");
    for (i = 0U; i < s_result_count; ++i) {
        test_uart_printf("  [%s] %s (%s)\r\n",
                         board_map_status_to_string(s_results[i].result.status),
                         s_results[i].result.test_name,
                         s_results[i].result.component);
    }

    if (fail > 0U) {
        test_uart_write_str("Failures  :\r\n");
        for (i = 0U; i < s_result_count; ++i) {
            if (s_results[i].result.status == TEST_STATUS_FAIL) {
                test_uart_printf("- %s: %s\r\n",
                                 s_results[i].result.test_name,
                                 s_results[i].result.detail);
            }
        }
    }

    if (fixture > 0U) {
        test_uart_write_str("Fixture   :\r\n");
        for (i = 0U; i < s_result_count; ++i) {
            if (s_results[i].result.status == TEST_STATUS_REQUIRES_FIXTURE) {
                test_uart_printf("- %s: %s\r\n",
                                 s_results[i].result.test_name,
                                 s_results[i].result.operator_action_required);
            }
        }
    }

#if BOARD_ENABLE_CSV_LOG
    test_uart_printf("CSV_SUMMARY,%s,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
                     (fail == 0U) ? "PASS" : "FAIL",
                     (unsigned long)pass,
                     (unsigned long)fail,
                     (unsigned long)warn,
                     (unsigned long)skip,
                     (unsigned long)fixture,
                     (unsigned long)s_result_count);
#endif
}

void board_test_list_tests(void)
{
    size_t i;

    test_uart_write_str("\r\nAvailable tests:\r\n");
    for (i = 0U; i < (sizeof(s_test_cases) / sizeof(s_test_cases[0])); ++i) {
        test_uart_printf("- %-22s group=%-8s mode=%-11s component=%s\r\n",
                         s_test_cases[i].name,
                         s_test_cases[i].group,
                         board_map_mode_to_string(s_test_cases[i].mode),
                         s_test_cases[i].component);
    }
}

void board_test_dump_board_map(void)
{
    const board_signal_map_t *signals;
    size_t i;
    size_t count = 0U;

    signals = board_map_get_signals(&count);
    test_uart_write_str("\r\nBOARD_MAP,mcu_pin,net,peripheral,component,test_mode,test_method,pass_fail\r\n");

    for (i = 0U; i < count; ++i) {
        test_uart_printf("BOARD_MAP,%s,%s,%s,%s,%s,%s,%s\r\n",
                         signals[i].mcu_pin,
                         signals[i].net,
                         signals[i].peripheral,
                         signals[i].external_component,
                         board_map_mode_to_string(signals[i].test_mode),
                         signals[i].test_method,
                         signals[i].pass_fail_criteria);
    }
}

void board_test_dump_gpio_policy(void)
{
    const board_gpio_policy_t *policy;
    size_t count = 0U;
    size_t i;

    policy = board_map_get_gpio_policy(&count);
    test_uart_write_str("\r\nGPIO_POLICY,mcu_pin,net,class,state,note\r\n");

    for (i = 0U; i < count; ++i) {
        const char *state = "NA";
        if (policy[i].port != NULL) {
            state = (HAL_GPIO_ReadPin(policy[i].port, policy[i].pin) == GPIO_PIN_SET) ? "HIGH" : "LOW";
        }
        test_uart_printf("GPIO_POLICY,%s,%s,%s,%s,%s\r\n",
                         policy[i].mcu_pin,
                         policy[i].net,
                         board_map_gpio_class_to_string(policy[i].gpio_class),
                         state,
                         policy[i].note);
    }
}

void board_test_dump_registers_ade(uint8_t index)
{
    test_ade_dump_registers(index);
}
