#ifndef BOARD_TEST_H
#define BOARD_TEST_H

#include "board_map.h"

typedef void (*board_test_fn_t)(board_test_result_t *result);

typedef struct {
    const char *name;
    const char *group;
    const char *component;
    test_mode_t mode;
    board_test_fn_t run;
} board_test_case_t;

void board_test_init(void);
void board_test_run_all(void);
void board_test_run_group(const char *group_name);
void board_test_run_named(const char *test_name);
void board_test_run_ade_index(uint8_t index);
void board_test_run_analog_fixture(void);
void board_test_print_summary(void);
void board_test_list_tests(void);
void board_test_dump_board_map(void);
void board_test_dump_gpio_policy(void);
void board_test_dump_registers_ade(uint8_t index);
void board_test_set_verbose(uint8_t verbose);
uint8_t board_test_get_verbose(void);

void board_test_set_result(board_test_result_t *result,
                           const char *name,
                           const char *component,
                           test_status_t status,
                           test_mode_t mode,
                           const char *measured,
                           const char *detail,
                           const char *operator_action);

void board_test_log_result(const board_test_result_t *result);

/* Module tests */
void test_reset_source(board_test_result_t *result);
void test_boot_and_reset_sanity(board_test_result_t *result);
void test_clock_tree(board_test_result_t *result);
void test_clock_systick(board_test_result_t *result);
void test_uart_console_path(board_test_result_t *result);
void test_gpio_walk_outputs(board_test_result_t *result);
void test_gpio_log_inputs(board_test_result_t *result);
void test_power_internal_rails(board_test_result_t *result);
void test_power_rail_inference(board_test_result_t *result);
void test_ethernet_phy(board_test_result_t *result);
void test_ethernet_link(board_test_result_t *result);
void test_wifi_sdio_presence(board_test_result_t *result);
void test_bt_uart_hci(board_test_result_t *result);
void test_ade_scan_all(board_test_result_t *result);
void test_ade_single_device(uint8_t index, board_test_result_t *result);
void test_ade_irq_lines(board_test_result_t *result);
void test_ade_register_snapshot(board_test_result_t *result);
void test_led_button(board_test_result_t *result);
void test_analog_sanity(board_test_result_t *result);
void test_analog_fixture(board_test_result_t *result);
void test_analog_fixture_set_explicit(uint8_t explicit_request);
void test_memory_sram(board_test_result_t *result);
void test_memory_flash_crc(board_test_result_t *result);
void test_memory_microsd(board_test_result_t *result);
void test_usb_fs_device(board_test_result_t *result);

void test_ethernet_print_info(void);
void test_wifi_print_info(void);
void test_ade_dump_registers(uint8_t index);
void test_usb_irq_handler(void);

#endif
