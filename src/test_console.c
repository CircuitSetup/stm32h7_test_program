#include "test_console.h"

#include "board_test.h"
#include "main.h"
#include "test_uart.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CONSOLE_LINE_MAX 160U

static void print_prompt(void)
{
    test_uart_write_str("board_test> ");
}

static int str_ieq(const char *a, const char *b)
{
    while ((*a != '\0') && (*b != '\0')) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) {
            return 0;
        }
        ++a;
        ++b;
    }
    return (*a == '\0') && (*b == '\0');
}

static void print_help(void)
{
    test_uart_write_str("\r\nCommands:\r\n");
    test_uart_write_str("  help\r\n");
    test_uart_write_str("  run_all\r\n");
    test_uart_write_str("  run <group|test_name>\r\n");
    test_uart_write_str("  run ade <index>\r\n");
    test_uart_write_str("  run analog_fixture\r\n");
    test_uart_write_str("  list\r\n");
    test_uart_write_str("  summary\r\n");
    test_uart_write_str("  dump_board_map\r\n");
    test_uart_write_str("  dump_gpio\r\n");
    test_uart_write_str("  ade_scan\r\n");
    test_uart_write_str("  run usb\r\n");
    test_uart_write_str("  eth_info\r\n");
    test_uart_write_str("  wifi_info\r\n");
    test_uart_write_str("  dump_registers ade <index>\r\n");
    test_uart_write_str("  set verbose 0|1\r\n");
    test_uart_write_str("  reboot\r\n\r\n");
}

static void handle_run_command(char *arg1, char *arg2)
{
    if ((arg1 == NULL) || (arg1[0] == '\0')) {
        test_uart_write_str("Usage: run <group|test_name>\r\n");
        return;
    }

    if (str_ieq(arg1, "analog_fixture")) {
        board_test_run_analog_fixture();
        return;
    }

    if (str_ieq(arg1, "ade") && (arg2 != NULL)) {
        unsigned long idx = strtoul(arg2, NULL, 0);
        if ((idx >= 1UL) && (idx <= BOARD_ADE_COUNT)) {
            test_uart_printf("Running ADE single-device test for index %lu...\r\n", idx);
            board_test_run_ade_index((uint8_t)idx);
            return;
        }
        test_uart_write_str("ADE index must be 1..8\r\n");
        return;
    }

    if (str_ieq(arg1, "core") ||
        str_ieq(arg1, "gpio") ||
        str_ieq(arg1, "power") ||
        str_ieq(arg1, "eth") ||
        str_ieq(arg1, "wifi") ||
        str_ieq(arg1, "bt") ||
        str_ieq(arg1, "ade") ||
        str_ieq(arg1, "analog") ||
        str_ieq(arg1, "reset") ||
        str_ieq(arg1, "uart") ||
        str_ieq(arg1, "usb") ||
        str_ieq(arg1, "memory") ||
        str_ieq(arg1, "human")) {
        if (str_ieq(arg1, "ade")) {
            test_uart_write_str("Running ADE group (scan, IRQ idle, register snapshot)...\r\n");
        } else {
            test_uart_printf("Running group '%s'...\r\n", arg1);
        }
        board_test_run_group(arg1);
        return;
    }

    if (str_ieq(arg1, "reset")) {
        board_test_run_group("reset");
        return;
    }

    board_test_run_named(arg1);
}

static void handle_command(char *line)
{
    char *cmd;
    char *arg1;
    char *arg2;
    char *arg3;

    cmd = strtok(line, " \t");
    if (cmd == NULL) {
        return;
    }

    arg1 = strtok(NULL, " \t");
    arg2 = strtok(NULL, " \t");
    arg3 = strtok(NULL, " \t");

    if (str_ieq(cmd, "help")) {
        print_help();
        return;
    }

    if (str_ieq(cmd, "run_all")) {
        test_uart_write_str("Running full test sequence...\r\n");
        board_test_run_all();
        return;
    }

    if (str_ieq(cmd, "run")) {
        handle_run_command(arg1, arg2);
        return;
    }

    if (str_ieq(cmd, "list")) {
        board_test_list_tests();
        return;
    }

    if (str_ieq(cmd, "summary")) {
        board_test_print_summary();
        return;
    }

    if (str_ieq(cmd, "dump_board_map")) {
        board_test_dump_board_map();
        return;
    }

    if (str_ieq(cmd, "dump_gpio")) {
        board_test_dump_gpio_policy();
        return;
    }

    if (str_ieq(cmd, "ade_scan")) {
        board_test_run_group("ade");
        return;
    }

    if (str_ieq(cmd, "eth_info")) {
        test_ethernet_print_info();
        return;
    }

    if (str_ieq(cmd, "wifi_info")) {
        test_wifi_print_info();
        return;
    }

    if (str_ieq(cmd, "dump_registers")) {
        if ((arg1 != NULL) && str_ieq(arg1, "ade") && (arg2 != NULL)) {
            unsigned long idx = strtoul(arg2, NULL, 0);
            if ((idx >= 1UL) && (idx <= BOARD_ADE_COUNT)) {
                board_test_dump_registers_ade((uint8_t)idx);
            } else {
                test_uart_write_str("Usage: dump_registers ade <index 1..8>\r\n");
            }
        } else {
            test_uart_write_str("Usage: dump_registers ade <index 1..8>\r\n");
        }
        return;
    }

    if (str_ieq(cmd, "set")) {
        if ((arg1 != NULL) && str_ieq(arg1, "verbose") && (arg2 != NULL)) {
            unsigned long v = strtoul(arg2, NULL, 0);
            board_test_set_verbose((v != 0UL) ? 1U : 0U);
            test_uart_printf("verbose=%u\r\n", board_test_get_verbose());
        } else {
            test_uart_write_str("Usage: set verbose 0|1\r\n");
        }
        return;
    }

    if (str_ieq(cmd, "reboot")) {
        test_uart_write_str("Rebooting...\r\n");
        HAL_Delay(20U);
        NVIC_SystemReset();
        return;
    }

    (void)arg3;
    test_uart_write_str("Unknown command. Type 'help'.\r\n");
}

void test_console_banner(void)
{
    test_uart_printf("\r\n%s %s\r\n", FW_NAME, FW_VERSION);
    test_uart_printf("board=%s mcu=STM32H743VIT6 build=%s %s\r\n", FW_BOARD_REV, __DATE__, __TIME__);
    test_uart_printf("UID=%08lX-%08lX-%08lX\r\n",
                     (unsigned long)HAL_GetUIDw0(),
                     (unsigned long)HAL_GetUIDw1(),
                     (unsigned long)HAL_GetUIDw2());
    test_uart_write_str("Console on USART3 (PD8/PD9 test points).\r\n");
    test_uart_write_str("Type 'help' for commands.\r\n\r\n");
}

void test_console_show_prompt(void)
{
    print_prompt();
}

void test_console_poll(void)
{
    char line[CONSOLE_LINE_MAX];
    int n = test_uart_read_line(line, sizeof(line), 20U);

    if (n <= 0) {
        return;
    }

    handle_command(line);
    test_console_show_prompt();
}
