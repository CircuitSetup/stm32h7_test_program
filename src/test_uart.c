#include "test_uart.h"

#include "board_test.h"

#include "cmsis_os2.h"
#include "main.h"
#include "network_manager.h"
#include "test_rtt.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define UART_TX_TIMEOUT_MS 2000U

static volatile uint8_t s_uart_console_enabled = 1U;
static osMutexId_t s_input_mutex;
static volatile uint32_t s_uart_line_overflows;
static volatile uint32_t s_uart_ignored_chars;
static volatile uint32_t s_uart_command_parse_errors;
static volatile uint32_t s_uart_crlf_tail_consumed;
static volatile uint32_t s_uart_last_prompt_tick;
static volatile uint8_t s_uart_last_command_returned_to_prompt;

static osMutexId_t test_uart_input_mutex(void)
{
    static const osMutexAttr_t input_mutex_attr = {
        .name = "uart_input"
    };

    if (s_input_mutex != NULL) {
        return s_input_mutex;
    }

    if ((osKernelGetState() == osKernelRunning) || (osKernelGetState() == osKernelReady)) {
        s_input_mutex = osMutexNew(&input_mutex_attr);
    }

    return s_input_mutex;
}

static bool test_uart_get_char(uint8_t *ch, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    if (ch == NULL) {
        return false;
    }

    while ((HAL_GetTick() - start) < timeout_ms) {
        if (test_rtt_read(ch, 1U) == 1U) {
            return true;
        }

        if ((s_uart_console_enabled != 0U) &&
            network_manager_console_uart_allowed() &&
            (HAL_UART_Receive(&huart3, ch, 1U, 1U) == HAL_OK)) {
            return true;
        }
    }

    return false;
}

static void test_uart_consume_line_tail(void)
{
    if (test_rtt_consume_line_tail()) {
        s_uart_crlf_tail_consumed++;
    }
}

static int test_uart_read_line_internal(char *buffer,
                                        size_t buffer_len,
                                        uint32_t timeout_ms,
                                        uint8_t masked)
{
    uint32_t start;
    size_t idx = 0U;
    bool overflowed = false;
    osMutexId_t input_mutex = test_uart_input_mutex();

    if ((buffer == NULL) || (buffer_len < 2U)) {
        return -1;
    }

    if ((input_mutex != NULL) && (osMutexAcquire(input_mutex, timeout_ms) != osOK)) {
        return 0;
    }

    memset(buffer, 0, buffer_len);
    start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms) {
        uint8_t ch;

        if (!test_uart_get_char(&ch, 20U)) {
            continue;
        }

        if ((ch == '\r') || (ch == '\n')) {
            if (idx == 0U) {
                continue;
            }
            buffer[idx] = '\0';
            test_uart_write_str("\r\n");
            test_uart_consume_line_tail();
            if (input_mutex != NULL) {
                (void)osMutexRelease(input_mutex);
            }
            return (int)idx;
        }

        if ((ch == 0x08U) || (ch == 0x7FU)) {
            if (idx > 0U) {
                idx--;
                test_uart_write_str("\b \b");
            }
            continue;
        }

        if ((ch < 0x20U) || (ch > 0x7EU)) {
            s_uart_ignored_chars++;
            continue;
        }

        if (idx < (buffer_len - 1U)) {
            static const uint8_t star = '*';

            buffer[idx++] = (char)ch;
            if (masked != 0U) {
                test_uart_write(&star, 1U);
            } else {
                test_uart_write(&ch, 1U);
            }
        } else if (!overflowed) {
            overflowed = true;
            s_uart_line_overflows++;
        }
    }

    if (idx == 0U) {
        if (input_mutex != NULL) {
            (void)osMutexRelease(input_mutex);
        }
        return 0;
    }

    buffer[idx] = '\0';
    if (input_mutex != NULL) {
        (void)osMutexRelease(input_mutex);
    }
    return (int)idx;
}

void test_uart_init(void)
{
    test_rtt_init();
    test_rtt_flush_rx();
    test_uart_printf("[RTT] transport initialized boot=%lu\r\n",
                     (unsigned long)test_rtt_boot_sequence());
}

void test_uart_write(const uint8_t *data, size_t len)
{
    if ((data == NULL) || (len == 0U)) {
        return;
    }

    (void)test_rtt_write(data, len);

#if BOARD_LOG_MIRROR_UART
    if ((s_uart_console_enabled != 0U) && network_manager_console_uart_allowed()) {
        (void)HAL_UART_Transmit(&huart3, (uint8_t *)data, (uint16_t)len, UART_TX_TIMEOUT_MS);
    }
#endif
}

void test_uart_write_str(const char *text)
{
    if (text == NULL) {
        return;
    }

    test_uart_write((const uint8_t *)text, strlen(text));
}

void test_uart_printf(const char *fmt, ...)
{
    char buffer[256];
    va_list args;
    int n;

    if (fmt == NULL) {
        return;
    }

    va_start(args, fmt);
    n = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (n < 0) {
        return;
    }

    if ((size_t)n >= sizeof(buffer)) {
        n = (int)sizeof(buffer) - 1;
    }

    test_uart_write((const uint8_t *)buffer, (size_t)n);
}

bool test_uart_read_bytes(uint8_t *buffer, size_t len, uint32_t timeout_ms)
{
    uint32_t start;
    size_t offset = 0U;
    osMutexId_t input_mutex = test_uart_input_mutex();

    if ((buffer == NULL) || (len == 0U)) {
        return false;
    }

    if ((input_mutex != NULL) && (osMutexAcquire(input_mutex, timeout_ms) != osOK)) {
        return false;
    }

    start = HAL_GetTick();

    while (offset < len) {
        uint32_t elapsed = HAL_GetTick() - start;
        uint32_t remaining = (elapsed >= timeout_ms) ? 0U : (timeout_ms - elapsed);

        if (remaining == 0U) {
            break;
        }

        if (test_rtt_read(&buffer[offset], 1U) == 1U) {
            offset++;
            continue;
        }

        if ((s_uart_console_enabled != 0U) &&
            network_manager_console_uart_allowed() &&
            (HAL_UART_Receive(&huart3, &buffer[offset], 1U, (remaining > 10U) ? 10U : remaining) == HAL_OK)) {
            offset++;
        }
    }

    if (input_mutex != NULL) {
        (void)osMutexRelease(input_mutex);
    }
    return (offset == len);
}

int test_uart_read_line(char *buffer, size_t buffer_len, uint32_t timeout_ms)
{
    return test_uart_read_line_internal(buffer, buffer_len, timeout_ms, 0U);
}

int test_uart_read_line_masked(char *buffer, size_t buffer_len, uint32_t timeout_ms)
{
    return test_uart_read_line_internal(buffer, buffer_len, timeout_ms, 1U);
}

void test_uart_flush_rx(void)
{
    uint8_t ch;

    while (test_rtt_read(&ch, 1U) == 1U) {
        /* flush RTT down channel */
    }

    while ((s_uart_console_enabled != 0U) &&
           network_manager_console_uart_allowed() &&
           (HAL_UART_Receive(&huart3, &ch, 1U, 1U) == HAL_OK)) {
        /* flush */
    }
}

void test_uart_flush_uart_rx(void)
{
    uint8_t ch;

    while (HAL_UART_Receive(&huart3, &ch, 1U, 1U) == HAL_OK) {
        /* flush UART only; RTT may contain queued operator input */
    }
}

void test_uart_set_uart_console_enabled(bool enabled)
{
    s_uart_console_enabled = enabled ? 1U : 0U;
}

bool test_uart_uart_console_enabled(void)
{
    return (s_uart_console_enabled != 0U);
}

void test_uart_note_command_parse_error(void)
{
    s_uart_command_parse_errors++;
}

void test_uart_note_command_started(void)
{
    s_uart_last_command_returned_to_prompt = 0U;
}

void test_uart_note_prompt_shown(void)
{
    s_uart_last_prompt_tick = HAL_GetTick();
    s_uart_last_command_returned_to_prompt = 1U;
}

void test_uart_print_rtt_info(void)
{
    test_rtt_stats_t stats;

    memset(&stats, 0, sizeof(stats));
    test_rtt_get_stats(&stats);

    test_uart_printf("RTT: enabled=%u initialized=%u boot=%lu cb=0x%08lX up=0x%08lX down=0x%08lX\r\n",
                     (unsigned int)stats.enabled,
                     (unsigned int)stats.initialized,
                     (unsigned long)stats.boot_sequence,
                     (unsigned long)stats.cb_addr,
                     (unsigned long)stats.up_buffer_addr,
                     (unsigned long)stats.down_buffer_addr);
    test_uart_printf("  Up: size=%lu wr=%lu rd=%lu free=%lu writes=%lu bytes=%lu dropped=%lu partial=%lu full=%lu\r\n",
                     (unsigned long)stats.up_size,
                     (unsigned long)stats.up_wr,
                     (unsigned long)stats.up_rd,
                     (unsigned long)stats.up_free,
                     (unsigned long)stats.write_calls,
                     (unsigned long)stats.bytes_written,
                     (unsigned long)stats.bytes_dropped,
                     (unsigned long)stats.partial_writes,
                     (unsigned long)stats.full_events);
    test_uart_printf("  Down: size=%lu wr=%lu rd=%lu pending=%lu reads=%lu bytes=%lu flushed=%lu\r\n",
                     (unsigned long)stats.down_size,
                     (unsigned long)stats.down_wr,
                     (unsigned long)stats.down_rd,
                     (unsigned long)stats.down_pending,
                     (unsigned long)stats.read_calls,
                     (unsigned long)stats.bytes_read,
                     (unsigned long)stats.flush_bytes);
    test_uart_printf("  Console: uart_enabled=%u line_overflows=%lu ignored_chars=%lu parse_errors=%lu crlf_tail=%lu last_prompt=%lu command_active=%u mode=nonblocking_drop\r\n",
                     test_uart_uart_console_enabled() ? 1U : 0U,
                     (unsigned long)s_uart_line_overflows,
                     (unsigned long)s_uart_ignored_chars,
                     (unsigned long)s_uart_command_parse_errors,
                     (unsigned long)s_uart_crlf_tail_consumed,
                     (unsigned long)s_uart_last_prompt_tick,
                     (s_uart_last_command_returned_to_prompt == 0U) ? 1U : 0U);
}

void test_uart_console_path(board_test_result_t *result)
{
    if (huart3.gState == HAL_UART_STATE_READY) {
        board_test_set_result(result,
                              "uart.console",
                              "USART3",
                              TEST_STATUS_PASS,
                              TEST_MODE_AUTOMATIC,
                              "USART3 ready (PD8/PD9)",
                              "Primary console UART initialized.",
                              "Keep BT_REG_ON low during console interaction unless running BT test.");
    } else {
        board_test_set_result(result,
                              "uart.console",
                              "USART3",
                              TEST_STATUS_FAIL,
                              TEST_MODE_AUTOMATIC,
                              "USART3 not ready",
                              "HAL UART state is not ready.",
                              "Check USART3 clock and pin mux (PD8/PD9).");
    }
}
