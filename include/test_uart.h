#ifndef TEST_UART_H
#define TEST_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void test_uart_init(void);
void test_uart_write(const uint8_t *data, size_t len);
void test_uart_write_str(const char *text);
void test_uart_printf(const char *fmt, ...);
int test_uart_read_line(char *buffer, size_t buffer_len, uint32_t timeout_ms);
int test_uart_read_line_masked(char *buffer, size_t buffer_len, uint32_t timeout_ms);
bool test_uart_read_bytes(uint8_t *buffer, size_t len, uint32_t timeout_ms);
void test_uart_flush_rx(void);
void test_uart_set_uart_console_enabled(bool enabled);
bool test_uart_uart_console_enabled(void);

#endif
