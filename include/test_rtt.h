#ifndef TEST_RTT_H
#define TEST_RTT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void test_rtt_init(void);
bool test_rtt_enabled(void);
size_t test_rtt_write(const uint8_t *data, size_t len);
size_t test_rtt_read(uint8_t *data, size_t len);

#endif
