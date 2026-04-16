#ifndef TEST_RTT_H
#define TEST_RTT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t enabled;
    uint8_t initialized;
    uintptr_t cb_addr;
    uintptr_t up_buffer_addr;
    uintptr_t down_buffer_addr;
    uint32_t up_size;
    uint32_t down_size;
    uint32_t up_wr;
    uint32_t up_rd;
    uint32_t up_free;
    uint32_t down_wr;
    uint32_t down_rd;
    uint32_t down_pending;
    uint32_t write_calls;
    uint32_t bytes_written;
    uint32_t bytes_dropped;
    uint32_t partial_writes;
    uint32_t full_events;
    uint32_t read_calls;
    uint32_t bytes_read;
    uint32_t flush_bytes;
    uint32_t boot_sequence;
} test_rtt_stats_t;

void test_rtt_init(void);
bool test_rtt_enabled(void);
size_t test_rtt_write(const uint8_t *data, size_t len);
size_t test_rtt_read(uint8_t *data, size_t len);
bool test_rtt_consume_line_tail(void);
void test_rtt_flush_rx(void);
void test_rtt_get_stats(test_rtt_stats_t *stats);
uint32_t test_rtt_boot_sequence(void);

#endif
