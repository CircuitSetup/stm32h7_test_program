#include "test_rtt.h"

#include "main.h"

#include <string.h>

#if BOARD_ENABLE_SEGGER_RTT

#define RTT_MAX_NUM_UP_BUFFERS 1U
#define RTT_MAX_NUM_DOWN_BUFFERS 1U
#define RTT_MODE_NO_BLOCK_SKIP 0U

typedef struct {
    const char *sName;
    char *pBuffer;
    unsigned int SizeOfBuffer;
    unsigned int WrOff;
    volatile unsigned int RdOff;
    unsigned int Flags;
} rtt_buffer_up_t;

typedef struct {
    const char *sName;
    char *pBuffer;
    unsigned int SizeOfBuffer;
    volatile unsigned int WrOff;
    unsigned int RdOff;
    unsigned int Flags;
} rtt_buffer_down_t;

typedef struct {
    char acID[16];
    int MaxNumUpBuffers;
    int MaxNumDownBuffers;
    rtt_buffer_up_t aUp[RTT_MAX_NUM_UP_BUFFERS];
    rtt_buffer_down_t aDown[RTT_MAX_NUM_DOWN_BUFFERS];
} rtt_cb_t;

static char s_rtt_up_buffer[BOARD_RTT_UP_BUFFER_SIZE] __attribute__((section(".rtt"), aligned(32)));
static char s_rtt_down_buffer[BOARD_RTT_DOWN_BUFFER_SIZE] __attribute__((section(".rtt"), aligned(32)));

static rtt_cb_t _SEGGER_RTT __attribute__((used, section(".rtt"), aligned(32)));
#define s_rtt_cb _SEGGER_RTT
static uint8_t s_rtt_initialized = 0U;
static test_rtt_stats_t s_rtt_stats;
static uint32_t s_rtt_boot_sequence;

static uint32_t rtt_ring_pending(unsigned int wr, unsigned int rd, unsigned int size)
{
    if (size == 0U) {
        return 0U;
    }

    if (wr >= rd) {
        return wr - rd;
    }
    return size - (rd - wr);
}

static uint32_t rtt_ring_free(unsigned int wr, unsigned int rd, unsigned int size)
{
    uint32_t pending;

    if (size <= 1U) {
        return 0U;
    }

    pending = rtt_ring_pending(wr, rd, size);
    if (pending >= size) {
        return 0U;
    }
    return (size - pending) - 1U;
}

static void rtt_cache_clean(const void *addr, size_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    uintptr_t start;
    uintptr_t end;

    if ((len == 0U) || ((SCB->CCR & SCB_CCR_DC_Msk) == 0U)) {
        return;
    }

    start = ((uintptr_t)addr) & ~(uintptr_t)31U;
    end = (((uintptr_t)addr + len) + 31U) & ~(uintptr_t)31U;
    SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
#else
    (void)addr;
    (void)len;
#endif
}

static void rtt_cache_invalidate(const void *addr, size_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    uintptr_t start;
    uintptr_t end;

    if ((len == 0U) || ((SCB->CCR & SCB_CCR_DC_Msk) == 0U)) {
        return;
    }

    start = ((uintptr_t)addr) & ~(uintptr_t)31U;
    end = (((uintptr_t)addr + len) + 31U) & ~(uintptr_t)31U;
    SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
#else
    (void)addr;
    (void)len;
#endif
}

void test_rtt_init(void)
{
    if (s_rtt_initialized != 0U) {
        return;
    }

    memset(&s_rtt_cb, 0, sizeof(s_rtt_cb));
    memset(s_rtt_up_buffer, 0, sizeof(s_rtt_up_buffer));
    memset(s_rtt_down_buffer, 0, sizeof(s_rtt_down_buffer));
    (void)memcpy(s_rtt_cb.acID, "SEGGER RTT", 10U);

    s_rtt_cb.MaxNumUpBuffers = (int)RTT_MAX_NUM_UP_BUFFERS;
    s_rtt_cb.MaxNumDownBuffers = (int)RTT_MAX_NUM_DOWN_BUFFERS;

    s_rtt_cb.aUp[0].sName = "Terminal";
    s_rtt_cb.aUp[0].pBuffer = s_rtt_up_buffer;
    s_rtt_cb.aUp[0].SizeOfBuffer = BOARD_RTT_UP_BUFFER_SIZE;
    s_rtt_cb.aUp[0].Flags = RTT_MODE_NO_BLOCK_SKIP;

    s_rtt_cb.aDown[0].sName = "Terminal";
    s_rtt_cb.aDown[0].pBuffer = s_rtt_down_buffer;
    s_rtt_cb.aDown[0].SizeOfBuffer = BOARD_RTT_DOWN_BUFFER_SIZE;
    s_rtt_cb.aDown[0].Flags = RTT_MODE_NO_BLOCK_SKIP;

    rtt_cache_clean(&s_rtt_cb, sizeof(s_rtt_cb));
    rtt_cache_clean(s_rtt_up_buffer, sizeof(s_rtt_up_buffer));
    rtt_cache_clean(s_rtt_down_buffer, sizeof(s_rtt_down_buffer));
    __DMB();

    s_rtt_boot_sequence++;
    memset(&s_rtt_stats, 0, sizeof(s_rtt_stats));
    s_rtt_stats.enabled = 1U;
    s_rtt_stats.initialized = 1U;
    s_rtt_stats.cb_addr = (uintptr_t)&s_rtt_cb;
    s_rtt_stats.up_buffer_addr = (uintptr_t)s_rtt_up_buffer;
    s_rtt_stats.down_buffer_addr = (uintptr_t)s_rtt_down_buffer;
    s_rtt_stats.up_size = BOARD_RTT_UP_BUFFER_SIZE;
    s_rtt_stats.down_size = BOARD_RTT_DOWN_BUFFER_SIZE;
    s_rtt_stats.boot_sequence = s_rtt_boot_sequence;
    s_rtt_initialized = 1U;
}

bool test_rtt_enabled(void)
{
    return true;
}

size_t test_rtt_write(const uint8_t *data, size_t len)
{
    rtt_buffer_up_t *up;
    unsigned int wr;
    unsigned int next_wr;
    unsigned int rd;
    unsigned int start_wr;
    size_t written = 0U;

    if ((data == NULL) || (len == 0U)) {
        return 0U;
    }

    if (s_rtt_initialized == 0U) {
        test_rtt_init();
    }

    up = &s_rtt_cb.aUp[0];
    wr = up->WrOff;
    start_wr = wr;
    s_rtt_stats.write_calls++;

    while (written < len) {
        rtt_cache_invalidate((const void *)&up->RdOff, sizeof(up->RdOff));
        rd = up->RdOff;

        next_wr = wr + 1U;
        if (next_wr >= up->SizeOfBuffer) {
            next_wr = 0U;
        }

        if (next_wr == rd) {
            s_rtt_stats.full_events++;
            break;
        }

        up->pBuffer[wr] = (char)data[written];
        wr = next_wr;
        written++;
    }

    if (written > 0U) {
        if (wr > start_wr) {
            rtt_cache_clean(&up->pBuffer[start_wr], (size_t)(wr - start_wr));
        } else {
            rtt_cache_clean(&up->pBuffer[start_wr], (size_t)(up->SizeOfBuffer - start_wr));
            if (wr > 0U) {
                rtt_cache_clean(&up->pBuffer[0], (size_t)wr);
            }
        }

        __DMB();
        up->WrOff = wr;
        rtt_cache_clean(&up->WrOff, sizeof(up->WrOff));
    }

    s_rtt_stats.bytes_written += (uint32_t)written;
    if (written < len) {
        s_rtt_stats.partial_writes++;
        s_rtt_stats.bytes_dropped += (uint32_t)(len - written);
    }

    return written;
}

size_t test_rtt_read(uint8_t *data, size_t len)
{
    rtt_buffer_down_t *down;
    unsigned int wr;
    unsigned int rd;
    size_t read_len = 0U;

    if ((data == NULL) || (len == 0U)) {
        return 0U;
    }

    if (s_rtt_initialized == 0U) {
        test_rtt_init();
    }

    s_rtt_stats.read_calls++;
    down = &s_rtt_cb.aDown[0];
    rtt_cache_invalidate((const void *)&down->RdOff, sizeof(down->RdOff));
    rd = down->RdOff;

    while (read_len < len) {
        rtt_cache_invalidate((const void *)&down->WrOff, sizeof(down->WrOff));
        wr = down->WrOff;

        if (rd == wr) {
            break;
        }

        rtt_cache_invalidate(&down->pBuffer[rd], 1U);
        data[read_len] = (uint8_t)down->pBuffer[rd];
        rd++;
        if (rd >= down->SizeOfBuffer) {
            rd = 0U;
        }
        read_len++;
    }

    if (read_len > 0U) {
        down->RdOff = rd;
        rtt_cache_clean(&down->RdOff, sizeof(down->RdOff));
        s_rtt_stats.bytes_read += (uint32_t)read_len;
    }

    return read_len;
}

bool test_rtt_consume_line_tail(void)
{
    rtt_buffer_down_t *down;
    unsigned int wr;
    unsigned int rd;
    uint8_t ch;

    if (s_rtt_initialized == 0U) {
        test_rtt_init();
    }

    down = &s_rtt_cb.aDown[0];
    rtt_cache_invalidate((const void *)&down->WrOff, sizeof(down->WrOff));
    wr = down->WrOff;
    rd = down->RdOff;

    if (rd == wr) {
        return false;
    }

    rtt_cache_invalidate(&down->pBuffer[rd], 1U);
    ch = (uint8_t)down->pBuffer[rd];
    if ((ch != '\r') && (ch != '\n')) {
        return false;
    }

    rd++;
    if (rd >= down->SizeOfBuffer) {
        rd = 0U;
    }

    down->RdOff = rd;
    rtt_cache_clean(&down->RdOff, sizeof(down->RdOff));
    s_rtt_stats.bytes_read++;
    return true;
}

void test_rtt_flush_rx(void)
{
    uint8_t ch;

    while (test_rtt_read(&ch, 1U) == 1U) {
        s_rtt_stats.flush_bytes++;
    }
}

void test_rtt_get_stats(test_rtt_stats_t *stats)
{
    rtt_buffer_up_t *up;
    rtt_buffer_down_t *down;
    unsigned int up_wr = 0U;
    unsigned int up_rd = 0U;
    unsigned int down_wr = 0U;
    unsigned int down_rd = 0U;

    if (stats == NULL) {
        return;
    }

    if (s_rtt_initialized == 0U) {
        test_rtt_init();
    }

    up = &s_rtt_cb.aUp[0];
    down = &s_rtt_cb.aDown[0];

    rtt_cache_invalidate((const void *)&up->RdOff, sizeof(up->RdOff));
    rtt_cache_invalidate((const void *)&down->WrOff, sizeof(down->WrOff));
    up_wr = up->WrOff;
    up_rd = up->RdOff;
    down_wr = down->WrOff;
    down_rd = down->RdOff;

    *stats = s_rtt_stats;
    stats->initialized = s_rtt_initialized;
    stats->boot_sequence = s_rtt_boot_sequence;
    stats->cb_addr = (uintptr_t)&s_rtt_cb;
    stats->up_buffer_addr = (uintptr_t)s_rtt_up_buffer;
    stats->down_buffer_addr = (uintptr_t)s_rtt_down_buffer;
    stats->up_size = up->SizeOfBuffer;
    stats->down_size = down->SizeOfBuffer;
    stats->up_wr = up_wr;
    stats->up_rd = up_rd;
    stats->up_free = rtt_ring_free(up_wr, up_rd, up->SizeOfBuffer);
    stats->down_wr = down_wr;
    stats->down_rd = down_rd;
    stats->down_pending = rtt_ring_pending(down_wr, down_rd, down->SizeOfBuffer);
}

uint32_t test_rtt_boot_sequence(void)
{
    return s_rtt_boot_sequence;
}

#else

void test_rtt_init(void)
{
}

bool test_rtt_enabled(void)
{
    return false;
}

size_t test_rtt_write(const uint8_t *data, size_t len)
{
    (void)data;
    (void)len;
    return 0U;
}

size_t test_rtt_read(uint8_t *data, size_t len)
{
    (void)data;
    (void)len;
    return 0U;
}

bool test_rtt_consume_line_tail(void)
{
    return false;
}

void test_rtt_flush_rx(void)
{
}

void test_rtt_get_stats(test_rtt_stats_t *stats)
{
    if (stats != NULL) {
        memset(stats, 0, sizeof(*stats));
    }
}

uint32_t test_rtt_boot_sequence(void)
{
    return 0U;
}

#endif
