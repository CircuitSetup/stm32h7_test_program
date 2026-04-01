#include "test_rtt.h"

#include "main.h"

#include <string.h>

#if BOARD_ENABLE_SEGGER_RTT

#define RTT_MAX_NUM_UP_BUFFERS 1U
#define RTT_MAX_NUM_DOWN_BUFFERS 1U
#define RTT_MODE_NO_BLOCK_SKIP 0U
#define RTT_WRITE_WAIT_MS 5000U

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

static char s_rtt_up_buffer[BOARD_RTT_UP_BUFFER_SIZE];
static char s_rtt_down_buffer[BOARD_RTT_DOWN_BUFFER_SIZE];

static rtt_cb_t s_rtt_cb __attribute__((used));
static uint8_t s_rtt_initialized = 0U;

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
    uint32_t wait_start;
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
    wait_start = HAL_GetTick();

    while (written < len) {
        rtt_cache_invalidate((const void *)&up->RdOff, sizeof(up->RdOff));
        rd = up->RdOff;

        next_wr = wr + 1U;
        if (next_wr >= up->SizeOfBuffer) {
            next_wr = 0U;
        }

        if (next_wr == rd) {
            if ((HAL_GetTick() - wait_start) >= RTT_WRITE_WAIT_MS) {
                break; /* Timeout waiting for host reader. */
            }
            HAL_Delay(1U);
            continue;
        }

        up->pBuffer[wr] = (char)data[written];
        wr = next_wr;
        written++;
        wait_start = HAL_GetTick();
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

    down = &s_rtt_cb.aDown[0];
    rd = down->RdOff;

    while (read_len < len) {
        rtt_cache_invalidate((const void *)&down->WrOff, sizeof(down->WrOff));
        wr = down->WrOff;

        if (rd == wr) {
            break;
        }

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
    }

    return read_len;
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

#endif
