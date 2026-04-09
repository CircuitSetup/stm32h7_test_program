#include "network_manager.h"

#include "ap6256_wifi_runtime.h"
#include "cmsis_os2.h"
#include "main.h"
#include "network_ethernet.h"
#include "test_uart.h"

#include <string.h>

static network_owner_t s_network_owner = NETWORK_OWNER_IDLE;
static osMutexId_t s_network_mutex;
static network_manager_stats_t s_network_stats;

static void network_manager_apply_owner(network_owner_t owner)
{
    network_owner_t previous_owner = s_network_owner;

    if (owner == s_network_owner) {
        return;
    }

    if (previous_owner == NETWORK_OWNER_WIFI) {
        ap6256_wifi_runtime_suspend();
    }

    switch (owner) {
    case NETWORK_OWNER_WIFI:
        network_ethernet_suspend();
        break;
    case NETWORK_OWNER_BLUETOOTH:
        network_ethernet_suspend();
        break;
    case NETWORK_OWNER_ETHERNET:
        network_ethernet_resume();
        break;
    case NETWORK_OWNER_IDLE:
    default:
        break;
    }

    if (owner != s_network_owner) {
        s_network_stats.transitions++;
    }

    s_network_owner = owner;
    s_network_stats.owner = owner;
    test_uart_set_uart_console_enabled(owner != NETWORK_OWNER_BLUETOOTH);
}

const char *network_manager_owner_name(network_owner_t owner)
{
    switch (owner) {
    case NETWORK_OWNER_ETHERNET:
        return "ETHERNET";
    case NETWORK_OWNER_WIFI:
        return "WIFI";
    case NETWORK_OWNER_BLUETOOTH:
        return "BT";
    case NETWORK_OWNER_IDLE:
    default:
        return "IDLE";
    }
}

void network_manager_init(void)
{
    static const osMutexAttr_t mutex_attr = {
        .name = "network_mgr"
    };

    s_network_owner = NETWORK_OWNER_IDLE;
    memset(&s_network_stats, 0, sizeof(s_network_stats));
    s_network_stats.owner = NETWORK_OWNER_IDLE;
    if ((osKernelGetState() == osKernelRunning) || (osKernelGetState() == osKernelReady)) {
        if (s_network_mutex == NULL) {
            s_network_mutex = osMutexNew(&mutex_attr);
        }
    }
    test_uart_set_uart_console_enabled(true);
}

bool network_manager_acquire(network_owner_t owner, uint32_t timeout_ms)
{
    s_network_stats.acquire_attempts++;

    if (s_network_mutex != NULL) {
        if (osMutexAcquire(s_network_mutex, timeout_ms) != osOK) {
            s_network_stats.acquire_failures++;
            return false;
        }
    }

    network_manager_apply_owner(owner);

    if (s_network_mutex != NULL) {
        (void)osMutexRelease(s_network_mutex);
    }

    return true;
}

void network_manager_release(network_owner_t owner)
{
    if (s_network_mutex != NULL) {
        if (osMutexAcquire(s_network_mutex, 250U) != osOK) {
            return;
        }
    }

    if (s_network_owner == owner) {
        s_network_stats.releases++;
        network_manager_apply_owner(NETWORK_OWNER_IDLE);
    }

    if (s_network_mutex != NULL) {
        (void)osMutexRelease(s_network_mutex);
    }
}

void network_manager_request(network_owner_t owner)
{
    (void)network_manager_acquire(owner, 30000U);
}

network_owner_t network_manager_get_owner(void)
{
    return s_network_owner;
}

bool network_manager_console_uart_allowed(void)
{
    return (s_network_owner != NETWORK_OWNER_BLUETOOTH);
}

void network_manager_get_stats(network_manager_stats_t *stats)
{
    if (stats != NULL) {
        *stats = s_network_stats;
    }
}

void network_manager_print_info(void)
{
    test_uart_printf("Radio owner: %s\r\n", network_manager_owner_name(s_network_owner));
    test_uart_printf("  transitions=%lu acquire_attempts=%lu acquire_failures=%lu releases=%lu uart_console=%u\r\n",
                     (unsigned long)s_network_stats.transitions,
                     (unsigned long)s_network_stats.acquire_attempts,
                     (unsigned long)s_network_stats.acquire_failures,
                     (unsigned long)s_network_stats.releases,
                     network_manager_console_uart_allowed() ? 1U : 0U);
}
