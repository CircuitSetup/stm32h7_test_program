#include "network_manager.h"

#include "ap6256_bt_runtime.h"
#include "ap6256_wifi_runtime.h"
#include "cmsis_os2.h"
#include "main.h"
#include "network_ethernet.h"
#include "test_uart.h"

#include <string.h>

static network_owner_t s_network_owner = NETWORK_OWNER_IDLE;
static osMutexId_t s_network_mutex;
static network_manager_stats_t s_network_stats;
static osThreadId_t s_wifi_worker_thread;
static osThreadId_t s_bt_worker_thread;

static const osThreadAttr_t s_wifi_worker_attr = {
    .name = "wifi_task",
    .priority = osPriorityBelowNormal,
    .stack_size = 6144U
};

static const osThreadAttr_t s_bt_worker_attr = {
    .name = "bt_task",
    .priority = osPriorityBelowNormal,
    .stack_size = 6144U
};

static void network_manager_wifi_worker(void *argument)
{
    (void)argument;

    for (;;) {
        ap6256_wifi_runtime_poll();
        osDelay(10U);
    }
}

static void network_manager_bt_worker(void *argument)
{
    (void)argument;

    for (;;) {
        ap6256_bt_runtime_poll();
        osDelay(1U);
    }
}

static bool network_manager_start_wifi_worker(void)
{
    if (s_wifi_worker_thread != NULL) {
        return true;
    }

    if (osKernelGetState() != osKernelRunning) {
        return false;
    }

    s_wifi_worker_thread = osThreadNew(network_manager_wifi_worker, NULL, &s_wifi_worker_attr);
    return (s_wifi_worker_thread != NULL);
}

static bool network_manager_start_bt_worker(void)
{
    if (s_bt_worker_thread != NULL) {
        return true;
    }

    if (osKernelGetState() != osKernelRunning) {
        return false;
    }

    s_bt_worker_thread = osThreadNew(network_manager_bt_worker, NULL, &s_bt_worker_attr);
    return (s_bt_worker_thread != NULL);
}

static void network_manager_stop_wifi_worker(void)
{
    if (s_wifi_worker_thread == NULL) {
        return;
    }

    (void)osThreadTerminate(s_wifi_worker_thread);
    s_wifi_worker_thread = NULL;
}

static void network_manager_stop_bt_worker(void)
{
    if (s_bt_worker_thread == NULL) {
        return;
    }

    (void)osThreadTerminate(s_bt_worker_thread);
    s_bt_worker_thread = NULL;
}

static bool network_manager_apply_owner(network_owner_t owner)
{
    network_owner_t previous_owner = s_network_owner;

    if (owner == s_network_owner) {
        return true;
    }

    switch (owner) {
    case NETWORK_OWNER_WIFI:
        if (!network_manager_start_wifi_worker()) {
            return false;
        }
        break;
    case NETWORK_OWNER_BLUETOOTH:
        if (!network_manager_start_bt_worker()) {
            return false;
        }
        break;
    case NETWORK_OWNER_ETHERNET:
    case NETWORK_OWNER_IDLE:
    default:
        break;
    }

    if ((previous_owner == NETWORK_OWNER_WIFI) && (owner != NETWORK_OWNER_WIFI)) {
        ap6256_wifi_runtime_suspend();
        network_manager_stop_wifi_worker();
    }

    if ((previous_owner == NETWORK_OWNER_BLUETOOTH) && (owner != NETWORK_OWNER_BLUETOOTH)) {
        ap6256_bt_runtime_suspend();
        network_manager_stop_bt_worker();
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
    return true;
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
    s_wifi_worker_thread = NULL;
    s_bt_worker_thread = NULL;
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

    if (!network_manager_apply_owner(owner)) {
        s_network_stats.acquire_failures++;
        if (s_network_mutex != NULL) {
            (void)osMutexRelease(s_network_mutex);
        }
        return false;
    }

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
    test_uart_printf("  wifi_worker=%u bt_worker=%u\r\n",
                     (s_wifi_worker_thread != NULL) ? 1U : 0U,
                     (s_bt_worker_thread != NULL) ? 1U : 0U);
}
