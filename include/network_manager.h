#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    NETWORK_OWNER_IDLE = 0,
    NETWORK_OWNER_ETHERNET,
    NETWORK_OWNER_WIFI,
    NETWORK_OWNER_BLUETOOTH
} network_owner_t;

typedef struct {
    uint32_t acquire_attempts;
    uint32_t acquire_failures;
    uint32_t transitions;
    uint32_t releases;
    network_owner_t owner;
} network_manager_stats_t;

void network_manager_init(void);
bool network_manager_acquire(network_owner_t owner, uint32_t timeout_ms);
void network_manager_release(network_owner_t owner);
void network_manager_request(network_owner_t owner);
network_owner_t network_manager_get_owner(void);
bool network_manager_console_uart_allowed(void);
const char *network_manager_owner_name(network_owner_t owner);
void network_manager_get_stats(network_manager_stats_t *stats);
void network_manager_print_info(void);

#ifdef __cplusplus
}
#endif

#endif /* NETWORK_MANAGER_H */
