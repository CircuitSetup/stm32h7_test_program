#ifndef AP6256_BT_PERIPHERAL_RUNTIME_H
#define AP6256_BT_PERIPHERAL_RUNTIME_H

#include "ap6256_driver.h"

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t patchram_loaded;
    uint8_t advertising_started;
    uint8_t connected;
    uint32_t read_count;
    char peer_address[24];
    char local_name[32];
    char service_uuid[40];
} ap6256_bt_peripheral_runtime_summary_t;

void ap6256_bt_peripheral_runtime_suspend(void);
ap6256_status_t ap6256_bt_peripheral_runtime_run_interactive(ap6256_bt_peripheral_runtime_summary_t *summary,
                                                             char *detail,
                                                             size_t detail_len);

#endif /* AP6256_BT_PERIPHERAL_RUNTIME_H */
