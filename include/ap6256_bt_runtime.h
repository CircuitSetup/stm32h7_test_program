#ifndef AP6256_BT_RUNTIME_H
#define AP6256_BT_RUNTIME_H

#include "ap6256_driver.h"

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t patchram_loaded;
    uint8_t device_count;
    uint8_t service_count;
    uint8_t connected;
    int8_t selected_rssi;
    char selected_address[24];
    char selected_name[32];
    char selected_service_uuid[40];
} ap6256_bt_runtime_summary_t;

uint8_t ap6256_bt_runtime_has_cached_selection(void);
void ap6256_bt_runtime_poll(void);
ap6256_status_t ap6256_bt_runtime_run_interactive(ap6256_bt_runtime_summary_t *summary,
                                                  char *detail,
                                                  size_t detail_len);
ap6256_status_t ap6256_bt_runtime_run_cached(ap6256_bt_runtime_summary_t *summary,
                                             char *detail,
                                             size_t detail_len);

#endif /* AP6256_BT_RUNTIME_H */
