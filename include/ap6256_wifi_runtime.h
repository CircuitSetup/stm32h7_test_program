#ifndef AP6256_WIFI_RUNTIME_H
#define AP6256_WIFI_RUNTIME_H

#include "ap6256_driver.h"

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t scan_results_count;
    uint8_t connected;
    int16_t selected_rssi;
    char selected_ssid[33];
    char leased_ip[16];
    char leased_mask[16];
    char leased_gateway[16];
} ap6256_wifi_runtime_summary_t;

void ap6256_wifi_runtime_poll(void);
void ap6256_wifi_runtime_suspend(void);
uint8_t ap6256_wifi_runtime_has_cached_profile(void);
void ap6256_wifi_runtime_set_reference_nvram(uint8_t enable);
uint8_t ap6256_wifi_runtime_reference_nvram_enabled(void);
ap6256_status_t ap6256_wifi_runtime_run_interactive(ap6256_wifi_runtime_summary_t *summary,
                                                    char *detail,
                                                    size_t detail_len);
ap6256_status_t ap6256_wifi_runtime_run_cached(ap6256_wifi_runtime_summary_t *summary,
                                               char *detail,
                                               size_t detail_len);

#endif /* AP6256_WIFI_RUNTIME_H */
