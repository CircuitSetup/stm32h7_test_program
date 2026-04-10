#ifndef AP6256_CONNECTIVITY_H
#define AP6256_CONNECTIVITY_H

#include "ap6256_assets.h"
#include "ap6256_connectivity_config.h"
#include "ap6256_driver.h"

#include <stddef.h>
#include <stdint.h>

typedef enum {
    AP6256_WIFI_SECURITY_UNKNOWN = 0,
    AP6256_WIFI_SECURITY_OPEN,
    AP6256_WIFI_SECURITY_WPA2_PSK
} ap6256_wifi_security_t;

typedef struct {
    ap6256_status_t transport_status;
    ap6256_wifi_diag_t transport_diag;
    uint8_t transport_present;
    uint8_t stack_ready;
    uint8_t owner_active;
    uint8_t assets_ready;
    uint8_t scan_results_count;
    char last_ssid[33];
    ap6256_wifi_security_t last_security;
    uint8_t password_captured;
    uint8_t dhcp_bound;
    int16_t last_rssi;
    char leased_ip[16];
    char leased_mask[16];
    char leased_gateway[16];
    uint32_t runtime_chip_id_raw;
    uint32_t runtime_ram_base_addr;
    uint32_t runtime_ram_size_bytes;
    uint32_t runtime_nvram_packed_len;
    uint32_t runtime_nvram_padded_len;
    uint32_t runtime_nvram_footer_word;
    uint32_t runtime_bus_stage;
    uint32_t runtime_sr_control1;
    uint32_t runtime_wlan_ioctrl;
    uint32_t runtime_wlan_resetctrl;
    uint32_t runtime_socram_ioctrl;
    uint32_t runtime_socram_resetctrl;
    uint8_t runtime_nvram_using_reference;
    uint8_t runtime_chip_clock_csr;
    char last_error[96];
    uint32_t last_update_ms;
} ap6256_wifi_state_t;

typedef struct {
    ap6256_status_t transport_status;
    ap6256_bt_diag_t transport_diag;
    uint8_t transport_ready;
    uint8_t stack_ready;
    uint8_t owner_active;
    uint8_t assets_ready;
    uint8_t patchram_loaded;
    uint8_t scan_results_count;
    uint8_t discovered_services_count;
    uint8_t connected;
    int8_t selected_device_rssi;
    char selected_device[24];
    char selected_name[32];
    char selected_service_uuid[40];
    char connection_state[24];
    char last_error[96];
    uint32_t last_update_ms;
} ap6256_bt_state_t;

void ap6256_connectivity_init(void);

ap6256_status_t ap6256_connectivity_probe_wifi_transport(ap6256_wifi_diag_t *diag);
ap6256_status_t ap6256_connectivity_probe_bt_transport(ap6256_bt_diag_t *diag);

int ap6256_connectivity_prompt_wifi_credentials(uint32_t timeout_ms);

const ap6256_wifi_state_t *ap6256_connectivity_get_wifi_state(void);
const ap6256_bt_state_t *ap6256_connectivity_get_bt_state(void);

const char *ap6256_connectivity_stack_note(void);
const char *ap6256_connectivity_stack_blocker(void);
const char *ap6256_connectivity_wifi_security_name(ap6256_wifi_security_t security);
uint8_t ap6256_connectivity_full_stack_ready(void);

void ap6256_connectivity_set_wifi_note(const char *text);
void ap6256_connectivity_set_wifi_profile(const char *ssid,
                                          ap6256_wifi_security_t security,
                                          uint8_t password_captured);
void ap6256_connectivity_set_wifi_runtime(uint8_t stack_ready,
                                          uint8_t owner_active,
                                          uint8_t scan_results_count,
                                          int16_t rssi);
void ap6256_connectivity_set_wifi_ip(const char *ip,
                                     const char *mask,
                                     const char *gateway,
                                     uint8_t dhcp_bound);
void ap6256_connectivity_set_wifi_compat(uint32_t chip_id_raw,
                                         uint32_t ram_base_addr,
                                         uint32_t ram_size_bytes,
                                         uint32_t nvram_packed_len,
                                         uint32_t nvram_padded_len,
                                         uint32_t nvram_footer_word,
                                         uint8_t nvram_using_reference,
                                         uint32_t bus_stage,
                                         uint8_t chip_clock_csr,
                                         uint32_t sr_control1,
                                         uint32_t wlan_ioctrl,
                                         uint32_t wlan_resetctrl,
                                         uint32_t socram_ioctrl,
                                         uint32_t socram_resetctrl);

void ap6256_connectivity_set_bt_note(const char *text);
void ap6256_connectivity_set_bt_runtime(uint8_t stack_ready,
                                        uint8_t owner_active,
                                        uint8_t patchram_loaded,
                                        uint8_t scan_results_count,
                                        uint8_t discovered_services_count,
                                        uint8_t connected,
                                        const char *connection_state);
void ap6256_connectivity_set_bt_selection(const char *address,
                                          const char *name,
                                          int8_t rssi,
                                          const char *service_uuid);

void ap6256_connectivity_print_wifi_info(void);
void ap6256_connectivity_print_bt_info(void);

#endif
