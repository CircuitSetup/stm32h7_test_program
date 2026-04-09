#include "ap6256_connectivity.h"

#include "ap6256_bt_runtime.h"
#include "ap6256_wifi_runtime.h"
#include "network_manager.h"
#include "test_uart.h"

#include <stdio.h>
#include <string.h>

static const char s_stack_note[] =
    "AP6256 BCM43456 assets are embedded; CYW43-backed Wi-Fi and BTstack-backed Bluetooth runtimes are active.";
static const char s_stack_blocker[] =
    "none";

static ap6256_wifi_state_t s_wifi_state;
static ap6256_bt_state_t s_bt_state;

static void ap6256_connectivity_print_asset_line(const char *label,
                                                 const ap6256_embedded_asset_t *asset)
{
    if ((label == NULL) || (asset == NULL)) {
        return;
    }

    test_uart_printf("  %s: %s size=%lu sha256=%s\r\n",
                     label,
                     asset->filename,
                     (unsigned long)asset->size,
                     asset->sha256);
}

static void ap6256_connectivity_copy_text(char *dst, size_t dst_len, const char *src)
{
    if ((dst == NULL) || (dst_len == 0U)) {
        return;
    }

    if (src == NULL) {
        dst[0] = '\0';
        return;
    }

    (void)snprintf(dst, dst_len, "%s", src);
}

static void ap6256_connectivity_set_wifi_error(const char *text)
{
    ap6256_connectivity_copy_text(s_wifi_state.last_error,
                                  sizeof(s_wifi_state.last_error),
                                  text);
}

static void ap6256_connectivity_set_bt_error(const char *text)
{
    ap6256_connectivity_copy_text(s_bt_state.last_error,
                                  sizeof(s_bt_state.last_error),
                                  text);
}

void ap6256_connectivity_init(void)
{
    memset(&s_wifi_state, 0, sizeof(s_wifi_state));
    memset(&s_bt_state, 0, sizeof(s_bt_state));
    s_wifi_state.transport_status = AP6256_STATUS_TIMEOUT;
    s_bt_state.transport_status = AP6256_STATUS_TIMEOUT;
    s_wifi_state.last_security = AP6256_WIFI_SECURITY_UNKNOWN;
    s_wifi_state.assets_ready = ap6256_assets_ready();
    s_bt_state.assets_ready = s_wifi_state.assets_ready;
    ap6256_connectivity_copy_text(s_bt_state.connection_state,
                                  sizeof(s_bt_state.connection_state),
                                  "idle");
    ap6256_connectivity_set_wifi_error(s_stack_note);
    ap6256_connectivity_set_bt_error(s_stack_note);
}

ap6256_status_t ap6256_connectivity_probe_wifi_transport(ap6256_wifi_diag_t *diag)
{
    ap6256_status_t st;

    memset(&s_wifi_state.transport_diag, 0, sizeof(s_wifi_state.transport_diag));
    st = ap6256_wifi_transport_probe(&s_wifi_state.transport_diag);

    s_wifi_state.transport_status = st;
    s_wifi_state.transport_present = ((st == AP6256_STATUS_OK) &&
                                      (s_wifi_state.transport_diag.cmd5_ready != 0U)) ? 1U : 0U;
    s_wifi_state.assets_ready = ap6256_assets_ready();
    s_wifi_state.last_update_ms = HAL_GetTick();

    if (s_wifi_state.transport_present != 0U) {
        ap6256_connectivity_set_wifi_error(s_stack_note);
    } else {
        ap6256_connectivity_set_wifi_error("AP6256 Wi-Fi transport probe did not reach CMD5 ready and required CCCR reads.");
    }

    if (diag != NULL) {
        *diag = s_wifi_state.transport_diag;
    }

    return st;
}

ap6256_status_t ap6256_connectivity_probe_bt_transport(ap6256_bt_diag_t *diag)
{
    ap6256_status_t st;

    memset(&s_bt_state.transport_diag, 0, sizeof(s_bt_state.transport_diag));
    st = ap6256_bt_hci_probe(&s_bt_state.transport_diag);

    s_bt_state.transport_status = st;
    s_bt_state.transport_ready = ((st == AP6256_STATUS_OK) &&
                                  (s_bt_state.transport_diag.reset_event_seen != 0U) &&
                                  (s_bt_state.transport_diag.version_event_seen != 0U) &&
                                  (s_bt_state.transport_diag.reset_status == 0x00U) &&
                                  (s_bt_state.transport_diag.version_status == 0x00U)) ? 1U : 0U;
    s_bt_state.assets_ready = ap6256_assets_ready();
    s_bt_state.last_update_ms = HAL_GetTick();

    if (s_bt_state.transport_ready != 0U) {
        ap6256_connectivity_set_bt_error(s_stack_note);
    } else {
        ap6256_connectivity_set_bt_error("AP6256 Bluetooth HCI transport probe did not reach command-complete readiness.");
    }

    if (diag != NULL) {
        *diag = s_bt_state.transport_diag;
    }

    return st;
}

int ap6256_connectivity_prompt_wifi_credentials(uint32_t timeout_ms)
{
    char ssid[sizeof(s_wifi_state.last_ssid)];
    char password[65];
    int ssid_len;
    int password_len;

    memset(ssid, 0, sizeof(ssid));
    memset(password, 0, sizeof(password));

    test_uart_write_str("Wi-Fi SSID: ");
    ssid_len = test_uart_read_line(ssid, sizeof(ssid), timeout_ms);
    if (ssid_len <= 0) {
        ap6256_connectivity_set_wifi_error("Timed out waiting for Wi-Fi SSID input.");
        return ssid_len;
    }

    test_uart_write_str("Wi-Fi password (leave blank for open network): ");
    password_len = test_uart_read_line_masked(password, sizeof(password), timeout_ms);
    if (password_len < 0) {
        ap6256_connectivity_set_wifi_error("Invalid Wi-Fi password input state.");
        return password_len;
    }

    ap6256_connectivity_copy_text(s_wifi_state.last_ssid,
                                  sizeof(s_wifi_state.last_ssid),
                                  ssid);
    s_wifi_state.password_captured = (password_len > 0) ? 1U : 0U;
    s_wifi_state.last_security = (password_len > 0) ? AP6256_WIFI_SECURITY_WPA2_PSK
                                                    : AP6256_WIFI_SECURITY_OPEN;
    ap6256_connectivity_set_wifi_error(s_stack_note);

    return ssid_len;
}

const ap6256_wifi_state_t *ap6256_connectivity_get_wifi_state(void)
{
    return &s_wifi_state;
}

const ap6256_bt_state_t *ap6256_connectivity_get_bt_state(void)
{
    return &s_bt_state;
}

const char *ap6256_connectivity_stack_note(void)
{
    return s_stack_note;
}

const char *ap6256_connectivity_stack_blocker(void)
{
    return s_stack_blocker;
}

uint8_t ap6256_connectivity_full_stack_ready(void)
{
    return (uint8_t)((s_wifi_state.stack_ready != 0U) && (s_bt_state.stack_ready != 0U));
}

void ap6256_connectivity_set_wifi_note(const char *text)
{
    ap6256_connectivity_set_wifi_error(text);
    s_wifi_state.last_update_ms = HAL_GetTick();
}

void ap6256_connectivity_set_wifi_profile(const char *ssid,
                                          ap6256_wifi_security_t security,
                                          uint8_t password_captured)
{
    ap6256_connectivity_copy_text(s_wifi_state.last_ssid,
                                  sizeof(s_wifi_state.last_ssid),
                                  ssid);
    s_wifi_state.last_security = security;
    s_wifi_state.password_captured = password_captured;
    s_wifi_state.last_update_ms = HAL_GetTick();
}

void ap6256_connectivity_set_wifi_runtime(uint8_t stack_ready,
                                          uint8_t owner_active,
                                          uint8_t scan_results_count,
                                          int16_t rssi)
{
    s_wifi_state.stack_ready = stack_ready;
    s_wifi_state.owner_active = owner_active;
    s_wifi_state.scan_results_count = scan_results_count;
    s_wifi_state.last_rssi = rssi;
    s_wifi_state.last_update_ms = HAL_GetTick();
}

void ap6256_connectivity_set_wifi_ip(const char *ip,
                                     const char *mask,
                                     const char *gateway,
                                     uint8_t dhcp_bound)
{
    s_wifi_state.dhcp_bound = dhcp_bound;
    ap6256_connectivity_copy_text(s_wifi_state.leased_ip, sizeof(s_wifi_state.leased_ip), ip);
    ap6256_connectivity_copy_text(s_wifi_state.leased_mask, sizeof(s_wifi_state.leased_mask), mask);
    ap6256_connectivity_copy_text(s_wifi_state.leased_gateway, sizeof(s_wifi_state.leased_gateway), gateway);
    s_wifi_state.last_update_ms = HAL_GetTick();
}

void ap6256_connectivity_set_bt_note(const char *text)
{
    ap6256_connectivity_set_bt_error(text);
    s_bt_state.last_update_ms = HAL_GetTick();
}

void ap6256_connectivity_set_bt_runtime(uint8_t stack_ready,
                                        uint8_t owner_active,
                                        uint8_t patchram_loaded,
                                        uint8_t scan_results_count,
                                        uint8_t discovered_services_count,
                                        uint8_t connected,
                                        const char *connection_state)
{
    s_bt_state.stack_ready = stack_ready;
    s_bt_state.owner_active = owner_active;
    s_bt_state.patchram_loaded = patchram_loaded;
    s_bt_state.scan_results_count = scan_results_count;
    s_bt_state.discovered_services_count = discovered_services_count;
    s_bt_state.connected = connected;
    ap6256_connectivity_copy_text(s_bt_state.connection_state,
                                  sizeof(s_bt_state.connection_state),
                                  connection_state);
    s_bt_state.last_update_ms = HAL_GetTick();
}

void ap6256_connectivity_set_bt_selection(const char *address,
                                          const char *name,
                                          int8_t rssi,
                                          const char *service_uuid)
{
    ap6256_connectivity_copy_text(s_bt_state.selected_device,
                                  sizeof(s_bt_state.selected_device),
                                  address);
    ap6256_connectivity_copy_text(s_bt_state.selected_name,
                                  sizeof(s_bt_state.selected_name),
                                  name);
    ap6256_connectivity_copy_text(s_bt_state.selected_service_uuid,
                                  sizeof(s_bt_state.selected_service_uuid),
                                  service_uuid);
    s_bt_state.selected_device_rssi = rssi;
    s_bt_state.last_update_ms = HAL_GetTick();
}

const char *ap6256_connectivity_wifi_security_name(ap6256_wifi_security_t security)
{
    switch (security) {
    case AP6256_WIFI_SECURITY_OPEN:
        return "open";
    case AP6256_WIFI_SECURITY_WPA2_PSK:
        return "wpa2_psk";
    case AP6256_WIFI_SECURITY_UNKNOWN:
    default:
        return "unknown";
    }
}

void ap6256_connectivity_print_wifi_info(void)
{
    const ap6256_wifi_state_t *state = &s_wifi_state;

    test_uart_printf("Wi-Fi transport: %s (status=%s) OCR=0x%08lX CMD5_ready=%u tries=%u\r\n",
                     (state->transport_present != 0U) ? "detected" : "not detected",
                     ap6256_status_to_string(state->transport_status),
                     (unsigned long)state->transport_diag.ocr,
                     state->transport_diag.cmd5_ready,
                     state->transport_diag.cmd5_attempts);
    test_uart_printf("  CCCR=0x%02X SDIO_REV=0x%02X IOE=0x%02X IOR=0x%02X CIS=0x%06lX disabled_cmd5=%u\r\n",
                     state->transport_diag.cccr_rev,
                     state->transport_diag.sdio_rev,
                     state->transport_diag.io_enable,
                     state->transport_diag.io_ready,
                     (unsigned long)state->transport_diag.cis_ptr,
                     state->transport_diag.disabled_cmd5_response);
    test_uart_printf("  WL/BT requested=%u/%u pin_level=%u/%u CMD52_CCCR ok=%u tries=%u\r\n",
                     state->transport_diag.wl_reg_on,
                     state->transport_diag.bt_reg_on,
                     state->transport_diag.wl_pin_level,
                     state->transport_diag.bt_pin_level,
                     state->transport_diag.cccr_read_ok,
                     state->transport_diag.cccr_read_attempts);
    test_uart_printf("  F1/F2 ready=%u/%u block=%u/%u chip_clk=0x%02X chip_id=0x%04X raw=0x%08lX\r\n",
                     state->transport_diag.function1_ready,
                     state->transport_diag.function2_ready,
                     state->transport_diag.function1_block_size,
                     state->transport_diag.function2_block_size,
                     state->transport_diag.chip_clock_csr,
                     ap6256_bcm_chip_id_from_raw(state->transport_diag.chip_id_raw),
                     (unsigned long)state->transport_diag.chip_id_raw);
    test_uart_printf("  Last SSID='%s' security=%s password_captured=%u\r\n",
                     (state->last_ssid[0] != '\0') ? state->last_ssid : "n/a",
                     ap6256_connectivity_wifi_security_name(state->last_security),
                     state->password_captured);
    test_uart_printf("  Runtime: stack_ready=%u owner_active=%u scan_results=%u rssi=%d dhcp=%u ip=%s mask=%s gw=%s\r\n",
                     state->stack_ready,
                     state->owner_active,
                     state->scan_results_count,
                     (int)state->last_rssi,
                     state->dhcp_bound,
                     (state->leased_ip[0] != '\0') ? state->leased_ip : "n/a",
                     (state->leased_mask[0] != '\0') ? state->leased_mask : "n/a",
                     (state->leased_gateway[0] != '\0') ? state->leased_gateway : "n/a");
    test_uart_printf("  Cached session profile: %u\r\n",
                     ap6256_wifi_runtime_has_cached_profile());
    test_uart_printf("  Embedded assets ready: %u\r\n", state->assets_ready);
    ap6256_connectivity_print_asset_line("Wi-Fi FW", ap6256_assets_wifi_firmware());
    ap6256_connectivity_print_asset_line("Wi-Fi CLM", ap6256_assets_wifi_clm_blob());
    ap6256_connectivity_print_asset_line("Wi-Fi NVRAM", ap6256_assets_wifi_nvram());
    test_uart_printf("  Radio owner: %s\r\n",
                     network_manager_owner_name(network_manager_get_owner()));
    test_uart_printf("  Full stack ready: %u\r\n",
                     ap6256_connectivity_full_stack_ready());
    test_uart_printf("  Connectivity note: %s\r\n",
                     (state->last_error[0] != '\0') ? state->last_error : s_stack_note);
    test_uart_write_str("  Wi-Fi blocker: none (CYW43-backed Wi-Fi runtime is active).\r\n");
}

void ap6256_connectivity_print_bt_info(void)
{
    const ap6256_bt_state_t *state = &s_bt_state;

    test_uart_printf("BT HCI: %s (status=%s) reset=%u/s%02X ver=%u/s%02X frames=%lu\r\n",
                     (state->transport_ready != 0U) ? "responsive" : "not responsive",
                     ap6256_status_to_string(state->transport_status),
                     state->transport_diag.reset_event_seen,
                     state->transport_diag.reset_status,
                     state->transport_diag.version_event_seen,
                     state->transport_diag.version_status,
                     (unsigned long)state->transport_diag.event_frames_seen);
    test_uart_printf("  HCI_VER=0x%02X HCI_REV=0x%04X LMP_VER=0x%02X MFG=0x%04X LMP_SUB=0x%04X\r\n",
                     state->transport_diag.hci_version,
                     state->transport_diag.hci_revision,
                     state->transport_diag.lmp_version,
                     state->transport_diag.manufacturer,
                     state->transport_diag.lmp_subversion);
    test_uart_printf("  Interactive BLE selection: device address and service UUID are chosen at runtime.\r\n");
    test_uart_printf("  Timeouts scan=%lu connect=%lu discovery=%lu ms\r\n",
                     (unsigned long)AP6256_BT_SCAN_TIMEOUT_MS,
                     (unsigned long)AP6256_BT_CONNECT_TIMEOUT_MS,
                     (unsigned long)AP6256_BT_DISCOVERY_TIMEOUT_MS);
    test_uart_printf("  Runtime: stack_ready=%u owner_active=%u patchram=%u scan_results=%u services=%u connected=%u state=%s\r\n",
                     state->stack_ready,
                     state->owner_active,
                     state->patchram_loaded,
                     state->scan_results_count,
                     state->discovered_services_count,
                     state->connected,
                     (state->connection_state[0] != '\0') ? state->connection_state : "n/a");
    test_uart_printf("  Selected device=%s name='%s' rssi=%d service=%s\r\n",
                     (state->selected_device[0] != '\0') ? state->selected_device : "n/a",
                     (state->selected_name[0] != '\0') ? state->selected_name : "n/a",
                     (int)state->selected_device_rssi,
                     (state->selected_service_uuid[0] != '\0') ? state->selected_service_uuid : "n/a");
    test_uart_printf("  Cached session selection: %u\r\n",
                     ap6256_bt_runtime_has_cached_selection());
    test_uart_printf("  Embedded assets ready: %u\r\n", state->assets_ready);
    ap6256_connectivity_print_asset_line("BT PatchRAM", ap6256_assets_bt_patchram());
    ap6256_connectivity_print_asset_line("Reference NVRAM", ap6256_assets_reference_nvram());
    test_uart_printf("  Radio owner: %s\r\n",
                     network_manager_owner_name(network_manager_get_owner()));
    test_uart_printf("  Full stack ready: %u\r\n",
                     ap6256_connectivity_full_stack_ready());
    test_uart_printf("  Connectivity note: %s\r\n",
                     (state->last_error[0] != '\0') ? state->last_error : s_stack_note);
    test_uart_write_str("  BT blocker: none (BTstack-backed Bluetooth runtime is active).\r\n");
}
