#include "ap6256_connectivity.h"

#include "ap6256_bt_runtime.h"
#include "ap6256_cyw43_compat.h"
#include "ap6256_cyw43_port.h"
#include "ap6256_wifi_runtime.h"
#include "cyw43.h"
#include "network_manager.h"
#include "test_uart.h"

#include <stdio.h>
#include <string.h>

static const char s_stack_note[] =
    "AP6256 BCM43456 assets are embedded; Wi-Fi and Bluetooth workers start on demand under the radio owner.";
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

    test_uart_printf("  %s: %s size=%lu sha256=%s",
                     label,
                     asset->filename,
                     (unsigned long)asset->size,
                     asset->sha256);
    if ((asset->version_hint != NULL) && (asset->version_hint[0] != '\0')) {
        test_uart_printf(" ver=%s", asset->version_hint);
    }
    if ((asset->source_url != NULL) && (asset->source_url[0] != '\0')) {
        test_uart_printf(" src=%s", asset->source_url);
    }
    test_uart_write_str("\r\n");
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

static const char *ap6256_connectivity_wifi_stage_name(uint32_t stage)
{
    switch (stage) {
    case 0U:
        return "none";
    case 1U:
        return "start";
    case 2U:
        return "backplane_up";
    case 3U:
        return "backplane_ready";
    case 4U:
        return "alp_set";
    case 5U:
        return "fw_nvram";
    case 6U:
        return "ht_ready";
    case 7U:
        return "f2_ready";
    case 8U:
        return "clm_load";
    case 9U:
        return "txglom";
    case 10U:
        return "apsta";
    case 101U:
        return "poll_hdr";
    case 102U:
        return "poll_payload";
    case 103U:
        return "poll_parse";
    default:
        return "other";
    }
}

static const char *ap6256_connectivity_checkpoint_result_name(uint32_t result)
{
    return ap6256_cyw43_checkpoint_result_name(result);
}

static const char *ap6256_connectivity_packet_source_name(uint32_t source)
{
    switch (source) {
    case AP6256_CYW43_PACKET_SRC_NONE:
        return "none";
    case AP6256_CYW43_PACKET_SRC_DAT1:
        return "dat1";
    case AP6256_CYW43_PACKET_SRC_CCCR_F2:
        return "cccr_f2";
    case AP6256_CYW43_PACKET_SRC_F1_MAILBOX:
        return "f1_mbox";
    case AP6256_CYW43_PACKET_SRC_ERROR:
        return "error";
    default:
        return "unknown";
    }
}

static const char *ap6256_connectivity_ioctl_phase_name(uint32_t phase)
{
    switch (phase) {
    case AP6256_CYW43_IOCTL_PHASE_NONE:
        return "none";
    case AP6256_CYW43_IOCTL_PHASE_SCAN_WAKE:
        return "scan_wake";
    case AP6256_CYW43_IOCTL_PHASE_SEND_FAIL:
        return "send_fail";
    case AP6256_CYW43_IOCTL_PHASE_WAIT_NO_PACKET:
        return "wait_no_packet";
    case AP6256_CYW43_IOCTL_PHASE_WAIT_CMD53:
        return "wait_cmd53";
    case AP6256_CYW43_IOCTL_PHASE_WRONG_ID:
        return "wrong_id";
    case AP6256_CYW43_IOCTL_PHASE_MALFORMED:
        return "malformed";
    case AP6256_CYW43_IOCTL_PHASE_CONTROL_STATUS:
        return "control_status";
    case AP6256_CYW43_IOCTL_PHASE_OK:
        return "ok";
    case AP6256_CYW43_IOCTL_PHASE_SEND_CREDIT:
        return "send_credit";
    case AP6256_CYW43_IOCTL_PHASE_SEND_CREDIT_TIMEOUT:
        return "send_credit_timeout";
    case AP6256_CYW43_IOCTL_PHASE_ACCEPTED_ASYNC:
        return "accepted_async";
    case AP6256_CYW43_IOCTL_PHASE_TX_ACCEPTED:
        return "tx_accepted";
    default:
        return "unknown";
    }
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
    s_wifi_state.runtime_ram_base_addr = AP6256_CYW43_RAM_BASE;
    s_wifi_state.runtime_ram_size_bytes = AP6256_CYW43_RAM_SIZE_BYTES;
    s_wifi_state.runtime_boot_mode = AP6256_CYW43_BOOT_CM3_SOCRAM;
    s_wifi_state.runtime_profile = AP6256_CYW43_PROFILE_BASELINE;
    s_wifi_state.runtime_nvram_using_reference = (ap6256_wifi_runtime_reference_nvram_enabled() == 0U) ? 1U : 0U;
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

void ap6256_connectivity_set_wifi_selection_diag(const uint8_t bssid[6],
                                                 uint16_t channel,
                                                 uint8_t selected_5g,
                                                 const char *fixture_classification)
{
    if (bssid != NULL) {
        memcpy(s_wifi_state.runtime_selected_bssid, bssid, sizeof(s_wifi_state.runtime_selected_bssid));
    } else {
        memset(s_wifi_state.runtime_selected_bssid, 0, sizeof(s_wifi_state.runtime_selected_bssid));
    }
    s_wifi_state.runtime_selected_channel = (uint8_t)((channel <= 255U) ? channel : 0U);
    s_wifi_state.runtime_selected_5g = (selected_5g != 0U) ? 1U : 0U;
    ap6256_connectivity_copy_text(s_wifi_state.runtime_fixture_classification,
                                  sizeof(s_wifi_state.runtime_fixture_classification),
                                  fixture_classification);
    s_wifi_state.last_update_ms = HAL_GetTick();
}

void ap6256_connectivity_set_wifi_candidate_diag(uint8_t candidate_index,
                                                 uint8_t candidate_count)
{
    s_wifi_state.runtime_selected_candidate_index = candidate_index;
    s_wifi_state.runtime_candidate_count = candidate_count;
    s_wifi_state.last_update_ms = HAL_GetTick();
}

void ap6256_connectivity_set_wifi_selection_security_diag(uint8_t auth_mode,
                                                          uint8_t security_flags,
                                                          uint16_t akm_flags,
                                                          uint16_t pairwise_flags,
                                                          uint16_t group_flags,
                                                          uint8_t mfp,
                                                          uint16_t rsn_cap,
                                                          uint16_t chanspec)
{
    s_wifi_state.runtime_selected_auth_mode = auth_mode;
    s_wifi_state.runtime_selected_security_flags = security_flags;
    s_wifi_state.runtime_selected_akm_flags = akm_flags;
    s_wifi_state.runtime_selected_pairwise_flags = pairwise_flags;
    s_wifi_state.runtime_selected_group_flags = group_flags;
    s_wifi_state.runtime_selected_mfp = mfp;
    s_wifi_state.runtime_selected_rsn_cap = rsn_cap;
    s_wifi_state.runtime_selected_chanspec = chanspec;
    s_wifi_state.last_update_ms = HAL_GetTick();
}

void ap6256_connectivity_set_wifi_phy_diag(uint8_t valid,
                                           uint8_t assoc_channel,
                                           uint8_t assoc_5g,
                                           uint8_t wifi5_capable,
                                           uint8_t assoc_wifi5,
                                           uint32_t assoc_chanspec,
                                           uint32_t vhtmode,
                                           uint32_t nmode,
                                           uint32_t band,
                                           const char *fw_version,
                                           const char *clm_version,
                                           const char *country,
                                           const char *caps)
{
    s_wifi_state.runtime_phy_valid = valid;
    s_wifi_state.runtime_assoc_channel = assoc_channel;
    s_wifi_state.runtime_assoc_5g = assoc_5g;
    s_wifi_state.runtime_wifi5_capable = wifi5_capable;
    s_wifi_state.runtime_assoc_wifi5 = assoc_wifi5;
    s_wifi_state.runtime_assoc_chanspec = assoc_chanspec;
    s_wifi_state.runtime_vhtmode = vhtmode;
    s_wifi_state.runtime_nmode = nmode;
    s_wifi_state.runtime_band = band;
    ap6256_connectivity_copy_text(s_wifi_state.runtime_fw_version,
                                  sizeof(s_wifi_state.runtime_fw_version),
                                  fw_version);
    ap6256_connectivity_copy_text(s_wifi_state.runtime_clm_version,
                                  sizeof(s_wifi_state.runtime_clm_version),
                                  clm_version);
    ap6256_connectivity_copy_text(s_wifi_state.runtime_country,
                                  sizeof(s_wifi_state.runtime_country),
                                  country);
    ap6256_connectivity_copy_text(s_wifi_state.runtime_caps,
                                  sizeof(s_wifi_state.runtime_caps),
                                  caps);
    s_wifi_state.last_update_ms = HAL_GetTick();
}

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
                                         uint32_t socram_resetctrl,
                                         uint32_t profile,
                                         uint32_t checkpoint,
                                         uint32_t checkpoint_result,
                                         uint32_t last_success_checkpoint,
                                         uint8_t checkpoint_function,
                                         uint32_t checkpoint_address,
                                         uint32_t checkpoint_write_value,
                                         uint32_t checkpoint_readback_value,
                                         int32_t checkpoint_status,
                                         uint8_t wakeup_ctrl,
                                         uint8_t sleep_csr,
                                         uint8_t cardcap,
                                         uint8_t io_ready,
                                         uint8_t backplane_is_write,
                                         uint8_t backplane_width_bytes,
                                         uint32_t backplane_address,
                                         int32_t backplane_status)
{
    s_wifi_state.runtime_chip_id_raw = chip_id_raw;
    s_wifi_state.runtime_ram_base_addr = ram_base_addr;
    s_wifi_state.runtime_ram_size_bytes = ram_size_bytes;
    s_wifi_state.runtime_nvram_packed_len = nvram_packed_len;
    s_wifi_state.runtime_nvram_padded_len = nvram_padded_len;
    s_wifi_state.runtime_nvram_footer_word = nvram_footer_word;
    s_wifi_state.runtime_nvram_using_reference = nvram_using_reference;
    s_wifi_state.runtime_bus_stage = bus_stage;
    s_wifi_state.runtime_chip_clock_csr = chip_clock_csr;
    s_wifi_state.runtime_sr_control1 = sr_control1;
    s_wifi_state.runtime_wlan_ioctrl = wlan_ioctrl;
    s_wifi_state.runtime_wlan_resetctrl = wlan_resetctrl;
    s_wifi_state.runtime_socram_ioctrl = socram_ioctrl;
    s_wifi_state.runtime_socram_resetctrl = socram_resetctrl;
    s_wifi_state.runtime_profile = profile;
    s_wifi_state.runtime_checkpoint = checkpoint;
    s_wifi_state.runtime_checkpoint_result = checkpoint_result;
    s_wifi_state.runtime_last_success_checkpoint = last_success_checkpoint;
    s_wifi_state.runtime_checkpoint_function = checkpoint_function;
    s_wifi_state.runtime_checkpoint_address = checkpoint_address;
    s_wifi_state.runtime_checkpoint_write_value = checkpoint_write_value;
    s_wifi_state.runtime_checkpoint_readback_value = checkpoint_readback_value;
    s_wifi_state.runtime_checkpoint_status = checkpoint_status;
    s_wifi_state.runtime_wakeup_ctrl = wakeup_ctrl;
    s_wifi_state.runtime_sleep_csr = sleep_csr;
    s_wifi_state.runtime_cardcap = cardcap;
    s_wifi_state.runtime_io_ready = io_ready;
    s_wifi_state.runtime_backplane_is_write = backplane_is_write;
    s_wifi_state.runtime_backplane_width_bytes = backplane_width_bytes;
    s_wifi_state.runtime_backplane_address = backplane_address;
    s_wifi_state.runtime_backplane_status = backplane_status;
    s_wifi_state.last_update_ms = HAL_GetTick();
}

void ap6256_connectivity_set_wifi_boot_diag(uint32_t boot_mode,
                                            uint32_t cpu_wrapper_addr,
                                            uint32_t ram_wrapper_addr,
                                            uint32_t firmware_addr,
                                            uint32_t nvram_addr,
                                            uint32_t footer_addr,
                                            uint32_t cpu_core_id,
                                            uint32_t ram_core_id,
                                            uint32_t reset_vector_addr,
                                            uint32_t reset_vector_value,
                                            uint32_t verify_mismatch_addr,
                                            uint32_t verify_expected,
                                            uint32_t verify_actual)
{
    s_wifi_state.runtime_boot_mode = boot_mode;
    s_wifi_state.runtime_cpu_wrapper_addr = cpu_wrapper_addr;
    s_wifi_state.runtime_ram_wrapper_addr = ram_wrapper_addr;
    s_wifi_state.runtime_firmware_addr = firmware_addr;
    s_wifi_state.runtime_nvram_addr = nvram_addr;
    s_wifi_state.runtime_footer_addr = footer_addr;
    s_wifi_state.runtime_cpu_core_id = cpu_core_id;
    s_wifi_state.runtime_ram_core_id = ram_core_id;
    s_wifi_state.runtime_reset_vector_addr = reset_vector_addr;
    s_wifi_state.runtime_reset_vector_value = reset_vector_value;
    s_wifi_state.runtime_verify_mismatch_addr = verify_mismatch_addr;
    s_wifi_state.runtime_verify_expected = verify_expected;
    s_wifi_state.runtime_verify_actual = verify_actual;
    s_wifi_state.last_update_ms = HAL_GetTick();
}

void ap6256_connectivity_set_wifi_poll_diag(uint8_t packet_pending,
                                            uint8_t packet_pending_source,
                                            uint8_t dat1_level,
                                            uint8_t cccr_int_pending,
                                            uint32_t f1_int_status,
                                            int32_t packet_pending_status,
                                            int32_t kso_status,
                                            uint32_t ioctl_phase,
                                            uint32_t ioctl_kind,
                                            uint32_t ioctl_cmd,
                                            uint32_t ioctl_iface,
                                            uint32_t ioctl_len,
                                            uint32_t ioctl_id,
                                            int32_t ioctl_status,
                                            int32_t ioctl_poll,
                                            uint8_t send_flow_control,
                                            uint8_t send_tx_seq,
                                            uint8_t send_credit,
                                            uint8_t send_synthetic_credit,
                                            int32_t send_credit_status,
                                            uint32_t wait_no_packet_count,
                                            uint32_t wait_recovery_count,
                                            uint32_t wait_forced_probe_count,
                                            uint32_t wait_resend_count,
                                            uint8_t ioctl_recovery_attempted,
                                            uint8_t ioctl_forced_probe_attempted,
                                            uint8_t ioctl_resend_attempted)
{
    s_wifi_state.runtime_packet_pending = packet_pending;
    s_wifi_state.runtime_packet_pending_source = packet_pending_source;
    s_wifi_state.runtime_dat1_level = dat1_level;
    s_wifi_state.runtime_cccr_int_pending = cccr_int_pending;
    s_wifi_state.runtime_f1_int_status = f1_int_status;
    s_wifi_state.runtime_packet_pending_status = packet_pending_status;
    s_wifi_state.runtime_kso_status = kso_status;
    s_wifi_state.runtime_ioctl_phase = ioctl_phase;
    s_wifi_state.runtime_ioctl_kind = ioctl_kind;
    s_wifi_state.runtime_ioctl_cmd = ioctl_cmd;
    s_wifi_state.runtime_ioctl_iface = ioctl_iface;
    s_wifi_state.runtime_ioctl_len = ioctl_len;
    s_wifi_state.runtime_ioctl_id = ioctl_id;
    s_wifi_state.runtime_ioctl_status = ioctl_status;
    s_wifi_state.runtime_ioctl_poll = ioctl_poll;
    s_wifi_state.runtime_send_flow_control = send_flow_control;
    s_wifi_state.runtime_send_tx_seq = send_tx_seq;
    s_wifi_state.runtime_send_credit = send_credit;
    s_wifi_state.runtime_send_synthetic_credit = send_synthetic_credit;
    s_wifi_state.runtime_send_credit_status = send_credit_status;
    s_wifi_state.runtime_wait_no_packet_count = wait_no_packet_count;
    s_wifi_state.runtime_wait_recovery_count = wait_recovery_count;
    s_wifi_state.runtime_wait_forced_probe_count = wait_forced_probe_count;
    s_wifi_state.runtime_wait_resend_count = wait_resend_count;
    s_wifi_state.runtime_ioctl_recovery_attempted = ioctl_recovery_attempted;
    s_wifi_state.runtime_ioctl_forced_probe_attempted = ioctl_forced_probe_attempted;
    s_wifi_state.runtime_ioctl_resend_attempted = ioctl_resend_attempted;
    s_wifi_state.last_update_ms = HAL_GetTick();
}

void ap6256_connectivity_set_wifi_assoc_diag(const ap6256_wifi_assoc_diag_t *diag)
{
    if (diag == NULL) {
        memset(&s_wifi_state.runtime_assoc_diag, 0, sizeof(s_wifi_state.runtime_assoc_diag));
    } else {
        s_wifi_state.runtime_assoc_diag = *diag;
    }
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

void ap6256_connectivity_set_bt_uart_diag(uint32_t tx_blocks,
                                          uint32_t tx_bytes,
                                          uint32_t rx_irq_bytes,
                                          uint32_t rx_blocks_complete,
                                          uint32_t rx_errors,
                                          uint32_t rx_overruns,
                                          uint16_t pending_len,
                                          uint16_t pending_offset,
                                          uint16_t ring_count,
                                          uint8_t irq_active,
                                          uint8_t rx_active)
{
    s_bt_state.uart_tx_blocks = tx_blocks;
    s_bt_state.uart_tx_bytes = tx_bytes;
    s_bt_state.uart_rx_irq_bytes = rx_irq_bytes;
    s_bt_state.uart_rx_blocks_complete = rx_blocks_complete;
    s_bt_state.uart_rx_errors = rx_errors;
    s_bt_state.uart_rx_overruns = rx_overruns;
    s_bt_state.uart_pending_len = pending_len;
    s_bt_state.uart_pending_offset = pending_offset;
    s_bt_state.uart_ring_count = ring_count;
    s_bt_state.uart_irq_active = irq_active;
    s_bt_state.uart_rx_active = rx_active;
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

void ap6256_connectivity_set_bt_read_count(uint32_t read_count)
{
    s_bt_state.gatt_read_count = read_count;
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

static const char *ap6256_connectivity_rx_class_name(uint32_t rx_class)
{
    switch (rx_class) {
    case AP6256_CYW43_RX_CLASS_NONE:
        return "none";
    case AP6256_CYW43_RX_CLASS_CONTROL:
        return "control";
    case AP6256_CYW43_RX_CLASS_ASYNC:
        return "async_event";
    case AP6256_CYW43_RX_CLASS_DATA:
        return "data";
    case AP6256_CYW43_RX_CLASS_MALFORMED:
        return "malformed";
    case AP6256_CYW43_RX_CLASS_RUNT:
        return "runt";
    case AP6256_CYW43_RX_CLASS_UNSUPPORTED:
        return "unsupported";
    default:
        return "unknown";
    }
}

static const char *ap6256_connectivity_scan_auth_name(uint8_t auth_mode)
{
    if ((auth_mode & 0x04U) != 0U) {
        return "wpa2";
    }
    if ((auth_mode & 0x02U) != 0U) {
        return "wpa";
    }
    if ((auth_mode & 0x01U) != 0U) {
        return "wep";
    }
    return "open";
}

static const char *ap6256_connectivity_scan_mfp_name(uint8_t mfp)
{
    if (mfp == CYW43_SCAN_MFP_REQUIRED) {
        return "required";
    }
    if (mfp == CYW43_SCAN_MFP_CAPABLE) {
        return "capable";
    }
    return "none";
}

static const char *ap6256_connectivity_scan_akm_name(uint16_t akm_flags)
{
    if ((akm_flags & CYW43_SCAN_AKM_SAE) != 0U) {
        if ((akm_flags & (CYW43_SCAN_AKM_PSK | CYW43_SCAN_AKM_PSK_SHA256)) != 0U) {
            return "psk+sae";
        }
        return "sae";
    }
    if ((akm_flags & CYW43_SCAN_AKM_PSK_SHA256) != 0U) {
        return "psk-sha256";
    }
    if ((akm_flags & CYW43_SCAN_AKM_PSK) != 0U) {
        return "psk";
    }
    if ((akm_flags & (CYW43_SCAN_AKM_8021X | CYW43_SCAN_AKM_8021X_SHA256)) != 0U) {
        return "802.1x";
    }
    if ((akm_flags & CYW43_SCAN_AKM_OWE) != 0U) {
        return "owe";
    }
    return "unknown";
}

static const char *ap6256_connectivity_scan_cipher_name(uint16_t cipher_flags)
{
    if ((cipher_flags & CYW43_SCAN_CIPHER_CCMP) != 0U) {
        if ((cipher_flags & CYW43_SCAN_CIPHER_TKIP) != 0U) {
            return "ccmp+tkip";
        }
        return "ccmp";
    }
    if ((cipher_flags & CYW43_SCAN_CIPHER_TKIP) != 0U) {
        return "tkip";
    }
    if ((cipher_flags & CYW43_SCAN_CIPHER_GCMP) != 0U) {
        return "gcmp";
    }
    if ((cipher_flags & (CYW43_SCAN_CIPHER_WEP40 | CYW43_SCAN_CIPHER_WEP104)) != 0U) {
        return "wep";
    }
    return "unknown";
}

void ap6256_connectivity_print_wifi_info(void)
{
    const ap6256_wifi_state_t *state = &s_wifi_state;
    ap6256_cyw43_pre_reset_diag_t pre_reset;
    ap6256_cyw43_control_tx_diag_t tx_diag;
    char breadcrumb_reset_flags[64];
    char boot_reset_flags[64];

    memset(&pre_reset, 0, sizeof(pre_reset));
    ap6256_cyw43_port_get_pre_reset_diag(&pre_reset);
    memset(&tx_diag, 0, sizeof(tx_diag));
    ap6256_cyw43_port_get_control_tx_diag(&tx_diag);

    ap6256_cyw43_port_format_reset_flags(ap6256_cyw43_port_breadcrumb_reset_flags(),
                                         breadcrumb_reset_flags,
                                         sizeof(breadcrumb_reset_flags));
    ap6256_cyw43_port_format_reset_flags(ap6256_cyw43_port_boot_reset_flags(),
                                         boot_reset_flags,
                                         sizeof(boot_reset_flags));

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
    test_uart_printf("  Wi-Fi backend: bcm43456_fullmac join_events=%lu async_events=%lu\r\n",
                     (unsigned long)ap6256_cyw43_port_join_event_count(),
                     (unsigned long)ap6256_cyw43_port_async_event_count());
    test_uart_printf("  Wi-Fi selection: bssid=%02X:%02X:%02X:%02X:%02X:%02X ch=%u band=%s candidate=%u/%u fixture=%s\r\n",
                     state->runtime_selected_bssid[0],
                     state->runtime_selected_bssid[1],
                     state->runtime_selected_bssid[2],
                     state->runtime_selected_bssid[3],
                     state->runtime_selected_bssid[4],
                     state->runtime_selected_bssid[5],
                     state->runtime_selected_channel,
                     (state->runtime_selected_5g != 0U) ? "5GHz" :
                         ((state->runtime_selected_channel != 0U) ? "2.4GHz" : "n/a"),
                     state->runtime_selected_candidate_index,
                     state->runtime_candidate_count,
                     (state->runtime_fixture_classification[0] != '\0') ?
                         state->runtime_fixture_classification : "n/a");
    test_uart_printf("  Wi-Fi BSS security: sec=%s flags=0x%02X akm=%s(0x%04X) pair=%s(0x%04X) group=%s(0x%04X) mfp=%s rsncap=0x%04X chanspec=0x%04X\r\n",
                     ap6256_connectivity_scan_auth_name(state->runtime_selected_auth_mode),
                     state->runtime_selected_security_flags,
                     ap6256_connectivity_scan_akm_name(state->runtime_selected_akm_flags),
                     state->runtime_selected_akm_flags,
                     ap6256_connectivity_scan_cipher_name(state->runtime_selected_pairwise_flags),
                     state->runtime_selected_pairwise_flags,
                     ap6256_connectivity_scan_cipher_name(state->runtime_selected_group_flags),
                     state->runtime_selected_group_flags,
                     ap6256_connectivity_scan_mfp_name(state->runtime_selected_mfp),
                     state->runtime_selected_rsn_cap,
                     state->runtime_selected_chanspec);
    test_uart_printf("  Wi-Fi PHY: valid=%u assoc_ch=%u band=%s chanspec=0x%04lX nmode=%lu vhtmode=%lu wifi5_capable=%u assoc_wifi5=%u wl_band=%lu\r\n",
                     state->runtime_phy_valid,
                     state->runtime_assoc_channel,
                     (state->runtime_assoc_5g != 0U) ? "5GHz" :
                         ((state->runtime_assoc_channel != 0U) ? "2.4GHz" : "n/a"),
                     (unsigned long)state->runtime_assoc_chanspec,
                     (unsigned long)state->runtime_nmode,
                     (unsigned long)state->runtime_vhtmode,
                     state->runtime_wifi5_capable,
                     state->runtime_assoc_wifi5,
                     (unsigned long)state->runtime_band);
    test_uart_printf("  Wi-Fi FW/caps: ver='%s' clm='%s' country='%s' caps='%s'\r\n",
                     (state->runtime_fw_version[0] != '\0') ? state->runtime_fw_version : "n/a",
                     (state->runtime_clm_version[0] != '\0') ? state->runtime_clm_version : "n/a",
                     (state->runtime_country[0] != '\0') ? state->runtime_country : "n/a",
                     (state->runtime_caps[0] != '\0') ? state->runtime_caps : "n/a");
    test_uart_printf("  CYW43 compat: chip=0x%04X rev=%u raw=0x%08lX rambase=0x%05lX ram=0x%05lX stage=%lu/%s profile=%s nvram=%s %lu/%lu footer=0x%08lX\r\n",
                     ap6256_cyw43_chip_id_from_raw(state->runtime_chip_id_raw),
                     ap6256_cyw43_chip_rev_from_raw(state->runtime_chip_id_raw),
                     (unsigned long)state->runtime_chip_id_raw,
                     (unsigned long)state->runtime_ram_base_addr,
                     (unsigned long)state->runtime_ram_size_bytes,
                     (unsigned long)state->runtime_bus_stage,
                     ap6256_connectivity_wifi_stage_name(state->runtime_bus_stage),
                     ap6256_cyw43_profile_name(state->runtime_profile),
                     (state->runtime_nvram_using_reference != 0U) ? "ap6256" : "generic",
                     (unsigned long)state->runtime_nvram_packed_len,
                     (unsigned long)state->runtime_nvram_padded_len,
                     (unsigned long)state->runtime_nvram_footer_word);
    test_uart_printf("  Boot diag: boot=%s cpu=%s/0x%03lX wrap=0x%08lX ram=%s/0x%03lX wrap=0x%08lX fw=0x%08lX nv=0x%08lX foot=0x%08lX resetvec=0x%08lX->0x%08lX\r\n",
                     ap6256_cyw43_boot_mode_name(state->runtime_boot_mode),
                     ap6256_cyw43_core_name(state->runtime_cpu_core_id),
                     (unsigned long)state->runtime_cpu_core_id,
                     (unsigned long)state->runtime_cpu_wrapper_addr,
                     ap6256_cyw43_core_name(state->runtime_ram_core_id),
                     (unsigned long)state->runtime_ram_core_id,
                     (unsigned long)state->runtime_ram_wrapper_addr,
                     (unsigned long)state->runtime_firmware_addr,
                     (unsigned long)state->runtime_nvram_addr,
                     (unsigned long)state->runtime_footer_addr,
                     (unsigned long)state->runtime_reset_vector_addr,
                     (unsigned long)state->runtime_reset_vector_value);
    test_uart_printf("  Verify diag: mismatch=0x%08lX exp=0x%08lX got=0x%08lX\r\n",
                     (unsigned long)state->runtime_verify_mismatch_addr,
                     (unsigned long)state->runtime_verify_expected,
                     (unsigned long)state->runtime_verify_actual);
    test_uart_printf("  Core diag: clkcsr=0x%02X sr_ctl1=0x%08lX wlan_io=0x%08lX wlan_rst=0x%08lX socram_io=0x%08lX socram_rst=0x%08lX\r\n",
                     state->runtime_chip_clock_csr,
                     (unsigned long)state->runtime_sr_control1,
                     (unsigned long)state->runtime_wlan_ioctrl,
                     (unsigned long)state->runtime_wlan_resetctrl,
                     (unsigned long)state->runtime_socram_ioctrl,
                     (unsigned long)state->runtime_socram_resetctrl);
    test_uart_printf("  Checkpoint: %s res=%s last_ok=%s fn=%u addr=0x%05lX wr=0x%08lX rd=0x%08lX st=%ld\r\n",
                     ap6256_cyw43_checkpoint_name(state->runtime_checkpoint),
                     ap6256_connectivity_checkpoint_result_name(state->runtime_checkpoint_result),
                     ap6256_cyw43_checkpoint_name(state->runtime_last_success_checkpoint),
                     state->runtime_checkpoint_function,
                     (unsigned long)state->runtime_checkpoint_address,
                     (unsigned long)state->runtime_checkpoint_write_value,
                     (unsigned long)state->runtime_checkpoint_readback_value,
                     (long)state->runtime_checkpoint_status);
    test_uart_printf("  Stage5 diag: wake=0x%02X sleep=0x%02X cardcap=0x%02X iordy=0x%02X bp=%s addr=0x%05lX w=%u st=%ld\r\n",
                     state->runtime_wakeup_ctrl,
                     state->runtime_sleep_csr,
                     state->runtime_cardcap,
                     state->runtime_io_ready,
                     (state->runtime_backplane_is_write != 0U) ? "wr" : "rd",
                     (unsigned long)state->runtime_backplane_address,
                     state->runtime_backplane_width_bytes,
                     (long)state->runtime_backplane_status);
    test_uart_printf("  Breadcrumb: valid=%u stage=%s/%lu detail=%ld tick=%lu boot_reset=%s(0x%08lX) crumb_reset=%s(0x%08lX) setup_rc=%ld\r\n",
                     ap6256_cyw43_port_breadcrumb_valid(),
                     ap6256_cyw43_port_breadcrumb_name(ap6256_cyw43_port_breadcrumb_stage()),
                     (unsigned long)ap6256_cyw43_port_breadcrumb_stage(),
                     (long)ap6256_cyw43_port_breadcrumb_detail(),
                     (unsigned long)ap6256_cyw43_port_breadcrumb_tick_ms(),
                     boot_reset_flags,
                     (unsigned long)ap6256_cyw43_port_boot_reset_flags(),
                     breadcrumb_reset_flags,
                     (unsigned long)ap6256_cyw43_port_breadcrumb_reset_flags(),
                     (long)ap6256_cyw43_port_setup_status());
    test_uart_printf("  Pre-reset diag: valid=%u bc=%s/%lu detail=%ld tick=%lu io_cur=%s %lu/%lu if=%lu len=%lu id=%lu st=%ld poll=%ld assoc=%u/%02X:%02X:%02X:%02X:%02X:%02X/ch%u/%s/cs%04X/auth%02X cand=%u/%u pend=%u/%s dat1=%u irq=%02X f1=%08lX c52=%lu/%08lX st=%ld c53=%c/f%u/b%u/bs%lu/l%lu/st%ld/fr%u rx=%s/ch%u ev=%u/%u/%u fc=%u/%u/%u syn=%u\r\n",
                     pre_reset.valid,
                     ap6256_cyw43_port_breadcrumb_name(pre_reset.breadcrumb_stage),
                     (unsigned long)pre_reset.breadcrumb_stage,
                     (long)pre_reset.breadcrumb_detail,
                     (unsigned long)pre_reset.tick_ms,
                     ap6256_connectivity_ioctl_phase_name(pre_reset.ioctl_phase),
                     (unsigned long)pre_reset.ioctl_kind,
                     (unsigned long)pre_reset.ioctl_cmd,
                     (unsigned long)pre_reset.ioctl_iface,
                     (unsigned long)pre_reset.ioctl_len,
                     (unsigned long)pre_reset.ioctl_id,
                     (long)pre_reset.ioctl_status,
                     (long)pre_reset.ioctl_poll,
                     pre_reset.assoc_target_valid,
                     pre_reset.assoc_target_bssid[0],
                     pre_reset.assoc_target_bssid[1],
                     pre_reset.assoc_target_bssid[2],
                     pre_reset.assoc_target_bssid[3],
                     pre_reset.assoc_target_bssid[4],
                     pre_reset.assoc_target_bssid[5],
                     pre_reset.assoc_target_channel,
                     (pre_reset.assoc_target_5g != 0U) ? "5G" : "2G",
                     pre_reset.assoc_target_chanspec,
                     pre_reset.assoc_target_auth_code,
                     pre_reset.assoc_candidate_index,
                     pre_reset.assoc_candidate_count,
                     pre_reset.packet_pending,
                     ap6256_connectivity_packet_source_name(pre_reset.packet_pending_source),
                     pre_reset.dat1_level,
                     pre_reset.cccr_int_pending,
                     (unsigned long)pre_reset.f1_int_status,
                     (unsigned long)pre_reset.last_cmd,
                     (unsigned long)pre_reset.last_cmd_arg,
                     (long)pre_reset.last_cmd_status,
                     (pre_reset.cmd53_write != 0U) ? 'w' : 'r',
                     pre_reset.cmd53_function,
                     pre_reset.cmd53_block_mode,
                     (unsigned long)pre_reset.cmd53_block_size,
                     (unsigned long)pre_reset.cmd53_length,
                     (long)pre_reset.cmd53_status,
                     pre_reset.cmd53_frame_size,
                     ap6256_connectivity_rx_class_name(pre_reset.rx_class),
                     pre_reset.rx_channel,
                     pre_reset.async_event_type,
                     pre_reset.async_event_status,
                     pre_reset.async_event_reason,
                     pre_reset.send_flow_control,
                     pre_reset.send_tx_seq,
                     pre_reset.send_credit,
                     pre_reset.send_synthetic_credit);
    test_uart_printf("  Poll diag: pend=%u src=%s dat1=%u irq=0x%02X f1int=0x%08lX pst=%ld kso=%ld ioctl=%s %lu/%lu if=%lu len=%lu id=%lu st=%ld poll=%ld done=%lu/%lu if=%lu id=%lu st=%ld poll=%ld np=%lu rec=%lu fp=%lu rs=%lu try=%u/%u/%u\r\n",
                     state->runtime_packet_pending,
                     ap6256_connectivity_packet_source_name(state->runtime_packet_pending_source),
                     state->runtime_dat1_level,
                     state->runtime_cccr_int_pending,
                     (unsigned long)state->runtime_f1_int_status,
                     (long)state->runtime_packet_pending_status,
                     (long)state->runtime_kso_status,
                     ap6256_connectivity_ioctl_phase_name(state->runtime_ioctl_phase),
                     (unsigned long)state->runtime_ioctl_kind,
                     (unsigned long)state->runtime_ioctl_cmd,
                     (unsigned long)state->runtime_ioctl_iface,
                     (unsigned long)state->runtime_ioctl_len,
                     (unsigned long)state->runtime_ioctl_id,
                     (long)state->runtime_ioctl_status,
                     (long)state->runtime_ioctl_poll,
                     (unsigned long)ap6256_cyw43_port_last_completed_ioctl_kind(),
                     (unsigned long)ap6256_cyw43_port_last_completed_ioctl_cmd(),
                     (unsigned long)ap6256_cyw43_port_last_completed_ioctl_iface(),
                     (unsigned long)ap6256_cyw43_port_last_completed_ioctl_id(),
                     (long)ap6256_cyw43_port_last_completed_ioctl_status(),
                     (long)ap6256_cyw43_port_last_completed_ioctl_poll(),
                     (unsigned long)state->runtime_wait_no_packet_count,
                     (unsigned long)state->runtime_wait_recovery_count,
                     (unsigned long)state->runtime_wait_forced_probe_count,
                     (unsigned long)state->runtime_wait_resend_count,
                     state->runtime_ioctl_recovery_attempted,
                     state->runtime_ioctl_forced_probe_attempted,
                     state->runtime_ioctl_resend_attempted);
    test_uart_printf("  Send diag: flow=%u seq=%u credit=%u synth=%u st=%ld\r\n",
                     state->runtime_send_flow_control,
                     state->runtime_send_tx_seq,
                     state->runtime_send_credit,
                     state->runtime_send_synthetic_credit,
                     (long)state->runtime_send_credit_status);
    test_uart_printf("  SDIO diag: c53=%c/f%u/b%u/bs%lu/l%lu/st%ld/fr%u/n%lu c52=%lu/0x%08lX st=%ld rsp=0x%08lX\r\n",
                     (ap6256_cyw43_port_last_cmd53_write() != 0U) ? 'w' : 'r',
                     ap6256_cyw43_port_last_cmd53_function(),
                     ap6256_cyw43_port_last_cmd53_block_mode(),
                     (unsigned long)ap6256_cyw43_port_last_cmd53_block_size(),
                     (unsigned long)ap6256_cyw43_port_last_cmd53_length(),
                     (long)ap6256_cyw43_port_last_cmd53_status(),
                     ap6256_cyw43_port_last_cmd53_frame_size(),
                     (unsigned long)ap6256_cyw43_port_last_cmd53_count(),
                     (unsigned long)ap6256_cyw43_port_last_cmd(),
                     (unsigned long)ap6256_cyw43_port_last_cmd_arg(),
                     (long)ap6256_cyw43_port_last_cmd_status(),
                     (unsigned long)ap6256_cyw43_port_last_cmd_response());
    test_uart_printf("  RX diag: class=%s chan=%u sdpcm=%u payload=%u first=0x%08lX ev=%lu/%lu/%lu f=0x%lX\r\n",
                     ap6256_connectivity_rx_class_name(ap6256_cyw43_port_last_rx_class()),
                     ap6256_cyw43_port_last_rx_channel(),
                     ap6256_cyw43_port_last_rx_sdpcm_len(),
                     ap6256_cyw43_port_last_rx_payload_len(),
                     (unsigned long)ap6256_cyw43_port_last_rx_first_word(),
                     (unsigned long)ap6256_cyw43_port_last_async_event_type(),
                     (unsigned long)ap6256_cyw43_port_last_async_event_status(),
                     (unsigned long)ap6256_cyw43_port_last_async_event_reason(),
                     (unsigned long)ap6256_cyw43_port_last_async_event_flags());
    test_uart_printf("  Assoc diag: valid=%u class=%s status=%ld join=0x%08lX sel=%u sec=%u target=%02X:%02X:%02X:%02X:%02X:%02X/ch%u/cs%04X auth=0x%08lX\r\n",
                     state->runtime_assoc_diag.valid,
                     (state->runtime_assoc_diag.failure_class[0] != '\0') ?
                         state->runtime_assoc_diag.failure_class : "none",
                     (long)state->runtime_assoc_diag.link_status,
                     (unsigned long)state->runtime_assoc_diag.join_state,
                     state->runtime_assoc_diag.selected_5g,
                     state->runtime_assoc_diag.secure,
                     state->runtime_assoc_diag.target_bssid[0],
                     state->runtime_assoc_diag.target_bssid[1],
                     state->runtime_assoc_diag.target_bssid[2],
                     state->runtime_assoc_diag.target_bssid[3],
                     state->runtime_assoc_diag.target_bssid[4],
                     state->runtime_assoc_diag.target_bssid[5],
                     state->runtime_assoc_diag.target_channel,
                     state->runtime_assoc_diag.target_chanspec,
                     (unsigned long)state->runtime_assoc_diag.auth_type);
    test_uart_printf("  Assoc state: assoc=%u/m%u %02X:%02X:%02X:%02X:%02X:%02X ev=%lu/%lu/%lu r=%lu f=0x%lX pje=%u ple=%u keyed=%u\r\n",
                     state->runtime_assoc_diag.assoc_seen,
                     state->runtime_assoc_diag.assoc_matches,
                     state->runtime_assoc_diag.assoc_bssid[0],
                     state->runtime_assoc_diag.assoc_bssid[1],
                     state->runtime_assoc_diag.assoc_bssid[2],
                     state->runtime_assoc_diag.assoc_bssid[3],
                     state->runtime_assoc_diag.assoc_bssid[4],
                     state->runtime_assoc_diag.assoc_bssid[5],
                     (unsigned long)state->runtime_assoc_diag.join_event_count,
                     (unsigned long)state->runtime_assoc_diag.join_event_type,
                     (unsigned long)state->runtime_assoc_diag.join_event_status,
                     (unsigned long)state->runtime_assoc_diag.join_event_reason,
                     (unsigned long)state->runtime_assoc_diag.join_event_flags,
                     state->runtime_assoc_diag.post_join_event_seen,
                     state->runtime_assoc_diag.post_join_link_evidence,
                     state->runtime_assoc_diag.keyed_seen);
    test_uart_printf("  Assoc probe: bssid=%ld/%u cs=%ld/0x%04lX ctr=%ld/0x%08lX/%08lX/%08lX ai=%ld/0x%08lX/%08lX/%08lX rx=%s/%u/0x%08lX\r\n",
                     (long)state->runtime_assoc_diag.get_bssid_rc,
                     state->runtime_assoc_diag.get_bssid_valid,
                     (long)state->runtime_assoc_diag.chanspec_rc,
                     (unsigned long)state->runtime_assoc_diag.chanspec,
                     (long)state->runtime_assoc_diag.counters_rc,
                     (unsigned long)state->runtime_assoc_diag.counters_hash,
                     (unsigned long)state->runtime_assoc_diag.counters_first,
                     (unsigned long)state->runtime_assoc_diag.counters_second,
                     (long)state->runtime_assoc_diag.assoc_info_rc,
                     (unsigned long)state->runtime_assoc_diag.assoc_info_hash,
                     (unsigned long)state->runtime_assoc_diag.assoc_info_first,
                     (unsigned long)state->runtime_assoc_diag.assoc_info_second,
                     ap6256_connectivity_rx_class_name(state->runtime_assoc_diag.last_rx_class),
                     state->runtime_assoc_diag.last_rx_payload_len,
                     (unsigned long)state->runtime_assoc_diag.last_rx_first_word);
    test_uart_printf("  TX ctl: valid=%u io=%lu/%lu if=%lu len=%lu id=%lu sdpcm=%lu xfer=%lu bs=%lu crc=0x%08lX first=%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\r\n",
                     tx_diag.valid,
                     (unsigned long)tx_diag.kind,
                     (unsigned long)tx_diag.cmd,
                     (unsigned long)tx_diag.iface,
                     (unsigned long)tx_diag.len,
                     (unsigned long)tx_diag.id,
                     (unsigned long)tx_diag.sdpcm_len,
                     (unsigned long)tx_diag.transfer_len,
                     (unsigned long)tx_diag.block_size,
                     (unsigned long)tx_diag.checksum,
                     tx_diag.first64[0],
                     tx_diag.first64[1],
                     tx_diag.first64[2],
                     tx_diag.first64[3],
                     tx_diag.first64[4],
                     tx_diag.first64[5],
                     tx_diag.first64[6],
                     tx_diag.first64[7],
                     tx_diag.first64[8],
                     tx_diag.first64[9],
                     tx_diag.first64[10],
                     tx_diag.first64[11],
                     tx_diag.first64[12],
                     tx_diag.first64[13],
                     tx_diag.first64[14],
                     tx_diag.first64[15]);
    test_uart_printf("  Cached session profile: %u\r\n",
                     ap6256_wifi_runtime_has_cached_profile());
    test_uart_printf("  Embedded assets ready: %u\r\n", state->assets_ready);
    test_uart_printf("  Wi-Fi asset profile: name=%s default_nvram=%s fw_hint=%s src=%s\r\n",
                     ap6256_assets_wifi_profile_name(),
                     (ap6256_assets_wifi_profile_default_generic_nvram() != 0U) ? "profile" : "ap6256",
                     ap6256_assets_wifi_profile_firmware_version_hint(),
                     ap6256_assets_wifi_profile_source_url());
    ap6256_connectivity_print_asset_line("Wi-Fi FW", ap6256_assets_wifi_firmware());
    ap6256_connectivity_print_asset_line("Wi-Fi CLM", ap6256_assets_wifi_clm_blob());
    ap6256_connectivity_print_asset_line("Wi-Fi NVRAM (profile/generic)", ap6256_assets_wifi_nvram());
    ap6256_connectivity_print_asset_line("Wi-Fi NVRAM (AP6256 module)", ap6256_assets_reference_nvram());
    test_uart_printf("  Radio owner: %s\r\n",
                     network_manager_owner_name(network_manager_get_owner()));
    test_uart_printf("  Full stack ready: %u\r\n",
                     ap6256_connectivity_full_stack_ready());
    test_uart_printf("  Connectivity note: %s\r\n",
                     (state->last_error[0] != '\0') ? state->last_error : s_stack_note);
    test_uart_write_str("  Wi-Fi blocker: none (Wi-Fi runtime starts only while WIFI owns the radio).\r\n");
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
    test_uart_printf("  UART diag: tx=%lu/%lu rx_irq=%lu rx_blk=%lu err=%lu ov=%lu pend=%u/%u ring=%u irq=%u active=%u\r\n",
                     (unsigned long)state->uart_tx_blocks,
                     (unsigned long)state->uart_tx_bytes,
                     (unsigned long)state->uart_rx_irq_bytes,
                     (unsigned long)state->uart_rx_blocks_complete,
                     (unsigned long)state->uart_rx_errors,
                     (unsigned long)state->uart_rx_overruns,
                     (unsigned)state->uart_pending_offset,
                     (unsigned)state->uart_pending_len,
                     (unsigned)state->uart_ring_count,
                     (unsigned)state->uart_irq_active,
                     (unsigned)state->uart_rx_active);
    test_uart_printf("  Selected device=%s name='%s' rssi=%d service=%s reads=%lu\r\n",
                     (state->selected_device[0] != '\0') ? state->selected_device : "n/a",
                     (state->selected_name[0] != '\0') ? state->selected_name : "n/a",
                     (int)state->selected_device_rssi,
                     (state->selected_service_uuid[0] != '\0') ? state->selected_service_uuid : "n/a",
                     (unsigned long)state->gatt_read_count);
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
    test_uart_write_str("  BT blocker: none (BTstack runtime starts only while BT owns the radio).\r\n");
}
