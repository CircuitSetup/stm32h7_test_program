#include "ap6256_connectivity.h"

#include "ap6256_bt_runtime.h"
#include "ap6256_cyw43_compat.h"
#include "ap6256_wifi_runtime.h"
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
    test_uart_printf("  Cached session profile: %u\r\n",
                     ap6256_wifi_runtime_has_cached_profile());
    test_uart_printf("  Embedded assets ready: %u\r\n", state->assets_ready);
    ap6256_connectivity_print_asset_line("Wi-Fi FW", ap6256_assets_wifi_firmware());
    ap6256_connectivity_print_asset_line("Wi-Fi CLM", ap6256_assets_wifi_clm_blob());
    ap6256_connectivity_print_asset_line("Wi-Fi NVRAM (manual fallback)", ap6256_assets_wifi_nvram());
    ap6256_connectivity_print_asset_line("Wi-Fi NVRAM (runtime default)", ap6256_assets_reference_nvram());
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
    test_uart_write_str("  BT blocker: none (BTstack runtime starts only while BT owns the radio).\r\n");
}
