#include "ap6256_wifi_runtime.h"

#include "ap6256_connectivity.h"
#include "ap6256_cyw43_compat.h"
#include "ap6256_cyw43_port.h"
#include "ap6256_assets.h"
#include "cyw43.h"
#include "cyw43_country.h"
#include "network_manager.h"
#include "test_uart.h"

#include "cmsis_os2.h"
#include "lwip/ip4_addr.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define AP6256_WIFI_MAX_SCAN_RESULTS      16U
#define AP6256_WIFI_JOIN_TIMEOUT_MS       20000U

typedef struct {
    uint8_t valid;
    uint8_t secure;
    uint8_t auth_mode;
    uint8_t ssid_len;
    uint8_t bssid[6];
    uint16_t channel;
    int16_t rssi;
    char ssid[33];
} ap6256_wifi_scan_entry_t;

typedef struct {
    uint8_t initialized;
    uint8_t stack_ready;
    uint8_t link_up;
    uint8_t has_cached_profile;
    volatile uint8_t poll_paused;
    uint8_t cached_secure;
    uint8_t last_scan_count;
    int16_t last_rssi;
    char cached_ssid[33];
    char cached_password[65];
    char last_ip[16];
    char last_mask[16];
    char last_gateway[16];
    ap6256_wifi_scan_entry_t scan[AP6256_WIFI_MAX_SCAN_RESULTS];
} ap6256_wifi_runtime_state_t;

static ap6256_wifi_runtime_state_t s_wifi_runtime;

static const char *ap6256_wifi_runtime_packet_source_name(uint32_t source);

static const uint32_t s_wifi_runtime_profiles[] = {
    AP6256_CYW43_PROFILE_BASELINE,
    AP6256_CYW43_PROFILE_BASELINE_PLUS_TCM
};

void ap6256_wifi_runtime_set_poll_paused(uint8_t paused)
{
    s_wifi_runtime.poll_paused = (paused != 0U) ? 1U : 0U;
}

uint8_t ap6256_wifi_runtime_poll_paused(void)
{
    return s_wifi_runtime.poll_paused;
}

static void ap6256_wifi_runtime_release_owner_with_breadcrumb(uint32_t stage, int32_t detail)
{
    ap6256_cyw43_port_record_breadcrumb(stage, detail);
    network_manager_release(NETWORK_OWNER_WIFI);
    ap6256_wifi_runtime_set_poll_paused(0U);
}

static void ap6256_wifi_runtime_clear_scan_results(void)
{
    memset(s_wifi_runtime.scan, 0, sizeof(s_wifi_runtime.scan));
    s_wifi_runtime.last_scan_count = 0U;
}

static const char *ap6256_wifi_runtime_security_name(uint8_t auth_mode)
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

static ap6256_wifi_security_t ap6256_wifi_runtime_state_security(void)
{
    return (s_wifi_runtime.cached_secure != 0U) ? AP6256_WIFI_SECURITY_WPA2_PSK
                                                : AP6256_WIFI_SECURITY_OPEN;
}

static void ap6256_wifi_runtime_update_ip_state(void)
{
    const struct netif *netif = &cyw43_state.netif[CYW43_ITF_STA];
    char ip[16];
    char mask[16];
    char gateway[16];
    uint8_t dhcp_bound;

    memset(ip, 0, sizeof(ip));
    memset(mask, 0, sizeof(mask));
    memset(gateway, 0, sizeof(gateway));

    if ((netif->flags & NETIF_FLAG_UP) != 0U) {
        (void)ip4addr_ntoa_r(netif_ip4_addr(netif), ip, sizeof(ip));
        (void)ip4addr_ntoa_r(netif_ip4_netmask(netif), mask, sizeof(mask));
        (void)ip4addr_ntoa_r(netif_ip4_gw(netif), gateway, sizeof(gateway));
    }

    dhcp_bound = (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP) ? 1U : 0U;

    (void)snprintf(s_wifi_runtime.last_ip, sizeof(s_wifi_runtime.last_ip), "%s", ip);
    (void)snprintf(s_wifi_runtime.last_mask, sizeof(s_wifi_runtime.last_mask), "%s", mask);
    (void)snprintf(s_wifi_runtime.last_gateway, sizeof(s_wifi_runtime.last_gateway), "%s", gateway);

    ap6256_connectivity_set_wifi_ip(ip, mask, gateway, dhcp_bound);
}

static int ap6256_wifi_runtime_scan_cb(void *env, const cyw43_ev_scan_result_t *result)
{
    uint32_t i;
    uint32_t insert_index = AP6256_WIFI_MAX_SCAN_RESULTS;
    (void)env;

    if ((result == NULL) ||
        (result->ssid_len == 0U) ||
        (result->ssid_len >= sizeof(s_wifi_runtime.scan[0].ssid))) {
        return 0;
    }

    for (i = 0U; i < AP6256_WIFI_MAX_SCAN_RESULTS; ++i) {
        if ((s_wifi_runtime.scan[i].valid != 0U) &&
            (s_wifi_runtime.scan[i].ssid_len == result->ssid_len) &&
            (memcmp(s_wifi_runtime.scan[i].ssid, result->ssid, result->ssid_len) == 0)) {
            insert_index = i;
            break;
        }
        if ((insert_index == AP6256_WIFI_MAX_SCAN_RESULTS) &&
            (s_wifi_runtime.scan[i].valid == 0U)) {
            insert_index = i;
        }
    }

    if (insert_index >= AP6256_WIFI_MAX_SCAN_RESULTS) {
        return 0;
    }

    if ((s_wifi_runtime.scan[insert_index].valid != 0U) &&
        (s_wifi_runtime.scan[insert_index].rssi > result->rssi)) {
        return 0;
    }

    memset(&s_wifi_runtime.scan[insert_index], 0, sizeof(s_wifi_runtime.scan[insert_index]));
    s_wifi_runtime.scan[insert_index].valid = 1U;
    s_wifi_runtime.scan[insert_index].secure = (result->auth_mode != 0U) ? 1U : 0U;
    s_wifi_runtime.scan[insert_index].auth_mode = result->auth_mode;
    s_wifi_runtime.scan[insert_index].ssid_len = result->ssid_len;
    memcpy(s_wifi_runtime.scan[insert_index].bssid, result->bssid, sizeof(result->bssid));
    memcpy(s_wifi_runtime.scan[insert_index].ssid, result->ssid, result->ssid_len);
    s_wifi_runtime.scan[insert_index].ssid[result->ssid_len] = '\0';
    s_wifi_runtime.scan[insert_index].channel = result->channel;
    s_wifi_runtime.scan[insert_index].rssi = result->rssi;

    return 0;
}

static void ap6256_wifi_runtime_sort_scan_results(void)
{
    uint32_t i;
    uint32_t j;

    for (i = 0U; i < AP6256_WIFI_MAX_SCAN_RESULTS; ++i) {
        for (j = i + 1U; j < AP6256_WIFI_MAX_SCAN_RESULTS; ++j) {
            ap6256_wifi_scan_entry_t tmp;

            if (s_wifi_runtime.scan[j].valid == 0U) {
                continue;
            }
            if ((s_wifi_runtime.scan[i].valid == 0U) ||
                (s_wifi_runtime.scan[j].rssi > s_wifi_runtime.scan[i].rssi)) {
                tmp = s_wifi_runtime.scan[i];
                s_wifi_runtime.scan[i] = s_wifi_runtime.scan[j];
                s_wifi_runtime.scan[j] = tmp;
            }
        }
    }

    s_wifi_runtime.last_scan_count = 0U;
    for (i = 0U; i < AP6256_WIFI_MAX_SCAN_RESULTS; ++i) {
        if (s_wifi_runtime.scan[i].valid != 0U) {
            s_wifi_runtime.last_scan_count++;
        }
    }
}

static void ap6256_wifi_runtime_print_scan_results(void)
{
    uint32_t i;

    test_uart_write_str("\r\nNearby Wi-Fi networks:\r\n");
    for (i = 0U; i < AP6256_WIFI_MAX_SCAN_RESULTS; ++i) {
        if (s_wifi_runtime.scan[i].valid == 0U) {
            continue;
        }

        test_uart_printf("  %lu. %-32s RSSI=%d ch=%u sec=%s\r\n",
                         (unsigned long)(i + 1U),
                         s_wifi_runtime.scan[i].ssid,
                         (int)s_wifi_runtime.scan[i].rssi,
                         s_wifi_runtime.scan[i].channel,
                         ap6256_wifi_runtime_security_name(s_wifi_runtime.scan[i].auth_mode));
    }
}

static int ap6256_wifi_runtime_prompt_index(const char *prompt, uint32_t max_index)
{
    char line[16];
    char *endptr = NULL;
    unsigned long selected;
    int line_len;

    if (max_index == 0U) {
        return -1;
    }

    test_uart_write_str(prompt);
    line_len = test_uart_read_line(line, sizeof(line), AP6256_WIFI_PROMPT_TIMEOUT_MS);
    if (line_len <= 0) {
        return -1;
    }

    selected = strtoul(line, &endptr, 10);
    if ((endptr == line) || (*endptr != '\0') || (selected == 0UL) || (selected > max_index)) {
        return -1;
    }

    return (int)(selected - 1UL);
}

static bool ap6256_wifi_runtime_wait_for_scan_complete(uint32_t timeout_ms)
{
    uint32_t start_ms = HAL_GetTick();
    uint32_t last_diag_ms = start_ms;

    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SCAN_WAIT, 0);
    while (cyw43_wifi_scan_active(&cyw43_state)) {
        ap6256_cyw43_port_poll();

        if ((HAL_GetTick() - start_ms) >= timeout_ms) {
            return false;
        }
        if ((HAL_GetTick() - last_diag_ms) >= 1000U) {
            last_diag_ms = HAL_GetTick();
            ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SCAN_WAIT,
                                                (int32_t)(last_diag_ms - start_ms));
            test_uart_printf("[ INFO ] wifi.connect stage: scan wait %lums ev=%lu/%lu/%lu pend=%u src=%s\r\n",
                             (unsigned long)(last_diag_ms - start_ms),
                             (unsigned long)ap6256_cyw43_port_async_event_count(),
                             (unsigned long)ap6256_cyw43_port_last_async_event_type(),
                             (unsigned long)ap6256_cyw43_port_last_async_event_status(),
                             ap6256_cyw43_port_packet_pending(),
                             ap6256_wifi_runtime_packet_source_name(ap6256_cyw43_port_packet_pending_source()));
        }
        osDelay(20U);
    }

    return true;
}

static bool ap6256_wifi_runtime_wait_for_link(uint32_t timeout_ms, int *final_status)
{
    uint32_t start_ms = HAL_GetTick();

    if (final_status != NULL) {
        *final_status = CYW43_LINK_DOWN;
    }

    while ((HAL_GetTick() - start_ms) < timeout_ms) {
    ap6256_cyw43_port_poll();

    int status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);

        if (final_status != NULL) {
            *final_status = status;
        }

        if (status == CYW43_LINK_UP) {
            return true;
        }

        if ((status == CYW43_LINK_FAIL) ||
            (status == CYW43_LINK_NONET) ||
            (status == CYW43_LINK_BADAUTH)) {
            return false;
        }

        osDelay(50U);
    }

    return false;
}

static void ap6256_wifi_runtime_capture_profile(const ap6256_wifi_scan_entry_t *entry,
                                                const char *password)
{
    size_t password_len = 0U;

    if (entry == NULL) {
        return;
    }

    (void)snprintf(s_wifi_runtime.cached_ssid, sizeof(s_wifi_runtime.cached_ssid), "%s", entry->ssid);
    s_wifi_runtime.cached_secure = entry->secure;
    s_wifi_runtime.has_cached_profile = 1U;

    memset(s_wifi_runtime.cached_password, 0, sizeof(s_wifi_runtime.cached_password));
    if ((password != NULL) && (entry->secure != 0U)) {
        password_len = strnlen(password, sizeof(s_wifi_runtime.cached_password) - 1U);
        memcpy(s_wifi_runtime.cached_password, password, password_len);
    }

    ap6256_connectivity_set_wifi_profile(s_wifi_runtime.cached_ssid,
                                         ap6256_wifi_runtime_state_security(),
                                         (entry->secure != 0U) ? 1U : 0U);
}

static void ap6256_wifi_runtime_reset_driver_state(void)
{
    cyw43_poll = NULL;
    memset(&cyw43_state, 0, sizeof(cyw43_state));
}

static bool ap6256_wifi_runtime_wait_for_poll_ptr(uint32_t timeout_ms)
{
    uint32_t start_ms = HAL_GetTick();

    while ((HAL_GetTick() - start_ms) < timeout_ms) {
        if (cyw43_poll != NULL) {
            return true;
        }
        osDelay(1U);
    }

    return false;
}

static bool ap6256_wifi_runtime_wait_for_sta_ready(uint32_t timeout_ms)
{
    uint32_t start_ms = HAL_GetTick();

    while ((HAL_GetTick() - start_ms) < timeout_ms) {
        ap6256_cyw43_port_poll();
        if ((cyw43_state.itf_state & (1U << CYW43_ITF_STA)) != 0U) {
            return true;
        }
        osDelay(1U);
    }

    return false;
}

static const char *ap6256_wifi_runtime_bus_init_ret_name(int32_t bus_init_ret)
{
    switch (bus_init_ret) {
    case 0:
        return "not_started";
    case -CYW43_EIO:
        return "io_error";
    case -CYW43_ETIMEDOUT:
        return "timeout";
    case -CYW43_EINVAL:
        return "bad_param";
    case -CYW43_EPERM:
        return "unsupported_chip";
    default:
        return "unknown";
    }
}

static const char *ap6256_wifi_runtime_stage_name(uint32_t stage)
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

static void ap6256_wifi_runtime_publish_compat_diag(void)
{
    ap6256_connectivity_set_wifi_compat(ap6256_cyw43_port_chip_id_raw(),
                                        ap6256_cyw43_port_ram_base_addr(),
                                        ap6256_cyw43_port_ram_size_bytes(),
                                        ap6256_cyw43_port_nvram_packed_len(),
                                        ap6256_cyw43_port_nvram_padded_len(),
                                        ap6256_cyw43_port_nvram_footer_word(),
                                        ap6256_cyw43_port_nvram_using_reference(),
                                        ap6256_cyw43_port_bus_stage(),
                                        ap6256_cyw43_port_chip_clock_csr_diag(),
                                        ap6256_cyw43_port_sr_control1_diag(),
                                        ap6256_cyw43_port_wlan_ioctrl_diag(),
                                        ap6256_cyw43_port_wlan_resetctrl_diag(),
                                        ap6256_cyw43_port_socram_ioctrl_diag(),
                                        ap6256_cyw43_port_socram_resetctrl_diag(),
                                        ap6256_cyw43_port_profile(),
                                        ap6256_cyw43_port_checkpoint(),
                                        ap6256_cyw43_port_checkpoint_result(),
                                        ap6256_cyw43_port_last_success_checkpoint(),
                                        ap6256_cyw43_port_checkpoint_function(),
                                        ap6256_cyw43_port_checkpoint_address(),
                                        ap6256_cyw43_port_checkpoint_write_value(),
                                        ap6256_cyw43_port_checkpoint_readback_value(),
                                        ap6256_cyw43_port_checkpoint_status(),
                                        ap6256_cyw43_port_wakeup_ctrl_diag(),
                                        ap6256_cyw43_port_sleep_csr_diag(),
                                        ap6256_cyw43_port_cardcap_diag(),
                                        ap6256_cyw43_port_io_ready_diag(),
                                        ap6256_cyw43_port_backplane_is_write(),
                                        ap6256_cyw43_port_backplane_width_bytes(),
                                        ap6256_cyw43_port_backplane_address(),
                                        ap6256_cyw43_port_backplane_status());
    ap6256_connectivity_set_wifi_boot_diag(ap6256_cyw43_port_boot_mode(),
                                           ap6256_cyw43_port_cpu_wrapper_addr(),
                                           ap6256_cyw43_port_ram_wrapper_addr(),
                                           ap6256_cyw43_port_firmware_addr(),
                                           ap6256_cyw43_port_nvram_addr(),
                                           ap6256_cyw43_port_footer_addr(),
                                           ap6256_cyw43_port_cpu_core_id(),
                                           ap6256_cyw43_port_ram_core_id(),
                                           ap6256_cyw43_port_reset_vector_addr(),
                                           ap6256_cyw43_port_reset_vector_value(),
                                           ap6256_cyw43_port_verify_mismatch_addr(),
                                           ap6256_cyw43_port_verify_expected(),
                                           ap6256_cyw43_port_verify_actual());
    ap6256_connectivity_set_wifi_poll_diag(ap6256_cyw43_port_packet_pending(),
                                           ap6256_cyw43_port_packet_pending_source(),
                                           ap6256_cyw43_port_dat1_level(),
                                           ap6256_cyw43_port_cccr_int_pending(),
                                           ap6256_cyw43_port_f1_int_status(),
                                           ap6256_cyw43_port_packet_pending_status(),
                                           ap6256_cyw43_port_kso_status(),
                                           ap6256_cyw43_port_last_ioctl_phase(),
                                           ap6256_cyw43_port_last_ioctl_kind(),
                                           ap6256_cyw43_port_last_ioctl_cmd(),
                                           ap6256_cyw43_port_last_ioctl_iface(),
                                           ap6256_cyw43_port_last_ioctl_len(),
                                           ap6256_cyw43_port_last_ioctl_id(),
                                           ap6256_cyw43_port_last_ioctl_status(),
                                           ap6256_cyw43_port_last_ioctl_poll(),
                                           ap6256_cyw43_port_send_flow_control(),
                                           ap6256_cyw43_port_send_tx_seq(),
                                           ap6256_cyw43_port_send_credit(),
                                           ap6256_cyw43_port_send_synthetic_credit(),
                                           ap6256_cyw43_port_send_credit_status());
}

static void ap6256_wifi_runtime_seed_compat_diag(uint32_t profile)
{
    ap6256_connectivity_set_wifi_compat(0U,
                                        ap6256_cyw43_profile_ram_base(profile),
                                        AP6256_CYW43_RAM_SIZE_BYTES,
                                        0U,
                                        0U,
                                        0U,
                                        (ap6256_cyw43_port_reference_nvram_enabled() == 0U) ? 1U : 0U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        profile,
                                        0U,
                                        AP6256_CYW43_CP_RESULT_PENDING,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        0U);
    ap6256_connectivity_set_wifi_boot_diag(ap6256_cyw43_profile_boot_mode(profile),
                                           0U,
                                           0U,
                                           ap6256_cyw43_profile_ram_base(profile),
                                           0U,
                                           0U,
                                           0U,
                                           0U,
                                           0U,
                                           0U,
                                           0U,
                                           0U,
                                           0U);
    ap6256_connectivity_set_wifi_poll_diag(0U,
                                           AP6256_CYW43_PACKET_SRC_NONE,
                                           0U,
                                           0U,
                                           0U,
                                           0,
                                           0,
                                           AP6256_CYW43_IOCTL_PHASE_NONE,
                                           0U,
                                           0U,
                                           0U,
                                           0U,
                                           0U,
                                           0,
                                           0,
                                           0U,
                                           0U,
                                           0U,
                                           0U,
                                           0);
}

static const char *ap6256_wifi_runtime_packet_source_name(uint32_t source)
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

static const char *ap6256_wifi_runtime_ioctl_phase_name(uint32_t phase)
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
    default:
        return "unknown";
    }
}

static const char *ap6256_wifi_runtime_scan_wake_step_name(uint32_t step)
{
    switch (step) {
    case AP6256_CYW43_SCAN_WAKE_NONE:
        return "none";
    case AP6256_CYW43_SCAN_WAKE_WAKE_CTRL_READ:
        return "wake_rd";
    case AP6256_CYW43_SCAN_WAKE_WAKE_CTRL_WRITE:
        return "wake_wr";
    case AP6256_CYW43_SCAN_WAKE_KSO:
        return "kso";
    case AP6256_CYW43_SCAN_WAKE_SLEEP_VERIFY:
        return "sleep";
    case AP6256_CYW43_SCAN_WAKE_HT_VERIFY:
        return "ht";
    case AP6256_CYW43_SCAN_WAKE_IORDY_VERIFY:
        return "iordy";
    case AP6256_CYW43_SCAN_WAKE_OK:
        return "ok";
    default:
        return "unknown";
    }
}

static void ap6256_wifi_runtime_format_scan_start_detail(char *detail, size_t detail_len, int rc)
{
    (void)snprintf(detail,
                   detail_len,
                   "scan_start rc=%d bc=%s/%lu ph=%s sw=%s io=%lu/%lu id=%lu st=%ld p=%ld pend=%u src=%s irq=%02X f1=%08lX fc=%u/%u/%u syn=%u c53=%c/f%u/b%u/bs%lu/l%lu/st%ld/fr%u c52=%lu/%05lX",
                   rc,
                   ap6256_cyw43_port_breadcrumb_name(ap6256_cyw43_port_breadcrumb_stage()),
                   (unsigned long)ap6256_cyw43_port_breadcrumb_stage(),
                   ap6256_wifi_runtime_ioctl_phase_name(ap6256_cyw43_port_last_ioctl_phase()),
                   ap6256_wifi_runtime_scan_wake_step_name(ap6256_cyw43_port_scan_wake_step()),
                   (unsigned long)ap6256_cyw43_port_last_ioctl_kind(),
                   (unsigned long)ap6256_cyw43_port_last_ioctl_cmd(),
                   (unsigned long)ap6256_cyw43_port_last_ioctl_id(),
                   (long)ap6256_cyw43_port_last_ioctl_status(),
                   (long)ap6256_cyw43_port_last_ioctl_poll(),
                   ap6256_cyw43_port_packet_pending(),
                   ap6256_wifi_runtime_packet_source_name(ap6256_cyw43_port_packet_pending_source()),
                   ap6256_cyw43_port_cccr_int_pending(),
                   (unsigned long)ap6256_cyw43_port_f1_int_status(),
                   ap6256_cyw43_port_send_flow_control(),
                   ap6256_cyw43_port_send_tx_seq(),
                   ap6256_cyw43_port_send_credit(),
                   ap6256_cyw43_port_send_synthetic_credit(),
                   (ap6256_cyw43_port_last_cmd53_write() != 0U) ? 'w' : 'r',
                   ap6256_cyw43_port_last_cmd53_function(),
                   ap6256_cyw43_port_last_cmd53_block_mode(),
                   (unsigned long)ap6256_cyw43_port_last_cmd53_block_size(),
                   (unsigned long)ap6256_cyw43_port_last_cmd53_length(),
                   (long)ap6256_cyw43_port_last_cmd53_status(),
                   ap6256_cyw43_port_last_cmd53_frame_size(),
                   (unsigned long)ap6256_cyw43_port_last_cmd(),
                   (unsigned long)((ap6256_cyw43_port_last_cmd_arg() >> 9U) & 0x1FFFFUL));
}

static void ap6256_wifi_runtime_format_bus_detail(char *detail, size_t detail_len)
{
    (void)snprintf(detail,
                   detail_len,
                   "CYW43 bus=%ld/%s st=%lu/%s boot=%s core=%s cp=%s res=%s pf=%s chip=%04X/r%u ram=%05lX nv=%s:%lu/%lu clk=%02X resetvec=%08lX fail=%08lX c53=%lu",
                   (long)ap6256_cyw43_port_last_bus_init_ret(),
                   ap6256_wifi_runtime_bus_init_ret_name(ap6256_cyw43_port_last_bus_init_ret()),
                   (unsigned long)ap6256_cyw43_port_bus_stage(),
                   ap6256_wifi_runtime_stage_name(ap6256_cyw43_port_bus_stage()),
                   ap6256_cyw43_boot_mode_name(ap6256_cyw43_port_boot_mode()),
                   ap6256_cyw43_core_name(ap6256_cyw43_port_cpu_core_id()),
                   ap6256_cyw43_checkpoint_name(ap6256_cyw43_port_checkpoint()),
                   ap6256_cyw43_checkpoint_result_name(ap6256_cyw43_port_checkpoint_result()),
                   ap6256_cyw43_profile_name(ap6256_cyw43_port_profile()),
                   ap6256_cyw43_chip_id_from_raw(ap6256_cyw43_port_chip_id_raw()),
                   ap6256_cyw43_chip_rev_from_raw(ap6256_cyw43_port_chip_id_raw()),
                   (unsigned long)ap6256_cyw43_port_ram_size_bytes(),
                   ap6256_cyw43_port_nvram_using_reference() ? "ap6256" : "generic",
                   (unsigned long)ap6256_cyw43_port_nvram_packed_len(),
                   (unsigned long)ap6256_cyw43_port_nvram_padded_len(),
                   (unsigned)ap6256_cyw43_port_chip_clock_csr_diag(),
                   (unsigned long)ap6256_cyw43_port_reset_vector_value(),
                   (unsigned long)ap6256_cyw43_port_verify_mismatch_addr(),
                   (unsigned long)ap6256_cyw43_port_last_cmd53_count());
}

static bool ap6256_wifi_runtime_try_profile(uint32_t profile,
                                            char *detail,
                                            size_t detail_len)
{
    test_uart_printf("[ INFO ] wifi.connect stage: cyw43 port init (%s)\r\n",
                     ap6256_cyw43_profile_name(profile));
    ap6256_cyw43_port_init();
    ap6256_cyw43_port_set_profile(profile);
    ap6256_wifi_runtime_seed_compat_diag(profile);
    ap6256_wifi_runtime_reset_driver_state();

    if (ap6256_assets_ready() == 0U) {
        (void)snprintf(detail,
                       detail_len,
                       "AP6256 assets are not ready.");
        ap6256_connectivity_set_wifi_note(detail);
        ap6256_cyw43_port_deinit();
        return false;
    }

    test_uart_write_str("[ INFO ] wifi.connect stage: cyw43_init\r\n");
    cyw43_init(&cyw43_state);

    test_uart_write_str("[ INFO ] wifi.connect stage: cyw43_wifi_set_up enter\r\n");
    cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_STA, true, CYW43_COUNTRY_USA);
    test_uart_write_str("[ INFO ] wifi.connect stage: cyw43_wifi_set_up exit\r\n");
    if ((cyw43_state.itf_state & (1U << CYW43_ITF_STA)) == 0U) {
        ap6256_wifi_runtime_publish_compat_diag();
        (void)snprintf(detail,
                       detail_len,
                       "STA setup failed: setup_rc=%ld pf=%s itf=0x%08lx poll=%p bc=%s/%lu",
                       (long)ap6256_cyw43_port_setup_status(),
                       ap6256_cyw43_profile_name(ap6256_cyw43_port_profile()),
                       (unsigned long)cyw43_state.itf_state,
                       (void *)cyw43_poll,
                       ap6256_cyw43_port_breadcrumb_name(ap6256_cyw43_port_breadcrumb_stage()),
                       (unsigned long)ap6256_cyw43_port_breadcrumb_stage());
        ap6256_connectivity_set_wifi_note(detail);
        cyw43_deinit(&cyw43_state);
        ap6256_cyw43_port_deinit();
        ap6256_wifi_runtime_reset_driver_state();
        return false;
    }

    test_uart_write_str("[ INFO ] wifi.connect stage: wait poll ptr\r\n");
    if (!ap6256_wifi_runtime_wait_for_poll_ptr(1500U)) {
        ap6256_wifi_runtime_publish_compat_diag();
        ap6256_wifi_runtime_format_bus_detail(detail, detail_len);
        ap6256_connectivity_set_wifi_note(detail);
        cyw43_deinit(&cyw43_state);
        ap6256_cyw43_port_deinit();
        ap6256_wifi_runtime_reset_driver_state();
        return false;
    }

    test_uart_write_str("[ INFO ] wifi.connect stage: wait STA ready\r\n");
    if (!ap6256_wifi_runtime_wait_for_sta_ready(1500U)) {
        ap6256_wifi_runtime_publish_compat_diag();
        (void)snprintf(detail,
                       detail_len,
                       "STA not ready: pf=%s itf=0x%08lx poll=%p chip=%04X/r%u st=%lu/%s clk=%02X",
                       ap6256_cyw43_profile_name(ap6256_cyw43_port_profile()),
                       (unsigned long)cyw43_state.itf_state,
                       (void *)cyw43_poll,
                       ap6256_cyw43_chip_id_from_raw(ap6256_cyw43_port_chip_id_raw()),
                       ap6256_cyw43_chip_rev_from_raw(ap6256_cyw43_port_chip_id_raw()),
                       (unsigned long)ap6256_cyw43_port_bus_stage(),
                       ap6256_wifi_runtime_stage_name(ap6256_cyw43_port_bus_stage()),
                       (unsigned)ap6256_cyw43_port_chip_clock_csr_diag());
        ap6256_connectivity_set_wifi_note(detail);
        cyw43_deinit(&cyw43_state);
        ap6256_cyw43_port_deinit();
        ap6256_wifi_runtime_reset_driver_state();
        return false;
    }

    return true;
}

static bool ap6256_wifi_runtime_ensure_ready(char *detail, size_t detail_len)
{
    uint32_t profile_index;

    if (s_wifi_runtime.initialized != 0U) {
        test_uart_write_str("[ INFO ] wifi.connect stage: runtime already initialized\r\n");
        return true;
    }

    for (profile_index = 0U;
         profile_index < (sizeof(s_wifi_runtime_profiles) / sizeof(s_wifi_runtime_profiles[0]));
         ++profile_index) {
        uint32_t profile = s_wifi_runtime_profiles[profile_index];

        if (ap6256_wifi_runtime_try_profile(profile, detail, detail_len)) {
            test_uart_printf("[ INFO ] wifi.connect stage: selected profile %s\r\n",
                             ap6256_cyw43_profile_name(profile));
            test_uart_write_str("[ INFO ] wifi.connect stage: runtime ready\r\n");
            ap6256_wifi_runtime_publish_compat_diag();
            s_wifi_runtime.initialized = 1U;
            s_wifi_runtime.stack_ready = 1U;
            ap6256_connectivity_set_wifi_runtime(1U, 1U, s_wifi_runtime.last_scan_count, s_wifi_runtime.last_rssi);
            ap6256_connectivity_set_wifi_note("BCM43456 CYW43 runtime ready.");
            return true;
        }

        test_uart_printf("[ INFO ] wifi.connect stage: profile %s failed\r\n",
                         ap6256_cyw43_profile_name(profile));
    }

    ap6256_connectivity_set_wifi_note(detail);

    return false;
}

static void ap6256_wifi_runtime_disconnect_current(void)
{
    if ((s_wifi_runtime.initialized == 0U) || (s_wifi_runtime.poll_paused != 0U)) {
        return;
    }

    (void)cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
    osDelay(100U);
    s_wifi_runtime.link_up = 0U;
    s_wifi_runtime.last_rssi = 0;
    memset(s_wifi_runtime.last_ip, 0, sizeof(s_wifi_runtime.last_ip));
    memset(s_wifi_runtime.last_mask, 0, sizeof(s_wifi_runtime.last_mask));
    memset(s_wifi_runtime.last_gateway, 0, sizeof(s_wifi_runtime.last_gateway));
    ap6256_connectivity_set_wifi_ip("", "", "", 0U);
}

static ap6256_status_t ap6256_wifi_runtime_run_common(const char *ssid,
                                                      const char *password,
                                                      uint8_t secure,
                                                      ap6256_wifi_runtime_summary_t *summary,
                                                      char *detail,
                                                      size_t detail_len)
{
    int rc;
    int final_status;
    uint32_t auth_type;
    int32_t rssi = 0;

    if ((ssid == NULL) || (ssid[0] == '\0')) {
        (void)snprintf(detail, detail_len, "No Wi-Fi SSID is available for this run.");
        return AP6256_STATUS_BAD_PARAM;
    }

    if (!ap6256_wifi_runtime_ensure_ready(detail, detail_len)) {
        return AP6256_STATUS_IO_ERROR;
    }

    ap6256_wifi_runtime_disconnect_current();

    auth_type = secure ? CYW43_AUTH_WPA2_AES_PSK : CYW43_AUTH_OPEN;
    rc = cyw43_wifi_join(&cyw43_state,
                         strlen(ssid),
                         (const uint8_t *)ssid,
                         secure ? strlen(password) : 0U,
                         (const uint8_t *)(secure ? password : ""),
                         auth_type,
                         NULL,
                         CYW43_CHANNEL_NONE);
    if (rc != 0) {
        (void)snprintf(detail, detail_len, "Wi-Fi join request failed for '%s' (rc=%d).", ssid, rc);
        ap6256_connectivity_set_wifi_note(detail);
        return AP6256_STATUS_IO_ERROR;
    }

    test_uart_write_str("[ INFO ] wifi.connect stage: wait link/dhcp\r\n");
    if (!ap6256_wifi_runtime_wait_for_link(AP6256_WIFI_DHCP_TIMEOUT_MS, &final_status)) {
        const char *reason = "timed out waiting for association/DHCP";

        if (final_status == CYW43_LINK_BADAUTH) {
            reason = "authentication failed";
        } else if (final_status == CYW43_LINK_NONET) {
            reason = "SSID disappeared or was not found";
        } else if (final_status == CYW43_LINK_FAIL) {
            reason = "firmware reported join failure";
        }

        (void)snprintf(detail, detail_len, "Wi-Fi join for '%s' failed: %s.", ssid, reason);
        ap6256_connectivity_set_wifi_note(detail);
        ap6256_connectivity_set_wifi_ip("", "", "", 0U);
        return AP6256_STATUS_TIMEOUT;
    }

    s_wifi_runtime.link_up = 1U;
    if (cyw43_wifi_get_rssi(&cyw43_state, &rssi) == 0) {
        s_wifi_runtime.last_rssi = (int16_t)rssi;
    }
    ap6256_wifi_runtime_update_ip_state();
    ap6256_connectivity_set_wifi_runtime(1U, 1U, s_wifi_runtime.last_scan_count, s_wifi_runtime.last_rssi);

    if (summary != NULL) {
        memset(summary, 0, sizeof(*summary));
        summary->scan_results_count = s_wifi_runtime.last_scan_count;
        summary->connected = 1U;
        summary->selected_rssi = s_wifi_runtime.last_rssi;
        (void)snprintf(summary->selected_ssid, sizeof(summary->selected_ssid), "%s", ssid);
        (void)snprintf(summary->leased_ip, sizeof(summary->leased_ip), "%s", s_wifi_runtime.last_ip);
        (void)snprintf(summary->leased_mask, sizeof(summary->leased_mask), "%s", s_wifi_runtime.last_mask);
        (void)snprintf(summary->leased_gateway, sizeof(summary->leased_gateway), "%s", s_wifi_runtime.last_gateway);
    }

    (void)snprintf(detail,
                   detail_len,
                   "Joined '%s' and acquired DHCP lease %s.",
                   ssid,
                   (s_wifi_runtime.last_ip[0] != '\0') ? s_wifi_runtime.last_ip : "n/a");
    ap6256_connectivity_set_wifi_note(detail);
    return AP6256_STATUS_OK;
}

void ap6256_wifi_runtime_poll(void)
{
    int32_t rssi = 0;

    if (s_wifi_runtime.initialized == 0U) {
        return;
    }

    ap6256_cyw43_port_poll();

    if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) >= CYW43_LINK_JOIN) {
        if (cyw43_wifi_get_rssi(&cyw43_state, &rssi) == 0) {
            s_wifi_runtime.last_rssi = (int16_t)rssi;
        }
        ap6256_wifi_runtime_update_ip_state();
    }

    ap6256_connectivity_set_wifi_runtime(s_wifi_runtime.stack_ready,
                                         (network_manager_get_owner() == NETWORK_OWNER_WIFI) ? 1U : 0U,
                                         s_wifi_runtime.last_scan_count,
                                         s_wifi_runtime.last_rssi);
}

void ap6256_wifi_runtime_suspend(void)
{
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SUSPEND,
                                        s_wifi_runtime.initialized);
    if (s_wifi_runtime.initialized == 0U) {
        ap6256_connectivity_set_wifi_runtime(1U, 0U, s_wifi_runtime.last_scan_count, s_wifi_runtime.last_rssi);
        return;
    }

    ap6256_wifi_runtime_disconnect_current();
    cyw43_deinit(&cyw43_state);
    ap6256_cyw43_port_deinit();
    s_wifi_runtime.initialized = 0U;
    s_wifi_runtime.link_up = 0U;
    ap6256_wifi_runtime_publish_compat_diag();
    ap6256_connectivity_set_wifi_runtime(1U, 0U, s_wifi_runtime.last_scan_count, s_wifi_runtime.last_rssi);
    ap6256_connectivity_set_wifi_note("BCM43456 Wi-Fi runtime suspended.");
}

uint8_t ap6256_wifi_runtime_has_cached_profile(void)
{
    return s_wifi_runtime.has_cached_profile;
}

void ap6256_wifi_runtime_set_reference_nvram(uint8_t enable)
{
    ap6256_cyw43_port_set_reference_nvram_enabled(enable);
    ap6256_wifi_runtime_publish_compat_diag();
}

uint8_t ap6256_wifi_runtime_reference_nvram_enabled(void)
{
    return ap6256_cyw43_port_reference_nvram_enabled();
}

ap6256_status_t ap6256_wifi_runtime_run_interactive(ap6256_wifi_runtime_summary_t *summary,
                                                    char *detail,
                                                    size_t detail_len)
{
    int scan_index;
    cyw43_wifi_scan_options_t opts;
    ap6256_wifi_scan_entry_t *selected;
    char password[65];
    int password_len;
    int rc;

    if ((detail == NULL) || (detail_len == 0U)) {
        return AP6256_STATUS_BAD_PARAM;
    }

    if (!network_manager_acquire(NETWORK_OWNER_WIFI, 30000U)) {
        (void)snprintf(detail, detail_len, "Timed out waiting for Wi-Fi radio ownership.");
        return AP6256_STATUS_TIMEOUT;
    }

    test_uart_write_str("[ INFO ] wifi.connect stage: acquired wifi owner\r\n");
    ap6256_wifi_runtime_set_poll_paused(1U);
    memset(password, 0, sizeof(password));
    if (!ap6256_wifi_runtime_ensure_ready(detail, detail_len)) {
        ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE, AP6256_STATUS_IO_ERROR);
        return AP6256_STATUS_IO_ERROR;
    }

    test_uart_write_str("[ INFO ] wifi.connect stage: start scan\r\n");
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SCAN_START, 0);
    ap6256_wifi_runtime_clear_scan_results();
    memset(&opts, 0, sizeof(opts));
    opts.scan_type = 0;
    rc = cyw43_wifi_scan(&cyw43_state, &opts, NULL, ap6256_wifi_runtime_scan_cb);
    if (rc != 0) {
        cyw43_state.wifi_scan_state = 0;
        cyw43_state.wifi_scan_cb = NULL;
        cyw43_state.wifi_scan_env = NULL;
        ap6256_wifi_runtime_publish_compat_diag();
        ap6256_wifi_runtime_format_scan_start_detail(detail, detail_len, rc);
        ap6256_connectivity_set_wifi_note(detail);
        ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE, rc);
        return AP6256_STATUS_IO_ERROR;
    }
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SCAN_ACCEPTED, 0);

    if (!ap6256_wifi_runtime_wait_for_scan_complete(AP6256_WIFI_SCAN_TIMEOUT_MS)) {
        ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SCAN_TIMEOUT,
                                            (int32_t)AP6256_WIFI_SCAN_TIMEOUT_MS);
        cyw43_state.wifi_scan_state = 0;
        cyw43_state.wifi_scan_cb = NULL;
        cyw43_state.wifi_scan_env = NULL;
        ap6256_wifi_runtime_publish_compat_diag();
        (void)snprintf(detail,
                       detail_len,
                       "scan_timeout %lums state=%u ev=%lu/%lu/%lu ph=%s pend=%u src=%s irq=%02X f1=%08lX poll=%ld/%ld hdr=%04X/%04X",
                       (unsigned long)AP6256_WIFI_SCAN_TIMEOUT_MS,
                       (unsigned)cyw43_state.wifi_scan_state,
                       (unsigned long)ap6256_cyw43_port_async_event_count(),
                       (unsigned long)ap6256_cyw43_port_last_async_event_type(),
                       (unsigned long)ap6256_cyw43_port_last_async_event_status(),
                       ap6256_wifi_runtime_ioctl_phase_name(ap6256_cyw43_port_last_ioctl_phase()),
                       ap6256_cyw43_port_packet_pending(),
                       ap6256_wifi_runtime_packet_source_name(ap6256_cyw43_port_packet_pending_source()),
                       ap6256_cyw43_port_cccr_int_pending(),
                       (unsigned long)ap6256_cyw43_port_f1_int_status(),
                       (long)ap6256_cyw43_port_poll_header_read_status(),
                       (long)ap6256_cyw43_port_poll_payload_read_status(),
                       ap6256_cyw43_port_poll_hdr0(),
                       ap6256_cyw43_port_poll_hdr1());
        ap6256_connectivity_set_wifi_note(detail);
        ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE, AP6256_STATUS_TIMEOUT);
        return AP6256_STATUS_TIMEOUT;
    }

    test_uart_write_str("[ INFO ] wifi.connect stage: scan complete\r\n");
    ap6256_wifi_runtime_sort_scan_results();
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SCAN_COMPLETE,
                                        s_wifi_runtime.last_scan_count);
    ap6256_connectivity_set_wifi_runtime(1U, 1U, s_wifi_runtime.last_scan_count, s_wifi_runtime.last_rssi);

    if (s_wifi_runtime.last_scan_count == 0U) {
        (void)snprintf(detail, detail_len, "No Wi-Fi access points were found during the scan window.");
        ap6256_connectivity_set_wifi_note(detail);
        ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE, AP6256_STATUS_TIMEOUT);
        return AP6256_STATUS_TIMEOUT;
    }

    test_uart_write_str("[ INFO ] wifi.connect stage: prompt ssid selection\r\n");
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_PROMPT_SSID,
                                        s_wifi_runtime.last_scan_count);
    ap6256_wifi_runtime_print_scan_results();
    scan_index = ap6256_wifi_runtime_prompt_index("Select Wi-Fi network number: ",
                                                  s_wifi_runtime.last_scan_count);
    if (scan_index < 0) {
        (void)snprintf(detail, detail_len, "Wi-Fi network selection timed out or was invalid.");
        ap6256_connectivity_set_wifi_note(detail);
        ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE, AP6256_STATUS_BAD_PARAM);
        return AP6256_STATUS_BAD_PARAM;
    }

    selected = &s_wifi_runtime.scan[(uint32_t)scan_index];
    if ((selected->auth_mode != 0U) && ((selected->auth_mode & 0x04U) == 0U)) {
        (void)snprintf(detail,
                       detail_len,
                       "Selected SSID '%s' uses unsupported security mode '%s'.",
                       selected->ssid,
                       ap6256_wifi_runtime_security_name(selected->auth_mode));
        ap6256_connectivity_set_wifi_note(detail);
        ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE, AP6256_STATUS_BAD_PARAM);
        return AP6256_STATUS_BAD_PARAM;
    }

    if (selected->secure != 0U) {
        test_uart_write_str("[ INFO ] wifi.connect stage: prompt password\r\n");
        test_uart_write_str("Wi-Fi password: ");
        password_len = test_uart_read_line_masked(password, sizeof(password), AP6256_WIFI_PROMPT_TIMEOUT_MS);
        if (password_len <= 0) {
            (void)snprintf(detail, detail_len, "Timed out waiting for Wi-Fi password input.");
            ap6256_connectivity_set_wifi_note(detail);
            ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE, AP6256_STATUS_TIMEOUT);
            return AP6256_STATUS_TIMEOUT;
        }
    }

    test_uart_write_str("[ INFO ] wifi.connect stage: join start\r\n");
    ap6256_wifi_runtime_capture_profile(selected, password);
    {
        ap6256_status_t st = ap6256_wifi_runtime_run_common(selected->ssid,
                                                            password,
                                                            selected->secure,
                                                            summary,
                                                            detail,
                                                            detail_len);
        ap6256_wifi_runtime_set_poll_paused(0U);
        return st;
    }
}

ap6256_status_t ap6256_wifi_runtime_run_cached(ap6256_wifi_runtime_summary_t *summary,
                                               char *detail,
                                               size_t detail_len)
{
    if ((detail == NULL) || (detail_len == 0U)) {
        return AP6256_STATUS_BAD_PARAM;
    }

    if (s_wifi_runtime.has_cached_profile == 0U) {
        (void)snprintf(detail, detail_len, "No cached Wi-Fi profile is available. Run 'run wifi' first.");
        return AP6256_STATUS_BAD_PARAM;
    }

    if (!network_manager_acquire(NETWORK_OWNER_WIFI, 30000U)) {
        (void)snprintf(detail, detail_len, "Timed out waiting for Wi-Fi radio ownership.");
        return AP6256_STATUS_TIMEOUT;
    }

    ap6256_wifi_runtime_set_poll_paused(1U);
    {
        ap6256_status_t st = ap6256_wifi_runtime_run_common(s_wifi_runtime.cached_ssid,
                                                            s_wifi_runtime.cached_password,
                                                            s_wifi_runtime.cached_secure,
                                                            summary,
                                                            detail,
                                                            detail_len);
        ap6256_wifi_runtime_set_poll_paused(0U);
        return st;
    }
}
