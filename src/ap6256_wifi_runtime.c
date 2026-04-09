#include "ap6256_wifi_runtime.h"

#include "ap6256_connectivity.h"
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

    if ((result == NULL) || (result->ssid_len == 0U)) {
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

    while (cyw43_wifi_scan_active(&cyw43_state)) {
        ap6256_cyw43_port_poll();

        if ((HAL_GetTick() - start_ms) >= timeout_ms) {
            return false;
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

static bool ap6256_wifi_runtime_wait_for_poll_ptr(uint32_t timeout_ms)
{
    uint32_t start_ms = HAL_GetTick();

    while ((HAL_GetTick() - start_ms) < timeout_ms) {
        ap6256_cyw43_port_poll();
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

static bool ap6256_wifi_runtime_ensure_ready(char *detail, size_t detail_len)
{
    if (s_wifi_runtime.initialized != 0U) {
        return true;
    }

    ap6256_cyw43_port_init();

    if (ap6256_assets_ready() == 0U) {
        (void)snprintf(detail,
                       detail_len,
                       "AP6256 assets are not ready.");
        ap6256_connectivity_set_wifi_note(detail);
        ap6256_cyw43_port_deinit();
        return false;
    }

    cyw43_init(&cyw43_state);

    if (!ap6256_wifi_runtime_wait_for_poll_ptr(500U)) {
        (void)snprintf(detail,
                       detail_len,
                       "cyw43_init ran but cyw43_poll stayed NULL.");
        ap6256_connectivity_set_wifi_note(detail);
        cyw43_deinit(&cyw43_state);
        ap6256_cyw43_port_deinit();
        return false;
    }

    cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_STA, true, CYW43_COUNTRY_USA);

    if (!ap6256_wifi_runtime_wait_for_sta_ready(1000U)) {
        (void)snprintf(detail,
                       detail_len,
                       "STA never became ready: itf_state=0x%08lx poll=%p.",
                       (unsigned long)cyw43_state.itf_state,
                       (void *)cyw43_poll);
        ap6256_connectivity_set_wifi_note(detail);
        cyw43_deinit(&cyw43_state);
        ap6256_cyw43_port_deinit();
        return false;
    }

    s_wifi_runtime.initialized = 1U;
    s_wifi_runtime.stack_ready = 1U;
    ap6256_connectivity_set_wifi_runtime(1U, 1U, s_wifi_runtime.last_scan_count, s_wifi_runtime.last_rssi);
    ap6256_connectivity_set_wifi_note("BCM43456 CYW43 runtime ready.");
    return true;
}

static void ap6256_wifi_runtime_disconnect_current(void)
{
    if (s_wifi_runtime.initialized == 0U) {
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
    if (s_wifi_runtime.initialized == 0U) {
        ap6256_connectivity_set_wifi_runtime(1U, 0U, s_wifi_runtime.last_scan_count, s_wifi_runtime.last_rssi);
        return;
    }

    ap6256_wifi_runtime_disconnect_current();
    cyw43_deinit(&cyw43_state);
    ap6256_cyw43_port_deinit();
    s_wifi_runtime.initialized = 0U;
    s_wifi_runtime.link_up = 0U;
    ap6256_connectivity_set_wifi_runtime(1U, 0U, s_wifi_runtime.last_scan_count, s_wifi_runtime.last_rssi);
    ap6256_connectivity_set_wifi_note("BCM43456 Wi-Fi runtime suspended.");
}

uint8_t ap6256_wifi_runtime_has_cached_profile(void)
{
    return s_wifi_runtime.has_cached_profile;
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

    memset(password, 0, sizeof(password));
    if (!ap6256_wifi_runtime_ensure_ready(detail, detail_len)) {
        network_manager_release(NETWORK_OWNER_WIFI);
        return AP6256_STATUS_IO_ERROR;
    }

    ap6256_wifi_runtime_clear_scan_results();
    memset(&opts, 0, sizeof(opts));
    opts.scan_type = 0;
    rc = cyw43_wifi_scan(&cyw43_state, &opts, NULL, ap6256_wifi_runtime_scan_cb);
    if (rc != 0) {
        (void)snprintf(detail, detail_len, "Wi-Fi scan start failed (rc=%d).", rc);
        ap6256_connectivity_set_wifi_note(detail);
        network_manager_release(NETWORK_OWNER_WIFI);
        return AP6256_STATUS_IO_ERROR;
    }

    if (!ap6256_wifi_runtime_wait_for_scan_complete(AP6256_WIFI_SCAN_TIMEOUT_MS)) {
        (void)snprintf(detail, detail_len, "Wi-Fi scan timed out after %lu ms.", (unsigned long)AP6256_WIFI_SCAN_TIMEOUT_MS);
        ap6256_connectivity_set_wifi_note(detail);
        network_manager_release(NETWORK_OWNER_WIFI);
        return AP6256_STATUS_TIMEOUT;
    }

    ap6256_wifi_runtime_sort_scan_results();
    ap6256_connectivity_set_wifi_runtime(1U, 1U, s_wifi_runtime.last_scan_count, s_wifi_runtime.last_rssi);

    if (s_wifi_runtime.last_scan_count == 0U) {
        (void)snprintf(detail, detail_len, "No Wi-Fi access points were found during the scan window.");
        ap6256_connectivity_set_wifi_note(detail);
        network_manager_release(NETWORK_OWNER_WIFI);
        return AP6256_STATUS_TIMEOUT;
    }

    ap6256_wifi_runtime_print_scan_results();
    scan_index = ap6256_wifi_runtime_prompt_index("Select Wi-Fi network number: ",
                                                  s_wifi_runtime.last_scan_count);
    if (scan_index < 0) {
        (void)snprintf(detail, detail_len, "Wi-Fi network selection timed out or was invalid.");
        ap6256_connectivity_set_wifi_note(detail);
        network_manager_release(NETWORK_OWNER_WIFI);
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
        network_manager_release(NETWORK_OWNER_WIFI);
        return AP6256_STATUS_BAD_PARAM;
    }

    if (selected->secure != 0U) {
        test_uart_write_str("Wi-Fi password: ");
        password_len = test_uart_read_line_masked(password, sizeof(password), AP6256_WIFI_PROMPT_TIMEOUT_MS);
        if (password_len <= 0) {
            (void)snprintf(detail, detail_len, "Timed out waiting for Wi-Fi password input.");
            ap6256_connectivity_set_wifi_note(detail);
            network_manager_release(NETWORK_OWNER_WIFI);
            return AP6256_STATUS_TIMEOUT;
        }
    }

    ap6256_wifi_runtime_capture_profile(selected, password);
    return ap6256_wifi_runtime_run_common(selected->ssid,
                                          password,
                                          selected->secure,
                                          summary,
                                          detail,
                                          detail_len);
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

    return ap6256_wifi_runtime_run_common(s_wifi_runtime.cached_ssid,
                                          s_wifi_runtime.cached_password,
                                          s_wifi_runtime.cached_secure,
                                          summary,
                                          detail,
                                          detail_len);
}
