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
#include "lwip/dhcp.h"
#include "lwip/ip4_addr.h"
#include "lwip/netifapi.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define AP6256_WIFI_MAX_SCAN_RESULTS      32U
#define AP6256_WIFI_JOIN_TIMEOUT_MS       20000U
#define AP6256_WIFI_SCAN_FORCE_5G         (-5)
#define AP6256_WIFI_CHANNEL_5G_UNKNOWN    0xFFFFU
#define AP6256_CYW43_IOCTL_SET_BAND       ((142U << 1U) | 1U)
#define AP6256_CYW43_IOCTL_SET_CHANNEL    ((30U << 1U) | 1U)
#define AP6256_CYW43_JOIN_STATE_KEYED     0x0800U

#ifndef AP6256_WIFI_ENABLE_FORCED_5G_SCAN
#define AP6256_WIFI_ENABLE_FORCED_5G_SCAN 0
#endif

typedef struct {
    uint8_t valid;
    uint8_t secure;
    uint8_t auth_mode;
    uint8_t ssid_len;
    uint8_t bssid[6];
    uint16_t channel;
    uint16_t chanspec;
    uint16_t rsn_cap;
    uint16_t pairwise_cipher_flags;
    uint16_t group_cipher_flags;
    uint16_t akm_flags;
    uint8_t mfp;
    uint8_t security_flags;
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
    uint8_t cached_bssid[6];
    uint16_t cached_channel;
    char cached_ssid[33];
    char cached_password[65];
    char last_ip[16];
    char last_mask[16];
    char last_gateway[16];
    ap6256_wifi_scan_entry_t scan[AP6256_WIFI_MAX_SCAN_RESULTS];
} ap6256_wifi_runtime_state_t;

static ap6256_wifi_runtime_state_t s_wifi_runtime;
static uint8_t s_wifi_runtime_selection_was_index;

static const char *ap6256_wifi_runtime_packet_source_name(uint32_t source);
static int ap6256_wifi_runtime_probe_associated_bssid(uint8_t bssid_out[6]);
static int ap6256_wifi_runtime_set_ioctl_u32(uint32_t cmd, uint32_t value) __attribute__((unused));
static bool ap6256_wifi_runtime_wait_for_scan_complete(uint32_t timeout_ms);

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

static const char *ap6256_wifi_runtime_mfp_name(uint8_t mfp)
{
    if (mfp == CYW43_SCAN_MFP_REQUIRED) {
        return "required";
    }
    if (mfp == CYW43_SCAN_MFP_CAPABLE) {
        return "capable";
    }
    return "none";
}

static const char *ap6256_wifi_runtime_akm_name(uint16_t akm_flags)
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

static const char *ap6256_wifi_runtime_cipher_name(uint16_t cipher_flags)
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

static uint8_t ap6256_wifi_runtime_scan_security_supported(const ap6256_wifi_scan_entry_t *entry,
                                                           char *reason,
                                                           size_t reason_len)
{
    uint16_t supported_akm;

    if (entry == NULL) {
        return 0U;
    }
    if (entry->secure == 0U) {
        return 1U;
    }

    supported_akm = CYW43_SCAN_AKM_PSK | CYW43_SCAN_AKM_PSK_SHA256;
    if ((entry->security_flags != 0U) &&
        ((entry->security_flags & (CYW43_SCAN_SEC_RSN | CYW43_SCAN_SEC_WPA)) == 0U)) {
        if (reason != NULL) {
            (void)snprintf(reason, reason_len, "unsupported legacy security '%s'",
                           ap6256_wifi_runtime_security_name(entry->auth_mode));
        }
        return 0U;
    }
    if ((entry->akm_flags != 0U) && ((entry->akm_flags & supported_akm) == 0U)) {
        if (reason != NULL) {
            (void)snprintf(reason, reason_len, "unsupported AKM '%s'",
                           ap6256_wifi_runtime_akm_name(entry->akm_flags));
        }
        return 0U;
    }
    if (entry->mfp == CYW43_SCAN_MFP_REQUIRED) {
        if (reason != NULL) {
            (void)snprintf(reason, reason_len, "unsupported PMF/MFP required");
        }
        return 0U;
    }
    return 1U;
}

static uint8_t ap6256_wifi_runtime_channel_is_5g(uint16_t channel)
{
    return ((channel > 14U) || (channel == AP6256_WIFI_CHANNEL_5G_UNKNOWN)) ? 1U : 0U;
}

static const char *ap6256_wifi_runtime_channel_band_name(uint16_t channel)
{
    if (channel == 0U) {
        return "n/a";
    }
    if (channel == AP6256_WIFI_CHANNEL_5G_UNKNOWN) {
        return "5GHz?";
    }
    return (ap6256_wifi_runtime_channel_is_5g(channel) != 0U) ? "5GHz" : "2.4GHz";
}

static int ap6256_wifi_runtime_bssid_matches(const uint8_t lhs[6], const uint8_t rhs[6])
{
    return (memcmp(lhs, rhs, 6U) == 0) ? 1 : 0;
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
            (ap6256_wifi_runtime_bssid_matches(s_wifi_runtime.scan[i].bssid, result->bssid) != 0)) {
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
    s_wifi_runtime.scan[insert_index].security_flags = result->security_flags;
    s_wifi_runtime.scan[insert_index].akm_flags = result->akm_flags;
    s_wifi_runtime.scan[insert_index].pairwise_cipher_flags = result->pairwise_cipher_flags;
    s_wifi_runtime.scan[insert_index].group_cipher_flags = result->group_cipher_flags;
    s_wifi_runtime.scan[insert_index].rsn_cap = result->rsn_cap;
    s_wifi_runtime.scan[insert_index].mfp = result->mfp;
    s_wifi_runtime.scan[insert_index].ssid_len = result->ssid_len;
    memcpy(s_wifi_runtime.scan[insert_index].bssid, result->bssid, sizeof(result->bssid));
    memcpy(s_wifi_runtime.scan[insert_index].ssid, result->ssid, result->ssid_len);
    s_wifi_runtime.scan[insert_index].ssid[result->ssid_len] = '\0';
    s_wifi_runtime.scan[insert_index].channel = result->channel;
    s_wifi_runtime.scan[insert_index].chanspec = result->chanspec;
    s_wifi_runtime.scan[insert_index].rssi = result->rssi;

    return 0;
}

static uint32_t ap6256_wifi_runtime_count_scan_results(void)
{
    uint32_t i;
    uint32_t count = 0U;

    for (i = 0U; i < AP6256_WIFI_MAX_SCAN_RESULTS; ++i) {
        if (s_wifi_runtime.scan[i].valid != 0U) {
            count++;
        }
    }

    return count;
}

static uint8_t ap6256_wifi_runtime_has_5g_scan_result(void)
{
    uint32_t i;

    for (i = 0U; i < AP6256_WIFI_MAX_SCAN_RESULTS; ++i) {
        if ((s_wifi_runtime.scan[i].valid != 0U) &&
            (ap6256_wifi_runtime_channel_is_5g(s_wifi_runtime.scan[i].channel) != 0U)) {
            return 1U;
        }
    }

    return 0U;
}

static ap6256_wifi_scan_entry_t *ap6256_wifi_runtime_find_best_ssid_5g(const char *ssid)
{
    ap6256_wifi_scan_entry_t *best = NULL;

    if (ssid == NULL) {
        return NULL;
    }

    for (uint32_t i = 0U; i < AP6256_WIFI_MAX_SCAN_RESULTS; ++i) {
        if ((s_wifi_runtime.scan[i].valid == 0U) ||
            (ap6256_wifi_runtime_channel_is_5g(s_wifi_runtime.scan[i].channel) == 0U) ||
            (strcmp(s_wifi_runtime.scan[i].ssid, ssid) != 0)) {
            continue;
        }

        if ((best == NULL) || (s_wifi_runtime.scan[i].rssi > best->rssi)) {
            best = &s_wifi_runtime.scan[i];
        }
    }

    return best;
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

static int ap6256_wifi_runtime_add_manual_hidden_network(const char *ssid)
{
    size_t ssid_len;
    uint32_t insert_index = AP6256_WIFI_MAX_SCAN_RESULTS;

    if (ssid == NULL) {
        return -1;
    }

    ssid_len = strnlen(ssid, sizeof(s_wifi_runtime.scan[0].ssid));
    if ((ssid_len == 0U) || (ssid_len >= sizeof(s_wifi_runtime.scan[0].ssid))) {
        return -1;
    }

    for (uint32_t i = 0U; i < AP6256_WIFI_MAX_SCAN_RESULTS; ++i) {
        if ((s_wifi_runtime.scan[i].valid != 0U) &&
            (strcmp(s_wifi_runtime.scan[i].ssid, ssid) == 0)) {
            return (int)i;
        }
        if ((insert_index == AP6256_WIFI_MAX_SCAN_RESULTS) &&
            (s_wifi_runtime.scan[i].valid == 0U)) {
            insert_index = i;
        }
    }

    if (insert_index >= AP6256_WIFI_MAX_SCAN_RESULTS) {
        return -1;
    }

    memset(&s_wifi_runtime.scan[insert_index], 0, sizeof(s_wifi_runtime.scan[insert_index]));
    s_wifi_runtime.scan[insert_index].valid = 1U;
    s_wifi_runtime.scan[insert_index].secure = 1U;
    s_wifi_runtime.scan[insert_index].auth_mode = 0x04U; /* WPA2-PSK */
    s_wifi_runtime.scan[insert_index].security_flags = CYW43_SCAN_SEC_RSN;
    s_wifi_runtime.scan[insert_index].akm_flags = CYW43_SCAN_AKM_PSK;
    s_wifi_runtime.scan[insert_index].pairwise_cipher_flags = CYW43_SCAN_CIPHER_CCMP;
    s_wifi_runtime.scan[insert_index].group_cipher_flags = CYW43_SCAN_CIPHER_CCMP;
    s_wifi_runtime.scan[insert_index].mfp = CYW43_SCAN_MFP_NONE;
    s_wifi_runtime.scan[insert_index].ssid_len = (uint8_t)ssid_len;
    memcpy(s_wifi_runtime.scan[insert_index].ssid, ssid, ssid_len);
    s_wifi_runtime.scan[insert_index].ssid[ssid_len] = '\0';
    s_wifi_runtime.scan[insert_index].channel = AP6256_WIFI_CHANNEL_5G_UNKNOWN;
    s_wifi_runtime.scan[insert_index].rssi = -127;

    s_wifi_runtime.last_scan_count = ap6256_wifi_runtime_count_scan_results();
    test_uart_printf("[ INFO ] wifi.connect stage: manual hidden 5GHz SSID '%s'\r\n", ssid);
    return (int)insert_index;
}

static void ap6256_wifi_runtime_print_scan_results(void)
{
    uint32_t i;

    test_uart_write_str("\r\nNearby Wi-Fi networks:\r\n");
    for (i = 0U; i < AP6256_WIFI_MAX_SCAN_RESULTS; ++i) {
        if (s_wifi_runtime.scan[i].valid == 0U) {
            continue;
        }

        test_uart_printf("  %lu. %-32s RSSI=%d ch=%u/%s bssid=%02X:%02X:%02X:%02X:%02X:%02X sec=%s akm=%s pair=%s grp=%s mfp=%s cs=0x%04X\r\n",
                         (unsigned long)(i + 1U),
                         s_wifi_runtime.scan[i].ssid,
                         (int)s_wifi_runtime.scan[i].rssi,
                         s_wifi_runtime.scan[i].channel,
                         ap6256_wifi_runtime_channel_band_name(s_wifi_runtime.scan[i].channel),
                         s_wifi_runtime.scan[i].bssid[0],
                         s_wifi_runtime.scan[i].bssid[1],
                         s_wifi_runtime.scan[i].bssid[2],
                         s_wifi_runtime.scan[i].bssid[3],
                         s_wifi_runtime.scan[i].bssid[4],
                         s_wifi_runtime.scan[i].bssid[5],
                         ap6256_wifi_runtime_security_name(s_wifi_runtime.scan[i].auth_mode),
                         ap6256_wifi_runtime_akm_name(s_wifi_runtime.scan[i].akm_flags),
                         ap6256_wifi_runtime_cipher_name(s_wifi_runtime.scan[i].pairwise_cipher_flags),
                         ap6256_wifi_runtime_cipher_name(s_wifi_runtime.scan[i].group_cipher_flags),
                         ap6256_wifi_runtime_mfp_name(s_wifi_runtime.scan[i].mfp),
                         s_wifi_runtime.scan[i].chanspec);
    }
}

static int ap6256_wifi_runtime_prompt_network_selection(void)
{
    char line[64];
    char *endptr = NULL;
    unsigned long selected;
    uint32_t i;
    int line_len;

    test_uart_write_str("Select Wi-Fi network number or SSID (hidden 5GHz SSID allowed): ");
    line_len = test_uart_read_line(line, sizeof(line), AP6256_WIFI_PROMPT_TIMEOUT_MS);
    if (line_len <= 0) {
        return -1;
    }

    s_wifi_runtime_selection_was_index = 0U;
    selected = strtoul(line, &endptr, 10);
    if ((endptr != line) && (*endptr == '\0') &&
        (selected > 0UL) && (selected <= s_wifi_runtime.last_scan_count)) {
        s_wifi_runtime_selection_was_index = 1U;
        return (int)(selected - 1UL);
    }

    for (i = 0U; i < AP6256_WIFI_MAX_SCAN_RESULTS; ++i) {
        if ((s_wifi_runtime.scan[i].valid != 0U) &&
            (strcmp(s_wifi_runtime.scan[i].ssid, line) == 0)) {
            return (int)i;
        }
    }

    return ap6256_wifi_runtime_add_manual_hidden_network(line);
}

static ap6256_wifi_scan_entry_t *ap6256_wifi_runtime_resolve_hidden_5g_selection(const char *ssid,
                                                                                 char *detail,
                                                                                 size_t detail_len)
{
    cyw43_wifi_scan_options_t opts;
    ap6256_wifi_scan_entry_t *resolved;
    int rc;

    if ((ssid == NULL) || (ssid[0] == '\0')) {
        return NULL;
    }

    test_uart_printf("[ INFO ] wifi.connect stage: directed hidden 5GHz scan ssid=%s\r\n", ssid);

    ap6256_wifi_runtime_clear_scan_results();
    memset(&opts, 0, sizeof(opts));
    opts.scan_type = 0;
    opts.channel_num = AP6256_WIFI_SCAN_FORCE_5G;
    opts.ssid_len = (uint32_t)strnlen(ssid, sizeof(opts.ssid));
    if ((opts.ssid_len == 0U) || (opts.ssid_len > sizeof(opts.ssid))) {
        (void)snprintf(detail, detail_len, "Invalid hidden 5GHz SSID '%s'.", ssid);
        return NULL;
    }
    memcpy(opts.ssid, ssid, opts.ssid_len);

    rc = cyw43_wifi_scan(&cyw43_state, &opts, NULL, ap6256_wifi_runtime_scan_cb);
    if (rc != 0) {
        cyw43_state.wifi_scan_state = 0;
        cyw43_state.wifi_scan_cb = NULL;
        cyw43_state.wifi_scan_env = NULL;
        (void)snprintf(detail,
                       detail_len,
                       "Directed 5GHz scan for hidden SSID '%s' failed to start (rc=%d).",
                       ssid,
                       rc);
        return NULL;
    }

    if (!ap6256_wifi_runtime_wait_for_scan_complete(AP6256_WIFI_SCAN_TIMEOUT_MS)) {
        cyw43_state.wifi_scan_state = 0;
        cyw43_state.wifi_scan_cb = NULL;
        cyw43_state.wifi_scan_env = NULL;
        (void)snprintf(detail,
                       detail_len,
                       "Hidden 5GHz SSID '%s' did not respond during directed scan.",
                       ssid);
        return NULL;
    }

    cyw43_state.wifi_scan_state = 0;
    cyw43_state.wifi_scan_cb = NULL;
    cyw43_state.wifi_scan_env = NULL;
    ap6256_wifi_runtime_sort_scan_results();
    resolved = ap6256_wifi_runtime_find_best_ssid_5g(ssid);
    if (resolved == NULL) {
        (void)snprintf(detail,
                       detail_len,
                       "Hidden 5GHz SSID '%s' was not found; join was not attempted.",
                       ssid);
        return NULL;
    }

    test_uart_printf("[ INFO ] wifi.connect stage: hidden 5GHz resolved bssid=%02X:%02X:%02X:%02X:%02X:%02X ch=%u rssi=%d sec=%s akm=%s pair=%s grp=%s mfp=%s cs=0x%04X\r\n",
                     resolved->bssid[0],
                     resolved->bssid[1],
                     resolved->bssid[2],
                     resolved->bssid[3],
                     resolved->bssid[4],
                     resolved->bssid[5],
                     resolved->channel,
                     (int)resolved->rssi,
                     ap6256_wifi_runtime_security_name(resolved->auth_mode),
                     ap6256_wifi_runtime_akm_name(resolved->akm_flags),
                     ap6256_wifi_runtime_cipher_name(resolved->pairwise_cipher_flags),
                     ap6256_wifi_runtime_cipher_name(resolved->group_cipher_flags),
                     ap6256_wifi_runtime_mfp_name(resolved->mfp),
                     resolved->chanspec);
    return resolved;
}

static ap6256_wifi_scan_entry_t *ap6256_wifi_runtime_prefer_5g_selection(ap6256_wifi_scan_entry_t *fallback,
                                                                         char *detail,
                                                                         size_t detail_len,
                                                                         const char **fixture_classification)
{
    ap6256_wifi_scan_entry_t *visible_5g;
    ap6256_wifi_scan_entry_t *directed_5g;

    if (fixture_classification != NULL) {
        *fixture_classification = "not_checked";
    }
    if (fallback == NULL) {
        return NULL;
    }

    if (ap6256_wifi_runtime_channel_is_5g(fallback->channel) != 0U) {
        if (fixture_classification != NULL) {
            *fixture_classification = "selected_5g_bss";
        }
        return fallback;
    }

    visible_5g = ap6256_wifi_runtime_find_best_ssid_5g(fallback->ssid);
    if (visible_5g != NULL) {
        test_uart_printf("[ INFO ] wifi.connect stage: prefer visible 5GHz BSSID for SSID '%s'\r\n",
                         fallback->ssid);
        if (fixture_classification != NULL) {
            *fixture_classification = "visible_5g_bss";
        }
        return visible_5g;
    }

    /*
     * brcmfmac exposes all BSSIDs and lets cfg80211 choose by band/channel.
     * Our interactive console often starts from a human SSID string, so do one
     * bounded directed 5 GHz scan before accepting a 2.4 GHz fallback.
     */
    directed_5g = ap6256_wifi_runtime_resolve_hidden_5g_selection(fallback->ssid,
                                                                  detail,
                                                                  detail_len);
    if (directed_5g != NULL) {
        if (fixture_classification != NULL) {
            *fixture_classification = "directed_5g_bss";
        }
        return directed_5g;
    }

    test_uart_printf("[ INFO ] wifi.connect stage: fixture_no_5g_bss ssid=%s; using visible %s candidate\r\n",
                     fallback->ssid,
                     ap6256_wifi_runtime_channel_band_name(fallback->channel));
    if (fixture_classification != NULL) {
        *fixture_classification = "fixture_no_5g_bss";
    }
    return fallback;
}

static bool ap6256_wifi_runtime_wait_for_scan_complete(uint32_t timeout_ms)
{
    uint32_t start_ms = HAL_GetTick();
    uint32_t last_diag_ms = start_ms;

    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SCAN_WAIT, 0);
    while (cyw43_wifi_scan_active(&cyw43_state)) {
        ap6256_cyw43_port_poll();

        if ((ap6256_cyw43_port_poll_header_read_status() == -CYW43_ETIMEDOUT) ||
            (ap6256_cyw43_port_poll_payload_read_status() == -CYW43_ETIMEDOUT)) {
            return false;
        }

        if ((HAL_GetTick() - start_ms) >= timeout_ms) {
            if (ap6256_wifi_runtime_count_scan_results() > 0U) {
                cyw43_state.wifi_scan_state = 2;
                return true;
            }
            return false;
        }
        if ((HAL_GetTick() - last_diag_ms) >= 1000U) {
            last_diag_ms = HAL_GetTick();
            ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SCAN_WAIT,
                                                (int32_t)(last_diag_ms - start_ms));
            test_uart_printf("[ INFO ] wifi.connect stage: scan wait %lums results=%lu ev=%lu/%lu/%lu pend=%u src=%s\r\n",
                             (unsigned long)(last_diag_ms - start_ms),
                             (unsigned long)ap6256_wifi_runtime_count_scan_results(),
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
    uint32_t last_diag_ms = start_ms;
    uint8_t assoc_probe_done = 0U;
    uint8_t dhcp_restart_done = 0U;

    if (final_status != NULL) {
        *final_status = CYW43_LINK_DOWN;
    }

    while ((HAL_GetTick() - start_ms) < timeout_ms) {
        uint32_t now_ms;
        int status;

        ap6256_cyw43_port_poll();

        now_ms = HAL_GetTick();
        status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);

        if (final_status != NULL) {
            *final_status = status;
        }

        if (status == CYW43_LINK_UP) {
            return true;
        }

        if ((status == CYW43_LINK_NOIP) && (dhcp_restart_done == 0U)) {
            struct netif *sta_netif = &cyw43_state.netif[CYW43_ITF_STA];

            dhcp_restart_done = 1U;
            test_uart_write_str("[ INFO ] wifi.connect stage: restart DHCP after Wi-Fi link\r\n");
            (void)netifapi_dhcp_release_and_stop(sta_netif);
            (void)netifapi_dhcp_start(sta_netif);
        }

        if ((status == CYW43_LINK_FAIL) ||
            (status == CYW43_LINK_NONET) ||
            (status == CYW43_LINK_BADAUTH)) {
            return false;
        }

        if ((now_ms - last_diag_ms) >= 2000U) {
            uint8_t assoc_bssid[6];
            int assoc_seen = 0;

            memset(assoc_bssid, 0, sizeof(assoc_bssid));
            /*
             * GET_BSSID is a useful AP6256 fallback when firmware associates
             * without producing the exact CYW43 event sequence, but probing too
             * early can return a firmware "not associated" status while the
             * join is still in progress. Keep it delayed and one-shot so slow
             * or rejected APs fail cleanly instead of destabilising the run.
             */
            if (((now_ms - start_ms) >= 4000U) && (assoc_probe_done == 0U)) {
                assoc_probe_done = 1U;
                assoc_seen = ap6256_wifi_runtime_probe_associated_bssid(assoc_bssid);
                if ((assoc_seen != 0) &&
                    ((cyw43_state.wifi_join_state & AP6256_CYW43_JOIN_STATE_KEYED) != 0U)) {
                    cyw43_cb_tcpip_set_link_up(&cyw43_state, CYW43_ITF_STA);
                }
            }
            last_diag_ms = now_ms;
            test_uart_printf("[ INFO ] wifi.connect stage: join wait %lums status=%d join=0x%08lX assoc=%d %02X:%02X:%02X:%02X:%02X:%02X ev=%lu/%lu/%lu r=%lu f=0x%lX\r\n",
                             (unsigned long)(now_ms - start_ms),
                             status,
                             (unsigned long)cyw43_state.wifi_join_state,
                             assoc_seen,
                             assoc_bssid[0],
                             assoc_bssid[1],
                             assoc_bssid[2],
                             assoc_bssid[3],
                             assoc_bssid[4],
                             assoc_bssid[5],
                             (unsigned long)ap6256_cyw43_port_async_event_count(),
                             (unsigned long)ap6256_cyw43_port_last_async_event_type(),
                             (unsigned long)ap6256_cyw43_port_last_async_event_status(),
                             (unsigned long)ap6256_cyw43_port_last_async_event_reason(),
                             (unsigned long)ap6256_cyw43_port_last_async_event_flags());
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
    memcpy(s_wifi_runtime.cached_bssid, entry->bssid, sizeof(s_wifi_runtime.cached_bssid));
    s_wifi_runtime.cached_channel = entry->channel;
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
                                           ap6256_cyw43_port_send_credit_status(),
                                           ap6256_cyw43_port_wait_no_packet_count(),
                                           ap6256_cyw43_port_wait_recovery_count(),
                                           ap6256_cyw43_port_wait_forced_probe_count(),
                                           ap6256_cyw43_port_wait_resend_count(),
                                           ap6256_cyw43_port_ioctl_recovery_attempted(),
                                           ap6256_cyw43_port_ioctl_forced_probe_attempted(),
                                           ap6256_cyw43_port_ioctl_resend_attempted());
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
                                           0,
                                           0U,
                                           0U,
                                           0U,
                                           0U,
                                           0U,
                                           0U,
                                           0U);
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
    case AP6256_CYW43_IOCTL_PHASE_SEND_CREDIT_TIMEOUT:
        return "send_credit_timeout";
    case AP6256_CYW43_IOCTL_PHASE_ACCEPTED_ASYNC:
        return "accepted_async";
    default:
        return "unknown";
    }
}

static uint8_t ap6256_wifi_runtime_should_retry_scan_start(uint32_t phase)
{
    switch (phase) {
    case AP6256_CYW43_IOCTL_PHASE_SCAN_WAKE:
    case AP6256_CYW43_IOCTL_PHASE_SEND_FAIL:
    case AP6256_CYW43_IOCTL_PHASE_WAIT_NO_PACKET:
    case AP6256_CYW43_IOCTL_PHASE_WAIT_CMD53:
    case AP6256_CYW43_IOCTL_PHASE_SEND_CREDIT:
    case AP6256_CYW43_IOCTL_PHASE_SEND_CREDIT_TIMEOUT:
        return 1U;
    default:
        return 0U;
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
                   "scan_start rc=%d bc=%s/%lu ph=%s sw=%s io=%lu/%lu id=%lu st=%ld p=%ld done=%lu/%lu id=%lu st=%ld p=%ld pend=%u src=%s irq=%02X f1=%08lX fc=%u/%u/%u syn=%u np=%lu rec=%lu fp=%lu rs=%lu try=%u/%u/%u c53=%c/f%u/b%u/bs%lu/l%lu/st%ld/fr%u c52=%lu/%05lX",
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
                   (unsigned long)ap6256_cyw43_port_last_completed_ioctl_kind(),
                   (unsigned long)ap6256_cyw43_port_last_completed_ioctl_cmd(),
                   (unsigned long)ap6256_cyw43_port_last_completed_ioctl_id(),
                   (long)ap6256_cyw43_port_last_completed_ioctl_status(),
                   (long)ap6256_cyw43_port_last_completed_ioctl_poll(),
                   ap6256_cyw43_port_packet_pending(),
                   ap6256_wifi_runtime_packet_source_name(ap6256_cyw43_port_packet_pending_source()),
                   ap6256_cyw43_port_cccr_int_pending(),
                   (unsigned long)ap6256_cyw43_port_f1_int_status(),
                   ap6256_cyw43_port_send_flow_control(),
                   ap6256_cyw43_port_send_tx_seq(),
                   ap6256_cyw43_port_send_credit(),
                   ap6256_cyw43_port_send_synthetic_credit(),
                   (unsigned long)ap6256_cyw43_port_wait_no_packet_count(),
                   (unsigned long)ap6256_cyw43_port_wait_recovery_count(),
                   (unsigned long)ap6256_cyw43_port_wait_forced_probe_count(),
                   (unsigned long)ap6256_cyw43_port_wait_resend_count(),
                   ap6256_cyw43_port_ioctl_recovery_attempted(),
                   ap6256_cyw43_port_ioctl_forced_probe_attempted(),
                   ap6256_cyw43_port_ioctl_resend_attempted(),
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
            int pm_rc;

            test_uart_printf("[ INFO ] wifi.connect stage: selected profile %s\r\n",
                             ap6256_cyw43_profile_name(profile));
            cyw43_state.trace_flags |= CYW43_TRACE_ASYNC_EV;
            test_uart_write_str("[ INFO ] wifi.connect stage: set STA PM none\r\n");
            pm_rc = cyw43_wifi_pm(&cyw43_state, CYW43_NONE_PM);
            if (pm_rc != 0) {
                (void)snprintf(detail,
                               detail_len,
                               "STA power-save disable failed before scan/join (rc=%d).",
                               pm_rc);
                ap6256_connectivity_set_wifi_note(detail);
                cyw43_deinit(&cyw43_state);
                ap6256_cyw43_port_deinit();
                ap6256_wifi_runtime_reset_driver_state();
                return false;
            }
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
    if (s_wifi_runtime.initialized == 0U) {
        return;
    }

    if (s_wifi_runtime.link_up != 0U) {
        (void)cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
        osDelay(100U);
    }
    cyw43_state.wifi_join_state = 0U;
    s_wifi_runtime.link_up = 0U;
    s_wifi_runtime.last_rssi = 0;
    memset(s_wifi_runtime.last_ip, 0, sizeof(s_wifi_runtime.last_ip));
    memset(s_wifi_runtime.last_mask, 0, sizeof(s_wifi_runtime.last_mask));
    memset(s_wifi_runtime.last_gateway, 0, sizeof(s_wifi_runtime.last_gateway));
    ap6256_connectivity_set_wifi_ip("", "", "", 0U);
}

static const char *ap6256_wifi_runtime_auth_name(uint32_t auth_type)
{
    switch (auth_type) {
    case CYW43_AUTH_OPEN:
        return "open";
    case CYW43_AUTH_WPA2_AES_PSK:
        return "wpa2_aes";
    case CYW43_AUTH_WPA2_MIXED_PSK:
        return "wpa2_mixed";
    case CYW43_AUTH_WPA_TKIP_PSK:
        return "wpa_tkip";
    default:
        return "unknown";
    }
}

static int ap6256_wifi_runtime_bssid_is_valid(const uint8_t *bssid)
{
    uint8_t all_zero = 1U;
    uint8_t all_ff = 1U;

    if (bssid == NULL) {
        return 0;
    }

    for (size_t i = 0U; i < 6U; ++i) {
        if (bssid[i] != 0U) {
            all_zero = 0U;
        }
        if (bssid[i] != 0xFFU) {
            all_ff = 0U;
        }
    }

    return ((all_zero == 0U) && (all_ff == 0U)) ? 1 : 0;
}

#define AP6256_CYW43_IOCTL_GET_BSSID (0x2EU)
#define AP6256_CYW43_IOCTL_GET_VAR   (0x20CU)
#define AP6256_CYW43_IOCTL_SET_VAR   (0x20FU)

static uint32_t ap6256_wifi_runtime_get_le32(const uint8_t *buf)
{
    return ((uint32_t)buf[0]) |
           ((uint32_t)buf[1] << 8U) |
           ((uint32_t)buf[2] << 16U) |
           ((uint32_t)buf[3] << 24U);
}

static void ap6256_wifi_runtime_put_le32(uint8_t *buf, uint32_t value)
{
    buf[0] = (uint8_t)(value & 0xFFU);
    buf[1] = (uint8_t)((value >> 8) & 0xFFU);
    buf[2] = (uint8_t)((value >> 16) & 0xFFU);
    buf[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static int ap6256_wifi_runtime_probe_associated_bssid(uint8_t bssid_out[6])
{
    int rc;

    if (bssid_out == NULL) {
        return 0;
    }

    memset(bssid_out, 0, 6U);
    rc = cyw43_ioctl(&cyw43_state,
                     AP6256_CYW43_IOCTL_GET_BSSID,
                     6U,
                     bssid_out,
                     CYW43_ITF_STA);
    return ((rc == 0) && (ap6256_wifi_runtime_bssid_is_valid(bssid_out) != 0)) ? 1 : 0;
}

static int ap6256_wifi_runtime_get_iovar_raw(const char *name, uint8_t *buf, size_t len)
{
    size_t name_len;
    int rc;

    if ((name == NULL) || (buf == NULL) || (len == 0U)) {
        return -1;
    }

    name_len = strlen(name) + 1U;
    if (name_len > len) {
        return -1;
    }

    memset(buf, 0, len);
    memcpy(buf, name, name_len);
    rc = cyw43_ioctl(&cyw43_state,
                     AP6256_CYW43_IOCTL_GET_VAR,
                     len,
                     buf,
                     CYW43_ITF_STA);
    if (rc != 0) {
        memset(buf, 0, len);
    }
    return rc;
}

static int ap6256_wifi_runtime_get_iovar_u32(const char *name, uint32_t *value)
{
    uint8_t buf[32];
    int rc;

    if (value == NULL) {
        return -1;
    }

    *value = 0U;
    rc = ap6256_wifi_runtime_get_iovar_raw(name, buf, sizeof(buf));
    if (rc == 0) {
        *value = ap6256_wifi_runtime_get_le32(buf);
    }
    return rc;
}

static int ap6256_wifi_runtime_set_ioctl_u32(uint32_t cmd, uint32_t value)
{
    uint8_t buf[4];

    ap6256_wifi_runtime_put_le32(buf, value);
    return cyw43_ioctl(&cyw43_state,
                       cmd,
                       sizeof(buf),
                       buf,
                       CYW43_ITF_STA);
}

static void ap6256_wifi_runtime_copy_printable(char *dst, size_t dst_len, const uint8_t *src, size_t src_len)
{
    size_t out = 0U;

    if ((dst == NULL) || (dst_len == 0U)) {
        return;
    }

    dst[0] = '\0';
    if (src == NULL) {
        return;
    }

    for (size_t i = 0U; (i < src_len) && (out + 1U < dst_len); ++i) {
        uint8_t ch = src[i];

        if (ch == '\0') {
            break;
        }
        if ((ch == '\r') || (ch == '\n') || (ch == '\t')) {
            ch = ' ';
        } else if ((ch < 0x20U) || (ch > 0x7EU)) {
            ch = '.';
        }
        dst[out++] = (char)ch;
    }
    dst[out] = '\0';
}

static uint8_t ap6256_wifi_runtime_text_has_token(const char *text, const char *token)
{
    return ((text != NULL) && (token != NULL) && (strstr(text, token) != NULL)) ? 1U : 0U;
}

static uint8_t ap6256_wifi_runtime_capture_phy_diag(uint16_t selected_channel,
                                                    uint8_t *assoc_channel_out)
{
    uint8_t buf[160];
    char fw_version[96];
    char caps[128];
    uint32_t vhtmode = 0U;
    uint32_t nmode = 0U;
    uint32_t band = 0U;
    uint32_t chanspec = 0U;
    uint8_t assoc_channel = (uint8_t)((selected_channel <= 255U) ? selected_channel : 0U);
    uint8_t assoc_5g;
    uint8_t wifi5_capable;
    uint8_t assoc_wifi5;
    uint8_t valid = 0U;

    memset(fw_version, 0, sizeof(fw_version));
    memset(caps, 0, sizeof(caps));

    if (ap6256_wifi_runtime_get_iovar_raw("ver", buf, sizeof(buf)) == 0) {
        ap6256_wifi_runtime_copy_printable(fw_version, sizeof(fw_version), buf, sizeof(buf));
        valid = 1U;
    }
    if (ap6256_wifi_runtime_get_iovar_raw("cap", buf, sizeof(buf)) == 0) {
        ap6256_wifi_runtime_copy_printable(caps, sizeof(caps), buf, sizeof(buf));
        valid = 1U;
    }
    if (ap6256_wifi_runtime_get_iovar_u32("vhtmode", &vhtmode) == 0) {
        valid = 1U;
    }
    if (ap6256_wifi_runtime_get_iovar_u32("nmode", &nmode) == 0) {
        valid = 1U;
    }
    if (ap6256_wifi_runtime_get_iovar_u32("band", &band) == 0) {
        valid = 1U;
    }
    if (ap6256_wifi_runtime_get_iovar_u32("chanspec", &chanspec) == 0) {
        uint8_t chanspec_channel = (uint8_t)(chanspec & 0xFFU);
        if (chanspec_channel != 0U) {
            assoc_channel = chanspec_channel;
        }
        valid = 1U;
    }

    assoc_5g = ap6256_wifi_runtime_channel_is_5g(assoc_channel);
    wifi5_capable = ((vhtmode != 0U) ||
                     (ap6256_wifi_runtime_text_has_token(caps, "vht") != 0U) ||
                     (ap6256_wifi_runtime_text_has_token(caps, "802.11ac") != 0U) ||
                     (ap6256_wifi_runtime_text_has_token(caps, "11ac") != 0U)) ? 1U : 0U;
    assoc_wifi5 = ((assoc_5g != 0U) && (wifi5_capable != 0U)) ? 1U : 0U;

    ap6256_connectivity_set_wifi_phy_diag(valid,
                                          assoc_channel,
                                          assoc_5g,
                                          wifi5_capable,
                                          assoc_wifi5,
                                          chanspec,
                                          vhtmode,
                                          nmode,
                                          band,
                                          fw_version,
                                          caps);
    test_uart_printf("[ INFO ] wifi.connect stage: phy diag ch=%u/%s chanspec=0x%04lX nmode=%lu vhtmode=%lu wifi5_capable=%u assoc_wifi5=%u\r\n",
                     assoc_channel,
                     ap6256_wifi_runtime_channel_band_name(assoc_channel),
                     (unsigned long)chanspec,
                     (unsigned long)nmode,
                     (unsigned long)vhtmode,
                     wifi5_capable,
                     assoc_wifi5);
    if (assoc_channel_out != NULL) {
        *assoc_channel_out = assoc_channel;
    }
    return assoc_wifi5;
}

static ap6256_status_t ap6256_wifi_runtime_run_common(const char *ssid,
                                                      const char *password,
                                                      uint8_t secure,
                                                      const uint8_t *bssid,
                                                      uint16_t channel,
                                                      ap6256_wifi_runtime_summary_t *summary,
                                                      char *detail,
                                                      size_t detail_len)
{
    int rc;
    int final_status;
    int32_t rssi = 0;
    uint32_t auth_candidates[2];
    uint32_t auth_count;
    uint32_t auth_index;
    uint32_t selected_auth = CYW43_AUTH_OPEN;
    uint8_t joined = 0U;
    uint8_t actual_assoc_channel = 0U;
    uint8_t actual_assoc_wifi5 = 0U;

    if ((ssid == NULL) || (ssid[0] == '\0')) {
        (void)snprintf(detail, detail_len, "No Wi-Fi SSID is available for this run.");
        return AP6256_STATUS_BAD_PARAM;
    }

    if (!ap6256_wifi_runtime_ensure_ready(detail, detail_len)) {
        return AP6256_STATUS_IO_ERROR;
    }

    if (secure != 0U) {
        auth_candidates[0] = CYW43_AUTH_WPA2_AES_PSK;
        /*
         * brcmfmac derives the exact WPA/WPA2 auth and cipher mix from the BSS
         * profile. Our compact scan classifier only reports "wpa2", so keep a
         * bounded WPA2-mixed fallback for APs that advertise WPA2 but require a
         * mixed WPA/WPA2 auth mask before the firmware supplicant emits PSK_SUP.
         */
        auth_candidates[1] = CYW43_AUTH_WPA2_MIXED_PSK;
        auth_count = 2U;
    } else {
        auth_candidates[0] = CYW43_AUTH_OPEN;
        auth_count = 1U;
    }

    for (auth_index = 0U; auth_index < auth_count; ++auth_index) {
        uint32_t auth_type = auth_candidates[auth_index];

        selected_auth = auth_type;
        ap6256_wifi_runtime_disconnect_current();
        test_uart_printf("[ INFO ] wifi.connect stage: join auth=%s\r\n",
                         ap6256_wifi_runtime_auth_name(auth_type));
        const uint8_t *join_bssid = NULL;
        uint32_t join_channel = CYW43_CHANNEL_NONE;
        if ((bssid != NULL) && (channel != 0U) && (channel != AP6256_WIFI_CHANNEL_5G_UNKNOWN)) {
            /*
             * brcmfmac carries the selected BSSID/chanspec into association.
             * Do this for both 2.4 GHz and 5 GHz so a multi-BSSID SSID cannot
             * drift to a different AP than the one discovered during scan.
             */
            test_uart_printf("[ INFO ] wifi.connect stage: directed join bssid=%02X:%02X:%02X:%02X:%02X:%02X ch=%u/%s\r\n",
                             bssid[0], bssid[1], bssid[2],
                             bssid[3], bssid[4], bssid[5],
                             (unsigned)channel,
                             ap6256_wifi_runtime_channel_band_name(channel));
            join_bssid = bssid;
            join_channel = channel;
        }

        rc = cyw43_wifi_join(&cyw43_state,
                             strlen(ssid),
                             (const uint8_t *)ssid,
                             secure ? strlen(password) : 0U,
                             (const uint8_t *)(secure ? password : ""),
                             auth_type,
                             join_bssid,
                             join_channel);
        if (rc != 0) {
            if ((auth_index + 1U) < auth_count) {
                continue;
            }
            (void)snprintf(detail,
                           detail_len,
                           "Wi-Fi join request failed for '%s' auth=%s rc=%d io=%lu/%lu if=%lu len=%lu id=%lu st=%ld poll=%ld c52=%lu/%08lX c53=%c/f%u/b%u/bs%lu/l%lu/st%ld.",
                           ssid,
                           ap6256_wifi_runtime_auth_name(auth_type),
                           rc,
                           (unsigned long)ap6256_cyw43_port_last_ioctl_kind(),
                           (unsigned long)ap6256_cyw43_port_last_ioctl_cmd(),
                           (unsigned long)ap6256_cyw43_port_last_ioctl_iface(),
                           (unsigned long)ap6256_cyw43_port_last_ioctl_len(),
                           (unsigned long)ap6256_cyw43_port_last_ioctl_id(),
                           (long)ap6256_cyw43_port_last_ioctl_status(),
                           (long)ap6256_cyw43_port_last_ioctl_poll(),
                           (unsigned long)ap6256_cyw43_port_last_cmd(),
                           (unsigned long)ap6256_cyw43_port_last_cmd_arg(),
                           ap6256_cyw43_port_last_cmd53_write() ? 'w' : 'r',
                           ap6256_cyw43_port_last_cmd53_function(),
                           ap6256_cyw43_port_last_cmd53_block_mode(),
                           (unsigned long)ap6256_cyw43_port_last_cmd53_block_size(),
                           (unsigned long)ap6256_cyw43_port_last_cmd53_length(),
                           (long)ap6256_cyw43_port_last_cmd53_status());
            ap6256_connectivity_set_wifi_note(detail);
            return AP6256_STATUS_IO_ERROR;
        }

        test_uart_printf("[ INFO ] wifi.connect stage: wait link/dhcp auth=%s\r\n",
                         ap6256_wifi_runtime_auth_name(auth_type));
        if (ap6256_wifi_runtime_wait_for_link(AP6256_WIFI_DHCP_TIMEOUT_MS, &final_status)) {
            joined = 1U;
            break;
        }

        if ((final_status != CYW43_LINK_BADAUTH) && ((auth_index + 1U) < auth_count)) {
            test_uart_printf("[ INFO ] wifi.connect stage: retry join auth=%s\r\n",
                             ap6256_wifi_runtime_auth_name(auth_candidates[auth_index + 1U]));
            continue;
        }

        {
            const char *reason = "timed out waiting for association/DHCP";

            if (final_status == CYW43_LINK_BADAUTH) {
                reason = "authentication failed";
            } else if (final_status == CYW43_LINK_NONET) {
                reason = "SSID disappeared or was not found";
            } else if (final_status == CYW43_LINK_FAIL) {
                reason = "firmware reported join failure";
            }

            (void)snprintf(detail,
                           detail_len,
                           "Wi-Fi join for '%s' failed with auth=%s: %s.",
                           ssid,
                           ap6256_wifi_runtime_auth_name(auth_type),
                           reason);
            ap6256_connectivity_set_wifi_note(detail);
            ap6256_connectivity_set_wifi_ip("", "", "", 0U);
            return AP6256_STATUS_TIMEOUT;
        }
    }

    if (joined == 0U) {
        (void)snprintf(detail, detail_len, "Wi-Fi join failed without a final link result.");
        ap6256_connectivity_set_wifi_note(detail);
        return AP6256_STATUS_TIMEOUT;
    }

    s_wifi_runtime.link_up = 1U;
    if (cyw43_wifi_get_rssi(&cyw43_state, &rssi) == 0) {
        s_wifi_runtime.last_rssi = (int16_t)rssi;
    }
    ap6256_wifi_runtime_update_ip_state();
    actual_assoc_wifi5 = ap6256_wifi_runtime_capture_phy_diag(channel, &actual_assoc_channel);
    ap6256_connectivity_set_wifi_runtime(1U, 1U, s_wifi_runtime.last_scan_count, s_wifi_runtime.last_rssi);

    if ((ap6256_wifi_runtime_channel_is_5g(channel) != 0U) &&
        (actual_assoc_wifi5 == 0U)) {
        (void)snprintf(detail,
                       detail_len,
                       "Selected 5GHz BSS for '%s' did not associate on 5GHz (actual ch=%u/%s, fallback=%u).",
                       ssid,
                       actual_assoc_channel,
                       ap6256_wifi_runtime_channel_band_name(actual_assoc_channel),
                       0U);
        ap6256_connectivity_set_wifi_note(detail);
        return AP6256_STATUS_TIMEOUT;
    }

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
                   "Joined '%s' on %s with auth=%s and acquired DHCP lease %s; wifi5=%u.",
                   ssid,
                   ap6256_wifi_runtime_channel_band_name(actual_assoc_channel),
                   ap6256_wifi_runtime_auth_name(selected_auth),
                   (s_wifi_runtime.last_ip[0] != '\0') ? s_wifi_runtime.last_ip : "n/a",
                   actual_assoc_wifi5);
    ap6256_connectivity_set_wifi_note(detail);
    return AP6256_STATUS_OK;
}

void ap6256_wifi_runtime_poll(void)
{
    int32_t rssi = 0;

    if ((s_wifi_runtime.initialized == 0U) || (s_wifi_runtime.poll_paused != 0U)) {
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
    uint8_t scan_start_recovery_attempted = 0U;
    ap6256_wifi_scan_entry_t selected_copy;
    const char *fixture_classification = "not_checked";

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

scan_start_retry:
    test_uart_write_str("[ INFO ] wifi.connect stage: start scan\r\n");
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SCAN_START,
                                        (int32_t)scan_start_recovery_attempted);
    ap6256_wifi_runtime_clear_scan_results();
    memset(&opts, 0, sizeof(opts));
    opts.scan_type = 0;
    rc = cyw43_wifi_scan(&cyw43_state, &opts, NULL, ap6256_wifi_runtime_scan_cb);
    if (rc != 0) {
        cyw43_state.wifi_scan_state = 0;
        cyw43_state.wifi_scan_cb = NULL;
        cyw43_state.wifi_scan_env = NULL;
        ap6256_wifi_runtime_publish_compat_diag();

        if ((scan_start_recovery_attempted == 0U) &&
            (ap6256_wifi_runtime_should_retry_scan_start(ap6256_cyw43_port_last_ioctl_phase()) != 0U)) {
            scan_start_recovery_attempted = 1U;
            test_uart_write_str("[ INFO ] wifi.connect stage: scan start recovery\r\n");
            if (((ap6256_cyw43_port_last_ioctl_phase() == AP6256_CYW43_IOCTL_PHASE_WAIT_NO_PACKET) ||
                 (ap6256_cyw43_port_last_ioctl_phase() == AP6256_CYW43_IOCTL_PHASE_SEND_FAIL) ||
                 (ap6256_cyw43_port_last_ioctl_phase() == AP6256_CYW43_IOCTL_PHASE_WAIT_CMD53)) &&
                (ap6256_cyw43_port_runtime_f2_block_size() != 64U)) {
                test_uart_printf("[ INFO ] wifi.connect stage: retry with F2 block size 64 after %s at bs=%lu\r\n",
                                 ap6256_wifi_runtime_ioctl_phase_name(ap6256_cyw43_port_last_ioctl_phase()),
                                 (unsigned long)ap6256_cyw43_port_runtime_f2_block_size());
                ap6256_cyw43_port_set_runtime_f2_block_size(64U);
            }
            ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SUSPEND, rc);
            ap6256_wifi_runtime_suspend();
            if (!ap6256_wifi_runtime_ensure_ready(detail, detail_len)) {
                ap6256_connectivity_set_wifi_note(detail);
                ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE,
                                                                  AP6256_STATUS_IO_ERROR);
                return AP6256_STATUS_IO_ERROR;
            }
            goto scan_start_retry;
        }

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
    cyw43_state.wifi_scan_state = 0;
    cyw43_state.wifi_scan_cb = NULL;
    cyw43_state.wifi_scan_env = NULL;
    for (uint32_t drain_ms = 0U; drain_ms < 500U; drain_ms += 20U) {
        ap6256_cyw43_port_poll();
        osDelay(20U);
    }

    /*
     * Keep the extra forced-band scan as an opt-in bench diagnostic. Hidden
     * 5 GHz SSIDs are handled by the manual selection path below; forcing a
     * second scan before the operator chooses a network can perturb the next
     * join control path on AP6256.
     */
#if AP6256_WIFI_ENABLE_FORCED_5G_SCAN
    if (ap6256_wifi_runtime_has_5g_scan_result() == 0U) {
        int band_rc;
        uint8_t band_recovery_needed = 0U;

        test_uart_write_str("[ INFO ] wifi.connect stage: start 5GHz scan\r\n");
        band_rc = ap6256_wifi_runtime_set_ioctl_u32(AP6256_CYW43_IOCTL_SET_BAND, 1U); /* WLC_BAND_5G */
        test_uart_printf("[ INFO ] wifi.connect stage: force 5GHz band rc=%d\r\n", band_rc);
        if (band_rc == 0) {
            memset(&opts, 0, sizeof(opts));
            opts.scan_type = 0;
            opts.channel_num = AP6256_WIFI_SCAN_FORCE_5G;
            if (s_wifi_runtime.has_cached_profile != 0U) {
                size_t cached_len = strlen(s_wifi_runtime.cached_ssid);

                if ((cached_len > 0U) && (cached_len <= sizeof(opts.ssid))) {
                    opts.ssid_len = (uint32_t)cached_len;
                    memcpy(opts.ssid, s_wifi_runtime.cached_ssid, cached_len);
                    test_uart_printf("[ INFO ] wifi.connect stage: directed 5GHz scan ssid=%s\r\n",
                                     s_wifi_runtime.cached_ssid);
                }
            }
            rc = cyw43_wifi_scan(&cyw43_state, &opts, NULL, ap6256_wifi_runtime_scan_cb);
            if (rc == 0) {
                ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SCAN_ACCEPTED, 5);
                if (!ap6256_wifi_runtime_wait_for_scan_complete(AP6256_WIFI_SCAN_TIMEOUT_MS)) {
                    test_uart_write_str("[ INFO ] wifi.connect stage: 5GHz scan timeout\r\n");
                }
                cyw43_state.wifi_scan_state = 0;
                cyw43_state.wifi_scan_cb = NULL;
                cyw43_state.wifi_scan_env = NULL;
                for (uint32_t drain_ms = 0U; drain_ms < 500U; drain_ms += 20U) {
                    ap6256_cyw43_port_poll();
                    osDelay(20U);
                }
            } else {
                test_uart_printf("[ INFO ] wifi.connect stage: 5GHz scan start failed rc=%d\r\n", rc);
            }
        } else {
            test_uart_write_str("[ INFO ] wifi.connect stage: skip 5GHz scan after band switch failure\r\n");
            band_recovery_needed = 1U;
        }
        band_rc = ap6256_wifi_runtime_set_ioctl_u32(AP6256_CYW43_IOCTL_SET_BAND, 0U); /* WLC_BAND_AUTO */
        test_uart_printf("[ INFO ] wifi.connect stage: restore auto band rc=%d\r\n", band_rc);
        if (band_rc != 0) {
            band_recovery_needed = 1U;
        }
        if (band_recovery_needed != 0U) {
            test_uart_write_str("[ INFO ] wifi.connect stage: recover after band restore failure\r\n");
            ap6256_wifi_runtime_suspend();
            if (!ap6256_wifi_runtime_ensure_ready(detail, detail_len)) {
                ap6256_connectivity_set_wifi_note(detail);
                ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE,
                                                                  AP6256_STATUS_IO_ERROR);
                return AP6256_STATUS_IO_ERROR;
            }
        }
    }
#else
    if (ap6256_wifi_runtime_has_5g_scan_result() == 0U) {
        test_uart_write_str("[ INFO ] wifi.connect stage: no visible 5GHz BSSID; hidden 5GHz SSID may be entered manually\r\n");
    }
#endif

    ap6256_wifi_runtime_sort_scan_results();
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SCAN_COMPLETE,
                                        s_wifi_runtime.last_scan_count);
    ap6256_connectivity_set_wifi_runtime(1U, 1U, s_wifi_runtime.last_scan_count, s_wifi_runtime.last_rssi);
    if (summary != NULL) {
        memset(summary, 0, sizeof(*summary));
        summary->scan_results_count = s_wifi_runtime.last_scan_count;
        summary->selected_rssi = s_wifi_runtime.last_rssi;
    }

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
    scan_index = ap6256_wifi_runtime_prompt_network_selection();
    if (scan_index < 0) {
        (void)snprintf(detail, detail_len, "Wi-Fi network selection timed out or was invalid.");
        ap6256_connectivity_set_wifi_note(detail);
        ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE, AP6256_STATUS_BAD_PARAM);
        return AP6256_STATUS_BAD_PARAM;
    }

    selected = &s_wifi_runtime.scan[(uint32_t)scan_index];
    if (selected->channel == AP6256_WIFI_CHANNEL_5G_UNKNOWN) {
        char hidden_ssid[33];

        (void)snprintf(hidden_ssid, sizeof(hidden_ssid), "%s", selected->ssid);
        selected = ap6256_wifi_runtime_resolve_hidden_5g_selection(hidden_ssid, detail, detail_len);
        if (selected == NULL) {
            ap6256_connectivity_set_wifi_note(detail);
            ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE,
                                                              AP6256_STATUS_TIMEOUT);
            return AP6256_STATUS_TIMEOUT;
        }
        fixture_classification = "directed_5g_bss";
    } else if (s_wifi_runtime_selection_was_index != 0U) {
        /*
         * A numbered selection is an explicit BSSID choice. Respect it like
         * cfg80211/brcmfmac would; SSID text entry is the path that asks the
         * manufacturing flow to prefer/qualify a 5 GHz BSS when one exists.
         */
        fixture_classification =
            (ap6256_wifi_runtime_channel_is_5g(selected->channel) != 0U) ?
                "selected_5g_bss" : "selected_exact_bss";
    } else {
        selected_copy = *selected;
        selected = ap6256_wifi_runtime_prefer_5g_selection(&selected_copy,
                                                           detail,
                                                           detail_len,
                                                           &fixture_classification);
        if (selected == NULL) {
            (void)snprintf(detail, detail_len, "Wi-Fi selection could not be resolved.");
            ap6256_connectivity_set_wifi_note(detail);
            ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE,
                                                              AP6256_STATUS_BAD_PARAM);
            return AP6256_STATUS_BAD_PARAM;
        }
    }
    ap6256_connectivity_set_wifi_selection_diag(selected->bssid,
                                                selected->channel,
                                                ap6256_wifi_runtime_channel_is_5g(selected->channel),
                                                fixture_classification);
    ap6256_connectivity_set_wifi_selection_security_diag(selected->auth_mode,
                                                         selected->security_flags,
                                                         selected->akm_flags,
                                                         selected->pairwise_cipher_flags,
                                                         selected->group_cipher_flags,
                                                         selected->mfp,
                                                         selected->rsn_cap,
                                                         selected->chanspec);
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
    {
        char unsupported_reason[64];
        unsupported_reason[0] = '\0';
        if (ap6256_wifi_runtime_scan_security_supported(selected,
                                                        unsupported_reason,
                                                        sizeof(unsupported_reason)) == 0U) {
            (void)snprintf(detail,
                           detail_len,
                           "Selected SSID '%s' uses %s; sec=%s akm=%s pair=%s mfp=%s.",
                           selected->ssid,
                           unsupported_reason,
                           ap6256_wifi_runtime_security_name(selected->auth_mode),
                           ap6256_wifi_runtime_akm_name(selected->akm_flags),
                           ap6256_wifi_runtime_cipher_name(selected->pairwise_cipher_flags),
                           ap6256_wifi_runtime_mfp_name(selected->mfp));
            ap6256_connectivity_set_wifi_note(detail);
            ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE,
                                                              AP6256_STATUS_BAD_PARAM);
            return AP6256_STATUS_BAD_PARAM;
        }
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

    selected_copy = *selected;
    selected = &selected_copy;
    if (ap6256_wifi_runtime_channel_is_5g(selected->channel) != 0U) {
        /*
         * Hidden 5 GHz association depends on the firmware context from the
         * directed scan that just found the BSSID. Keep that session alive and
         * let the brcmfmac-style join path carry BSSID/chanspec into connect.
         */
        test_uart_write_str("[ INFO ] wifi.connect stage: keep directed 5GHz scan context for join\r\n");
    } else {
        /*
         * BCM43456 delivers scan results through async ESCAN events, but the
         * first normal control iovar after a visible scan can stall while
         * firmware is unwinding scan state. Restart for the 2.4 GHz/visible path
         * where the selected BSSID/channel is enough to rejoin deterministically.
         */
        test_uart_write_str("[ INFO ] wifi.connect stage: recover radio after scan before join\r\n");
        ap6256_wifi_runtime_suspend();
        if (!ap6256_wifi_runtime_ensure_ready(detail, detail_len)) {
            ap6256_connectivity_set_wifi_note(detail);
            ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE,
                                                              AP6256_STATUS_IO_ERROR);
            return AP6256_STATUS_IO_ERROR;
        }
    }

    test_uart_write_str("[ INFO ] wifi.connect stage: join start\r\n");
    ap6256_wifi_runtime_capture_profile(selected, password);
    {
        ap6256_status_t st = ap6256_wifi_runtime_run_common(selected->ssid,
                                                            password,
                                                            selected->secure,
                                                            selected->bssid,
                                                            selected->channel,
                                                            summary,
                                                            detail,
                                                            detail_len);
        ap6256_wifi_runtime_set_poll_paused(0U);
        ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE, st);
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
    ap6256_connectivity_set_wifi_selection_diag(s_wifi_runtime.cached_bssid,
                                                s_wifi_runtime.cached_channel,
                                                ap6256_wifi_runtime_channel_is_5g(s_wifi_runtime.cached_channel),
                                                "cached_profile");
    ap6256_connectivity_set_wifi_selection_security_diag(s_wifi_runtime.cached_secure ? 0x04U : 0U,
                                                         s_wifi_runtime.cached_secure ? CYW43_SCAN_SEC_RSN : 0U,
                                                         s_wifi_runtime.cached_secure ? CYW43_SCAN_AKM_PSK : 0U,
                                                         s_wifi_runtime.cached_secure ? CYW43_SCAN_CIPHER_CCMP : 0U,
                                                         s_wifi_runtime.cached_secure ? CYW43_SCAN_CIPHER_CCMP : 0U,
                                                         CYW43_SCAN_MFP_NONE,
                                                         0U,
                                                         0U);
    {
        ap6256_status_t st = ap6256_wifi_runtime_run_common(s_wifi_runtime.cached_ssid,
                                                            s_wifi_runtime.cached_password,
                                                            s_wifi_runtime.cached_secure,
                                                            s_wifi_runtime.cached_bssid,
                                                            s_wifi_runtime.cached_channel,
                                                            summary,
                                                            detail,
                                                            detail_len);
        ap6256_wifi_runtime_set_poll_paused(0U);
        ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE, st);
        return st;
    }
}
