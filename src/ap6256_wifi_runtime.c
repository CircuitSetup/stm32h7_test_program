#include "ap6256_wifi_runtime.h"

#include "ap6256_connectivity.h"
#include "ap6256_cyw43_compat.h"
#include "ap6256_cyw43_port.h"
#include "ap6256_assets.h"
#include "cyw43.h"
#include "cyw43_country.h"
#include "cyw43_ll.h"
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
#define AP6256_WIFI_SCAN_QUIET_COMPLETE_MS 3000U
#define AP6256_WIFI_SCAN_QUIET_STABLE_MS 1500U
#define AP6256_WIFI_SCAN_NO_PROGRESS_TIMEOUT_MS 10000U
#define AP6256_WIFI_SCAN_FORCE_5G         (-5)
#define AP6256_WIFI_CHANNEL_5G_UNKNOWN    0xFFFFU
#define AP6256_WIFI_SCAN_RECOVERY_F2_BLOCK_SIZE 64U
#define AP6256_WIFI_JOIN_F2_BLOCK_SIZE     512U
#define AP6256_WIFI_DIRECTED_5G_SCAN_TIMEOUT_MS 15000U
#define AP6256_WIFI_PROFILE_BROAD_SCAN_TIMEOUT_MS 8000U
/*
 * BCM43456 5 GHz association can take multiple seconds after scan cleanup,
 * channel priming, and WPA2/transition-mode security setup. A 1.2 s guard was
 * useful for catching resets, but it cuts off normal FullMAC auth/assoc before
 * firmware has a chance to emit LINK/PSK evidence.
 */
#define AP6256_WIFI_5G_JOIN_NO_PROGRESS_MS 1500U
#define AP6256_WIFI_MAX_JOIN_CANDIDATES   AP6256_WIFI_MAX_SCAN_RESULTS
#define AP6256_CYW43_IOCTL_SET_BAND       ((142U << 1U) | 1U)
#define AP6256_CYW43_IOCTL_SET_CHANNEL    ((30U << 1U) | 1U)
#define AP6256_CYW43_JOIN_STATE_KEYED     0x0800U
#define AP6256_CYW43_JOIN_STATE_AUTH      0x0200U
#define AP6256_CYW43_JOIN_STATE_LINK      0x0400U
#define AP6256_CYW43_JOIN_STATE_PROGRESS  (AP6256_CYW43_JOIN_STATE_AUTH | \
                                           AP6256_CYW43_JOIN_STATE_LINK | \
                                           AP6256_CYW43_JOIN_STATE_KEYED)

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
    uint8_t cached_auth_mode;
    uint8_t cached_mfp;
    uint8_t cached_security_flags;
    uint8_t last_scan_count;
    int16_t last_rssi;
    uint8_t cached_bssid[6];
    uint16_t cached_channel;
    uint16_t cached_chanspec;
    uint16_t cached_akm_flags;
    uint16_t cached_pairwise_cipher_flags;
    uint16_t cached_group_cipher_flags;
    char cached_ssid[33];
    char cached_password[65];
    char last_ip[16];
    char last_mask[16];
    char last_gateway[16];
    ap6256_wifi_scan_entry_t scan[AP6256_WIFI_MAX_SCAN_RESULTS];
} ap6256_wifi_runtime_state_t;

static ap6256_wifi_runtime_state_t s_wifi_runtime;
static ap6256_wifi_scan_entry_t s_wifi_runtime_join_candidates[AP6256_WIFI_MAX_JOIN_CANDIDATES];
static uint8_t s_wifi_runtime_selection_was_index;
static uint8_t s_wifi_runtime_join_candidate_index;
static uint8_t s_wifi_runtime_join_candidate_count;

static const char *ap6256_wifi_runtime_packet_source_name(uint32_t source);
static const char *ap6256_wifi_runtime_rx_class_name(uint32_t rx_class);
static int ap6256_wifi_runtime_probe_associated_bssid(uint8_t bssid_out[6]);
static int ap6256_wifi_runtime_set_ioctl_u32(uint32_t cmd, uint32_t value) __attribute__((unused));
static bool ap6256_wifi_runtime_wait_for_scan_complete_ex(uint32_t timeout_ms,
                                                          uint8_t accept_partial_on_timeout);
static bool ap6256_wifi_runtime_wait_for_scan_complete(uint32_t timeout_ms);
static void ap6256_wifi_runtime_reset_driver_state(void);
static void ap6256_wifi_runtime_sort_join_candidates(ap6256_wifi_scan_entry_t *candidates,
                                                     uint32_t candidate_count);

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

static bool ap6256_wifi_runtime_prepare_scan_sdio_policy(char *detail, size_t detail_len)
{
    int32_t block_rc;

    /*
     * Hardware logs showed AP6256 scan events only after CYW43 was brought up
     * with 64-byte F2 blocks. Switching an already-running 512-byte session to
     * 64 bytes is not equivalent on this firmware, so make the scan policy the
     * bus-init default and switch back to 512 only for 5 GHz association.
     */
    ap6256_cyw43_port_set_runtime_f2_block_size(AP6256_WIFI_SCAN_RECOVERY_F2_BLOCK_SIZE);
    if (s_wifi_runtime.initialized == 0U) {
        return true;
    }

    block_rc = ap6256_cyw43_port_apply_runtime_f2_block_size();
    if (block_rc != 0) {
        if ((detail != NULL) && (detail_len != 0U)) {
            (void)snprintf(detail,
                           detail_len,
                           "Failed to apply scan F2 block size %u rc=%ld.",
                           (unsigned)AP6256_WIFI_SCAN_RECOVERY_F2_BLOCK_SIZE,
                           (long)block_rc);
        }
        return false;
    }
    return true;
}

static bool ap6256_wifi_runtime_prepare_join_sdio_policy(char *detail, size_t detail_len)
{
    /*
     * Known-good AP6256 2.4 GHz association did not reprogram the live F2
     * block-size policy between scan and WLC_SET_SSID. Leave the bus in the
     * exact mode that completed scan; changing F2 framing here has repeatedly
     * made association nondeterministic.
     */
    (void)detail;
    (void)detail_len;
    return true;
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

static uint16_t ap6256_wifi_runtime_primary_chanspec(uint16_t channel)
{
    if (channel == 0U || channel == AP6256_WIFI_CHANNEL_5G_UNKNOWN) {
        return 0U;
    }

    return (uint16_t)(channel |
                      0x1000U |
                      ((ap6256_wifi_runtime_channel_is_5g(channel) != 0U) ? 0xC000U : 0U));
}

static uint8_t ap6256_wifi_runtime_primary_channel_from_chanspec(uint16_t chanspec)
{
    uint8_t center_or_primary = (uint8_t)(chanspec & 0x00FFU);
    uint8_t ctl_sb = (uint8_t)((chanspec >> 8U) & 0x07U);
    uint16_t bw = (uint16_t)(chanspec & 0x3800U);

    if (center_or_primary == 0U) {
        return 0U;
    }

    if ((bw == 0x2000U) && (center_or_primary > 6U)) {
        /*
         * D11ac 80 MHz chanspecs carry the center channel in bits 0..7 and the
         * control sideband in bits 8..10. Convert examples seen from BCM43456:
         * 0xE09B -> primary 149, 0xE19B -> 153, 0xE23A -> 60.
         */
        return (uint8_t)(center_or_primary - 6U + (ctl_sb * 4U));
    }

    return center_or_primary;
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

static ap6256_wifi_scan_entry_t *ap6256_wifi_runtime_find_best_ssid_any(const char *ssid)
{
    ap6256_wifi_scan_entry_t *best = NULL;

    if (ssid == NULL) {
        return NULL;
    }

    for (uint32_t i = 0U; i < AP6256_WIFI_MAX_SCAN_RESULTS; ++i) {
        if ((s_wifi_runtime.scan[i].valid == 0U) ||
            (strcmp(s_wifi_runtime.scan[i].ssid, ssid) != 0)) {
            continue;
        }

        if ((best == NULL) || (s_wifi_runtime.scan[i].rssi > best->rssi)) {
            best = &s_wifi_runtime.scan[i];
        }
    }

    return best;
}

static ap6256_wifi_scan_entry_t *ap6256_wifi_runtime_find_best_ssid_channel(const char *ssid,
                                                                            uint16_t channel)
{
    ap6256_wifi_scan_entry_t *best = NULL;

    if ((ssid == NULL) || (channel == 0U)) {
        return NULL;
    }

    for (uint32_t i = 0U; i < s_wifi_runtime.last_scan_count; ++i) {
        ap6256_wifi_scan_entry_t *entry = &s_wifi_runtime.scan[i];

        if ((entry->channel != channel) ||
            (strcmp(entry->ssid, ssid) != 0)) {
            continue;
        }
        if ((best == NULL) || (entry->rssi > best->rssi)) {
            best = entry;
        }
    }

    return best;
}

static uint32_t ap6256_wifi_runtime_collect_ssid_candidates(const char *ssid,
                                                            uint16_t preferred_channel,
                                                            uint8_t require_5g,
                                                            ap6256_wifi_scan_entry_t *candidates,
                                                            uint32_t max_candidates,
                                                            uint32_t *unsupported_count)
{
    uint32_t count = 0U;

    if (unsupported_count != NULL) {
        *unsupported_count = 0U;
    }
    if ((ssid == NULL) || (candidates == NULL) || (max_candidates == 0U)) {
        return 0U;
    }

    /*
     * s_wifi_runtime.scan is sorted by RSSI before this helper is called. Copy
     * candidates in that order so the join loop tries the strongest legal
     * BSS first, matching the cfg80211/brcmfmac selection model.
     */
    for (uint32_t i = 0U; i < AP6256_WIFI_MAX_SCAN_RESULTS; ++i) {
        char unsupported_reason[64];
        const ap6256_wifi_scan_entry_t *entry = &s_wifi_runtime.scan[i];

        if ((entry->valid == 0U) ||
            (strcmp(entry->ssid, ssid) != 0)) {
            continue;
        }
        if ((require_5g != 0U) &&
            (ap6256_wifi_runtime_channel_is_5g(entry->channel) == 0U)) {
            continue;
        }
        if ((preferred_channel != 0U) && (entry->channel != preferred_channel)) {
            continue;
        }

        unsupported_reason[0] = '\0';
        if (ap6256_wifi_runtime_scan_security_supported(entry,
                                                        unsupported_reason,
                                                        sizeof(unsupported_reason)) == 0U) {
            if (unsupported_count != NULL) {
                (*unsupported_count)++;
            }
            test_uart_printf("[ INFO ] wifi.connect stage: skip unsupported BSS ssid=%s ch=%u/%s bssid=%02X:%02X:%02X:%02X:%02X:%02X reason=%s\r\n",
                             entry->ssid,
                             entry->channel,
                             ap6256_wifi_runtime_channel_band_name(entry->channel),
                             entry->bssid[0],
                             entry->bssid[1],
                             entry->bssid[2],
                             entry->bssid[3],
                             entry->bssid[4],
                             entry->bssid[5],
                             unsupported_reason);
            continue;
        }

        candidates[count++] = *entry;
        if (count >= max_candidates) {
            break;
        }
    }

    ap6256_wifi_runtime_sort_join_candidates(candidates, count);
    return count;
}

static uint32_t ap6256_wifi_runtime_collect_ssid_5g_candidates(const char *ssid,
                                                               uint16_t preferred_channel,
                                                               ap6256_wifi_scan_entry_t *candidates,
                                                               uint32_t max_candidates,
                                                               uint32_t *unsupported_count)
{
    return ap6256_wifi_runtime_collect_ssid_candidates(ssid,
                                                       preferred_channel,
                                                       1U,
                                                       candidates,
                                                       max_candidates,
                                                       unsupported_count);
}

static void ap6256_wifi_runtime_sort_join_candidates(ap6256_wifi_scan_entry_t *candidates,
                                                     uint32_t candidate_count)
{
    if (candidates == NULL) {
        return;
    }

    for (uint32_t i = 0U; i < candidate_count; ++i) {
        for (uint32_t j = i + 1U; j < candidate_count; ++j) {
            /*
             * brcmfmac/cfg80211 try the selected BSS, which normally means
             * the strongest compatible candidate. Keep that deterministic order
             * and let the join state machine move to the next candidate only
             * after a classified failure.
             */
            if (candidates[j].rssi > candidates[i].rssi) {
                ap6256_wifi_scan_entry_t tmp = candidates[i];
                candidates[i] = candidates[j];
                candidates[j] = tmp;
            }
        }
    }
}

static void ap6256_wifi_runtime_print_join_candidates(const char *ssid,
                                                      const ap6256_wifi_scan_entry_t *candidates,
                                                      uint32_t candidate_count)
{
    test_uart_printf("[ INFO ] wifi.connect stage: join candidates ssid=%s count=%lu\r\n",
                     (ssid != NULL) ? ssid : "n/a",
                     (unsigned long)candidate_count);
    for (uint32_t i = 0U; i < candidate_count; ++i) {
        const ap6256_wifi_scan_entry_t *entry = &candidates[i];

        test_uart_printf("[ INFO ] wifi.connect stage: candidate %lu/%lu bssid=%02X:%02X:%02X:%02X:%02X:%02X ch=%u/%s rssi=%d sec=%s akm=%s pair=%s grp=%s mfp=%s cs=0x%04X\r\n",
                         (unsigned long)(i + 1U),
                         (unsigned long)candidate_count,
                         entry->bssid[0],
                         entry->bssid[1],
                         entry->bssid[2],
                         entry->bssid[3],
                         entry->bssid[4],
                         entry->bssid[5],
                         entry->channel,
                         ap6256_wifi_runtime_channel_band_name(entry->channel),
                         (int)entry->rssi,
                         ap6256_wifi_runtime_security_name(entry->auth_mode),
                         ap6256_wifi_runtime_akm_name(entry->akm_flags),
                         ap6256_wifi_runtime_cipher_name(entry->pairwise_cipher_flags),
                         ap6256_wifi_runtime_cipher_name(entry->group_cipher_flags),
                         ap6256_wifi_runtime_mfp_name(entry->mfp),
                         entry->chanspec);
    }
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

static int ap6256_wifi_runtime_add_manual_directed_5g_network(const char *ssid)
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
    test_uart_printf("[ INFO ] wifi.connect stage: manual directed 5GHz SSID '%s'\r\n", ssid);
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

    test_uart_write_str("Select Wi-Fi network number or SSID (directed 5GHz SSID allowed): ");
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

    return ap6256_wifi_runtime_add_manual_directed_5g_network(line);
}

static ap6256_wifi_scan_entry_t *ap6256_wifi_runtime_resolve_directed_5g_selection_channel(const char *ssid,
                                                                                           uint16_t preferred_channel,
                                                                                           char *detail,
                                                                                           size_t detail_len)
{
    cyw43_wifi_scan_options_t opts;
    ap6256_wifi_scan_entry_t *resolved;
    int rc;

    if ((ssid == NULL) || (ssid[0] == '\0')) {
        return NULL;
    }

    if (preferred_channel > 14U) {
        test_uart_printf("[ INFO ] wifi.connect stage: directed 5GHz SSID scan ssid=%s ch=%u\r\n",
                         ssid,
                         (unsigned)preferred_channel);
    } else {
        test_uart_printf("[ INFO ] wifi.connect stage: directed 5GHz SSID scan ssid=%s\r\n", ssid);
    }

    ap6256_wifi_runtime_clear_scan_results();
    memset(&opts, 0, sizeof(opts));
    opts.scan_type = 0;
    opts.channel_num = (preferred_channel > 14U) ? (int32_t)preferred_channel
                                                 : AP6256_WIFI_SCAN_FORCE_5G;
    opts.ssid_len = (uint32_t)strnlen(ssid, sizeof(opts.ssid));
    if ((opts.ssid_len == 0U) || (opts.ssid_len > sizeof(opts.ssid))) {
        (void)snprintf(detail, detail_len, "Invalid directed 5GHz SSID '%s'.", ssid);
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
                       "Directed 5GHz scan for SSID '%s' failed to start (rc=%d).",
                       ssid,
                       rc);
        return NULL;
    }

    if (!ap6256_wifi_runtime_wait_for_scan_complete_ex(AP6256_WIFI_DIRECTED_5G_SCAN_TIMEOUT_MS, 1U)) {
        cyw43_state.wifi_scan_state = 0;
        cyw43_state.wifi_scan_cb = NULL;
        cyw43_state.wifi_scan_env = NULL;
        (void)snprintf(detail,
                       detail_len,
                       (preferred_channel > 14U) ?
                           "5GHz SSID '%s' did not respond during directed scan on requested channel." :
                           "5GHz SSID '%s' did not respond during directed scan.",
                       ssid);
        return NULL;
    }

    cyw43_state.wifi_scan_state = 0;
    cyw43_state.wifi_scan_cb = NULL;
    cyw43_state.wifi_scan_env = NULL;
    ap6256_wifi_runtime_sort_scan_results();
    resolved = (preferred_channel > 14U) ?
        ap6256_wifi_runtime_find_best_ssid_channel(ssid, preferred_channel) :
        ap6256_wifi_runtime_find_best_ssid_5g(ssid);
    if (resolved == NULL) {
        (void)snprintf(detail,
                       detail_len,
                       "5GHz SSID '%s' was not found by directed scan; join was not attempted.",
                       ssid);
        return NULL;
    }

    test_uart_printf("[ INFO ] wifi.connect stage: directed 5GHz resolved bssid=%02X:%02X:%02X:%02X:%02X:%02X ch=%u rssi=%d sec=%s akm=%s pair=%s grp=%s mfp=%s cs=0x%04X\r\n",
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

static ap6256_wifi_scan_entry_t *ap6256_wifi_runtime_resolve_directed_5g_selection(const char *ssid,
                                                                                   char *detail,
                                                                                   size_t detail_len)
{
    return ap6256_wifi_runtime_resolve_directed_5g_selection_channel(ssid,
                                                                     0U,
                                                                     detail,
                                                                     detail_len);
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
    directed_5g = ap6256_wifi_runtime_resolve_directed_5g_selection(fallback->ssid,
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

static bool ap6256_wifi_runtime_wait_for_scan_complete_ex(uint32_t timeout_ms,
                                                          uint8_t accept_partial_on_timeout)
{
    uint32_t start_ms = HAL_GetTick();
    uint32_t last_diag_ms = start_ms;

    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SCAN_WAIT, 0);
    while (cyw43_wifi_scan_active(&cyw43_state)) {
        uint32_t now_ms;

        ap6256_cyw43_port_poll();
        now_ms = HAL_GetTick();

        if ((ap6256_cyw43_port_poll_header_read_status() == -CYW43_ETIMEDOUT) ||
            (ap6256_cyw43_port_poll_payload_read_status() == -CYW43_ETIMEDOUT)) {
            return false;
        }

        if ((now_ms - start_ms) >= timeout_ms) {
            if ((accept_partial_on_timeout != 0U) &&
                (ap6256_wifi_runtime_count_scan_results() > 0U)) {
                cyw43_state.wifi_scan_state = 2;
                return true;
            }
            return false;
        }
        if ((now_ms - last_diag_ms) >= 1000U) {
            last_diag_ms = now_ms;
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

static bool ap6256_wifi_runtime_wait_for_scan_complete(uint32_t timeout_ms)
{
    return ap6256_wifi_runtime_wait_for_scan_complete_ex(timeout_ms, 1U);
}

static void ap6256_wifi_runtime_poll_burst(uint32_t polls, uint32_t delay_every)
{
    for (uint32_t i = 0; i < polls; ++i) {
        ap6256_cyw43_port_poll();
        /*
         * Association and DHCP are event/data driven. A short yield keeps the
         * tcpip_thread and SDIO ISR-side bookkeeping moving without adding a
         * long blind sleep between F2 drains.
         */
        if ((delay_every != 0U) && (((i + 1U) % delay_every) == 0U)) {
            osDelay(1U);
        }
    }
}

static void ap6256_wifi_runtime_quiesce_scan_events(const char *reason)
{
    uint32_t start_ms = HAL_GetTick();
    uint32_t quiet_start_ms = start_ms;
    uint32_t last_event_count = ap6256_cyw43_port_async_event_count();

    test_uart_printf("[ INFO ] wifi.connect stage: scan event quiesce %s\r\n",
                     (reason != NULL) ? reason : "before join");
    while ((HAL_GetTick() - start_ms) < AP6256_WIFI_SCAN_QUIET_COMPLETE_MS) {
        uint32_t event_count;

        ap6256_cyw43_port_poll();
        event_count = ap6256_cyw43_port_async_event_count();
        if (event_count != last_event_count) {
            last_event_count = event_count;
            quiet_start_ms = HAL_GetTick();
        } else if ((HAL_GetTick() - quiet_start_ms) >= AP6256_WIFI_SCAN_QUIET_STABLE_MS) {
            break;
        }
        osDelay(20U);
    }
}

static void ap6256_wifi_runtime_restart_dhcp_after_link(struct netif *sta_netif)
{
    ip4_addr_t zero;

    if (sta_netif == NULL) {
        return;
    }

    IP4_ADDR(&zero, 0, 0, 0, 0);
    (void)netifapi_dhcp_release_and_stop(sta_netif);
    (void)netifapi_netif_set_addr(sta_netif, &zero, &zero, &zero);
    (void)netifapi_netif_set_up(sta_netif);
    (void)netifapi_netif_set_link_up(sta_netif);
    (void)netifapi_dhcp_start(sta_netif);
}

static bool ap6256_wifi_runtime_wait_for_link(uint32_t timeout_ms,
                                              int *final_status,
                                              const uint8_t *target_bssid,
                                              uint8_t secure,
                                              uint8_t selected_5g)
{
    uint32_t start_ms = HAL_GetTick();
    uint32_t last_diag_ms = start_ms;
    uint8_t dhcp_restart_done = 0U;
    uint8_t assoc_forced_link = 0U;
    uint8_t last_assoc_seen = 0U;
    uint8_t last_assoc_matches = 0U;
    uint8_t last_assoc_bssid[6];
    uint32_t last_assoc_probe_ms = 0U;
    uint32_t start_event_count = ap6256_cyw43_port_join_event_count();

    memset(last_assoc_bssid, 0, sizeof(last_assoc_bssid));
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_JOIN_WAIT, 0);

    if (final_status != NULL) {
        *final_status = CYW43_LINK_DOWN;
    }

    if (selected_5g != 0U) {
        /*
         * BCM43456 can reset this board when the host immediately polls F2
         * after starting a 5 GHz association. Give firmware a quiet
         * window to process the association request before the event/data drain
         * loop starts; success is still proved by events/BSSID/DHCP below.
         */
        ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_JOIN_WAIT, -300);
        osDelay(300U);
    }

    while ((HAL_GetTick() - start_ms) < timeout_ms) {
        uint32_t now_ms;
        int status;

        ap6256_wifi_runtime_poll_burst(4U, 2U);

        now_ms = HAL_GetTick();
        status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);

        if (final_status != NULL) {
            *final_status = status;
        }

        ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_JOIN_WAIT, status);

        if (status == CYW43_LINK_UP) {
            ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_JOIN_RESULT, status);
            return true;
        }

        /*
         * brcmfmac advances connection completion from firmware evidence
         * rather than requiring one exact event order. BCM43456/AP6256 5 GHz
         * joins can confirm association via GET_BSSID and F2 traffic while the
         * CYW43 43439 state machine remains at LINK_JOIN. Once the selected
         * BSSID is confirmed, bring the lwIP link up so DHCP/EAPOL/data can
         * flow; auth failures still win via PSK/AUTH/DEAUTH events below.
         */
        uint8_t post_join_event_seen =
            (ap6256_cyw43_port_join_event_count() != start_event_count) ? 1U : 0U;
        uint8_t post_join_link_evidence =
            ((cyw43_state.wifi_join_state &
              (AP6256_CYW43_JOIN_STATE_LINK | AP6256_CYW43_JOIN_STATE_KEYED)) != 0U) ? 1U : 0U;

        if ((status == CYW43_LINK_JOIN) &&
            (assoc_forced_link == 0U) &&
            (dhcp_restart_done == 0U) &&
            (((selected_5g == 0U) &&
              ((now_ms - start_ms) >= 250U)) ||
             (((cyw43_state.wifi_join_state & AP6256_CYW43_JOIN_STATE_PROGRESS) != 0U) &&
              ((post_join_event_seen != 0U) ||
               (post_join_link_evidence != 0U)))) &&
            ((now_ms - start_ms) >= 100U) &&
            ((now_ms - last_assoc_probe_ms) >= 250U)) {
            uint8_t assoc_bssid[6];
            int assoc_seen;

            last_assoc_probe_ms = now_ms;
            memset(assoc_bssid, 0, sizeof(assoc_bssid));
            assoc_seen = ap6256_wifi_runtime_probe_associated_bssid(assoc_bssid);
            last_assoc_seen = (assoc_seen != 0) ? 1U : 0U;
            if (last_assoc_seen != 0U) {
                memcpy(last_assoc_bssid, assoc_bssid, sizeof(last_assoc_bssid));
                last_assoc_matches =
                    ((target_bssid == NULL) ||
                     (ap6256_wifi_runtime_bssid_matches(assoc_bssid, target_bssid) != 0)) ? 1U : 0U;
            } else {
                last_assoc_matches = 0U;
            }

            if ((last_assoc_matches != 0U) &&
                (assoc_forced_link == 0U) &&
                ((secure == 0U) ||
                 ((cyw43_state.wifi_join_state & AP6256_CYW43_JOIN_STATE_KEYED) != 0U)) &&
                (status == CYW43_LINK_JOIN)) {
                struct netif *sta_netif = &cyw43_state.netif[CYW43_ITF_STA];

                assoc_forced_link = 1U;
                dhcp_restart_done = 1U;
                test_uart_printf("[ INFO ] wifi.connect stage: association confirmed by BSSID; start DHCP bssid=%02X:%02X:%02X:%02X:%02X:%02X join=0x%08lX rx=%s/%u\r\n",
                                 assoc_bssid[0],
                                 assoc_bssid[1],
                                 assoc_bssid[2],
                                 assoc_bssid[3],
                                 assoc_bssid[4],
                                 assoc_bssid[5],
                                 (unsigned long)cyw43_state.wifi_join_state,
                                 ap6256_wifi_runtime_rx_class_name(ap6256_cyw43_port_last_rx_class()),
                                 ap6256_cyw43_port_last_rx_payload_len());
                cyw43_cb_tcpip_set_link_up(&cyw43_state, CYW43_ITF_STA);
                ap6256_wifi_runtime_restart_dhcp_after_link(sta_netif);
                ap6256_wifi_runtime_poll_burst(24U, 4U);
                status = CYW43_LINK_NOIP;
                if (final_status != NULL) {
                    *final_status = status;
                }
            }
        }

        if ((status == CYW43_LINK_NOIP) && (dhcp_restart_done == 0U)) {
            struct netif *sta_netif = &cyw43_state.netif[CYW43_ITF_STA];

            dhcp_restart_done = 1U;
            test_uart_write_str("[ INFO ] wifi.connect stage: restart DHCP after Wi-Fi link\r\n");
            ap6256_wifi_runtime_restart_dhcp_after_link(sta_netif);
            ap6256_wifi_runtime_poll_burst(24U, 4U);
        } else if (status == CYW43_LINK_NOIP) {
            /*
             * DHCP is pure data/event traffic. Do not keep injecting GET_BSSID
             * control ioctls once association is proven; keep F2 drained and
             * let lwIP's DHCP timer drive retransmits.
             */
            ap6256_wifi_runtime_poll_burst(16U, 4U);
        }

        if ((status == CYW43_LINK_FAIL) ||
            (status == CYW43_LINK_NONET) ||
            (status == CYW43_LINK_BADAUTH)) {
            ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_JOIN_RESULT, status);
            return false;
        }

        if ((selected_5g != 0U) &&
            (post_join_link_evidence == 0U) &&
            (last_assoc_seen == 0U) &&
            ((now_ms - start_ms) >= AP6256_WIFI_5G_JOIN_NO_PROGRESS_MS)) {
            if (final_status != NULL) {
                *final_status = CYW43_LINK_NONET;
            }
            test_uart_printf("[ INFO ] wifi.connect stage: 5GHz join no progress %lums status=%d join=0x%08lX ev=%lu/%lu/%lu rx=%s/%u; try next candidate\r\n",
                             (unsigned long)(now_ms - start_ms),
                             status,
                             (unsigned long)cyw43_state.wifi_join_state,
                             (unsigned long)ap6256_cyw43_port_join_event_count(),
                             (unsigned long)ap6256_cyw43_port_last_async_event_type(),
                             (unsigned long)ap6256_cyw43_port_last_async_event_status(),
                             ap6256_wifi_runtime_rx_class_name(ap6256_cyw43_port_last_rx_class()),
                             ap6256_cyw43_port_last_rx_payload_len());
            ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_JOIN_TIMEOUT,
                                                (int32_t)(HAL_GetTick() - start_ms));
            return false;
        }

        if ((now_ms - last_diag_ms) >= 500U) {
            last_diag_ms = now_ms;
            test_uart_printf("[ INFO ] wifi.connect stage: join wait %lums status=%d join=0x%08lX assoc=%u/m%u %02X:%02X:%02X:%02X:%02X:%02X ev=%lu/%lu/%lu r=%lu f=0x%lX rx=%s/%u/%04X/%u/%u>%u/%08lX tx=%u/%u/%04X/%u/%u>%u/%ld sm=%08lX/%04X ch=%08lX/%04X fc=%u/%u/%u/%u/%ld\r\n",
                             (unsigned long)(now_ms - start_ms),
                             status,
                             (unsigned long)cyw43_state.wifi_join_state,
                             last_assoc_seen,
                             last_assoc_matches,
                             last_assoc_bssid[0],
                             last_assoc_bssid[1],
                             last_assoc_bssid[2],
                             last_assoc_bssid[3],
                             last_assoc_bssid[4],
                             last_assoc_bssid[5],
                             (unsigned long)ap6256_cyw43_port_join_event_count(),
                             (unsigned long)ap6256_cyw43_port_last_async_event_type(),
                             (unsigned long)ap6256_cyw43_port_last_async_event_status(),
                             (unsigned long)ap6256_cyw43_port_last_async_event_reason(),
                             (unsigned long)ap6256_cyw43_port_last_async_event_flags(),
                             ap6256_wifi_runtime_rx_class_name(ap6256_cyw43_port_last_rx_class()),
                             ap6256_cyw43_port_last_rx_payload_len(),
                             ap6256_cyw43_port_last_rx_ethertype(),
                             ap6256_cyw43_port_last_rx_ip_proto(),
                             ap6256_cyw43_port_last_rx_src_port(),
                             ap6256_cyw43_port_last_rx_dst_port(),
                             (unsigned long)ap6256_cyw43_port_last_rx_first_word(),
                             ap6256_cyw43_port_last_tx_itf(),
                             ap6256_cyw43_port_last_tx_payload_len(),
                             ap6256_cyw43_port_last_tx_ethertype(),
                             ap6256_cyw43_port_last_tx_ip_proto(),
                             ap6256_cyw43_port_last_tx_src_port(),
                             ap6256_cyw43_port_last_tx_dst_port(),
                             (long)ap6256_cyw43_port_last_tx_status(),
                             (unsigned long)ap6256_cyw43_port_last_tx_src_mac_hi(),
                             ap6256_cyw43_port_last_tx_src_mac_lo(),
                             (unsigned long)ap6256_cyw43_port_last_tx_dhcp_chaddr_hi(),
                             ap6256_cyw43_port_last_tx_dhcp_chaddr_lo(),
                             ap6256_cyw43_port_send_flow_control(),
                             ap6256_cyw43_port_send_tx_seq(),
                             ap6256_cyw43_port_send_credit(),
                             ap6256_cyw43_port_send_synthetic_credit(),
                             (long)ap6256_cyw43_port_send_credit_status());
        }

        osDelay(50U);
    }

    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_JOIN_TIMEOUT,
                                        (int32_t)(HAL_GetTick() - start_ms));
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
    s_wifi_runtime.cached_auth_mode = entry->auth_mode;
    s_wifi_runtime.cached_mfp = entry->mfp;
    s_wifi_runtime.cached_security_flags = entry->security_flags;
    memcpy(s_wifi_runtime.cached_bssid, entry->bssid, sizeof(s_wifi_runtime.cached_bssid));
    s_wifi_runtime.cached_channel = entry->channel;
    s_wifi_runtime.cached_chanspec = entry->chanspec;
    s_wifi_runtime.cached_akm_flags = entry->akm_flags;
    s_wifi_runtime.cached_pairwise_cipher_flags = entry->pairwise_cipher_flags;
    s_wifi_runtime.cached_group_cipher_flags = entry->group_cipher_flags;
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

static const char *ap6256_wifi_runtime_rx_class_name(uint32_t rx_class)
{
    switch (rx_class) {
    case AP6256_CYW43_RX_CLASS_NONE:
        return "none";
    case AP6256_CYW43_RX_CLASS_CONTROL:
        return "control";
    case AP6256_CYW43_RX_CLASS_ASYNC:
        return "async";
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
    case AP6256_CYW43_IOCTL_PHASE_TX_ACCEPTED:
        return "tx_accepted";
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
        ap6256_wifi_runtime_format_bus_detail(detail, detail_len);
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
            /*
             * brcmfmac disables/updates power-save policy as part of interface
             * bring-up. The known-good AP6256 2.4 GHz DHCP path also had STA PM
             * disabled before scan/join. Keep this best-effort so a missing
             * control completion is diagnostic noise, not a setup blocker.
             */
            test_uart_write_str("[ INFO ] wifi.connect stage: set STA PM none\r\n");
            pm_rc = cyw43_wifi_pm(&cyw43_state, CYW43_NONE_PM);
            if (pm_rc != 0) {
                test_uart_printf("[ INFO ] wifi.connect stage: STA PM none returned rc=%d; continuing\r\n",
                                 pm_rc);
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

static bool ap6256_wifi_runtime_recover_radio_before_join(const char *reason,
                                                          char *detail,
                                                          size_t detail_len)
{
    test_uart_printf("[ INFO ] wifi.connect stage: clear scan state %s before join\r\n",
                     (reason != NULL) ? reason : "after scan");

    if (s_wifi_runtime.initialized == 0U) {
        return ap6256_wifi_runtime_ensure_ready(detail, detail_len);
    }

    /*
     * Keep the firmware alive between scan and association. Runtime owns scan
     * cleanup here: detach host scan callbacks/state, drain pending async
     * frames, then require a quiet window before association. Sending a fresh
     * ESCAN abort from the low-level join path was observed as stale ABORT
     * events during 5 GHz join_wait and correlated with target resets.
     */
    cyw43_state.wifi_scan_state = 0;
    cyw43_state.wifi_scan_cb = NULL;
    cyw43_state.wifi_scan_env = NULL;
    ap6256_wifi_runtime_poll_burst(24U, 6U);
    ap6256_wifi_runtime_quiesce_scan_events(reason);
    return true;
}

static void ap6256_wifi_runtime_disconnect_current(void)
{
    int link_status;

    if (s_wifi_runtime.initialized == 0U) {
        return;
    }

    link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    if ((s_wifi_runtime.link_up != 0U) || (link_status >= CYW43_LINK_NOIP)) {
        (void)cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
        osDelay(100U);
        ap6256_wifi_runtime_poll_burst(16U, 4U);
    }
    cyw43_state.wifi_join_state = 0U;
    s_wifi_runtime.link_up = 0U;
    s_wifi_runtime.last_rssi = 0;
    memset(s_wifi_runtime.last_ip, 0, sizeof(s_wifi_runtime.last_ip));
    memset(s_wifi_runtime.last_mask, 0, sizeof(s_wifi_runtime.last_mask));
    memset(s_wifi_runtime.last_gateway, 0, sizeof(s_wifi_runtime.last_gateway));
    ap6256_connectivity_set_wifi_ip("", "", "", 0U);
}

static void ap6256_wifi_runtime_clear_failed_join_for_next_candidate(void)
{
    if (s_wifi_runtime.initialized == 0U) {
        return;
    }

    /*
     * Keep the scan evidence collected before the first candidate. A full
     * CYW43/firmware restart was wiping the firmware scan cache and causing
     * the following directed scan to return zero BSSIDs, so later 5 GHz
     * candidates were never actually attempted.
     */
    (void)cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
    osDelay(80U);
    ap6256_wifi_runtime_poll_burst(24U, 4U);
    cyw43_state.wifi_join_state = 0U;
    s_wifi_runtime.link_up = 0U;
}

static bool ap6256_wifi_runtime_restart_radio_for_next_candidate(const char *reason,
                                                                 char *detail,
                                                                 size_t detail_len)
{
    test_uart_printf("[ INFO ] wifi.connect stage: restart Wi-Fi runtime %s\r\n",
                     (reason != NULL) ? reason : "before next candidate");

    if (s_wifi_runtime.initialized != 0U) {
        (void)cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
        osDelay(40U);
        ap6256_wifi_runtime_poll_burst(24U, 6U);
        cyw43_deinit(&cyw43_state);
        ap6256_cyw43_port_deinit();
        s_wifi_runtime.initialized = 0U;
        s_wifi_runtime.link_up = 0U;
        ap6256_wifi_runtime_reset_driver_state();
        osDelay(150U);
    }

    if (!ap6256_wifi_runtime_ensure_ready(detail, detail_len)) {
        return false;
    }
    cyw43_state.wifi_join_state = 0U;
    s_wifi_runtime.link_up = 0U;
    return true;
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
    case CYW43_AUTH_WPA3_WPA2_AES_PSK:
        return "wpa3_wpa2";
    case CYW43_AUTH_WPA3_SAE_AES_PSK:
        return "wpa3_sae";
    default:
        return "unknown";
    }
}

static uint32_t ap6256_wifi_runtime_select_auth(uint8_t security_flags,
                                                uint16_t akm_flags,
                                                uint16_t pairwise_cipher_flags,
                                                uint16_t group_cipher_flags,
                                                uint8_t mfp,
                                                uint8_t selected_5g)
{
    const uint8_t has_rsn = ((security_flags & CYW43_SCAN_SEC_RSN) != 0U) ? 1U : 0U;
    const uint8_t has_wpa = ((security_flags & CYW43_SCAN_SEC_WPA) != 0U) ? 1U : 0U;
    const uint8_t has_tkip =
        (((pairwise_cipher_flags | group_cipher_flags) & CYW43_SCAN_CIPHER_TKIP) != 0U) ? 1U : 0U;
    const uint8_t has_ccmp =
        (((pairwise_cipher_flags | group_cipher_flags) & CYW43_SCAN_CIPHER_CCMP) != 0U) ? 1U : 0U;
    const uint8_t has_psk = ((akm_flags & CYW43_SCAN_AKM_PSK) != 0U) ? 1U : 0U;
    const uint8_t has_sae = ((akm_flags & CYW43_SCAN_AKM_SAE) != 0U) ? 1U : 0U;

    /*
     * Match brcmfmac's "configure from the selected BSS" behavior, but keep
     * the v1 scope to open + WPA/WPA2 PSK. Prefer WPA2/CCMP when advertised;
     * only use mixed mode when the BSS really advertises WPA1/TKIP/mixed facts.
     *
     * BCM43456/AP6256 transition-mode APs advertise both PSK and SAE with
     * optional MFP. The BCM43456 firmware handles SAE/PSK transition auth
     * internally when the passphrase and MFP-capable policy are programmed.
     * Use the transition auth value for PSK+SAE BSSs so the association RSN
     * policy matches the AP; the 5 GHz path is kept safe by the SSID-only join
     * shape rather than by stripping the security mode down to WPA2-only.
     */
    if ((has_sae != 0U) && (has_psk != 0U) && (mfp != CYW43_SCAN_MFP_REQUIRED)) {
        if ((has_ccmp != 0U) || (has_rsn != 0U)) {
            /*
             * 5 GHz transition-mode association is the unstable path on
             * BCM43456/AP6256. brcmfmac's cfg80211 path can select the WPA2
             * PSK leg for PSK+SAE/MFPC BSSs when PMF is not required; keep
             * that conservative leg for 5 GHz while preserving transition
             * auth on the already-proven 2.4 GHz regression path.
             */
            if (selected_5g != 0U) {
                return CYW43_AUTH_WPA2_AES_PSK;
            }
            return CYW43_AUTH_WPA3_WPA2_AES_PSK;
        }
    }

    if ((has_wpa != 0U) || (has_tkip != 0U)) {
        if (has_rsn != 0U) {
            return CYW43_AUTH_WPA2_MIXED_PSK;
        }
        return CYW43_AUTH_WPA_TKIP_PSK;
    }

    if ((has_rsn != 0U) || (has_ccmp != 0U) ||
        ((akm_flags & (CYW43_SCAN_AKM_PSK | CYW43_SCAN_AKM_PSK_SHA256)) != 0U)) {
        return CYW43_AUTH_WPA2_AES_PSK;
    }

    return CYW43_AUTH_WPA2_AES_PSK;
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

static uint16_t ap6256_wifi_runtime_validated_join_chanspec(uint16_t channel,
                                                            uint16_t preferred_chanspec)
{
    uint8_t buf[512];
    uint32_t count;
    uint16_t first_matching = 0U;
    uint16_t derived = ap6256_wifi_runtime_primary_chanspec(channel);

    if ((channel == 0U) || (channel == AP6256_WIFI_CHANNEL_5G_UNKNOWN)) {
        return preferred_chanspec;
    }
    if (preferred_chanspec == 0U) {
        preferred_chanspec = derived;
    }

    /*
     * BCM43456 is happiest when connect requests carry the primary channel
     * chanspec, not the raw 80 MHz scan chanspec. brcmfmac gets a full cfg80211
     * chandef and firmware feature negotiation around connect; this MCU path
     * only needs to steer the initial association to the right primary channel,
     * then verifies the final VHT/Wi-Fi-5 link after association. Keep the raw
     * scan chanspec in diagnostics, but prefer the legal 20 MHz primary for the
     * actual join hint/payload.
     */
    if ((ap6256_wifi_runtime_channel_is_5g(channel) != 0U) && (derived != 0U)) {
        return derived;
    }

    if (ap6256_wifi_runtime_get_iovar_raw("chanspecs", buf, sizeof(buf)) != 0) {
        return preferred_chanspec;
    }

    count = ap6256_wifi_runtime_get_le32(buf);
    if (count > ((sizeof(buf) - 4U) / 4U)) {
        count = (sizeof(buf) - 4U) / 4U;
    }

    for (uint32_t i = 0U; i < count; ++i) {
        uint16_t candidate = (uint16_t)ap6256_wifi_runtime_get_le32(&buf[4U + (i * 4U)]);
        uint8_t candidate_primary = ap6256_wifi_runtime_primary_channel_from_chanspec(candidate);

        if (candidate_primary != (uint8_t)(channel & 0x00FFU)) {
            continue;
        }
        if (first_matching == 0U) {
            first_matching = candidate;
        }
        if ((derived != 0U) && (candidate == derived)) {
            return derived;
        }
        if (candidate == preferred_chanspec) {
            first_matching = candidate;
        }
    }

    if ((ap6256_wifi_runtime_channel_is_5g(channel) != 0U) && (derived != 0U)) {
        return derived;
    }

    return (first_matching != 0U) ? first_matching : preferred_chanspec;
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
    char clm_version[96];
    char country_text[32];
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
    memset(clm_version, 0, sizeof(clm_version));
    memset(country_text, 0, sizeof(country_text));
    memset(caps, 0, sizeof(caps));

    if (ap6256_wifi_runtime_get_iovar_raw("ver", buf, sizeof(buf)) == 0) {
        ap6256_wifi_runtime_copy_printable(fw_version, sizeof(fw_version), buf, sizeof(buf));
        valid = 1U;
    }
    if (ap6256_wifi_runtime_get_iovar_raw("cap", buf, sizeof(buf)) == 0) {
        ap6256_wifi_runtime_copy_printable(caps, sizeof(caps), buf, sizeof(buf));
        valid = 1U;
    }
    if (ap6256_wifi_runtime_get_iovar_raw("clmver", buf, sizeof(buf)) == 0) {
        ap6256_wifi_runtime_copy_printable(clm_version, sizeof(clm_version), buf, sizeof(buf));
        valid = 1U;
    }
    if (ap6256_wifi_runtime_get_iovar_raw("country", buf, sizeof(buf)) == 0) {
        char c0 = ((buf[0] >= 0x20U) && (buf[0] <= 0x7EU)) ? (char)buf[0] : '?';
        char c1 = ((buf[1] >= 0x20U) && (buf[1] <= 0x7EU)) ? (char)buf[1] : '?';
        uint32_t rev = ap6256_wifi_runtime_get_le32(&buf[4]);

        (void)snprintf(country_text, sizeof(country_text), "%c%c/%lu", c0, c1, (unsigned long)rev);
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
                                          clm_version,
                                          country_text,
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
                                                      uint8_t auth_mode,
                                                      uint8_t security_flags,
                                                      uint16_t akm_flags,
                                                      uint16_t pairwise_cipher_flags,
                                                      uint16_t group_cipher_flags,
                                                      uint8_t mfp,
                                                      const uint8_t *bssid,
                                                      uint16_t channel,
                                                      uint16_t chanspec,
                                                      ap6256_wifi_runtime_summary_t *summary,
                                                      char *detail,
                                                      size_t detail_len)
{
    int rc;
    int final_status;
    int32_t rssi = 0;
    uint32_t selected_auth = CYW43_AUTH_OPEN;
    uint8_t actual_assoc_channel = 0U;
    uint8_t actual_assoc_wifi5 = 0U;
    uint8_t assoc_bssid[6];
    uint8_t assoc_bssid_valid = 0U;

    memset(assoc_bssid, 0, sizeof(assoc_bssid));

    if ((ssid == NULL) || (ssid[0] == '\0')) {
        (void)snprintf(detail, detail_len, "No Wi-Fi SSID is available for this run.");
        return AP6256_STATUS_BAD_PARAM;
    }

    if (!ap6256_wifi_runtime_ensure_ready(detail, detail_len)) {
        return AP6256_STATUS_IO_ERROR;
    }

    if (secure != 0U) {
        if (mfp == CYW43_SCAN_MFP_REQUIRED) {
            (void)snprintf(detail,
                           detail_len,
                           "Selected SSID '%s' requires PMF/MFP; unsupported_security.",
                           ssid);
            ap6256_connectivity_set_wifi_note(detail);
            return AP6256_STATUS_BAD_PARAM;
        }
        if ((akm_flags != 0U) &&
            ((akm_flags & (CYW43_SCAN_AKM_PSK | CYW43_SCAN_AKM_PSK_SHA256)) == 0U)) {
            (void)snprintf(detail,
                           detail_len,
                           "Selected SSID '%s' uses unsupported AKM '%s'.",
                           ssid,
                           ap6256_wifi_runtime_akm_name(akm_flags));
            ap6256_connectivity_set_wifi_note(detail);
            return AP6256_STATUS_BAD_PARAM;
        }

        (void)auth_mode;
        selected_auth = ap6256_wifi_runtime_select_auth(security_flags,
                                                        akm_flags,
                                                        pairwise_cipher_flags,
                                                        group_cipher_flags,
                                                        mfp,
                                                        ap6256_wifi_runtime_channel_is_5g(channel));
    } else {
        selected_auth = CYW43_AUTH_OPEN;
    }

    final_status = CYW43_LINK_DOWN;
    ap6256_wifi_runtime_disconnect_current();
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_JOIN_START,
                                        (int32_t)((secure != 0U) ? 1 : 0));
    test_uart_printf("[ INFO ] wifi.connect stage: join auth=%s\r\n",
                     ap6256_wifi_runtime_auth_name(selected_auth));

    const uint8_t *join_bssid = NULL;
    const uint8_t *validation_bssid = bssid;
    uint32_t join_channel = CYW43_CHANNEL_NONE;
    uint16_t assoc_target_chanspec = chanspec;
    if ((bssid != NULL) && (channel != 0U) && (channel != AP6256_WIFI_CHANNEL_5G_UNKNOWN)) {
        uint16_t join_chanspec = ((ap6256_wifi_runtime_channel_is_5g(channel) != 0U) &&
                                  (chanspec != 0U))
            ? chanspec
            : ap6256_wifi_runtime_primary_chanspec(channel);

        if (join_chanspec == 0U) {
            join_chanspec = chanspec;
        }
        join_chanspec = ap6256_wifi_runtime_validated_join_chanspec(channel, join_chanspec);
        assoc_target_chanspec = join_chanspec;

        test_uart_printf("[ INFO ] wifi.connect stage: selected BSS bssid=%02X:%02X:%02X:%02X:%02X:%02X ch=%u/%s scan_cs=0x%04X join_cs=0x%04X\r\n",
                         bssid[0], bssid[1], bssid[2],
                         bssid[3], bssid[4], bssid[5],
                         (unsigned)channel,
                         ap6256_wifi_runtime_channel_band_name(channel),
                         chanspec,
                         join_chanspec);
        if (ap6256_wifi_runtime_channel_is_5g(channel) != 0U) {
            /*
             * Use the directed scan result to prove that a 5 GHz BSS exists,
             * but keep the association command SSID-only. Directed
             * BSSID/chanspec WLC_SET_SSID is still reset-prone on this board;
             * the important fix is to preserve firmware scan cache by not
             * issuing a low-level ESCAN abort immediately before join.
             */
            test_uart_write_str("[ INFO ] wifi.connect stage: cache-backed ssid-only 5GHz join\r\n");
            join_bssid = NULL;
            validation_bssid = bssid;
            join_channel = channel;
        } else {
            /*
             * Keep 2.4 GHz on CYW43's original SSID-only association shape.
             * Directed BSSID/chanspec joins on this transition-mode AP
             * associate briefly, then drop before the WPA key exchange
             * completes. SSID-only reaches PSK_SUP/KEYED and exposes the
             * remaining DHCP/data-plane issue without regressing auth.
             */
            test_uart_write_str("[ INFO ] wifi.connect stage: ssid-only 2.4GHz join\r\n");
            join_bssid = NULL;
            validation_bssid = NULL;
            join_channel = CYW43_CHANNEL_NONE;
        }
    }
    ap6256_cyw43_port_record_assoc_target(bssid,
                                          channel,
                                          ap6256_wifi_runtime_channel_is_5g(channel),
                                          assoc_target_chanspec,
                                          selected_auth,
                                          s_wifi_runtime_join_candidate_index,
                                          s_wifi_runtime_join_candidate_count);
    if (ap6256_wifi_runtime_channel_is_5g(channel) != 0U) {
        uint8_t assoc_diag_channel = 0U;

        test_uart_printf("[ INFO ] wifi.connect stage: 5GHz fw profile=%s nvram_default=%s\r\n",
                         ap6256_assets_wifi_profile_name(),
                         (ap6256_assets_wifi_profile_default_generic_nvram() != 0U) ? "profile" : "ap6256");
        (void)ap6256_wifi_runtime_capture_phy_diag(channel, &assoc_diag_channel);
    }

    rc = cyw43_wifi_join(&cyw43_state,
                         strlen(ssid),
                         (const uint8_t *)ssid,
                         secure ? strlen(password) : 0U,
                         (const uint8_t *)(secure ? password : ""),
                         selected_auth,
                         join_bssid,
                         join_channel);
    if (rc != 0) {
        (void)snprintf(detail,
                       detail_len,
                       "Wi-Fi join request failed for '%s' auth=%s rc=%d io=%lu/%lu if=%lu len=%lu id=%lu st=%ld poll=%ld c52=%lu/%08lX c53=%c/f%u/b%u/bs%lu/l%lu/st%ld.",
                       ssid,
                       ap6256_wifi_runtime_auth_name(selected_auth),
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
                     ap6256_wifi_runtime_auth_name(selected_auth));
    if (!ap6256_wifi_runtime_wait_for_link(AP6256_WIFI_DHCP_TIMEOUT_MS,
                                           &final_status,
                                           validation_bssid,
                                           secure,
                                           ap6256_wifi_runtime_channel_is_5g(channel))) {
        const char *reason = "timed out waiting for association/DHCP";

        if (final_status == CYW43_LINK_BADAUTH) {
            reason = "authentication failed";
        } else if (final_status == CYW43_LINK_NONET) {
            reason = "SSID disappeared or was not found";
        } else if (final_status == CYW43_LINK_FAIL) {
            reason = "firmware reported join failure";
        } else if (final_status == CYW43_LINK_NOIP) {
            reason = "dhcp_timeout: Wi-Fi link is up but no DHCP lease was acquired";
        } else if ((secure != 0U) &&
                   ((cyw43_state.wifi_join_state & AP6256_CYW43_JOIN_STATE_KEYED) == 0U)) {
            reason = "auth_failure: WPA/WPA2 key exchange did not complete";
            final_status = CYW43_LINK_BADAUTH;
        }

        (void)snprintf(detail,
                       detail_len,
                       "Wi-Fi join for '%s' failed with auth=%s: %s.",
                       ssid,
                       ap6256_wifi_runtime_auth_name(selected_auth),
                       reason);
        ap6256_connectivity_set_wifi_note(detail);
        ap6256_connectivity_set_wifi_ip("", "", "", 0U);
        return AP6256_STATUS_TIMEOUT;
    }

    s_wifi_runtime.link_up = 1U;
    if (validation_bssid != NULL) {
        assoc_bssid_valid =
            (ap6256_wifi_runtime_probe_associated_bssid(assoc_bssid) != 0) ? 1U : 0U;
        test_uart_printf("[ INFO ] wifi.connect stage: associated bssid valid=%u %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                         assoc_bssid_valid,
                         assoc_bssid[0],
                         assoc_bssid[1],
                         assoc_bssid[2],
                         assoc_bssid[3],
                         assoc_bssid[4],
                         assoc_bssid[5]);
        if ((assoc_bssid_valid != 0U) && (memcmp(assoc_bssid, validation_bssid, sizeof(assoc_bssid)) != 0)) {
            (void)snprintf(detail,
                           detail_len,
                           "Selected BSSID for '%s' did not match associated BSSID.",
                           ssid);
            ap6256_connectivity_set_wifi_note(detail);
            return AP6256_STATUS_TIMEOUT;
        }
    }
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

static ap6256_status_t ap6256_wifi_runtime_attempt_join_entry(const ap6256_wifi_scan_entry_t *entry,
                                                              const char *password,
                                                              const char *fixture_classification,
                                                              uint8_t candidate_index,
                                                              uint8_t candidate_count,
                                                              ap6256_wifi_runtime_summary_t *summary,
                                                              char *detail,
                                                              size_t detail_len)
{
    ap6256_wifi_scan_entry_t selected_copy;
    ap6256_status_t st;
    char unsupported_reason[64];

    if ((entry == NULL) || (detail == NULL) || (detail_len == 0U)) {
        return AP6256_STATUS_BAD_PARAM;
    }

    ap6256_connectivity_set_wifi_candidate_diag(candidate_index, candidate_count);
    ap6256_connectivity_set_wifi_selection_diag(entry->bssid,
                                                entry->channel,
                                                ap6256_wifi_runtime_channel_is_5g(entry->channel),
                                                fixture_classification);
    ap6256_connectivity_set_wifi_selection_security_diag(entry->auth_mode,
                                                         entry->security_flags,
                                                         entry->akm_flags,
                                                         entry->pairwise_cipher_flags,
                                                         entry->group_cipher_flags,
                                                         entry->mfp,
                                                         entry->rsn_cap,
                                                         entry->chanspec);

    unsupported_reason[0] = '\0';
    if (ap6256_wifi_runtime_scan_security_supported(entry,
                                                    unsupported_reason,
                                                    sizeof(unsupported_reason)) == 0U) {
        (void)snprintf(detail,
                       detail_len,
                       "Selected SSID '%s' uses %s; sec=%s akm=%s pair=%s mfp=%s.",
                       entry->ssid,
                       unsupported_reason,
                       ap6256_wifi_runtime_security_name(entry->auth_mode),
                       ap6256_wifi_runtime_akm_name(entry->akm_flags),
                       ap6256_wifi_runtime_cipher_name(entry->pairwise_cipher_flags),
                       ap6256_wifi_runtime_mfp_name(entry->mfp));
        ap6256_connectivity_set_wifi_note(detail);
        return AP6256_STATUS_BAD_PARAM;
    }

    if ((entry->secure != 0U) && ((password == NULL) || (password[0] == '\0'))) {
        (void)snprintf(detail, detail_len, "SSID '%s' requires a password.", entry->ssid);
        ap6256_connectivity_set_wifi_note(detail);
        return AP6256_STATUS_BAD_PARAM;
    }

    selected_copy = *entry;
    if (!ap6256_wifi_runtime_recover_radio_before_join("after scan", detail, detail_len)) {
        ap6256_connectivity_set_wifi_note(detail);
        return AP6256_STATUS_IO_ERROR;
    }
    if (ap6256_wifi_runtime_channel_is_5g(selected_copy.channel) != 0U) {
        int32_t block_rc;

        ap6256_cyw43_port_set_runtime_f2_block_size(AP6256_WIFI_JOIN_F2_BLOCK_SIZE);
        block_rc = ap6256_cyw43_port_apply_runtime_f2_block_size();
        if (block_rc != 0) {
            (void)snprintf(detail,
                           detail_len,
                           "Failed to apply 5GHz join F2 block size %u rc=%ld.",
                           (unsigned)AP6256_WIFI_JOIN_F2_BLOCK_SIZE,
                           (long)block_rc);
            ap6256_connectivity_set_wifi_note(detail);
            return AP6256_STATUS_IO_ERROR;
        }
    }

    test_uart_printf("[ INFO ] wifi.connect stage: profile selected candidate=%u/%u bssid=%02X:%02X:%02X:%02X:%02X:%02X ch=%u/%s cs=0x%04X rssi=%d\r\n",
                     candidate_index,
                     candidate_count,
                     selected_copy.bssid[0],
                     selected_copy.bssid[1],
                     selected_copy.bssid[2],
                     selected_copy.bssid[3],
                     selected_copy.bssid[4],
                     selected_copy.bssid[5],
                     selected_copy.channel,
                     ap6256_wifi_runtime_channel_band_name(selected_copy.channel),
                     selected_copy.chanspec,
                     (int)selected_copy.rssi);
    test_uart_printf("[ INFO ] wifi.connect stage: join start f2bs=%lu\r\n",
                     (unsigned long)ap6256_cyw43_port_runtime_f2_block_size());

    s_wifi_runtime_join_candidate_index = candidate_index;
    s_wifi_runtime_join_candidate_count = candidate_count;
    st = ap6256_wifi_runtime_run_common(selected_copy.ssid,
                                        password,
                                        selected_copy.secure,
                                        selected_copy.auth_mode,
                                        selected_copy.security_flags,
                                        selected_copy.akm_flags,
                                        selected_copy.pairwise_cipher_flags,
                                        selected_copy.group_cipher_flags,
                                        selected_copy.mfp,
                                        selected_copy.bssid,
                                        selected_copy.channel,
                                        selected_copy.chanspec,
                                        summary,
                                        detail,
                                        detail_len);
    if (st == AP6256_STATUS_OK) {
        ap6256_wifi_runtime_capture_profile(&selected_copy, password);
    }
    return st;
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
    if (!ap6256_wifi_runtime_prepare_scan_sdio_policy(detail, detail_len)) {
        ap6256_connectivity_set_wifi_note(detail);
        ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE,
                                                          AP6256_STATUS_IO_ERROR);
        return AP6256_STATUS_IO_ERROR;
    }
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
            int32_t block_rc = 0;

            scan_start_recovery_attempted = 1U;
            test_uart_write_str("[ INFO ] wifi.connect stage: scan start recovery\r\n");
            if (((ap6256_cyw43_port_last_ioctl_phase() == AP6256_CYW43_IOCTL_PHASE_WAIT_NO_PACKET) ||
                 (ap6256_cyw43_port_last_ioctl_phase() == AP6256_CYW43_IOCTL_PHASE_SEND_FAIL) ||
                 (ap6256_cyw43_port_last_ioctl_phase() == AP6256_CYW43_IOCTL_PHASE_WAIT_CMD53)) &&
                (ap6256_cyw43_port_runtime_f2_block_size() != AP6256_WIFI_SCAN_RECOVERY_F2_BLOCK_SIZE)) {
                test_uart_printf("[ INFO ] wifi.connect stage: retry with F2 block size 64 after %s at bs=%lu\r\n",
                                 ap6256_wifi_runtime_ioctl_phase_name(ap6256_cyw43_port_last_ioctl_phase()),
                                 (unsigned long)ap6256_cyw43_port_runtime_f2_block_size());
                ap6256_cyw43_port_set_runtime_f2_block_size(AP6256_WIFI_SCAN_RECOVERY_F2_BLOCK_SIZE);
            }

            block_rc = ap6256_cyw43_port_apply_runtime_f2_block_size();
            if (block_rc != 0) {
                (void)snprintf(detail,
                               detail_len,
                               "Failed to apply scan-recovery F2 block size %lu rc=%ld.",
                               (unsigned long)ap6256_cyw43_port_runtime_f2_block_size(),
                               (long)block_rc);
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

    if (ap6256_wifi_runtime_has_5g_scan_result() == 0U) {
        test_uart_write_str("[ INFO ] wifi.connect stage: start visible 5GHz scan\r\n");
        memset(&opts, 0, sizeof(opts));
        opts.scan_type = 0;
        opts.channel_num = AP6256_WIFI_SCAN_FORCE_5G;
        rc = cyw43_wifi_scan(&cyw43_state, &opts, NULL, ap6256_wifi_runtime_scan_cb);
        if (rc == 0) {
            ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SCAN_ACCEPTED, 5);
            if (!ap6256_wifi_runtime_wait_for_scan_complete(AP6256_WIFI_SCAN_TIMEOUT_MS)) {
                test_uart_write_str("[ INFO ] wifi.connect stage: visible 5GHz scan timeout\r\n");
            }
            cyw43_state.wifi_scan_state = 0;
            cyw43_state.wifi_scan_cb = NULL;
            cyw43_state.wifi_scan_env = NULL;
            for (uint32_t drain_ms = 0U; drain_ms < 500U; drain_ms += 20U) {
                ap6256_cyw43_port_poll();
                osDelay(20U);
            }
            ap6256_wifi_runtime_sort_scan_results();
        } else {
            test_uart_printf("[ INFO ] wifi.connect stage: visible 5GHz scan start failed rc=%d\r\n", rc);
        }
        if (ap6256_wifi_runtime_has_5g_scan_result() == 0U) {
            test_uart_write_str("[ INFO ] wifi.connect stage: no visible 5GHz BSSID in current list; directed 5GHz SSID may be entered manually\r\n");
        }
    }

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
        char directed_ssid[33];

        (void)snprintf(directed_ssid, sizeof(directed_ssid), "%s", selected->ssid);
        selected = ap6256_wifi_runtime_resolve_directed_5g_selection(directed_ssid, detail, detail_len);
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
    if (!ap6256_wifi_runtime_recover_radio_before_join("after visible scan", detail, detail_len)) {
        ap6256_connectivity_set_wifi_note(detail);
        ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE,
                                                          AP6256_STATUS_IO_ERROR);
        return AP6256_STATUS_IO_ERROR;
    }

    test_uart_printf("[ INFO ] wifi.connect stage: join start f2bs=%lu\r\n",
                     (unsigned long)ap6256_cyw43_port_runtime_f2_block_size());
    ap6256_connectivity_set_wifi_candidate_diag(1U, 1U);
    s_wifi_runtime_join_candidate_index = 1U;
    s_wifi_runtime_join_candidate_count = 1U;
    ap6256_wifi_runtime_capture_profile(selected, password);
    {
        ap6256_status_t st = ap6256_wifi_runtime_run_common(selected->ssid,
                                                            password,
                                                            selected->secure,
                                                            selected->auth_mode,
                                                            selected->security_flags,
                                                            selected->akm_flags,
                                                            selected->pairwise_cipher_flags,
                                                            selected->group_cipher_flags,
                                                            selected->mfp,
                                                            selected->bssid,
                                                            selected->channel,
                                                            selected->chanspec,
                                                            summary,
                                                            detail,
                                                            detail_len);
        ap6256_wifi_runtime_set_poll_paused(0U);
        ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE, st);
        return st;
    }
}

ap6256_status_t ap6256_wifi_runtime_run_profile(const char *ssid,
                                                const char *password,
                                                uint16_t preferred_channel,
                                                ap6256_wifi_runtime_summary_t *summary,
                                                char *detail,
                                                size_t detail_len)
{
    cyw43_wifi_scan_options_t opts;
    ap6256_wifi_scan_entry_t *selected = NULL;
    ap6256_wifi_scan_entry_t selected_copy;
    const char *fixture_classification = "not_checked";
    int rc;
    uint8_t scan_start_recovery_attempted = 0U;
    uint32_t join_candidate_count = 0U;
    uint32_t unsupported_candidate_count = 0U;
    ap6256_wifi_scan_entry_t fallback_candidate;
    uint8_t fallback_candidate_valid = 0U;

    if ((detail == NULL) || (detail_len == 0U)) {
        return AP6256_STATUS_BAD_PARAM;
    }
    if ((ssid == NULL) || (ssid[0] == '\0')) {
        (void)snprintf(detail, detail_len, "No Wi-Fi SSID was provided.");
        return AP6256_STATUS_BAD_PARAM;
    }

    if (!network_manager_acquire(NETWORK_OWNER_WIFI, 30000U)) {
        (void)snprintf(detail, detail_len, "Timed out waiting for Wi-Fi radio ownership.");
        return AP6256_STATUS_TIMEOUT;
    }

    test_uart_write_str("[ INFO ] wifi.connect stage: acquired wifi owner\r\n");
    ap6256_wifi_runtime_set_poll_paused(1U);
    if (!ap6256_wifi_runtime_prepare_scan_sdio_policy(detail, detail_len)) {
        ap6256_connectivity_set_wifi_note(detail);
        ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE,
                                                          AP6256_STATUS_IO_ERROR);
        return AP6256_STATUS_IO_ERROR;
    }
    if (!ap6256_wifi_runtime_ensure_ready(detail, detail_len)) {
        ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE,
                                                          AP6256_STATUS_IO_ERROR);
        return AP6256_STATUS_IO_ERROR;
    }

    if (preferred_channel > 14U) {
        test_uart_printf("[ INFO ] wifi.connect stage: profile directed 5GHz scan ssid=%s requested_ch=%u\r\n",
                         ssid,
                         (unsigned)preferred_channel);
        selected = ap6256_wifi_runtime_resolve_directed_5g_selection(ssid, detail, detail_len);
        if (selected != NULL) {
            join_candidate_count =
                ap6256_wifi_runtime_collect_ssid_5g_candidates(ssid,
                                                               preferred_channel,
                                                               s_wifi_runtime_join_candidates,
                                                               AP6256_WIFI_MAX_JOIN_CANDIDATES,
                                                               &unsupported_candidate_count);
        }
        fixture_classification = (join_candidate_count != 0U) ? "selected_channel_5g_bss" : "fixture_no_channel_bss";
        if ((selected != NULL) && (join_candidate_count == 0U)) {
            (void)snprintf(detail,
                           detail_len,
                           "SSID '%s' was not found on requested 5GHz channel %u; found best channel %u.",
                           ssid,
                           (unsigned)preferred_channel,
                           (unsigned)selected->channel);
        }
        if (join_candidate_count == 0U) {
            if (detail[0] == '\0') {
                (void)snprintf(detail,
                               detail_len,
                               "SSID '%s' was not found on requested 5GHz channel %u.",
                               ssid,
                               (unsigned)preferred_channel);
            }
            ap6256_connectivity_set_wifi_selection_diag(NULL,
                                                        preferred_channel,
                                                        1U,
                                                        fixture_classification);
            ap6256_connectivity_set_wifi_candidate_diag(0U, 0U);
            ap6256_connectivity_set_wifi_note(detail);
            ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE,
                                                              AP6256_STATUS_TIMEOUT);
            return AP6256_STATUS_TIMEOUT;
        }
        ap6256_wifi_runtime_print_join_candidates(ssid,
                                                  s_wifi_runtime_join_candidates,
                                                  join_candidate_count);
        goto profile_try_candidates;
    }

profile_scan_retry:
    test_uart_printf("[ INFO ] wifi.connect stage: profile scan ssid=%s\r\n", ssid);
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
            int32_t block_rc = 0;

            scan_start_recovery_attempted = 1U;
            test_uart_write_str("[ INFO ] wifi.connect stage: scan start recovery\r\n");
            if (((ap6256_cyw43_port_last_ioctl_phase() == AP6256_CYW43_IOCTL_PHASE_WAIT_NO_PACKET) ||
                 (ap6256_cyw43_port_last_ioctl_phase() == AP6256_CYW43_IOCTL_PHASE_SEND_FAIL) ||
                 (ap6256_cyw43_port_last_ioctl_phase() == AP6256_CYW43_IOCTL_PHASE_WAIT_CMD53)) &&
                (ap6256_cyw43_port_runtime_f2_block_size() != AP6256_WIFI_SCAN_RECOVERY_F2_BLOCK_SIZE)) {
                test_uart_printf("[ INFO ] wifi.connect stage: retry with F2 block size 64 after %s at bs=%lu\r\n",
                                 ap6256_wifi_runtime_ioctl_phase_name(ap6256_cyw43_port_last_ioctl_phase()),
                                 (unsigned long)ap6256_cyw43_port_runtime_f2_block_size());
                ap6256_cyw43_port_set_runtime_f2_block_size(AP6256_WIFI_SCAN_RECOVERY_F2_BLOCK_SIZE);
            }

            block_rc = ap6256_cyw43_port_apply_runtime_f2_block_size();
            if (block_rc != 0) {
                (void)snprintf(detail,
                               detail_len,
                               "Failed to apply scan-recovery F2 block size %lu rc=%ld.",
                               (unsigned long)ap6256_cyw43_port_runtime_f2_block_size(),
                               (long)block_rc);
                ap6256_connectivity_set_wifi_note(detail);
                ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE,
                                                                  AP6256_STATUS_IO_ERROR);
                return AP6256_STATUS_IO_ERROR;
            }
            goto profile_scan_retry;
        }

        ap6256_wifi_runtime_format_scan_start_detail(detail, detail_len, rc);
        ap6256_connectivity_set_wifi_note(detail);
        ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE, rc);
        return AP6256_STATUS_IO_ERROR;
    }

    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SCAN_ACCEPTED, 0);
    if (!ap6256_wifi_runtime_wait_for_scan_complete(AP6256_WIFI_PROFILE_BROAD_SCAN_TIMEOUT_MS)) {
        uint8_t transport_timeout =
            ((ap6256_cyw43_port_poll_header_read_status() == -CYW43_ETIMEDOUT) ||
             (ap6256_cyw43_port_poll_payload_read_status() == -CYW43_ETIMEDOUT)) ? 1U : 0U;

        cyw43_state.wifi_scan_state = 0;
        cyw43_state.wifi_scan_cb = NULL;
        cyw43_state.wifi_scan_env = NULL;
        if (transport_timeout != 0U) {
            (void)snprintf(detail, detail_len, "Wi-Fi scan transport timeout while searching for '%s'.", ssid);
            ap6256_connectivity_set_wifi_note(detail);
            ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE,
                                                              AP6256_STATUS_TIMEOUT);
            return AP6256_STATUS_TIMEOUT;
        }
        test_uart_printf("[ INFO ] wifi.connect stage: profile broad scan produced no result for '%s'; trying directed 5GHz scan\r\n",
                         ssid);
    } else {
        cyw43_state.wifi_scan_state = 0;
        cyw43_state.wifi_scan_cb = NULL;
        cyw43_state.wifi_scan_env = NULL;
    }

    for (uint32_t drain_ms = 0U; drain_ms < 500U; drain_ms += 20U) {
        ap6256_cyw43_port_poll();
        osDelay(20U);
    }
    ap6256_wifi_runtime_sort_scan_results();

    if ((preferred_channel == 0U) || (preferred_channel > 14U)) {
        join_candidate_count =
            ap6256_wifi_runtime_collect_ssid_5g_candidates(ssid,
                                                           0U,
                                                           s_wifi_runtime_join_candidates,
                                                           AP6256_WIFI_MAX_JOIN_CANDIDATES,
                                                           &unsupported_candidate_count);
    }

    if ((preferred_channel != 0U) && (preferred_channel <= 14U)) {
        join_candidate_count =
            ap6256_wifi_runtime_collect_ssid_candidates(ssid,
                                                        preferred_channel,
                                                        0U,
                                                        s_wifi_runtime_join_candidates,
                                                        AP6256_WIFI_MAX_JOIN_CANDIDATES,
                                                        &unsupported_candidate_count);
        selected = ap6256_wifi_runtime_find_best_ssid_channel(ssid, preferred_channel);
        fixture_classification = (join_candidate_count != 0U) ? "selected_channel_bss" : "fixture_no_channel_bss";
        if (selected == NULL) {
            (void)snprintf(detail,
                           detail_len,
                           "SSID '%s' was not found on requested channel %u.",
                           ssid,
                           (unsigned)preferred_channel);
        }
        if (join_candidate_count != 0U) {
            ap6256_wifi_runtime_print_join_candidates(ssid,
                                                      s_wifi_runtime_join_candidates,
                                                      join_candidate_count);
            goto profile_try_candidates;
        }
    } else {
        selected = ap6256_wifi_runtime_find_best_ssid_any(ssid);
        fixture_classification = (selected != NULL) ? "selected_exact_bss" : "fixture_no_bss";
        if ((selected != NULL) &&
            (ap6256_wifi_runtime_channel_is_5g(selected->channel) == 0U)) {
            fallback_candidate = *selected;
            fallback_candidate_valid = 1U;
        }
    }
    if (((preferred_channel == 0U) || (preferred_channel > 14U)) &&
        (join_candidate_count == 0U)) {
        if ((selected == NULL) ||
            (ap6256_wifi_runtime_channel_is_5g(selected->channel) == 0U)) {
            test_uart_printf("[ INFO ] wifi.connect stage: profile broad scan found %lu results but not '%s'; trying directed 5GHz scan\r\n",
                             (unsigned long)s_wifi_runtime.last_scan_count,
                             ssid);
            selected = ap6256_wifi_runtime_resolve_directed_5g_selection(ssid, detail, detail_len);
            if (selected != NULL) {
                join_candidate_count =
                    ap6256_wifi_runtime_collect_ssid_5g_candidates(ssid,
                                                                   0U,
                                                                   s_wifi_runtime_join_candidates,
                                                                   AP6256_WIFI_MAX_JOIN_CANDIDATES,
                                                                   &unsupported_candidate_count);
            }
            fixture_classification = (join_candidate_count != 0U) ? "directed_5g_bss" : "fixture_no_5g_bss";
        }
    }
    if (((preferred_channel == 0U) || (preferred_channel > 14U)) &&
        (join_candidate_count != 0U)) {
        ap6256_wifi_runtime_print_join_candidates(ssid,
                                                  s_wifi_runtime_join_candidates,
                                                  join_candidate_count);
        goto profile_try_candidates;
    }
    if ((selected == NULL) && (fallback_candidate_valid != 0U)) {
        selected = &fallback_candidate;
        fixture_classification = "fallback_2g_bss";
    }
    if (selected == NULL) {
        if (detail[0] == '\0') {
            (void)snprintf(detail,
                           detail_len,
                           "SSID '%s' was not found during visible or directed 5GHz scan.",
                           ssid);
        }
    }
    if (selected == NULL) {
        if (strcmp(fixture_classification, "fixture_no_5g_bss") == 0) {
            ap6256_connectivity_set_wifi_selection_diag(NULL, 0U, 1U, fixture_classification);
            ap6256_connectivity_set_wifi_candidate_diag(0U, 0U);
        }
        ap6256_connectivity_set_wifi_note(detail);
        ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE,
                                                          AP6256_STATUS_TIMEOUT);
        return AP6256_STATUS_TIMEOUT;
    }

profile_try_candidates:
    if (join_candidate_count != 0U) {
        ap6256_status_t final_candidate_status = AP6256_STATUS_TIMEOUT;

        for (uint32_t i = 0U; i < join_candidate_count; ++i) {
            ap6256_wifi_scan_entry_t attempt = s_wifi_runtime_join_candidates[i];
            uint16_t derived_chanspec = ap6256_wifi_runtime_primary_chanspec(attempt.channel);
            uint8_t candidate_index = (uint8_t)((i + 1U) & 0x7FU);
            uint8_t candidate_count = (uint8_t)(join_candidate_count & 0x7FU);
            ap6256_status_t st;

            detail[0] = '\0';
            if ((i != 0U) && (ap6256_wifi_runtime_channel_is_5g(attempt.channel) != 0U)) {
                if (!ap6256_wifi_runtime_restart_radio_for_next_candidate("before next 5GHz candidate",
                                                                          detail,
                                                                          detail_len)) {
                    final_candidate_status = AP6256_STATUS_IO_ERROR;
                    if (detail[0] == '\0') {
                        (void)snprintf(detail,
                                       detail_len,
                                       "Failed to restart Wi-Fi runtime before candidate %lu/%lu.",
                                       (unsigned long)(i + 1U),
                                       (unsigned long)join_candidate_count);
                    }
                    break;
                }

                test_uart_printf("[ INFO ] wifi.connect stage: prime scan cache for candidate %lu/%lu ch=%u\r\n",
                                 (unsigned long)(i + 1U),
                                 (unsigned long)join_candidate_count,
                                 (unsigned)attempt.channel);
                if (ap6256_wifi_runtime_resolve_directed_5g_selection_channel(ssid,
                                                                               attempt.channel,
                                                                               detail,
                                                                               detail_len) == NULL) {
                    final_candidate_status = AP6256_STATUS_TIMEOUT;
                    test_uart_printf("[ INFO ] wifi.connect stage: candidate %lu/%lu scan-cache prime failed detail=%s\r\n",
                                     (unsigned long)(i + 1U),
                                     (unsigned long)join_candidate_count,
                                     (detail[0] != '\0') ? detail : "n/a");
                    continue;
                }
            }

            st = ap6256_wifi_runtime_attempt_join_entry(&attempt,
                                                        password,
                                                        fixture_classification,
                                                        candidate_index,
                                                        candidate_count,
                                                        summary,
                                                        detail,
                                                        detail_len);
            final_candidate_status = st;
            if (st == AP6256_STATUS_OK) {
                ap6256_wifi_runtime_set_poll_paused(0U);
                ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE, st);
                return st;
            }

            test_uart_printf("[ INFO ] wifi.connect stage: candidate %lu/%lu failed st=%d detail=%s\r\n",
                             (unsigned long)(i + 1U),
                             (unsigned long)join_candidate_count,
                             (int)st,
                             (detail[0] != '\0') ? detail : "n/a");

            if ((st == AP6256_STATUS_BAD_PARAM) ||
                (strstr(detail, "unsupported") != NULL)) {
                break;
            }

            /*
             * No-progress on one BSSID does not prove the raw VHT chanspec was
             * rejected, and retrying the same BSSID has repeatedly left stale
             * firmware state for the next candidate. Move straight to the next
             * scanned BSSID; a derived-chanspec retry should only come back if
             * firmware explicitly rejects the raw association command.
             */
            if ((attempt.chanspec != 0U) &&
                (derived_chanspec != 0U) &&
                (attempt.chanspec != derived_chanspec)) {
                test_uart_printf("[ INFO ] wifi.connect stage: skip chanspec fallback for candidate %lu/%lu raw=0x%04X derived=0x%04X\r\n",
                                 (unsigned long)(i + 1U),
                                 (unsigned long)join_candidate_count,
                                 attempt.chanspec,
                                 derived_chanspec);
            }

            if ((i + 1U) < join_candidate_count) {
                if (ap6256_wifi_runtime_channel_is_5g(attempt.channel) != 0U) {
                    test_uart_printf("[ INFO ] wifi.connect stage: stop failed 5GHz join before candidate %lu/%lu\r\n",
                                     (unsigned long)(i + 2U),
                                     (unsigned long)join_candidate_count);
                    ap6256_wifi_runtime_clear_failed_join_for_next_candidate();
                }
            }
        }

        if (detail[0] == '\0') {
            (void)snprintf(detail,
                           detail_len,
                           "All %lu candidates for SSID '%s' failed; unsupported=%lu.",
                           (unsigned long)join_candidate_count,
                           ssid,
                           (unsigned long)unsupported_candidate_count);
        }
        ap6256_connectivity_set_wifi_note(detail);
        ap6256_wifi_runtime_set_poll_paused(0U);
        ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE,
                                                          final_candidate_status);
        return final_candidate_status;
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

    if ((selected->secure != 0U) && ((password == NULL) || (password[0] == '\0'))) {
        (void)snprintf(detail, detail_len, "SSID '%s' requires a password.", selected->ssid);
        ap6256_connectivity_set_wifi_note(detail);
        ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE,
                                                          AP6256_STATUS_BAD_PARAM);
        return AP6256_STATUS_BAD_PARAM;
    }

    selected_copy = *selected;
    selected = &selected_copy;
    if (!ap6256_wifi_runtime_recover_radio_before_join("after profile scan", detail, detail_len)) {
        ap6256_connectivity_set_wifi_note(detail);
        ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE,
                                                          AP6256_STATUS_IO_ERROR);
        return AP6256_STATUS_IO_ERROR;
    }

    test_uart_printf("[ INFO ] wifi.connect stage: profile selected bssid=%02X:%02X:%02X:%02X:%02X:%02X ch=%u/%s cs=0x%04X rssi=%d\r\n",
                     selected->bssid[0],
                     selected->bssid[1],
                     selected->bssid[2],
                     selected->bssid[3],
                     selected->bssid[4],
                     selected->bssid[5],
                     selected->channel,
                     ap6256_wifi_runtime_channel_band_name(selected->channel),
                     selected->chanspec,
                     (int)selected->rssi);
    test_uart_printf("[ INFO ] wifi.connect stage: join start f2bs=%lu\r\n",
                     (unsigned long)ap6256_cyw43_port_runtime_f2_block_size());
    ap6256_connectivity_set_wifi_candidate_diag(1U, 1U);
    s_wifi_runtime_join_candidate_index = 1U;
    s_wifi_runtime_join_candidate_count = 1U;
    ap6256_wifi_runtime_capture_profile(selected, password);
    {
        ap6256_status_t st = ap6256_wifi_runtime_run_common(selected->ssid,
                                                            password,
                                                            selected->secure,
                                                            selected->auth_mode,
                                                            selected->security_flags,
                                                            selected->akm_flags,
                                                            selected->pairwise_cipher_flags,
                                                            selected->group_cipher_flags,
                                                            selected->mfp,
                                                            selected->bssid,
                                                            selected->channel,
                                                            selected->chanspec,
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
    if (!ap6256_wifi_runtime_prepare_join_sdio_policy(detail, detail_len)) {
        ap6256_connectivity_set_wifi_note(detail);
        ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE,
                                                          AP6256_STATUS_IO_ERROR);
        return AP6256_STATUS_IO_ERROR;
    }
    ap6256_connectivity_set_wifi_selection_diag(s_wifi_runtime.cached_bssid,
                                                s_wifi_runtime.cached_channel,
                                                ap6256_wifi_runtime_channel_is_5g(s_wifi_runtime.cached_channel),
                                                "cached_profile");
    ap6256_connectivity_set_wifi_selection_security_diag(s_wifi_runtime.cached_auth_mode,
                                                         s_wifi_runtime.cached_security_flags,
                                                         s_wifi_runtime.cached_akm_flags,
                                                         s_wifi_runtime.cached_pairwise_cipher_flags,
                                                         s_wifi_runtime.cached_group_cipher_flags,
                                                         s_wifi_runtime.cached_mfp,
                                                         0U,
                                                         0U);
    {
        ap6256_status_t st = ap6256_wifi_runtime_run_common(s_wifi_runtime.cached_ssid,
                                                            s_wifi_runtime.cached_password,
                                                            s_wifi_runtime.cached_secure,
                                                            s_wifi_runtime.cached_auth_mode,
                                                            s_wifi_runtime.cached_security_flags,
                                                            s_wifi_runtime.cached_akm_flags,
                                                            s_wifi_runtime.cached_pairwise_cipher_flags,
                                                            s_wifi_runtime.cached_group_cipher_flags,
                                                            s_wifi_runtime.cached_mfp,
                                                            s_wifi_runtime.cached_bssid,
                                                            s_wifi_runtime.cached_channel,
                                                            s_wifi_runtime.cached_chanspec,
                                                            summary,
                                                            detail,
                                                            detail_len);
        ap6256_wifi_runtime_set_poll_paused(0U);
        ap6256_wifi_runtime_release_owner_with_breadcrumb(AP6256_CYW43_BREADCRUMB_RELEASE, st);
        return st;
    }
}
