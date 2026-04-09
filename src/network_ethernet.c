#include "network_ethernet.h"

#include "board_test.h"
#include "main.h"
#include "network_stack.h"
#include "test_uart.h"

#include "lwip/dhcp.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/netifapi.h"
#include "lwip/pbuf.h"
#include "lwip/prot/ethernet.h"
#include "lwip/prot/icmp.h"
#include "lwip/prot/ip4.h"
#include "lwip/tcpip.h"
#include "netif/etharp.h"
#include "netif/ethernet.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define NETWORK_ETH_PHY_ADDR               1U
#define NETWORK_ETH_DHCP_TIMEOUT_MS        30000U
#define NETWORK_ETH_PING_TIMEOUT_MS        30000U
#define NETWORK_ETH_LINK_POLL_MS           250U
#define NETWORK_ETH_LINK_TEST_TIMEOUT_MS   65000U
#define NETWORK_ETH_RX_BUFFER_SIZE         1536U
#define NETWORK_ETH_RX_BUFFER_COUNT        (ETH_RX_DESC_CNT * 2U)
#define NETWORK_ETH_TX_BUFFER_SIZE         1536U
#define NETWORK_ETH_TX_TIMEOUT_MS          100U
#define NETWORK_ETH_HOSTNAME               "karios48-test"
#define NETWORK_ETH_IP_PROTO_ICMP          1U

#define NETWORK_ETH_PHY_REG_BMCR           0x00U
#define NETWORK_ETH_PHY_REG_BMSR           0x01U
#define NETWORK_ETH_PHY_REG_PHYID1         0x02U
#define NETWORK_ETH_PHY_REG_PHYID2         0x03U
#define NETWORK_ETH_PHY_REG_PHYSTS         0x10U
#define NETWORK_ETH_PHY_REG_RBR            0x17U
#define NETWORK_ETH_PHY_REG_PHYCR          0x19U
#define NETWORK_ETH_PHY_REG_10BTSCR        0x1AU

#define NETWORK_ETH_BMCR_LOOPBACK          0x4000U
#define NETWORK_ETH_BMCR_AN_ENABLE         0x1000U
#define NETWORK_ETH_BMCR_POWER_DOWN        0x0800U
#define NETWORK_ETH_BMCR_ISOLATE           0x0400U

#define NETWORK_ETH_PHYSTS_LINK_STATUS     0x0001U
#define NETWORK_ETH_PHYSTS_SPEED_10        0x0002U
#define NETWORK_ETH_PHYSTS_DUPLEX          0x0004U

#define NETWORK_ETH_RBR_RMII_MASTER        0x4000U
#define NETWORK_ETH_RBR_PMD_LOOP           0x0100U
#define NETWORK_ETH_RBR_SCMII_RX           0x0080U
#define NETWORK_ETH_RBR_SCMII_TX           0x0040U
#define NETWORK_ETH_RBR_RMII_MODE          0x0020U
#define NETWORK_ETH_RBR_RMII_REV1_0        0x0010U

#define NETWORK_ETH_PHYCR_PHYADDR_MASK     0x001FU

#define NETWORK_ETH_10BTSCR_LP_DIS         0x0080U
#define NETWORK_ETH_10BTSCR_FORCE_LINK_10  0x0040U

#define NETWORK_ETH_PHYID1_EXPECTED        0x2000U
#define NETWORK_ETH_PHYID2_MASK            0xFFF0U
#define NETWORK_ETH_PHYID2_EXPECTED        0x5CE0U

typedef enum {
    NETWORK_ETH_STATE_PHY_FAIL = 0,
    NETWORK_ETH_STATE_NO_LINK,
    NETWORK_ETH_STATE_DHCP_IN_PROGRESS,
    NETWORK_ETH_STATE_LEASED_WAITING_PING,
    NETWORK_ETH_STATE_QUALIFIED_PASS,
    NETWORK_ETH_STATE_QUALIFIED_FAIL
} network_eth_state_t;

typedef enum {
    NETWORK_ETH_RX_FREE = 0,
    NETWORK_ETH_RX_DMA,
    NETWORK_ETH_RX_PENDING
} network_eth_rx_buffer_state_t;

typedef struct network_eth_rx_fragment {
    uint8_t *buffer;
    uint16_t len;
    uint8_t buffer_index;
    uint8_t in_use;
    struct network_eth_rx_fragment *next;
} network_eth_rx_fragment_t;

typedef struct {
    uint8_t boot_started;
    uint8_t init_ok;
    uint8_t suspended;
    uint8_t phy_valid;
    uint8_t link_up;
    uint8_t mac_started;
    uint8_t dhcp_started;
    uint8_t dhcp_bound;
    uint8_t ping_window_active;
    uint8_t speed_mbps;
    uint8_t full_duplex;
    uint8_t phy_addr;
    uint16_t phy_id1;
    uint16_t phy_id2;
    uint16_t bmcr;
    uint16_t bmsr;
    uint16_t physts;
    uint16_t rbr;
    uint16_t phycr;
    uint16_t tenbtscr;
    uint32_t ping_request_count;
    uint32_t ping_reply_count;
    uint32_t last_link_poll_ms;
    uint32_t dhcp_deadline_ms;
    uint32_t ping_deadline_ms;
    network_eth_state_t state;
    char detail[160];
} network_eth_context_t;

static ETH_HandleTypeDef s_eth_handle;
static struct netif s_netif;
static ETH_TxPacketConfigTypeDef s_tx_config;
static network_eth_context_t s_eth_ctx;

static ETH_DMADescTypeDef s_eth_rx_desc[ETH_RX_DESC_CNT]
    __attribute__((aligned(32), section(".eth_rx_desc")));
static ETH_DMADescTypeDef s_eth_tx_desc[ETH_TX_DESC_CNT]
    __attribute__((aligned(32), section(".eth_tx_desc")));
static uint8_t s_eth_rx_buffers[NETWORK_ETH_RX_BUFFER_COUNT][NETWORK_ETH_RX_BUFFER_SIZE]
    __attribute__((aligned(32), section(".eth_rx_buf")));

static uint8_t s_eth_tx_buffer[NETWORK_ETH_TX_BUFFER_SIZE] __attribute__((aligned(32)));
static uint8_t s_eth_rx_buffer_state[NETWORK_ETH_RX_BUFFER_COUNT];
static network_eth_rx_fragment_t s_eth_rx_fragments[NETWORK_ETH_RX_BUFFER_COUNT];
static uint8_t s_eth_mac_addr[6];

u32_t sys_now(void)
{
    return (u32_t)HAL_GetTick();
}

u32_t sys_jiffies(void)
{
    return (u32_t)HAL_GetTick();
}

static void network_eth_cache_clean(const void *addr, size_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    uintptr_t start;
    uintptr_t end;

    if ((addr == NULL) || (len == 0U) || ((SCB->CCR & SCB_CCR_DC_Msk) == 0U)) {
        return;
    }

    start = ((uintptr_t)addr) & ~(uintptr_t)31U;
    end = (((uintptr_t)addr + len) + 31U) & ~(uintptr_t)31U;
    SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
#else
    (void)addr;
    (void)len;
#endif
}

static void network_eth_cache_invalidate(const void *addr, size_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    uintptr_t start;
    uintptr_t end;

    if ((addr == NULL) || (len == 0U) || ((SCB->CCR & SCB_CCR_DC_Msk) == 0U)) {
        return;
    }

    start = ((uintptr_t)addr) & ~(uintptr_t)31U;
    end = (((uintptr_t)addr + len) + 31U) & ~(uintptr_t)31U;
    SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
#else
    (void)addr;
    (void)len;
#endif
}

static const char *network_eth_state_name(network_eth_state_t state)
{
    switch (state) {
    case NETWORK_ETH_STATE_PHY_FAIL:
        return "PHY_FAIL";
    case NETWORK_ETH_STATE_NO_LINK:
        return "NO_LINK";
    case NETWORK_ETH_STATE_DHCP_IN_PROGRESS:
        return "DHCP_IN_PROGRESS";
    case NETWORK_ETH_STATE_LEASED_WAITING_PING:
        return "LEASED_WAITING_PING";
    case NETWORK_ETH_STATE_QUALIFIED_PASS:
        return "QUALIFIED_PASS";
    case NETWORK_ETH_STATE_QUALIFIED_FAIL:
        return "QUALIFIED_FAIL";
    default:
        return "UNKNOWN";
    }
}

static uint32_t network_eth_time_remaining_ms(uint32_t deadline_ms)
{
    uint32_t now = HAL_GetTick();

    if ((deadline_ms == 0U) || ((int32_t)(deadline_ms - now) <= 0)) {
        return 0U;
    }

    return deadline_ms - now;
}

static void network_eth_zero_ip(ip4_addr_t *addr)
{
    if (addr != NULL) {
        IP4_ADDR(addr, 0U, 0U, 0U, 0U);
    }
}

static void network_eth_clear_netif_addresses(void)
{
    ip4_addr_t zero;

    network_eth_zero_ip(&zero);
    (void)netifapi_netif_set_addr(&s_netif, &zero, &zero, &zero);
}

static void network_eth_set_detail(const char *detail)
{
    if (detail == NULL) {
        s_eth_ctx.detail[0] = '\0';
        return;
    }

    (void)snprintf(s_eth_ctx.detail, sizeof(s_eth_ctx.detail), "%s", detail);
}

static void network_eth_build_mac_address(void)
{
    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();
    uint32_t uid2 = HAL_GetUIDw2();

    s_eth_mac_addr[0] = 0x02U;
    s_eth_mac_addr[1] = (uint8_t)(uid0 & 0xFFU);
    s_eth_mac_addr[2] = (uint8_t)((uid0 >> 8) & 0xFFU);
    s_eth_mac_addr[3] = (uint8_t)((uid0 >> 16) & 0xFFU);
    s_eth_mac_addr[4] = (uint8_t)(uid1 & 0xFFU);
    s_eth_mac_addr[5] = (uint8_t)(uid2 & 0xFFU);
}

static void network_eth_format_mac(char *dst, size_t dst_len)
{
    if ((dst == NULL) || (dst_len == 0U)) {
        return;
    }

    (void)snprintf(dst,
                   dst_len,
                   "%02X:%02X:%02X:%02X:%02X:%02X",
                   s_eth_mac_addr[0],
                   s_eth_mac_addr[1],
                   s_eth_mac_addr[2],
                   s_eth_mac_addr[3],
                   s_eth_mac_addr[4],
                   s_eth_mac_addr[5]);
}

static void network_eth_format_ip(char *dst, size_t dst_len, const ip4_addr_t *addr)
{
    if ((dst == NULL) || (dst_len == 0U)) {
        return;
    }

    if (addr == NULL) {
        dst[0] = '\0';
        return;
    }

    if (ip4addr_ntoa_r(addr, dst, (int)dst_len) == NULL) {
        (void)snprintf(dst, dst_len, "0.0.0.0");
    }
}

static void network_eth_release_rx_fragments(network_eth_rx_fragment_t *fragment)
{
    while (fragment != NULL) {
        network_eth_rx_fragment_t *next = fragment->next;

        if (fragment->buffer_index < NETWORK_ETH_RX_BUFFER_COUNT) {
            s_eth_rx_buffer_state[fragment->buffer_index] = NETWORK_ETH_RX_FREE;
        }
        fragment->buffer = NULL;
        fragment->len = 0U;
        fragment->buffer_index = 0U;
        fragment->in_use = 0U;
        fragment->next = NULL;
        fragment = next;
    }
}

static network_eth_rx_fragment_t *network_eth_alloc_fragment(void)
{
    size_t i;

    for (i = 0U; i < NETWORK_ETH_RX_BUFFER_COUNT; ++i) {
        if (s_eth_rx_fragments[i].in_use == 0U) {
            s_eth_rx_fragments[i].in_use = 1U;
            s_eth_rx_fragments[i].next = NULL;
            return &s_eth_rx_fragments[i];
        }
    }

    return NULL;
}

static int network_eth_buffer_index_from_ptr(const uint8_t *buffer)
{
    size_t i;

    if (buffer == NULL) {
        return -1;
    }

    for (i = 0U; i < NETWORK_ETH_RX_BUFFER_COUNT; ++i) {
        if (buffer == s_eth_rx_buffers[i]) {
            return (int)i;
        }
    }

    return -1;
}

static bool network_eth_read_phy_reg(uint32_t reg, uint16_t *value)
{
    uint32_t raw = 0U;

    if ((value == NULL) || (s_eth_ctx.init_ok == 0U)) {
        return false;
    }

    if (HAL_ETH_ReadPHYRegister(&s_eth_handle, NETWORK_ETH_PHY_ADDR, reg, &raw) != HAL_OK) {
        return false;
    }

    *value = (uint16_t)(raw & 0xFFFFU);
    return true;
}

static bool network_eth_read_phy_reg_addr(uint8_t phy_addr, uint32_t reg, uint16_t *value)
{
    uint32_t raw = 0U;

    if ((value == NULL) || (s_eth_ctx.init_ok == 0U)) {
        return false;
    }

    if (HAL_ETH_ReadPHYRegister(&s_eth_handle, phy_addr, reg, &raw) != HAL_OK) {
        return false;
    }

    *value = (uint16_t)(raw & 0xFFFFU);
    return true;
}

static bool network_eth_scan_for_dp83640(uint8_t *found_addr, uint16_t *id1, uint16_t *id2)
{
    uint8_t addr;

    for (addr = 0U; addr < 32U; ++addr) {
        uint16_t local_id1 = 0U;
        uint16_t local_id2 = 0U;

        if (!network_eth_read_phy_reg_addr(addr, NETWORK_ETH_PHY_REG_PHYID1, &local_id1) ||
            !network_eth_read_phy_reg_addr(addr, NETWORK_ETH_PHY_REG_PHYID2, &local_id2)) {
            continue;
        }

        if ((local_id1 == NETWORK_ETH_PHYID1_EXPECTED) &&
            ((local_id2 & NETWORK_ETH_PHYID2_MASK) == NETWORK_ETH_PHYID2_EXPECTED)) {
            if (found_addr != NULL) {
                *found_addr = addr;
            }
            if (id1 != NULL) {
                *id1 = local_id1;
            }
            if (id2 != NULL) {
                *id2 = local_id2;
            }
            return true;
        }
    }

    return false;
}

static bool network_eth_validate_phy(void)
{
    uint16_t id1 = 0U;
    uint16_t id2 = 0U;
    uint16_t bmsr_first = 0U;
    uint8_t found_addr = 0xFFU;
    uint16_t found_id1 = 0U;
    uint16_t found_id2 = 0U;

    s_eth_ctx.phy_valid = 0U;
    s_eth_ctx.link_up = 0U;
    s_eth_ctx.speed_mbps = 0U;
    s_eth_ctx.full_duplex = 0U;
    s_eth_ctx.phy_addr = NETWORK_ETH_PHY_ADDR;

    if (!network_eth_read_phy_reg(NETWORK_ETH_PHY_REG_PHYID1, &id1) ||
        !network_eth_read_phy_reg(NETWORK_ETH_PHY_REG_PHYID2, &id2)) {
        if (network_eth_scan_for_dp83640(&found_addr, &found_id1, &found_id2)) {
            (void)snprintf(s_eth_ctx.detail,
                           sizeof(s_eth_ctx.detail),
                           "DP83640 detected at MDIO address %u, expected address %u.",
                           found_addr,
                           NETWORK_ETH_PHY_ADDR);
        } else {
            network_eth_set_detail("No DP83640 response at MDIO address 1.");
        }
        s_eth_ctx.state = NETWORK_ETH_STATE_PHY_FAIL;
        return false;
    }

    s_eth_ctx.phy_id1 = id1;
    s_eth_ctx.phy_id2 = id2;

    if ((id1 != NETWORK_ETH_PHYID1_EXPECTED) ||
        ((id2 & NETWORK_ETH_PHYID2_MASK) != NETWORK_ETH_PHYID2_EXPECTED)) {
        (void)snprintf(s_eth_ctx.detail,
                       sizeof(s_eth_ctx.detail),
                       "Unexpected PHY ID at address 1: id1=0x%04X id2=0x%04X.",
                       id1,
                       id2);
        s_eth_ctx.state = NETWORK_ETH_STATE_PHY_FAIL;
        return false;
    }

    if (!network_eth_read_phy_reg(NETWORK_ETH_PHY_REG_BMCR, &s_eth_ctx.bmcr) ||
        !network_eth_read_phy_reg(NETWORK_ETH_PHY_REG_BMSR, &bmsr_first) ||
        !network_eth_read_phy_reg(NETWORK_ETH_PHY_REG_BMSR, &s_eth_ctx.bmsr) ||
        !network_eth_read_phy_reg(NETWORK_ETH_PHY_REG_PHYSTS, &s_eth_ctx.physts) ||
        !network_eth_read_phy_reg(NETWORK_ETH_PHY_REG_RBR, &s_eth_ctx.rbr) ||
        !network_eth_read_phy_reg(NETWORK_ETH_PHY_REG_PHYCR, &s_eth_ctx.phycr) ||
        !network_eth_read_phy_reg(NETWORK_ETH_PHY_REG_10BTSCR, &s_eth_ctx.tenbtscr)) {
        network_eth_set_detail("DP83640 responded, but one or more PHY register reads failed.");
        s_eth_ctx.state = NETWORK_ETH_STATE_PHY_FAIL;
        return false;
    }

    (void)bmsr_first;

    if ((s_eth_ctx.phycr & NETWORK_ETH_PHYCR_PHYADDR_MASK) != NETWORK_ETH_PHY_ADDR) {
        (void)snprintf(s_eth_ctx.detail,
                       sizeof(s_eth_ctx.detail),
                       "PHYCR reports PHYADDR=%u, expected %u.",
                       (unsigned int)(s_eth_ctx.phycr & NETWORK_ETH_PHYCR_PHYADDR_MASK),
                       NETWORK_ETH_PHY_ADDR);
        s_eth_ctx.state = NETWORK_ETH_STATE_PHY_FAIL;
        return false;
    }

    if ((s_eth_ctx.rbr & NETWORK_ETH_RBR_RMII_MODE) == 0U) {
        network_eth_set_detail("DP83640 is not latched for RMII mode.");
        s_eth_ctx.state = NETWORK_ETH_STATE_PHY_FAIL;
        return false;
    }

    if ((s_eth_ctx.rbr & NETWORK_ETH_RBR_RMII_MASTER) == 0U) {
        network_eth_set_detail("DP83640 is not latched for RMII master mode.");
        s_eth_ctx.state = NETWORK_ETH_STATE_PHY_FAIL;
        return false;
    }

    if ((s_eth_ctx.rbr & (NETWORK_ETH_RBR_PMD_LOOP |
                          NETWORK_ETH_RBR_SCMII_RX |
                          NETWORK_ETH_RBR_SCMII_TX)) != 0U) {
        network_eth_set_detail("RBR shows loopback or single-clock MII mode enabled.");
        s_eth_ctx.state = NETWORK_ETH_STATE_PHY_FAIL;
        return false;
    }

    if ((s_eth_ctx.bmcr & (NETWORK_ETH_BMCR_LOOPBACK |
                           NETWORK_ETH_BMCR_POWER_DOWN |
                           NETWORK_ETH_BMCR_ISOLATE)) != 0U) {
        network_eth_set_detail("BMCR shows loopback, isolate, or power-down enabled.");
        s_eth_ctx.state = NETWORK_ETH_STATE_PHY_FAIL;
        return false;
    }

    if ((s_eth_ctx.tenbtscr & (NETWORK_ETH_10BTSCR_FORCE_LINK_10 |
                               NETWORK_ETH_10BTSCR_LP_DIS)) != 0U) {
        network_eth_set_detail("10BTSCR shows forced-link or NLP-disable enabled.");
        s_eth_ctx.state = NETWORK_ETH_STATE_PHY_FAIL;
        return false;
    }

    s_eth_ctx.phy_valid = 1U;
    s_eth_ctx.link_up = ((s_eth_ctx.physts & NETWORK_ETH_PHYSTS_LINK_STATUS) != 0U) ? 1U : 0U;
    s_eth_ctx.speed_mbps = ((s_eth_ctx.physts & NETWORK_ETH_PHYSTS_SPEED_10) != 0U) ? 10U : 100U;
    s_eth_ctx.full_duplex = ((s_eth_ctx.physts & NETWORK_ETH_PHYSTS_DUPLEX) != 0U) ? 1U : 0U;

    if (s_eth_ctx.link_up != 0U) {
        network_eth_set_detail("DP83640 validated at address 1 in RMII master mode.");
    } else {
        network_eth_set_detail("DP83640 validated at address 1; waiting for link.");
    }

    return true;
}

static void network_eth_stop_mac(void)
{
    if (s_eth_ctx.mac_started != 0U) {
        (void)HAL_ETH_Stop(&s_eth_handle);
        s_eth_ctx.mac_started = 0U;
    }

    (void)netifapi_netif_set_down(&s_netif);
    (void)netifapi_netif_set_link_down(&s_netif);
}

static void network_eth_reset_qualification(void)
{
    if (s_eth_ctx.dhcp_started != 0U) {
        (void)netifapi_dhcp_release_and_stop(&s_netif);
        s_eth_ctx.dhcp_started = 0U;
    }

    s_eth_ctx.dhcp_bound = 0U;
    s_eth_ctx.ping_window_active = 0U;
    s_eth_ctx.dhcp_deadline_ms = 0U;
    s_eth_ctx.ping_deadline_ms = 0U;
    s_eth_ctx.ping_request_count = 0U;
    s_eth_ctx.ping_reply_count = 0U;
    network_eth_clear_netif_addresses();
}

static void network_eth_begin_dhcp_session(bool force_restart)
{
    err_t err;

    if ((s_eth_ctx.phy_valid == 0U) || (s_eth_ctx.link_up == 0U)) {
        return;
    }

    s_eth_ctx.ping_window_active = 0U;
    s_eth_ctx.ping_deadline_ms = 0U;
    s_eth_ctx.ping_request_count = 0U;
    s_eth_ctx.ping_reply_count = 0U;
    s_eth_ctx.dhcp_bound = 0U;
    s_eth_ctx.dhcp_deadline_ms = HAL_GetTick() + NETWORK_ETH_DHCP_TIMEOUT_MS;

    if (force_restart && (s_eth_ctx.dhcp_started != 0U)) {
        (void)netifapi_dhcp_release_and_stop(&s_netif);
        s_eth_ctx.dhcp_started = 0U;
        network_eth_clear_netif_addresses();
    }

    if (s_eth_ctx.dhcp_started != 0U) {
        s_eth_ctx.state = NETWORK_ETH_STATE_DHCP_IN_PROGRESS;
        network_eth_set_detail("Link is up; waiting for DHCP lease.");
        return;
    }

    err = netifapi_dhcp_start(&s_netif);
    if (err != ERR_OK) {
        (void)snprintf(s_eth_ctx.detail,
                       sizeof(s_eth_ctx.detail),
                       "dhcp_start failed with err=%d.",
                       (int)err);
        s_eth_ctx.state = NETWORK_ETH_STATE_QUALIFIED_FAIL;
        return;
    }

    s_eth_ctx.dhcp_started = 1U;
    s_eth_ctx.state = NETWORK_ETH_STATE_DHCP_IN_PROGRESS;
    network_eth_set_detail("Link is up; waiting for DHCP lease.");
}

static void network_eth_open_ping_window(void)
{
    s_eth_ctx.ping_request_count = 0U;
    s_eth_ctx.ping_reply_count = 0U;
    s_eth_ctx.ping_window_active = 1U;
    s_eth_ctx.ping_deadline_ms = HAL_GetTick() + NETWORK_ETH_PING_TIMEOUT_MS;
    s_eth_ctx.state = NETWORK_ETH_STATE_LEASED_WAITING_PING;
    network_eth_set_detail("DHCP lease acquired; waiting for an external ping.");
}

static bool network_eth_packet_matches_local_request(const uint8_t *frame, size_t frame_len)
{
    const struct eth_hdr *eth_hdr;
    const struct ip_hdr *ip_hdr;
    const struct icmp_echo_hdr *icmp_hdr;
    const uint8_t *ip_ptr;
    size_t ip_header_len;

    if ((frame == NULL) || (frame_len < (SIZEOF_ETH_HDR + IP_HLEN + sizeof(struct icmp_echo_hdr)))) {
        return false;
    }

    eth_hdr = (const struct eth_hdr *)frame;
    if (lwip_ntohs(eth_hdr->type) != ETHTYPE_IP) {
        return false;
    }

    ip_ptr = frame + SIZEOF_ETH_HDR;
    ip_hdr = (const struct ip_hdr *)ip_ptr;
    ip_header_len = (size_t)IPH_HL_BYTES(ip_hdr);
    if ((IPH_V(ip_hdr) != 4U) ||
        (IPH_PROTO(ip_hdr) != NETWORK_ETH_IP_PROTO_ICMP) ||
        (ip_header_len < IP_HLEN) ||
        (frame_len < (SIZEOF_ETH_HDR + ip_header_len + sizeof(struct icmp_echo_hdr)))) {
        return false;
    }

    icmp_hdr = (const struct icmp_echo_hdr *)(ip_ptr + ip_header_len);
    if (ICMPH_TYPE(icmp_hdr) != ICMP_ECHO) {
        return false;
    }

    return (ip_hdr->dest.addr == netif_ip4_addr(&s_netif)->addr);
}

static bool network_eth_packet_matches_local_reply(const uint8_t *frame, size_t frame_len)
{
    const struct eth_hdr *eth_hdr;
    const struct ip_hdr *ip_hdr;
    const struct icmp_echo_hdr *icmp_hdr;
    const uint8_t *ip_ptr;
    size_t ip_header_len;

    if ((frame == NULL) || (frame_len < (SIZEOF_ETH_HDR + IP_HLEN + sizeof(struct icmp_echo_hdr)))) {
        return false;
    }

    eth_hdr = (const struct eth_hdr *)frame;
    if (lwip_ntohs(eth_hdr->type) != ETHTYPE_IP) {
        return false;
    }

    ip_ptr = frame + SIZEOF_ETH_HDR;
    ip_hdr = (const struct ip_hdr *)ip_ptr;
    ip_header_len = (size_t)IPH_HL_BYTES(ip_hdr);
    if ((IPH_V(ip_hdr) != 4U) ||
        (IPH_PROTO(ip_hdr) != NETWORK_ETH_IP_PROTO_ICMP) ||
        (ip_header_len < IP_HLEN) ||
        (frame_len < (SIZEOF_ETH_HDR + ip_header_len + sizeof(struct icmp_echo_hdr)))) {
        return false;
    }

    icmp_hdr = (const struct icmp_echo_hdr *)(ip_ptr + ip_header_len);
    if (ICMPH_TYPE(icmp_hdr) != ICMP_ER) {
        return false;
    }

    return (ip_hdr->src.addr == netif_ip4_addr(&s_netif)->addr);
}

static void network_eth_count_rx_ping_request(const uint8_t *frame, size_t frame_len)
{
    if ((s_eth_ctx.dhcp_bound != 0U) &&
        (s_eth_ctx.ping_window_active != 0U) &&
        network_eth_packet_matches_local_request(frame, frame_len)) {
        s_eth_ctx.ping_request_count++;
    }
}

static void network_eth_count_tx_ping_reply(const uint8_t *frame, size_t frame_len)
{
    if ((s_eth_ctx.dhcp_bound != 0U) &&
        (s_eth_ctx.ping_window_active != 0U) &&
        network_eth_packet_matches_local_reply(frame, frame_len)) {
        s_eth_ctx.ping_reply_count++;
    }
}

static void network_eth_update_state_machine(void)
{
    bool bound = false;
    bool was_bound = (s_eth_ctx.dhcp_bound != 0U);
    uint32_t now = HAL_GetTick();

    if ((s_eth_ctx.init_ok == 0U) || (s_eth_ctx.phy_valid == 0U)) {
        return;
    }

    bound = (dhcp_supplied_address(&s_netif) != 0) ? true : false;

    if (bound) {
        s_eth_ctx.dhcp_bound = 1U;
        if ((!was_bound) &&
            (s_eth_ctx.state != NETWORK_ETH_STATE_QUALIFIED_PASS) &&
            (s_eth_ctx.state != NETWORK_ETH_STATE_QUALIFIED_FAIL)) {
            network_eth_open_ping_window();
        }

        if ((s_eth_ctx.ping_request_count > 0U) && (s_eth_ctx.ping_reply_count > 0U)) {
            s_eth_ctx.ping_window_active = 0U;
            s_eth_ctx.ping_deadline_ms = 0U;
            s_eth_ctx.state = NETWORK_ETH_STATE_QUALIFIED_PASS;
            network_eth_set_detail("Lease acquired and external ping exchange observed.");
        } else if ((s_eth_ctx.ping_window_active != 0U) &&
                   ((int32_t)(s_eth_ctx.ping_deadline_ms - now) <= 0)) {
            s_eth_ctx.ping_window_active = 0U;
            s_eth_ctx.state = NETWORK_ETH_STATE_QUALIFIED_FAIL;
            network_eth_set_detail("Ping qualification timed out after DHCP lease.");
        } else if ((s_eth_ctx.ping_window_active != 0U) &&
                   (s_eth_ctx.state != NETWORK_ETH_STATE_QUALIFIED_PASS)) {
            s_eth_ctx.state = NETWORK_ETH_STATE_LEASED_WAITING_PING;
            network_eth_set_detail("DHCP lease acquired; waiting for an external ping.");
        }
        return;
    }

    s_eth_ctx.dhcp_bound = 0U;
    if (s_eth_ctx.dhcp_started == 0U) {
        return;
    }

    if ((int32_t)(s_eth_ctx.dhcp_deadline_ms - now) <= 0) {
        s_eth_ctx.state = NETWORK_ETH_STATE_QUALIFIED_FAIL;
        network_eth_set_detail("DHCP lease was not acquired before timeout.");
    } else {
        s_eth_ctx.state = NETWORK_ETH_STATE_DHCP_IN_PROGRESS;
        network_eth_set_detail("Link is up; waiting for DHCP lease.");
    }
}

static void network_eth_service_rx(void)
{
    void *packet = NULL;

    while (HAL_ETH_ReadData(&s_eth_handle, &packet) == HAL_OK) {
        network_eth_rx_fragment_t *fragment = (network_eth_rx_fragment_t *)packet;
        struct pbuf *pbuf = NULL;
        size_t total_len = 0U;
        size_t offset = 0U;
        network_eth_rx_fragment_t *iter = NULL;

        for (iter = fragment; iter != NULL; iter = iter->next) {
            total_len += iter->len;
        }

        if ((total_len == 0U) || (total_len > NETWORK_ETH_RX_BUFFER_SIZE)) {
            network_eth_release_rx_fragments(fragment);
            packet = NULL;
            continue;
        }

        pbuf = pbuf_alloc(PBUF_RAW, (u16_t)total_len, PBUF_RAM);
        if ((pbuf == NULL) || (pbuf->payload == NULL)) {
            if (pbuf != NULL) {
                pbuf_free(pbuf);
            }
            network_eth_release_rx_fragments(fragment);
            packet = NULL;
            continue;
        }

        for (iter = fragment; iter != NULL; iter = iter->next) {
            network_eth_cache_invalidate(iter->buffer, iter->len);
            (void)memcpy((uint8_t *)pbuf->payload + offset, iter->buffer, iter->len);
            offset += iter->len;
        }

        network_eth_count_rx_ping_request((const uint8_t *)pbuf->payload, total_len);
        network_eth_release_rx_fragments(fragment);

        if (s_netif.input(pbuf, &s_netif) != ERR_OK) {
            pbuf_free(pbuf);
        }

        packet = NULL;
    }
}

static err_t network_eth_low_level_output(struct netif *netif, struct pbuf *pbuf)
{
    ETH_BufferTypeDef tx_buffer;
    struct pbuf *q;
    size_t total_len = 0U;

    (void)netif;

    if ((s_eth_ctx.mac_started == 0U) || (pbuf == NULL)) {
        return ERR_IF;
    }

    for (q = pbuf; q != NULL; q = q->next) {
        if ((total_len + q->len) > sizeof(s_eth_tx_buffer)) {
            return ERR_IF;
        }
        (void)memcpy(s_eth_tx_buffer + total_len, q->payload, q->len);
        total_len += q->len;
    }

    network_eth_count_tx_ping_reply(s_eth_tx_buffer, total_len);
    network_eth_cache_clean(s_eth_tx_buffer, total_len);

    tx_buffer.buffer = s_eth_tx_buffer;
    tx_buffer.len = (uint32_t)total_len;
    tx_buffer.next = NULL;

    s_tx_config.Length = (uint32_t)total_len;
    s_tx_config.TxBuffer = &tx_buffer;
    s_tx_config.pData = s_eth_tx_buffer;
    s_eth_handle.TxDescList.CurrentPacketAddress = (uint32_t *)s_eth_tx_buffer;

    if (HAL_ETH_Transmit(&s_eth_handle, &s_tx_config, NETWORK_ETH_TX_TIMEOUT_MS) != HAL_OK) {
        return ERR_IF;
    }

    (void)HAL_ETH_ReleaseTxPacket(&s_eth_handle);
    return ERR_OK;
}

static void network_eth_apply_mac_config(void)
{
    ETH_MACConfigTypeDef mac_config;

    if (HAL_ETH_GetMACConfig(&s_eth_handle, &mac_config) != HAL_OK) {
        return;
    }

    mac_config.Speed = (s_eth_ctx.speed_mbps == 10U) ? ETH_SPEED_10M : ETH_SPEED_100M;
    mac_config.DuplexMode = (s_eth_ctx.full_duplex != 0U) ? ETH_FULLDUPLEX_MODE : ETH_HALFDUPLEX_MODE;
    (void)HAL_ETH_SetMACConfig(&s_eth_handle, &mac_config);
}

static err_t network_eth_netif_init(struct netif *netif)
{
    ip4_addr_t zero;

    LWIP_ASSERT("netif != NULL", (netif != NULL));

    network_eth_zero_ip(&zero);

    netif->hostname = NETWORK_ETH_HOSTNAME;
    netif->name[0] = 'e';
    netif->name[1] = 't';
    netif->output = etharp_output;
    netif->linkoutput = network_eth_low_level_output;
    netif->hwaddr_len = ETH_HWADDR_LEN;
    (void)memcpy(netif->hwaddr, s_eth_mac_addr, sizeof(s_eth_mac_addr));
    netif->mtu = 1500U;
    netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET;

    memset(&s_eth_handle, 0, sizeof(s_eth_handle));
    s_eth_handle.Instance = ETH;
    s_eth_handle.Init.MACAddr = s_eth_mac_addr;
    s_eth_handle.Init.MediaInterface = HAL_ETH_RMII_MODE;
    s_eth_handle.Init.RxDesc = s_eth_rx_desc;
    s_eth_handle.Init.TxDesc = s_eth_tx_desc;
    s_eth_handle.Init.RxBuffLen = NETWORK_ETH_RX_BUFFER_SIZE;

    if (HAL_ETH_Init(&s_eth_handle) != HAL_OK) {
        s_eth_ctx.init_ok = 0U;
        s_eth_ctx.state = NETWORK_ETH_STATE_PHY_FAIL;
        network_eth_set_detail("HAL_ETH_Init failed.");
        return ERR_IF;
    }

    memset(&s_tx_config, 0, sizeof(s_tx_config));
    s_tx_config.Attributes = ETH_TX_PACKETS_FEATURES_CRCPAD;
    s_tx_config.CRCPadCtrl = ETH_CRC_PAD_INSERT;
    s_tx_config.ChecksumCtrl = ETH_CHECKSUM_DISABLE;

    network_eth_clear_netif_addresses();
    netif_set_addr(netif, &zero, &zero, &zero);
    s_eth_ctx.init_ok = 1U;
    return ERR_OK;
}

void HAL_ETH_MspInit(ETH_HandleTypeDef *heth)
{
    (void)heth;

    __HAL_RCC_ETH1MAC_CLK_ENABLE();
    __HAL_RCC_ETH1TX_CLK_ENABLE();
    __HAL_RCC_ETH1RX_CLK_ENABLE();
}

void HAL_ETH_MspDeInit(ETH_HandleTypeDef *heth)
{
    (void)heth;

    __HAL_RCC_ETH1MAC_CLK_DISABLE();
    __HAL_RCC_ETH1TX_CLK_DISABLE();
    __HAL_RCC_ETH1RX_CLK_DISABLE();
}

void HAL_ETH_RxAllocateCallback(uint8_t **buffer)
{
    size_t i;

    if (buffer == NULL) {
        return;
    }

    *buffer = NULL;

    for (i = 0U; i < NETWORK_ETH_RX_BUFFER_COUNT; ++i) {
        if (s_eth_rx_buffer_state[i] == NETWORK_ETH_RX_FREE) {
            s_eth_rx_buffer_state[i] = NETWORK_ETH_RX_DMA;
            network_eth_cache_invalidate(s_eth_rx_buffers[i], NETWORK_ETH_RX_BUFFER_SIZE);
            *buffer = s_eth_rx_buffers[i];
            return;
        }
    }
}

void HAL_ETH_RxLinkCallback(void **p_start, void **p_end, uint8_t *buffer, uint16_t length)
{
    int buffer_index;
    network_eth_rx_fragment_t *fragment;

    if ((p_start == NULL) || (p_end == NULL) || (buffer == NULL)) {
        return;
    }

    buffer_index = network_eth_buffer_index_from_ptr(buffer);
    if (buffer_index < 0) {
        return;
    }

    fragment = network_eth_alloc_fragment();
    if (fragment == NULL) {
        s_eth_rx_buffer_state[buffer_index] = NETWORK_ETH_RX_FREE;
        return;
    }

    fragment->buffer = buffer;
    fragment->len = length;
    fragment->buffer_index = (uint8_t)buffer_index;
    fragment->next = NULL;
    s_eth_rx_buffer_state[buffer_index] = NETWORK_ETH_RX_PENDING;

    if (*p_start == NULL) {
        *p_start = fragment;
    } else {
        ((network_eth_rx_fragment_t *)(*p_end))->next = fragment;
    }

    *p_end = fragment;
}

static void network_eth_format_measured(char *dst, size_t dst_len)
{
    const char *speed = (s_eth_ctx.speed_mbps == 0U) ? "unknown" :
                        ((s_eth_ctx.speed_mbps == 10U) ? "10M" : "100M");
    const char *duplex = (s_eth_ctx.full_duplex != 0U) ? "full" : "half";

    if ((dst == NULL) || (dst_len == 0U)) {
        return;
    }

    (void)snprintf(dst,
                   dst_len,
                   "st=%s,phy=%u,id=%04X:%04X,rbr=%04X,phycr=%04X,link=%s,spd=%s,dup=%s,ping=%lu/%lu",
                   network_eth_state_name(s_eth_ctx.state),
                   s_eth_ctx.phy_addr,
                   s_eth_ctx.phy_id1,
                   s_eth_ctx.phy_id2,
                   s_eth_ctx.rbr,
                   s_eth_ctx.phycr,
                   (s_eth_ctx.link_up != 0U) ? "up" : "down",
                   speed,
                   duplex,
                   (unsigned long)s_eth_ctx.ping_request_count,
                   (unsigned long)s_eth_ctx.ping_reply_count);
}

static void network_eth_print_lease_line(void)
{
    char mac[18];
    char ip[16];
    char netmask[16];
    char gateway[16];

    network_eth_format_mac(mac, sizeof(mac));
    network_eth_format_ip(ip, sizeof(ip), netif_ip4_addr(&s_netif));
    network_eth_format_ip(netmask, sizeof(netmask), netif_ip4_netmask(&s_netif));
    network_eth_format_ip(gateway, sizeof(gateway), netif_ip4_gw(&s_netif));

    test_uart_printf("ETH lease: mac=%s ip=%s mask=%s gw=%s\r\n", mac, ip, netmask, gateway);
}

static void network_eth_restart_link_test_session(void)
{
    if ((s_eth_ctx.phy_valid == 0U) || (s_eth_ctx.link_up == 0U)) {
        return;
    }

    if (s_eth_ctx.dhcp_bound != 0U) {
        network_eth_open_ping_window();
    } else {
        network_eth_begin_dhcp_session(true);
    }
}

void network_ethernet_boot_start(void)
{
    ip4_addr_t zero;

    if (s_eth_ctx.boot_started != 0U) {
        return;
    }

    memset(&s_eth_ctx, 0, sizeof(s_eth_ctx));
    memset(s_eth_rx_buffer_state, 0, sizeof(s_eth_rx_buffer_state));
    memset(s_eth_rx_fragments, 0, sizeof(s_eth_rx_fragments));
    network_eth_build_mac_address();
    s_eth_ctx.state = NETWORK_ETH_STATE_PHY_FAIL;
    s_eth_ctx.phy_addr = NETWORK_ETH_PHY_ADDR;
    s_eth_ctx.boot_started = 1U;

    if (!network_stack_is_ready()) {
        s_eth_ctx.init_ok = 0U;
        network_eth_set_detail("lwIP tcpip thread is not ready.");
        return;
    }

    network_eth_zero_ip(&zero);
    if (netifapi_netif_add(&s_netif, &zero, &zero, &zero, NULL, network_eth_netif_init, tcpip_input) != ERR_OK) {
        s_eth_ctx.init_ok = 0U;
        s_eth_ctx.state = NETWORK_ETH_STATE_PHY_FAIL;
        network_eth_set_detail("netif_add failed.");
        return;
    }

    (void)netifapi_netif_set_default(&s_netif);
    (void)netifapi_netif_set_link_down(&s_netif);
    (void)netifapi_netif_set_down(&s_netif);
    network_ethernet_poll();
}

void network_ethernet_poll(void)
{
    uint32_t now;

    if ((s_eth_ctx.boot_started == 0U) || (s_eth_ctx.init_ok == 0U)) {
        return;
    }

    if (s_eth_ctx.suspended != 0U) {
        return;
    }

    now = HAL_GetTick();

    if ((s_eth_ctx.last_link_poll_ms == 0U) ||
        ((now - s_eth_ctx.last_link_poll_ms) >= NETWORK_ETH_LINK_POLL_MS)) {
        s_eth_ctx.last_link_poll_ms = now;

        if (!network_eth_validate_phy()) {
            network_eth_stop_mac();
            network_eth_reset_qualification();
            s_eth_ctx.state = NETWORK_ETH_STATE_PHY_FAIL;
            return;
        }

        if (s_eth_ctx.link_up == 0U) {
            network_eth_stop_mac();
            network_eth_reset_qualification();
            s_eth_ctx.state = NETWORK_ETH_STATE_NO_LINK;
            network_eth_set_detail("DP83640 validated; no live Ethernet link.");
            return;
        }

        if (s_eth_ctx.mac_started == 0U) {
            network_eth_apply_mac_config();
            if (HAL_ETH_Start(&s_eth_handle) != HAL_OK) {
                s_eth_ctx.state = NETWORK_ETH_STATE_QUALIFIED_FAIL;
                network_eth_set_detail("HAL_ETH_Start failed after link-up.");
                return;
            }

            s_eth_ctx.mac_started = 1U;
            (void)netifapi_netif_set_up(&s_netif);
            (void)netifapi_netif_set_link_up(&s_netif);
        }

        if ((s_eth_ctx.dhcp_started == 0U) && (s_eth_ctx.dhcp_bound == 0U)) {
            network_eth_begin_dhcp_session(false);
        }
    }

    if (s_eth_ctx.mac_started != 0U) {
        network_eth_service_rx();
        (void)HAL_ETH_ReleaseTxPacket(&s_eth_handle);
    }

    network_eth_update_state_machine();
}

void network_ethernet_suspend(void)
{
    if (s_eth_ctx.boot_started == 0U) {
        return;
    }

    network_eth_stop_mac();
    network_eth_reset_qualification();
    s_eth_ctx.suspended = 1U;
    s_eth_ctx.state = NETWORK_ETH_STATE_NO_LINK;
    network_eth_set_detail("Ethernet suspended by network manager.");
}

void network_ethernet_resume(void)
{
    network_ethernet_boot_start();
    s_eth_ctx.suspended = 0U;
    network_ethernet_poll();
}

void network_ethernet_test_phy(board_test_result_t *result)
{
    char measured[128];

    network_ethernet_boot_start();
    network_ethernet_poll();
    network_eth_format_measured(measured, sizeof(measured));

    if ((s_eth_ctx.init_ok == 0U) || (s_eth_ctx.phy_valid == 0U)) {
        board_test_set_result(result,
                              "eth.phy",
                              "DP83640 PHY",
                              TEST_STATUS_FAIL,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              s_eth_ctx.detail,
                              "Check DP83640 power, RMII master-mode straps, and MDIO address straps.");
        return;
    }

    board_test_set_result(result,
                          "eth.phy",
                          "DP83640 PHY",
                          TEST_STATUS_PASS,
                          TEST_MODE_AUTOMATIC,
                          measured,
                          s_eth_ctx.detail,
                          "");
}

void network_ethernet_test_link(board_test_result_t *result)
{
    char measured[128];
    uint32_t start_ms;
    uint8_t lease_announced = 0U;

    network_ethernet_boot_start();
    network_ethernet_poll();

    if ((s_eth_ctx.init_ok == 0U) || (s_eth_ctx.phy_valid == 0U)) {
        network_eth_format_measured(measured, sizeof(measured));
        board_test_set_result(result,
                              "eth.link",
                              "DP83640/RJ45",
                              TEST_STATUS_FAIL,
                              TEST_MODE_INTERACTIVE,
                              measured,
                              s_eth_ctx.detail,
                              "Check the DP83640 address-1 straps and RMII master-mode clocking.");
        return;
    }

    if (s_eth_ctx.link_up == 0U) {
        network_eth_format_measured(measured, sizeof(measured));
        board_test_set_result(result,
                              "eth.link",
                              "DP83640/RJ45",
                              TEST_STATUS_SKIP,
                              TEST_MODE_INTERACTIVE,
                              measured,
                              s_eth_ctx.detail,
                              "Connect an active Ethernet cable to a DHCP-enabled network and rerun 'run eth'.");
        return;
    }

    network_eth_restart_link_test_session();
    start_ms = HAL_GetTick();

    while ((HAL_GetTick() - start_ms) < NETWORK_ETH_LINK_TEST_TIMEOUT_MS) {
        network_ethernet_poll();

        if ((s_eth_ctx.dhcp_bound != 0U) && (lease_announced == 0U)) {
            network_eth_print_lease_line();
            test_uart_write_str("Ping the leased IP from another device within 30 seconds.\r\n");
            lease_announced = 1U;
        }

        if (s_eth_ctx.state == NETWORK_ETH_STATE_QUALIFIED_PASS) {
            break;
        }

        if ((s_eth_ctx.state == NETWORK_ETH_STATE_PHY_FAIL) ||
            (s_eth_ctx.state == NETWORK_ETH_STATE_NO_LINK) ||
            (s_eth_ctx.state == NETWORK_ETH_STATE_QUALIFIED_FAIL)) {
            break;
        }

        HAL_Delay(10U);
    }

    network_eth_format_measured(measured, sizeof(measured));

    if (s_eth_ctx.state == NETWORK_ETH_STATE_QUALIFIED_PASS) {
        board_test_set_result(result,
                              "eth.link",
                              "DP83640/RJ45",
                              TEST_STATUS_PASS,
                              TEST_MODE_INTERACTIVE,
                              measured,
                              "DP83640 validated, DHCP lease acquired, and external ping observed.",
                              "");
    } else if (s_eth_ctx.state == NETWORK_ETH_STATE_NO_LINK) {
        board_test_set_result(result,
                              "eth.link",
                              "DP83640/RJ45",
                              TEST_STATUS_SKIP,
                              TEST_MODE_INTERACTIVE,
                              measured,
                              s_eth_ctx.detail,
                              "Connect an active Ethernet cable to a DHCP-enabled network and rerun 'run eth'.");
    } else if ((s_eth_ctx.state == NETWORK_ETH_STATE_DHCP_IN_PROGRESS) ||
               ((s_eth_ctx.state == NETWORK_ETH_STATE_QUALIFIED_FAIL) && (s_eth_ctx.dhcp_bound == 0U))) {
        board_test_set_result(result,
                              "eth.link",
                              "DP83640/RJ45",
                              TEST_STATUS_FAIL,
                              TEST_MODE_INTERACTIVE,
                              measured,
                              s_eth_ctx.detail,
                              "Verify the connected network provides DHCP service to this port.");
    } else {
        board_test_set_result(result,
                              "eth.link",
                              "DP83640/RJ45",
                              TEST_STATUS_FAIL,
                              TEST_MODE_INTERACTIVE,
                              measured,
                              s_eth_ctx.detail,
                              "Ping the leased IP from another device while the qualification window is active.");
    }
}

void network_ethernet_print_info(void)
{
    char mac[18];
    char ip[16];
    char netmask[16];
    char gateway[16];

    network_ethernet_boot_start();
    network_ethernet_poll();

    network_eth_format_mac(mac, sizeof(mac));
    network_eth_format_ip(ip, sizeof(ip), netif_ip4_addr(&s_netif));
    network_eth_format_ip(netmask, sizeof(netmask), netif_ip4_netmask(&s_netif));
    network_eth_format_ip(gateway, sizeof(gateway), netif_ip4_gw(&s_netif));

    test_uart_write_str("ETH info:\r\n");
    test_uart_printf("  state=%s detail=%s\r\n",
                     network_eth_state_name(s_eth_ctx.state),
                     (s_eth_ctx.detail[0] != '\0') ? s_eth_ctx.detail : "n/a");
    test_uart_printf("  suspended=%u\r\n", s_eth_ctx.suspended);
    test_uart_printf("  expected_phy_addr=%u actual_phy_addr=%u\r\n",
                     NETWORK_ETH_PHY_ADDR,
                     (unsigned int)(s_eth_ctx.phycr & NETWORK_ETH_PHYCR_PHYADDR_MASK));
    test_uart_printf("  phy_id=0x%04X:0x%04X bmcr=0x%04X bmsr=0x%04X physts=0x%04X\r\n",
                     s_eth_ctx.phy_id1,
                     s_eth_ctx.phy_id2,
                     s_eth_ctx.bmcr,
                     s_eth_ctx.bmsr,
                     s_eth_ctx.physts);
    test_uart_printf("  rbr=0x%04X phycr=0x%04X 10btscr=0x%04X rmii=%u master=%u rev1_0=%u\r\n",
                     s_eth_ctx.rbr,
                     s_eth_ctx.phycr,
                     s_eth_ctx.tenbtscr,
                     (unsigned int)((s_eth_ctx.rbr & NETWORK_ETH_RBR_RMII_MODE) != 0U),
                     (unsigned int)((s_eth_ctx.rbr & NETWORK_ETH_RBR_RMII_MASTER) != 0U),
                     (unsigned int)((s_eth_ctx.rbr & NETWORK_ETH_RBR_RMII_REV1_0) != 0U));
    test_uart_printf("  mac=%s ip=%s mask=%s gw=%s\r\n", mac, ip, netmask, gateway);
    test_uart_printf("  link=%s speed=%u duplex=%s dhcp_remaining_ms=%lu ping_remaining_ms=%lu\r\n",
                     (s_eth_ctx.link_up != 0U) ? "up" : "down",
                     (unsigned int)s_eth_ctx.speed_mbps,
                     (s_eth_ctx.full_duplex != 0U) ? "full" : "half",
                     (unsigned long)network_eth_time_remaining_ms(s_eth_ctx.dhcp_deadline_ms),
                     (unsigned long)network_eth_time_remaining_ms(s_eth_ctx.ping_deadline_ms));
    test_uart_printf("  ping_requests=%lu ping_replies=%lu\r\n",
                     (unsigned long)s_eth_ctx.ping_request_count,
                     (unsigned long)s_eth_ctx.ping_reply_count);
}
