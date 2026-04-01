#include "board_test.h"

#include "test_uart.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define ETH_MDIO_MAX_PHY_ADDR 31U
#define ETH_LINK_WAIT_MS 10000U
#define ETH_LINK_OBSERVE_MS 1200U

/* DP83640 Clause-22 register addresses / bitfields. */
#define ETH_PHY_REG_BMCR 0U
#define ETH_PHY_REG_BMSR 1U
#define ETH_PHY_REG_PHYID1 2U
#define ETH_PHY_REG_PHYID2 3U
#define ETH_PHY_REG_PHYSTS 16U
#define ETH_PHY_REG_10BTSCR 0x1AU

#define ETH_BMCR_LOOPBACK (0x4000U)
#define ETH_BMCR_RESET (0x8000U)
#define ETH_BMCR_SPEED_SELECT (0x2000U)
#define ETH_BMCR_AN_ENABLE (0x1000U)
#define ETH_BMCR_POWER_DOWN (0x0800U)
#define ETH_BMCR_ISOLATE (0x0400U)
#define ETH_BMCR_RESTART_AN (0x0200U)
#define ETH_BMCR_DUPLEX_MODE (0x0100U)
#define ETH_BMSR_LINK_STATUS (0x0004U)
#define ETH_BMSR_AN_COMPLETE (0x0020U)
#define ETH_PHYSTS_LINK_STATUS (0x0001U)
#define ETH_PHYSTS_SPEED_10 (0x0002U)
#define ETH_PHYSTS_DUPLEX (0x0004U)
#define ETH_PHYSTS_LOOPBACK_STATUS (0x0008U)
#define ETH_PHYSTS_AN_COMPLETE (0x0010U)
#define ETH_PHYSTS_DESCRAMBLER_LOCK (0x0200U)
#define ETH_PHYSTS_SIGNAL_DETECT (0x0400U)
#define ETH_PHYSTS_PAGE_RECEIVED (0x0100U)

#define ETH_10BTSCR_LOOPBACK_10_DIS (0x0100U)
#define ETH_10BTSCR_LP_DIS (0x0080U)
#define ETH_10BTSCR_FORCE_LINK_10 (0x0040U)

/* Keep reset disabled in default run path to avoid forcing renegotiation before link check. */
#define ETH_PHY_SOFT_RESET_IN_PHY_TEST 0
/* Keep autoneg restart disabled in link test to avoid masking passive PHY status observation. */
#define ETH_PHY_RESTART_AN_IN_LINK_TEST 0
/* Keep active BMCR/10BTSCR recovery disabled by default; use passive observation for link qualification. */
#define ETH_PHY_ACTIVE_RECOVERY_IN_LINK_TEST 0

static uint8_t s_phy_addr = 0xFFU;
static uint16_t s_phy_id1 = 0U;
static uint16_t s_phy_id2 = 0U;
static uint8_t s_phy_found = 0U;

static void mdio_delay(void)
{
    volatile uint32_t i;
    /* Keep MDC well under 2.5MHz across HSE/HSI fallback clock trees. */
    for (i = 0U; i < 160U; ++i) {
        __NOP();
    }
}

static void mdc_set(GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, state);
    mdio_delay();
}

static void mdio_pin_output(void)
{
    GPIO_InitTypeDef gpio;

    memset(&gpio, 0, sizeof(gpio));
    gpio.Pin = GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void mdio_pin_input(void)
{
    GPIO_InitTypeDef gpio;

    memset(&gpio, 0, sizeof(gpio));
    gpio.Pin = GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void mdio_write_bit(uint8_t bit)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, (bit != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    mdc_set(GPIO_PIN_SET);
    mdc_set(GPIO_PIN_RESET);
}

static uint8_t mdio_read_bit(void)
{
    uint8_t bit;

    mdc_set(GPIO_PIN_SET);
    bit = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) ? 1U : 0U;
    mdc_set(GPIO_PIN_RESET);
    return bit;
}

static void mdio_send_bits(uint32_t value, uint8_t bit_count)
{
    int8_t i;

    for (i = (int8_t)(bit_count - 1U); i >= 0; --i) {
        mdio_write_bit((uint8_t)((value >> i) & 0x1U));
    }
}

static uint16_t mdio_recv_16(void)
{
    uint16_t value = 0U;
    uint8_t i;

    for (i = 0U; i < 16U; ++i) {
        value = (uint16_t)((value << 1U) | mdio_read_bit());
    }

    return value;
}

static void mdio_bus_init(void)
{
    GPIO_InitTypeDef gpio;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    memset(&gpio, 0, sizeof(gpio));
    gpio.Pin = GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOC, &gpio);

    mdio_pin_output();

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
}

static bool mdio_read_reg(uint8_t phy_addr, uint8_t reg_addr, uint16_t *out)
{
    uint16_t data;
    uint8_t ta;

    if (out == NULL) {
        return false;
    }

    mdio_bus_init();

    mdio_pin_output();
    mdio_send_bits(0xFFFFFFFFU, 32U);
    mdio_send_bits(0x1U, 2U); /* start */
    mdio_send_bits(0x2U, 2U); /* read */
    mdio_send_bits(phy_addr, 5U);
    mdio_send_bits(reg_addr, 5U);

    mdio_pin_input();
    (void)mdio_read_bit();
    ta = mdio_read_bit();
    data = mdio_recv_16();

    mdio_pin_output();
    mdio_write_bit(1U);

    if (ta != 0U) {
        return false;
    }

    *out = data;
    return true;
}

static bool mdio_write_reg(uint8_t phy_addr, uint8_t reg_addr, uint16_t value)
{
    mdio_bus_init();

    mdio_pin_output();
    mdio_send_bits(0xFFFFFFFFU, 32U);
    mdio_send_bits(0x1U, 2U); /* start */
    mdio_send_bits(0x1U, 2U); /* write */
    mdio_send_bits(phy_addr, 5U);
    mdio_send_bits(reg_addr, 5U);
    mdio_send_bits(0x2U, 2U); /* turnaround 10 */
    mdio_send_bits(value, 16U);
    mdio_write_bit(1U);

    return true;
}

static bool phy_scan(uint8_t *addr, uint16_t *id1, uint16_t *id2)
{
    uint8_t p;

    for (p = 0U; p <= ETH_MDIO_MAX_PHY_ADDR; ++p) {
        uint16_t r2 = 0U;
        uint16_t r3 = 0U;

        if (!mdio_read_reg(p, ETH_PHY_REG_PHYID1, &r2)) {
            continue;
        }
        if (!mdio_read_reg(p, ETH_PHY_REG_PHYID2, &r3)) {
            continue;
        }

        if ((r2 == 0x0000U) || (r2 == 0xFFFFU)) {
            continue;
        }
        if ((r3 == 0x0000U) || (r3 == 0xFFFFU)) {
            continue;
        }

        if (addr != NULL) {
            *addr = p;
        }
        if (id1 != NULL) {
            *id1 = r2;
        }
        if (id2 != NULL) {
            *id2 = r3;
        }
        return true;
    }

    return false;
}

void test_ethernet_phy(board_test_result_t *result)
{
    uint8_t addr;
    uint16_t id1;
    uint16_t id2;
    uint16_t bmcr;
    uint8_t reset_attempted = 0U;
    uint8_t reset_cleared = 0U;
    char measured[128];

    if (!phy_scan(&addr, &id1, &id2)) {
        s_phy_found = 0U;
        board_test_set_result(result,
                              "eth.phy",
                              "DP83640 PHY",
                              TEST_STATUS_FAIL,
                              TEST_MODE_AUTOMATIC,
                              "mdio_scan=none",
                              "No PHY responded on MDIO addresses 0..31.",
                              "Check VDD_ETH rail, MDIO pull-up (R39), and RMII/MDIO routing.");
        return;
    }

    s_phy_found = 1U;
    s_phy_addr = addr;
    s_phy_id1 = id1;
    s_phy_id2 = id2;

    if (!mdio_read_reg(addr, ETH_PHY_REG_BMCR, &bmcr)) {
        board_test_set_result(result,
                              "eth.phy",
                              "DP83640 PHY",
                              TEST_STATUS_FAIL,
                              TEST_MODE_AUTOMATIC,
                              "bmcr read failed",
                              "PHY ID was read but BMCR read failed.",
                              "Inspect MDC/MDIO integrity.");
        return;
    }

#if ETH_PHY_SOFT_RESET_IN_PHY_TEST
    uint32_t t0 = 0U;
    reset_attempted = 1U;
    (void)mdio_write_reg(addr, ETH_PHY_REG_BMCR, (uint16_t)(bmcr | ETH_BMCR_RESET));
    t0 = HAL_GetTick();
    do {
        if (!mdio_read_reg(addr, ETH_PHY_REG_BMCR, &bmcr)) {
            break;
        }
        if ((bmcr & ETH_BMCR_RESET) == 0U) {
            reset_cleared = 1U;
            break;
        }
    } while ((HAL_GetTick() - t0) < 200U);
#endif

    (void)snprintf(measured,
                   sizeof(measured),
                   "phy_addr=%u,id1=0x%04X,id2=0x%04X,bmcr=0x%04X,reset=%s",
                   addr,
                   id1,
                   id2,
                   bmcr,
                   (reset_attempted != 0U) ? ((reset_cleared != 0U) ? "ok" : "timeout") : "skipped");

    if ((reset_attempted == 0U) || (reset_cleared != 0U)) {
        board_test_set_result(result,
                              "eth.phy",
                              "DP83640 PHY",
                              TEST_STATUS_PASS,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "MDIO scan/read sequence succeeded.",
                              "");
    } else {
        board_test_set_result(result,
                              "eth.phy",
                              "DP83640 PHY",
                              TEST_STATUS_WARN,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "PHY reset bit did not clear within timeout.",
                              "Check PHY clock source (Y3 25MHz), strap network, and reset tree.");
    }
}

void test_ethernet_link(board_test_result_t *result)
{
    const uint8_t active_recovery = (uint8_t)ETH_PHY_ACTIVE_RECOVERY_IN_LINK_TEST;
    uint16_t bmsr = 0U;
    uint16_t bmsr_latched = 0U;
    uint16_t bmcr_before = 0U;
    uint16_t bmcr_after = 0U;
    uint16_t bmcr_target = 0U;
    uint16_t tenbtscr_before = 0U;
    uint16_t tenbtscr_after = 0U;
    uint16_t tenbtscr_target = 0U;
    uint16_t physts = 0U;
    uint8_t physts_valid = 0U;
    uint8_t bmcr_recovered = 0U;
    uint8_t loopback_before = 0U;
    uint8_t loopback_after = 0U;
    uint8_t loopback_after_bmcr = 0U;
    uint8_t loopback_after_physts = 0U;
    uint8_t recovery_reset_attempted = 0U;
    uint8_t recovery_reset_cleared = 0U;
    uint8_t an_complete = 0U;
    uint8_t physts_signal_detect = 0U;
    uint8_t physts_descr_lock = 0U;
    uint8_t physts_page_rx = 0U;
    uint8_t physts_link_seen = 0U;
    uint8_t physts_an_complete_seen = 0U;
    uint8_t physts_signal_detect_seen = 0U;
    uint8_t physts_descr_lock_seen = 0U;
    uint8_t physts_page_rx_seen = 0U;
    uint8_t force_link10_before = 0U;
    uint8_t force_link10_after = 0U;
    uint8_t lp_dis_before = 0U;
    uint8_t lp_dis_after = 0U;
    uint8_t forced_link_cfg_recovered = 0U;
    uint8_t loopback_state_mismatch = 0U;
    bool link_up = false;
    bool link_ambiguous = false;
    bool cable_hint = false;
    bool cable_hint_strong = false;
    bool comm_hint = false;
    const char *speed_cfg = "unknown";
    const char *duplex_cfg = "unknown";
    uint32_t start_ms;
    char measured[128];

    if (s_phy_found == 0U) {
        test_ethernet_phy(result);
        if (result->status == TEST_STATUS_FAIL) {
            return;
        }
    }

    if (!mdio_read_reg(s_phy_addr, ETH_PHY_REG_BMCR, &bmcr_before)) {
        board_test_set_result(result,
                              "eth.link",
                              "DP83640/RJ45",
                              TEST_STATUS_FAIL,
                              TEST_MODE_INTERACTIVE,
                              "bmcr read failed",
                              "Could not read BMCR before link check.",
                              "Check cable and PHY supply/clock.");
        return;
    }

    if (mdio_read_reg(s_phy_addr, ETH_PHY_REG_10BTSCR, &tenbtscr_before)) {
        tenbtscr_target = tenbtscr_before;
        force_link10_before = ((tenbtscr_before & ETH_10BTSCR_FORCE_LINK_10) != 0U) ? 1U : 0U;
        lp_dis_before = ((tenbtscr_before & ETH_10BTSCR_LP_DIS) != 0U) ? 1U : 0U;
        if ((active_recovery != 0U) &&
            ((tenbtscr_target & (ETH_10BTSCR_FORCE_LINK_10 | ETH_10BTSCR_LP_DIS)) != 0U)) {
            tenbtscr_target &= (uint16_t)~(ETH_10BTSCR_FORCE_LINK_10 | ETH_10BTSCR_LP_DIS);
            forced_link_cfg_recovered = 1U;
            (void)mdio_write_reg(s_phy_addr, ETH_PHY_REG_10BTSCR, tenbtscr_target);
            HAL_Delay(5U);
        }
    }

    bmcr_target = bmcr_before;
    loopback_before = ((bmcr_before & ETH_BMCR_LOOPBACK) != 0U) ? 1U : 0U;
    if (active_recovery != 0U) {
        if ((bmcr_target & (ETH_BMCR_LOOPBACK | ETH_BMCR_POWER_DOWN | ETH_BMCR_ISOLATE)) != 0U) {
            bmcr_target &= (uint16_t)~(ETH_BMCR_LOOPBACK | ETH_BMCR_POWER_DOWN | ETH_BMCR_ISOLATE);
            bmcr_recovered = 1U;
        }

        if ((bmcr_target & ETH_BMCR_AN_ENABLE) == 0U) {
            bmcr_target |= ETH_BMCR_AN_ENABLE;
            bmcr_recovered = 1U;
        }

        if (bmcr_recovered != 0U) {
            (void)mdio_write_reg(s_phy_addr, ETH_PHY_REG_BMCR, bmcr_target);
            HAL_Delay(20U);

            if (mdio_read_reg(s_phy_addr, ETH_PHY_REG_BMCR, &bmcr_after) &&
                ((bmcr_after & ETH_BMCR_LOOPBACK) != 0U)) {
                uint32_t t0 = 0U;
                recovery_reset_attempted = 1U;

                (void)mdio_write_reg(s_phy_addr, ETH_PHY_REG_BMCR, (uint16_t)(bmcr_after | ETH_BMCR_RESET));
                t0 = HAL_GetTick();
                do {
                    if (!mdio_read_reg(s_phy_addr, ETH_PHY_REG_BMCR, &bmcr_after)) {
                        break;
                    }
                    if ((bmcr_after & ETH_BMCR_RESET) == 0U) {
                        recovery_reset_cleared = 1U;
                        break;
                    }
                } while ((HAL_GetTick() - t0) < 250U);

                if (recovery_reset_cleared != 0U) {
                    bmcr_target = bmcr_after;
                    bmcr_target &= (uint16_t)~(ETH_BMCR_LOOPBACK | ETH_BMCR_POWER_DOWN | ETH_BMCR_ISOLATE);
                    bmcr_target |= ETH_BMCR_AN_ENABLE;
                    (void)mdio_write_reg(s_phy_addr, ETH_PHY_REG_BMCR, bmcr_target);
                    HAL_Delay(20U);
                }
            }
        }
    }

    if (((bmcr_target & ETH_BMCR_AN_ENABLE) != 0U) &&
#if ETH_PHY_RESTART_AN_IN_LINK_TEST
        (1)
#else
        (0)
#endif
    ) {
        (void)mdio_write_reg(s_phy_addr, ETH_PHY_REG_BMCR, (uint16_t)(bmcr_target | ETH_BMCR_RESTART_AN));
    }

    start_ms = HAL_GetTick();
    while ((HAL_GetTick() - start_ms) < ETH_LINK_WAIT_MS) {
        if (mdio_read_reg(s_phy_addr, ETH_PHY_REG_BMSR, &bmsr_latched) &&
            mdio_read_reg(s_phy_addr, ETH_PHY_REG_BMSR, &bmsr)) {
            if ((bmsr & ETH_BMSR_LINK_STATUS) != 0U) {
                link_up = true;
            }
            if ((bmsr & ETH_BMSR_AN_COMPLETE) != 0U) {
                an_complete = 1U;
            }
        }

        if (mdio_read_reg(s_phy_addr, ETH_PHY_REG_PHYSTS, &physts)) {
            physts_valid = 1U;
            if ((physts & ETH_PHYSTS_LINK_STATUS) != 0U) {
                link_up = true;
                physts_link_seen = 1U;
            }
            if ((physts & ETH_PHYSTS_AN_COMPLETE) != 0U) {
                physts_an_complete_seen = 1U;
            }
            if ((physts & ETH_PHYSTS_SIGNAL_DETECT) != 0U) {
                physts_signal_detect_seen = 1U;
            }
            if ((physts & ETH_PHYSTS_DESCRAMBLER_LOCK) != 0U) {
                physts_descr_lock_seen = 1U;
            }
            if ((physts & ETH_PHYSTS_PAGE_RECEIVED) != 0U) {
                physts_page_rx_seen = 1U;
            }
        }

        if ((HAL_GetTick() - start_ms) >= ETH_LINK_OBSERVE_MS) {
            break;
        }

        HAL_Delay(100U);
    }

    if (!mdio_read_reg(s_phy_addr, ETH_PHY_REG_BMCR, &bmcr_after)) {
        bmcr_after = 0U;
    }
    if (mdio_read_reg(s_phy_addr, ETH_PHY_REG_10BTSCR, &tenbtscr_after)) {
        force_link10_after = ((tenbtscr_after & ETH_10BTSCR_FORCE_LINK_10) != 0U) ? 1U : 0U;
        lp_dis_after = ((tenbtscr_after & ETH_10BTSCR_LP_DIS) != 0U) ? 1U : 0U;
    }

    loopback_after_bmcr = ((bmcr_after & ETH_BMCR_LOOPBACK) != 0U) ? 1U : 0U;
    if (physts_valid != 0U) {
        physts_signal_detect = ((physts & ETH_PHYSTS_SIGNAL_DETECT) != 0U) ? 1U : 0U;
        physts_descr_lock = ((physts & ETH_PHYSTS_DESCRAMBLER_LOCK) != 0U) ? 1U : 0U;
        physts_page_rx = ((physts & ETH_PHYSTS_PAGE_RECEIVED) != 0U) ? 1U : 0U;
        loopback_after_physts = ((physts & ETH_PHYSTS_LOOPBACK_STATUS) != 0U) ? 1U : 0U;
        loopback_after = loopback_after_physts;
        if (loopback_after_physts != loopback_after_bmcr) {
            loopback_state_mismatch = 1U;
        }
    } else {
        loopback_after = loopback_after_bmcr;
    }
    cable_hint = (physts_signal_detect_seen != 0U) ||
                 (physts_descr_lock_seen != 0U) ||
                 (physts_page_rx_seen != 0U) ||
                 (an_complete != 0U) ||
                 (physts_an_complete_seen != 0U);
    cable_hint_strong = (physts_signal_detect_seen != 0U) && (physts_descr_lock_seen != 0U);
    comm_hint = link_up &&
                (cable_hint_strong ||
                 (physts_link_seen != 0U &&
                  (an_complete != 0U || physts_an_complete_seen != 0U || physts_page_rx_seen != 0U)));
    link_ambiguous = (loopback_after != 0U) ||
                     (force_link10_after != 0U) ||
                     (loopback_state_mismatch != 0U);

    speed_cfg = ((bmcr_after & ETH_BMCR_SPEED_SELECT) != 0U) ? "100M" : "10M";
    duplex_cfg = ((bmcr_after & ETH_BMCR_DUPLEX_MODE) != 0U) ? "full" : "half";
    if (physts_valid != 0U) {
        speed_cfg = ((physts & ETH_PHYSTS_SPEED_10) != 0U) ? "10M" : "100M";
        duplex_cfg = ((physts & ETH_PHYSTS_DUPLEX) != 0U) ? "full" : "half";
    }

    (void)snprintf(measured,
                   sizeof(measured),
                   "phy=%u,link=%s,spd=%s,dup=%s,bmcr=%04X>%04X,bmsr=%04X,ph=%s%04X,an=%u,sd=%u,dsl=%u,pg=%u,lpb=%u/%u,mis=%u",
                   s_phy_addr,
                   link_up ? "up" : "down",
                   speed_cfg,
                   duplex_cfg,
                   bmcr_before,
                   bmcr_after,
                   bmsr,
                   (physts_valid != 0U) ? "" : "na:",
                   physts,
                   an_complete,
                   physts_signal_detect_seen,
                   physts_descr_lock_seen,
                   physts_page_rx_seen,
                   loopback_before,
                   loopback_after,
                   loopback_state_mismatch);

    /* Keep these values available for optional deeper diagnostics without warning noise in passive mode. */
    (void)recovery_reset_attempted;
    (void)physts_signal_detect;
    (void)physts_descr_lock;
    (void)physts_page_rx;
    (void)force_link10_before;
    (void)lp_dis_before;
    (void)lp_dis_after;
    (void)forced_link_cfg_recovered;

    if ((link_up) && !link_ambiguous) {
        board_test_set_result(result,
                              "eth.link",
                              "DP83640/RJ45",
                              TEST_STATUS_PASS,
                              TEST_MODE_INTERACTIVE,
                              measured,
                              "Ethernet link detected. RJ45 LEDs are PHY-driven and operator-observable only.",
                              "");
    } else if (link_ambiguous) {
        if (!cable_hint) {
            board_test_set_result(result,
                                  "eth.link",
                                  "DP83640/RJ45",
                                  TEST_STATUS_SKIP,
                                  TEST_MODE_INTERACTIVE,
                                  measured,
                                  "No external cable signal indicators detected while PHY forced/loopback state is active.",
                                  "Connect active cable/link partner, and verify BMCR[14]/10BTSCR[6] can be cleared.");
        } else if (!link_up) {
            board_test_set_result(result,
                                  "eth.link",
                                  "DP83640/RJ45",
                                  TEST_STATUS_FAIL,
                                  TEST_MODE_INTERACTIVE,
                                  measured,
                                  "External cable signal was detected, but PHY did not assert link.",
                                  "Check cable integrity, remote link partner, and PHY analog front-end.");
        } else if (comm_hint) {
            board_test_set_result(result,
                                  "eth.link",
                                  "DP83640/RJ45",
                                  TEST_STATUS_PASS,
                                  TEST_MODE_INTERACTIVE,
                                  measured,
                                  "Link is up with external cable activity evidence despite forced/loopback indicators.",
                                  "For unambiguous production link qualification, clear forced-link/loopback controls.");
        } else {
            board_test_set_result(result,
                                  "eth.link",
                                  "DP83640/RJ45",
                                  TEST_STATUS_WARN,
                                  TEST_MODE_INTERACTIVE,
                                  measured,
                                  "Cable activity is partially indicated, but communication state remains ambiguous due forced/loopback indicators.",
                                  "Disable PHY forced-link/loopback controls (or fix MDIO write path) before using eth.link as strict pass/fail.");
        }
    } else {
        if (cable_hint) {
            board_test_set_result(result,
                                  "eth.link",
                                  "DP83640/RJ45",
                                  TEST_STATUS_FAIL,
                                  TEST_MODE_INTERACTIVE,
                                  measured,
                                  "External cable signal is present, but PHY did not establish link.",
                                  "Check cable integrity, far-end port configuration, and PHY analog front-end.");
        } else {
            board_test_set_result(result,
                                  "eth.link",
                                  "DP83640/RJ45",
                                  TEST_STATUS_SKIP,
                                  TEST_MODE_INTERACTIVE,
                                  measured,
                                  "No link status asserted and no cable-signal indicators detected.",
                                  "Connect active cable/link partner and rerun 'run eth'.");
        }
    }
}

void test_ethernet_print_info(void)
{
    uint16_t bmcr = 0U;
    uint16_t bmsr = 0U;
    uint16_t bmsr_latched = 0U;
    uint16_t physts = 0U;
    uint16_t tenbtscr = 0U;
    uint8_t physts_valid = 0U;
    uint8_t tenbtscr_valid = 0U;

    if (s_phy_found == 0U) {
        if (!phy_scan(&s_phy_addr, &s_phy_id1, &s_phy_id2)) {
            test_uart_write_str("ETH info: PHY not yet detected. Run 'run eth'.\r\n");
            return;
        }
        s_phy_found = 1U;
    }

    test_uart_printf("ETH info: phy_addr=%u phy_id=0x%04X:0x%04X\r\n",
                     s_phy_addr,
                     s_phy_id1,
                     s_phy_id2);
    test_uart_write_str("RMII pins: PA1/PA2/PA7/PC1/PC4/PC5/PB11/PB12/PB13\r\n");
    test_uart_write_str("PHY: DP83640TVV/NOPB, Magjack: HR911130A\r\n");

    if (mdio_read_reg(s_phy_addr, ETH_PHY_REG_BMCR, &bmcr) &&
        mdio_read_reg(s_phy_addr, ETH_PHY_REG_BMSR, &bmsr_latched) &&
        mdio_read_reg(s_phy_addr, ETH_PHY_REG_BMSR, &bmsr)) {
        if (mdio_read_reg(s_phy_addr, ETH_PHY_REG_PHYSTS, &physts)) {
            physts_valid = 1U;
        }
        if (mdio_read_reg(s_phy_addr, ETH_PHY_REG_10BTSCR, &tenbtscr)) {
            tenbtscr_valid = 1U;
        }

        test_uart_printf("ETH regs: bmcr=0x%04X bmsr=0x%04X bmsr_latched=0x%04X physts=%s0x%04X 10btscr=%s0x%04X\r\n",
                         bmcr,
                         bmsr,
                         bmsr_latched,
                         (physts_valid != 0U) ? "" : "n/a:",
                         physts,
                         (tenbtscr_valid != 0U) ? "" : "n/a:",
                         tenbtscr);
    } else {
        test_uart_write_str("ETH regs: read failed (check MDIO/MDC).\r\n");
    }
}
