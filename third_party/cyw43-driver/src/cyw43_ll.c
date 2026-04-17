/*
 * This file is part of the cyw43-driver
 *
 * Copyright (C) 2019-2022 George Robotics Pty Ltd
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Any redistribution, use, or modification in source or binary form is done
 *    solely for personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY THE LICENSOR AND COPYRIGHT OWNER "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE LICENSOR OR COPYRIGHT OWNER BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This software is also available for use with certain devices under different
 * terms, as set out in the top level LICENSE file.  For commercial licensing
 * options please email contact@georgerobotics.com.au.
 */

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <inttypes.h>

#include "ap6256_assets.h"
#include "ap6256_cyw43_compat.h"
#include "ap6256_cyw43_port.h"
#include "cyw43.h"
#include "cyw43_config.h"
#include "cyw43_country.h"
#include "cyw43_ll.h"
#include "cyw43_internal.h"
#include "cyw43_stats.h"

#if CYW43_USE_SPI
#include "cyw43_spi.h"
#include "cyw43_debug_pins.h"
#else
#include "cyw43_sdio.h"
#endif

struct pbuf;
uint16_t pbuf_copy_partial(const struct pbuf *p, void *dataptr, uint16_t len, uint16_t offset);

#ifndef NDEBUG
extern bool enable_spi_packet_dumping;
#endif

#define USE_KSO (1)

/*
 * BCM43456 / BCM4345/9-class parts expose 819200 bytes of dongle RAM
 * (0xC8000). The default CYW43 value of 512 KiB is for 43439-era silicon and
 * places the NVRAM footer at the wrong address for AP6256.
 */
#define CYW43_RAM_SIZE (AP6256_CYW43_RAM_SIZE_BYTES)
#define AP6256_NVRAM_BLOB_MAX_LEN (4096)
#define AP6256_CYW43_CONTROL_CREDIT_RECOVERY_US (250000U)
#define AP6256_CYW43_ASSOC_CREDIT_RECOVERY_US   (500000U)
#define AP6256_CYW43_DATA_CREDIT_RECOVERY_US    (250000U)
#ifndef AP6256_CYW43_ENABLE_SYNTHETIC_CONTROL_CREDIT
#define AP6256_CYW43_ENABLE_SYNTHETIC_CONTROL_CREDIT (0)
#endif
#ifndef AP6256_CYW43_SAFE_5G_FEM_NVRAM
#define AP6256_CYW43_SAFE_5G_FEM_NVRAM (0)
#endif

#define ALIGN_UINT(val, align) (((val) + (align) - 1) & ~((align) - 1))

// Configure the padding needed for data sent to cyw43_write_bytes().
// cyw43_read_bytes() also needs padding, but that's handled separately.
#if CYW43_USE_SPI
#define CYW43_WRITE_BYTES_PAD(len) ALIGN_UINT((len), 4)
#define CYW43_SDPCM_WRITE_BYTES_PAD(len) ALIGN_UINT((len), 4)
#else
#define CYW43_WRITE_BYTES_PAD(len) ALIGN_UINT((len), 64)
#define CYW43_SDPCM_WRITE_BYTES_PAD(len) ALIGN_UINT((len), ap6256_cyw43_port_runtime_f2_block_size())
#endif

// Configure the active level of the host interrupt pin.
#if CYW43_USE_SPI
#if defined(CYW43_PIN_WL_HOST_WAKE)
static const int host_interrupt_pin_active = 1;
#elif defined(CYW43_PIN_WL_IRQ)
static const int host_interrupt_pin_active = 0;
#endif
#else
static const int host_interrupt_pin_active = 0;
#endif

#if CYW43_USE_STATS
// Storage for some debug stats
uint32_t cyw43_stats[CYW43_STAT_LAST];
#endif

static inline uint32_t cyw43_get_le32(const uint8_t *buf) {
    return buf[0] | buf[1] << 8 | buf[2] << 16 | buf[3] << 24;
}

static inline void cyw43_put_le16(uint8_t *buf, uint32_t x) {
    buf[0] = x;
    buf[1] = x >> 8;
}

static inline void cyw43_put_le32(uint8_t *buf, uint32_t x) {
    buf[0] = x;
    buf[1] = x >> 8;
    buf[2] = x >> 16;
    buf[3] = x >> 24;
}

static void ap6256_checkpoint_record(uint32_t checkpoint,
                                     uint8_t function,
                                     uint32_t address,
                                     uint32_t write_value,
                                     uint32_t readback_value,
                                     int32_t status);

#if CYW43_RESOURCE_VERIFY_DOWNLOAD
static void cyw43_xxd(size_t len, const uint8_t *buf) {
    for (size_t i = 0; i < len; ++i) {
        CYW43_PRINTF(" %02x", buf[i]);
        if (i % 32 == 31) {
            CYW43_PRINTF("\n");
        }
    }
    CYW43_PRINTF("\n");
}
#endif

/*******************************************************************************/
// CYW43 constants and types

#define SDPCM_HEADER_LEN (sizeof(struct sdpcm_header_t))
#define IOCTL_HEADER_LEN (sizeof(struct ioctl_header_t))
#define BDC_HEADER_LEN (sizeof(struct sdpcm_bdc_header_t))

#define SDIO_FUNCTION2_WATERMARK    (0x10008)
#define SDIO_BACKPLANE_ADDRESS_LOW  (0x1000a)
#define SDIO_BACKPLANE_ADDRESS_MID  (0x1000b)
#define SDIO_BACKPLANE_ADDRESS_HIGH (0x1000c)
#define SDIO_CHIP_CLOCK_CSR         (0x1000e)
#define SDIO_WAKEUP_CTRL            (0x1001e)
#define SDIO_SLEEP_CSR              (0x1001f)

#define I_HMB_SW_MASK            (0x000000f0)
#define I_HMB_FC_CHANGE               (1 << 5)

#define CHIPCOMMON_BASE_ADDRESS  (0x18000000)
#define SDIO_BASE_ADDRESS        (0x18002000)
#define WLAN_ARMCM3_BASE_ADDRESS (0x18003000)
#define SOCSRAM_BASE_ADDRESS     (0x18004000)
#define BACKPLANE_ADDR_MASK      (0x7fff)
#define WRAPPER_REGISTER_OFFSET  (0x100000)

#define SBSDIO_SB_ACCESS_2_4B_FLAG  0x08000

#define CHIPCOMMON_SR_CONTROL1   (CHIPCOMMON_BASE_ADDRESS + 0x508)
#define CHIPCOMMON_PMU_CONTROL   (CHIPCOMMON_BASE_ADDRESS + 0x600)
#define CHIPCOMMON_EROMPTR       (CHIPCOMMON_BASE_ADDRESS + 0x0FC)
#define SDIO_INT_STATUS          (SDIO_BASE_ADDRESS + 0x20)
#define SDIO_INT_HOST_MASK       (SDIO_BASE_ADDRESS + 0x24)
#define SDIO_FUNCTION_INT_MASK   (SDIO_BASE_ADDRESS + 0x34)
#define SDIO_TO_SB_MAILBOX       (SDIO_BASE_ADDRESS + 0x40)
#define SOCSRAM_BANKX_INDEX      (SOCSRAM_BASE_ADDRESS + 0x10)
#define SOCSRAM_BANKX_PDA        (SOCSRAM_BASE_ADDRESS + 0x44)

#define SBSDIO_ALP_AVAIL_REQ     (0x08)
#define SBSDIO_HT_AVAIL_REQ      (0x10)
#define SBSDIO_ALP_AVAIL         (0x40)
#define SBSDIO_HT_AVAIL          (0x80)

#define CHIPCOMMON_PMU_CTL_RES_SHIFT   (13U)
#define CHIPCOMMON_PMU_CTL_RES_RELOAD  (0x2U)

#define AI_IOCTRL_OFFSET         (0x408)
#define SICF_CPUHALT             (0x0020)
#define SICF_FGC                 (0x0002)
#define SICF_CLOCK_EN            (0x0001)
#define AI_RESETCTRL_OFFSET      (0x800)
#define AIRC_RESET               (1)

#define AP6256_BCMA_CORE_CHIPCOMMON   AP6256_CYW43_CORE_CHIPCOMMON
#define AP6256_BCMA_CORE_INTERNAL_MEM AP6256_CYW43_CORE_INTERNAL_MEM
#define AP6256_BCMA_CORE_80211        AP6256_CYW43_CORE_80211
#define AP6256_BCMA_CORE_SDIO_DEV     AP6256_CYW43_CORE_SDIO_DEV
#define AP6256_BCMA_CORE_ARM_CM3      AP6256_CYW43_CORE_ARM_CM3
#define AP6256_BCMA_CORE_ARM_CR4      AP6256_CYW43_CORE_ARM_CR4
#define AP6256_BCMA_CORE_SYS_MEM      AP6256_CYW43_CORE_SYS_MEM

#define AP6256_DMP_DESC_TYPE_MSK      0x0000000FU
#define AP6256_DMP_DESC_EMPTY         0x00000000U
#define AP6256_DMP_DESC_VALID         0x00000001U
#define AP6256_DMP_DESC_COMPONENT     0x00000001U
#define AP6256_DMP_DESC_MASTER_PORT   0x00000003U
#define AP6256_DMP_DESC_ADDRESS       0x00000005U
#define AP6256_DMP_DESC_ADDRSIZE_GT32 0x00000008U
#define AP6256_DMP_DESC_EOT           0x0000000FU
#define AP6256_DMP_COMP_PARTNUM       0x000FFF00U
#define AP6256_DMP_COMP_PARTNUM_S     8U
#define AP6256_DMP_COMP_REVISION      0xFF000000U
#define AP6256_DMP_COMP_REVISION_S    24U
#define AP6256_DMP_COMP_NUM_SWRAP     0x00F80000U
#define AP6256_DMP_COMP_NUM_SWRAP_S   19U
#define AP6256_DMP_COMP_NUM_MWRAP     0x0007C000U
#define AP6256_DMP_COMP_NUM_MWRAP_S   14U
#define AP6256_DMP_SLAVE_ADDR_BASE    0xFFFFF000U
#define AP6256_DMP_SLAVE_TYPE         0x000000C0U
#define AP6256_DMP_SLAVE_TYPE_S       6U
#define AP6256_DMP_SLAVE_TYPE_SLAVE   0U
#define AP6256_DMP_SLAVE_TYPE_SWRAP   2U
#define AP6256_DMP_SLAVE_TYPE_MWRAP   3U
#define AP6256_DMP_SLAVE_SIZE_TYPE    0x00000030U
#define AP6256_DMP_SLAVE_SIZE_TYPE_S  4U
#define AP6256_DMP_SLAVE_SIZE_4K      0U
#define AP6256_DMP_SLAVE_SIZE_8K      1U
#define AP6256_DMP_SLAVE_SIZE_DESC    3U

#define AP6256_AI_MAX_CORES           16U
#define AP6256_RESET_VECTOR_ADDRESS   0x00000000UL

/*
 * BCM43456 reports D11ac-style chanspecs (for example 2.4 GHz channel 6 as
 * 0x1006). Use the same compact 20 MHz chanspec encoding for explicit scan
 * and directed-join channel lists.
 */
#define AP6256_CH_MAX_2G_CHANNEL          14U
#define AP6256_CHSPEC_D11AC_BW_20         0x1000U
#define AP6256_CHSPEC_D11AC_BND_2G        0x0000U
#define AP6256_CHSPEC_D11AC_BND_5G        0xC000U
#define AP6256_CHSPEC_20MHZ(channel) \
    ((uint16_t)((channel) | AP6256_CHSPEC_D11AC_BW_20 | \
                (((channel) <= AP6256_CH_MAX_2G_CHANNEL) ? \
                    AP6256_CHSPEC_D11AC_BND_2G : AP6256_CHSPEC_D11AC_BND_5G)))

#define AP6256_BRCMF_SSID_LEN                  36U
#define AP6256_BRCMF_JOIN_SCAN_LEN             20U
#define AP6256_BRCMF_EXT_JOIN_ASSOC_OFFSET     (AP6256_BRCMF_SSID_LEN + AP6256_BRCMF_JOIN_SCAN_LEN)
#define AP6256_BRCMF_ASSOC_FIXED_LEN           12U
#define AP6256_BRCMF_EXT_JOIN_ONE_CHAN_LEN     (AP6256_BRCMF_EXT_JOIN_ASSOC_OFFSET + AP6256_BRCMF_ASSOC_FIXED_LEN + 2U)
#define AP6256_BRCMF_SCAN_JOIN_ACTIVE_MS       320U
#define AP6256_BRCMF_SCAN_JOIN_PASSIVE_MS      400U
#define AP6256_BRCMF_SCAN_JOIN_PROBE_MS        20U

static uint16_t ap6256_join_chanspec(uint32_t channel)
{
    if ((channel != CYW43_CHANNEL_NONE) &&
        ((channel & CYW43_CHANNEL_CHANSPEC_FLAG) != 0U)) {
        return (uint16_t)(channel & 0xffffU);
    }
    return AP6256_CHSPEC_20MHZ(channel);
}

static uint32_t ap6256_primary_channel_from_join_channel(uint32_t channel)
{
    if ((channel & CYW43_CHANNEL_CHANSPEC_FLAG) == 0U) {
        return channel;
    }

    uint16_t chanspec = (uint16_t)(channel & 0xffffU);
    uint32_t center_or_primary = (uint32_t)(chanspec & 0x00ffU);
    uint32_t ctl_sb = (uint32_t)((chanspec >> 8U) & 0x07U);
    uint32_t bw = (uint32_t)(chanspec & 0x3800U);

    if ((bw == 0x2000U) && (center_or_primary > 6U)) {
        /*
         * 80 MHz D11ac chanspecs use the low byte as center channel and bits
         * 8..10 as control sideband. Convert to the primary channel used by
         * WLC_SET_CHANNEL before connect: 0xE09B -> 149, 0xE19B -> 153,
         * 0xE23A -> 60.
         */
        return center_or_primary - 6U + (ctl_sb * 4U);
    }

    return center_or_primary;
}

static uint8_t ap6256_join_channel_is_5g(uint32_t channel)
{
    if ((channel == CYW43_CHANNEL_NONE) || (channel == 0U)) {
        return 0U;
    }
    if ((channel & CYW43_CHANNEL_CHANSPEC_FLAG) != 0U) {
        return (((uint16_t)(channel & 0xffffU) & AP6256_CHSPEC_D11AC_BND_5G) != 0U) ? 1U : 0U;
    }
    return (channel > AP6256_CH_MAX_2G_CHANNEL) ? 1U : 0U;
}

static size_t __attribute__((unused)) ap6256_build_join_pref_5g(uint8_t *buf, size_t buf_len)
{
    const uint8_t BRCMF_JOIN_PREF_RSSI = 1U;
    const uint8_t BRCMF_JOIN_PREF_BAND = 3U;
    const uint8_t WLC_BAND_5G = 1U;

    if ((buf == NULL) || (buf_len < 8U)) {
        return 0U;
    }

    /*
     * brcmfmac sets join_pref before association when cfg80211 asks for band
     * preference. Keep this as a hidden diagnostic path only; normal AP6256
     * joins use directed brcmf_join_params and prove success by control
     * completion or association/link events.
     */
    memset(buf, 0, 8U);
    buf[0] = BRCMF_JOIN_PREF_BAND;
    buf[1] = 2U;
    buf[2] = 0U;
    buf[3] = WLC_BAND_5G;
    buf[4] = BRCMF_JOIN_PREF_RSSI;
    buf[5] = 2U;
    buf[6] = 0U;
    buf[7] = 0U;
    return 8U;
}

static size_t ap6256_build_brcmf_join_params(uint8_t *buf,
                                             size_t buf_len,
                                             size_t ssid_len,
                                             const uint8_t *ssid,
                                             const uint8_t *bssid,
                                             uint32_t channel)
{
    const size_t ssid_struct_len = 4U + 32U;
    const size_t assoc_fixed_len = 12U;
    const size_t assoc_one_chanspec_len = 16U;
    size_t payload_len = ssid_struct_len;

    if ((buf == NULL) || (ssid == NULL) || (ssid_len > 32U) ||
        (buf_len < (ssid_struct_len + assoc_one_chanspec_len))) {
        return 0U;
    }

    memset(buf, 0, ssid_struct_len + assoc_one_chanspec_len);
    cyw43_put_le32(buf, ssid_len);
    memcpy(buf + 4U, ssid, ssid_len);

    if (bssid != NULL) {
        memcpy(buf + ssid_struct_len, bssid, 6U);
        payload_len += assoc_fixed_len;

        if (channel != CYW43_CHANNEL_NONE) {
            cyw43_put_le32(buf + ssid_struct_len + 8U, 1U);
            cyw43_put_le16(buf + ssid_struct_len + 12U, ap6256_join_chanspec(channel));
            payload_len = ssid_struct_len + assoc_one_chanspec_len;
        }
    }

    return payload_len;
}

static size_t __attribute__((unused)) ap6256_build_brcmf_ext_join_params(uint8_t *buf,
                                                                         size_t buf_len,
                                                                         size_t ssid_len,
                                                                         const uint8_t *ssid,
                                                                         const uint8_t *bssid,
                                                                         uint32_t channel)
{
    const size_t assoc_offset = AP6256_BRCMF_EXT_JOIN_ASSOC_OFFSET;
    const uint32_t default_scan = 0xFFFFFFFFUL;
    uint32_t nprobes = default_scan;
    uint32_t active_time = default_scan;
    uint32_t passive_time = default_scan;
    uint32_t home_time = default_scan;
    size_t payload_len = assoc_offset + AP6256_BRCMF_ASSOC_FIXED_LEN;

    if ((buf == NULL) || (ssid == NULL) || (ssid_len > 32U) ||
        (buf_len < AP6256_BRCMF_EXT_JOIN_ONE_CHAN_LEN)) {
        return 0U;
    }

    memset(buf, 0, AP6256_BRCMF_EXT_JOIN_ONE_CHAN_LEN);
    cyw43_put_le32(buf, ssid_len);
    memcpy(buf + 4U, ssid, ssid_len);

    /*
     * brcmfmac uses the "join" iovar first and includes bounded scan dwell
     * parameters when it has an exact channel/chanspec. This matters after our
     * scan/control path has been recovered or reopened: the dongle can no
     * longer rely on a previous escan cache to find the target BSS.
     */
    cyw43_put_le32(buf + AP6256_BRCMF_SSID_LEN, 0xFFFFFFFFUL); /* scan_type = -1/default */
    if (channel != CYW43_CHANNEL_NONE) {
        nprobes = AP6256_BRCMF_SCAN_JOIN_ACTIVE_MS / AP6256_BRCMF_SCAN_JOIN_PROBE_MS;
        active_time = AP6256_BRCMF_SCAN_JOIN_ACTIVE_MS;
        passive_time = AP6256_BRCMF_SCAN_JOIN_PASSIVE_MS;
    }
    cyw43_put_le32(buf + AP6256_BRCMF_SSID_LEN + 4U, nprobes);
    cyw43_put_le32(buf + AP6256_BRCMF_SSID_LEN + 8U, active_time);
    cyw43_put_le32(buf + AP6256_BRCMF_SSID_LEN + 12U, passive_time);
    cyw43_put_le32(buf + AP6256_BRCMF_SSID_LEN + 16U, home_time);

    if (bssid != NULL) {
        memcpy(buf + assoc_offset, bssid, 6U);
    } else {
        memset(buf + assoc_offset, 0xFF, 6U);
    }

    if (channel != CYW43_CHANNEL_NONE) {
        cyw43_put_le32(buf + assoc_offset + 8U, 1U);
        cyw43_put_le16(buf + assoc_offset + 12U, ap6256_join_chanspec(channel));
        payload_len = AP6256_BRCMF_EXT_JOIN_ONE_CHAN_LEN;
    }

    return payload_len;
}

#ifndef AP6256_CYW43_FORCE_SDIO_POLL
#define AP6256_CYW43_FORCE_SDIO_POLL 0
#endif

#define SPI_F2_WATERMARK (32)
#define SDIO_F2_WATERMARK (8)

#define WWD_STA_INTERFACE (0)
#define WWD_AP_INTERFACE (1)
#define WWD_P2P_INTERFACE (2)

// for cyw43_sdpcm_send_common
#define CONTROL_HEADER (0)
#define ASYNCEVENT_HEADER (1)
#define DATA_HEADER (2)

#define CDCF_IOC_ID_SHIFT (16)
#define CDCF_IOC_ID_MASK (0xffff0000)
#define CDCF_IOC_IF_SHIFT (12)
#define CDCF_IOC_IF_MASK (0x0000f000)

#define SDPCM_GET (0)
#define SDPCM_SET (2)

#define WLC_UP (2)
#define WLC_SET_INFRA (20)
#define WLC_SET_AUTH (22)
#define WLC_GET_BSSID (23)
#define WLC_GET_SSID (25)
#define WLC_SET_SSID (26)
#define WLC_SET_CHANNEL (30)
#define WLC_SCAN (50)
#define WLC_DISASSOC (52)
#define WLC_GET_ANTDIV (63)
#define WLC_SET_ANTDIV (64)
#define WLC_SET_DTIMPRD (78)
#define WLC_GET_PM (85)
#define WLC_SET_PM (86)
#define WLC_SET_GMODE (110)
#define WLC_SET_WSEC (134)
#define WLC_SET_BAND (142)
#define WLC_GET_ASSOCLIST (159)
#define WLC_SET_WPA_AUTH (165)
#define WLC_SET_VAR (263)
#define WLC_GET_VAR (262)
#define WLC_SET_WSEC_PMK (268)

// SDIO bus specifics
#define SDIOD_CCCR_IOEN (0x02)
#define SDIOD_CCCR_IORDY (0x03)
#define SDIOD_CCCR_INTEN (0x04)
#define SDIOD_CCCR_INTPEND (0x05)
#define SDIOD_CCCR_BICTRL (0x07)
#define SDIOD_CCCR_BLKSIZE_0 (0x10)
#define SDIOD_CCCR_SPEED_CONTROL (0x13)
#define SDIOD_CCCR_BRCM_CARDCAP (0xf0)
#define SDIOD_CCCR_BRCM_CARDCTRL (0xf1)
#define SDIOD_SEP_INT_CTL (0xf2)
#define SDIOD_CCCR_F1BLKSIZE_0 (0x110)
#define SDIOD_CCCR_F1BLKSIZE_1 (0x111)
#define SDIOD_CCCR_F2BLKSIZE_0 (0x210)
#define SDIOD_CCCR_F2BLKSIZE_1 (0x211)
#define INTR_CTL_MASTER_EN (0x01)
#define INTR_CTL_FUNC1_EN (0x02)
#define INTR_CTL_FUNC2_EN (0x04)
#define SDIO_FUNC_ENABLE_1 (0x02)
#define SDIO_FUNC_ENABLE_2 (0x04)
#define SDIO_FUNC_READY_1 (0x02)
#define SDIO_FUNC_READY_2 (0x04)
#define SDIO_64B_BLOCK (64)
#define SDIOD_CCCR_BRCM_CARDCTRL_WLANRESET (0x02)
#define SDIO_CHIP_CLOCK_CSR (0x1000e)
#define SDIO_PULL_UP (0x1000f)

// SDIO_CHIP_CLOCK_CSR bits
#define SBSDIO_ALP_AVAIL (0x40)
#define SBSDIO_FORCE_HW_CLKREQ_OFF (0x20)
#define SBSDIO_ALP_AVAIL_REQ (0x08)
#define SBSDIO_FORCE_ALP (0x01)
#define SBSDIO_FORCE_HT            ((uint32_t)0x02)

// SDIOD_CCCR_BRCM_CARDCAP bits
#define SDIOD_CCCR_BRCM_CARDCAP_CMD14_SUPPORT ((uint32_t)0x02) // Supports CMD14
#define SDIOD_CCCR_BRCM_CARDCAP_CMD14_EXT   ((uint32_t)0x04) // CMD14 is allowed in FSM command state
#define SDIOD_CCCR_BRCM_CARDCAP_CMD_NODEC   ((uint32_t)0x08) // sdiod_aos does not decode any command

// SDIOD_SEP_INT_CTL bits
#define SEP_INTR_CTL_MASK                   ((uint32_t)0x01) // out-of-band interrupt mask
#define SEP_INTR_CTL_EN                     ((uint32_t)0x02) // out-of-band interrupt output enable
#define SEP_INTR_CTL_POL                    ((uint32_t)0x04) // out-of-band interrupt polarity

// SDIO_WAKEUP_CTRL bits
#define SBSDIO_WCTRL_WAKE_TILL_ALP_AVAIL    ((uint32_t)(1 << 0)) // WakeTillAlpAvail bit
#define SBSDIO_WCTRL_WAKE_TILL_HT_AVAIL     ((uint32_t)(1 << 1)) // WakeTillHTAvail bit

// SDIO_SLEEP_CSR bits
#define SBSDIO_SLPCSR_KEEP_SDIO_ON          ((uint32_t)(1 << 0)) // KeepSdioOn bit
#define SBSDIO_SLPCSR_DEVICE_ON             ((uint32_t)(1 << 1)) // DeviceOn bit

// For determining security type from a scan
#define DOT11_CAP_PRIVACY             (0x0010)
#define DOT11_IE_ID_RSN               (48)
#define DOT11_IE_ID_VENDOR_SPECIFIC   (221)
#define WPA_OUI_TYPE1                 "\x00\x50\xF2\x01"
#define RSN_CAP_MFPR_MASK             (1u << 6)
#define RSN_CAP_MFPC_MASK             (1u << 7)

#define SLEEP_MAX (50)

// Multicast registered group addresses
#define MAX_MULTICAST_REGISTERED_ADDRESS (10)

#define CYW_INT_FROM_LL(ll) ((cyw43_int_t *)(ll))
#define CYW_INT_TO_LL(in) ((cyw43_ll_t *)(in))

#define CYW_EAPOL_KEY_TIMEOUT (5000)
#define AP6256_CYW43_IOCTL_NO_PACKET_RECOVERY_LOOPS (64U)
#define AP6256_CYW43_IOCTL_NO_PACKET_BUDGET_LOOPS   (1200U)
#define AP6256_CYW43_ESCAN_NO_PACKET_BUDGET_LOOPS   (80U)
#define AP6256_CYW43_ASSOC_NO_PACKET_BUDGET_LOOPS   (3000U)
#define AP6256_CYW43_JOIN_SETUP_NO_PACKET_BUDGET_LOOPS (40U)
#define AP6256_CYW43_ASSOC_RESPONSE_WINDOW_US       (3000000U)
#define AP6256_CYW43_JOIN_SETUP_RESPONSE_WINDOW_US  (250000U)
#define AP6256_CYW43_WL_TXPWR_OVERRIDE              (0x80000000U)
/*
 * Keep the brcmfmac-style directed association payload available in normal
 * runtime. A previous RTT automation issue made several runs look like command
 * failures when the console input was actually corrupted; the production path
 * should be evidence-based rather than SSID-only shortcuts.
 */
#ifndef AP6256_CYW43_DIRECTED_ASSOC_PAYLOAD
#define AP6256_CYW43_DIRECTED_ASSOC_PAYLOAD          (1U)
#endif

#ifndef AP6256_CYW43_ENABLE_JOIN_PREF
#define AP6256_CYW43_ENABLE_JOIN_PREF                (1U)
#endif

#ifndef AP6256_CYW43_USE_EXT_JOIN_IOVAR
#define AP6256_CYW43_USE_EXT_JOIN_IOVAR              (0U)
#endif

#ifndef AP6256_CYW43_5G_JOIN_IOVAR_ONLY
#define AP6256_CYW43_5G_JOIN_IOVAR_ONLY              (0U)
#endif

#ifndef AP6256_CYW43_5G_BROADCAST_EXT_JOIN
#define AP6256_CYW43_5G_BROADCAST_EXT_JOIN           (0U)
#endif

#ifndef AP6256_CYW43_5G_JOIN_QTXPOWER_QDBM
#define AP6256_CYW43_5G_JOIN_QTXPOWER_QDBM           (1U)
#endif

#ifndef AP6256_CYW43_5G_JOIN_SAFE_VHT_OFF
#define AP6256_CYW43_5G_JOIN_SAFE_VHT_OFF            (1U)
#endif

#ifndef AP6256_CYW43_5G_JOIN_SET_CHANNEL_STARTER
#define AP6256_CYW43_5G_JOIN_SET_CHANNEL_STARTER     (0U)
#endif

#ifndef AP6256_CYW43_ABORT_SCAN_BEFORE_JOIN
#define AP6256_CYW43_ABORT_SCAN_BEFORE_JOIN          (1U)
#endif

// Errors generated by sdpcm_process_rx_packet.
#define CYW43_ERROR_WRONG_PAYLOAD_TYPE (-9)

#define AUTH_TYPE_OPEN 0
#define AUTH_TYPE_SAE 3

// management frame protection
#define MFP_NONE 0
#define MFP_CAPABLE 1
#define MFP_REQUIRED 2

// Values used for STA and AP auth settings
#define CYW43_WPA_AUTH_PSK (0x0004)
#define CYW43_WPA2_AUTH_PSK (0x0080)
#define CYW43_WPA3_AUTH_SAE_PSK (0x40000)
#define AP6256_WIFI_JOIN_STATE_KIND_MASK (0x000fU)
#define AP6256_WIFI_JOIN_STATE_ACTIVE    (0x0001U)
#define AP6256_WIFI_JOIN_STATE_FAIL      (0x0002U)
#define AP6256_WIFI_JOIN_STATE_NONET     (0x0003U)
#define AP6256_WIFI_JOIN_STATE_BADAUTH   (0x0004U)
#define AP6256_WIFI_JOIN_STATE_AUTH      (0x0200U)
#define AP6256_WIFI_JOIN_STATE_LINK      (0x0400U)
#define AP6256_WIFI_JOIN_STATE_KEYED     (0x0800U)

// Max password length
#define CYW43_WPA_MAX_PASSWORD_LEN 64
#define CYW43_WPA_SAE_MAX_PASSWORD_LEN 128

static int cyw43_ll_sdpcm_poll_device(cyw43_int_t *self, size_t *len, uint8_t **buf);
static int cyw43_write_iovar_n(cyw43_int_t *self, const char *var, size_t len, const void *buf, uint32_t iface);
static int ap6256_cyw43_scan_wake(cyw43_int_t *self, cyw43_ll_t *self_in);

static uint32_t s_ap6256_pending_ioctl_cmd;
static uint32_t s_ap6256_pending_ioctl_iface;
static uint32_t s_ap6256_pending_ioctl_kind;
static bool s_ap6256_pending_ioctl_association_trigger;
static uint16_t s_ap6256_scan_sync_id;
static uint8_t s_ap6256_scan_active;

static bool ap6256_join_no_response_ok(int ret) {
    return (ret != 0) &&
           (ap6256_cyw43_port_last_ioctl_phase() == AP6256_CYW43_IOCTL_PHASE_WAIT_NO_PACKET);
}

static int ap6256_program_wpa2_psk_assoc_ie(cyw43_int_t *self, uint32_t wpa_auth, uint32_t mfp) {
    /*
     * brcmfmac programs the RSN/WPA IE before association so firmware knows the
     * exact AKM/cipher set requested by cfg80211. This matters on WPA2/WPA3
     * transition APs: advertise a WPA2-PSK/CCMP association IE and mirror MFPC
     * when selected, but never MFPR/SAE unless the MCU path has explicit SAE
     * support. That keeps the association strictly WPA2-PSK while matching the
     * BSS's management-frame-protection capability.
     */
    static const uint8_t rsn_wpa2_psk_ccmp_template[] = {
        0x30, 0x14,             /* RSN IE, length 20 */
        0x01, 0x00,             /* version */
        0x00, 0x0f, 0xac, 0x04, /* group cipher: CCMP */
        0x01, 0x00,             /* pairwise cipher count */
        0x00, 0x0f, 0xac, 0x04, /* pairwise cipher: CCMP */
        0x01, 0x00,             /* AKM count */
        0x00, 0x0f, 0xac, 0x02, /* AKM: PSK */
        0x00, 0x00              /* RSN caps: no PMF */
    };
    uint8_t rsn_wpa2_psk_ccmp[sizeof(rsn_wpa2_psk_ccmp_template)];
    int ret;

    if (((wpa_auth & CYW43_WPA2_AUTH_PSK) == 0U) ||
        ((wpa_auth & CYW43_WPA3_AUTH_SAE_PSK) != 0U)) {
        return 0;
    }
    memcpy(rsn_wpa2_psk_ccmp, rsn_wpa2_psk_ccmp_template, sizeof(rsn_wpa2_psk_ccmp));
    if (mfp == MFP_CAPABLE) {
        /*
         * RSN Capabilities bit 7 is MFPC. Do not set MFPR; this keeps the
         * association WPA2-PSK compatible while matching transition-mode BSSs
         * that advertise management-frame protection capability.
         */
        rsn_wpa2_psk_ccmp[20] = 0x80U;
    }

    ret = cyw43_write_iovar_n(self,
                              "wpaie",
                              sizeof(rsn_wpa2_psk_ccmp),
                              rsn_wpa2_psk_ccmp,
                              WWD_STA_INTERFACE);
    if ((ret != 0) && ap6256_join_no_response_ok(ret)) {
        ret = 0;
    }
    return ret;
}

static bool ap6256_ioctl_is_association_trigger(uint32_t kind, uint32_t cmd) {
    return (kind == SDPCM_SET) && ((cmd == WLC_SET_SSID) || (cmd == WLC_DISASSOC));
}

static bool ap6256_ioctl_is_join_iovar(uint32_t kind, uint32_t cmd, size_t len, const uint8_t *buf) {
    if ((kind != SDPCM_SET) || (cmd != WLC_SET_VAR) || (buf == NULL) || (len < 5U)) {
        return false;
    }
    return memcmp(buf, "join", 5U) == 0;
}

static bool ap6256_iovar_name_is(const uint8_t *buf, size_t len, const char *name) {
    size_t name_len = strlen(name) + 1U;

    return (buf != NULL) && (len >= name_len) && (memcmp(buf, name, name_len) == 0);
}

static bool ap6256_ioctl_is_join_setup(uint32_t kind, uint32_t cmd, size_t len, const uint8_t *buf) {
    if (kind != SDPCM_SET) {
        return false;
    }

    switch (cmd) {
    case WLC_SET_WSEC:
    case WLC_SET_WSEC_PMK:
    case WLC_SET_INFRA:
    case WLC_SET_AUTH:
    case WLC_SET_WPA_AUTH:
    case WLC_SET_CHANNEL:
        return true;
    default:
        break;
    }

    if ((cmd != WLC_SET_VAR) || (buf == NULL)) {
        return false;
    }

    return ap6256_iovar_name_is(buf, len, "wpaie") ||
           ap6256_iovar_name_is(buf, len, "sup_wpa") ||
           ap6256_iovar_name_is(buf, len, "sup_wpa2_eapver") ||
           ap6256_iovar_name_is(buf, len, "sup_wpa_tmo") ||
           ap6256_iovar_name_is(buf, len, "sae_password") ||
           ap6256_iovar_name_is(buf, len, "auth") ||
           ap6256_iovar_name_is(buf, len, "mfp") ||
           ap6256_iovar_name_is(buf, len, "wpa_auth") ||
           ap6256_iovar_name_is(buf, len, "join_pref") ||
           ap6256_iovar_name_is(buf, len, "qtxpower") ||
           ap6256_iovar_name_is(buf, len, "mpc") ||
           ap6256_iovar_name_is(buf, len, "roam_off") ||
           ap6256_iovar_name_is(buf, len, "ampdu") ||
           ap6256_iovar_name_is(buf, len, "amsdu") ||
           ap6256_iovar_name_is(buf, len, "ampdu_ba_wsize") ||
           ap6256_iovar_name_is(buf, len, "ampdu_mpdu") ||
           ap6256_iovar_name_is(buf, len, "ampdu_rx_factor") ||
           ap6256_iovar_name_is(buf, len, "frameburst") ||
           ap6256_iovar_name_is(buf, len, "nmode") ||
           ap6256_iovar_name_is(buf, len, "vhtmode") ||
           ap6256_iovar_name_is(buf, len, "mimo_bw_cap");
}

static bool ap6256_join_event_proves_started(void) {
    uint32_t join_state = cyw43_state.wifi_join_state;
    uint32_t kind = join_state & AP6256_WIFI_JOIN_STATE_KIND_MASK;

    if ((kind == AP6256_WIFI_JOIN_STATE_FAIL) ||
        (kind == AP6256_WIFI_JOIN_STATE_NONET) ||
        (kind == AP6256_WIFI_JOIN_STATE_BADAUTH)) {
        return true;
    }
    return (join_state & (AP6256_WIFI_JOIN_STATE_AUTH |
                          AP6256_WIFI_JOIN_STATE_LINK |
                          AP6256_WIFI_JOIN_STATE_KEYED)) != 0U;
}

static bool ap6256_join_should_drop_prelink_data(void) {
    uint32_t join_state = cyw43_state.wifi_join_state;
    uint32_t kind = join_state & AP6256_WIFI_JOIN_STATE_KIND_MASK;

    if (kind != AP6256_WIFI_JOIN_STATE_ACTIVE) {
        return false;
    }

#if CYW43_LWIP
    /*
     * cyw43_cb_process_async_event() intentionally collapses WIFI_JOIN_STATE_ALL
     * back to ACTIVE after PSK_SUP/LINK success, then marks the lwIP netif link
     * up. Once the netif is up, DHCP/EAPOL/data frames are no longer pre-link
     * traffic and must be delivered.
     */
    if (cyw43_tcpip_link_status(&cyw43_state, WWD_STA_INTERFACE) >= CYW43_LINK_NOIP) {
        return false;
    }
#endif

    /*
     * Do not deliver data/EAPOL-looking frames to lwIP while the firmware is
     * still proving association. brcmfmac only exposes netdev RX after carrier
     * evidence; on this AP6256 path the reset signature is a DATA_HEADER packet
     * during join_wait before AUTH/ASSOC/LINK/PSK state exists.
     */
    return (join_state & AP6256_WIFI_JOIN_STATE_KEYED) == 0U;
}

static bool cyw43_sdpcm_tx_window_open(cyw43_int_t *self) {
    uint8_t window = (uint8_t)(self->wwd_sdpcm_last_bus_data_credit -
                               self->wwd_sdpcm_packet_transmit_sequence_number);
    return (uint8_t)(self->wlan_flow_control == 0U && window != 0U && (window & 0x80U) == 0U);
}

void cyw43_ll_init(cyw43_ll_t *self_in, void *cb_data) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);

    self->cb_data = cb_data;
    self->cur_backplane_window = 0;
    self->wwd_sdpcm_packet_transmit_sequence_number = 0;
    self->wwd_sdpcm_last_bus_data_credit = 1; // we get an immediate stall if this isn't done?
    self->wlan_flow_control = 0;
    self->wwd_sdpcm_requested_ioctl_id = 0;
    self->bus_is_up = false;
    self->had_successful_packet = false;
    self->bus_data = 0;
}

void cyw43_ll_deinit(cyw43_ll_t *self_in) {
    #if CYW43_USE_SPI
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);
    cyw43_spi_deinit(self);
    #else
    (void)self_in;
    #endif
}

/*******************************************************************************/
// low level read/write

static uint32_t cyw43_read_reg(cyw43_int_t *self, uint32_t fn, uint32_t reg, size_t size) {
    assert(fn == BACKPLANE_FUNCTION);
    if (size == 1) {
        return cyw43_read_reg_u8(self, fn, reg);
    } else {
        assert(size == 4);
        return cyw43_read_reg_u32(self, fn, reg);
    }
}

static int cyw43_write_reg(cyw43_int_t *self, uint32_t fn, uint32_t reg, size_t size, uint32_t val) {
    assert(fn == BACKPLANE_FUNCTION);
    if (size == 1) {
        return cyw43_write_reg_u8(self, fn, reg, val);
    } else {
        assert(size == 4);
        return cyw43_write_reg_u32(self, fn, reg, val);
    }
}

/*******************************************************************************/
// backplane stuff

static void cyw43_set_backplane_window(cyw43_int_t *self, uint32_t addr) {
    addr = addr & ~BACKPLANE_ADDR_MASK;
    if (addr == self->cur_backplane_window) {
        return;
    }
    if ((addr & 0xff000000) != (self->cur_backplane_window & 0xff000000)) {
        cyw43_write_reg(self, BACKPLANE_FUNCTION, SDIO_BACKPLANE_ADDRESS_HIGH, 1, addr >> 24);
    }
    if ((addr & 0x00ff0000) != (self->cur_backplane_window & 0x00ff0000)) {
        cyw43_write_reg(self, BACKPLANE_FUNCTION, SDIO_BACKPLANE_ADDRESS_MID, 1, addr >> 16);
    }
    if ((addr & 0x0000ff00) != (self->cur_backplane_window & 0x0000ff00)) {
        cyw43_write_reg(self, BACKPLANE_FUNCTION, SDIO_BACKPLANE_ADDRESS_LOW, 1, addr >> 8);
    }
    self->cur_backplane_window = addr;
}

static uint32_t cyw43_read_backplane(cyw43_int_t *self, uint32_t addr, size_t size) {
    cyw43_set_backplane_window(self, addr);
    addr &= BACKPLANE_ADDR_MASK;
    #if CYW43_USE_SPI
    addr |= SBSDIO_SB_ACCESS_2_4B_FLAG;
    #else
    if (size == 4) {
        addr |= SBSDIO_SB_ACCESS_2_4B_FLAG;
    }
    #endif
    uint32_t reg = cyw43_read_reg(self, BACKPLANE_FUNCTION, addr, size);
    if (ap6256_cyw43_port_bus_stage() >= 5U && ap6256_cyw43_port_bus_stage() < 101U) {
        ap6256_cyw43_port_record_backplane_access(0U, self->cur_backplane_window | (addr & BACKPLANE_ADDR_MASK), (uint8_t)size, 0);
    }
    cyw43_set_backplane_window(self, CHIPCOMMON_BASE_ADDRESS);
    return reg;
}

static void cyw43_write_backplane(cyw43_int_t *self, uint32_t addr, size_t size, uint32_t val) {
    cyw43_set_backplane_window(self, addr);
    addr &= BACKPLANE_ADDR_MASK;
    #if CYW43_USE_SPI
    addr |= SBSDIO_SB_ACCESS_2_4B_FLAG;
    #else
    if (size == 4) {
        addr |= SBSDIO_SB_ACCESS_2_4B_FLAG;
    }
    #endif
    cyw43_write_reg(self, BACKPLANE_FUNCTION, addr, size, val);
    if (ap6256_cyw43_port_bus_stage() >= 5U && ap6256_cyw43_port_bus_stage() < 101U) {
        ap6256_cyw43_port_record_backplane_access(1U, self->cur_backplane_window | (addr & BACKPLANE_ADDR_MASK), (uint8_t)size, 0);
    }
    cyw43_set_backplane_window(self, CHIPCOMMON_BASE_ADDRESS);
}

static int cyw43_check_valid_chipset_firmware(cyw43_int_t *self, size_t len, uintptr_t source) {
    (void)self;
    // get the last bit of the firmware, the last 800 bytes
    uint32_t fw_end = 800;
    const uint8_t *b = (const uint8_t *)source + len - fw_end;

    // get length of trailer
    fw_end -= 16; // skip DVID trailer
    uint32_t trail_len = b[fw_end - 2] | b[fw_end - 1] << 8;

    if (trail_len < 500 && b[fw_end - 3] == '\0') {
        for (int i = 80; i < (int)trail_len; ++i) {
            if (strncmp((const char *)&b[fw_end - 3 - i], "Version: ", 9) == 0) {
                // valid chipset firmware found
                // print wifi firmware version info
                CYW43_DEBUG("%s\n", &b[fw_end - 3 - i]);
                return 0;
            }
        }
    }

    // Raw brcmfmac-style firmware blobs do not carry the compact DVID trailer
    // that the reference CYW43 combined images use. Accept those blobs if they
    // still carry a recognizable version string near the tail of the image.
    {
        size_t scan_len = MIN(len, (size_t)32768);
        const uint8_t *scan = (const uint8_t *)source + len - scan_len;

        for (size_t off = 0; off + 9U <= scan_len; ++off) {
            if (memcmp(scan + off, "Version: ", 9) == 0) {
                CYW43_DEBUG("raw firmware trailer accepted\n");
                return 0;
            }
        }
    }

    CYW43_WARN("could not find valid firmware\n");
    return CYW43_FAIL_FAST_CHECK(-CYW43_EIO);
}

#if AP6256_CYW43_SAFE_5G_FEM_NVRAM
typedef struct {
    const char *key;
    const char *line;
} ap6256_nvram_override_t;

static const ap6256_nvram_override_t ap6256_safe_5g_fem_overrides[] = {
    /*
     * Hidden bench-only profile for isolating 5 GHz FEM/PA faults. Do not
     * enable for normal AP6256 qualification: the module NVRAM owns
     * extpagain5g, swctrlmap_5g, and maxp5ga0. Forcing this profile can leave
     * 5 GHz RX scan working while TX/auth frames never reach the AP.
     */
    { "extpagain5g", "extpagain5g=0" },
    { "swctrlmap_5g", "swctrlmap_5g=0x00000000,0x00000000,0x00000000,0x000000,0x000" },
    { "swctrlmapext_5g", "swctrlmapext_5g=0x00000000,0x00000000,0x00000000,0x000000,0x000" },
    { "maxp5ga0", "maxp5ga0=20,20,20,20" },
};

static const char *ap6256_nvram_override_for_entry(const uint8_t *entry, size_t entry_len) {
    for (size_t i = 0U; i < (sizeof(ap6256_safe_5g_fem_overrides) / sizeof(ap6256_safe_5g_fem_overrides[0])); ++i) {
        const char *key = ap6256_safe_5g_fem_overrides[i].key;
        size_t key_len = strlen(key);

        if ((entry_len > key_len) &&
            (entry[key_len] == '=') &&
            (memcmp(entry, key, key_len) == 0)) {
            return ap6256_safe_5g_fem_overrides[i].line;
        }
    }

    return NULL;
}
#endif

static size_t ap6256_prepare_nvram_blob(const uint8_t *src, size_t src_len, uint8_t *dst, size_t dst_len) {
    size_t src_i;
    size_t dst_i;

    if ((src == NULL) || (dst == NULL) || (src_len == 0U) || (dst_len < 2U)) {
        return 0U;
    }

    for (src_i = 0U; src_i < src_len; ++src_i) {
        if (src[src_i] == '\0') {
            size_t copy_len = src_len;

            if (copy_len > dst_len) {
                return 0U;
            }

            memcpy(dst, src, copy_len);
            if (dst[copy_len - 1U] != '\0') {
                if (copy_len >= dst_len) {
                    return 0U;
                }
                dst[copy_len++] = '\0';
            }
            return copy_len;
        }
    }

    src_i = 0U;
    dst_i = 0U;
    while (src_i < src_len) {
        size_t line_start = src_i;
        size_t line_end = src_i;
        size_t trimmed_start;
        size_t trimmed_end;

        while ((line_end < src_len) && (src[line_end] != '\n') && (src[line_end] != '\r')) {
            ++line_end;
        }

        trimmed_start = line_start;
        while ((trimmed_start < line_end) &&
               ((src[trimmed_start] == ' ') || (src[trimmed_start] == '\t'))) {
            ++trimmed_start;
        }

        trimmed_end = line_end;
        while ((trimmed_end > trimmed_start) &&
               ((src[trimmed_end - 1U] == ' ') || (src[trimmed_end - 1U] == '\t'))) {
            --trimmed_end;
        }

        if ((trimmed_start < trimmed_end) && (src[trimmed_start] != '#')) {
            const uint8_t *line = &src[trimmed_start];
            size_t line_len = trimmed_end - trimmed_start;
#if AP6256_CYW43_SAFE_5G_FEM_NVRAM
            const char *override_line = ap6256_nvram_override_for_entry(line, line_len);

            if (override_line != NULL) {
                line = (const uint8_t *)override_line;
                line_len = strlen(override_line);
            }
#endif

            if ((dst_i + line_len + 1U) >= dst_len) {
                return 0U;
            }

            memcpy(&dst[dst_i], line, line_len);
            dst_i += line_len;
            dst[dst_i++] = '\0';
        }

        src_i = line_end;
        while ((src_i < src_len) && ((src[src_i] == '\n') || (src[src_i] == '\r'))) {
            ++src_i;
        }
    }

    if ((dst_i + 1U) >= dst_len) {
        return 0U;
    }

    dst[dst_i++] = '\0';
    return dst_i;
}

static int ap6256_validate_nvram_blob(const uint8_t *blob,
                                      size_t packed_len,
                                      size_t padded_len,
                                      uint32_t *footer_word_out) {
    uint32_t footer_word;

    if ((blob == NULL) || (packed_len < 2U) || (padded_len < packed_len) ||
        (padded_len > CYW43_RAM_SIZE) || ((padded_len % 4U) != 0U)) {
        return CYW43_FAIL_FAST_CHECK(-CYW43_EINVAL);
    }

    if ((blob[packed_len - 1U] != '\0') || (blob[packed_len - 2U] != '\0')) {
        return CYW43_FAIL_FAST_CHECK(-CYW43_EINVAL);
    }

    footer_word = ((~((uint32_t)(padded_len / 4U)) & 0xFFFFU) << 16U) |
                  ((uint32_t)(padded_len / 4U) & 0xFFFFU);
    if ((((footer_word >> 16U) ^ footer_word) & 0xFFFFU) != 0xFFFFU) {
        return CYW43_FAIL_FAST_CHECK(-CYW43_EINVAL);
    }

    if (footer_word_out != NULL) {
        *footer_word_out = footer_word;
    }
    return 0;
}

static int cyw43_download_resource(cyw43_int_t *self, uint32_t addr, size_t len, uintptr_t source) {
    CYW43_VDEBUG("writing %u bytes to 0x%x\n", (uint32_t)len, (uint32_t)addr);

    uint32_t block_size = CYW43_BUS_MAX_BLOCK_SIZE;
    #if !CYW43_USE_SPI
    // The STM32H7/AP6256 SDIO path is much happier using byte-mode backplane
    // transfers during firmware download than larger block-mode bursts.
    if (block_size > 64) {
        block_size = 64;
    }
    #endif
    uint8_t padded_block[CYW43_BUS_MAX_BLOCK_SIZE];

    #if CYW43_VERBOSE_DEBUG
    uint32_t t_start = cyw43_hal_ticks_us();
    #endif

    for (size_t offset = 0; offset < len; offset += block_size) {
        CYW43_EVENT_POLL_HOOK;

        size_t sz = block_size;
        if (offset + sz > len) {
            sz = len - offset;
        }
        uint32_t dest_addr = addr + offset;
        assert(((dest_addr & BACKPLANE_ADDR_MASK) + sz) <= (BACKPLANE_ADDR_MASK + 1));
        cyw43_set_backplane_window(self, dest_addr);
        const uint8_t *src = (const uint8_t *)source + offset;
        size_t transfer_len = CYW43_WRITE_BYTES_PAD(sz);
        dest_addr &= BACKPLANE_ADDR_MASK;
        #if CYW43_USE_SPI
        dest_addr |= SBSDIO_SB_ACCESS_2_4B_FLAG;
        #else
        dest_addr |= SBSDIO_SB_ACCESS_2_4B_FLAG;
        #endif
        if (transfer_len != sz) {
            memset(padded_block, 0, sizeof(padded_block));
            memcpy(padded_block, src, sz);
            src = padded_block;
        }
        int ret = cyw43_write_bytes(self, BACKPLANE_FUNCTION, dest_addr, transfer_len, src);
        if (ret != 0) {

            return CYW43_FAIL_FAST_CHECK(ret);
        }
    }

    #if CYW43_VERBOSE_DEBUG
    uint32_t t_end = cyw43_hal_ticks_us();
    uint32_t dt = t_end - t_start;
    CYW43_VDEBUG("done dnload; dt = %u us; speed = %u kbytes/sec\n", (unsigned int)dt, (unsigned int)(len * 1000 / dt));
    #endif

    #if CYW43_RESOURCE_VERIFY_DOWNLOAD

    // Verification of 380k takes about 40ms using a 512-byte transfer size
    const size_t verify_block_size = CYW43_BUS_MAX_BLOCK_SIZE;
    uint8_t buf[verify_block_size];

    #if CYW43_VERBOSE_DEBUG
    t_start = cyw43_hal_ticks_us();
    #endif

    for (size_t offset = 0; offset < len; offset += verify_block_size) {
        size_t sz = verify_block_size;
        if (offset + sz > len) {
            sz = len - offset;
        }
        uint32_t dest_addr = addr + offset;
        assert(((dest_addr & BACKPLANE_ADDR_MASK) + sz) <= (BACKPLANE_ADDR_MASK + 1));
        cyw43_set_backplane_window(self, dest_addr);
        cyw43_read_bytes(self, BACKPLANE_FUNCTION, dest_addr & BACKPLANE_ADDR_MASK, sz, buf);
        const uint8_t *src = (const uint8_t *)source + offset;
        if (memcmp(buf, src, sz) != 0) {
            CYW43_WARN("fail verify at address 0x%08x:\n", (unsigned int)dest_addr);
            cyw43_xxd(sz, src);
            cyw43_xxd(sz, buf);
            return CYW43_FAIL_FAST_CHECK(-CYW43_EIO);
        }
    }

    #if CYW43_VERBOSE_DEBUG
    t_end = cyw43_hal_ticks_us();
    dt = t_end - t_start;
    CYW43_VDEBUG("done verify; dt = %u us; speed = %u kbytes/sec\n", (unsigned int)dt, (unsigned int)(len * 1000 / dt));
    #endif

    #endif // CYW43_RESOURCE_VERIFY_DOWNLOAD

    return 0;
}

/*******************************************************************************/
// Async event parsing

typedef struct _cyw43_scan_result_internal_t {
    uint32_t version;
    uint32_t length;
    uint8_t bssid[6];
    uint16_t beacon_period;
    uint16_t capability;
    uint8_t ssid_len;
    uint8_t ssid[32];
    uint32_t rateset_count;
    uint8_t rateset_rates[16];
    uint16_t chanspec;
    uint16_t atim_window;
    uint8_t dtim_period;
    int16_t rssi;
    int8_t phy_noise;
    uint8_t n_cap;
    uint32_t nbss_cap;
    uint8_t ctl_ch;
    uint32_t reserved32[1];
    uint8_t flags;
    uint8_t reserved[3];
    uint8_t basic_mcs[16];
    uint16_t ie_offset;
    uint32_t ie_length;
    int16_t SNR;
} cyw43_scan_result_internal_t;

#define AP6256_CYW43_ASYNC_EVENT_MIN_LEN ((size_t)offsetof(cyw43_async_event_t, u))
#define AP6256_CYW43_ESCAN_RESULT_OFFSET ((size_t)offsetof(cyw43_async_event_t, u.scan_result))
#define AP6256_CYW43_ESCAN_BSS_OFFSET    (AP6256_CYW43_ESCAN_RESULT_OFFSET + 12U)

static inline uint32_t cyw43_be32toh(uint32_t x) {
    return (x >> 24) | (x >> 8 & 0xff00) | (x << 8 & 0xff0000) | x << 24;
}

static inline uint16_t cyw43_be16toh(uint16_t x) {
    return (x >> 8) | (x << 8);
}

static uint16_t cyw43_get_le16_unaligned(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static bool cyw43_scan_suite_oui_valid(const uint8_t *suite, bool rsn_suite) {
    const uint8_t *oui = rsn_suite ? (const uint8_t *)"\x00\x0f\xac" :
                                     (const uint8_t *)"\x00\x50\xf2";
    return memcmp(suite, oui, 3) == 0;
}

static uint16_t cyw43_scan_cipher_from_suite(const uint8_t *suite, bool rsn_suite) {
    if (!cyw43_scan_suite_oui_valid(suite, rsn_suite)) {
        return 0;
    }

    switch (suite[3]) {
        case 1:
            return CYW43_SCAN_CIPHER_WEP40;
        case 2:
            return CYW43_SCAN_CIPHER_TKIP;
        case 4:
            return CYW43_SCAN_CIPHER_CCMP;
        case 5:
            return CYW43_SCAN_CIPHER_WEP104;
        case 6:
            return CYW43_SCAN_CIPHER_BIP;
        case 8:
            return CYW43_SCAN_CIPHER_GCMP;
        default:
            return 0;
    }
}

static uint16_t cyw43_scan_akm_from_suite(const uint8_t *suite, bool rsn_suite) {
    if (!cyw43_scan_suite_oui_valid(suite, rsn_suite)) {
        return 0;
    }

    switch (suite[3]) {
        case 1:
            return CYW43_SCAN_AKM_8021X;
        case 2:
            return CYW43_SCAN_AKM_PSK;
        case 5:
            return CYW43_SCAN_AKM_8021X_SHA256;
        case 6:
            return CYW43_SCAN_AKM_PSK_SHA256;
        case 8:
            return rsn_suite ? CYW43_SCAN_AKM_SAE : 0;
        case 18:
            return rsn_suite ? CYW43_SCAN_AKM_OWE : 0;
        default:
            return 0;
    }
}

static void cyw43_scan_parse_wpa_rsn_ie(const uint8_t *ie,
                                        bool is_rsn,
                                        uint16_t *pairwise_flags,
                                        uint16_t *group_flags,
                                        uint16_t *akm_flags,
                                        uint16_t *rsn_cap,
                                        uint8_t *mfp) {
    const uint8_t *body = ie + 2;
    size_t body_len = ie[1];
    size_t off;
    uint16_t count;

    if (!is_rsn) {
        if ((body_len < 4U) || (memcmp(body, WPA_OUI_TYPE1, 4U) != 0)) {
            return;
        }
        body += 4;
        body_len -= 4U;
    }

    if (body_len < 8U) {
        return;
    }

    off = 0U;
    off += 2U; /* version */

    *group_flags |= cyw43_scan_cipher_from_suite(body + off, is_rsn);
    off += 4U;

    if (off + 2U > body_len) {
        return;
    }
    count = cyw43_get_le16_unaligned(body + off);
    off += 2U;
    if (off + ((size_t)count * 4U) > body_len) {
        return;
    }
    for (uint16_t i = 0; i < count; ++i) {
        *pairwise_flags |= cyw43_scan_cipher_from_suite(body + off, is_rsn);
        off += 4U;
    }

    if (off + 2U > body_len) {
        return;
    }
    count = cyw43_get_le16_unaligned(body + off);
    off += 2U;
    if (off + ((size_t)count * 4U) > body_len) {
        return;
    }
    for (uint16_t i = 0; i < count; ++i) {
        *akm_flags |= cyw43_scan_akm_from_suite(body + off, is_rsn);
        off += 4U;
    }

    if (is_rsn && (off + 2U <= body_len)) {
        *rsn_cap = cyw43_get_le16_unaligned(body + off);
        if ((*rsn_cap & RSN_CAP_MFPR_MASK) != 0U) {
            *mfp = CYW43_SCAN_MFP_REQUIRED;
        } else if (((*rsn_cap & RSN_CAP_MFPC_MASK) != 0U) && (*mfp != CYW43_SCAN_MFP_REQUIRED)) {
            *mfp = CYW43_SCAN_MFP_CAPABLE;
        }
    }
}

static void cyw43_ll_wifi_parse_scan_result(cyw43_async_event_t *ev) {
    struct _scan_result_t {
        uint32_t buflen;
        uint32_t version;
        uint16_t sync_id;
        uint16_t bss_count;
        cyw43_scan_result_internal_t bss;
    } *scan_res = (void *)((uint8_t *)ev + 48);

    if (scan_res->bss.ie_offset + scan_res->bss.ie_length > scan_res->bss.length) {
        // invalid length
        ev->status = (uint32_t)-1; // set invalid
        return;
    }

    // Parse IE elements. brcmfmac validates ie_offset/ie_length and uses the
    // RSN/WPA IEs to classify AKM, ciphers and MFP before connect; keep the
    // same useful facts in our compact MCU scan result.
    uint8_t *ie_ptr = (uint8_t *)&scan_res->bss + scan_res->bss.ie_offset;
    uint8_t *ie_top = ie_ptr + scan_res->bss.ie_length;
    uint8_t *ie_rsn = NULL;
    uint8_t *ie_wpa = NULL;
    while (ie_ptr + 2U <= ie_top) {
        uint8_t ie_type = ie_ptr[0];
        uint8_t ie_len = ie_ptr[1];
        if (ie_ptr + 2 + ie_len <= ie_top) {
            // valid IE element
            if (ie_type == DOT11_IE_ID_RSN) {
                ie_rsn = ie_ptr;
            } else if (ie_type == DOT11_IE_ID_VENDOR_SPECIFIC) {
                if (memcmp(ie_ptr + 2, WPA_OUI_TYPE1, 4) == 0) {
                    ie_wpa = ie_ptr;
                }
            }
        } else {
            break;
        }
        ie_ptr += 2 + ie_len;
    }
    int security = 0;// OPEN
    uint8_t security_flags = 0;
    uint16_t pairwise_flags = 0;
    uint16_t group_flags = 0;
    uint16_t akm_flags = 0;
    uint16_t rsn_cap = 0;
    uint8_t mfp = CYW43_SCAN_MFP_NONE;
    if (ie_rsn != NULL) {
        cyw43_scan_parse_wpa_rsn_ie(ie_rsn,
                                    true,
                                    &pairwise_flags,
                                    &group_flags,
                                    &akm_flags,
                                    &rsn_cap,
                                    &mfp);
        security |= 4;// WPA2;
        security_flags |= CYW43_SCAN_SEC_RSN;
    }
    if (ie_wpa != NULL) {
        cyw43_scan_parse_wpa_rsn_ie(ie_wpa,
                                    false,
                                    &pairwise_flags,
                                    &group_flags,
                                    &akm_flags,
                                    &rsn_cap,
                                    &mfp);
        security |= 2;// WPA;
        security_flags |= CYW43_SCAN_SEC_WPA;
    }
    if (scan_res->bss.capability & DOT11_CAP_PRIVACY) {
        if ((security_flags & (CYW43_SCAN_SEC_RSN | CYW43_SCAN_SEC_WPA)) == 0U) {
            security |= 1;// WEP_PSK;
            security_flags |= CYW43_SCAN_SEC_WEP;
        }
    }

    ev->u.scan_result.chanspec = scan_res->bss.chanspec;
    /*
     * brcmfmac exposes the BSS primary/control channel to cfg80211 and keeps
     * the raw chanspec as separate PHY metadata. BCM43456 reports 80 MHz BSS
     * chanspecs with a center-channel-like low byte (for example 0xE09B),
     * which is not the channel that should guide association. Use ctl_ch when
     * firmware provides it, while preserving raw chanspec for diagnostics.
     */
    ev->u.scan_result.channel = (scan_res->bss.ctl_ch != 0U) ?
                                scan_res->bss.ctl_ch :
                                (scan_res->bss.chanspec & 0xffU);
    ev->u.scan_result.auth_mode = security;
    ev->u.scan_result.security_flags = security_flags;
    ev->u.scan_result.group_cipher_flags = group_flags;
    ev->u.scan_result.pairwise_cipher_flags = pairwise_flags;
    ev->u.scan_result.akm_flags = akm_flags;
    ev->u.scan_result.rsn_cap = rsn_cap;
    ev->u.scan_result.mfp = mfp;

    return;
}

static cyw43_async_event_t *cyw43_ll_parse_async_event(size_t len, uint8_t *buf) {
    if ((buf == NULL) || (len < AP6256_CYW43_ASYNC_EVENT_MIN_LEN)) {
        ap6256_cyw43_port_record_async_event(0xFFFFFFFFU, (uint32_t)len, 0U, 0U);
        return NULL;
    }

    // buf = &spid_buf[46], so the event data structure is only aligned to 2 bytes.
    // We do get a hard fault with unaligned data (not sure exactly why) so relocate
    // the data to an aligned address, using custom code for efficiency.
    // TODO: improve by only copying the data we need to for any given event.
    {
        uint32_t *d = (void *)&buf[-2];
        uint16_t *s = (void *)&buf[0];
        for (size_t i = (len + 3) >> 2; i; --i) {
            *d++ = s[0] | s[1] << 16;
            s += 2;
        }
    }

    // Cast data to the async event struct
    cyw43_async_event_t *ev = (void *)&buf[-2];

    // Convert endianness of fields
    ev->flags = cyw43_be16toh(ev->flags);
    ev->event_type = cyw43_be32toh(ev->event_type);
    ev->status = cyw43_be32toh(ev->status);
    ev->reason = cyw43_be32toh(ev->reason);

    // Parse additional type-specific event data
    if (ev->event_type == CYW43_EV_ESCAN_RESULT && ev->status == CYW43_STATUS_PARTIAL) {
        if (len < (AP6256_CYW43_ESCAN_BSS_OFFSET + sizeof(cyw43_scan_result_internal_t))) {
            ev->status = (uint32_t)-1;
            return ev;
        }
        cyw43_ll_wifi_parse_scan_result(ev);
    } else if ((ev->event_type == CYW43_EV_ESCAN_RESULT) &&
               (ev->status != CYW43_STATUS_PARTIAL)) {
        /*
         * Track firmware scan lifetime separately from the monotonically
         * increasing sync id. The sync id remains useful for diagnostics and
         * future scans, but a completed/aborted ESCAN must not make the join
         * path send another abort immediately before WLC_SET_SSID.
         */
        s_ap6256_scan_active = 0U;
    }

    return ev;
}

/*******************************************************************************/
// SDPCM stuff

struct sdpcm_header_t {
    uint16_t size;
    uint16_t size_com;
    uint8_t sequence;
    uint8_t channel_and_flags;
    uint8_t next_length;
    uint8_t header_length;
    uint8_t wireless_flow_control;
    uint8_t bus_data_credit;
    uint8_t reserved[2];
};

// buf must be writable and have:
//  - SDPCM_HEADER_LEN bytes at the start for writing the headers
//  - readable data at the end for padding to get to 64 byte alignment
static int cyw43_sdpcm_send_common(cyw43_int_t *self, uint32_t kind, size_t len, uint8_t *buf) {
    // validate args
    if (kind != CONTROL_HEADER && kind != DATA_HEADER) {
        return CYW43_FAIL_FAST_CHECK(-CYW43_EINVAL);
    }

    cyw43_ll_bus_sleep((void *)self, false);
    ap6256_cyw43_port_record_send_credit(self->wlan_flow_control,
                                         self->wwd_sdpcm_packet_transmit_sequence_number,
                                         self->wwd_sdpcm_last_bus_data_credit,
                                         0U,
                                         0);

    /*
     * brcmfmac keeps association triggers on the control path, but it still
     * treats the SDPCM sequence/credit window as authoritative. The reset seen
     * on AP6256 happens immediately after a 5 GHz association control write,
     * so WLC_SET_SSID must not be pushed on inferred credit.
     */
    bool association_trigger = (kind == CONTROL_HEADER) &&
        (s_ap6256_pending_ioctl_association_trigger ||
         ap6256_ioctl_is_association_trigger(s_ap6256_pending_ioctl_kind,
                                             s_ap6256_pending_ioctl_cmd));
    bool require_tx_credit = (kind == DATA_HEADER) || association_trigger;

    if (require_tx_credit && !cyw43_sdpcm_tx_window_open(self)) {
        CYW43_VDEBUG("[CYW43;%u] STALL(%u;%u-%u)\n", (int)cyw43_hal_ticks_ms(), self->wlan_flow_control, self->wwd_sdpcm_packet_transmit_sequence_number, self->wwd_sdpcm_last_bus_data_credit);

        uint32_t start_us = cyw43_hal_ticks_us();
        uint32_t last_poke = start_us - 100000;
        uint32_t wait_loops = 0U;
        uint8_t synthetic_credit = 0U;
        ap6256_cyw43_port_set_ioctl_phase(AP6256_CYW43_IOCTL_PHASE_SEND_CREDIT);
        ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SDPCM_CREDIT_WAIT,
                                            (int32_t)(((kind & 0xFFU) << 16U) | (len & 0xFFFFU)));
        for (;;) {
            uint32_t cur_us = cyw43_hal_ticks_us();
            if (cur_us - last_poke >= 100000) {
                CYW43_VDEBUG("STALL(%u;%u-%u): do poke at %u us\n", self->wlan_flow_control, self->wwd_sdpcm_packet_transmit_sequence_number, self->wwd_sdpcm_last_bus_data_credit, (unsigned int)(cur_us - start_us));
                last_poke = cur_us;
                #if !CYW43_USE_SPI
                cyw43_write_backplane(self, SDIO_TO_SB_MAILBOX, 4, 1 << 3);
                #endif
            }
            size_t res_len;
            uint8_t *res_buf;
            int ret = cyw43_ll_sdpcm_poll_device(self, &res_len, &res_buf);
            ap6256_cyw43_port_record_send_credit(self->wlan_flow_control,
                                                 self->wwd_sdpcm_packet_transmit_sequence_number,
                                                 self->wwd_sdpcm_last_bus_data_credit,
                                                 synthetic_credit,
                                                 ret);
            if (ret != -1) {
                // CYW43_WARN("STALL(%u;%u-%u): got %d, %u\n", self->wlan_flow_control, self->wwd_sdpcm_packet_transmit_sequence_number, self->wwd_sdpcm_last_bus_data_credit, ret, res_len);
            }
            /*
            if (ret == CONTROL_HEADER) {
                // it seems that res_len is always the length of the argument in buf
                memmove(buf, res_buf, len < res_len ? len : res_len);
                return;
            } else*/
            if (ret == ASYNCEVENT_HEADER) {
                cyw43_async_event_t *ev = cyw43_ll_parse_async_event(res_len, res_buf);
                if (ev != NULL) {
                    cyw43_cb_process_async_event(self, ev);
                }
            } else if (ret == DATA_HEADER) {
                // Don't process it due to possible reentrancy issues (eg sending another ETH as part of the reception)
                // printf("STALL: not processing ethernet packet\n");
                // cyw43_tcpip_process_ethernet(self, res_len, res_buf);
            } else if (ret >= 0) {
                // printf("cyw43_do_ioctl: got unexpected packet %d\n", ret);
            }
            if (cyw43_sdpcm_tx_window_open(self)) {
                // CYW43_WARN("STALL(%u;%u-%u): done in %u us\n", self->wlan_flow_control, self->wwd_sdpcm_packet_transmit_sequence_number, self->wwd_sdpcm_last_bus_data_credit, (unsigned int)(cur_us - start_us));
                break;
            }
            if (AP6256_CYW43_ENABLE_SYNTHETIC_CONTROL_CREDIT
                && kind == CONTROL_HEADER
                && !association_trigger
                && !self->wlan_flow_control
                && ret == -1
                && synthetic_credit == 0U
                && (cur_us - start_us) > AP6256_CYW43_CONTROL_CREDIT_RECOVERY_US) {
                /*
                 * BCM43456/AP6256 can reach the first post-boot scan ioctl with
                 * no visible packet-pending signal to harvest a fresh SDPCM
                 * credit. Allow exactly one control frame through after the
                 * firmware is awake and not flow-controlling; data frames still
                 * require a real credit.
                 */
                synthetic_credit = 1U;
                self->wwd_sdpcm_last_bus_data_credit = (uint8_t)(self->wwd_sdpcm_packet_transmit_sequence_number + 1U);
                ap6256_cyw43_port_record_send_credit(self->wlan_flow_control,
                                                     self->wwd_sdpcm_packet_transmit_sequence_number,
                                                     self->wwd_sdpcm_last_bus_data_credit,
                                                     synthetic_credit,
                                                     0);
                break;
            }
#if CYW43_LWIP
            if (kind == DATA_HEADER
                && !self->wlan_flow_control
                && ret == -1
                && synthetic_credit == 0U
                && (cur_us - start_us) > AP6256_CYW43_DATA_CREDIT_RECOVERY_US
                && cyw43_tcpip_link_status(&cyw43_state, WWD_STA_INTERFACE) >= CYW43_LINK_NOIP) {
                /*
                 * After PSK_SUP/KEYED the AP6256 firmware can present RX data
                 * while withholding a fresh SDPCM TX credit. brcmfmac keeps
                 * the netdev queue stopped until credits recover; on this MCU
                 * path that starves DHCP forever. Allow one post-link data
                 * frame through when firmware is awake and not flow-controlled.
                 */
                synthetic_credit = 1U;
                self->wwd_sdpcm_last_bus_data_credit =
                    (uint8_t)(self->wwd_sdpcm_packet_transmit_sequence_number + 1U);
                ap6256_cyw43_port_record_send_credit(self->wlan_flow_control,
                                                     self->wwd_sdpcm_packet_transmit_sequence_number,
                                                     self->wwd_sdpcm_last_bus_data_credit,
                                                     synthetic_credit,
                                                     0);
                break;
            }
#endif
            if (cur_us - start_us > CYW43_IOCTL_TIMEOUT_US) {
                CYW43_WARN("STALL(%u;%u-%u): timeout\n", self->wlan_flow_control, self->wwd_sdpcm_packet_transmit_sequence_number, self->wwd_sdpcm_last_bus_data_credit);
                ap6256_cyw43_port_record_send_credit(self->wlan_flow_control,
                                                     self->wwd_sdpcm_packet_transmit_sequence_number,
                                                     self->wwd_sdpcm_last_bus_data_credit,
                                                     synthetic_credit,
                                                     -CYW43_ETIMEDOUT);
                ap6256_cyw43_port_set_ioctl_phase(AP6256_CYW43_IOCTL_PHASE_SEND_CREDIT_TIMEOUT);
                return CYW43_FAIL_FAST_CHECK(-CYW43_ETIMEDOUT);
            }
            if (++wait_loops > 5000U) {
                ap6256_cyw43_port_record_send_credit(self->wlan_flow_control,
                                                     self->wwd_sdpcm_packet_transmit_sequence_number,
                                                     self->wwd_sdpcm_last_bus_data_credit,
                                                     synthetic_credit,
                                                     -CYW43_ETIMEDOUT);
                ap6256_cyw43_port_set_ioctl_phase(AP6256_CYW43_IOCTL_PHASE_SEND_CREDIT_TIMEOUT);
                return CYW43_FAIL_FAST_CHECK(-CYW43_ETIMEDOUT);
            }
            CYW43_SDPCM_SEND_COMMON_WAIT;
        }
    }

    size_t size = SDPCM_HEADER_LEN + len;

    // create header
    struct sdpcm_header_t *header = (void *)&buf[0];
    header->size = size;
    header->size_com = ~size & 0xffff;
    header->sequence = self->wwd_sdpcm_packet_transmit_sequence_number;
    header->channel_and_flags = kind;
    header->next_length = 0;
    header->header_length = SDPCM_HEADER_LEN + (kind == DATA_HEADER ? 2 : 0);
    header->wireless_flow_control = 0;
    header->bus_data_credit = 0;
    header->reserved[0] = 0;
    header->reserved[1] = 0;

    self->wwd_sdpcm_packet_transmit_sequence_number += 1;
    ap6256_cyw43_port_record_send_credit(self->wlan_flow_control,
                                         self->wwd_sdpcm_packet_transmit_sequence_number,
                                         self->wwd_sdpcm_last_bus_data_credit,
                                         ap6256_cyw43_port_send_synthetic_credit(),
                                         0);

    /*
     * brcmfmac keeps small control frames as byte-mode CMD53 transfers when
     * they fit within the programmed F2 block size. Padding every association
     * control frame to bs64/l128 made BCM43456 reset immediately after directed
     * 5 GHz join TX. With F2 set to 512 for association, keep sub-512 control
     * frames 4-byte aligned so cyw43_sdio_cmd53() selects byte mode.
     */
    size_t transfer_len = CYW43_SDPCM_WRITE_BYTES_PAD(size);
    if ((kind == CONTROL_HEADER) &&
        (size <= 512U) &&
        (ap6256_cyw43_port_runtime_f2_block_size() >= 512U)) {
        transfer_len = ALIGN_UINT(size, 4U);
    }
    if (kind == CONTROL_HEADER) {
        ap6256_cyw43_port_record_control_tx_frame(kind,
                                                  s_ap6256_pending_ioctl_cmd,
                                                  s_ap6256_pending_ioctl_iface,
                                                  (len >= 16U) ? (uint32_t)(len - 16U) : 0U,
                                                  self->wwd_sdpcm_requested_ioctl_id,
                                                  (uint32_t)size,
                                                  (uint32_t)transfer_len,
                                                  ap6256_cyw43_port_runtime_f2_block_size(),
                                                  buf,
                                                  (uint32_t)size);
    }
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SDPCM_TX,
                                        (int32_t)(((kind & 0xFFU) << 16U) |
                                                  (transfer_len & 0xFFFFU)));
    int tx_ret = cyw43_write_bytes(self, WLAN_FUNCTION, 0, transfer_len, buf);
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SDPCM_TX, tx_ret);
    return tx_ret;
}

struct ioctl_header_t {
    uint32_t cmd;
    uint32_t len; // lower 16 is output len; upper 16 is input len
    uint32_t flags;
    uint32_t status;
};

#define CASE_RETURN_STRING(value) case value: \
        return # value;

#if CYW43_VERBOSE_DEBUG
static const char *ioctl_cmd_name(int id) {
    switch (id)
    {
        CASE_RETURN_STRING(WLC_UP)
        CASE_RETURN_STRING(WLC_SET_INFRA)
        CASE_RETURN_STRING(WLC_SET_AUTH)
        CASE_RETURN_STRING(WLC_GET_BSSID)
        CASE_RETURN_STRING(WLC_GET_SSID)
        CASE_RETURN_STRING(WLC_SET_SSID)
        CASE_RETURN_STRING(WLC_SET_CHANNEL)
        CASE_RETURN_STRING(WLC_SCAN)
        CASE_RETURN_STRING(WLC_DISASSOC)
        CASE_RETURN_STRING(WLC_GET_ANTDIV)
        CASE_RETURN_STRING(WLC_SET_ANTDIV)
        CASE_RETURN_STRING(WLC_SET_DTIMPRD)
        CASE_RETURN_STRING(WLC_GET_PM)
        CASE_RETURN_STRING(WLC_SET_PM)
        CASE_RETURN_STRING(WLC_SET_GMODE)
        CASE_RETURN_STRING(WLC_SET_WSEC)
        CASE_RETURN_STRING(WLC_SET_BAND)
        CASE_RETURN_STRING(WLC_GET_ASSOCLIST)
        CASE_RETURN_STRING(WLC_SET_WPA_AUTH)
        CASE_RETURN_STRING(WLC_SET_VAR)
        CASE_RETURN_STRING(WLC_GET_VAR)
        CASE_RETURN_STRING(WLC_SET_WSEC_PMK)
        default:
            assert(false);
            return "unknown";
    }
}
#endif

static int cyw43_send_ioctl(cyw43_int_t *self, uint32_t kind, uint32_t cmd, size_t len, const uint8_t *buf, uint32_t iface) {
    int send_ret;

    if (SDPCM_HEADER_LEN + 16 + len > sizeof(self->spid_buf)) {
        return CYW43_FAIL_FAST_CHECK(-CYW43_EINVAL);
    }

    self->wwd_sdpcm_requested_ioctl_id += 1;
    s_ap6256_pending_ioctl_cmd = cmd;
    s_ap6256_pending_ioctl_iface = iface;
    s_ap6256_pending_ioctl_kind = kind;
    s_ap6256_pending_ioctl_association_trigger =
        ap6256_ioctl_is_association_trigger(kind, cmd) ||
        ap6256_ioctl_is_join_iovar(kind, cmd, len, buf);
    uint32_t flags = ((((uint32_t)self->wwd_sdpcm_requested_ioctl_id) << CDCF_IOC_ID_SHIFT) & CDCF_IOC_ID_MASK)
        | kind | (iface << CDCF_IOC_IF_SHIFT);

    // create header
    struct ioctl_header_t *header = (void *)&self->spid_buf[SDPCM_HEADER_LEN];
    header->cmd = cmd;
    header->len = len & 0xffff;
    header->flags = flags;
    header->status = 0;

    // copy in payload
    memmove(self->spid_buf + SDPCM_HEADER_LEN + 16, buf, len);

    // do transfer
    CYW43_VDEBUG("Sending cmd %s (%u) len %u flags %u status %u\n", ioctl_cmd_name(header->cmd), header->cmd, header->len, header->flags, header->status);
    if (header->cmd == WLC_SET_VAR || header->cmd == WLC_GET_VAR) {
        CYW43_VDEBUG("%s %s\n", ioctl_cmd_name(header->cmd), (const char *)buf);
    }
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_IOCTL_SEND,
                                        (int32_t)(((cmd & 0x3FFU) << 16U) |
                                                  (self->wwd_sdpcm_requested_ioctl_id & 0xFFFFU)));
    send_ret = cyw43_sdpcm_send_common(self, CONTROL_HEADER, 16 + len, self->spid_buf);
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_IOCTL_SEND, send_ret);
    return send_ret;
}

struct sdpcm_bdc_header_t {
    uint8_t flags;
    uint8_t priority;
    uint8_t flags2;
    uint8_t data_offset;
};

int cyw43_ll_send_ethernet(cyw43_ll_t *self_in, int itf, size_t len, const void *buf, bool is_pbuf) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);

    if (SDPCM_HEADER_LEN + 2 + sizeof(struct sdpcm_bdc_header_t) + len > sizeof(self->spid_buf)) {
        return CYW43_FAIL_FAST_CHECK(-CYW43_EINVAL);
    }

    // create header
    // there are 2 bytes of padding after the sdpcm header,
    // corresponding to the +2 in cyw43_sdpcm_send_common for DATA_HEADER
    struct sdpcm_bdc_header_t *header = (void *)&self->spid_buf[SDPCM_HEADER_LEN + 2];
    header->flags = 0x20;
    header->priority = 0;
    header->flags2 = itf;
    header->data_offset = 0;

    // copy in payload
    if (is_pbuf) {
        pbuf_copy_partial((const struct pbuf *)buf, self->spid_buf + SDPCM_HEADER_LEN + 6, len, 0);
    } else {
        memmove(self->spid_buf + SDPCM_HEADER_LEN + 6, buf, len);
    }

    // do transfer
    return cyw43_sdpcm_send_common(self, DATA_HEADER, 6 + len, self->spid_buf);
}

static int sdpcm_process_rx_packet(cyw43_int_t *self, uint8_t *buf, size_t *out_len, uint8_t **out_buf) {
    const struct sdpcm_header_t *header = (const void *)buf;
    if (header->size != (~header->size_com & 0xffff)) {
        // invalid packet, just ignore it
        CYW43_DEBUG("Ignoring invalid packet\n");
        ap6256_cyw43_port_record_rx_frame(AP6256_CYW43_RX_CLASS_MALFORMED,
                                          header->channel_and_flags & 0x0f,
                                          header->size,
                                          0U,
                                          buf);
        return -2;
    }
    if (header->size < SDPCM_HEADER_LEN) {
        // packet too small, just ignore it
        CYW43_DEBUG("Ignoring too small packet\n");
        ap6256_cyw43_port_record_rx_frame(AP6256_CYW43_RX_CLASS_MALFORMED,
                                          header->channel_and_flags & 0x0f,
                                          header->size,
                                          0U,
                                          buf);
        return -3;
    }
    if ((header->size > sizeof(self->spid_buf)) ||
        (header->header_length < SDPCM_HEADER_LEN) ||
        (header->header_length > header->size)) {
        CYW43_DEBUG("Ignoring malformed SDPCM packet size=%u header_len=%u\n",
                    (unsigned int)header->size,
                    (unsigned int)header->header_length);
        ap6256_cyw43_port_record_rx_frame(AP6256_CYW43_RX_CLASS_MALFORMED,
                                          header->channel_and_flags & 0x0f,
                                          header->size,
                                          0U,
                                          buf);
        return -3;
    }

    #ifndef NDEBUG
    // Update flow control
    if (self->wlan_flow_control != header->wireless_flow_control) {
        CYW43_DEBUG("WLAN: changed flow control: %d -> %d\n", self->wlan_flow_control, header->wireless_flow_control);
    }
    #endif
    self->wlan_flow_control = header->wireless_flow_control;

    if ((header->channel_and_flags & 0x0f) < 3) {
        // a valid header, check the bus data credit
        uint8_t tx_max = header->bus_data_credit;
        if ((uint8_t)(tx_max - self->wwd_sdpcm_packet_transmit_sequence_number) > 0x40U) {
            tx_max = (uint8_t)(self->wwd_sdpcm_packet_transmit_sequence_number + 2U);
        }
        self->wwd_sdpcm_last_bus_data_credit = tx_max;
    }

    if (header->size == SDPCM_HEADER_LEN) {
        // flow control packet with no data
        CYW43_DEBUG("Ignoring flow control packet\n");
        ap6256_cyw43_port_record_rx_frame(AP6256_CYW43_RX_CLASS_NONE,
                                          header->channel_and_flags & 0x0f,
                                          header->size,
                                          0U,
                                          buf);
        return -4;
    }

    switch (header->channel_and_flags & 0x0f) {
        case CONTROL_HEADER: {
            if (header->size < SDPCM_HEADER_LEN + IOCTL_HEADER_LEN) {
                // packet too small, just ignore it
                CYW43_DEBUG("Ignoring too small control packet\n");
                ap6256_cyw43_port_record_rx_frame(AP6256_CYW43_RX_CLASS_RUNT,
                                                  CONTROL_HEADER,
                                                  header->size,
                                                  0U,
                                                  buf);
                return -5;
            }
            const struct ioctl_header_t *ioctl_header = (const void *)&buf[header->header_length];
            if (((const uint8_t *)ioctl_header + IOCTL_HEADER_LEN) > ((const uint8_t *)header + header->size)) {
                CYW43_DEBUG("Ignoring malformed control packet\n");
                ap6256_cyw43_port_record_rx_frame(AP6256_CYW43_RX_CLASS_MALFORMED,
                                                  CONTROL_HEADER,
                                                  header->size,
                                                  0U,
                                                  buf);
                return -5;
            }

            // TODO need to handle errors and pass them up
            // if (ioctl_header->status != 0 || (ioctl_header->flags & 0xffff) != 0)
            // printf("CTRL HDR %lx %lx %lx %d\n", ioctl_header->cmd, ioctl_header->len, ioctl_header->flags, 2000 - ioctl_header->status);

            uint16_t id = (ioctl_header->flags & CDCF_IOC_ID_MASK) >> CDCF_IOC_ID_SHIFT;
            if (id != self->wwd_sdpcm_requested_ioctl_id) {
                // id doesn't match the last one sent, just ignore it
                CYW43_DEBUG("Ignoring packet with wrong id %d != %d\n", id, self->wwd_sdpcm_requested_ioctl_id);
                return -6;
            }
            if (ioctl_header->cmd != s_ap6256_pending_ioctl_cmd) {
                CYW43_DEBUG("Ignoring stale ioctl cmd %u != %u\n",
                            (unsigned int)ioctl_header->cmd,
                            (unsigned int)s_ap6256_pending_ioctl_cmd);
                return -6;
            }
            if (((ioctl_header->flags & CDCF_IOC_IF_MASK) >> CDCF_IOC_IF_SHIFT) !=
                (s_ap6256_pending_ioctl_iface & 0x0fU)) {
                CYW43_DEBUG("Ignoring ioctl iface %u != %u\n",
                            (unsigned int)((ioctl_header->flags & CDCF_IOC_IF_MASK) >> CDCF_IOC_IF_SHIFT),
                            (unsigned int)(s_ap6256_pending_ioctl_iface & 0x0fU));
                return -6;
            }
            if ((ioctl_header->flags & 0x3U) != (s_ap6256_pending_ioctl_kind & 0x3U)) {
                CYW43_DEBUG("Ignoring ioctl kind %u != %u\n",
                            (unsigned int)(ioctl_header->flags & 0x3U),
                            (unsigned int)(s_ap6256_pending_ioctl_kind & 0x3U));
                return -6;
            }
            // at this point the packet matches the last request sent and can be processed
            #pragma GCC diagnostic push
            #pragma GCC diagnostic ignored "-Wcast-qual"
            uint8_t *payload = (uint8_t *)ioctl_header + IOCTL_HEADER_LEN;
            size_t len = header->size - ((const uint8_t *)payload - (const uint8_t *)header);
            #pragma GCC diagnostic pop
            *out_len = len;
            *out_buf = payload;
            ap6256_cyw43_port_record_rx_frame(AP6256_CYW43_RX_CLASS_CONTROL,
                                              CONTROL_HEADER,
                                              header->size,
                                              (uint16_t)MIN(len, 0xffffU),
                                              payload);
            CYW43_VDEBUG("got ioctl response id=0x%x len=%d\n", id, len);
            return CONTROL_HEADER;
        }
        case DATA_HEADER: {
            if (header->size <= SDPCM_HEADER_LEN + BDC_HEADER_LEN) {
                // packet too small, just ignore it
                CYW43_DEBUG("Ignoring too small data packet\n");
                ap6256_cyw43_port_record_rx_frame(AP6256_CYW43_RX_CLASS_RUNT,
                                                  DATA_HEADER,
                                                  header->size,
                                                  0U,
                                                  buf);
                return -7;
            }
            // get bdc header
            const struct sdpcm_bdc_header_t *bdc_header = (const void *)&buf[header->header_length];
            if (((const uint8_t *)bdc_header + BDC_HEADER_LEN) > ((const uint8_t *)header + header->size)) {
                CYW43_DEBUG("Ignoring malformed data packet\n");
                ap6256_cyw43_port_record_rx_frame(AP6256_CYW43_RX_CLASS_MALFORMED,
                                                  DATA_HEADER,
                                                  header->size,
                                                  0U,
                                                  buf);
                return -7;
            }
            // get the interface number
            int itf = bdc_header->flags2;
            // get payload (skip variable length header)
            #pragma GCC diagnostic push
            #pragma GCC diagnostic ignored "-Wcast-qual"
            uint8_t *payload = (uint8_t *)bdc_header + BDC_HEADER_LEN + (bdc_header->data_offset << 2);
            if (payload > ((uint8_t *)header + header->size)) {
                CYW43_DEBUG("Ignoring malformed data payload\n");
                ap6256_cyw43_port_record_rx_frame(AP6256_CYW43_RX_CLASS_MALFORMED,
                                                  DATA_HEADER,
                                                  header->size,
                                                  0U,
                                                  buf);
                return -7;
            }
            size_t len = header->size - ((const uint8_t *)payload - (const uint8_t *)header);
            #pragma GCC diagnostic pop

            if (len < 42U) {
                /*
                 * Drop runt Ethernet payloads before they reach lwIP. ARP is
                 * 42 bytes without FCS; DHCP/EAPOL are larger. A shorter frame
                 * after association is malformed host input, and brcmfmac does
                 * not pass such packets up as valid netdev RX.
                 */
                CYW43_DEBUG("Ignoring runt data payload len=%u\n", (unsigned int)len);
                ap6256_cyw43_port_record_rx_frame(AP6256_CYW43_RX_CLASS_RUNT,
                                                  DATA_HEADER,
                                                  header->size,
                                                  (uint16_t)MIN(len, 0xffffU),
                                                  payload);
                return -7;
            }

            // at this point we have just the payload, ready to process
            *out_len = len | (uint32_t)itf << 31;
            *out_buf = payload;
            ap6256_cyw43_port_record_rx_frame(AP6256_CYW43_RX_CLASS_DATA,
                                              DATA_HEADER,
                                              header->size,
                                              (uint16_t)MIN(len, 0xffffU),
                                              payload);
            return DATA_HEADER;
        }
        case ASYNCEVENT_HEADER: {
            if (header->size <= SDPCM_HEADER_LEN + BDC_HEADER_LEN) {
                // packet too small, just ignore it
                CYW43_DEBUG("Ignoring too small async packet\n");
                ap6256_cyw43_port_record_rx_frame(AP6256_CYW43_RX_CLASS_RUNT,
                                                  ASYNCEVENT_HEADER,
                                                  header->size,
                                                  0U,
                                                  buf);
                return -8;
            }
            // get bdc header
            const struct sdpcm_bdc_header_t *bdc_header = (const void *)&buf[header->header_length];
            if (((const uint8_t *)bdc_header + BDC_HEADER_LEN) > ((const uint8_t *)header + header->size)) {
                CYW43_DEBUG("Ignoring malformed async packet\n");
                ap6256_cyw43_port_record_rx_frame(AP6256_CYW43_RX_CLASS_MALFORMED,
                                                  ASYNCEVENT_HEADER,
                                                  header->size,
                                                  0U,
                                                  buf);
                return -8;
            }
            // get payload (skip variable length header)
            #pragma GCC diagnostic push
            #pragma GCC diagnostic ignored "-Wcast-qual"
            uint8_t *payload = (uint8_t *)bdc_header + BDC_HEADER_LEN + (bdc_header->data_offset << 2);
            if (payload > ((uint8_t *)header + header->size)) {
                CYW43_DEBUG("Ignoring malformed async payload\n");
                ap6256_cyw43_port_record_rx_frame(AP6256_CYW43_RX_CLASS_MALFORMED,
                                                  ASYNCEVENT_HEADER,
                                                  header->size,
                                                  0U,
                                                  buf);
                return -8;
            }
            size_t len = header->size - ((const uint8_t *)payload - (const uint8_t *)header);
            #pragma GCC diagnostic pop

            if (len < 24U) {
                CYW43_DEBUG("Ignoring too small async ethernet event\n");
                ap6256_cyw43_port_record_rx_frame(AP6256_CYW43_RX_CLASS_RUNT,
                                                  ASYNCEVENT_HEADER,
                                                  header->size,
                                                  (uint16_t)MIN(len, 0xffffU),
                                                  payload);
                return -8;
            }

            // payload is actually an ethernet packet with type 0x886c
            if (!(payload[12] == 0x88 && payload[13] == 0x6c)) {
                // ethernet packet doesn't have the correct type
                // Note - this happens during startup but appears to be expected
                CYW43_VDEBUG("wrong payload type 0x%02x 0x%02x\n", payload[12], payload[13]);
                ap6256_cyw43_port_record_rx_frame(AP6256_CYW43_RX_CLASS_UNSUPPORTED,
                                                  ASYNCEVENT_HEADER,
                                                  header->size,
                                                  (uint16_t)MIN(len, 0xffffU),
                                                  payload);
                return CYW43_ERROR_WRONG_PAYLOAD_TYPE;
            }

            // check the Broadcom OUI
            if (!(payload[19] == 0x00 && payload[20] == 0x10 && payload[21] == 0x18)) {
                // incorrect OUI
                CYW43_DEBUG("incorrect oui\n");
                ap6256_cyw43_port_record_rx_frame(AP6256_CYW43_RX_CLASS_UNSUPPORTED,
                                                  ASYNCEVENT_HEADER,
                                                  header->size,
                                                  (uint16_t)MIN(len, 0xffffU),
                                                  payload);
                return -10;
            }

            // const struct wwd_event_header_t *event = (const void*)&payload[24];
            *out_len = len - 24;
            *out_buf = payload + 24;
            ap6256_cyw43_port_record_rx_frame(AP6256_CYW43_RX_CLASS_ASYNC,
                                              ASYNCEVENT_HEADER,
                                              header->size,
                                              (uint16_t)MIN(len - 24U, 0xffffU),
                                              payload + 24);

            CYW43_VDEBUG("async header header size=%d header length=%d out_len=%d\n", header->size, header->header_length, *out_len);
            return ASYNCEVENT_HEADER;
        }
        default: {
            // unknown header, just ignore it
            CYW43_DEBUG("unknown header\n");
            ap6256_cyw43_port_record_rx_frame(AP6256_CYW43_RX_CLASS_UNSUPPORTED,
                                              header->channel_and_flags & 0x0f,
                                              header->size,
                                              0U,
                                              buf);
            return -11;
        }
    }
}

#if !CYW43_USE_SPI // SDIO version follows

static bool cyw43_ll_sdio_packet_pending(cyw43_int_t *self) {
    static uint32_t s_ap6256_no_irq_probe_divider = 0U;
    uint8_t packet_pending = 0U;
    uint8_t pending_source = AP6256_CYW43_PACKET_SRC_NONE;
    uint32_t f1_int_status = 0U;
    bool sample_f1_mailbox = false;
    int dat1_level = cyw43_cb_read_host_interrupt_pin(self->cb_data);
    int cccr_int_pending = -1;
    int32_t status = 0;

    if (dat1_level == host_interrupt_pin_active) {
        packet_pending = 1U;
        pending_source = AP6256_CYW43_PACKET_SRC_DAT1;
        sample_f1_mailbox = true;
        s_ap6256_no_irq_probe_divider = 0U;
    } else {
        /*
         * DAT1 is the in-band interrupt signal. In the common no-work case,
         * avoid hammering CMD52 INTPEND reads every poll while still keeping a
         * sparse probe to catch missed edges.
         */
        if (++s_ap6256_no_irq_probe_divider < 16U) {
            ap6256_cyw43_port_record_packet_pending(0U,
                                                    AP6256_CYW43_PACKET_SRC_NONE,
                                                    (dat1_level >= 0) ? (uint8_t)dat1_level : 0xFFU,
                                                    0xFFU,
                                                    0U,
                                                    0);
            return false;
        }
        s_ap6256_no_irq_probe_divider = 0U;

    }

    cccr_int_pending = cyw43_read_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_INTPEND);
    if (cccr_int_pending < 0) {
        status = cccr_int_pending;
        pending_source = AP6256_CYW43_PACKET_SRC_ERROR;
    }

    if ((cccr_int_pending >= 0) && ((cccr_int_pending & SDIO_FUNC_ENABLE_2) != 0)) {
        packet_pending = 1U;
        pending_source = AP6256_CYW43_PACKET_SRC_CCCR_F2;
        sample_f1_mailbox = true;
    }

    if ((cccr_int_pending >= 0) && ((cccr_int_pending & SDIO_FUNC_ENABLE_1) != 0)) {
        sample_f1_mailbox = true;
    }

    /* Only sample F1 mailbox when DAT1/CCCR indicates possible work. */
    if (sample_f1_mailbox) {
        f1_int_status = cyw43_read_backplane(self, SDIO_INT_STATUS, 4);
        if ((f1_int_status & I_HMB_SW_MASK) != 0U) {
            packet_pending = 1U;
            pending_source = AP6256_CYW43_PACKET_SRC_F1_MAILBOX;
        }
    }

    if ((packet_pending == 0U) && (pending_source != AP6256_CYW43_PACKET_SRC_ERROR)) {
        pending_source = AP6256_CYW43_PACKET_SRC_NONE;
    }

    ap6256_cyw43_port_record_packet_pending(packet_pending,
                                            pending_source,
                                            (dat1_level >= 0) ? (uint8_t)dat1_level : 0xFFU,
                                            (cccr_int_pending >= 0) ? (uint8_t)cccr_int_pending : 0xFFU,
                                            f1_int_status,
                                            status);
    return packet_pending != 0U;
}

static int cyw43_ll_sdpcm_poll_device(cyw43_int_t *self, size_t *len, uint8_t **buf) {
    int ret;

    /*
     * Do not blindly read F2. On this STM32H7/AP6256 path, an F2 CMD53 read
     * with no pending SDPCM packet can timeout and hide the real state of the
     * firmware. Gate F2 reads on DAT1/CCCR/F1 mailbox status.
     */
    if (!self->had_successful_packet && !cyw43_ll_sdio_packet_pending(self)) {
        self->had_successful_packet = false;
        ap6256_cyw43_port_set_poll_diag(-1, 0, 0U, 0U);
        return -1;
    }

    cyw43_ll_bus_sleep((void *)self, false);

    #if CYW43_CLEAR_SDIO_INT
    if (!self->had_successful_packet) {
        // Clear interrupt status so that HOST_WAKE/SDIO line is cleared
        CYW43_VDEBUG("Reading SDIO_INT_STATUS\n");
        uint32_t int_status = cyw43_read_backplane(self, SDIO_INT_STATUS, 4);
        if (int_status & I_HMB_SW_MASK) {
            CYW43_STAT_INC(SDIO_INT_CLEAR);
            CYW43_VDEBUG("Clearing SDIO_INT_STATUS 0x%x\n", (int)(int_status & 0xf0));
            cyw43_write_backplane(self, SDIO_INT_STATUS, 4, int_status & 0xf0);
        }
    }
    #endif

    uint16_t hdr[2] = { 0U, 0U };
    ap6256_cyw43_port_set_bus_stage(101U);
    ap6256_checkpoint_record(AP6256_CYW43_CP_POLL_HEADER, WLAN_FUNCTION, 0U, 0U, 0U, 0);
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_IOCTL_WAIT,
                                        (int32_t)AP6256_CYW43_IOCTL_PHASE_WAIT_CMD53);
    ret = cyw43_read_bytes(self, WLAN_FUNCTION, 0, 4, (void *)hdr);
    ap6256_cyw43_port_set_poll_diag(ret, 0, hdr[0], hdr[1]);
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_IOCTL_WAIT, ret);
    if (ret != 0) {
        self->had_successful_packet = false;
        return ret;
    }
    if (hdr[0] == 0 && hdr[1] == 0) {
        // no packets
        self->had_successful_packet = false;
        return -1;
    }
    self->had_successful_packet = true;
    if ((hdr[0] ^ hdr[1]) != 0xffff) {
        CYW43_WARN("error: hdr mismatch %04x ^ %04x\n", hdr[0], hdr[1]);
        return -1;
    }
    if ((hdr[0] < SDPCM_HEADER_LEN) || (hdr[0] > sizeof(self->spid_buf))) {
        CYW43_WARN("error: invalid SDPCM length %u\n", (unsigned int)hdr[0]);
        self->had_successful_packet = false;
        return -3;
    }
    size_t sz_align = hdr[0] - 4;
    if (sz_align <= 8) {
        sz_align = (sz_align + 7) & ~7;
    } else if (sz_align <= 16) {
        sz_align = (sz_align + 15) & ~15;
    } else {
        sz_align = (sz_align + 63) & ~63;
    }
    if ((sz_align + 4U) > sizeof(self->spid_buf)) {
        CYW43_WARN("error: aligned SDPCM length too large %u\n", (unsigned int)(sz_align + 4U));
        self->had_successful_packet = false;
        return -3;
    }
    ap6256_cyw43_port_set_bus_stage(102U);
    ap6256_checkpoint_record(AP6256_CYW43_CP_POLL_PAYLOAD, WLAN_FUNCTION, 0U, (uint32_t)sz_align, hdr[0], 0);
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_IOCTL_WAIT,
                                        (int32_t)(0x10000U | (sz_align & 0xFFFFU)));
    ret = cyw43_read_bytes(self, WLAN_FUNCTION, 0, sz_align, self->spid_buf + 4);
    ap6256_cyw43_port_set_poll_diag(ap6256_cyw43_port_poll_header_read_status(), ret, hdr[0], hdr[1]);
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_IOCTL_WAIT, ret);
    if (ret != 0) {
        self->had_successful_packet = false;
        return ret;
    }
    memcpy(self->spid_buf, hdr, 4);

    ap6256_cyw43_port_set_bus_stage(103U);
    ap6256_checkpoint_record(AP6256_CYW43_CP_POLL_PARSE, WLAN_FUNCTION, 0U, hdr[0], hdr[1], 0);
    return sdpcm_process_rx_packet(self, self->spid_buf, len, buf);
}

#else // SPI version follows

static int cyw43_ll_sdpcm_poll_device(cyw43_int_t *self, size_t *len, uint8_t **buf) {
    // First check the SDIO interrupt line to see if the WLAN notified us
    if (!self->had_successful_packet && cyw43_cb_read_host_interrupt_pin(self->cb_data) != host_interrupt_pin_active) {
        return -1;
    }

    cyw43_ll_bus_sleep((void *)self, false);

    if (!self->had_successful_packet) {
        // Clear interrupt status so that HOST_WAKE/SDIO line is cleared
        static uint16_t last_spi_int;

        uint16_t spi_int = cyw43_read_reg_u16(self, BUS_FUNCTION, SPI_INTERRUPT_REGISTER);
        if (last_spi_int != spi_int) {
            if (spi_int & BUS_OVERFLOW_UNDERFLOW) {
                CYW43_WARN("Bus error condition detected 0x%x\n", spi_int);
                CYW43_STAT_INC(BUS_ERROR);
                #if CYW43_USE_STATS
                assert(CYW43_STAT_GET(BUS_ERROR) < 100); // stop eventually
                #endif
            }
        }

        #if CYW43_CLEAR_SDIO_INT
        uint32_t sdio_int = cyw43_read_backplane(self, SDIO_INT_STATUS, 4);
        if (sdio_int & I_HMB_SW_MASK) {

            uint16_t f1_info_reg = cyw43_read_reg_u16(self, BUS_FUNCTION, SPI_FUNCTION1_INFO);
            CYW43_STAT_INC(SDIO_INT_CLEAR);

            if ((f1_info_reg & SPI_FUNCTIONX_READY) == 0) {
                logic_debug_set(pin_F1_NOT_READY, 1);
                logic_debug_set(pin_F1_NOT_READY, 0);
            }

            cyw43_write_backplane(self, SDIO_INT_STATUS, 4, sdio_int & I_HMB_SW_MASK);

            // Check for bus error
            uint16_t check_spi_int = cyw43_read_reg_u16(self, BUS_FUNCTION, SPI_INTERRUPT_REGISTER);

            // F1 overflow might start here, read back and check the status
            uint32_t check_sdio_int = cyw43_read_backplane(self, SDIO_INT_STATUS, 4);
            if (!(spi_int & BUS_OVERFLOW_UNDERFLOW) && (check_spi_int & BUS_OVERFLOW_UNDERFLOW)) {
                CYW43_WARN("Bus error condition detected from 0x%x to 0x%x with sdio from 0x%x to 0x%x\n",
                    spi_int, check_spi_int, sdio_int, check_sdio_int);
            }

            spi_int = check_spi_int;
        }
        #endif
        if (spi_int) {
            CYW43_STAT_INC(SPI_INT_CLEAR);
            cyw43_write_reg_u16(self, BUS_FUNCTION, SPI_INTERRUPT_REGISTER, spi_int);
        }
        last_spi_int = spi_int;
        if (!(spi_int & F2_PACKET_AVAILABLE)) {
            return -1;
        }
    }
    CYW43_STAT_INC(SPI_PACKET_AVAILABLE);

    // See whd_bus_spi_read_frame
    #define PAYLOAD_MTU 1500
    #define LINK_HEADER 30
    #define ETHERNET_SIZE 14
    #define LINK_MTU PAYLOAD_MTU + LINK_HEADER + ETHERNET_SIZE
    #define GSPI_PACKET_OVERHEAD 8
    uint32_t bus_gspi_status = 0;

    for (int i = 0; i < 1000; ++i) {
        bus_gspi_status = cyw43_read_reg_u32(self, BUS_FUNCTION, SPI_STATUS_REGISTER);
        if (bus_gspi_status != 0xFFFFFFFF) {
            break;
        }
    }
    CYW43_VDEBUG("bus_gspi_status 0x%x 0x%x\n", bus_gspi_status, bus_gspi_status >> 9);
    if (bus_gspi_status == 0xFFFFFFFF) {
        return -1;
    }

    uint32_t bytes_pending = 0;
    if (bus_gspi_status & GSPI_PACKET_AVAILABLE) {
        bytes_pending = (bus_gspi_status >> 9) & 0x7FF;
        if (bytes_pending == 0 || bytes_pending > (LINK_MTU - GSPI_PACKET_OVERHEAD) ||
            bus_gspi_status & F2_F3_FIFO_RD_UNDERFLOW) {
            CYW43_DEBUG("SPI invalid bytes pending %" PRIu32 "\n", bytes_pending);
            cyw43_write_reg_u8(self, BACKPLANE_FUNCTION, SPI_FRAME_CONTROL, (1 << 0));
            self->had_successful_packet = false;
            return -1;
        }
    } else {
        // No packet
        CYW43_VDEBUG("No packet\n");
        self->had_successful_packet = false;
        return -1;
    }
    int ret = cyw43_read_bytes(self, WLAN_FUNCTION, 0, bytes_pending, self->spid_buf);
    if (ret != 0) {
        return ret;
    }
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wcast-align"
    uint16_t *hdr = (uint16_t *)self->spid_buf;
    #pragma GCC diagnostic pop
    if (hdr[0] == 0 && hdr[1] == 0) {
        // no packets
        CYW43_DEBUG("No packet zero size header\n");
        self->had_successful_packet = false;
        return -1;
    }
    self->had_successful_packet = true;
    if ((hdr[0] ^ hdr[1]) != 0xffff) {
        CYW43_WARN("error: hdr mismatch %04x ^ %04x\n", hdr[0], hdr[1]);
        return -1;
    }
    return sdpcm_process_rx_packet(self, self->spid_buf, len, buf);
}

#endif

void cyw43_ll_process_packets(cyw43_ll_t *self_in) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);
    uint32_t packets_processed = 0;

    for (;;) {
        size_t len;
        uint8_t *buf;
        int ret = cyw43_ll_sdpcm_poll_device(self, &len, &buf);
        if (ret == -1) {
            // no packet
            break;
        } else if (ret == -4) {
            // flow control
            packets_processed++;
            if (packets_processed >= 8U) {
                self->had_successful_packet = false;
                break;
            }
        } else if (ret == CONTROL_HEADER) {
            /*
             * Association and escan are event-driven on BCM43456/AP6256: the
             * synchronous BCDC completion can arrive after the ioctl caller has
             * intentionally returned to the higher-level wait loop. Drain that
             * stale control frame here so it cannot sit at the head of F2 and
             * starve AUTH/ASSOC/LINK/PSK or ESCAN events.
             */
            packets_processed++;
        } else if (ret == ASYNCEVENT_HEADER) {
            cyw43_async_event_t *ev = cyw43_ll_parse_async_event(len, buf);
            if (ev != NULL) {
                cyw43_cb_process_async_event(self, ev);
            }
            packets_processed++;
        } else if (ret == DATA_HEADER) {
            if (!ap6256_join_should_drop_prelink_data()) {
                cyw43_cb_process_ethernet(self->cb_data, len >> 31, len & 0x7fffffff, buf);
            }
            packets_processed++;
        } else if (CYW43_USE_SPI && ret == CYW43_ERROR_WRONG_PAYLOAD_TYPE) {
            // Ignore this error when using the SPI interface.  It can occur when there
            // is a lot of traffic over the SPI (eg sending UDP packets continuously)
            // and seems to be harmless.
            CYW43_VDEBUG("got wrong payload type for packet\n");
            packets_processed++;
        } else {
            CYW43_DEBUG("got unexpected packet %d\n", ret);
            /*
             * The upstream loop keeps polling forever here, which is fine when the
             * transport is known-good. On this STM32/AP6256 integration, SDIO bring-up
             * failures can return the same negative error repeatedly and wedge the
             * caller inside cyw43_poll_func(). Break out so higher layers can timeout
             * and surface diagnostics instead of hanging the interactive test path.
             */
            self->had_successful_packet = false;
            break;
        }

        if (packets_processed >= 32U) {
            break;
        }
    }
}

// will read the ioctl from buf
// will then write the result (max len bytes) into buf
static uint32_t ap6256_ioctl_phase_from_poll(int poll_result) {
    if (poll_result == -1) {
        return AP6256_CYW43_IOCTL_PHASE_WAIT_NO_PACKET;
    }
    if (poll_result == -CYW43_ETIMEDOUT) {
        return AP6256_CYW43_IOCTL_PHASE_WAIT_CMD53;
    }
    if (poll_result == -5 || poll_result == -7 || poll_result == -8 || poll_result == -10 || poll_result == -11) {
        return AP6256_CYW43_IOCTL_PHASE_MALFORMED;
    }
    if (poll_result == -6) {
        return AP6256_CYW43_IOCTL_PHASE_WRONG_ID;
    }
    if (poll_result < 0) {
        return AP6256_CYW43_IOCTL_PHASE_WAIT_CMD53;
    }
    return AP6256_CYW43_IOCTL_PHASE_WAIT_NO_PACKET;
}

static bool ap6256_ioctl_is_escan_start(uint32_t kind, uint32_t cmd, size_t len, const uint8_t *buf) {
    if ((kind != SDPCM_SET) || (cmd != WLC_SET_VAR) || (buf == NULL) || (len < 6U)) {
        return false;
    }
    return (memcmp(buf, "escan", 6U) == 0);
}

static int cyw43_do_ioctl(cyw43_int_t *self, uint32_t kind, uint32_t cmd, size_t len, uint8_t *buf, uint32_t iface) {
    const bool allow_scan_resend = ap6256_ioctl_is_escan_start(kind, cmd, len, buf);
    const bool is_join_iovar = ap6256_ioctl_is_join_iovar(kind, cmd, len, buf);
    const bool is_join_setup = ap6256_ioctl_is_join_setup(kind, cmd, len, buf);
    uint8_t recovery_attempted = 0U;
    uint8_t forced_probe_attempted = 0U;
    uint8_t forced_probe_pending = 0U;
    uint32_t start;
    uint32_t wait_loops;
    uint32_t no_packet_loops;
    int last_poll = 0;
    int ret;
    const bool is_association_trigger = ap6256_ioctl_is_association_trigger(kind, cmd) ||
        is_join_iovar;
    const bool is_association_start = ((kind == SDPCM_SET) &&
        ((cmd == WLC_SET_SSID) || is_join_iovar));
    uint32_t ioctl_wait_timeout_us = is_join_setup
        ? AP6256_CYW43_JOIN_SETUP_RESPONSE_WINDOW_US
        : is_association_trigger
        ? AP6256_CYW43_ASSOC_RESPONSE_WINDOW_US
        : CYW43_IOCTL_TIMEOUT_US;

    ap6256_cyw43_port_set_ioctl_attempt_flags(0U, 0U, 0U);
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_IOCTL_SEND,
                                        (int32_t)(((cmd & 0x3FFU) << 16U) | (kind & 0xFFFFU)));

    ap6256_cyw43_port_begin_ioctl(kind,
                                  cmd,
                                  iface,
                                  (uint32_t)len,
                                  (uint32_t)(self->wwd_sdpcm_requested_ioctl_id + 1U));
    ret = cyw43_send_ioctl(self, kind, cmd, len, buf, iface);
    ap6256_cyw43_port_begin_ioctl(kind,
                                  cmd,
                                  iface,
                                  (uint32_t)len,
                                  self->wwd_sdpcm_requested_ioctl_id);
    if (ret != 0) {
        if (ap6256_cyw43_port_last_ioctl_phase() != AP6256_CYW43_IOCTL_PHASE_SEND_CREDIT_TIMEOUT) {
            ap6256_cyw43_port_set_ioctl_phase(AP6256_CYW43_IOCTL_PHASE_SEND_FAIL);
        }
        ap6256_cyw43_port_finish_ioctl(ret, ret);
        ap6256_cyw43_port_set_ioctl_attempt_flags(recovery_attempted,
                                                  forced_probe_attempted,
                                                  0U);
        ap6256_cyw43_port_record_ioctl(kind,
                                       cmd,
                                       iface,
                                       (uint32_t)len,
                                       self->wwd_sdpcm_requested_ioctl_id,
                                       ret,
                                       ret);
        return ret;
    }

    /*
     * BCM43456/AP6256 association is event-driven in practice. Past HIL runs
     * show the target reset while the host is synchronously polling for the
     * WLC_SET_SSID response, after the F2 TX has already succeeded. Match the
     * brcmfmac-style connect flow here: once the association control frame is
     * accepted by SDPCM, return to the join state machine and require later
     * AUTH/ASSOC/LINK/PSK/GET_BSSID evidence before DHCP. Normal setup ioctls,
     * escan, and disassoc still use the bounded response wait below.
     */
    if (is_association_start) {
        ap6256_cyw43_port_set_ioctl_phase(AP6256_CYW43_IOCTL_PHASE_ACCEPTED_ASYNC);
        ap6256_cyw43_port_finish_ioctl(1, -1);
        ap6256_cyw43_port_set_ioctl_attempt_flags(recovery_attempted,
                                                  forced_probe_attempted,
                                                  0U);
        ap6256_cyw43_port_record_ioctl(kind,
                                       cmd,
                                       iface,
                                       (uint32_t)len,
                                       self->wwd_sdpcm_requested_ioctl_id,
                                       1,
                                       -1);
        self->had_successful_packet = false;
        return 0;
    }

    /*
     * A previous successful packet only proves the last F2 read was legal. It
     * must not authorize arbitrary control waits, so normal ioctls re-check
     * interrupt/mailbox state before completion reads. Association is different
     * on this AP6256 path: firmware often suppresses the sync response and the
     * useful evidence is subsequent event/data traffic. Preserve the post-TX
     * poll runway for WLC_SET_SSID/join so DHCP RX is not starved.
     */
    if (!is_association_trigger) {
        self->had_successful_packet = false;
    }

    start = cyw43_hal_ticks_us();
    wait_loops = 0U;
    no_packet_loops = 0U;
    last_poll = 0;
    forced_probe_pending = 0U;
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_IOCTL_WAIT,
                                        (int32_t)(((cmd & 0x3FFU) << 16U) |
                                                  (self->wwd_sdpcm_requested_ioctl_id & 0xFFFFU)));
    while (cyw43_hal_ticks_us() - start < ioctl_wait_timeout_us) {
        size_t res_len;
        uint8_t *res_buf;

        if (forced_probe_pending != 0U) {
            bool had_successful_packet = self->had_successful_packet;
            self->had_successful_packet = true;
            forced_probe_pending = 0U;
            ret = cyw43_ll_sdpcm_poll_device(self, &res_len, &res_buf);
            if (!had_successful_packet && (ret == -1)) {
                self->had_successful_packet = false;
            }
        } else {
            ret = cyw43_ll_sdpcm_poll_device(self, &res_len, &res_buf);
        }
        last_poll = ret;
        ap6256_cyw43_port_update_ioctl_poll(last_poll);
        if (ret == CONTROL_HEADER) {
            const struct ioctl_header_t *ioctl_header = (const void *)(res_buf - IOCTL_HEADER_LEN);
            if (ioctl_header->status != 0U) {
                ap6256_cyw43_port_set_ioctl_phase(AP6256_CYW43_IOCTL_PHASE_CONTROL_STATUS);
                ap6256_cyw43_port_finish_ioctl((int32_t)ioctl_header->status, last_poll);
                ap6256_cyw43_port_set_ioctl_attempt_flags(recovery_attempted,
                                                          forced_probe_attempted,
                                                          0U);
                ap6256_cyw43_port_record_ioctl(kind,
                                               cmd,
                                               iface,
                                               (uint32_t)len,
                                               self->wwd_sdpcm_requested_ioctl_id,
                                               (int32_t)ioctl_header->status,
                                               last_poll);
                return CYW43_FAIL_FAST_CHECK(-CYW43_EIO);
            }
            #if CYW43_USE_STATS
            uint32_t time_us = cyw43_hal_ticks_us() - start;
            if (time_us > CYW43_STAT_GET(LONGEST_IOCTL_TIME)) {
                CYW43_STAT_SET(LONGEST_IOCTL_TIME, time_us);
            }
            #endif
            // it seems that res_len is always the length of the argument in buf
            memmove(buf, res_buf, len < res_len ? len : res_len);
            ap6256_cyw43_port_set_ioctl_phase(AP6256_CYW43_IOCTL_PHASE_OK);
            ap6256_cyw43_port_finish_ioctl(0, last_poll);
            ap6256_cyw43_port_set_ioctl_attempt_flags(recovery_attempted,
                                                      forced_probe_attempted,
                                                      0U);
            ap6256_cyw43_port_record_ioctl(kind,
                                           cmd,
                                           iface,
                                           (uint32_t)len,
                                           self->wwd_sdpcm_requested_ioctl_id,
                                           0,
                                           last_poll);
            if (is_association_trigger) {
                self->had_successful_packet = true;
            }
            return 0;
        } else if (ret == ASYNCEVENT_HEADER) {
            cyw43_async_event_t *ev = cyw43_ll_parse_async_event(res_len, res_buf);
            if (ev != NULL) {
                cyw43_cb_process_async_event(self, ev);
            }
            if (is_association_trigger && ap6256_join_event_proves_started()) {
                ap6256_cyw43_port_set_ioctl_phase(AP6256_CYW43_IOCTL_PHASE_OK);
                ap6256_cyw43_port_finish_ioctl(0, last_poll);
                ap6256_cyw43_port_set_ioctl_attempt_flags(recovery_attempted,
                                                          forced_probe_attempted,
                                                          0U);
                ap6256_cyw43_port_record_ioctl(kind,
                                               cmd,
                                               iface,
                                               (uint32_t)len,
                                               self->wwd_sdpcm_requested_ioctl_id,
                                               0,
                                               last_poll);
                self->had_successful_packet = true;
                return 0;
            }
        } else if (ret == DATA_HEADER) {
            if (!ap6256_join_should_drop_prelink_data()) {
                cyw43_cb_process_ethernet(self->cb_data, res_len >> 31, res_len & 0x7fffffff, res_buf);
            }
        } else if (ret >= 0) {
            CYW43_WARN("do_ioctl: got unexpected packet %d\n", ret);
        }

        if (ret == -1) {
            no_packet_loops++;
            ap6256_cyw43_port_note_wait_no_packet();

            if (allow_scan_resend && (no_packet_loops >= AP6256_CYW43_ESCAN_NO_PACKET_BUDGET_LOOPS)) {
                break;
            }

            if (is_association_trigger &&
                (no_packet_loops >= AP6256_CYW43_ASSOC_NO_PACKET_BUDGET_LOOPS)) {
                break;
            }

            if (is_join_setup &&
                (no_packet_loops >= AP6256_CYW43_JOIN_SETUP_NO_PACKET_BUDGET_LOOPS)) {
                break;
            }

            if (!is_association_trigger &&
                !is_join_setup &&
                (recovery_attempted == 0U) &&
                (no_packet_loops >= AP6256_CYW43_IOCTL_NO_PACKET_RECOVERY_LOOPS)) {
                int wake_ret;

                recovery_attempted = 1U;
                ap6256_cyw43_port_note_wait_recovery();
                ap6256_cyw43_port_set_ioctl_attempt_flags(recovery_attempted,
                                                          forced_probe_attempted,
                                                          0U);
                wake_ret = ap6256_cyw43_scan_wake(self, CYW_INT_TO_LL(self));
                if (wake_ret != 0) {
                    ap6256_cyw43_port_set_ioctl_phase(AP6256_CYW43_IOCTL_PHASE_SCAN_WAKE);
                    ap6256_cyw43_port_finish_ioctl(wake_ret, wake_ret);
                    ap6256_cyw43_port_set_ioctl_attempt_flags(recovery_attempted,
                                                              forced_probe_attempted,
                                                              0U);
                    ap6256_cyw43_port_record_ioctl(kind,
                                                   cmd,
                                                   iface,
                                                   (uint32_t)len,
                                                   self->wwd_sdpcm_requested_ioctl_id,
                                                   wake_ret,
                                                   wake_ret);
                    return wake_ret;
                }

                (void)cyw43_ll_sdio_packet_pending(self);
                forced_probe_pending = 1U;
                if (forced_probe_attempted == 0U) {
                    forced_probe_attempted = 1U;
                    ap6256_cyw43_port_note_wait_forced_probe();
                }
                ap6256_cyw43_port_set_ioctl_attempt_flags(recovery_attempted,
                                                          forced_probe_attempted,
                                                          0U);
            }

            if (!is_association_trigger &&
                !is_join_setup &&
                (no_packet_loops >= AP6256_CYW43_IOCTL_NO_PACKET_BUDGET_LOOPS)) {
                break;
            }
        } else {
            no_packet_loops = 0U;
        }

        ap6256_cyw43_port_set_ioctl_phase(ap6256_ioctl_phase_from_poll(ret));
        if ((ret != -1) || ((wait_loops & 0x1FU) == 0U)) {
            ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_IOCTL_WAIT, ret);
        }
        if (++wait_loops > 6000U) {
            break;
        }
        CYW43_DO_IOCTL_WAIT;
    }

    /*
     * BCM43456/AP6256 behaves like the Linux FullMAC escan path in practice:
     * after the escan control frame is accepted on F2, the useful completion
     * evidence is the ESCAN_RESULT event stream. On this board the synchronous
     * BCDC completion packet is not reliably generated for escan. The runtime
     * now bounds scan wait with no-progress and quiet-complete rules, so this
     * event-driven acceptance is safe for both the production 512-byte F2
     * profile and the 64-byte scan-recovery profile.
     */
    if (allow_scan_resend &&
        (last_poll == -1)) {
        ap6256_cyw43_port_set_ioctl_phase(AP6256_CYW43_IOCTL_PHASE_ACCEPTED_ASYNC);
        ap6256_cyw43_port_finish_ioctl(1, last_poll);
        ap6256_cyw43_port_set_ioctl_attempt_flags(recovery_attempted,
                                                  forced_probe_attempted,
                                                  0U);
        ap6256_cyw43_port_record_ioctl(kind,
                                       cmd,
                                       iface,
                                       (uint32_t)len,
                                       self->wwd_sdpcm_requested_ioctl_id,
                                       1,
                                       last_poll);
        return 0;
    }

    /*
     * BCM43456/AP6256 does not always return a synchronous BCDC completion for
     * WLC_SET_SSID/join. We still run the short ioctl wait above so any late
     * control response or immediate AUTH/LINK event is drained. If the wait
     * ends with no packet, treat the transmitted association command as
     * "started" and let the higher-level join/DHCP state machine prove success.
     */
    if (is_association_trigger &&
        (last_poll == -1)) {
        ap6256_cyw43_port_set_ioctl_phase(AP6256_CYW43_IOCTL_PHASE_ACCEPTED_ASYNC);
        ap6256_cyw43_port_finish_ioctl(1, last_poll);
        ap6256_cyw43_port_set_ioctl_attempt_flags(recovery_attempted,
                                                  forced_probe_attempted,
                                                  0U);
        ap6256_cyw43_port_record_ioctl(kind,
                                       cmd,
                                       iface,
                                       (uint32_t)len,
                                       self->wwd_sdpcm_requested_ioctl_id,
                                       1,
                                       last_poll);
        /*
         * No control/event/data packet was actually received. Leaving
         * had_successful_packet set here lets the next poll bypass the hybrid
         * packet-pending gate and issue an F2 header read with no mailbox
         * evidence. On BCM43456/AP6256 5 GHz joins that illegal post-assoc F2
         * read is the reset signature. The join wait loop must revalidate DAT1,
         * CCCR INTPEND, or F1 mailbox before touching F2.
         */
        self->had_successful_packet = false;
        return 0;
    }

    if (is_join_setup &&
        (last_poll == -1)) {
        ap6256_cyw43_port_set_ioctl_phase(AP6256_CYW43_IOCTL_PHASE_WAIT_NO_PACKET);
        ap6256_cyw43_port_finish_ioctl(-CYW43_ETIMEDOUT, last_poll);
        ap6256_cyw43_port_set_ioctl_attempt_flags(recovery_attempted,
                                                  forced_probe_attempted,
                                                  0U);
        ap6256_cyw43_port_record_ioctl(kind,
                                       cmd,
                                       iface,
                                       (uint32_t)len,
                                       self->wwd_sdpcm_requested_ioctl_id,
                                       -CYW43_ETIMEDOUT,
                                       last_poll);
        return CYW43_FAIL_FAST_CHECK(-CYW43_ETIMEDOUT);
    }

    CYW43_WARN("do_ioctl(%u, %u, %u): timeout\n", (unsigned int)kind, (unsigned int)cmd, (unsigned int)len);
    ap6256_cyw43_port_set_ioctl_phase(ap6256_ioctl_phase_from_poll(last_poll));
    ap6256_cyw43_port_finish_ioctl(-CYW43_ETIMEDOUT, last_poll);
    ap6256_cyw43_port_set_ioctl_attempt_flags(recovery_attempted,
                                              forced_probe_attempted,
                                              0U);
    ap6256_cyw43_port_record_ioctl(kind,
                                   cmd,
                                   iface,
                                   (uint32_t)len,
                                   self->wwd_sdpcm_requested_ioctl_id,
                                   -CYW43_ETIMEDOUT,
                                   last_poll);
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_IOCTL_WAIT,
                                        -CYW43_ETIMEDOUT);
    return CYW43_FAIL_FAST_CHECK(-CYW43_ETIMEDOUT);
}

int cyw43_ll_ioctl(cyw43_ll_t *self_in, uint32_t cmd, size_t len, uint8_t *buf, uint32_t iface) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);
    return cyw43_do_ioctl(self, cmd & 1 ? SDPCM_SET : SDPCM_GET, cmd >> 1, len, buf, iface);
}

/*******************************************************************************/
// high-level stuff

#define CORE_WLAN_ARM (1)
#define CORE_SOCRAM (2)

typedef struct {
    uint16_t id;
    uint8_t rev;
    uint32_t base;
    uint32_t wrapbase;
} ap6256_ai_core_t;

typedef struct {
    uint32_t boot_mode;
    uint32_t cpu_core_id;
    uint32_t ram_core_id;
    uint32_t cpu_base;
    uint32_t ram_core_base;
    uint32_t cpu_wrapper;
    uint32_t ram_wrapper;
    uint32_t firmware_addr;
    uint32_t nvram_addr;
    uint32_t footer_addr;
    uint32_t reset_vector_addr;
    uint32_t reset_vector_value;
} ap6256_boot_descriptor_t;

static ap6256_boot_descriptor_t s_ap6256_boot_descriptor;

static uint32_t get_core_address(int core_id) {
    if (core_id == CORE_WLAN_ARM) {
        return WLAN_ARMCM3_BASE_ADDRESS + WRAPPER_REGISTER_OFFSET;
    } else if (core_id == CORE_SOCRAM) {
        return SOCSRAM_BASE_ADDRESS + WRAPPER_REGISTER_OFFSET;
    } else {
        return 0;
    }
}

static uint32_t ap6256_dmp_desc_type(uint32_t val) {
    uint32_t type = val & AP6256_DMP_DESC_TYPE_MSK;
    if ((type & ~AP6256_DMP_DESC_ADDRSIZE_GT32) == AP6256_DMP_DESC_ADDRESS) {
        type = AP6256_DMP_DESC_ADDRESS;
    }
    return type;
}

static uint32_t ap6256_dmp_get_desc(cyw43_int_t *self, uint32_t *eromaddr, uint32_t *type) {
    uint32_t val = cyw43_read_backplane(self, *eromaddr, 4);
    *eromaddr += 4U;
    if (type != NULL) {
        *type = ap6256_dmp_desc_type(val);
    }
    return val;
}

static int ap6256_dmp_get_regaddr(cyw43_int_t *self,
                                  uint32_t *eromaddr,
                                  uint32_t *regbase,
                                  uint32_t *wrapbase) {
    uint32_t desc = 0U;
    uint32_t val;
    uint32_t wraptype;
    uint32_t guard = 0U;

    *regbase = 0U;
    *wrapbase = 0U;

    val = ap6256_dmp_get_desc(self, eromaddr, &desc);
    if (desc == AP6256_DMP_DESC_MASTER_PORT) {
        wraptype = AP6256_DMP_SLAVE_TYPE_MWRAP;
    } else if (desc == AP6256_DMP_DESC_ADDRESS) {
        *eromaddr -= 4U;
        wraptype = AP6256_DMP_SLAVE_TYPE_SWRAP;
    } else {
        *eromaddr -= 4U;
        return -CYW43_EIO;
    }

    do {
        uint32_t sztype;
        uint32_t stype;

        do {
            val = ap6256_dmp_get_desc(self, eromaddr, &desc);
            if (desc == AP6256_DMP_DESC_EOT) {
                *eromaddr -= 4U;
                return -CYW43_EIO;
            }
            if (++guard > 96U) {
                return -CYW43_EIO;
            }
        } while ((desc != AP6256_DMP_DESC_ADDRESS) && (desc != AP6256_DMP_DESC_COMPONENT));

        if (desc == AP6256_DMP_DESC_COMPONENT) {
            *eromaddr -= 4U;
            return 0;
        }

        if ((val & AP6256_DMP_DESC_ADDRSIZE_GT32) != 0U) {
            (void)ap6256_dmp_get_desc(self, eromaddr, NULL);
        }

        sztype = (val & AP6256_DMP_SLAVE_SIZE_TYPE) >> AP6256_DMP_SLAVE_SIZE_TYPE_S;
        if (sztype == AP6256_DMP_SLAVE_SIZE_DESC) {
            uint32_t szdesc = ap6256_dmp_get_desc(self, eromaddr, NULL);
            if ((szdesc & AP6256_DMP_DESC_ADDRSIZE_GT32) != 0U) {
                (void)ap6256_dmp_get_desc(self, eromaddr, NULL);
            }
        }

        if ((sztype != AP6256_DMP_SLAVE_SIZE_4K) && (sztype != AP6256_DMP_SLAVE_SIZE_8K)) {
            continue;
        }

        stype = (val & AP6256_DMP_SLAVE_TYPE) >> AP6256_DMP_SLAVE_TYPE_S;
        if ((*regbase == 0U) && (stype == AP6256_DMP_SLAVE_TYPE_SLAVE)) {
            *regbase = val & AP6256_DMP_SLAVE_ADDR_BASE;
        }
        if ((*wrapbase == 0U) && (stype == wraptype)) {
            *wrapbase = val & AP6256_DMP_SLAVE_ADDR_BASE;
        }
    } while ((*regbase == 0U) || (*wrapbase == 0U));

    return 0;
}

static uint8_t ap6256_ai_scan_cores(cyw43_int_t *self, ap6256_ai_core_t *cores, uint8_t max_cores) {
    uint32_t eromaddr = cyw43_read_backplane(self, CHIPCOMMON_EROMPTR, 4);
    uint32_t desc_type = 0U;
    uint8_t count = 0U;
    uint32_t guard = 0U;

    if ((cores == NULL) || (max_cores == 0U) || (eromaddr == 0U) || (eromaddr == 0xFFFFFFFFU)) {
        return 0U;
    }

    while ((desc_type != AP6256_DMP_DESC_EOT) && (count < max_cores) && (++guard < 192U)) {
        uint32_t val = ap6256_dmp_get_desc(self, &eromaddr, &desc_type);
        uint16_t id;
        uint8_t rev;
        uint32_t nmw;
        uint32_t nsw;
        uint32_t base = 0U;
        uint32_t wrap = 0U;

        if ((val & AP6256_DMP_DESC_VALID) == 0U) {
            continue;
        }
        if (desc_type == AP6256_DMP_DESC_EMPTY) {
            continue;
        }
        if (desc_type != AP6256_DMP_DESC_COMPONENT) {
            continue;
        }

        id = (uint16_t)((val & AP6256_DMP_COMP_PARTNUM) >> AP6256_DMP_COMP_PARTNUM_S);
        val = ap6256_dmp_get_desc(self, &eromaddr, &desc_type);
        if ((val & AP6256_DMP_DESC_TYPE_MSK) != AP6256_DMP_DESC_COMPONENT) {
            break;
        }

        nmw = (val & AP6256_DMP_COMP_NUM_MWRAP) >> AP6256_DMP_COMP_NUM_MWRAP_S;
        nsw = (val & AP6256_DMP_COMP_NUM_SWRAP) >> AP6256_DMP_COMP_NUM_SWRAP_S;
        rev = (uint8_t)((val & AP6256_DMP_COMP_REVISION) >> AP6256_DMP_COMP_REVISION_S);
        if ((nmw + nsw) == 0U) {
            continue;
        }

        if (ap6256_dmp_get_regaddr(self, &eromaddr, &base, &wrap) != 0) {
            continue;
        }
        if ((base == 0U) || (wrap == 0U)) {
            continue;
        }

        cores[count].id = id;
        cores[count].rev = rev;
        cores[count].base = base;
        cores[count].wrapbase = wrap;
        count++;
    }

    return count;
}

static const ap6256_ai_core_t *ap6256_find_core(const ap6256_ai_core_t *cores, uint8_t count, uint16_t id) {
    for (uint8_t i = 0U; i < count; ++i) {
        if (cores[i].id == id) {
            return &cores[i];
        }
    }
    return NULL;
}

static int ap6256_build_boot_descriptor(cyw43_int_t *self,
                                        uint32_t requested_boot_mode,
                                        uint32_t nvram_padded_len,
                                        uint32_t reset_vector_value,
                                        ap6256_boot_descriptor_t *desc) {
    ap6256_ai_core_t cores[AP6256_AI_MAX_CORES];
    uint8_t count = ap6256_ai_scan_cores(self, cores, AP6256_AI_MAX_CORES);
    const ap6256_ai_core_t *cr4 = ap6256_find_core(cores, count, AP6256_BCMA_CORE_ARM_CR4);
    const ap6256_ai_core_t *cm3 = ap6256_find_core(cores, count, AP6256_BCMA_CORE_ARM_CM3);
    const ap6256_ai_core_t *socram = ap6256_find_core(cores, count, AP6256_BCMA_CORE_INTERNAL_MEM);
    const ap6256_ai_core_t *sysmem = ap6256_find_core(cores, count, AP6256_BCMA_CORE_SYS_MEM);
    const ap6256_ai_core_t *ram = (sysmem != NULL) ? sysmem : socram;
    uint32_t boot_mode = requested_boot_mode;
    const ap6256_ai_core_t *cpu = NULL;

    memset(desc, 0, sizeof(*desc));

    if ((boot_mode == AP6256_CYW43_BOOT_CR4_TCM) && (cr4 != NULL)) {
        cpu = cr4;
    } else if ((boot_mode == AP6256_CYW43_BOOT_CM3_SOCRAM) && (cm3 != NULL)) {
        cpu = cm3;
    } else if (cr4 != NULL) {
        cpu = cr4;
        boot_mode = AP6256_CYW43_BOOT_CR4_TCM;
    } else if (cm3 != NULL) {
        cpu = cm3;
        boot_mode = AP6256_CYW43_BOOT_CM3_SOCRAM;
    }

    if (cpu == NULL) {
        ap6256_checkpoint_record(AP6256_CYW43_CP_TOPOLOGY,
                                 BACKPLANE_FUNCTION,
                                 CHIPCOMMON_EROMPTR,
                                 requested_boot_mode,
                                 cyw43_read_backplane(self, CHIPCOMMON_EROMPTR, 4),
                                 -CYW43_EIO);
        return CYW43_FAIL_FAST_CHECK(-CYW43_EIO);
    }

    if ((boot_mode == AP6256_CYW43_BOOT_CM3_SOCRAM) && (ram == NULL)) {
        ap6256_checkpoint_record(AP6256_CYW43_CP_TOPOLOGY,
                                 BACKPLANE_FUNCTION,
                                 cpu->wrapbase,
                                 cpu->id,
                                 0U,
                                 -CYW43_EIO);
        return CYW43_FAIL_FAST_CHECK(-CYW43_EIO);
    }

    desc->boot_mode = boot_mode;
    desc->cpu_core_id = cpu->id;
    desc->cpu_base = cpu->base;
    desc->cpu_wrapper = cpu->wrapbase;
    desc->ram_core_id = (ram != NULL) ? ram->id : 0U;
    desc->ram_core_base = (ram != NULL) ? ram->base : 0U;
    desc->ram_wrapper = (ram != NULL) ? ram->wrapbase : 0U;
    desc->firmware_addr = (boot_mode == AP6256_CYW43_BOOT_CR4_TCM) ? AP6256_CYW43_TCM_RAM_BASE
                                                                   : AP6256_CYW43_RAM_BASE;
    desc->nvram_addr = desc->firmware_addr + CYW43_RAM_SIZE - 4U - nvram_padded_len;
    desc->footer_addr = desc->firmware_addr + CYW43_RAM_SIZE - 4U;
    desc->reset_vector_addr = AP6256_RESET_VECTOR_ADDRESS;
    desc->reset_vector_value = (boot_mode == AP6256_CYW43_BOOT_CR4_TCM) ? reset_vector_value : 0U;

    ap6256_cyw43_port_set_boot_mode(desc->boot_mode);
    ap6256_cyw43_port_set_ram_base_addr(desc->firmware_addr);
    ap6256_cyw43_port_set_boot_descriptor(desc->cpu_wrapper,
                                          desc->ram_wrapper,
                                          desc->firmware_addr,
                                          desc->nvram_addr,
                                          desc->footer_addr);
    ap6256_cyw43_port_set_boot_topology(desc->cpu_core_id,
                                        desc->ram_core_id,
                                        desc->reset_vector_addr,
                                        desc->reset_vector_value);
    s_ap6256_boot_descriptor = *desc;
    ap6256_checkpoint_record(AP6256_CYW43_CP_TOPOLOGY,
                             BACKPLANE_FUNCTION,
                             desc->cpu_wrapper + AI_RESETCTRL_OFFSET,
                             desc->cpu_core_id,
                             cyw43_read_backplane(self, desc->cpu_wrapper + AI_RESETCTRL_OFFSET, 4),
                             0);
    return 0;
}

static int ap6256_core_disable_wrapper(cyw43_int_t *self,
                                       uint32_t wrapper_addr,
                                       uint32_t prereset,
                                       uint32_t reset) {
    uint32_t reg = cyw43_read_backplane(self, wrapper_addr + AI_RESETCTRL_OFFSET, 4);

    if ((reg & AIRC_RESET) != 0U) {
        goto in_reset_configure;
    }

    cyw43_write_backplane(self, wrapper_addr + AI_IOCTRL_OFFSET, 4, prereset | SICF_FGC | SICF_CLOCK_EN);
    cyw43_read_backplane(self, wrapper_addr + AI_IOCTRL_OFFSET, 4);
    cyw43_write_backplane(self, wrapper_addr + AI_RESETCTRL_OFFSET, 4, AIRC_RESET);
    cyw43_delay_ms(1);

    for (int count = 0; count < 50; ++count) {
        reg = cyw43_read_backplane(self, wrapper_addr + AI_RESETCTRL_OFFSET, 4);
        if ((reg & AIRC_RESET) != 0U) {
            goto in_reset_configure;
        }
        cyw43_delay_ms(1);
    }

    return CYW43_FAIL_FAST_CHECK(-CYW43_EIO);

in_reset_configure:
    cyw43_write_backplane(self, wrapper_addr + AI_IOCTRL_OFFSET, 4, reset | SICF_FGC | SICF_CLOCK_EN);
    cyw43_read_backplane(self, wrapper_addr + AI_IOCTRL_OFFSET, 4);
    return 0;
}

static int ap6256_core_reset_wrapper(cyw43_int_t *self,
                                     uint32_t wrapper_addr,
                                     uint32_t prereset,
                                     uint32_t reset,
                                     uint32_t postreset) {
    int ret = ap6256_core_disable_wrapper(self, wrapper_addr, prereset, reset);
    if (ret != 0) {
        return ret;
    }

    for (int count = 0; count < 50; ++count) {
        uint32_t reg = cyw43_read_backplane(self, wrapper_addr + AI_RESETCTRL_OFFSET, 4);
        if ((reg & AIRC_RESET) == 0U) {
            break;
        }
        cyw43_write_backplane(self, wrapper_addr + AI_RESETCTRL_OFFSET, 4, 0U);
        cyw43_delay_ms(1);
    }

    cyw43_write_backplane(self, wrapper_addr + AI_IOCTRL_OFFSET, 4, postreset | SICF_CLOCK_EN);
    cyw43_read_backplane(self, wrapper_addr + AI_IOCTRL_OFFSET, 4);
    cyw43_delay_ms(1);
    return 0;
}

static int ap6256_halt_cpu_core(cyw43_int_t *self, const ap6256_boot_descriptor_t *desc) {
    if (desc->boot_mode == AP6256_CYW43_BOOT_CR4_TCM) {
        uint32_t ioctl = cyw43_read_backplane(self, desc->cpu_wrapper + AI_IOCTRL_OFFSET, 4);
        ioctl &= SICF_CPUHALT;
        return ap6256_core_reset_wrapper(self, desc->cpu_wrapper, ioctl, SICF_CPUHALT, SICF_CPUHALT);
    }

    return ap6256_core_disable_wrapper(self, desc->cpu_wrapper, 0U, 0U);
}

static int ap6256_release_cpu_core(cyw43_int_t *self, const ap6256_boot_descriptor_t *desc) {
    if (desc->boot_mode == AP6256_CYW43_BOOT_CR4_TCM) {
        cyw43_write_backplane(self, SDIO_INT_STATUS, 4, 0xFFFFFFFFU);
        return ap6256_core_reset_wrapper(self, desc->cpu_wrapper, SICF_CPUHALT, 0U, 0U);
    }

    return ap6256_core_reset_wrapper(self, desc->cpu_wrapper, 0U, 0U, 0U);
}

static void device_core_is_up_by_wrapper(cyw43_int_t *self, uint32_t wrapper_addr) {
    uint32_t reg = cyw43_read_backplane(self, wrapper_addr + AI_IOCTRL_OFFSET, 4);
    if ((reg & (SICF_FGC | SICF_CLOCK_EN)) != SICF_CLOCK_EN) {
        CYW43_WARN("core not up\n");
    }
    reg = cyw43_read_backplane(self, wrapper_addr + AI_RESETCTRL_OFFSET, 4);
    if (reg & AIRC_RESET) {
        CYW43_WARN("core not up\n");
    }
    // if we get here then the core is up
    CYW43_VDEBUG("core wrapper 0x%" PRIx32 " IS up\n", wrapper_addr);
}

static void ap6256_record_core_diag(cyw43_int_t *self) {
    int chip_clock = cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR);
    uint32_t sr_control1 = cyw43_read_backplane(self, CHIPCOMMON_SR_CONTROL1, 4);
    uint32_t cpu_wrapper = (s_ap6256_boot_descriptor.cpu_wrapper != 0U)
                               ? s_ap6256_boot_descriptor.cpu_wrapper
                               : get_core_address(CORE_WLAN_ARM);
    uint32_t ram_wrapper = (s_ap6256_boot_descriptor.ram_wrapper != 0U)
                               ? s_ap6256_boot_descriptor.ram_wrapper
                               : get_core_address(CORE_SOCRAM);
    uint32_t wlan_ioctrl = cyw43_read_backplane(self, cpu_wrapper + AI_IOCTRL_OFFSET, 4);
    uint32_t wlan_resetctrl = cyw43_read_backplane(self, cpu_wrapper + AI_RESETCTRL_OFFSET, 4);
    uint32_t socram_ioctrl = (ram_wrapper != 0U) ? cyw43_read_backplane(self, ram_wrapper + AI_IOCTRL_OFFSET, 4) : 0U;
    uint32_t socram_resetctrl = (ram_wrapper != 0U) ? cyw43_read_backplane(self, ram_wrapper + AI_RESETCTRL_OFFSET, 4) : 0U;

    ap6256_cyw43_port_set_core_diag((chip_clock >= 0) ? (uint8_t)chip_clock : 0U,
                                    sr_control1,
                                    wlan_ioctrl,
                                    wlan_resetctrl,
                                    socram_ioctrl,
                                    socram_resetctrl);
}

static uint32_t ap6256_checkpoint_result_from_status(int32_t status) {
    if (status == 0) {
        return AP6256_CYW43_CP_RESULT_OK;
    }
    if (status == -CYW43_ETIMEDOUT) {
        return AP6256_CYW43_CP_RESULT_TIMEOUT;
    }
    return AP6256_CYW43_CP_RESULT_FAIL;
}

static void ap6256_checkpoint_record(uint32_t checkpoint,
                                     uint8_t function,
                                     uint32_t address,
                                     uint32_t write_value,
                                     uint32_t readback_value,
                                     int32_t status) {
    ap6256_cyw43_port_set_checkpoint(checkpoint,
                                     ap6256_checkpoint_result_from_status(status),
                                     function,
                                     address,
                                     write_value,
                                     readback_value,
                                     status);
}

static void ap6256_stage5_capture_snapshot(cyw43_int_t *self) {
    ap6256_cyw43_port_set_stage5_snapshot((uint8_t)cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_WAKEUP_CTRL),
                                          (uint8_t)cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_SLEEP_CSR),
                                          (uint8_t)cyw43_read_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_BRCM_CARDCAP),
                                          (uint8_t)cyw43_read_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_IORDY));
}

static int ap6256_verify_resource_words(cyw43_int_t *self,
                                        uint32_t checkpoint,
                                        uint32_t dest_addr,
                                        const uint8_t *source,
                                        size_t len) {
    size_t offsets[3];
    size_t offset_count = 0U;

    if ((source == NULL) || (len < 4U)) {
        ap6256_checkpoint_record(checkpoint,
                                 BACKPLANE_FUNCTION,
                                 dest_addr,
                                 0U,
                                 0U,
                                 -CYW43_EINVAL);
        return CYW43_FAIL_FAST_CHECK(-CYW43_EINVAL);
    }

    offsets[offset_count++] = 0U;
    if (len > 12U) {
        offsets[offset_count++] = (len / 2U) & ~(size_t)3U;
    }
    if (len > 4U) {
        size_t tail = (len - 4U) & ~(size_t)3U;
        if ((offset_count == 0U) || (tail != offsets[offset_count - 1U])) {
            offsets[offset_count++] = tail;
        }
    }

    for (size_t i = 0U; i < offset_count; ++i) {
        size_t off = offsets[i];
        uint32_t expected = cyw43_get_le32(source + off);
        uint32_t actual = cyw43_read_backplane(self, dest_addr + (uint32_t)off, 4);

        if (actual != expected) {
            ap6256_cyw43_port_set_verify_mismatch(dest_addr + (uint32_t)off, expected, actual);
            ap6256_checkpoint_record(checkpoint,
                                     BACKPLANE_FUNCTION,
                                     dest_addr + (uint32_t)off,
                                     expected,
                                     actual,
                                     -CYW43_EIO);
            return CYW43_FAIL_FAST_CHECK(-CYW43_EIO);
        }
    }

    ap6256_cyw43_port_set_verify_mismatch(0U, 0U, 0U);
    ap6256_checkpoint_record(checkpoint,
                             BACKPLANE_FUNCTION,
                             dest_addr + (uint32_t)offsets[offset_count - 1U],
                             cyw43_get_le32(source + offsets[offset_count - 1U]),
                             cyw43_get_le32(source + offsets[offset_count - 1U]),
                             0);
    return 0;
}

static int ap6256_verify_backplane_word(cyw43_int_t *self,
                                        uint32_t checkpoint,
                                        uint32_t addr,
                                        uint32_t expected) {
    uint32_t actual = cyw43_read_backplane(self, addr, 4);

    if (actual != expected) {
        ap6256_cyw43_port_set_verify_mismatch(addr, expected, actual);
        ap6256_checkpoint_record(checkpoint,
                                 BACKPLANE_FUNCTION,
                                 addr,
                                 expected,
                                 actual,
                                 -CYW43_EIO);
        return CYW43_FAIL_FAST_CHECK(-CYW43_EIO);
    }

    ap6256_cyw43_port_set_verify_mismatch(0U, 0U, 0U);
    ap6256_checkpoint_record(checkpoint,
                             BACKPLANE_FUNCTION,
                             addr,
                             expected,
                             actual,
                             0);
    return 0;
}

static int ap6256_checkpoint_write_u8(cyw43_int_t *self,
                                      uint32_t checkpoint,
                                      uint8_t function,
                                      uint32_t address,
                                      uint8_t write_value,
                                      uint8_t expect_mask,
                                      uint8_t expect_value) {
    cyw43_write_reg_u8(self, function, address, write_value);
    uint8_t readback = cyw43_read_reg_u8(self, function, address);
    int32_t status = (((uint32_t)readback & expect_mask) == expect_value) ? 0 : -CYW43_EIO;

    ap6256_checkpoint_record(checkpoint,
                             function,
                             address,
                             write_value,
                             readback,
                             status);
    if (status != 0) {
        ap6256_record_core_diag(self);
    }
    return status;
}

static int ap6256_checkpoint_write_backplane(cyw43_int_t *self,
                                             uint32_t checkpoint,
                                             uint32_t address,
                                             uint32_t write_value,
                                             uint32_t expect_mask,
                                             uint32_t expect_value) {
    cyw43_write_backplane(self, address, 4, write_value);
    uint32_t readback = cyw43_read_backplane(self, address, 4);
    int32_t status = ((readback & expect_mask) == expect_value) ? 0 : -CYW43_EIO;
    ap6256_cyw43_port_record_backplane_access(1U, address, 4U, status);

    ap6256_checkpoint_record(checkpoint,
                             BACKPLANE_FUNCTION,
                             address,
                             write_value,
                             readback,
                             status);
    if (status != 0) {
        ap6256_record_core_diag(self);
    }
    return status;
}

static int ap6256_validate_supported_chip(uint32_t chip_id_raw) {
    if (ap6256_cyw43_chip_supported(chip_id_raw) != 0U) {
        return 0;
    }

    CYW43_WARN("unsupported AP6256 chip raw=0x%08" PRIx32 " id=0x%04x rev=%u\n",
               chip_id_raw,
               ap6256_cyw43_chip_id_from_raw(chip_id_raw),
               ap6256_cyw43_chip_rev_from_raw(chip_id_raw));
    return CYW43_FAIL_FAST_CHECK(-CYW43_EPERM);
}

#if USE_KSO
// KSO mode (keep SDIO on)
static int cyw43_kso_set(cyw43_int_t *self, int value) {
    uint8_t write_value = 0;
    if (value) {
        write_value = SBSDIO_SLPCSR_KEEP_SDIO_ON;
    }
    // these can fail and it's still ok
    cyw43_write_reg_u8(self, BACKPLANE_FUNCTION, SDIO_SLEEP_CSR, write_value);
    cyw43_write_reg_u8(self, BACKPLANE_FUNCTION, SDIO_SLEEP_CSR, write_value);

    uint8_t compare_value, bmask;
    if (value) {
        // device WAKEUP through KSO:
        // write bit 0 & read back until
        // both bits 0(kso bit) & 1 (dev on status) are set
        compare_value = SBSDIO_SLPCSR_KEEP_SDIO_ON | SBSDIO_SLPCSR_DEVICE_ON;
        bmask = compare_value;
    } else {
        // Put device to sleep, turn off KSO
        compare_value = 0;
        // Check for bit0 only, bit1(devon status) may not get cleared right away
        bmask = SBSDIO_SLPCSR_KEEP_SDIO_ON;
    }

    for (int i = 0; i < 64; ++i) {
        // Reliable KSO bit set/clr:
        // Sdiod sleep write access appears to be in sync with PMU 32khz clk
        // just one write attempt may fail, (same is with read ?)
        // in any case, read it back until it matches written value
        // this can fail and it's still ok
        int read_value = cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_SLEEP_CSR);
        if (read_value >= 0 && ((read_value & bmask) == compare_value) && read_value != 0xff) {
            ap6256_cyw43_port_set_kso_status(0);
            return 0; // success
        }

        cyw43_delay_ms(1);

        cyw43_write_reg_u8(self, BACKPLANE_FUNCTION, SDIO_SLEEP_CSR, write_value);
    }

    CYW43_WARN("cyw43_kso_set(%d): failed\n", value);
    ap6256_cyw43_port_set_kso_status(-CYW43_ETIMEDOUT);
    return -CYW43_ETIMEDOUT;
}
#endif

static void cyw43_ll_bus_sleep_helper(cyw43_int_t *self, bool can_sleep) {
    #if USE_KSO

    (void)cyw43_kso_set(self, !can_sleep);

    #else

    if (can_sleep) {
        // clear request for HT
        cyw43_write_reg(self, BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR, 1, 0);
        return;
    }

    // make sure HT is available
    cyw43_write_reg_u8(self, BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR, SBSDIO_HT_AVAIL_REQ);
    // cyw43_write_reg_u8(self, BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR, SBSDIO_HT_AVAIL_REQ);
    for (int i = 0; i < 1000; ++i) {
        int reg = cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR);
        if (reg >= 0 && (reg & SBSDIO_HT_AVAIL)) {
            return;
        }
        cyw43_delay_ms(1);
        // cyw43_write_reg_u8(self, BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR, SBSDIO_HT_AVAIL_REQ);
    }

    CYW43_WARN("could not bring bus up\n");

    #endif
}

void cyw43_ll_bus_sleep(cyw43_ll_t *self_in, bool can_sleep) {
    cyw43_int_t *self = (void *)self_in;
    if (can_sleep) {
        if (!self->bus_is_up) {
            return;
        }
        CYW43_STAT_INC(SLEEP_COUNT);
        CYW43_VDEBUG("sleep bus\n");
        self->bus_is_up = false;
        cyw43_ll_bus_sleep_helper(self, true);
    } else {
        cyw43_cb_ensure_awake(self);
        if (self->bus_is_up) {
            return;
        }
        CYW43_STAT_INC(WAKE_COUNT);
        CYW43_VDEBUG("wake bus\n");
        cyw43_ll_bus_sleep_helper(self, false);
        self->bus_is_up = true;
    }
}

#if CYW43_USE_SPI
#define CLM_CHUNK_LEN 1024
#else
#define CLM_CHUNK_LEN 1024 + 512
#endif

static void cyw43_clm_load(cyw43_int_t *self, const uint8_t *clm_ptr, size_t clm_len) {
    // Reuse spid_buf but be careful to start at the right offset in it
    uint8_t *buf = &self->spid_buf[SDPCM_HEADER_LEN + 16];

    const size_t clm_dload_chunk_len = CLM_CHUNK_LEN;
    for (size_t off = 0; off < clm_len; off += clm_dload_chunk_len) {
        CYW43_EVENT_POLL_HOOK;

        uint32_t len = clm_dload_chunk_len;
        uint16_t flag = 1 << 12; // DLOAD_HANDLER_VER
        if (off == 0) {
            flag |= 2; // DL_BEGIN
        }
        if (off + len >= clm_len) {
            flag |= 4; // DL_END
            len = clm_len - off;
        }
        memcpy(buf, "clmload\x00", 8);
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wcast-align"
        static_assert(!(offsetof(cyw43_int_t, spid_buf) & 3), "");
        *(uint16_t *)(buf + 8) = flag;
        *(uint16_t *)(buf + 10) = 2;
        *(uint32_t *)(buf + 12) = len;
        *(uint32_t *)(buf + 16) = 0;
        #pragma GCC diagnostic pop
        memcpy(buf + 20, clm_ptr + off, len);

        CYW43_VDEBUG("clm data send %u/%u\n", off + len, clm_len);

        // Send data aligned to 8 bytes; padding comes from junk at end of buf
        cyw43_do_ioctl(self, SDPCM_SET, WLC_SET_VAR, ALIGN_UINT(20 + len, 8), buf, WWD_STA_INTERFACE);
    }

    CYW43_VDEBUG("clm data send done\n");
    // Check the status of the download
    memcpy(buf, "clmload_status\x00\x00\x00\x00\x00", 19);
    cyw43_do_ioctl(self, SDPCM_GET, WLC_GET_VAR, 19, buf, WWD_STA_INTERFACE);
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wcast-align"
    if (*(uint32_t *)buf != 0) {
        CYW43_WARN("CLM load failed\n");
    }
    #pragma GCC diagnostic pop
    CYW43_VDEBUG("clm data load ok\n");
}

static int cyw43_write_iovar_u32(cyw43_int_t *self, const char *var, uint32_t val, uint32_t iface) {
    uint8_t *buf = &self->spid_buf[SDPCM_HEADER_LEN + 16];
    size_t len = strlen(var) + 1;
    memcpy(buf, var, len);
    cyw43_put_le32(buf + len, val);
    return cyw43_do_ioctl(self, SDPCM_SET, WLC_SET_VAR, len + 4, buf, iface);
}

static int cyw43_write_iovar_u32_u32(cyw43_int_t *self, const char *var, uint32_t val0, uint32_t val1, uint32_t iface) {
    uint8_t *buf = &self->spid_buf[SDPCM_HEADER_LEN + 16];
    size_t len = strlen(var) + 1;
    memcpy(buf, var, len);
    cyw43_put_le32(buf + len, val0);
    cyw43_put_le32(buf + len + 4, val1);
    return cyw43_do_ioctl(self, SDPCM_SET, WLC_SET_VAR, len + 8, buf, iface);
}

// buf may point anywhere in self->spid_buf (or elsewhere)
static int cyw43_write_iovar_n(cyw43_int_t *self, const char *var, size_t len, const void *buf, uint32_t iface) {
    int ret;
    uint8_t *iobuf = &self->spid_buf[SDPCM_HEADER_LEN + 16];
    size_t varlen = strlen(var) + 1;
    memmove(iobuf + varlen, buf, len);
    memcpy(iobuf, var, varlen);
    if (strcmp(var, "escan") == 0) {
        ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_ESCAN_IOVAR,
                                            (int32_t)(varlen + len));
    }
    ret = cyw43_do_ioctl(self, SDPCM_SET, WLC_SET_VAR, varlen + len, iobuf, iface);
    if (strcmp(var, "escan") == 0) {
        ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_ESCAN_IOVAR, ret);
    }
    return ret;
}

int cyw43_ll_bus_init(cyw43_ll_t *self_in, const uint8_t *mac) {
    cyw43_int_t *self = (void *)self_in;
    const ap6256_embedded_asset_t *fw_asset = ap6256_assets_wifi_firmware();
    const ap6256_embedded_asset_t *clm_asset = ap6256_assets_wifi_clm_blob();
    /*
     * Default runtime uses the AP6256-specific NVRAM. The existing hidden
     * toggle now means "force generic fallback" for bench A/B testing.
     */
    const ap6256_embedded_asset_t *nvram_asset = (ap6256_cyw43_port_reference_nvram_enabled() != 0U)
                                                      ? ap6256_assets_wifi_nvram()
                                                      : ap6256_assets_reference_nvram();
    const uint8_t *fw_data;
    const uint8_t *clm_data;
    const uint8_t *wifi_nvram_data;
    static uint8_t s_ap6256_nvram_blob[AP6256_NVRAM_BLOB_MAX_LEN];
    size_t fw_len;
    size_t clm_len;
    size_t wifi_nvram_len_unpadded;
    size_t wifi_nvram_len;
    uint32_t chip_id_raw = 0U;
    uint32_t nvram_footer = 0U;
    uint32_t compat_profile = ap6256_cyw43_port_profile();
    uint32_t compat_boot_mode = ap6256_cyw43_profile_boot_mode(compat_profile);
    uint32_t compat_ram_base = ap6256_cyw43_profile_ram_base(compat_profile);
    uint32_t compat_fw_addr = compat_ram_base;
    uint32_t compat_nvram_addr = 0U;
    uint32_t compat_footer_addr = 0U;
    uint32_t compat_cpu_wrapper = get_core_address(CORE_WLAN_ARM);
    uint32_t compat_ram_wrapper = get_core_address(CORE_SOCRAM);
    uint32_t compat_reset_vector_value = 0U;
    ap6256_boot_descriptor_t compat_desc;
    int ret;

    memset(&compat_desc, 0, sizeof(compat_desc));
    s_ap6256_boot_descriptor = compat_desc;
    s_ap6256_scan_sync_id = 0U;
    s_ap6256_scan_active = 0U;
    self->startup_t0 = cyw43_hal_ticks_us();
    ap6256_cyw43_port_set_profile(compat_profile);
    ap6256_cyw43_port_set_boot_mode(compat_boot_mode);
    ap6256_cyw43_port_set_bus_stage(1U);
    ap6256_cyw43_port_set_ram_base_addr(compat_ram_base);
    ap6256_cyw43_port_set_ram_size_bytes(CYW43_RAM_SIZE);

    if ((fw_asset == NULL) || (clm_asset == NULL) || (nvram_asset == NULL) ||
        (fw_asset->data == NULL) || (clm_asset->data == NULL) || (nvram_asset->data == NULL) ||
        (fw_asset->size == 0U) || (clm_asset->size == 0U) || (nvram_asset->size == 0U)) {
        CYW43_WARN("required AP6256 assets are missing\n");
        return CYW43_FAIL_FAST_CHECK(-CYW43_EIO);
    }

    fw_data = fw_asset->data;
    clm_data = clm_asset->data;
    fw_len = fw_asset->size;
    clm_len = clm_asset->size;
    if (fw_len >= 4U) {
        compat_reset_vector_value = cyw43_get_le32(fw_data);
    }
    wifi_nvram_len_unpadded = ap6256_prepare_nvram_blob(nvram_asset->data,
                                                        nvram_asset->size,
                                                        s_ap6256_nvram_blob,
                                                        sizeof(s_ap6256_nvram_blob));
    if (wifi_nvram_len_unpadded == 0U) {
        CYW43_WARN("failed to prepare AP6256 NVRAM blob\n");
        return CYW43_FAIL_FAST_CHECK(-CYW43_EIO);
    }
    wifi_nvram_len = CYW43_WRITE_BYTES_PAD(wifi_nvram_len_unpadded);
    if (wifi_nvram_len > sizeof(s_ap6256_nvram_blob)) {
        CYW43_WARN("prepared AP6256 NVRAM blob does not fit padded buffer\n");
        return CYW43_FAIL_FAST_CHECK(-CYW43_EINVAL);
    }
    memset(&s_ap6256_nvram_blob[wifi_nvram_len_unpadded], 0, wifi_nvram_len - wifi_nvram_len_unpadded);
    ret = ap6256_validate_nvram_blob(s_ap6256_nvram_blob,
                                     wifi_nvram_len_unpadded,
                                     wifi_nvram_len,
                                     &nvram_footer);
    if (ret != 0) {
        CYW43_WARN("invalid AP6256 NVRAM blob\n");
        return ret;
    }
    ap6256_cyw43_port_set_nvram_metrics((uint32_t)wifi_nvram_len_unpadded,
                                        (uint32_t)wifi_nvram_len,
                                        nvram_footer,
                                        (ap6256_cyw43_port_reference_nvram_enabled() == 0U) ? 1U : 0U);
    compat_nvram_addr = compat_ram_base + CYW43_RAM_SIZE - 4U - (uint32_t)wifi_nvram_len;
    compat_footer_addr = compat_ram_base + CYW43_RAM_SIZE - 4U;
    ap6256_cyw43_port_set_boot_descriptor(compat_cpu_wrapper,
                                          compat_ram_wrapper,
                                          compat_fw_addr,
                                          compat_nvram_addr,
                                          compat_footer_addr);
    ap6256_cyw43_port_set_boot_topology((compat_boot_mode == AP6256_CYW43_BOOT_CR4_TCM)
                                            ? AP6256_BCMA_CORE_ARM_CR4
                                            : AP6256_BCMA_CORE_ARM_CM3,
                                        (compat_boot_mode == AP6256_CYW43_BOOT_CR4_TCM)
                                            ? 0U
                                            : AP6256_BCMA_CORE_INTERNAL_MEM,
                                        AP6256_RESET_VECTOR_ADDRESS,
                                        compat_reset_vector_value);
    wifi_nvram_data = s_ap6256_nvram_blob;

    #if CYW43_USE_SPI

    bool success = false;
    do {
        uint32_t val = WORD_LENGTH_32 | ENDIAN_BIG | HIGH_SPEED_MODE | WAKE_UP |
            0x4 << (8 * SPI_RESPONSE_DELAY) | INTR_WITH_STATUS << (8 * SPI_STATUS_ENABLE);
        #if defined(CYW43_PIN_WL_HOST_WAKE)
        val |= INTERRUPT_POLARITY_HIGH;
        #endif

        // Initialise
        if (cyw43_spi_init(self) != 0) {
            CYW43_DEBUG("Failed to initialise cyw43\n");
            break;
        }

        cyw43_spi_gpio_setup();
        CYW43_EVENT_POLL_HOOK;
        cyw43_spi_reset();
        CYW43_EVENT_POLL_HOOK;

        // Check test register can be read
        for (int i = 0; i < 10; ++i) {
            uint32_t reg = read_reg_u32_swap(self, BUS_FUNCTION, SPI_READ_TEST_REGISTER);
            if (reg == TEST_PATTERN) {
                goto chip_up;
            }
            cyw43_delay_ms(1);
        }
        CYW43_DEBUG("Failed to read test pattern\n");
        break; // failed
    chip_up:
        // Switch to 32bit mode
        CYW43_VDEBUG("setting SPI_BUS_CONTROL 0x%x\n", val);

        if (write_reg_u32_swap(self, BUS_FUNCTION, SPI_BUS_CONTROL, val) != 0) {
            break;
        }

        val = cyw43_read_reg_u32(self, BUS_FUNCTION, SPI_BUS_CONTROL);
        CYW43_VDEBUG("read SPI_BUS_CONTROL 0x%x\n", val);

        if (cyw43_write_reg_u8(self, BUS_FUNCTION, SPI_RESP_DELAY_F1, CYW43_BACKPLANE_READ_PAD_LEN_BYTES) != 0) {
            break;
        }

        // Make sure error interrupt bits are clear
        if (cyw43_write_reg_u8(self, BUS_FUNCTION, SPI_INTERRUPT_REGISTER,
            DATA_UNAVAILABLE | COMMAND_ERROR | DATA_ERROR | F1_OVERFLOW) != 0) {
            break;
        }

        // Enable a selection of interrupts
        uint16_t cyw43_interrupts = F2_F3_FIFO_RD_UNDERFLOW | F2_F3_FIFO_WR_OVERFLOW |
            COMMAND_ERROR | DATA_ERROR | F2_PACKET_AVAILABLE | F1_OVERFLOW;
        #if CYW43_ENABLE_BLUETOOTH
        cyw43_interrupts |= F1_INTR;
        #endif
        if (cyw43_write_reg_u16(self, BUS_FUNCTION, SPI_INTERRUPT_ENABLE_REGISTER, cyw43_interrupts) != 0) {
            break;
        }

        success = true;
    } while (false);
    if (!success) {
        CYW43_WARN("Failed to start CYW43\n");
        return CYW43_FAIL_FAST_CHECK(-CYW43_EIO);
    }

    #else

    // enumerate SDIO bus
    cyw43_sdio_transfer(0, 0, NULL); // ignore any errors
    cyw43_sdio_transfer(5, 0, NULL); // ignore any errors

    // get RCA
    uint32_t rca;
    if (cyw43_sdio_transfer(3, 0, &rca) != 0) {
        CYW43_WARN("SDIO enumerate error\n");
        return CYW43_FAIL_FAST_CHECK(-CYW43_EIO);
    }

    // select the card with RCA
    if (cyw43_sdio_transfer(7, rca, NULL) != 0) {
        CYW43_WARN("SDIO select error\n");
        return CYW43_FAIL_FAST_CHECK(-CYW43_EIO);
    }

    // set up backplane
    for (int i = 0; i < 100; ++i) {
        cyw43_write_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_IOEN, SDIO_FUNC_ENABLE_1);
        if (i != 0) {
            cyw43_delay_ms(1);
        }
        uint32_t reg = cyw43_read_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_IOEN);
        if (reg == SDIO_FUNC_ENABLE_1) {
            goto backplane_up;
        }
    }
    CYW43_WARN("no response from CYW43\n");
    return -CYW43_EIO;

backplane_up:
    CYW43_VDEBUG("backplane is up\n");
    ap6256_cyw43_port_set_bus_stage(2U);

    // set the bus to 4-bits
    // (we don't need to change our local SDIO config until we need cmd53)
    uint32_t reg = cyw43_read_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_BICTRL);
    cyw43_write_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_BICTRL, (reg & ~3) | 2);

    // set the block size
    cyw43_write_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_BLKSIZE_0, SDIO_64B_BLOCK);
    reg = cyw43_read_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_BLKSIZE_0);
    if (reg != SDIO_64B_BLOCK) {
        CYW43_WARN("can't set block size\n");
        return -CYW43_EIO;
    }

    cyw43_write_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_BLKSIZE_0, SDIO_64B_BLOCK);
    cyw43_write_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_F1BLKSIZE_0, SDIO_64B_BLOCK);
    cyw43_write_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_F1BLKSIZE_1, 0);
    cyw43_write_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_F2BLKSIZE_0,
                       ap6256_cyw43_port_runtime_f2_block_size() & 0xFFU);
    cyw43_write_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_F2BLKSIZE_1,
                       (ap6256_cyw43_port_runtime_f2_block_size() >> 8U) & 0xFFU);

    // Enable/Disable Client interrupts
    cyw43_write_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_INTEN, INTR_CTL_MASTER_EN | INTR_CTL_FUNC1_EN | INTR_CTL_FUNC2_EN);

    // enable more than 25MHz bus
    reg = cyw43_read_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_SPEED_CONTROL);
    assert(reg & 1); // device must support high-speed mode
    cyw43_write_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_SPEED_CONTROL, reg | 2);

    // enable high speed, 4-bit bus mode for local SDIO controller
    cyw43_sdio_enable_high_speed_4bit();

    // wait for backplane to be ready
    for (int i = 0; i < 10; ++i) {
        reg = cyw43_read_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_IORDY);
        if ((reg & SDIO_FUNC_READY_1) != 0) {
            goto backplane_ready;
        }
        cyw43_delay_ms(1);
    }
    CYW43_WARN("timeout waiting for backplane\n");
    return -CYW43_EIO;
backplane_ready:
    ap6256_cyw43_port_set_bus_stage(3U);
    chip_id_raw = cyw43_read_backplane(self, CHIPCOMMON_BASE_ADDRESS, 4);
    ap6256_cyw43_port_set_chip_id_raw(chip_id_raw);
    ret = ap6256_validate_supported_chip(chip_id_raw);
    if (ret != 0) {
        return ret;
    }
    ret = ap6256_build_boot_descriptor(self,
                                       compat_boot_mode,
                                       (uint32_t)wifi_nvram_len,
                                       compat_reset_vector_value,
                                       &compat_desc);
    if (ret != 0) {
        ap6256_record_core_diag(self);
        return ret;
    }
    compat_boot_mode = compat_desc.boot_mode;
    compat_ram_base = compat_desc.firmware_addr;
    compat_fw_addr = compat_desc.firmware_addr;
    compat_nvram_addr = compat_desc.nvram_addr;
    compat_footer_addr = compat_desc.footer_addr;
    compat_cpu_wrapper = compat_desc.cpu_wrapper;
    compat_ram_wrapper = compat_desc.ram_wrapper;

    #endif

    CYW43_VDEBUG("backplane is ready\n");

    // set the ALP
    #if !CYW43_USE_SPI
    cyw43_write_reg_u8(self, BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR, SBSDIO_FORCE_HW_CLKREQ_OFF | SBSDIO_ALP_AVAIL_REQ | SBSDIO_FORCE_ALP);
    ap6256_checkpoint_record(AP6256_CYW43_CP_ALP_REQUEST,
                             BACKPLANE_FUNCTION,
                             SDIO_CHIP_CLOCK_CSR,
                             SBSDIO_FORCE_HW_CLKREQ_OFF | SBSDIO_ALP_AVAIL_REQ | SBSDIO_FORCE_ALP,
                             cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR),
                             0);
    #else
    cyw43_write_reg_u8(self, BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR, SBSDIO_ALP_AVAIL_REQ);
    ap6256_checkpoint_record(AP6256_CYW43_CP_ALP_REQUEST,
                             BACKPLANE_FUNCTION,
                             SDIO_CHIP_CLOCK_CSR,
                             SBSDIO_ALP_AVAIL_REQ,
                             cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR),
                             0);
    #endif

    #if CYW43_ENABLE_BLUETOOTH
    // check we can set the watermark
    cyw43_write_reg_u8(self, BACKPLANE_FUNCTION, SDIO_FUNCTION2_WATERMARK, 0x10);
    uint8_t reg_8 = cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_FUNCTION2_WATERMARK);
    if (reg_8 != 0x10) {
        return -CYW43_EIO;
    }
    #endif

    for (int i = 0; i < 10; ++i) {
        uint8_t reg = cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR);
        if (reg & SBSDIO_ALP_AVAIL) {
            ap6256_checkpoint_record(AP6256_CYW43_CP_ALP_CONFIRM,
                                     BACKPLANE_FUNCTION,
                                     SDIO_CHIP_CLOCK_CSR,
                                     0U,
                                     reg,
                                     0);
            goto alp_set;
        }
        cyw43_delay_ms(1);
    }
    ap6256_checkpoint_record(AP6256_CYW43_CP_ALP_CONFIRM,
                             BACKPLANE_FUNCTION,
                             SDIO_CHIP_CLOCK_CSR,
                             0U,
                             cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR),
                             -CYW43_EIO);
    ap6256_record_core_diag(self);
    CYW43_WARN("timeout waiting for ALP to be set\n");
    return -CYW43_EIO;

alp_set:
    CYW43_VDEBUG("ALP is set\n");
    ap6256_cyw43_port_set_bus_stage(4U);

    // clear request for ALP
    ret = ap6256_checkpoint_write_u8(self,
                                     AP6256_CYW43_CP_ALP_CLEAR,
                                     BACKPLANE_FUNCTION,
                                     SDIO_CHIP_CLOCK_CSR,
                                     0U,
                                     SBSDIO_ALP_AVAIL_REQ | SBSDIO_FORCE_ALP,
                                     0U);
    if (ret != 0) {
        return ret;
    }

    #if !CYW43_USE_SPI
    // Enable F1 and F2
    ret = ap6256_checkpoint_write_u8(self,
                                     AP6256_CYW43_CP_FN_ENABLE,
                                     BUS_FUNCTION,
                                     SDIOD_CCCR_IOEN,
                                     SDIO_FUNC_ENABLE_1 | SDIO_FUNC_ENABLE_2,
                                     SDIO_FUNC_ENABLE_1 | SDIO_FUNC_ENABLE_2,
                                     SDIO_FUNC_ENABLE_1 | SDIO_FUNC_ENABLE_2);
    if (ret != 0) {
        return ret;
    }

    /*
     * Stage-4 recovery keeps the critical path minimal. Reintroduce the
     * additional BCM43456-specific steps one at a time under explicit
     * checkpoints so a single write/readback mismatch pinpoints the failure.
     */
    if (ap6256_cyw43_profile_enable_pmu(compat_profile) != 0U) {
        uint32_t pmu_ctl = cyw43_read_backplane(self, CHIPCOMMON_PMU_CONTROL, 4);
        uint32_t pmu_write = pmu_ctl | (CHIPCOMMON_PMU_CTL_RES_RELOAD << CHIPCOMMON_PMU_CTL_RES_SHIFT);

        ret = ap6256_checkpoint_write_backplane(self,
                                                AP6256_CYW43_CP_PMU_RELOAD,
                                                CHIPCOMMON_PMU_CONTROL,
                                                pmu_write,
                                                (uint32_t)(CHIPCOMMON_PMU_CTL_RES_RELOAD << CHIPCOMMON_PMU_CTL_RES_SHIFT),
                                                (uint32_t)(CHIPCOMMON_PMU_CTL_RES_RELOAD << CHIPCOMMON_PMU_CTL_RES_SHIFT));
        if (ret != 0) {
            return ret;
        }
    } else {
        ap6256_checkpoint_record(AP6256_CYW43_CP_PMU_RELOAD,
                                 BACKPLANE_FUNCTION,
                                 CHIPCOMMON_PMU_CONTROL,
                                 0U,
                                 cyw43_read_backplane(self, CHIPCOMMON_PMU_CONTROL, 4),
                                 0);
    }

    if (ap6256_cyw43_profile_enable_cardctrl(compat_profile) != 0U) {
        ret = ap6256_checkpoint_write_u8(self,
                                         AP6256_CYW43_CP_CARDCTRL,
                                         BUS_FUNCTION,
                                         SDIOD_CCCR_BRCM_CARDCTRL,
                                         SDIOD_CCCR_BRCM_CARDCTRL_WLANRESET,
                                         SDIOD_CCCR_BRCM_CARDCTRL_WLANRESET,
                                         SDIOD_CCCR_BRCM_CARDCTRL_WLANRESET);
        if (ret != 0) {
            return ret;
        }
    } else {
        ap6256_checkpoint_record(AP6256_CYW43_CP_CARDCTRL,
                                 BUS_FUNCTION,
                                 SDIOD_CCCR_BRCM_CARDCTRL,
                                 0U,
                                 cyw43_read_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_BRCM_CARDCTRL),
                                 0);
    }

    // here we can configure power saving and OOB interrupt

    // Enable active-low OOB interrupt (or value with SEP_INTR_CTL_POL to make it active high)
    ret = ap6256_checkpoint_write_u8(self,
                                     AP6256_CYW43_CP_SEP_INT,
                                     BUS_FUNCTION,
                                     SDIOD_SEP_INT_CTL,
                                     SEP_INTR_CTL_MASK | SEP_INTR_CTL_EN,
                                     SEP_INTR_CTL_MASK | SEP_INTR_CTL_EN,
                                     SEP_INTR_CTL_MASK | SEP_INTR_CTL_EN);
    if (ret != 0) {
        return ret;
    }

    // Enable F2 interrupt only
    ret = ap6256_checkpoint_write_u8(self,
                                     AP6256_CYW43_CP_INT_MASK,
                                     BUS_FUNCTION,
                                     SDIOD_CCCR_INTEN,
                                     INTR_CTL_MASTER_EN | INTR_CTL_FUNC2_EN,
                                     INTR_CTL_MASTER_EN | INTR_CTL_FUNC2_EN,
                                     INTR_CTL_MASTER_EN | INTR_CTL_FUNC2_EN);
    if (ret != 0) {
        return ret;
    }

    cyw43_read_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_IORDY);
    #endif

    // START OF DOWNLOAD_FIRMWARE

    ret = ap6256_halt_cpu_core(self, &compat_desc);
    if (ret != 0) {
        ap6256_checkpoint_record(AP6256_CYW43_CP_WLAN_DISABLE,
                                 BACKPLANE_FUNCTION,
                                 compat_cpu_wrapper + AI_RESETCTRL_OFFSET,
                                 0U,
                                 cyw43_read_backplane(self, compat_cpu_wrapper + AI_RESETCTRL_OFFSET, 4),
                                 ret);
        ap6256_record_core_diag(self);
        return ret;
    }
    if (compat_boot_mode == AP6256_CYW43_BOOT_CR4_TCM) {
        uint32_t ioctrl = cyw43_read_backplane(self, compat_cpu_wrapper + AI_IOCTRL_OFFSET, 4);
        int32_t halt_status = ((ioctrl & SICF_CPUHALT) != 0U) ? 0 : -CYW43_EIO;
        ap6256_checkpoint_record(AP6256_CYW43_CP_WLAN_DISABLE,
                                 BACKPLANE_FUNCTION,
                                 compat_cpu_wrapper + AI_IOCTRL_OFFSET,
                                 SICF_CPUHALT,
                                 ioctrl,
                                 halt_status);
        if (halt_status != 0) {
            return CYW43_FAIL_FAST_CHECK(halt_status);
        }
    } else {
        uint32_t rst = cyw43_read_backplane(self, compat_cpu_wrapper + AI_RESETCTRL_OFFSET, 4);
        int32_t rst_status = ((rst & AIRC_RESET) != 0U) ? 0 : -CYW43_EIO;
        ap6256_checkpoint_record(AP6256_CYW43_CP_WLAN_DISABLE,
                                 BACKPLANE_FUNCTION,
                                 compat_cpu_wrapper + AI_RESETCTRL_OFFSET,
                                 AIRC_RESET,
                                 rst,
                                 rst_status);
        if (rst_status != 0) {
            return CYW43_FAIL_FAST_CHECK(rst_status);
        }
    }
    ap6256_checkpoint_record(AP6256_CYW43_CP_CPU_HALT,
                             BACKPLANE_FUNCTION,
                             compat_cpu_wrapper + AI_IOCTRL_OFFSET,
                             (compat_boot_mode == AP6256_CYW43_BOOT_CR4_TCM) ? SICF_CPUHALT : AIRC_RESET,
                             cyw43_read_backplane(self, compat_cpu_wrapper + AI_IOCTRL_OFFSET, 4),
                             0);

    ap6256_checkpoint_record(AP6256_CYW43_CP_SOCRAM_DISABLE,
                             BACKPLANE_FUNCTION,
                             (compat_ram_wrapper != 0U) ? (compat_ram_wrapper + AI_RESETCTRL_OFFSET) : 0U,
                             0U,
                             (compat_ram_wrapper != 0U) ? cyw43_read_backplane(self, compat_ram_wrapper + AI_RESETCTRL_OFFSET, 4) : 0U,
                             0);
    ap6256_checkpoint_record(AP6256_CYW43_CP_SOCRAM_RESET,
                             BACKPLANE_FUNCTION,
                             (compat_ram_wrapper != 0U) ? (compat_ram_wrapper + AI_RESETCTRL_OFFSET) : 0U,
                             0U,
                             (compat_ram_wrapper != 0U) ? cyw43_read_backplane(self, compat_ram_wrapper + AI_RESETCTRL_OFFSET, 4) : 0U,
                             0);

    if (ap6256_cyw43_profile_enable_remap(compat_profile) != 0U) {
        // this is 4343x specific stuff: Disable remap for SRAM_3
        cyw43_write_backplane(self, SOCSRAM_BANKX_INDEX, 4, 0x3);
        ap6256_checkpoint_record(AP6256_CYW43_CP_SOCRAM_BANKIDX,
                                 BACKPLANE_FUNCTION,
                                 SOCSRAM_BANKX_INDEX,
                                 0x3U,
                                 cyw43_read_backplane(self, SOCSRAM_BANKX_INDEX, 4),
                                 0);
        cyw43_write_backplane(self, SOCSRAM_BANKX_PDA, 4, 0);
        ap6256_checkpoint_record(AP6256_CYW43_CP_SOCRAM_BANKPDA,
                                 BACKPLANE_FUNCTION,
                                 SOCSRAM_BANKX_PDA,
                                 0U,
                                 cyw43_read_backplane(self, SOCSRAM_BANKX_PDA, 4),
                                 0);
    } else {
        ap6256_checkpoint_record(AP6256_CYW43_CP_SOCRAM_BANKIDX,
                                 BACKPLANE_FUNCTION,
                                 SOCSRAM_BANKX_INDEX,
                                 0U,
                                 cyw43_read_backplane(self, SOCSRAM_BANKX_INDEX, 4),
                                 0);
        ap6256_checkpoint_record(AP6256_CYW43_CP_SOCRAM_BANKPDA,
                                 BACKPLANE_FUNCTION,
                                 SOCSRAM_BANKX_PDA,
                                 0U,
                                 cyw43_read_backplane(self, SOCSRAM_BANKX_PDA, 4),
                                 0);
    }

    ap6256_stage5_capture_snapshot(self);

    // Check that valid chipset firmware exists at the given source address.
    ret = cyw43_check_valid_chipset_firmware(self, fw_len, (uintptr_t)fw_data);
    if (ret != 0) {
        return ret;
    }

    // Download the main WiFi firmware blob to the 43xx device.
    ret = cyw43_download_resource(self, compat_fw_addr, fw_len, (uintptr_t)fw_data);
    if (ret != 0) {
        return ret;
    }
    ap6256_checkpoint_record(AP6256_CYW43_CP_FW_DOWNLOAD,
                             BACKPLANE_FUNCTION,
                             compat_fw_addr,
                             (uint32_t)fw_len,
                             0U,
                             0);
    ret = ap6256_verify_resource_words(self,
                                       AP6256_CYW43_CP_FW_VERIFY,
                                       compat_fw_addr,
                                       fw_data,
                                       fw_len);
    if (ret != 0) {
        return ret;
    }

    // Download the NVRAM to the 43xx device.
    ret = cyw43_download_resource(self,
                                  compat_nvram_addr,
                                  wifi_nvram_len_unpadded,
                                  (uintptr_t)wifi_nvram_data);
    if (ret != 0) {
        return ret;
    }
    ap6256_checkpoint_record(AP6256_CYW43_CP_NVRAM_DOWNLOAD,
                             BACKPLANE_FUNCTION,
                             compat_nvram_addr,
                             (uint32_t)wifi_nvram_len_unpadded,
                             0U,
                             0);
    ret = ap6256_verify_resource_words(self,
                                       AP6256_CYW43_CP_NVRAM_VERIFY,
                                       compat_nvram_addr,
                                       wifi_nvram_data,
                                       wifi_nvram_len_unpadded);
    if (ret != 0) {
        return ret;
    }
    uint32_t sz = nvram_footer;
    cyw43_write_backplane(self, compat_footer_addr, 4, sz);
    ap6256_cyw43_port_record_backplane_access(1U, compat_footer_addr, 4U, 0);
    ap6256_checkpoint_record(AP6256_CYW43_CP_NVRAM_FOOTER,
                             BACKPLANE_FUNCTION,
                             compat_footer_addr,
                             sz,
                             cyw43_read_backplane(self, compat_footer_addr, 4),
                             0);
    ret = ap6256_verify_backplane_word(self,
                                       AP6256_CYW43_CP_FOOTER_VERIFY,
                                       compat_footer_addr,
                                       sz);
    if (ret != 0) {
        return ret;
    }
    if (compat_boot_mode == AP6256_CYW43_BOOT_CR4_TCM) {
        cyw43_write_backplane(self, compat_desc.reset_vector_addr, 4, compat_desc.reset_vector_value);
        ret = ap6256_verify_backplane_word(self,
                                           AP6256_CYW43_CP_RESET_VECTOR,
                                           compat_desc.reset_vector_addr,
                                           compat_desc.reset_vector_value);
        if (ret != 0) {
            return ret;
        }
    } else {
        ap6256_checkpoint_record(AP6256_CYW43_CP_RESET_VECTOR,
                                 BACKPLANE_FUNCTION,
                                 compat_fw_addr,
                                 0U,
                                 cyw43_read_backplane(self, compat_fw_addr, 4),
                                 0);
    }
    ap6256_cyw43_port_set_bus_stage(5U);
    ap6256_stage5_capture_snapshot(self);

    ret = ap6256_release_cpu_core(self, &compat_desc);
    if (ret != 0) {
        ap6256_checkpoint_record(AP6256_CYW43_CP_CPU_RELEASE,
                                 BACKPLANE_FUNCTION,
                                 compat_cpu_wrapper + AI_IOCTRL_OFFSET,
                                 0U,
                                 cyw43_read_backplane(self, compat_cpu_wrapper + AI_IOCTRL_OFFSET, 4),
                                 ret);
        ap6256_record_core_diag(self);
        return ret;
    }
    device_core_is_up_by_wrapper(self, compat_cpu_wrapper);
    {
        uint32_t release_io = cyw43_read_backplane(self, compat_cpu_wrapper + AI_IOCTRL_OFFSET, 4);
        uint32_t release_rst = cyw43_read_backplane(self, compat_cpu_wrapper + AI_RESETCTRL_OFFSET, 4);
        int32_t release_status = (((release_io & SICF_CPUHALT) == 0U) &&
                                  ((release_rst & AIRC_RESET) == 0U)) ? 0 : -CYW43_EIO;
        ap6256_checkpoint_record(AP6256_CYW43_CP_CPU_RELEASE,
                                 BACKPLANE_FUNCTION,
                                 compat_cpu_wrapper + AI_IOCTRL_OFFSET,
                                 0U,
                                 release_io,
                                 release_status);
        if (release_status != 0) {
            ap6256_record_core_diag(self);
            return CYW43_FAIL_FAST_CHECK(release_status);
        }
    }
    ap6256_checkpoint_record(AP6256_CYW43_CP_WLAN_RESET,
                             BACKPLANE_FUNCTION,
                             compat_cpu_wrapper + AI_RESETCTRL_OFFSET,
                             0U,
                             cyw43_read_backplane(self, compat_cpu_wrapper + AI_RESETCTRL_OFFSET, 4),
                             0);
    ap6256_record_core_diag(self);
    ap6256_stage5_capture_snapshot(self);

    /*
     * BCM4345/9-class parts do not reliably transition from ALP-only firmware
     * boot into HT by themselves on this STM32 SDIO host. Match the Linux
     * brcmfmac flow and explicitly request HT before polling for HT_AVAIL.
     */
    {
        uint8_t wake_ctrl = cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_WAKEUP_CTRL);
        wake_ctrl |= SBSDIO_WCTRL_WAKE_TILL_HT_AVAIL;
        cyw43_write_reg_u8(self, BACKPLANE_FUNCTION, SDIO_WAKEUP_CTRL, wake_ctrl);
        cyw43_write_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_BRCM_CARDCAP, SDIOD_CCCR_BRCM_CARDCAP_CMD_NODEC);
        cyw43_kso_set(self, 1);
        ap6256_stage5_capture_snapshot(self);
        ap6256_checkpoint_record(AP6256_CYW43_CP_HT_WAKE,
                                 BACKPLANE_FUNCTION,
                                 SDIO_SLEEP_CSR,
                                 (uint32_t)wake_ctrl | ((uint32_t)SDIOD_CCCR_BRCM_CARDCAP_CMD_NODEC << 8),
                                 cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_SLEEP_CSR),
                                 0);
    }

    cyw43_write_reg_u8(self, BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR, SBSDIO_HT_AVAIL_REQ);
    ap6256_checkpoint_record(AP6256_CYW43_CP_HT_REQUEST,
                             BACKPLANE_FUNCTION,
                             SDIO_CHIP_CLOCK_CSR,
                             SBSDIO_HT_AVAIL_REQ,
                             cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR),
                             0);
    ap6256_record_core_diag(self);
    ap6256_stage5_capture_snapshot(self);

    // wait until HT clock is available; takes about 29ms
    for (int i = 0; i < 1000; ++i) {
        uint32_t reg = cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR);
        if (reg & SBSDIO_HT_AVAIL) {
            ap6256_checkpoint_record(AP6256_CYW43_CP_HT_WAIT_REQ,
                                     BACKPLANE_FUNCTION,
                                     SDIO_CHIP_CLOCK_CSR,
                                     SBSDIO_HT_AVAIL_REQ,
                                     reg,
                                     0);
            goto ht_ready;
        }
        cyw43_delay_ms(1);
    }
    ap6256_stage5_capture_snapshot(self);
    ap6256_checkpoint_record(AP6256_CYW43_CP_HT_WAIT_REQ,
                             BACKPLANE_FUNCTION,
                             SDIO_CHIP_CLOCK_CSR,
                             SBSDIO_HT_AVAIL_REQ,
                             cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR),
                             -CYW43_ETIMEDOUT);
    ap6256_record_core_diag(self);

    cyw43_write_reg_u8(self, BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR, SBSDIO_HT_AVAIL_REQ | SBSDIO_FORCE_HT);
    ap6256_checkpoint_record(AP6256_CYW43_CP_HT_REQUEST,
                             BACKPLANE_FUNCTION,
                             SDIO_CHIP_CLOCK_CSR,
                             SBSDIO_HT_AVAIL_REQ | SBSDIO_FORCE_HT,
                             cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR),
                             0);
    ap6256_record_core_diag(self);
    ap6256_stage5_capture_snapshot(self);

    for (int i = 0; i < 1000; ++i) {
        uint32_t reg = cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR);
        if (reg & SBSDIO_HT_AVAIL) {
            ap6256_checkpoint_record(AP6256_CYW43_CP_HT_WAIT_FORCE,
                                     BACKPLANE_FUNCTION,
                                     SDIO_CHIP_CLOCK_CSR,
                                     SBSDIO_HT_AVAIL_REQ | SBSDIO_FORCE_HT,
                                     reg,
                                     0);
            goto ht_ready;
        }
        cyw43_delay_ms(1);
    }
    ap6256_stage5_capture_snapshot(self);
    ap6256_checkpoint_record(AP6256_CYW43_CP_HT_WAIT_FORCE,
                             BACKPLANE_FUNCTION,
                             SDIO_CHIP_CLOCK_CSR,
                             SBSDIO_HT_AVAIL_REQ | SBSDIO_FORCE_HT,
                             cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR),
                             -CYW43_ETIMEDOUT);
    ap6256_record_core_diag(self);
    CYW43_WARN("HT not ready\n");
    return -CYW43_ETIMEDOUT;

ht_ready:
    ap6256_cyw43_port_set_bus_stage(6U);
    ap6256_record_core_diag(self);

    // Set up the interrupt mask and enable interrupts
    cyw43_write_backplane(self, SDIO_INT_HOST_MASK, 4, I_HMB_SW_MASK);

    #if !CYW43_USE_SPI
    // Enable F1 and F2 interrupts. This wasn't required for 4319 but is for the 43362
    cyw43_write_backplane(self, SDIO_FUNCTION_INT_MASK, 1, 2 | 1);

    // Lower F2 Watermark to avoid DMA Hang in F2 when SD Clock is stopped
    cyw43_write_reg_u8(self, BACKPLANE_FUNCTION, SDIO_FUNCTION2_WATERMARK, SDIO_F2_WATERMARK);
    #else

    #if CYW43_ENABLE_BLUETOOTH
    // Set up the interrupt mask and enable interrupts
    cyw43_write_backplane(self, SDIO_INT_HOST_MASK, 4, I_HMB_FC_CHANGE);
    #endif

    /* Lower F2 Watermark to avoid DMA Hang in F2 when SD Clock is stopped. */
    cyw43_write_reg_u8(self, BACKPLANE_FUNCTION, SDIO_FUNCTION2_WATERMARK, SPI_F2_WATERMARK);
    #endif

    // END OF DOWNLOAD_FIRMWARE

    for (int i = 0; i < 1000; ++i) {
        #if CYW43_USE_SPI
        // Wait for F2 to be ready
        uint32_t reg = cyw43_read_reg_u32(self, BUS_FUNCTION, SPI_STATUS_REGISTER);
        if (reg & STATUS_F2_RX_READY) {
            goto f2_ready;
        }
        #else
        uint32_t reg = cyw43_read_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_IORDY);
        if (reg & SDIO_FUNC_READY_2) {
            ap6256_checkpoint_record(AP6256_CYW43_CP_F2_WAIT,
                                     BUS_FUNCTION,
                                     SDIOD_CCCR_IORDY,
                                     0U,
                                     reg,
                                     0);
            goto f2_ready;
        }
        #endif
        cyw43_delay_ms(1);
    }
    ap6256_stage5_capture_snapshot(self);
    ap6256_checkpoint_record(AP6256_CYW43_CP_F2_WAIT,
                             BUS_FUNCTION,
                             SDIOD_CCCR_IORDY,
                             0U,
                             cyw43_read_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_IORDY),
                             -CYW43_ETIMEDOUT);
    ap6256_record_core_diag(self);
    CYW43_WARN("F2 not ready\n");
    return -CYW43_ETIMEDOUT;

f2_ready:
    ap6256_cyw43_port_set_bus_stage(7U);

    #if USE_KSO
    {
        uint8_t reg8 = cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_WAKEUP_CTRL);
        reg8 |= SBSDIO_WCTRL_WAKE_TILL_HT_AVAIL;
        cyw43_write_reg_u8(self, BACKPLANE_FUNCTION, SDIO_WAKEUP_CTRL, reg8);
        cyw43_write_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_BRCM_CARDCAP, SDIOD_CCCR_BRCM_CARDCAP_CMD_NODEC);
        cyw43_write_reg_u8(self, BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR, SBSDIO_FORCE_HT);
        reg8 = cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_SLEEP_CSR);
        if (!(reg8 & SBSDIO_SLPCSR_KEEP_SDIO_ON)) {
            reg8 |= SBSDIO_SLPCSR_KEEP_SDIO_ON;
            cyw43_write_reg_u8(self, BACKPLANE_FUNCTION, SDIO_SLEEP_CSR, reg8);
        }

        // TODO: This causes the device to fail after sleep. Check if whd_bus_spi_sleep is ever called
        #if 0 && CYW43_USE_SPI
        // whd_bus_spi_sleep
        uint32_t spi_bus_reg_value = cyw43_read_reg_u32(self, BUS_FUNCTION, SPI_BUS_CONTROL);
        spi_bus_reg_value &= ~(uint32_t)(WAKE_UP);
        cyw43_write_reg_u32(self, BUS_FUNCTION, SPI_BUS_CONTROL, spi_bus_reg_value);
        #endif
        // Put SPI interface block to sleep
        cyw43_write_reg_u8(self, BACKPLANE_FUNCTION, SDIO_PULL_UP, 0xf);
    }
    #endif

    // CLEAR PAD PULLS
    {
        cyw43_write_reg_u8(self, BACKPLANE_FUNCTION, SDIO_PULL_UP, 0);
        cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_PULL_UP);
    }

    #if CYW43_USE_SPI
    {
        // We always seem to start with a data unavailable error - so clear it now
        uint16_t spi_int_status = cyw43_read_reg_u16(self, BUS_FUNCTION, SPI_INTERRUPT_REGISTER);
        if (spi_int_status & DATA_UNAVAILABLE) {
            cyw43_write_reg_u16(self, BUS_FUNCTION, SPI_INTERRUPT_REGISTER, spi_int_status);
        }

    }
    #endif

    #ifdef NDEBUG
    // This will be a non-zero value if save/restore is enabled
    cyw43_read_backplane(self, CHIPCOMMON_SR_CONTROL1, 4);
    #endif

    cyw43_ll_bus_sleep(self_in, false);

    // Load the CLM data after the WLAN core is ready.
    ap6256_cyw43_port_set_bus_stage(8U);
    CYW43_VDEBUG("cyw43_clm_load start\n");
    cyw43_clm_load(self, clm_data, clm_len);
    ap6256_checkpoint_record(AP6256_CYW43_CP_CLM_LOAD,
                             BACKPLANE_FUNCTION,
                             0U,
                             (uint32_t)clm_len,
                             0U,
                             0);
    CYW43_VDEBUG("cyw43_clm_load done\n");

    ap6256_cyw43_port_set_bus_stage(9U);
    cyw43_write_iovar_u32(self, "bus:txglom", 0, WWD_STA_INTERFACE); // tx glomming off
    ap6256_checkpoint_record(AP6256_CYW43_CP_TXGLOM,
                             BACKPLANE_FUNCTION,
                             0U,
                             0U,
                             0U,
                             0);
    ap6256_cyw43_port_set_bus_stage(10U);
    cyw43_write_iovar_u32(self, "apsta", 1, WWD_STA_INTERFACE); // apsta on
    ap6256_checkpoint_record(AP6256_CYW43_CP_APSTA,
                             BACKPLANE_FUNCTION,
                             0U,
                             1U,
                             0U,
                             0);

    // Set the MAC (same one used for STA and AP interfaces)
    // If CYW43_USE_OTP_MAC is set then cyw43 otp should be used to store the mac and no mac is passed in.
    // When the chip hs been initialised we can read the otp
    // If it looks like otp is unset the call cyw43_hal_generate_laa_mac to get the "port" to make a mac address for us
    #if CYW43_USE_OTP_MAC
    uint8_t mac_buf[6];
    if (!mac) {
        const uint8_t default_mac[] = { 0x00, 0xA0, 0x50, 0xb5, 0x59, 0x5e };

        // Check the if the mac is set in otp. The default mac from nvram will be used if not
        int err = cyw43_ll_wifi_get_mac(self_in, mac_buf);
        if (err != 0 || memcmp(mac_buf, default_mac, sizeof(mac_buf)) == 0) {
            // mac not set in otp, so make one up
            cyw43_hal_generate_laa_mac(WWD_STA_INTERFACE, mac_buf);
            mac = mac_buf;
        }
    }
    #endif

    if (mac) {
        cyw43_write_iovar_n(self, "cur_etheraddr", 6, mac, WWD_STA_INTERFACE);
    }

    return 0;
}

/*******************************************************************************/
// WiFi stuff

static int cyw43_set_ioctl_u32(cyw43_int_t *self, uint32_t cmd, uint32_t val, uint32_t iface) {
    uint8_t *buf = &self->spid_buf[SDPCM_HEADER_LEN + 16];
    cyw43_put_le32(buf, val);
    return cyw43_do_ioctl(self, SDPCM_SET, cmd, 4, buf, iface);
}

static void cyw43_ap6256_event_mask_set(uint8_t *mask, uint32_t event_type) {
    mask[event_type >> 3] |= (uint8_t)(1U << (event_type & 7U));
}

static void cyw43_ap6256_build_sta_event_mask(uint8_t *mask, size_t mask_len) {
    memset(mask, 0, mask_len);
    cyw43_ap6256_event_mask_set(mask, CYW43_EV_SET_SSID);
    cyw43_ap6256_event_mask_set(mask, CYW43_EV_JOIN);
    cyw43_ap6256_event_mask_set(mask, CYW43_EV_AUTH);
    cyw43_ap6256_event_mask_set(mask, CYW43_EV_DEAUTH);
    cyw43_ap6256_event_mask_set(mask, CYW43_EV_DEAUTH_IND);
    cyw43_ap6256_event_mask_set(mask, CYW43_EV_ASSOC);
    cyw43_ap6256_event_mask_set(mask, CYW43_EV_DISASSOC);
    cyw43_ap6256_event_mask_set(mask, CYW43_EV_DISASSOC_IND);
    cyw43_ap6256_event_mask_set(mask, CYW43_EV_LINK);
    cyw43_ap6256_event_mask_set(mask, CYW43_EV_PRUNE);
    cyw43_ap6256_event_mask_set(mask, CYW43_EV_PSK_SUP);
    cyw43_ap6256_event_mask_set(mask, CYW43_EV_ICV_ERROR);
    cyw43_ap6256_event_mask_set(mask, CYW43_EV_ESCAN_RESULT);
    cyw43_ap6256_event_mask_set(mask, CYW43_EV_CSA_COMPLETE_IND);
    cyw43_ap6256_event_mask_set(mask, CYW43_EV_ASSOC_REQ_IE);
    cyw43_ap6256_event_mask_set(mask, CYW43_EV_ASSOC_RESP_IE);
}

static int cyw43_ap6256_program_sta_event_mask(cyw43_int_t *self) {
    uint8_t event_mask[4 + 32];
    int ret_global;

    /*
     * Keep the join event path simple and brcmfmac-like: program the global
     * event_msgs bitmap used by the STA firmware path. Do not send a bare
     * zeroed "bsscfg:event_msgs" payload here; on BCM43456 that can be
     * interpreted as clearing the per-BSS event mask, which leaves association
     * running silently with no AUTH/ASSOC/LINK/PSK_SUP events for the host.
     */
    cyw43_ap6256_build_sta_event_mask(event_mask + 4, 32);
    ret_global = cyw43_write_iovar_n(self, "event_msgs", 32, event_mask + 4, WWD_STA_INTERFACE);
    return ret_global;
}

static uint32_t cyw43_get_ioctl_u32(cyw43_int_t *self, uint32_t cmd, uint32_t iface) {
    uint8_t *buf = &self->spid_buf[SDPCM_HEADER_LEN + 16];
    cyw43_put_le32(buf, 0);
    cyw43_do_ioctl(self, SDPCM_GET, cmd, 4, buf, iface);
    return cyw43_get_le32(buf);
}

static uint32_t cyw43_read_iovar_u32(cyw43_int_t *self, const char *var, uint32_t iface) {
    uint8_t *buf = &self->spid_buf[SDPCM_HEADER_LEN + 16];
    size_t len = strlen(var) + 1;
    memcpy(buf, var, len);
    cyw43_put_le32(buf + len, 0);
    cyw43_do_ioctl(self, SDPCM_GET, WLC_GET_VAR, len + 4, buf, iface);
    return cyw43_get_le32(buf);
}

#if 0
#define WLC_SET_MONITOR (108)
int cyw43_set_monitor_mode(cyw43_ll_t *self, int value) {
    CYW_THREAD_ENTER;
    int ret = cyw43_ensure_up(self);
    if (ret) {
        CYW_THREAD_EXIT;
        return ret;
    }

    CYW_ENTER;
    self->is_monitor_mode = value;
    cyw43_write_iovar_u32(self, "allmulti", value, WWD_STA_INTERFACE);
    cyw43_set_ioctl_u32(self, WLC_SET_MONITOR, value, WWD_STA_INTERFACE);
    CYW_EXIT;
    CYW_THREAD_EXIT;

    return 0;
}
#endif

// Requires cyw43_ll_bus_init to have been called first
int cyw43_ll_wifi_on(cyw43_ll_t *self_in, uint32_t country) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);

    uint8_t *buf = &self->spid_buf[SDPCM_HEADER_LEN + 16];

    #if !CYW43_USE_SPI
    if (country == CYW43_COUNTRY_WORLDWIDE) {
        // For the 1DX CLM, we need to use revision 17 for the worldwide
        // country. This increases throughput by about 25% on PYBD.
        country |= (17 << 16);
    }
    #endif

    // Set country; takes about 32ms
    memcpy(buf, "country\x00", 8);
    cyw43_put_le32(buf + 8, country & 0xffff);
    /*
     * Preserve explicit revision 0. The upstream CYW43 helper used -1 when the
     * country revision field was zero, but brcmfmac sends the requested
     * country/revision pair directly. AP6256's NVRAM declares ccode=US,
     * regrev=0; using US/-1 can select different 5 GHz CLM/power behavior just
     * before association.
     */
    cyw43_put_le32(buf + 12, country >> 16);
    cyw43_put_le32(buf + 16, country & 0xffff);
    cyw43_do_ioctl(self, SDPCM_SET, WLC_SET_VAR, 20, buf, WWD_STA_INTERFACE);

    cyw43_delay_ms(50);

    #ifndef NDEBUG
    // Get and print CLM version
    memcpy(buf, "clmver\x00", 7);
    cyw43_do_ioctl(self, SDPCM_GET, WLC_GET_VAR, 128, buf, WWD_STA_INTERFACE);
    CYW43_DEBUG("%s\n", buf);
    #endif

    // Set antenna to chip antenna
    cyw43_set_ioctl_u32(self, WLC_SET_ANTDIV, 0, WWD_STA_INTERFACE);

    // Set some WiFi config
    cyw43_write_iovar_u32(self, "bus:txglom", 0, WWD_STA_INTERFACE); // tx glomming off
    cyw43_write_iovar_u32(self, "apsta", 1, WWD_STA_INTERFACE); // apsta on
    /*
     * brcmfmac keeps station connect/scan paths out of firmware autonomous
     * power/roam decisions while cfg80211 owns association. Keep these
     * best-effort: older firmware may reject an iovar, but accepted values make
     * scan/join/DHCP behavior much more deterministic on AP6256.
     */
    cyw43_write_iovar_u32(self, "mpc", 0, WWD_STA_INTERFACE);
    cyw43_write_iovar_u32(self, "roam_off", 1, WWD_STA_INTERFACE);
    cyw43_write_iovar_u32(self, "ampdu_ba_wsize", 8, WWD_STA_INTERFACE);
    cyw43_write_iovar_u32(self, "ampdu_mpdu", 4, WWD_STA_INTERFACE);
    cyw43_write_iovar_u32(self, "ampdu_rx_factor", 0, WWD_STA_INTERFACE);

    // This delay is needed for the WLAN chip to do some processing, otherwise
    // SDIOIT/OOB WL_HOST_WAKE IRQs in bus-sleep mode do no work correctly.
    uint32_t dt = cyw43_hal_ticks_us() - self->startup_t0;
    if (dt < 150000) {
        cyw43_delay_us(150000 - dt);
    }

    // Clear all async events
    memset(buf + 18 + 4, 0xff, 19); // enable them all
    #define CLR_EV(b, i) b[18 + 4 + i / 8] &= ~(1 << (i % 8))
    CLR_EV(buf, 19); // roam attempt occurred
    CLR_EV(buf, 20); // tx fail
    CLR_EV(buf, 40); // radio
    CLR_EV(buf, 44); // probe request
    CLR_EV(buf, 54); // interface change
    CLR_EV(buf, 71); // probe response
    #undef CLR_EV
    memcpy(buf, "bsscfg:event_msgs", 18);
    cyw43_do_ioctl(self, SDPCM_SET, WLC_SET_VAR, 18 + 4 + 19, buf, WWD_STA_INTERFACE);

    cyw43_delay_ms(50);

    // Set the interface as "up"
    cyw43_do_ioctl(self, SDPCM_SET, WLC_UP, 0, NULL, WWD_STA_INTERFACE);

    cyw43_delay_ms(50);

    return 0;
}

int cyw43_ll_wifi_get_mac(cyw43_ll_t *self_in, uint8_t *addr) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);
    uint8_t *buf = &self->spid_buf[SDPCM_HEADER_LEN + 16];

    // Get mac address
    memcpy(buf, "cur_etheraddr\x00\x00\x00\x00\x00\x00\x00", 14 + 6);
    int err = cyw43_do_ioctl(self, SDPCM_GET, WLC_GET_VAR, 14 + 6, buf, WWD_STA_INTERFACE);
    if (err == 0) {
        memcpy(addr, buf, 6);
    }
    return err;
}

int cyw43_ll_wifi_update_multicast_filter(cyw43_ll_t *self_in, uint8_t *addr, bool add) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);
    uint8_t *buf = &self->spid_buf[SDPCM_HEADER_LEN + 16];

    // query the current list
    memcpy(buf, "mcast_list", 11);
    memset(buf + 11, 0, 4 + MAX_MULTICAST_REGISTERED_ADDRESS * 6);
    int err = cyw43_do_ioctl(self, SDPCM_GET, WLC_GET_VAR, 11 + 4 + MAX_MULTICAST_REGISTERED_ADDRESS * 6, buf, WWD_STA_INTERFACE);
    if (err != 0) {
        return err;
    }

    // current number of addresses
    uint32_t n = cyw43_get_le32(buf);
    buf += 4;

    for (uint32_t i = 0; i < n; ++i) {
        if (memcmp(buf + i * 6, addr, 6) == 0) {
            if (add) {
                // addr already in the list
                return 0;
            } else {
                // remove this address
                if (i < n - 1) {
                    // replace with the end of the list
                    memcpy(buf + i * 6, buf + (n - 1) * 6, 6);
                }
                --n;
            }
        }
    }
    if (add) {
        if (n == MAX_MULTICAST_REGISTERED_ADDRESS) {
            return CYW43_FAIL_FAST_CHECK(-CYW43_EPERM);
        }
        memcpy(buf + n * 6, addr, 6);
        ++n;
    }

    // update number of addresses
    buf -= 4;
    cyw43_put_le32(buf, n);

    // write back address list
    cyw43_write_iovar_n(self, "mcast_list", 4 + MAX_MULTICAST_REGISTERED_ADDRESS * 6, buf, WWD_STA_INTERFACE);
    cyw43_delay_ms(50);

    return 0;
}

int cyw43_ll_wifi_pm(cyw43_ll_t *self_in, uint32_t pm, uint32_t pm_sleep_ret, uint32_t li_bcn, uint32_t li_dtim, uint32_t li_assoc) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);

    // set some power saving parameters
    // PM1 is very aggressive in power saving and reduces wifi throughput
    // PM2 only saves power when there is no wifi activity for some time

    // Value passed to pm2_sleep_ret measured in ms, must be multiple of 10, between 10 and 2000
    if (pm_sleep_ret < 1) {
        pm_sleep_ret = 1;
    } else if (pm_sleep_ret > 200) {
        pm_sleep_ret = 200;
    }
    cyw43_write_iovar_u32(self, "pm2_sleep_ret", pm_sleep_ret * 10, WWD_STA_INTERFACE);

    // these parameters set beacon intervals and are used to reduce power consumption
    // while associated to an AP but not doing tx/rx
    // bcn_li_xxx is what the CYW43x will do; assoc_listen is what is sent to the AP
    // bcn_li_dtim==0 means use bcn_li_bcn
    cyw43_write_iovar_u32(self, "bcn_li_bcn", li_bcn, WWD_STA_INTERFACE);
    cyw43_write_iovar_u32(self, "bcn_li_dtim", li_dtim, WWD_STA_INTERFACE);
    cyw43_write_iovar_u32(self, "assoc_listen", li_assoc, WWD_STA_INTERFACE);

    #if 0
    CYW43_PRINTF("pm2_sleep_ret: %lu\n", cyw43_read_iovar_u32(self, "pm2_sleep_ret", WWD_STA_INTERFACE));
    CYW43_PRINTF("bcn_li_bcn: %lu\n", cyw43_read_iovar_u32(self, "bcn_li_bcn", WWD_STA_INTERFACE));
    CYW43_PRINTF("bcn_li_dtim: %lu\n", cyw43_read_iovar_u32(self, "bcn_li_dtim", WWD_STA_INTERFACE));
    CYW43_PRINTF("assoc_listen: %lu\n", cyw43_read_iovar_u32(self, "assoc_listen", WWD_STA_INTERFACE));
    #endif

    cyw43_set_ioctl_u32(self, WLC_SET_PM, pm, WWD_STA_INTERFACE);

    // Set GMODE_AUTO
    uint8_t *buf = &self->spid_buf[SDPCM_HEADER_LEN + 16];
    cyw43_put_le32(buf, 1);
    cyw43_do_ioctl(self, SDPCM_SET, WLC_SET_GMODE, 4, buf, WWD_STA_INTERFACE);

    cyw43_put_le32(buf, 0); // any
    cyw43_do_ioctl(self, SDPCM_SET, WLC_SET_BAND, 4, buf, WWD_STA_INTERFACE);

    return 0;
}

int cyw43_ll_wifi_get_pm(cyw43_ll_t *self_in, uint32_t *pm, uint32_t *pm_sleep_ret, uint32_t *li_bcn, uint32_t *li_dtim, uint32_t *li_assoc) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);
    *pm_sleep_ret = cyw43_read_iovar_u32(self, "pm2_sleep_ret", WWD_STA_INTERFACE);
    *li_bcn = cyw43_read_iovar_u32(self, "bcn_li_bcn", WWD_STA_INTERFACE);
    *li_dtim = cyw43_read_iovar_u32(self, "bcn_li_dtim", WWD_STA_INTERFACE);
    *li_assoc = cyw43_read_iovar_u32(self, "assoc_listen", WWD_STA_INTERFACE);
    *pm = cyw43_get_ioctl_u32(self, WLC_GET_PM, WWD_STA_INTERFACE);
    return 0;
}

static int ap6256_cyw43_scan_wake_return(int ret) {
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SCAN_WAKE_EXIT, ret);
    return ret;
}

static int ap6256_cyw43_scan_wake(cyw43_int_t *self, cyw43_ll_t *self_in) {
    int wake_ctrl;
    int sleep_csr;
    int cardcap;
    int io_ready;
    int chip_clock;
    int ret = 0;
    uint8_t wake_ctrl_request;

    ap6256_cyw43_port_set_ioctl_phase(AP6256_CYW43_IOCTL_PHASE_SCAN_WAKE);
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SCAN_WAKE_ENTER, 0);
    cyw43_ll_bus_sleep(self_in, false);

    ap6256_cyw43_port_set_scan_wake_step(AP6256_CYW43_SCAN_WAKE_WAKE_CTRL_READ);
    wake_ctrl = cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_WAKEUP_CTRL);
    if (wake_ctrl < 0) {
        return ap6256_cyw43_scan_wake_return(CYW43_FAIL_FAST_CHECK(-CYW43_EIO));
    }

    ap6256_cyw43_port_set_scan_wake_step(AP6256_CYW43_SCAN_WAKE_WAKE_CTRL_WRITE);
    wake_ctrl_request = (uint8_t)((uint32_t)wake_ctrl | SBSDIO_WCTRL_WAKE_TILL_HT_AVAIL);
    ret = cyw43_write_reg_u8(self,
                             BACKPLANE_FUNCTION,
                             SDIO_WAKEUP_CTRL,
                             wake_ctrl_request);
    if (ret != 0) {
        return ap6256_cyw43_scan_wake_return(ret);
    }
    wake_ctrl = cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_WAKEUP_CTRL);
    if ((wake_ctrl < 0) || (((uint32_t)wake_ctrl & SBSDIO_WCTRL_WAKE_TILL_HT_AVAIL) == 0U)) {
        return ap6256_cyw43_scan_wake_return(CYW43_FAIL_FAST_CHECK(-CYW43_EIO));
    }

    ap6256_cyw43_port_set_scan_wake_step(AP6256_CYW43_SCAN_WAKE_KSO);
    #if USE_KSO
    ret = cyw43_kso_set(self, 1);
    if (ret != 0) {
        return ap6256_cyw43_scan_wake_return(ret);
    }
    #endif

    ap6256_cyw43_port_set_scan_wake_step(AP6256_CYW43_SCAN_WAKE_SLEEP_VERIFY);
    sleep_csr = cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_SLEEP_CSR);
    if (sleep_csr < 0) {
        return ap6256_cyw43_scan_wake_return(CYW43_FAIL_FAST_CHECK(-CYW43_EIO));
    }

    cardcap = cyw43_read_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_BRCM_CARDCAP);
    io_ready = cyw43_read_reg_u8(self, BUS_FUNCTION, SDIOD_CCCR_IORDY);
    chip_clock = cyw43_read_reg_u8(self, BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR);
    ap6256_cyw43_port_set_stage5_snapshot((wake_ctrl >= 0) ? (uint8_t)wake_ctrl : 0U,
                                          (sleep_csr >= 0) ? (uint8_t)sleep_csr : 0U,
                                          (cardcap >= 0) ? (uint8_t)cardcap : 0U,
                                          (io_ready >= 0) ? (uint8_t)io_ready : 0U);

    if ((sleep_csr & SBSDIO_SLPCSR_KEEP_SDIO_ON) == 0) {
        return ap6256_cyw43_scan_wake_return(CYW43_FAIL_FAST_CHECK(-CYW43_EIO));
    }
    ap6256_cyw43_port_set_scan_wake_step(AP6256_CYW43_SCAN_WAKE_HT_VERIFY);
    if ((cardcap < 0) || (chip_clock < 0)) {
        return ap6256_cyw43_scan_wake_return(CYW43_FAIL_FAST_CHECK(-CYW43_EIO));
    }
    if ((chip_clock & SBSDIO_HT_AVAIL) == 0) {
        return ap6256_cyw43_scan_wake_return(CYW43_FAIL_FAST_CHECK(-CYW43_ETIMEDOUT));
    }
    ap6256_cyw43_port_set_scan_wake_step(AP6256_CYW43_SCAN_WAKE_IORDY_VERIFY);
    if (io_ready < 0) {
        return ap6256_cyw43_scan_wake_return(CYW43_FAIL_FAST_CHECK(-CYW43_EIO));
    }
    if ((io_ready & SDIO_FUNC_READY_2) == 0) {
        return ap6256_cyw43_scan_wake_return(CYW43_FAIL_FAST_CHECK(-CYW43_ETIMEDOUT));
    }

    ap6256_cyw43_port_set_scan_wake_step(AP6256_CYW43_SCAN_WAKE_OK);
    return ap6256_cyw43_scan_wake_return(0);
}

static void ap6256_cyw43_drain_pending_packets(cyw43_int_t *self) {
    uint32_t start = cyw43_hal_ticks_ms();
    uint32_t packets = 0U;

    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SCAN_DRAIN_ENTER, 0);
    while (((cyw43_hal_ticks_ms() - start) < 250U) && (packets < 16U)) {
        size_t len;
        uint8_t *buf;
        int ret;

        #if !CYW43_USE_SPI
        if (!self->had_successful_packet && !cyw43_ll_sdio_packet_pending(self)) {
            break;
        }
        #endif

        ret = cyw43_ll_sdpcm_poll_device(self, &len, &buf);
        if (ret == -1) {
            break;
        } else if (ret == ASYNCEVENT_HEADER) {
            cyw43_async_event_t *ev = cyw43_ll_parse_async_event(len, buf);
            if (ev != NULL) {
                cyw43_cb_process_async_event(self, ev);
            }
        } else if (ret == DATA_HEADER) {
            if (!ap6256_join_should_drop_prelink_data()) {
                cyw43_cb_process_ethernet(self->cb_data, len >> 31, len & 0x7fffffff, buf);
            }
        }

        packets++;
        CYW43_DO_IOCTL_WAIT;
    }
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SCAN_DRAIN_EXIT,
                                        (int32_t)packets);
}

typedef struct _ap6256_brcmf_scan_abort_params_t {
    uint32_t ssid_len;
    uint8_t ssid[32];
    uint8_t bssid[6];
    int8_t bss_type;
    uint8_t scan_type;
    int32_t nprobes;
    int32_t active_time;
    int32_t passive_time;
    int32_t home_time;
    int32_t channel_num;
    uint16_t channel_list[1];
} ap6256_brcmf_scan_abort_params_t;

typedef struct _ap6256_brcmf_escan_abort_params_t {
    uint32_t version;
    uint16_t action;
    uint16_t sync_id;
    uint32_t ssid_len;
    uint8_t ssid[32];
    uint8_t bssid[6];
    int8_t bss_type;
    int8_t scan_type;
    int32_t nprobes;
    int32_t active_time;
    int32_t passive_time;
    int32_t home_time;
    int32_t channel_num;
    uint16_t channel_list[1];
} ap6256_brcmf_escan_abort_params_t;

static void __attribute__((unused)) ap6256_cyw43_escan_abort(cyw43_int_t *self, bool force) {
    ap6256_brcmf_escan_abort_params_t escan_abort;
    ap6256_brcmf_scan_abort_params_t abort_scan;

    if ((self == NULL) || ((force == false) && (s_ap6256_scan_active == 0U))) {
        return;
    }

    /*
     * BCM43456 on this board keeps delivering ESCAN_RESULT after a completed
     * directed scan if the host only detaches its callbacks. Send both abort
     * shapes: the escan action is what started this path, while WLC_SCAN mirrors
     * brcmfmac's scan-abort fallback before association.
    */
    memset(&escan_abort, 0, sizeof(escan_abort));
    escan_abort.version = 1; // ESCAN_REQ_VERSION
    escan_abort.action = 3; // WL_SCAN_ACTION_ABORT
    escan_abort.sync_id = s_ap6256_scan_sync_id;
    memset(escan_abort.bssid, 0xff, sizeof(escan_abort.bssid));
    escan_abort.bss_type = 2; // WICED_BSS_TYPE_ANY
    escan_abort.nprobes = -1;
    escan_abort.active_time = -1;
    escan_abort.passive_time = -1;
    escan_abort.home_time = -1;
    escan_abort.channel_num = 1;
    escan_abort.channel_list[0] = 0xffffU;
    (void)cyw43_write_iovar_n(self,
                              "escan",
                              sizeof(escan_abort),
                              &escan_abort,
                              WWD_STA_INTERFACE);
    ap6256_cyw43_drain_pending_packets(self);

    memset(&abort_scan, 0, sizeof(abort_scan));
    memset(abort_scan.bssid, 0xff, sizeof(abort_scan.bssid));
    abort_scan.bss_type = 2; // WICED_BSS_TYPE_ANY
    abort_scan.scan_type = 0;
    abort_scan.nprobes = -1;
    abort_scan.active_time = -1;
    abort_scan.passive_time = -1;
    abort_scan.home_time = -1;
    abort_scan.channel_num = 1;
    abort_scan.channel_list[0] = 0xffffU;

    uint32_t start_count = ap6256_cyw43_port_async_event_count();
    uint32_t start_ms = cyw43_hal_ticks_ms();
    (void)cyw43_do_ioctl(self,
                         SDPCM_SET,
                         WLC_SCAN,
                         sizeof(abort_scan),
                         (uint8_t *)&abort_scan,
                         WWD_STA_INTERFACE);
    while ((cyw43_hal_ticks_ms() - start_ms) < 1000U) {
        ap6256_cyw43_drain_pending_packets(self);
        if ((ap6256_cyw43_port_async_event_count() != start_count) &&
            (ap6256_cyw43_port_last_async_event_type() == CYW43_EV_ESCAN_RESULT) &&
            ((ap6256_cyw43_port_last_async_event_status() == 4U) ||
             (ap6256_cyw43_port_last_async_event_status() == 0U))) {
            break;
        }
        cyw43_delay_ms(20);
    }
    /*
     * brcmfmac keeps scan state busy/abort until the ESCAN completion path has
     * settled. BCM43456 can deliver one more ESCAN_RESULT(status=ABORT/COMPLETE)
     * after the first abort indication; starting association during that tail
     * leaves the firmware in a scan/join overlap and has been observed to reset
     * this AP6256 board. Require a short quiet window before returning.
     */
    {
        uint32_t quiet_start_ms = cyw43_hal_ticks_ms();
        uint32_t quiet_deadline_start_ms = quiet_start_ms;
        uint32_t seen_count = ap6256_cyw43_port_async_event_count();

        while (((cyw43_hal_ticks_ms() - quiet_start_ms) < 500U) &&
               ((cyw43_hal_ticks_ms() - quiet_deadline_start_ms) < 1500U)) {
            uint32_t count_before = ap6256_cyw43_port_async_event_count();

            ap6256_cyw43_drain_pending_packets(self);
            if (ap6256_cyw43_port_async_event_count() != count_before) {
                seen_count = ap6256_cyw43_port_async_event_count();
                quiet_start_ms = cyw43_hal_ticks_ms();
            } else if (ap6256_cyw43_port_async_event_count() != seen_count) {
                seen_count = ap6256_cyw43_port_async_event_count();
                quiet_start_ms = cyw43_hal_ticks_ms();
            }
            cyw43_delay_ms(20);
        }
    }
    ap6256_cyw43_drain_pending_packets(self);
    s_ap6256_scan_active = 0U;
}

#define AP6256_ESCAN_CHANNEL_COUNT 38U
#define AP6256_ESCAN_5G_CHANNEL_COUNT 25U
#define AP6256_ESCAN_FORCE_5G_CHANNEL_NUM (-5)

typedef struct _ap6256_cyw43_escan_options_t {
    uint32_t version;
    uint16_t action;
    uint16_t sync_id;
    uint32_t ssid_len;
    uint8_t ssid[32];
    uint8_t bssid[6];
    int8_t bss_type;
    int8_t scan_type;
    int32_t nprobes;
    int32_t active_time;
    int32_t passive_time;
    int32_t home_time;
    int32_t channel_num;
    uint16_t channel_list[AP6256_ESCAN_CHANNEL_COUNT];
} ap6256_cyw43_escan_options_t;

static uint32_t ap6256_cyw43_scan_channel_list(uint16_t *dst, uint8_t force_5g)
{
    static const uint8_t all_channels[AP6256_ESCAN_CHANNEL_COUNT] = {
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
        36, 40, 44, 48,
        52, 56, 60, 64,
        100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144,
        149, 153, 157, 161, 165,
    };
    static const uint8_t channels_5g[AP6256_ESCAN_5G_CHANNEL_COUNT] = {
        /*
         * Prefer non-DFS channels before DFS, but keep the list channel-complete
         * for visible 5 GHz-only fixtures that can move across the band.
         */
        36, 40, 44, 48,
        149, 153, 157, 161, 165,
        52, 56, 60, 64,
        100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144,
    };
    const uint8_t *channels = (force_5g != 0U) ? channels_5g : all_channels;
    uint32_t count = (force_5g != 0U) ? AP6256_ESCAN_5G_CHANNEL_COUNT : AP6256_ESCAN_CHANNEL_COUNT;

    for (uint32_t i = 0U; i < count; ++i) {
        dst[i] = AP6256_CHSPEC_20MHZ(channels[i]);
    }

    return count;
}

static void ap6256_cyw43_populate_dual_band_scan(ap6256_cyw43_escan_options_t *scan,
                                                 uint16_t sync_id,
                                                 const cyw43_wifi_scan_options_t *opts) {
    uint8_t force_5g = ((opts != NULL) &&
                        (opts->channel_num == AP6256_ESCAN_FORCE_5G_CHANNEL_NUM)) ? 1U : 0U;
    uint32_t channel_count;

    memset(scan, 0, sizeof(*scan));
    scan->version = 1; // ESCAN_REQ_VERSION
    scan->action = 1; // WL_SCAN_ACTION_START
    scan->sync_id = sync_id;
    memset(scan->bssid, 0xff, sizeof(scan->bssid));
    scan->bss_type = 2; // WICED_BSS_TYPE_ANY
    scan->scan_type = (opts != NULL) ? opts->scan_type : 0;
    if ((opts != NULL) && (opts->ssid_len > 0U) && (opts->ssid_len <= sizeof(scan->ssid))) {
        scan->ssid_len = opts->ssid_len;
        memcpy(scan->ssid, opts->ssid, opts->ssid_len);
    }

    /*
     * brcmfmac leaves dwell/probe values at firmware defaults for normal
     * broadcast escan. That is safer for 5 GHz/passive channels than the old
     * short AP6256 dwell times, which only found the 2.4 GHz Linksys BSSIDs.
     */
    scan->nprobes = -1;
    scan->active_time = -1;
    scan->passive_time = -1;
    scan->home_time = -1;

    if (force_5g != 0U) {
        /*
         * Directed 5 GHz scans need an explicit 5 GHz-only list so the firmware
         * sends probe requests on the relevant band. For normal visible scans,
         * mirror brcmfmac and leave channel_num at zero: firmware
         * then scans its regulatory/current channel set itself. The old
         * 38-channel host list started with 2.4 GHz and our bounded wait could
         * return before useful 5 GHz channels, hiding visible 5 GHz-only APs.
         */
        channel_count = ap6256_cyw43_scan_channel_list(scan->channel_list, force_5g);
        scan->channel_num = (int32_t)channel_count;
    } else {
        scan->channel_num = 0;
    }
}

int cyw43_ll_wifi_scan(cyw43_ll_t *self_in, cyw43_wifi_scan_options_t *opts) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);
    ap6256_cyw43_escan_options_t scan;
    uint16_t sync_id = (uint16_t)(s_ap6256_scan_sync_id + 1U);
    int wake_ret;

    if (s_ap6256_scan_active != 0U) {
        ap6256_cyw43_escan_abort(self, false);
    }

    if (sync_id == 0U) {
        sync_id = 1U;
    }
    s_ap6256_scan_sync_id = sync_id;
    s_ap6256_scan_active = 1U;
    ap6256_cyw43_populate_dual_band_scan(&scan, sync_id, opts);
    wake_ret = ap6256_cyw43_scan_wake(self, self_in);
    if (wake_ret != 0) {
        ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SCAN_WAKE_EXIT,
                                            wake_ret);
        ap6256_cyw43_port_record_ioctl(SDPCM_SET,
                                       WLC_SET_VAR,
                                       WWD_STA_INTERFACE,
                                       0U,
                                       self->wwd_sdpcm_requested_ioctl_id,
                                       wake_ret,
                                       wake_ret);
        return wake_ret;
    }
    ap6256_cyw43_drain_pending_packets(self);
    wake_ret = cyw43_write_iovar_n(self, "escan", sizeof(scan), &scan, WWD_STA_INTERFACE);
    if (wake_ret != 0) {
        s_ap6256_scan_active = 0U;
    }
    ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_SCAN_RETURN, wake_ret);
    return wake_ret;
}

int cyw43_ll_wifi_join(cyw43_ll_t *self_in, size_t ssid_len, const uint8_t *ssid, size_t key_len, const uint8_t *key, uint32_t auth_type, const uint8_t *bssid, uint32_t channel) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);

    uint8_t buf[2 + CYW43_WPA_SAE_MAX_PASSWORD_LEN];
    int ret;

    ret = ap6256_cyw43_scan_wake(self, self_in);
    if (ret != 0) {
        return ret;
    }
    if (AP6256_CYW43_ABORT_SCAN_BEFORE_JOIN != 0U) {
        ap6256_cyw43_escan_abort(self, ap6256_join_channel_is_5g(channel) != 0U);
    }

    (void)cyw43_write_iovar_u32(self, "ampdu_ba_wsize", 8, WWD_STA_INTERFACE);
    uint32_t wpa_auth = 0;
    uint32_t mfp = MFP_NONE;
    if (auth_type == CYW43_AUTH_OPEN) {
        wpa_auth = 0;
    } else if (auth_type == CYW43_AUTH_WPA2_AES_PSK) {
        wpa_auth = CYW43_WPA2_AUTH_PSK;
    } else if (auth_type == CYW43_AUTH_WPA2_MIXED_PSK) {
        wpa_auth = CYW43_WPA_AUTH_PSK | CYW43_WPA2_AUTH_PSK;
    } else if (auth_type == CYW43_AUTH_WPA_TKIP_PSK) {
        wpa_auth = CYW43_WPA_AUTH_PSK;
    } else if (auth_type == CYW43_AUTH_WPA3_SAE_AES_PSK) {
        wpa_auth = CYW43_WPA3_AUTH_SAE_PSK;
    } else if (auth_type == CYW43_AUTH_WPA3_WPA2_AES_PSK) {
        wpa_auth = CYW43_WPA3_AUTH_SAE_PSK | CYW43_WPA2_AUTH_PSK;
    } else {
        // Unsupported auth_type (security) value.
        return -CYW43_EINVAL;
    }

    // Check key length
    if (auth_type != CYW43_AUTH_OPEN && auth_type != CYW43_AUTH_WPA3_SAE_AES_PSK && key_len > CYW43_WPA_MAX_PASSWORD_LEN) {
        return -CYW43_EINVAL;
    } else if (auth_type == CYW43_AUTH_WPA3_SAE_AES_PSK && key_len > CYW43_WPA_SAE_MAX_PASSWORD_LEN) {
        return -CYW43_EINVAL;
    }

    if (wpa_auth == CYW43_WPA3_AUTH_SAE_PSK) {
        mfp = MFP_REQUIRED;
    } else if ((wpa_auth & CYW43_WPA3_AUTH_SAE_PSK) != 0U) {
        mfp = MFP_CAPABLE;
    }

    bool plain_wpa_psk =
        (mfp == MFP_NONE) &&
        ((wpa_auth & CYW43_WPA3_AUTH_SAE_PSK) == 0U) &&
        ((wpa_auth & (CYW43_WPA_AUTH_PSK | CYW43_WPA2_AUTH_PSK)) != 0U);

    if ((bssid == NULL) &&
        ((channel == CYW43_CHANNEL_NONE) ||
         ((channel > 0U) && (ap6256_join_channel_is_5g(channel) == 0U))) &&
        ((auth_type == CYW43_AUTH_WPA2_AES_PSK) ||
         (auth_type == CYW43_AUTH_WPA2_MIXED_PSK) ||
         (auth_type == CYW43_AUTH_WPA_TKIP_PSK))) {
        /*
         * Conservative SSID-only association path used for the 2.4 GHz
         * regression target. This mirrors the original CYW43 scalar setup and
         * keeps the newer BCM43456 directed/transition-mode machinery out of
         * plain WPA/WPA2 joins.
         */
        ret = cyw43_set_ioctl_u32(self, WLC_SET_WSEC, auth_type & 0xffU, WWD_STA_INTERFACE);
        if ((ret != 0) && !ap6256_join_no_response_ok(ret)) {
            return ret;
        }
        ret = cyw43_write_iovar_u32_u32(self, "bsscfg:sup_wpa", 0U, 1U, WWD_STA_INTERFACE);
        if ((ret != 0) && !ap6256_join_no_response_ok(ret)) {
            return ret;
        }
        ret = cyw43_write_iovar_u32_u32(self, "bsscfg:sup_wpa2_eapver", 0U, (uint32_t)-1, WWD_STA_INTERFACE);
        if ((ret != 0) && !ap6256_join_no_response_ok(ret)) {
            return ret;
        }
        ret = cyw43_write_iovar_u32_u32(self, "bsscfg:sup_wpa_tmo", 0U, CYW_EAPOL_KEY_TIMEOUT, WWD_STA_INTERFACE);
        if ((ret != 0) && !ap6256_join_no_response_ok(ret)) {
            return ret;
        }

        memset(buf, 0, sizeof(buf));
        cyw43_put_le16(buf, key_len);
        cyw43_put_le16(buf + 2, 1);
        memcpy(buf + 4, key, key_len);
        cyw43_delay_ms(2);
        ret = cyw43_do_ioctl(self, SDPCM_SET, WLC_SET_WSEC_PMK, 4 + CYW43_WPA_MAX_PASSWORD_LEN, buf, WWD_STA_INTERFACE);
        if ((ret != 0) && !ap6256_join_no_response_ok(ret)) {
            return ret;
        }

        ret = cyw43_set_ioctl_u32(self, WLC_SET_INFRA, 1, WWD_STA_INTERFACE);
        if ((ret != 0) && !ap6256_join_no_response_ok(ret)) {
            return ret;
        }
        ret = cyw43_set_ioctl_u32(self, WLC_SET_AUTH, AUTH_TYPE_OPEN, WWD_STA_INTERFACE);
        if ((ret != 0) && !ap6256_join_no_response_ok(ret)) {
            return ret;
        }
        ret = cyw43_write_iovar_u32(self,
                                    "mfp",
                                    mfp,
                                    WWD_STA_INTERFACE);
        if ((ret != 0) && !ap6256_join_no_response_ok(ret)) {
            return ret;
        }
        ret = cyw43_set_ioctl_u32(self, WLC_SET_WPA_AUTH, wpa_auth, WWD_STA_INTERFACE);
        if ((ret != 0) && !ap6256_join_no_response_ok(ret)) {
            return ret;
        }

        ret = cyw43_ap6256_program_sta_event_mask(self);
        if ((ret != 0) && !ap6256_join_no_response_ok(ret)) {
            return ret;
        }

        if (channel != CYW43_CHANNEL_NONE) {
            ret = cyw43_set_ioctl_u32(self,
                                      WLC_SET_CHANNEL,
                                      ap6256_primary_channel_from_join_channel(channel),
                                      WWD_STA_INTERFACE);
            if ((ret != 0) && !ap6256_join_no_response_ok(ret)) {
                return ret;
            }
        }

        cyw43_put_le32(self->last_ssid_joined, ssid_len);
        memcpy(self->last_ssid_joined + 4, ssid, ssid_len);
        return cyw43_do_ioctl(self, SDPCM_SET, WLC_SET_SSID, 36, self->last_ssid_joined, WWD_STA_INTERFACE);
    }

    ret = ap6256_program_wpa2_psk_assoc_ie(self, wpa_auth, mfp);
    if (ret != 0) {
        return ret;
    }

    CYW43_VDEBUG("Setting wsec=0x%x\n", auth_type & 0xff);
    ret = cyw43_set_ioctl_u32(self, WLC_SET_WSEC, auth_type & 0xff, WWD_STA_INTERFACE);
    if (ret != 0) {
        if (!ap6256_join_no_response_ok(ret)) {
            return ret;
        }
        ret = 0;
    }

    if (auth_type != CYW43_AUTH_OPEN && auth_type != CYW43_AUTH_WPA3_SAE_AES_PSK) {
        /*
         * Plain WPA/WPA2-PSK should stay on CYW43's original scalar setup:
         * WSEC + WSEC_PMK + WPA_AUTH is enough for firmware supplicant
         * bring-up, and avoids leaking pre-key EAPOL/data to lwIP. Keep the
         * brcmfmac-style supplicant iovars for transition/MFP cases, where
         * BCM43456 needs explicit policy to match the BSS RSN IE.
         */
        if (!plain_wpa_psk) {
            ret = cyw43_write_iovar_u32(self, "sup_wpa", 1, WWD_STA_INTERFACE);
            if (ret != 0) {
                if (!ap6256_join_no_response_ok(ret)) {
                    return ret;
                }
                ret = 0;
            }
            ret = cyw43_write_iovar_u32(self, "sup_wpa2_eapver", (uint32_t)-1, WWD_STA_INTERFACE);
            if (ret != 0) {
                if (!ap6256_join_no_response_ok(ret)) {
                    return ret;
                }
                ret = 0;
            }
            ret = cyw43_write_iovar_u32(self, "sup_wpa_tmo", 2500, WWD_STA_INTERFACE);
            if (ret != 0) {
                if (!ap6256_join_no_response_ok(ret)) {
                    return ret;
                }
                ret = 0;
            }
        }

        memset(buf, 0, sizeof(buf));
        cyw43_put_le16(buf, key_len);
        cyw43_put_le16(buf + 2, 1);
        memcpy(buf + 4, key, key_len);
        cyw43_delay_ms(2);

        CYW43_VDEBUG("Setting wsec_pmk %d\n", key_len);
        ret = cyw43_do_ioctl(self, SDPCM_SET, WLC_SET_WSEC_PMK, 4 + CYW43_WPA_MAX_PASSWORD_LEN, buf, WWD_STA_INTERFACE);
        if (ret != 0) {
            if (!ap6256_join_no_response_ok(ret)) {
                return ret;
            }
            ret = 0;
        }
    }

    if ((wpa_auth & CYW43_WPA3_AUTH_SAE_PSK) != 0U) {
        memset(buf, 0, 2 + CYW43_WPA_SAE_MAX_PASSWORD_LEN);
        cyw43_put_le16(buf, key_len);
        memcpy(buf + 2, key, key_len);
        cyw43_delay_ms(2);
        ret = cyw43_write_iovar_n(self, "sae_password", 2 + CYW43_WPA_SAE_MAX_PASSWORD_LEN, buf, WWD_STA_INTERFACE);
        if (ret != 0) {
            if (!ap6256_join_no_response_ok(ret)) {
                return ret;
            }
            ret = 0;
        }
    }

    CYW43_VDEBUG("Setting infra\n");
    ret = cyw43_set_ioctl_u32(self, WLC_SET_INFRA, 1, WWD_STA_INTERFACE);
    if (ret != 0) {
        if (!ap6256_join_no_response_ok(ret)) {
            return ret;
        }
        ret = 0;
    }

    CYW43_VDEBUG("Setting auth\n");
    ret = cyw43_set_ioctl_u32(self, WLC_SET_AUTH, (wpa_auth == CYW43_WPA3_AUTH_SAE_PSK) ? AUTH_TYPE_SAE : AUTH_TYPE_OPEN, WWD_STA_INTERFACE);
    if (ret != 0) {
        if (!ap6256_join_no_response_ok(ret)) {
            return ret;
        }
        ret = 0;
    }
    if (!plain_wpa_psk) {
        ret = cyw43_write_iovar_u32(self, "auth", (wpa_auth == CYW43_WPA3_AUTH_SAE_PSK) ? AUTH_TYPE_SAE : AUTH_TYPE_OPEN, WWD_STA_INTERFACE);
        if (ret != 0) {
            if (!ap6256_join_no_response_ok(ret)) {
                return ret;
            }
            ret = 0;
        }
    }
    if (!plain_wpa_psk || (mfp != MFP_NONE)) {
        ret = cyw43_write_iovar_u32(self, "mfp", mfp, WWD_STA_INTERFACE);
        if (ret != 0) {
            if (!ap6256_join_no_response_ok(ret)) {
                return ret;
            }
            ret = 0;
        }
    }

    CYW43_VDEBUG("Setting wpa auth 0x%x\n", wpa_auth);
    ret = cyw43_set_ioctl_u32(self, WLC_SET_WPA_AUTH, wpa_auth, WWD_STA_INTERFACE);
    if (ret != 0) {
        if (!ap6256_join_no_response_ok(ret)) {
            return ret;
        }
        ret = 0;
    }
    if (!plain_wpa_psk) {
        ret = cyw43_write_iovar_u32(self, "wpa_auth", wpa_auth, WWD_STA_INTERFACE);
        if (ret != 0) {
            if (!ap6256_join_no_response_ok(ret)) {
                return ret;
            }
            ret = 0;
        }
    }

    ret = cyw43_ap6256_program_sta_event_mask(self);
    if (ret != 0) {
        if (!ap6256_join_no_response_ok(ret)) {
            return ret;
        }
        ret = 0;
    }

    /*
     * Re-apply the RSN IE after the firmware security context is fully
     * programmed. brcmfmac installs the association IE as part of connect
     * setup after choosing WPA/auth/wsec policy; AP6256 transition-mode APs
     * can otherwise leave this MCU path with stale/default assoc capabilities
     * by the time WLC_SET_SSID is issued.
     */
    ret = ap6256_program_wpa2_psk_assoc_ie(self, wpa_auth, mfp);
    if (ret != 0) {
        return ret;
    }

    if ((ap6256_join_channel_is_5g(channel) != 0U) &&
        (AP6256_CYW43_5G_JOIN_QTXPOWER_QDBM != 0U)) {
        /*
         * brcmfmac exposes qtxpower as the normal firmware power-control knob.
         * Cap TX power for the first 5 GHz association frames so AP6256 cannot
         * reset the host while auth/assoc traffic starts. This keeps VHT enabled
         * for the eventual Wi-Fi 5 link; it only reduces initial TX amplitude.
         */
        ret = cyw43_write_iovar_u32(self,
                                    "qtxpower",
                                    AP6256_CYW43_WL_TXPWR_OVERRIDE |
                                        AP6256_CYW43_5G_JOIN_QTXPOWER_QDBM,
                                    WWD_STA_INTERFACE);
        if (ret != 0) {
            if (!ap6256_join_no_response_ok(ret)) {
                return ret;
            }
            ret = 0;
        }
    }

    if ((ap6256_join_channel_is_5g(channel) != 0U) &&
        (AP6256_CYW43_5G_JOIN_SAFE_VHT_OFF != 0U)) {
        /*
         * Keep this explicitly scoped to 5 GHz association stability testing:
         * force the first association attempt down to the most conservative
         * 11a-style feature set. If this stops the reset, the remaining task is
         * a controlled HT/VHT re-enable path; if it still resets, the blocker is
         * below MAC feature negotiation.
         */
        (void)cyw43_write_iovar_u32(self, "mpc", 0U, WWD_STA_INTERFACE);
        (void)cyw43_write_iovar_u32(self, "frameburst", 0U, WWD_STA_INTERFACE);
        (void)cyw43_write_iovar_u32(self, "ampdu", 0U, WWD_STA_INTERFACE);
        (void)cyw43_write_iovar_u32(self, "amsdu", 0U, WWD_STA_INTERFACE);
        (void)cyw43_write_iovar_u32(self, "nmode", 0U, WWD_STA_INTERFACE);
        ret = cyw43_write_iovar_u32(self, "vhtmode", 0U, WWD_STA_INTERFACE);
        if (ret != 0) {
            if (!ap6256_join_no_response_ok(ret)) {
                return ret;
            }
            ret = 0;
        }
        (void)cyw43_write_iovar_u32(self, "mimo_bw_cap", 0U, WWD_STA_INTERFACE);
        (void)cyw43_write_iovar_u32(self, "ampdu_ba_wsize", 2U, WWD_STA_INTERFACE);
        (void)cyw43_write_iovar_u32(self, "ampdu_mpdu", 1U, WWD_STA_INTERFACE);
    }

    cyw43_put_le32(self->last_ssid_joined, ssid_len);
    memcpy(self->last_ssid_joined + 4, ssid, ssid_len);
    if ((channel != CYW43_CHANNEL_NONE) && (AP6256_CYW43_ENABLE_JOIN_PREF != 0U)) {
        size_t join_pref_len = ap6256_build_join_pref_5g(buf, sizeof(buf));

        if (join_pref_len != 0U) {
            if (ap6256_join_channel_is_5g(channel) == 0U) {
                /*
                 * brcmfmac resets join preference to default unless cfg80211
                 * asks for band preference. This prevents a previous 5 GHz run
                 * from biasing the 2.4 GHz regression target.
                 */
                memset(buf, 0, join_pref_len);
                buf[0] = 1U; /* BRCMF_JOIN_PREF_RSSI */
                buf[1] = 2U;
            }
            ret = cyw43_write_iovar_n(self, "join_pref", join_pref_len, buf, WWD_STA_INTERFACE);
            if (ret != 0) {
                if (!ap6256_join_no_response_ok(ret)) {
                    return ret;
                }
                ret = 0;
            }
        }
    }

    if ((AP6256_CYW43_5G_JOIN_SET_CHANNEL_STARTER != 0U) &&
        (channel != CYW43_CHANNEL_NONE)) {
        uint32_t starter_channel = ap6256_primary_channel_from_join_channel(channel);

        /*
         * brcmfmac primes firmware with the requested channel before connect
         * when cfg80211 supplies one. Even when the association payload carries
         * a full chanspec, the starter command is the primary channel number.
         * Do not force the band here; keep band selection implicit so BCM43456
         * can use the directed chanspec/BSSID evidence in the following
         * WLC_SET_SSID payload.
         */
        ret = cyw43_set_ioctl_u32(self, WLC_SET_CHANNEL, starter_channel, WWD_STA_INTERFACE);
        if (ret != 0) {
            if (!ap6256_join_no_response_ok(ret)) {
                return ret;
            }
            ret = 0;
        }
    }

    if ((bssid == NULL) &&
        (ap6256_join_channel_is_5g(channel) != 0U) &&
        (AP6256_CYW43_5G_BROADCAST_EXT_JOIN != 0U)) {
        size_t ext_join_len = ap6256_build_brcmf_ext_join_params(buf,
                                                                 sizeof(buf),
                                                                 ssid_len,
                                                                 ssid,
                                                                 NULL,
                                                                 channel);
        if (ext_join_len == 0U) {
            return -CYW43_EINVAL;
        }

        /*
         * Broadcast-BSSID ext_join mirrors brcmfmac's channel-directed connect
         * path without using the BSSID/chanspec payload shape that resets this
         * AP6256 board. If firmware accepts the control TX, join completion is
         * still proved by events/DHCP above this layer.
         */
        CYW43_VDEBUG("Join iovar broadcast 5g\n");
        ret = cyw43_write_iovar_n(self, "join", ext_join_len, buf, WWD_STA_INTERFACE);
        if (ret == 0) {
            return 0;
        }
    }

    if ((bssid != NULL) && (AP6256_CYW43_DIRECTED_ASSOC_PAYLOAD != 0U)) {
        size_t set_ssid_len;

        if (AP6256_CYW43_USE_EXT_JOIN_IOVAR != 0U) {
            size_t ext_join_len = ap6256_build_brcmf_ext_join_params(buf,
                                                                     sizeof(buf),
                                                                     ssid_len,
                                                                     ssid,
                                                                     bssid,
                                                                     channel);
            if (ext_join_len == 0U) {
                return -CYW43_EINVAL;
            }

            /*
             * Keep brcmfmac's ext_join available as a bench toggle. The normal
             * AP6256 path currently prefers directed WLC_SET_SSID because the
             * firmware emits ESCAN_RESULT(status=ABORT) during ext_join and can
             * reset while scan/join overlap is active.
             */
            CYW43_VDEBUG("Join iovar directed\n");
            ret = cyw43_write_iovar_n(self, "join", ext_join_len, buf, WWD_STA_INTERFACE);
            if (ret == 0) {
                return 0;
            }
            if ((AP6256_CYW43_5G_JOIN_IOVAR_ONLY != 0U) &&
                (ap6256_join_channel_is_5g(channel) != 0U)) {
                return ret;
            }
        }

        /*
         * Use brcmfmac's directed WLC_SET_SSID/brcmf_join_params fallback:
         * SSID + BSSID + a single chanspec. When the runtime has a scanned
         * BCM43456 chanspec it is passed through with CYW43_CHANNEL_FROM_CHANSPEC;
         * otherwise this helper derives the normal 20 MHz primary-channel
         * chanspec. Success is not inferred from TX-only completion; the runtime
         * still requires a control completion or link/auth event, then DHCP.
         */
        set_ssid_len = ap6256_build_brcmf_join_params(buf,
                                                      sizeof(buf),
                                                      ssid_len,
                                                      ssid,
                                                      bssid,
                                                      channel);
        if (set_ssid_len == 0U) {
            return -CYW43_EINVAL;
        }
        CYW43_VDEBUG("Set ssid directed\n");
        ret = cyw43_do_ioctl(self, SDPCM_SET, WLC_SET_SSID, set_ssid_len, buf, WWD_STA_INTERFACE);
    } else {
        // join SSID
        CYW43_VDEBUG("Set ssid\n");
        ret = cyw43_do_ioctl(self, SDPCM_SET, WLC_SET_SSID, 36, self->last_ssid_joined, WWD_STA_INTERFACE);
    }

    return ret;
}

void cyw43_ll_wifi_set_wpa_auth(cyw43_ll_t *self_in) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);
    cyw43_set_ioctl_u32(self, WLC_SET_WPA_AUTH, CYW43_WPA_AUTH_PSK, WWD_STA_INTERFACE);
}

void cyw43_ll_wifi_rejoin(cyw43_ll_t *self_in) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);
    cyw43_do_ioctl(self, SDPCM_SET, WLC_SET_SSID, 36, self->last_ssid_joined, WWD_STA_INTERFACE);
}

int cyw43_ll_wifi_get_bssid(cyw43_ll_t *self_in, uint8_t *bssid) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);
    return cyw43_do_ioctl(self, SDPCM_GET, WLC_GET_BSSID, 6, bssid, WWD_STA_INTERFACE);
}

/*******************************************************************************/
// WiFi AP

int cyw43_ll_wifi_ap_init(cyw43_ll_t *self_in, size_t ssid_len, const uint8_t *ssid, uint32_t auth, size_t key_len, const uint8_t *key, uint32_t channel) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);

    uint8_t *buf = &self->spid_buf[SDPCM_HEADER_LEN + 16];

    // Get state of AP
    // TODO: this can fail with sdpcm status = 0xffffffe2 (NOTASSOCIATED)
    // in such a case the AP is not up and we should not check the result
    memcpy(buf, "bss\x00", 4);
    cyw43_put_le32(buf + 4, WWD_AP_INTERFACE);
    cyw43_do_ioctl(self, SDPCM_GET, WLC_GET_VAR, 8, buf, WWD_STA_INTERFACE);
    uint32_t res = cyw43_get_le32(buf); // le or be?

    if (res) {
        // already up ...
        return 0;
    }

    // Check key length
    if (auth != CYW43_AUTH_OPEN && auth != CYW43_AUTH_WPA3_SAE_AES_PSK && key_len > CYW43_WPA_MAX_PASSWORD_LEN) {
        return -CYW43_EINVAL;
    } else if (auth == CYW43_AUTH_WPA3_SAE_AES_PSK && key_len > CYW43_WPA_SAE_MAX_PASSWORD_LEN) {
        return -CYW43_EINVAL;
    }

    // set the AMPDU parameter for AP (window size = 2)
    cyw43_write_iovar_u32(self, "ampdu_ba_wsize", 2, WWD_STA_INTERFACE);

    // set SSID
    cyw43_put_le32(buf, WWD_AP_INTERFACE);
    cyw43_put_le32(buf + 4, ssid_len);
    memset(buf + 8, 0, 32);
    memcpy(buf + 8, ssid, ssid_len);
    cyw43_write_iovar_n(self, "bsscfg:ssid", 4 + 4 + 32, buf, WWD_STA_INTERFACE);

    // set channel
    cyw43_set_ioctl_u32(self, WLC_SET_CHANNEL, channel, WWD_STA_INTERFACE);

    // set security type
    cyw43_write_iovar_u32_u32(self, "bsscfg:wsec", WWD_AP_INTERFACE, auth, WWD_STA_INTERFACE);

    // set management frame protection
    uint32_t auth_mfp = MFP_NONE;
    if (auth == CYW43_AUTH_WPA3_SAE_AES_PSK) {
        auth_mfp = MFP_REQUIRED;
    } else if (auth == CYW43_AUTH_WPA3_WPA2_AES_PSK) {
        auth_mfp = MFP_CAPABLE;
    }
    cyw43_write_iovar_u32(self, "mfp", auth_mfp, WWD_AP_INTERFACE);

    if (auth == CYW43_AUTH_OPEN) {
        // nothing to do
    } else {
        // set WPA/WPA2 auth params
        uint32_t val = 0;
        if (auth == CYW43_AUTH_WPA_TKIP_PSK) {
            val = CYW43_WPA_AUTH_PSK;
        } else if (auth == CYW43_AUTH_WPA2_AES_PSK || auth == CYW43_AUTH_WPA2_MIXED_PSK) {
            val = CYW43_WPA_AUTH_PSK | CYW43_WPA2_AUTH_PSK;
        } else if (auth == CYW43_AUTH_WPA3_SAE_AES_PSK) {
            val = CYW43_WPA3_AUTH_SAE_PSK;
        } else if (auth == CYW43_AUTH_WPA3_WPA2_AES_PSK) {
            val = CYW43_WPA3_AUTH_SAE_PSK | CYW43_WPA2_AUTH_PSK;
        }
        assert(val);
        cyw43_write_iovar_u32_u32(self, "bsscfg:wpa_auth", WWD_AP_INTERFACE, val, WWD_STA_INTERFACE);

        // set password
        if (val & CYW43_WPA3_AUTH_SAE_PSK) {
            cyw43_put_le16(buf, key_len);
            memset(buf + 2, 0, CYW43_WPA_SAE_MAX_PASSWORD_LEN);
            memcpy(buf + 2, key, key_len);
            cyw43_delay_ms(2); // Delay required to allow radio firmware to be ready to receive PMK and avoid intermittent failure
            cyw43_write_iovar_n(self, "sae_password", 2 + CYW43_WPA_SAE_MAX_PASSWORD_LEN, buf, WWD_AP_INTERFACE);
        }
        if (val & (CYW43_WPA_AUTH_PSK | CYW43_WPA2_AUTH_PSK)) {
            cyw43_put_le16(buf, key_len);
            cyw43_put_le16(buf + 2, 1);
            memset(buf + 4, 0, CYW43_WPA_MAX_PASSWORD_LEN);
            memcpy(buf + 4, key, key_len);
            cyw43_delay_ms(2); // WICED has this
            cyw43_do_ioctl(self, SDPCM_SET, WLC_SET_WSEC_PMK, 2 + 2 + CYW43_WPA_MAX_PASSWORD_LEN, buf, WWD_AP_INTERFACE);
        }
    }

    if (auth == CYW43_AUTH_WPA3_SAE_AES_PSK || auth == CYW43_AUTH_WPA3_WPA2_AES_PSK) {
        cyw43_write_iovar_u32(self, "sae_max_pwe_loop", WWD_AP_INTERFACE, 5);
    }

    // set GMode to auto (value of 1)
    cyw43_set_ioctl_u32(self, WLC_SET_GMODE, 1, WWD_AP_INTERFACE);

    // set multicast tx rate to 11Mbps
    cyw43_write_iovar_u32(self, "2g_mrate", 11000000 / 500000, WWD_AP_INTERFACE);

    // set DTIM period
    cyw43_set_ioctl_u32(self, WLC_SET_DTIMPRD, 1, WWD_AP_INTERFACE);

    return 0;
}

int cyw43_ll_wifi_ap_set_up(cyw43_ll_t *self_in, bool up) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);

    cyw43_write_iovar_u32_u32(self, "bss", WWD_AP_INTERFACE, up, WWD_STA_INTERFACE);

    return 0;
}

int cyw43_ll_wifi_ap_get_stas(cyw43_ll_t *self_in, int *num_stas, uint8_t *macs) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);

    uint8_t *buf = &self->spid_buf[SDPCM_HEADER_LEN + 16];

    // Get max number of associated STAs
    memcpy(buf, "maxassoc\x00", 9);
    cyw43_put_le32(buf + 9, WWD_AP_INTERFACE);
    cyw43_do_ioctl(self, SDPCM_GET, WLC_GET_VAR, 9 + 4, buf, WWD_STA_INTERFACE);
    int err = cyw43_do_ioctl(self, SDPCM_GET, WLC_GET_VAR, 9 + 4, buf, WWD_STA_INTERFACE);
    if (err != 0) {
        return err;
    }
    uint32_t maxassoc = cyw43_get_le32(buf);
    uint32_t max_macs_buf = (sizeof(self->spid_buf) - SDPCM_HEADER_LEN - 16 - 4) / 6;
    maxassoc = MIN(maxassoc, max_macs_buf);

    if (macs == NULL) {
        // Return just the maximum number of STAs
        *num_stas = maxassoc;
        return 0;
    }

    // Make sure all MACs will fit in buffer; size specified in num_stas
    maxassoc = MIN(maxassoc, (uint32_t)*num_stas);

    // Get associated STAs
    cyw43_put_le32(buf, maxassoc);
    err = cyw43_do_ioctl(self, SDPCM_GET, WLC_GET_ASSOCLIST, 4 + maxassoc * 6, buf, WWD_AP_INTERFACE);
    if (err != 0) {
        return err;
    }
    uint32_t stas_connected = cyw43_get_le32(buf);
    *num_stas = MIN(stas_connected, maxassoc);
    memcpy(macs, buf + 4, *num_stas * 6);
    return 0;
}

#if CYW43_GPIO

int cyw43_ll_gpio_set(cyw43_ll_t *self_in, int gpio_n, bool gpio_en) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);
    if (!(gpio_n >= 0 && gpio_n < CYW43_NUM_GPIOS)) {
        return -1;
    }
    CYW43_VDEBUG("cyw43_set_gpio %d=%d\n", gpio_n, gpio_en);
    cyw43_write_iovar_u32_u32(self, "gpioout", 1 << gpio_n, gpio_en ? (1 << gpio_n) : 0, WWD_STA_INTERFACE);
    return 0;
}

int cyw43_ll_gpio_get(cyw43_ll_t *self_in, int gpio_n, bool *gpio_en) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);
    uint8_t *buf = &self->spid_buf[SDPCM_HEADER_LEN + 16];

    if (!(gpio_n >= 0 && gpio_n < CYW43_NUM_GPIOS)) {
        return -1;
    }

    memcpy(buf, "ccgpioin\x00", 9);
    int err = cyw43_do_ioctl(self, SDPCM_GET, WLC_GET_VAR, 9, buf, WWD_STA_INTERFACE);
    if (err != 0) {
        return err;
    }

    *gpio_en = (cyw43_get_le32(buf) & (1 << gpio_n)) ? true : false;
    CYW43_VDEBUG("cyw43_get_gpio %d=%s\n", gpio_n, (gpio_en ? "true" : "false"));
    return 0;
}

#endif

// Is there work to do?
bool cyw43_ll_has_work(cyw43_ll_t *self_in) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);
    #if !CYW43_USE_SPI
    return self->had_successful_packet || cyw43_ll_sdio_packet_pending(self);
    #else
    int int_pin = cyw43_cb_read_host_interrupt_pin(self->cb_data);
    return int_pin == host_interrupt_pin_active;
    #endif
}

#if CYW43_ENABLE_BLUETOOTH

bool cyw43_ll_bt_has_work(cyw43_ll_t *self_in) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);
    uint32_t int_status = cyw43_read_backplane(self, SDIO_INT_STATUS, 4);
    if (int_status & I_HMB_FC_CHANGE) {
        cyw43_write_backplane(self, SDIO_INT_STATUS, 4, int_status & I_HMB_FC_CHANGE);
        return true;
    }
    return false;
}

void cyw43_ll_write_backplane_reg(cyw43_ll_t *self_in, uint32_t addr, uint32_t val) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);
    cyw43_write_backplane(self, addr, sizeof(uint32_t), val);
}

uint32_t cyw43_ll_read_backplane_reg(cyw43_ll_t *self_in, uint32_t addr) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);
    return cyw43_read_backplane(self, addr, sizeof(uint32_t));
}

int cyw43_ll_write_backplane_mem(cyw43_ll_t *self_in, uint32_t addr, uint32_t len, const uint8_t *buf) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);
    while (len > 0) {
        const uint32_t backplane_addr_start = addr & BACKPLANE_ADDR_MASK;
        const uint32_t backplane_addr_end = MIN(backplane_addr_start + len, BACKPLANE_ADDR_MASK + 1);
        const uint32_t backplane_len = backplane_addr_end - backplane_addr_start;
        cyw43_set_backplane_window(self, addr);
        int ret = cyw43_write_bytes(self, BACKPLANE_FUNCTION, backplane_addr_start | SBSDIO_SB_ACCESS_2_4B_FLAG, backplane_len, buf);
        if (ret != 0) {
            CYW43_PRINTF("backplane write 0x%lx,0x%lx failed", addr, backplane_len);
        }
        addr += backplane_len;
        buf += backplane_len;
        len -= backplane_len;
    }
    cyw43_set_backplane_window(self, CHIPCOMMON_BASE_ADDRESS);
    return 0;
}

int cyw43_ll_read_backplane_mem(cyw43_ll_t *self_in, uint32_t addr, uint32_t len, uint8_t *buf) {
    cyw43_int_t *self = CYW_INT_FROM_LL(self_in);
    assert(len <= CYW43_BUS_MAX_BLOCK_SIZE);
    assert(((addr & BACKPLANE_ADDR_MASK) + len) <= (BACKPLANE_ADDR_MASK + 1));
    cyw43_set_backplane_window(self, addr);
    int ret = cyw43_read_bytes(self, BACKPLANE_FUNCTION, (addr & BACKPLANE_ADDR_MASK) | SBSDIO_SB_ACCESS_2_4B_FLAG, len, buf);
    cyw43_set_backplane_window(self, CHIPCOMMON_BASE_ADDRESS);
    return ret;
}

#endif
