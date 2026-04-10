#ifndef AP6256_CYW43_COMPAT_H
#define AP6256_CYW43_COMPAT_H

#include <stdint.h>

/*
 * BCM43456 reports itself as the BCM4345 family with revision 9 during
 * backplane bring-up. Keep the runtime locked to that exact family/revision
 * pair so we fail early on unexpected silicon instead of stumbling deeper into
 * firmware boot with 43439-era defaults.
 */
#define AP6256_CYW43_EXPECTED_CHIP_ID        0x4345U
#define AP6256_CYW43_EXPECTED_CHIP_REV       9U
/*
 * The vendored CYW43 path we are hardening is the older CM3/SOCRAM-style
 * bring-up, not the Linux CR4 buscore path. Keep firmware/NVRAM download at
 * offset 0 within dongle RAM for this runtime, while still retaining the larger
 * BCM4345/9 RAM size.
 */
#define AP6256_CYW43_RAM_BASE               0x00000000UL
#define AP6256_CYW43_TCM_RAM_BASE           0x00198000UL
#define AP6256_CYW43_RAM_SIZE_BYTES          0x000C8000UL

#define AP6256_CYW43_CORE_CHIPCOMMON        0x0800U
#define AP6256_CYW43_CORE_INTERNAL_MEM      0x080EU
#define AP6256_CYW43_CORE_80211             0x0812U
#define AP6256_CYW43_CORE_SDIO_DEV          0x0829U
#define AP6256_CYW43_CORE_ARM_CM3           0x082AU
#define AP6256_CYW43_CORE_ARM_CR4           0x083EU
#define AP6256_CYW43_CORE_SYS_MEM           0x0849U

typedef enum {
    AP6256_CYW43_PROFILE_BASELINE = 0U,
    AP6256_CYW43_PROFILE_BASELINE_PLUS_REMAP,
    AP6256_CYW43_PROFILE_BASELINE_PLUS_PMU,
    AP6256_CYW43_PROFILE_BASELINE_PLUS_REMAP_PMU,
    AP6256_CYW43_PROFILE_BASELINE_PLUS_CARDCTRL,
    AP6256_CYW43_PROFILE_BASELINE_PLUS_CARDCTRL_PMU,
    AP6256_CYW43_PROFILE_BASELINE_PLUS_TCM,
    AP6256_CYW43_PROFILE_BASELINE_PLUS_TCM_PMU
} ap6256_cyw43_compat_profile_t;

typedef enum {
    AP6256_CYW43_BOOT_CM3_SOCRAM = 0U,
    AP6256_CYW43_BOOT_CR4_TCM
} ap6256_cyw43_boot_mode_t;

typedef enum {
    AP6256_CYW43_CP_RESULT_PENDING = 0U,
    AP6256_CYW43_CP_RESULT_OK,
    AP6256_CYW43_CP_RESULT_FAIL,
    AP6256_CYW43_CP_RESULT_TIMEOUT
} ap6256_cyw43_checkpoint_result_t;

typedef enum {
    AP6256_CYW43_CP_NONE = 0U,
    AP6256_CYW43_CP_ALP_REQUEST,
    AP6256_CYW43_CP_ALP_CONFIRM,
    AP6256_CYW43_CP_ALP_CLEAR,
    AP6256_CYW43_CP_FN_ENABLE,
    AP6256_CYW43_CP_CARDCTRL,
    AP6256_CYW43_CP_PMU_RELOAD,
    AP6256_CYW43_CP_SEP_INT,
    AP6256_CYW43_CP_INT_MASK,
    AP6256_CYW43_CP_TOPOLOGY,
    AP6256_CYW43_CP_WLAN_DISABLE,
    AP6256_CYW43_CP_SOCRAM_DISABLE,
    AP6256_CYW43_CP_SOCRAM_RESET,
    AP6256_CYW43_CP_SOCRAM_BANKIDX,
    AP6256_CYW43_CP_SOCRAM_BANKPDA,
    AP6256_CYW43_CP_FW_DOWNLOAD,
    AP6256_CYW43_CP_FW_VERIFY,
    AP6256_CYW43_CP_NVRAM_DOWNLOAD,
    AP6256_CYW43_CP_NVRAM_VERIFY,
    AP6256_CYW43_CP_NVRAM_FOOTER,
    AP6256_CYW43_CP_FOOTER_VERIFY,
    AP6256_CYW43_CP_CPU_HALT,
    AP6256_CYW43_CP_RESET_VECTOR,
    AP6256_CYW43_CP_CPU_RELEASE,
    AP6256_CYW43_CP_WLAN_RESET,
    AP6256_CYW43_CP_HT_WAKE,
    AP6256_CYW43_CP_HT_REQUEST,
    AP6256_CYW43_CP_HT_WAIT_REQ,
    AP6256_CYW43_CP_HT_WAIT_FORCE,
    AP6256_CYW43_CP_F2_WAIT,
    AP6256_CYW43_CP_CLM_LOAD,
    AP6256_CYW43_CP_TXGLOM,
    AP6256_CYW43_CP_APSTA,
    AP6256_CYW43_CP_POLL_HEADER,
    AP6256_CYW43_CP_POLL_PAYLOAD,
    AP6256_CYW43_CP_POLL_PARSE
} ap6256_cyw43_checkpoint_t;

static inline uint16_t ap6256_cyw43_chip_id_from_raw(uint32_t chip_id_raw)
{
    return (uint16_t)(chip_id_raw & 0xFFFFU);
}

static inline uint8_t ap6256_cyw43_chip_rev_from_raw(uint32_t chip_id_raw)
{
    return (uint8_t)((chip_id_raw >> 16U) & 0x0FU);
}

static inline uint8_t ap6256_cyw43_chip_supported(uint32_t chip_id_raw)
{
    return (uint8_t)((ap6256_cyw43_chip_id_from_raw(chip_id_raw) == AP6256_CYW43_EXPECTED_CHIP_ID) &&
                     (ap6256_cyw43_chip_rev_from_raw(chip_id_raw) == AP6256_CYW43_EXPECTED_CHIP_REV));
}

static inline const char *ap6256_cyw43_checkpoint_name(uint32_t checkpoint)
{
    switch ((ap6256_cyw43_checkpoint_t)checkpoint) {
    case AP6256_CYW43_CP_ALP_REQUEST:
        return "alp_req";
    case AP6256_CYW43_CP_ALP_CONFIRM:
        return "alp_ok";
    case AP6256_CYW43_CP_ALP_CLEAR:
        return "alp_clr";
    case AP6256_CYW43_CP_FN_ENABLE:
        return "fn_en";
    case AP6256_CYW43_CP_CARDCTRL:
        return "cardctl";
    case AP6256_CYW43_CP_PMU_RELOAD:
        return "pmu";
    case AP6256_CYW43_CP_SEP_INT:
        return "sep_int";
    case AP6256_CYW43_CP_INT_MASK:
        return "int_mask";
    case AP6256_CYW43_CP_TOPOLOGY:
        return "topology";
    case AP6256_CYW43_CP_WLAN_DISABLE:
        return "wl_dis";
    case AP6256_CYW43_CP_SOCRAM_DISABLE:
        return "sr_dis";
    case AP6256_CYW43_CP_SOCRAM_RESET:
        return "sr_rst";
    case AP6256_CYW43_CP_SOCRAM_BANKIDX:
        return "sr_idx";
    case AP6256_CYW43_CP_SOCRAM_BANKPDA:
        return "sr_pda";
    case AP6256_CYW43_CP_FW_DOWNLOAD:
        return "fw";
    case AP6256_CYW43_CP_FW_VERIFY:
        return "fw_verify";
    case AP6256_CYW43_CP_NVRAM_DOWNLOAD:
        return "nvram";
    case AP6256_CYW43_CP_NVRAM_VERIFY:
        return "nvram_verify";
    case AP6256_CYW43_CP_NVRAM_FOOTER:
        return "nv_footer";
    case AP6256_CYW43_CP_FOOTER_VERIFY:
        return "footer_verify";
    case AP6256_CYW43_CP_CPU_HALT:
        return "cpu_halt";
    case AP6256_CYW43_CP_RESET_VECTOR:
        return "reset_vector";
    case AP6256_CYW43_CP_CPU_RELEASE:
        return "cpu_release";
    case AP6256_CYW43_CP_WLAN_RESET:
        return "wlan_rst";
    case AP6256_CYW43_CP_HT_WAKE:
        return "ht_wake";
    case AP6256_CYW43_CP_HT_REQUEST:
        return "ht_req";
    case AP6256_CYW43_CP_HT_WAIT_REQ:
        return "ht_wait_req";
    case AP6256_CYW43_CP_HT_WAIT_FORCE:
        return "ht_wait_force";
    case AP6256_CYW43_CP_F2_WAIT:
        return "f2_wait";
    case AP6256_CYW43_CP_CLM_LOAD:
        return "clm";
    case AP6256_CYW43_CP_TXGLOM:
        return "txglom";
    case AP6256_CYW43_CP_APSTA:
        return "apsta";
    case AP6256_CYW43_CP_POLL_HEADER:
        return "poll_hdr";
    case AP6256_CYW43_CP_POLL_PAYLOAD:
        return "poll_pay";
    case AP6256_CYW43_CP_POLL_PARSE:
        return "poll_parse";
    case AP6256_CYW43_CP_NONE:
    default:
        return "none";
    }
}

static inline const char *ap6256_cyw43_boot_mode_name(uint32_t boot_mode)
{
    switch ((ap6256_cyw43_boot_mode_t)boot_mode) {
    case AP6256_CYW43_BOOT_CR4_TCM:
        return "cr4_tcm";
    case AP6256_CYW43_BOOT_CM3_SOCRAM:
    default:
        return "cm3_socram";
    }
}

static inline const char *ap6256_cyw43_core_name(uint32_t core_id)
{
    switch ((uint16_t)core_id) {
    case AP6256_CYW43_CORE_ARM_CR4:
        return "cr4";
    case AP6256_CYW43_CORE_ARM_CM3:
        return "cm3";
    case AP6256_CYW43_CORE_INTERNAL_MEM:
        return "socram";
    case AP6256_CYW43_CORE_SYS_MEM:
        return "sysmem";
    case AP6256_CYW43_CORE_SDIO_DEV:
        return "sdio";
    case AP6256_CYW43_CORE_80211:
        return "d11";
    case AP6256_CYW43_CORE_CHIPCOMMON:
        return "chipc";
    default:
        return "unknown";
    }
}

static inline const char *ap6256_cyw43_checkpoint_result_name(uint32_t result)
{
    switch ((ap6256_cyw43_checkpoint_result_t)result) {
    case AP6256_CYW43_CP_RESULT_OK:
        return "ok";
    case AP6256_CYW43_CP_RESULT_FAIL:
        return "fail";
    case AP6256_CYW43_CP_RESULT_TIMEOUT:
        return "timeout";
    case AP6256_CYW43_CP_RESULT_PENDING:
    default:
        return "pending";
    }
}

static inline const char *ap6256_cyw43_profile_name(uint32_t profile)
{
    switch ((ap6256_cyw43_compat_profile_t)profile) {
    case AP6256_CYW43_PROFILE_BASELINE_PLUS_REMAP:
        return "baseline_plus_remap";
    case AP6256_CYW43_PROFILE_BASELINE_PLUS_PMU:
        return "baseline_plus_pmu";
    case AP6256_CYW43_PROFILE_BASELINE_PLUS_REMAP_PMU:
        return "baseline_plus_remap_pmu";
    case AP6256_CYW43_PROFILE_BASELINE_PLUS_CARDCTRL:
        return "baseline_plus_cardctrl";
    case AP6256_CYW43_PROFILE_BASELINE_PLUS_CARDCTRL_PMU:
        return "baseline_plus_cardctrl_pmu";
    case AP6256_CYW43_PROFILE_BASELINE_PLUS_TCM:
        return "baseline_plus_tcm";
    case AP6256_CYW43_PROFILE_BASELINE_PLUS_TCM_PMU:
        return "baseline_plus_tcm_pmu";
    case AP6256_CYW43_PROFILE_BASELINE:
    default:
        return "baseline";
    }
}

static inline uint8_t ap6256_cyw43_profile_enable_pmu(uint32_t profile)
{
    return (uint8_t)(((profile == AP6256_CYW43_PROFILE_BASELINE_PLUS_PMU) ||
                      (profile == AP6256_CYW43_PROFILE_BASELINE_PLUS_REMAP_PMU) ||
                      (profile == AP6256_CYW43_PROFILE_BASELINE_PLUS_CARDCTRL_PMU) ||
                      (profile == AP6256_CYW43_PROFILE_BASELINE_PLUS_TCM_PMU)) ? 1U : 0U);
}

static inline uint8_t ap6256_cyw43_profile_enable_remap(uint32_t profile)
{
    return (uint8_t)(((profile == AP6256_CYW43_PROFILE_BASELINE_PLUS_REMAP) ||
                      (profile == AP6256_CYW43_PROFILE_BASELINE_PLUS_REMAP_PMU)) ? 1U : 0U);
}

static inline uint8_t ap6256_cyw43_profile_enable_cardctrl(uint32_t profile)
{
    return (uint8_t)(((profile == AP6256_CYW43_PROFILE_BASELINE_PLUS_CARDCTRL) ||
                      (profile == AP6256_CYW43_PROFILE_BASELINE_PLUS_CARDCTRL_PMU)) ? 1U : 0U);
}

static inline uint32_t ap6256_cyw43_profile_ram_base(uint32_t profile)
{
    switch ((ap6256_cyw43_compat_profile_t)profile) {
    case AP6256_CYW43_PROFILE_BASELINE_PLUS_TCM:
    case AP6256_CYW43_PROFILE_BASELINE_PLUS_TCM_PMU:
        return AP6256_CYW43_TCM_RAM_BASE;
    default:
        return AP6256_CYW43_RAM_BASE;
    }
}

static inline uint32_t ap6256_cyw43_profile_boot_mode(uint32_t profile)
{
    switch ((ap6256_cyw43_compat_profile_t)profile) {
    case AP6256_CYW43_PROFILE_BASELINE_PLUS_TCM:
    case AP6256_CYW43_PROFILE_BASELINE_PLUS_TCM_PMU:
        return AP6256_CYW43_BOOT_CR4_TCM;
    default:
        return AP6256_CYW43_BOOT_CM3_SOCRAM;
    }
}

#endif /* AP6256_CYW43_COMPAT_H */
