#include "ap6256_cyw43_port.h"

#include "ap6256_cyw43_compat.h"
#include "ap6256_driver.h"
#include "cyw43_configport.h"
#include "generated/ap6256_assets_manifest.h"
#include "main.h"

#include "cmsis_os2.h"
#include "stm32h7xx_ll_sdmmc.h"

#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#define AP6256_CYW43_SDIO_CMD_TIMEOUT_MS 120U
#define AP6256_CYW43_SDIO_READY_TIMEOUT_MS 500U
#define AP6256_CYW43_SDIO_CMD_GUARD_LOOPS 2000000UL
#define AP6256_CYW43_WL_REG_ON_PORT GPIOC
#define AP6256_CYW43_WL_REG_ON_PIN  GPIO_PIN_7
#define AP6256_CYW43_SDIO_DAT1_PORT GPIOC
#define AP6256_CYW43_SDIO_DAT1_PIN  GPIO_PIN_9
#define AP6256_CYW43_BREADCRUMB_MAGIC 0xA6256C43UL
#define AP6256_CYW43_PRE_RESET_MAGIC 0xA6256D43UL
#define AP6256_CYW43_PRE_RESET_THROTTLE_MS 25U
#define AP6256_CYW43_NOISY_BREADCRUMB_PERSIST_MS 250U

static osMutexId_t s_cyw43_mutex;
static volatile uint8_t s_poll_pending;
static uint8_t s_sdio_open;
extern void (*cyw43_poll)(void);
static volatile uint32_t s_cyw43_sched_calls;
static volatile uintptr_t s_cyw43_last_sched_func;
static volatile uint32_t s_cyw43_sdio_init_calls;
static volatile uint32_t s_cyw43_sdio_reinit_calls;
static volatile int32_t s_cyw43_last_bus_init_ret;
static volatile uint32_t s_cyw43_bus_stage;
static volatile int32_t s_cyw43_poll_header_read_status;
static volatile int32_t s_cyw43_poll_payload_read_status;
static volatile uint16_t s_cyw43_poll_hdr0;
static volatile uint16_t s_cyw43_poll_hdr1;
static volatile uint8_t s_cyw43_last_cmd53_write;
static volatile uint8_t s_cyw43_last_cmd53_function;
static volatile uint8_t s_cyw43_last_cmd53_block_mode;
static volatile uint32_t s_cyw43_last_cmd53_block_size;
static volatile uint32_t s_cyw43_last_cmd53_length;
static volatile int32_t s_cyw43_last_cmd53_status;
static volatile uint16_t s_cyw43_last_cmd53_frame_size;
static volatile uint32_t s_cyw43_last_cmd53_count;
static volatile uint32_t s_cyw43_runtime_f2_block_size = AP6256_CYW43_SDIO_F2_BLOCK_SIZE;
static volatile uint32_t s_cyw43_last_cmd;
static volatile uint32_t s_cyw43_last_cmd_arg;
static volatile int32_t s_cyw43_last_cmd_status;
static volatile uint32_t s_cyw43_last_cmd_response;
static volatile uint32_t s_cyw43_chip_id_raw;
static volatile uint32_t s_cyw43_ram_base_addr;
static volatile uint32_t s_cyw43_ram_size_bytes;
static volatile uint32_t s_cyw43_boot_mode;
static volatile uint32_t s_cyw43_cpu_wrapper_addr;
static volatile uint32_t s_cyw43_ram_wrapper_addr;
static volatile uint32_t s_cyw43_firmware_addr;
static volatile uint32_t s_cyw43_nvram_addr;
static volatile uint32_t s_cyw43_footer_addr;
static volatile uint32_t s_cyw43_cpu_core_id;
static volatile uint32_t s_cyw43_ram_core_id;
static volatile uint32_t s_cyw43_reset_vector_addr;
static volatile uint32_t s_cyw43_reset_vector_value;
static volatile uint32_t s_cyw43_verify_mismatch_addr;
static volatile uint32_t s_cyw43_verify_expected;
static volatile uint32_t s_cyw43_verify_actual;
static volatile uint32_t s_cyw43_nvram_packed_len;
static volatile uint32_t s_cyw43_nvram_padded_len;
static volatile uint32_t s_cyw43_nvram_footer_word;
static volatile uint8_t s_cyw43_nvram_using_reference;
static volatile uint8_t s_cyw43_chip_clock_csr_diag;
static volatile uint32_t s_cyw43_sr_control1_diag;
static volatile uint32_t s_cyw43_wlan_ioctrl_diag;
static volatile uint32_t s_cyw43_wlan_resetctrl_diag;
static volatile uint32_t s_cyw43_socram_ioctrl_diag;
static volatile uint32_t s_cyw43_socram_resetctrl_diag;
static volatile uint8_t s_cyw43_wakeup_ctrl_diag;
static volatile uint8_t s_cyw43_sleep_csr_diag;
static volatile uint8_t s_cyw43_cardcap_diag;
static volatile uint8_t s_cyw43_io_ready_diag;
static volatile uint32_t s_cyw43_checkpoint;
static volatile uint32_t s_cyw43_checkpoint_result;
static volatile uint32_t s_cyw43_last_success_checkpoint;
static volatile uint8_t s_cyw43_checkpoint_function;
static volatile uint32_t s_cyw43_checkpoint_address;
static volatile uint32_t s_cyw43_checkpoint_write_value;
static volatile uint32_t s_cyw43_checkpoint_readback_value;
static volatile int32_t s_cyw43_checkpoint_status;
static volatile uint32_t s_cyw43_profile;
static volatile uint8_t s_cyw43_backplane_is_write;
static volatile uint32_t s_cyw43_backplane_address;
static volatile uint8_t s_cyw43_backplane_width_bytes;
static volatile int32_t s_cyw43_backplane_status;
/*
 * Runtime default: profile-selected. current_repo and most bench profiles use
 * AP6256-specific nvram_ap6256.txt; coherent firmware/NVRAM profiles can opt
 * into their bundled text file without changing the public console command.
 */
static volatile uint8_t s_cyw43_reference_nvram_enabled =
    (AP6256_ASSET_WIFI_PROFILE_DEFAULT_GENERIC_NVRAM != 0U) ? 1U : 0U;
static volatile uint32_t s_cyw43_last_async_event_type;
static volatile uint32_t s_cyw43_last_async_event_status;
static volatile uint32_t s_cyw43_last_async_event_reason;
static volatile uint32_t s_cyw43_last_async_event_flags;
static volatile uint32_t s_cyw43_last_join_event_type;
static volatile uint32_t s_cyw43_last_join_event_status;
static volatile uint32_t s_cyw43_last_join_event_reason;
static volatile uint32_t s_cyw43_last_join_event_flags;
static volatile uint32_t s_cyw43_async_event_count;
static volatile uint32_t s_cyw43_join_event_count;
static volatile uint32_t s_cyw43_last_ioctl_kind;
static volatile uint32_t s_cyw43_last_ioctl_cmd;
static volatile uint32_t s_cyw43_last_ioctl_iface;
static volatile uint32_t s_cyw43_last_ioctl_len;
static volatile uint32_t s_cyw43_last_ioctl_id;
static volatile int32_t s_cyw43_last_ioctl_status;
static volatile int32_t s_cyw43_last_ioctl_poll;
static volatile uint32_t s_cyw43_last_ioctl_phase;
static volatile uint32_t s_cyw43_last_completed_ioctl_kind;
static volatile uint32_t s_cyw43_last_completed_ioctl_cmd;
static volatile uint32_t s_cyw43_last_completed_ioctl_iface;
static volatile uint32_t s_cyw43_last_completed_ioctl_id;
static volatile int32_t s_cyw43_last_completed_ioctl_status;
static volatile int32_t s_cyw43_last_completed_ioctl_poll;
static volatile uint8_t s_cyw43_packet_pending;
static volatile uint8_t s_cyw43_packet_pending_source;
static volatile uint8_t s_cyw43_dat1_level;
static volatile uint8_t s_cyw43_cccr_int_pending;
static volatile uint32_t s_cyw43_f1_int_status;
static volatile int32_t s_cyw43_packet_pending_status;
static volatile int32_t s_cyw43_kso_status;
static volatile uint32_t s_cyw43_scan_wake_step;
static volatile uint8_t s_cyw43_send_flow_control;
static volatile uint8_t s_cyw43_send_tx_seq;
static volatile uint8_t s_cyw43_send_credit;
static volatile uint8_t s_cyw43_send_synthetic_credit;
static volatile int32_t s_cyw43_send_credit_status;
static volatile int32_t s_cyw43_setup_status;
static volatile uint32_t s_cyw43_breadcrumb_stage;
static volatile int32_t s_cyw43_breadcrumb_detail;
static volatile uint32_t s_cyw43_breadcrumb_tick_ms;
static volatile uint32_t s_cyw43_breadcrumb_reset_flags;
static volatile uint32_t s_cyw43_boot_reset_flags;
static volatile uint32_t s_cyw43_last_persist_breadcrumb_stage;
static volatile uint32_t s_cyw43_last_persist_breadcrumb_tick_ms;
static volatile uint32_t s_cyw43_wait_no_packet_count;
static volatile uint32_t s_cyw43_wait_recovery_count;
static volatile uint32_t s_cyw43_wait_forced_probe_count;
static volatile uint32_t s_cyw43_wait_resend_count;
static volatile uint8_t s_cyw43_ioctl_recovery_attempted;
static volatile uint8_t s_cyw43_ioctl_forced_probe_attempted;
static volatile uint8_t s_cyw43_ioctl_resend_attempted;
static volatile uint32_t s_cyw43_last_pre_reset_persist_tick;
static volatile uint8_t s_cyw43_assoc_target_valid;
static volatile uint8_t s_cyw43_assoc_target_bssid[6];
static volatile uint8_t s_cyw43_assoc_target_channel;
static volatile uint8_t s_cyw43_assoc_target_5g;
static volatile uint8_t s_cyw43_assoc_target_auth_code;
static volatile uint8_t s_cyw43_assoc_candidate_index;
static volatile uint8_t s_cyw43_assoc_candidate_count;
static volatile uint8_t s_cyw43_assoc_target_mfp;
static volatile uint16_t s_cyw43_assoc_target_chanspec;
static volatile uint16_t s_cyw43_assoc_target_rsn_cap;
static volatile uint8_t s_cyw43_last_rx_class;
static volatile uint8_t s_cyw43_last_rx_channel;
static volatile uint16_t s_cyw43_last_rx_sdpcm_len;
static volatile uint16_t s_cyw43_last_rx_payload_len;
static volatile uint16_t s_cyw43_last_rx_ethertype;
static volatile uint8_t s_cyw43_last_rx_ip_proto;
static volatile uint16_t s_cyw43_last_rx_src_port;
static volatile uint16_t s_cyw43_last_rx_dst_port;
static volatile uint32_t s_cyw43_last_rx_first_word;
static volatile uint8_t s_cyw43_last_tx_itf;
static volatile uint16_t s_cyw43_last_tx_payload_len;
static volatile uint16_t s_cyw43_last_tx_ethertype;
static volatile uint8_t s_cyw43_last_tx_ip_proto;
static volatile uint16_t s_cyw43_last_tx_src_port;
static volatile uint16_t s_cyw43_last_tx_dst_port;
static volatile uint32_t s_cyw43_last_tx_src_mac_hi;
static volatile uint16_t s_cyw43_last_tx_src_mac_lo;
static volatile uint32_t s_cyw43_last_tx_dhcp_chaddr_hi;
static volatile uint16_t s_cyw43_last_tx_dhcp_chaddr_lo;
static volatile int32_t s_cyw43_last_tx_status;
static ap6256_cyw43_control_tx_diag_t s_cyw43_control_tx_diag;

static void ap6256_cyw43_enable_backup_access(void)
{
    HAL_PWR_EnableBkUpAccess();
}

static void ap6256_cyw43_append_reset_flag(char *buffer, size_t buffer_len, const char *flag)
{
    size_t used;
    size_t remaining;

    if ((buffer == NULL) || (flag == NULL) || (buffer_len == 0U)) {
        return;
    }

    used = strlen(buffer);
    if (used >= (buffer_len - 1U)) {
        return;
    }

    if (used != 0U) {
        buffer[used++] = '|';
        buffer[used] = '\0';
    }

    remaining = buffer_len - used - 1U;
    (void)strncpy(&buffer[used], flag, remaining);
    buffer[buffer_len - 1U] = '\0';
}

static uint8_t ap6256_cyw43_pre_reset_valid_raw(void)
{
    return (RTC->BKP8R == AP6256_CYW43_PRE_RESET_MAGIC) ? 1U : 0U;
}

static uint8_t ap6256_cyw43_is_noisy_breadcrumb_stage(uint32_t stage)
{
    switch (stage) {
    case AP6256_CYW43_BREADCRUMB_IOCTL_WAIT:
    case AP6256_CYW43_BREADCRUMB_SCAN_WAIT:
    case AP6256_CYW43_BREADCRUMB_CMD53_READ:
    case AP6256_CYW43_BREADCRUMB_CMD53_WRITE:
    case AP6256_CYW43_BREADCRUMB_CMD52:
        return 1U;
    default:
        return 0U;
    }
}

static void ap6256_cyw43_port_persist_pre_reset_diag(uint8_t force)
{
    uint32_t now = HAL_GetTick();
    uint32_t iface_len;
    uint32_t packet_pack;
    uint32_t cmd53_pack;
    uint32_t send_pack;
    uint32_t assoc_target_bssid_lo;
    uint32_t assoc_target_bssid_hi;
    uint32_t assoc_target_meta;

    if ((force == 0U) &&
        ((now - s_cyw43_last_pre_reset_persist_tick) < AP6256_CYW43_PRE_RESET_THROTTLE_MS)) {
        return;
    }
    s_cyw43_last_pre_reset_persist_tick = now;

    iface_len = (s_cyw43_last_ioctl_iface & 0xFFU) |
                ((s_cyw43_last_ioctl_len & 0x00FFFFFFUL) << 8U);
    packet_pack = (s_cyw43_packet_pending & 0x01U) |
                  ((uint32_t)(s_cyw43_packet_pending_source & 0x07U) << 1U) |
                  ((uint32_t)s_cyw43_dat1_level << 8U) |
                  ((uint32_t)s_cyw43_cccr_int_pending << 16U) |
                  ((uint32_t)s_cyw43_last_rx_class << 24U);
    cmd53_pack = (s_cyw43_last_cmd53_write & 0x01U) |
                 ((uint32_t)(s_cyw43_last_cmd53_function & 0x07U) << 1U) |
                 ((uint32_t)(s_cyw43_last_cmd53_block_mode & 0x01U) << 4U) |
                 ((uint32_t)s_cyw43_last_cmd53_frame_size << 8U) |
                 ((uint32_t)s_cyw43_last_rx_channel << 24U);
    send_pack = (uint32_t)s_cyw43_send_flow_control |
                ((uint32_t)s_cyw43_send_tx_seq << 8U) |
                ((uint32_t)s_cyw43_send_credit << 16U) |
                ((uint32_t)(s_cyw43_send_synthetic_credit & 0x01U) << 24U);
    assoc_target_bssid_lo =
        ((uint32_t)s_cyw43_assoc_target_bssid[0]) |
        ((uint32_t)s_cyw43_assoc_target_bssid[1] << 8U) |
        ((uint32_t)s_cyw43_assoc_target_bssid[2] << 16U) |
        ((uint32_t)s_cyw43_assoc_target_bssid[3] << 24U);
    assoc_target_bssid_hi =
        ((uint32_t)s_cyw43_assoc_target_bssid[4]) |
        ((uint32_t)s_cyw43_assoc_target_bssid[5] << 8U) |
        ((uint32_t)s_cyw43_assoc_target_channel << 16U) |
        ((uint32_t)s_cyw43_assoc_target_auth_code << 24U);
    assoc_target_meta =
        ((uint32_t)s_cyw43_assoc_target_chanspec & 0xFFFFU) |
        ((uint32_t)(s_cyw43_assoc_target_5g & 0x01U) << 16U) |
        ((uint32_t)(s_cyw43_assoc_candidate_index & 0x7FU) << 17U) |
        ((uint32_t)(s_cyw43_assoc_candidate_count & 0x7FU) << 24U) |
        ((uint32_t)(s_cyw43_assoc_target_valid & 0x01U) << 31U);

    ap6256_cyw43_enable_backup_access();
    RTC->BKP6R = assoc_target_bssid_lo;
    RTC->BKP7R = assoc_target_bssid_hi;
    RTC->BKP8R = AP6256_CYW43_PRE_RESET_MAGIC;
    RTC->BKP9R = s_cyw43_breadcrumb_stage;
    RTC->BKP10R = (uint32_t)s_cyw43_breadcrumb_detail;
    RTC->BKP11R = s_cyw43_breadcrumb_tick_ms;
    RTC->BKP12R = s_cyw43_last_ioctl_phase;
    RTC->BKP13R = s_cyw43_last_ioctl_kind;
    RTC->BKP14R = s_cyw43_last_ioctl_cmd;
    RTC->BKP15R = iface_len;
    RTC->BKP16R = s_cyw43_last_ioctl_id;
    RTC->BKP17R = (uint32_t)s_cyw43_last_ioctl_status;
    RTC->BKP18R = (uint32_t)s_cyw43_last_ioctl_poll;
    RTC->BKP19R = packet_pack;
    RTC->BKP20R = assoc_target_meta;
    RTC->BKP21R = s_cyw43_f1_int_status;
    RTC->BKP22R = (uint32_t)s_cyw43_kso_status;
    RTC->BKP23R = s_cyw43_last_cmd;
    RTC->BKP24R = s_cyw43_last_cmd_arg;
    RTC->BKP25R = (uint32_t)s_cyw43_last_cmd_status;
    RTC->BKP26R =
        (s_cyw43_last_cmd_response & 0xFFU) |
        ((s_cyw43_last_async_event_type & 0xFFU) << 8U) |
        ((s_cyw43_last_async_event_status & 0xFFU) << 16U) |
        ((s_cyw43_last_async_event_reason & 0xFFU) << 24U);
    RTC->BKP27R = cmd53_pack;
    RTC->BKP28R = s_cyw43_last_cmd53_block_size;
    RTC->BKP29R = s_cyw43_last_cmd53_length;
    RTC->BKP30R = (uint32_t)s_cyw43_last_cmd53_status;
    RTC->BKP31R = send_pack;
}

static void ap6256_cyw43_port_read_pre_reset_diag(ap6256_cyw43_pre_reset_diag_t *diag)
{
    uint32_t iface_len;
    uint32_t packet_pack;
    uint32_t cmd53_pack;
    uint32_t send_pack;
    uint32_t assoc_target_bssid_lo;
    uint32_t assoc_target_bssid_hi;
    uint32_t assoc_target_meta;

    if (diag == NULL) {
        return;
    }

    memset(diag, 0, sizeof(*diag));
    if (ap6256_cyw43_pre_reset_valid_raw() == 0U) {
        return;
    }

    diag->valid = 1U;
    diag->breadcrumb_stage = RTC->BKP9R;
    diag->breadcrumb_detail = (int32_t)RTC->BKP10R;
    diag->tick_ms = RTC->BKP11R;
    diag->ioctl_phase = RTC->BKP12R;
    diag->ioctl_kind = RTC->BKP13R;
    diag->ioctl_cmd = RTC->BKP14R;
    iface_len = RTC->BKP15R;
    diag->ioctl_iface = iface_len & 0xFFU;
    diag->ioctl_len = (iface_len >> 8U) & 0x00FFFFFFUL;
    diag->ioctl_id = RTC->BKP16R;
    diag->ioctl_status = (int32_t)RTC->BKP17R;
    diag->ioctl_poll = (int32_t)RTC->BKP18R;
    packet_pack = RTC->BKP19R;
    diag->packet_pending = packet_pack & 0x01U;
    diag->packet_pending_source = (packet_pack >> 1U) & 0x07U;
    diag->dat1_level = (packet_pack >> 8U) & 0xFFU;
    diag->cccr_int_pending = (packet_pack >> 16U) & 0xFFU;
    diag->rx_class = (packet_pack >> 24U) & 0xFFU;
    diag->packet_pending_status = 0;
    diag->f1_int_status = RTC->BKP21R;
    diag->kso_status = (int32_t)RTC->BKP22R;
    diag->last_cmd = RTC->BKP23R;
    diag->last_cmd_arg = RTC->BKP24R;
    diag->last_cmd_status = (int32_t)RTC->BKP25R;
    diag->last_cmd_response = RTC->BKP26R & 0xFFU;
    diag->async_event_type = (RTC->BKP26R >> 8U) & 0xFFU;
    diag->async_event_status = (RTC->BKP26R >> 16U) & 0xFFU;
    diag->async_event_reason = (RTC->BKP26R >> 24U) & 0xFFU;
    diag->async_event_flags = 0U;
    cmd53_pack = RTC->BKP27R;
    diag->cmd53_write = cmd53_pack & 0x01U;
    diag->cmd53_function = (cmd53_pack >> 1U) & 0x07U;
    diag->cmd53_block_mode = (cmd53_pack >> 4U) & 0x01U;
    diag->cmd53_frame_size = (uint16_t)((cmd53_pack >> 8U) & 0xFFFFU);
    diag->rx_channel = (cmd53_pack >> 24U) & 0xFFU;
    diag->cmd53_block_size = RTC->BKP28R;
    diag->cmd53_length = RTC->BKP29R;
    diag->cmd53_status = (int32_t)RTC->BKP30R;
    diag->rx_payload_len = (uint16_t)(RTC->BKP29R & 0xFFFFU);
    assoc_target_bssid_lo = RTC->BKP6R;
    assoc_target_bssid_hi = RTC->BKP7R;
    assoc_target_meta = RTC->BKP20R;
    send_pack = RTC->BKP31R;
    diag->send_flow_control = send_pack & 0xFFU;
    diag->send_tx_seq = (send_pack >> 8U) & 0xFFU;
    diag->send_credit = (send_pack >> 16U) & 0xFFU;
    diag->send_synthetic_credit = (send_pack >> 24U) & 0x01U;
    diag->completed_ioctl_cmd = 0U;
    diag->completed_ioctl_status = 0;
    diag->completed_ioctl_poll = 0;
    diag->completed_ioctl_kind = 0U;
    diag->completed_ioctl_iface = 0U;
    diag->completed_ioctl_id = 0U;
    diag->assoc_target_bssid[0] = assoc_target_bssid_lo & 0xFFU;
    diag->assoc_target_bssid[1] = (assoc_target_bssid_lo >> 8U) & 0xFFU;
    diag->assoc_target_bssid[2] = (assoc_target_bssid_lo >> 16U) & 0xFFU;
    diag->assoc_target_bssid[3] = (assoc_target_bssid_lo >> 24U) & 0xFFU;
    diag->assoc_target_bssid[4] = assoc_target_bssid_hi & 0xFFU;
    diag->assoc_target_bssid[5] = (assoc_target_bssid_hi >> 8U) & 0xFFU;
    diag->assoc_target_channel = (assoc_target_bssid_hi >> 16U) & 0xFFU;
    diag->assoc_target_auth_code = (assoc_target_bssid_hi >> 24U) & 0xFFU;
    diag->assoc_target_chanspec = assoc_target_meta & 0xFFFFU;
    diag->assoc_target_5g = (assoc_target_meta >> 16U) & 0x01U;
    diag->assoc_candidate_index = (assoc_target_meta >> 17U) & 0x7FU;
    diag->assoc_candidate_count = (assoc_target_meta >> 24U) & 0x7FU;
    diag->assoc_target_valid = (assoc_target_meta >> 31U) & 0x01U;
}

void ap6256_cyw43_port_record_breadcrumb(uint32_t stage, int32_t detail)
{
    uint32_t tick_ms = HAL_GetTick();
    uint32_t reset_flags = RCC->RSR;
    uint8_t persist = 1U;

    s_cyw43_breadcrumb_stage = stage;
    s_cyw43_breadcrumb_detail = detail;
    s_cyw43_breadcrumb_tick_ms = tick_ms;
    s_cyw43_breadcrumb_reset_flags = reset_flags;

    if ((ap6256_cyw43_is_noisy_breadcrumb_stage(stage) != 0U) &&
        (detail >= 0) &&
        (s_cyw43_last_persist_breadcrumb_stage == stage) &&
        ((tick_ms - s_cyw43_last_persist_breadcrumb_tick_ms) < AP6256_CYW43_NOISY_BREADCRUMB_PERSIST_MS)) {
        persist = 0U;
    }

    if (persist == 0U) {
        return;
    }
    s_cyw43_last_persist_breadcrumb_stage = stage;
    s_cyw43_last_persist_breadcrumb_tick_ms = tick_ms;

    ap6256_cyw43_enable_backup_access();
    RTC->BKP1R = AP6256_CYW43_BREADCRUMB_MAGIC;
    RTC->BKP2R = stage;
    RTC->BKP3R = (uint32_t)detail;
    RTC->BKP4R = tick_ms;
    RTC->BKP5R = reset_flags;
    ap6256_cyw43_port_persist_pre_reset_diag(1U);
}

void ap6256_cyw43_port_capture_boot_reset_flags(uint32_t flags)
{
    s_cyw43_boot_reset_flags = flags;
}

uint8_t ap6256_cyw43_port_breadcrumb_valid(void)
{
    return (RTC->BKP1R == AP6256_CYW43_BREADCRUMB_MAGIC) ? 1U : 0U;
}

uint32_t ap6256_cyw43_port_breadcrumb_stage(void)
{
    return (ap6256_cyw43_port_breadcrumb_valid() != 0U) ? RTC->BKP2R : s_cyw43_breadcrumb_stage;
}

int32_t ap6256_cyw43_port_breadcrumb_detail(void)
{
    return (ap6256_cyw43_port_breadcrumb_valid() != 0U) ? (int32_t)RTC->BKP3R : s_cyw43_breadcrumb_detail;
}

uint32_t ap6256_cyw43_port_breadcrumb_tick_ms(void)
{
    return (ap6256_cyw43_port_breadcrumb_valid() != 0U) ? RTC->BKP4R : s_cyw43_breadcrumb_tick_ms;
}

uint32_t ap6256_cyw43_port_breadcrumb_reset_flags(void)
{
    return (ap6256_cyw43_port_breadcrumb_valid() != 0U) ? RTC->BKP5R : s_cyw43_breadcrumb_reset_flags;
}

uint32_t ap6256_cyw43_port_boot_reset_flags(void)
{
    return s_cyw43_boot_reset_flags;
}

const char *ap6256_cyw43_port_breadcrumb_name(uint32_t stage)
{
    switch (stage) {
    case AP6256_CYW43_BREADCRUMB_NONE:
        return "none";
    case AP6256_CYW43_BREADCRUMB_SETUP_ENTER:
        return "setup_enter";
    case AP6256_CYW43_BREADCRUMB_WIFI_ON:
        return "wifi_on";
    case AP6256_CYW43_BREADCRUMB_BUS_INIT:
        return "bus_init";
    case AP6256_CYW43_BREADCRUMB_LL_WIFI_ON:
        return "ll_wifi_on";
    case AP6256_CYW43_BREADCRUMB_PM:
        return "pm";
    case AP6256_CYW43_BREADCRUMB_TCPIP_INIT:
        return "tcpip_init";
    case AP6256_CYW43_BREADCRUMB_SETUP_EXIT:
        return "setup_exit";
    case AP6256_CYW43_BREADCRUMB_SCAN_START:
        return "scan_start";
    case AP6256_CYW43_BREADCRUMB_SCAN_ACCEPTED:
        return "scan_accepted";
    case AP6256_CYW43_BREADCRUMB_SCAN_WAIT:
        return "scan_wait";
    case AP6256_CYW43_BREADCRUMB_SCAN_COMPLETE:
        return "scan_complete";
    case AP6256_CYW43_BREADCRUMB_PROMPT_SSID:
        return "prompt_ssid";
    case AP6256_CYW43_BREADCRUMB_SCAN_TIMEOUT:
        return "scan_timeout";
    case AP6256_CYW43_BREADCRUMB_RELEASE:
        return "release";
    case AP6256_CYW43_BREADCRUMB_SUSPEND:
        return "suspend";
    case AP6256_CYW43_BREADCRUMB_SCAN_WAKE_ENTER:
        return "scan_wake_enter";
    case AP6256_CYW43_BREADCRUMB_SCAN_WAKE_EXIT:
        return "scan_wake_exit";
    case AP6256_CYW43_BREADCRUMB_SCAN_DRAIN_ENTER:
        return "scan_drain_enter";
    case AP6256_CYW43_BREADCRUMB_SCAN_DRAIN_EXIT:
        return "scan_drain_exit";
    case AP6256_CYW43_BREADCRUMB_ESCAN_IOVAR:
        return "escan_iovar";
    case AP6256_CYW43_BREADCRUMB_IOCTL_SEND:
        return "ioctl_send";
    case AP6256_CYW43_BREADCRUMB_IOCTL_WAIT:
        return "ioctl_wait";
    case AP6256_CYW43_BREADCRUMB_SDPCM_CREDIT_WAIT:
        return "sdpcm_credit_wait";
    case AP6256_CYW43_BREADCRUMB_SDPCM_TX:
        return "sdpcm_tx";
    case AP6256_CYW43_BREADCRUMB_CMD53_READ:
        return "cmd53_read";
    case AP6256_CYW43_BREADCRUMB_CMD53_WRITE:
        return "cmd53_write";
    case AP6256_CYW43_BREADCRUMB_CMD52:
        return "cmd52";
    case AP6256_CYW43_BREADCRUMB_SCAN_RETURN:
        return "scan_return";
    case AP6256_CYW43_BREADCRUMB_JOIN_START:
        return "join_start";
    case AP6256_CYW43_BREADCRUMB_JOIN_WAIT:
        return "join_wait";
    case AP6256_CYW43_BREADCRUMB_JOIN_TIMEOUT:
        return "join_timeout";
    case AP6256_CYW43_BREADCRUMB_JOIN_RESULT:
        return "join_result";
    case AP6256_CYW43_BREADCRUMB_HARDFAULT:
        return "hardfault";
    case AP6256_CYW43_BREADCRUMB_MEMMANAGE:
        return "memmanage";
    case AP6256_CYW43_BREADCRUMB_BUSFAULT:
        return "busfault";
    case AP6256_CYW43_BREADCRUMB_USAGEFAULT:
        return "usagefault";
    case AP6256_CYW43_BREADCRUMB_STACK_OVERFLOW:
        return "stack_overflow";
    case AP6256_CYW43_BREADCRUMB_MALLOC_FAILED:
        return "malloc_failed";
    default:
        return "unknown";
    }
}

void ap6256_cyw43_port_format_reset_flags(uint32_t flags, char *buffer, size_t buffer_len)
{
    if ((buffer == NULL) || (buffer_len == 0U)) {
        return;
    }

    buffer[0] = '\0';
    if ((flags & RCC_RSR_PORRSTF) != 0U) {
        ap6256_cyw43_append_reset_flag(buffer, buffer_len, "POR");
    }
    if ((flags & RCC_RSR_BORRSTF) != 0U) {
        ap6256_cyw43_append_reset_flag(buffer, buffer_len, "BOR");
    }
    if ((flags & RCC_RSR_PINRSTF) != 0U) {
        ap6256_cyw43_append_reset_flag(buffer, buffer_len, "PIN");
    }
    if ((flags & RCC_RSR_SFTRSTF) != 0U) {
        ap6256_cyw43_append_reset_flag(buffer, buffer_len, "SW");
    }
    if ((flags & RCC_RSR_IWDG1RSTF) != 0U) {
        ap6256_cyw43_append_reset_flag(buffer, buffer_len, "IWDG1");
    }
    if ((flags & RCC_RSR_WWDG1RSTF) != 0U) {
        ap6256_cyw43_append_reset_flag(buffer, buffer_len, "WWDG1");
    }
    if ((flags & RCC_RSR_LPWRRSTF) != 0U) {
        ap6256_cyw43_append_reset_flag(buffer, buffer_len, "LPWR");
    }
    if (buffer[0] == '\0') {
        (void)strncpy(buffer, "none", buffer_len - 1U);
        buffer[buffer_len - 1U] = '\0';
    }
}

void ap6256_cyw43_port_set_setup_status(int32_t status)
{
    s_cyw43_setup_status = status;
}

int32_t ap6256_cyw43_port_setup_status(void)
{
    return s_cyw43_setup_status;
}

static void ap6256_cyw43_port_record_cmd53(uint8_t write,
                                           uint8_t function,
                                           uint8_t block_mode,
                                           uint32_t block_size,
                                           uint32_t len,
                                           int32_t status,
                                           const uint8_t *buf)
{
    s_cyw43_last_cmd53_count++;
    s_cyw43_last_cmd53_write = write;
    s_cyw43_last_cmd53_function = function;
    s_cyw43_last_cmd53_block_mode = block_mode;
    s_cyw43_last_cmd53_block_size = block_size;
    s_cyw43_last_cmd53_length = len;
    s_cyw43_last_cmd53_status = status;
    s_cyw43_last_cmd53_frame_size = 0U;

    if ((write == 0U) && (status == 0) && (buf != NULL) && (len >= 2U)) {
        s_cyw43_last_cmd53_frame_size = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8U);
    }
    ap6256_cyw43_port_persist_pre_reset_diag(0U);
}

static osMutexId_t ap6256_cyw43_mutex(void)
{
    static const osMutexAttr_t mutex_attr = {
        .name = "cyw43",
        .attr_bits = osMutexRecursive
    };

    if (s_cyw43_mutex != NULL) {
        return s_cyw43_mutex;
    }

    if ((osKernelGetState() == osKernelRunning) || (osKernelGetState() == osKernelReady)) {
        s_cyw43_mutex = osMutexNew(&mutex_attr);
    }

    return s_cyw43_mutex;
}

static void ap6256_cyw43_build_mac(uint8_t mac[6])
{
    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();
    uint32_t uid2 = HAL_GetUIDw2();

    mac[0] = 0x02U;
    mac[1] = (uint8_t)(uid0 & 0xFFU);
    mac[2] = (uint8_t)((uid0 >> 8U) & 0xFFU);
    mac[3] = (uint8_t)((uid0 >> 16U) & 0xFFU);
    mac[4] = (uint8_t)(uid1 & 0xFFU);
    mac[5] = (uint8_t)(uid2 & 0xFFU);
}

static int ap6256_cyw43_status_to_errno(ap6256_status_t st)
{
    switch (st) {
    case AP6256_STATUS_OK:
        return 0;
    case AP6256_STATUS_TIMEOUT:
        return -CYW43_ETIMEDOUT;
    case AP6256_STATUS_BAD_PARAM:
        return -CYW43_EINVAL;
    case AP6256_STATUS_IO_ERROR:
    case AP6256_STATUS_PROTOCOL_ERROR:
    default:
        return -CYW43_EIO;
    }
}

typedef enum {
    AP6256_CYW43_RESP_NONE = 0U,
    AP6256_CYW43_RESP_SHORT = 1U,
    AP6256_CYW43_RESP_LONG = 3U
} ap6256_cyw43_resp_t;

static ap6256_status_t ap6256_cyw43_send_cmd(uint32_t cmd_idx,
                                             uint32_t arg,
                                             ap6256_cyw43_resp_t waitresp,
                                             uint8_t ignore_crc_fail,
                                             uint32_t *response)
{
    uint32_t start = HAL_GetTick();
    uint32_t guard_loops = 0U;
    uint32_t status;
    uint32_t cmd = SDMMC_CMD_CPSMEN | (cmd_idx & 0x3FU);

    if (waitresp == AP6256_CYW43_RESP_SHORT) {
        cmd |= SDMMC_CMD_WAITRESP_0;
    } else if (waitresp == AP6256_CYW43_RESP_LONG) {
        cmd |= SDMMC_CMD_WAITRESP_0 | SDMMC_CMD_WAITRESP_1;
    }

    SDMMC1->ICR = 0xFFFFFFFFU;
    SDMMC1->ARG = arg;
    SDMMC1->CMD = cmd;

    while (1) {
        status = SDMMC1->STA;

        if (waitresp == AP6256_CYW43_RESP_NONE) {
            if ((status & SDMMC_STA_CMDSENT) != 0U) {
                break;
            }
        } else {
            if ((status & SDMMC_STA_CMDREND) != 0U) {
                break;
            }
            if ((status & SDMMC_STA_CCRCFAIL) != 0U) {
                if (ignore_crc_fail != 0U) {
                    break;
                }
                SDMMC1->ICR = 0xFFFFFFFFU;
                return AP6256_STATUS_PROTOCOL_ERROR;
            }
        }

        if ((status & SDMMC_STA_CTIMEOUT) != 0U) {
            SDMMC1->ICR = 0xFFFFFFFFU;
            return AP6256_STATUS_TIMEOUT;
        }

        if ((HAL_GetTick() - start) > AP6256_CYW43_SDIO_CMD_TIMEOUT_MS) {
            SDMMC1->ICR = 0xFFFFFFFFU;
            return AP6256_STATUS_TIMEOUT;
        }

        if (++guard_loops > AP6256_CYW43_SDIO_CMD_GUARD_LOOPS) {
            SDMMC1->ICR = 0xFFFFFFFFU;
            return AP6256_STATUS_TIMEOUT;
        }
    }

    if (response != NULL) {
        *response = SDMMC1->RESP1;
    }

    SDMMC1->ICR = 0xFFFFFFFFU;
    return AP6256_STATUS_OK;
}

void ap6256_cyw43_port_init(void)
{
    (void)ap6256_cyw43_mutex();
    s_cyw43_setup_status = 0;
}

void ap6256_cyw43_port_deinit(void)
{
    cyw43_poll = NULL;
    s_poll_pending = 0U;
    s_cyw43_sched_calls = 0U;
    s_cyw43_last_sched_func = 0U;
    s_cyw43_sdio_init_calls = 0U;
    s_cyw43_sdio_reinit_calls = 0U;
    s_cyw43_last_bus_init_ret = 0;
    s_cyw43_bus_stage = 0U;
    s_cyw43_poll_header_read_status = 0;
    s_cyw43_poll_payload_read_status = 0;
    s_cyw43_poll_hdr0 = 0U;
    s_cyw43_poll_hdr1 = 0U;
    s_cyw43_last_cmd53_write = 0U;
    s_cyw43_last_cmd53_function = 0U;
    s_cyw43_last_cmd53_block_mode = 0U;
    s_cyw43_last_cmd53_block_size = 0U;
    s_cyw43_last_cmd53_length = 0U;
    s_cyw43_last_cmd53_status = 0;
    s_cyw43_last_cmd53_frame_size = 0U;
    s_cyw43_last_cmd53_count = 0U;
    s_cyw43_last_cmd = 0U;
    s_cyw43_last_cmd_arg = 0U;
    s_cyw43_last_cmd_status = 0;
    s_cyw43_last_cmd_response = 0U;
    s_cyw43_chip_id_raw = 0U;
    s_cyw43_ram_base_addr = 0U;
    s_cyw43_ram_size_bytes = 0U;
    s_cyw43_boot_mode = AP6256_CYW43_BOOT_CM3_SOCRAM;
    s_cyw43_cpu_wrapper_addr = 0U;
    s_cyw43_ram_wrapper_addr = 0U;
    s_cyw43_firmware_addr = 0U;
    s_cyw43_nvram_addr = 0U;
    s_cyw43_footer_addr = 0U;
    s_cyw43_cpu_core_id = 0U;
    s_cyw43_ram_core_id = 0U;
    s_cyw43_reset_vector_addr = 0U;
    s_cyw43_reset_vector_value = 0U;
    s_cyw43_verify_mismatch_addr = 0U;
    s_cyw43_verify_expected = 0U;
    s_cyw43_verify_actual = 0U;
    s_cyw43_nvram_packed_len = 0U;
    s_cyw43_nvram_padded_len = 0U;
    s_cyw43_nvram_footer_word = 0U;
    s_cyw43_nvram_using_reference = 0U;
    s_cyw43_chip_clock_csr_diag = 0U;
    s_cyw43_sr_control1_diag = 0U;
    s_cyw43_wlan_ioctrl_diag = 0U;
    s_cyw43_wlan_resetctrl_diag = 0U;
    s_cyw43_socram_ioctrl_diag = 0U;
    s_cyw43_socram_resetctrl_diag = 0U;
    s_cyw43_wakeup_ctrl_diag = 0U;
    s_cyw43_sleep_csr_diag = 0U;
    s_cyw43_cardcap_diag = 0U;
    s_cyw43_io_ready_diag = 0U;
    s_cyw43_checkpoint = 0U;
    s_cyw43_checkpoint_result = AP6256_CYW43_CP_RESULT_PENDING;
    s_cyw43_last_success_checkpoint = 0U;
    s_cyw43_checkpoint_function = 0U;
    s_cyw43_checkpoint_address = 0U;
    s_cyw43_checkpoint_write_value = 0U;
    s_cyw43_checkpoint_readback_value = 0U;
    s_cyw43_checkpoint_status = 0;
    s_cyw43_profile = AP6256_CYW43_PROFILE_BASELINE;
    s_cyw43_backplane_is_write = 0U;
    s_cyw43_backplane_address = 0U;
    s_cyw43_backplane_width_bytes = 0U;
    s_cyw43_backplane_status = 0;
    s_cyw43_last_async_event_type = 0U;
    s_cyw43_last_async_event_status = 0U;
    s_cyw43_last_async_event_reason = 0U;
    s_cyw43_last_async_event_flags = 0U;
    s_cyw43_last_join_event_type = 0U;
    s_cyw43_last_join_event_status = 0U;
    s_cyw43_last_join_event_reason = 0U;
    s_cyw43_last_join_event_flags = 0U;
    s_cyw43_async_event_count = 0U;
    s_cyw43_join_event_count = 0U;
    s_cyw43_last_ioctl_kind = 0U;
    s_cyw43_last_ioctl_cmd = 0U;
    s_cyw43_last_ioctl_iface = 0U;
    s_cyw43_last_ioctl_len = 0U;
    s_cyw43_last_ioctl_id = 0U;
    s_cyw43_last_ioctl_status = 0;
    s_cyw43_last_ioctl_poll = 0;
    s_cyw43_last_ioctl_phase = AP6256_CYW43_IOCTL_PHASE_NONE;
    s_cyw43_last_completed_ioctl_kind = 0U;
    s_cyw43_last_completed_ioctl_cmd = 0U;
    s_cyw43_last_completed_ioctl_iface = 0U;
    s_cyw43_last_completed_ioctl_id = 0U;
    s_cyw43_last_completed_ioctl_status = 0;
    s_cyw43_last_completed_ioctl_poll = 0;
    s_cyw43_packet_pending = 0U;
    s_cyw43_packet_pending_source = AP6256_CYW43_PACKET_SRC_NONE;
    s_cyw43_dat1_level = 0U;
    s_cyw43_cccr_int_pending = 0U;
    s_cyw43_f1_int_status = 0U;
    s_cyw43_packet_pending_status = 0;
    s_cyw43_kso_status = 0;
    s_cyw43_scan_wake_step = AP6256_CYW43_SCAN_WAKE_NONE;
    s_cyw43_send_flow_control = 0U;
    s_cyw43_send_tx_seq = 0U;
    s_cyw43_send_credit = 0U;
    s_cyw43_send_synthetic_credit = 0U;
    s_cyw43_send_credit_status = 0;
    s_cyw43_last_persist_breadcrumb_stage = AP6256_CYW43_BREADCRUMB_NONE;
    s_cyw43_last_persist_breadcrumb_tick_ms = 0U;
}

void ap6256_cyw43_thread_enter(void)
{
    osMutexId_t mutex = ap6256_cyw43_mutex();

    if (mutex != NULL) {
        (void)osMutexAcquire(mutex, osWaitForever);
    }
}

void ap6256_cyw43_thread_exit(void)
{
    osMutexId_t mutex = ap6256_cyw43_mutex();

    if (mutex != NULL) {
        (void)osMutexRelease(mutex);
    }
}

void ap6256_cyw43_port_wait_step(void)
{
    osKernelState_t kernel_state = osKernelGetState();

    if ((__get_PRIMASK() == 0U) &&
        ((kernel_state == osKernelRunning) || (kernel_state == osKernelReady))) {
        (void)osDelay(1U);
        return;
    }

    /*
     * If IRQs are masked or the scheduler is unavailable, avoid blocking on
     * osDelay() and do a short bounded spin so scan/ioctl loops can still
     * progress to their own timeout handling.
     */
    for (volatile uint32_t spin = 0U; spin < 12000U; ++spin) {
        __NOP();
    }
}

unsigned int cyw43_hal_ticks_us(void)
{
    return (unsigned int)(HAL_GetTick() * 1000U);
}

unsigned int cyw43_hal_ticks_ms(void)
{
    return (unsigned int)HAL_GetTick();
}

void cyw43_delay_us(unsigned int us)
{
    uint32_t start = HAL_GetTick() * 1000U;

    while (((HAL_GetTick() * 1000U) - start) < us) {
    }
}

void cyw43_delay_ms(unsigned int ms)
{
    osDelay(ms);
}

void cyw43_hal_get_mac(int interface, uint8_t mac[6])
{
    (void)interface;
    if (mac == NULL) {
        return;
    }
    ap6256_cyw43_build_mac(mac);
}

void cyw43_hal_pin_config(int pin, int mode, int pull, int alt)
{
    (void)pin;
    (void)mode;
    (void)pull;
    (void)alt;
}

void cyw43_hal_pin_config_irq_falling(int pin, int enable)
{
    /*
     * CYW43 uses WL_SDIO_1/DAT1 as its in-band interrupt source in SDIO mode.
     * On STM32H7, reconfiguring PC9 from SDMMC alternate-function into EXTI GPIO
     * steals DAT1 away from the 4-bit SDIO bus and can wedge subsequent transfers.
     *
     * Keep DAT1 owned by SDMMC1 at all times and treat interrupt enable/disable
     * as a logical no-op. The runtime already polls from a dedicated FreeRTOS task,
     * and cyw43_cb_read_host_interrupt_pin() can still sample the live DAT1 level.
     */
    (void)pin;
    (void)enable;
}

int cyw43_hal_pin_read(int pin)
{
    switch (pin) {
    case CYW43_PIN_WL_REG_ON:
        return (HAL_GPIO_ReadPin(AP6256_CYW43_WL_REG_ON_PORT, AP6256_CYW43_WL_REG_ON_PIN) == GPIO_PIN_SET) ? 1 : 0;
    case CYW43_PIN_WL_SDIO_1:
        return (HAL_GPIO_ReadPin(AP6256_CYW43_SDIO_DAT1_PORT, AP6256_CYW43_SDIO_DAT1_PIN) == GPIO_PIN_SET) ? 1 : 0;
    default:
        return 0;
    }
}

void cyw43_hal_pin_low(int pin)
{
    if (pin == CYW43_PIN_WL_REG_ON) {
        HAL_GPIO_WritePin(AP6256_CYW43_WL_REG_ON_PORT, AP6256_CYW43_WL_REG_ON_PIN, GPIO_PIN_RESET);
    }
}

void cyw43_hal_pin_high(int pin)
{
    if (pin == CYW43_PIN_WL_REG_ON) {
        HAL_GPIO_WritePin(AP6256_CYW43_WL_REG_ON_PORT, AP6256_CYW43_WL_REG_ON_PIN, GPIO_PIN_SET);
    }
}

void cyw43_schedule_internal_poll_dispatch(void (*func)(void))
{
    s_cyw43_sched_calls++;
    s_cyw43_last_sched_func = (uintptr_t)func;
    cyw43_poll = func;
    s_poll_pending = (func != NULL) ? 1U : 0U;
}

void ap6256_cyw43_port_poll(void)
{
    if (cyw43_poll == NULL) {
        return;
    }

    s_poll_pending = 0U;
    ap6256_cyw43_thread_enter();
    cyw43_poll();
    ap6256_cyw43_thread_exit();
}

uint8_t ap6256_cyw43_port_pending_poll(void)
{
    return s_poll_pending;
}

void cyw43_sdio_init(void)
{
    ap6256_status_t st;

    s_cyw43_sdio_init_calls++;
    st = ap6256_sdio_bus_prepare(1U, 0U);
    s_sdio_open = (st == AP6256_STATUS_OK) ? 1U : 0U;
}

void cyw43_sdio_reinit(void)
{
    s_cyw43_sdio_reinit_calls++;

    if (s_sdio_open == 0U) {
        cyw43_sdio_init();
    }
}

void cyw43_sdio_deinit(void)
{
    if (s_sdio_open != 0U) {
        ap6256_sdio_close();
        s_sdio_open = 0U;
    }
}

void cyw43_sdio_set_irq(bool enable)
{
    /*
     * See cyw43_hal_pin_config_irq_falling(): do not hand SDIO DAT1 to EXTI on
     * this board. Leave the pin mux on SDMMC1 and rely on the Wi-Fi poll task.
     */
    (void)enable;
}

void cyw43_sdio_enable_high_speed_4bit(void)
{
    /*
     * Keep the bus conservative during bring-up. The previous divider of 1 was
     * aggressive enough that short register traffic could work while the first
     * sustained firmware-download block transfer timed out.
     */
    MODIFY_REG(SDMMC1->CLKCR,
               SDMMC_CLKCR_WIDBUS | SDMMC_CLKCR_CLKDIV,
               SDMMC_CLKCR_WIDBUS_0 | 8U);
}

int cyw43_sdio_transfer(uint32_t cmd, uint32_t arg, uint32_t *resp)
{
    ap6256_status_t st;
    uint32_t local_resp = 0U;

    switch (cmd) {
    case 0U:
        st = ap6256_cyw43_send_cmd(0U, 0U, AP6256_CYW43_RESP_NONE, 1U, NULL);
        break;
    case 3U:
        st = ap6256_cyw43_send_cmd(3U, 0U, AP6256_CYW43_RESP_SHORT, 1U, &local_resp);
        break;
    case 5U:
        st = ap6256_cyw43_send_cmd(5U, arg, AP6256_CYW43_RESP_SHORT, 1U, &local_resp);
        break;
    case 7U:
        st = ap6256_cyw43_send_cmd(7U, arg, AP6256_CYW43_RESP_SHORT, 1U, NULL);
        if (st == AP6256_STATUS_OK) {
            ap6256_sdio_session_adopt((uint16_t)((arg >> 16U) & 0xFFFFU));
        }
        break;
    case 52U: {
        uint8_t value = 0U;
        uint8_t function = (uint8_t)((arg >> 28U) & 0x07U);
        uint32_t address = (arg >> 9U) & 0x1FFFFU;
        uint8_t write = ((arg & 0x80000000UL) != 0U) ? 1U : 0U;
        if (write != 0U) {
            st = ap6256_sdio_cmd52_write_u8(function, address, (uint8_t)(arg & 0xFFU));
            if (st == AP6256_STATUS_OK) {
                local_resp = (arg & 0xFFU);
            }
        } else {
            st = ap6256_sdio_cmd52_read_u8(function, address, &value);
            if (st == AP6256_STATUS_OK) {
                local_resp = value;
            }
        }
        if (st != AP6256_STATUS_OK) {
            int32_t detail = (int32_t)(((uint32_t)function << 24U) |
                                       ((uint32_t)write << 23U) |
                                       (address & 0x1FFFFUL));
            ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_CMD52, detail);
            ap6256_cyw43_port_record_breadcrumb(AP6256_CYW43_BREADCRUMB_CMD52,
                                                ap6256_cyw43_status_to_errno(st));
        }
        break;
    }
    default:
        st = AP6256_STATUS_BAD_PARAM;
        break;
    }

    if (resp != NULL) {
        *resp = local_resp;
    }

    ap6256_cyw43_port_record_last_cmd(cmd, arg, ap6256_cyw43_status_to_errno(st), local_resp);
    return ap6256_cyw43_status_to_errno(st);
}

int cyw43_sdio_transfer_cmd53(bool write, uint32_t block_size, uint32_t arg, size_t len, uint8_t *buf)
{
    ap6256_status_t st;
    int result;
    uint8_t function = (uint8_t)((arg >> 28U) & 0x07U);
    uint8_t block_mode = (uint8_t)((arg >> 27U) & 0x01U);
    uint8_t op_code = (uint8_t)((arg >> 26U) & 0x01U);
    uint32_t address = (arg >> 9U) & 0x1FFFFU;
    if (write) {
        st = ap6256_sdio_cmd53_write_ex(function, address, buf, (uint32_t)len, block_mode, op_code, block_size);
    } else {
        st = ap6256_sdio_cmd53_read_ex(function, address, buf, (uint32_t)len, block_mode, op_code, block_size);
    }

    result = ap6256_cyw43_status_to_errno(st);
    ap6256_cyw43_port_record_cmd53(write ? 1U : 0U,
                                   function,
                                   block_mode,
                                   block_size,
                                   (uint32_t)len,
                                   result,
                                   buf);
    if (result != 0) {
        int32_t detail = (int32_t)(((uint32_t)function << 24U) |
                                   ((uint32_t)(write ? 1U : 0U) << 23U) |
                                   ((uint32_t)block_mode << 22U) |
                                   ((uint32_t)len & 0x003FFFFFUL));
        ap6256_cyw43_port_record_breadcrumb(write ? AP6256_CYW43_BREADCRUMB_CMD53_WRITE
                                                  : AP6256_CYW43_BREADCRUMB_CMD53_READ,
                                            detail);
        ap6256_cyw43_port_record_breadcrumb(write ? AP6256_CYW43_BREADCRUMB_CMD53_WRITE
                                                  : AP6256_CYW43_BREADCRUMB_CMD53_READ,
                                            result);
    }
    return result;
}

void EXTI9_5_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(AP6256_CYW43_SDIO_DAT1_PIN) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(AP6256_CYW43_SDIO_DAT1_PIN);
        s_poll_pending = 1U;
    }
}

uint32_t ap6256_cyw43_port_sched_calls(void)
{
    return s_cyw43_sched_calls;
}

uintptr_t ap6256_cyw43_port_last_sched_func(void)
{
    return s_cyw43_last_sched_func;
}

uint32_t ap6256_cyw43_port_sdio_init_calls(void)
{
    return s_cyw43_sdio_init_calls;
}

uint32_t ap6256_cyw43_port_sdio_reinit_calls(void)
{
    return s_cyw43_sdio_reinit_calls;
}

void ap6256_cyw43_port_record_last_cmd(uint32_t cmd, uint32_t arg, int32_t status, uint32_t response)
{
    s_cyw43_last_cmd = cmd;
    s_cyw43_last_cmd_arg = arg;
    s_cyw43_last_cmd_status = status;
    s_cyw43_last_cmd_response = response;
    ap6256_cyw43_port_persist_pre_reset_diag(0U);
}

uint32_t ap6256_cyw43_port_last_cmd(void)
{
    return s_cyw43_last_cmd;
}

uint32_t ap6256_cyw43_port_last_cmd_arg(void)
{
    return s_cyw43_last_cmd_arg;
}

int32_t ap6256_cyw43_port_last_cmd_status(void)
{
    return s_cyw43_last_cmd_status;
}

uint32_t ap6256_cyw43_port_last_cmd_response(void)
{
    return s_cyw43_last_cmd_response;
}

void ap6256_cyw43_port_set_chip_id_raw(uint32_t value)
{
    s_cyw43_chip_id_raw = value;
}

uint32_t ap6256_cyw43_port_chip_id_raw(void)
{
    return s_cyw43_chip_id_raw;
}

void ap6256_cyw43_port_set_ram_size_bytes(uint32_t value)
{
    s_cyw43_ram_size_bytes = value;
}

void ap6256_cyw43_port_set_ram_base_addr(uint32_t value)
{
    s_cyw43_ram_base_addr = value;
}

uint32_t ap6256_cyw43_port_ram_base_addr(void)
{
    return s_cyw43_ram_base_addr;
}

uint32_t ap6256_cyw43_port_ram_size_bytes(void)
{
    return s_cyw43_ram_size_bytes;
}

void ap6256_cyw43_port_set_boot_mode(uint32_t value)
{
    s_cyw43_boot_mode = value;
}

uint32_t ap6256_cyw43_port_boot_mode(void)
{
    return s_cyw43_boot_mode;
}

void ap6256_cyw43_port_set_boot_descriptor(uint32_t cpu_wrapper,
                                           uint32_t ram_wrapper,
                                           uint32_t firmware_addr,
                                           uint32_t nvram_addr,
                                           uint32_t footer_addr)
{
    s_cyw43_cpu_wrapper_addr = cpu_wrapper;
    s_cyw43_ram_wrapper_addr = ram_wrapper;
    s_cyw43_firmware_addr = firmware_addr;
    s_cyw43_nvram_addr = nvram_addr;
    s_cyw43_footer_addr = footer_addr;
}

void ap6256_cyw43_port_set_boot_topology(uint32_t cpu_core_id,
                                         uint32_t ram_core_id,
                                         uint32_t reset_vector_addr,
                                         uint32_t reset_vector_value)
{
    s_cyw43_cpu_core_id = cpu_core_id;
    s_cyw43_ram_core_id = ram_core_id;
    s_cyw43_reset_vector_addr = reset_vector_addr;
    s_cyw43_reset_vector_value = reset_vector_value;
}

uint32_t ap6256_cyw43_port_cpu_wrapper_addr(void)
{
    return s_cyw43_cpu_wrapper_addr;
}

uint32_t ap6256_cyw43_port_ram_wrapper_addr(void)
{
    return s_cyw43_ram_wrapper_addr;
}

uint32_t ap6256_cyw43_port_firmware_addr(void)
{
    return s_cyw43_firmware_addr;
}

uint32_t ap6256_cyw43_port_nvram_addr(void)
{
    return s_cyw43_nvram_addr;
}

uint32_t ap6256_cyw43_port_footer_addr(void)
{
    return s_cyw43_footer_addr;
}

uint32_t ap6256_cyw43_port_cpu_core_id(void)
{
    return s_cyw43_cpu_core_id;
}

uint32_t ap6256_cyw43_port_ram_core_id(void)
{
    return s_cyw43_ram_core_id;
}

uint32_t ap6256_cyw43_port_reset_vector_addr(void)
{
    return s_cyw43_reset_vector_addr;
}

uint32_t ap6256_cyw43_port_reset_vector_value(void)
{
    return s_cyw43_reset_vector_value;
}

void ap6256_cyw43_port_set_verify_mismatch(uint32_t address,
                                           uint32_t expected,
                                           uint32_t actual)
{
    s_cyw43_verify_mismatch_addr = address;
    s_cyw43_verify_expected = expected;
    s_cyw43_verify_actual = actual;
}

uint32_t ap6256_cyw43_port_verify_mismatch_addr(void)
{
    return s_cyw43_verify_mismatch_addr;
}

uint32_t ap6256_cyw43_port_verify_expected(void)
{
    return s_cyw43_verify_expected;
}

uint32_t ap6256_cyw43_port_verify_actual(void)
{
    return s_cyw43_verify_actual;
}

void ap6256_cyw43_port_set_nvram_metrics(uint32_t packed_len,
                                         uint32_t padded_len,
                                         uint32_t footer_word,
                                         uint8_t using_reference)
{
    s_cyw43_nvram_packed_len = packed_len;
    s_cyw43_nvram_padded_len = padded_len;
    s_cyw43_nvram_footer_word = footer_word;
    s_cyw43_nvram_using_reference = using_reference;
}

uint32_t ap6256_cyw43_port_nvram_packed_len(void)
{
    return s_cyw43_nvram_packed_len;
}

uint32_t ap6256_cyw43_port_nvram_padded_len(void)
{
    return s_cyw43_nvram_padded_len;
}

uint32_t ap6256_cyw43_port_nvram_footer_word(void)
{
    return s_cyw43_nvram_footer_word;
}

uint8_t ap6256_cyw43_port_nvram_using_reference(void)
{
    return s_cyw43_nvram_using_reference;
}

void ap6256_cyw43_port_set_core_diag(uint8_t chip_clock_csr,
                                     uint32_t sr_control1,
                                     uint32_t wlan_ioctrl,
                                     uint32_t wlan_resetctrl,
                                     uint32_t socram_ioctrl,
                                     uint32_t socram_resetctrl)
{
    s_cyw43_chip_clock_csr_diag = chip_clock_csr;
    s_cyw43_sr_control1_diag = sr_control1;
    s_cyw43_wlan_ioctrl_diag = wlan_ioctrl;
    s_cyw43_wlan_resetctrl_diag = wlan_resetctrl;
    s_cyw43_socram_ioctrl_diag = socram_ioctrl;
    s_cyw43_socram_resetctrl_diag = socram_resetctrl;
}

uint8_t ap6256_cyw43_port_chip_clock_csr_diag(void)
{
    return s_cyw43_chip_clock_csr_diag;
}

uint32_t ap6256_cyw43_port_sr_control1_diag(void)
{
    return s_cyw43_sr_control1_diag;
}

uint32_t ap6256_cyw43_port_wlan_ioctrl_diag(void)
{
    return s_cyw43_wlan_ioctrl_diag;
}

uint32_t ap6256_cyw43_port_wlan_resetctrl_diag(void)
{
    return s_cyw43_wlan_resetctrl_diag;
}

uint32_t ap6256_cyw43_port_socram_ioctrl_diag(void)
{
    return s_cyw43_socram_ioctrl_diag;
}

uint32_t ap6256_cyw43_port_socram_resetctrl_diag(void)
{
    return s_cyw43_socram_resetctrl_diag;
}

void ap6256_cyw43_port_set_checkpoint(uint32_t checkpoint,
                                      uint32_t checkpoint_result,
                                      uint8_t function,
                                      uint32_t address,
                                      uint32_t write_value,
                                      uint32_t readback_value,
                                      int32_t status)
{
    s_cyw43_checkpoint = checkpoint;
    s_cyw43_checkpoint_result = checkpoint_result;
    if (checkpoint_result == AP6256_CYW43_CP_RESULT_OK) {
        s_cyw43_last_success_checkpoint = checkpoint;
    }
    s_cyw43_checkpoint_function = function;
    s_cyw43_checkpoint_address = address;
    s_cyw43_checkpoint_write_value = write_value;
    s_cyw43_checkpoint_readback_value = readback_value;
    s_cyw43_checkpoint_status = status;
}

uint32_t ap6256_cyw43_port_checkpoint(void)
{
    return s_cyw43_checkpoint;
}

uint32_t ap6256_cyw43_port_checkpoint_result(void)
{
    return s_cyw43_checkpoint_result;
}

uint32_t ap6256_cyw43_port_last_success_checkpoint(void)
{
    return s_cyw43_last_success_checkpoint;
}

uint8_t ap6256_cyw43_port_checkpoint_function(void)
{
    return s_cyw43_checkpoint_function;
}

uint32_t ap6256_cyw43_port_checkpoint_address(void)
{
    return s_cyw43_checkpoint_address;
}

uint32_t ap6256_cyw43_port_checkpoint_write_value(void)
{
    return s_cyw43_checkpoint_write_value;
}

uint32_t ap6256_cyw43_port_checkpoint_readback_value(void)
{
    return s_cyw43_checkpoint_readback_value;
}

int32_t ap6256_cyw43_port_checkpoint_status(void)
{
    return s_cyw43_checkpoint_status;
}

void ap6256_cyw43_port_set_profile(uint32_t profile)
{
    s_cyw43_profile = profile;
}

uint32_t ap6256_cyw43_port_profile(void)
{
    return s_cyw43_profile;
}

void ap6256_cyw43_port_set_stage5_snapshot(uint8_t wakeup_ctrl,
                                           uint8_t sleep_csr,
                                           uint8_t cardcap,
                                           uint8_t io_ready)
{
    s_cyw43_wakeup_ctrl_diag = wakeup_ctrl;
    s_cyw43_sleep_csr_diag = sleep_csr;
    s_cyw43_cardcap_diag = cardcap;
    s_cyw43_io_ready_diag = io_ready;
}

uint8_t ap6256_cyw43_port_wakeup_ctrl_diag(void)
{
    return s_cyw43_wakeup_ctrl_diag;
}

uint8_t ap6256_cyw43_port_sleep_csr_diag(void)
{
    return s_cyw43_sleep_csr_diag;
}

uint8_t ap6256_cyw43_port_cardcap_diag(void)
{
    return s_cyw43_cardcap_diag;
}

uint8_t ap6256_cyw43_port_io_ready_diag(void)
{
    return s_cyw43_io_ready_diag;
}

void ap6256_cyw43_port_record_backplane_access(uint8_t is_write,
                                               uint32_t address,
                                               uint8_t width_bytes,
                                               int32_t status)
{
    s_cyw43_backplane_is_write = is_write;
    s_cyw43_backplane_address = address;
    s_cyw43_backplane_width_bytes = width_bytes;
    s_cyw43_backplane_status = status;
}

uint8_t ap6256_cyw43_port_backplane_is_write(void)
{
    return s_cyw43_backplane_is_write;
}

uint32_t ap6256_cyw43_port_backplane_address(void)
{
    return s_cyw43_backplane_address;
}

uint8_t ap6256_cyw43_port_backplane_width_bytes(void)
{
    return s_cyw43_backplane_width_bytes;
}

int32_t ap6256_cyw43_port_backplane_status(void)
{
    return s_cyw43_backplane_status;
}

void ap6256_cyw43_port_record_async_event(uint32_t event_type,
                                          uint32_t status,
                                          uint32_t reason,
                                          uint32_t flags)
{
    s_cyw43_async_event_count++;
    s_cyw43_last_async_event_type = event_type;
    s_cyw43_last_async_event_status = status;
    s_cyw43_last_async_event_reason = reason;
    s_cyw43_last_async_event_flags = flags;
    ap6256_cyw43_port_persist_pre_reset_diag(1U);
}

void ap6256_cyw43_port_record_join_event(uint32_t event_type,
                                         uint32_t status,
                                         uint32_t reason,
                                         uint32_t flags)
{
    s_cyw43_join_event_count++;
    s_cyw43_last_join_event_type = event_type;
    s_cyw43_last_join_event_status = status;
    s_cyw43_last_join_event_reason = reason;
    s_cyw43_last_join_event_flags = flags;
    s_cyw43_last_async_event_type = event_type;
    s_cyw43_last_async_event_status = status;
    s_cyw43_last_async_event_reason = reason;
    s_cyw43_last_async_event_flags = flags;
    ap6256_cyw43_port_persist_pre_reset_diag(1U);
}

void ap6256_cyw43_port_reset_join_event_window(void)
{
    s_cyw43_last_join_event_type = 0U;
    s_cyw43_last_join_event_status = 0U;
    s_cyw43_last_join_event_reason = 0U;
    s_cyw43_last_join_event_flags = 0U;
}

uint32_t ap6256_cyw43_port_last_async_event_type(void)
{
    return s_cyw43_last_async_event_type;
}

uint32_t ap6256_cyw43_port_last_async_event_status(void)
{
    return s_cyw43_last_async_event_status;
}

uint32_t ap6256_cyw43_port_last_async_event_reason(void)
{
    return s_cyw43_last_async_event_reason;
}

uint32_t ap6256_cyw43_port_last_async_event_flags(void)
{
    return s_cyw43_last_async_event_flags;
}

uint32_t ap6256_cyw43_port_last_join_event_type(void)
{
    return s_cyw43_last_join_event_type;
}

uint32_t ap6256_cyw43_port_last_join_event_status(void)
{
    return s_cyw43_last_join_event_status;
}

uint32_t ap6256_cyw43_port_last_join_event_reason(void)
{
    return s_cyw43_last_join_event_reason;
}

uint32_t ap6256_cyw43_port_last_join_event_flags(void)
{
    return s_cyw43_last_join_event_flags;
}

uint32_t ap6256_cyw43_port_async_event_count(void)
{
    return s_cyw43_async_event_count;
}

uint32_t ap6256_cyw43_port_join_event_count(void)
{
    return s_cyw43_join_event_count;
}

void ap6256_cyw43_port_record_ioctl(uint32_t kind,
                                    uint32_t cmd,
                                    uint32_t iface,
                                    uint32_t len,
                                    uint32_t id,
                                    int32_t status,
                                    int32_t last_poll)
{
    s_cyw43_last_completed_ioctl_kind = kind;
    s_cyw43_last_completed_ioctl_cmd = cmd;
    s_cyw43_last_completed_ioctl_iface = iface;
    s_cyw43_last_completed_ioctl_id = id;
    s_cyw43_last_completed_ioctl_status = status;
    s_cyw43_last_completed_ioctl_poll = last_poll;

    s_cyw43_last_ioctl_kind = kind;
    s_cyw43_last_ioctl_cmd = cmd;
    s_cyw43_last_ioctl_iface = iface;
    s_cyw43_last_ioctl_len = len;
    s_cyw43_last_ioctl_id = id;
    s_cyw43_last_ioctl_status = status;
    s_cyw43_last_ioctl_poll = last_poll;
    ap6256_cyw43_port_persist_pre_reset_diag(1U);
}

void ap6256_cyw43_port_begin_ioctl(uint32_t kind,
                                   uint32_t cmd,
                                   uint32_t iface,
                                   uint32_t len,
                                   uint32_t id)
{
    s_cyw43_last_ioctl_kind = kind;
    s_cyw43_last_ioctl_cmd = cmd;
    s_cyw43_last_ioctl_iface = iface;
    s_cyw43_last_ioctl_len = len;
    s_cyw43_last_ioctl_id = id;
    s_cyw43_last_ioctl_status = 0;
    s_cyw43_last_ioctl_poll = 0;
    s_cyw43_last_ioctl_phase = AP6256_CYW43_IOCTL_PHASE_NONE;
    ap6256_cyw43_port_persist_pre_reset_diag(1U);
}

void ap6256_cyw43_port_update_ioctl_poll(int32_t last_poll)
{
    s_cyw43_last_ioctl_poll = last_poll;
    ap6256_cyw43_port_persist_pre_reset_diag(0U);
}

void ap6256_cyw43_port_finish_ioctl(int32_t status, int32_t last_poll)
{
    s_cyw43_last_ioctl_status = status;
    s_cyw43_last_ioctl_poll = last_poll;
    ap6256_cyw43_port_persist_pre_reset_diag(1U);
}

void ap6256_cyw43_port_set_ioctl_phase(uint32_t phase)
{
    s_cyw43_last_ioctl_phase = phase;
    ap6256_cyw43_port_persist_pre_reset_diag(0U);
}

uint32_t ap6256_cyw43_port_last_ioctl_kind(void)
{
    return s_cyw43_last_ioctl_kind;
}

uint32_t ap6256_cyw43_port_last_ioctl_cmd(void)
{
    return s_cyw43_last_ioctl_cmd;
}

uint32_t ap6256_cyw43_port_last_ioctl_iface(void)
{
    return s_cyw43_last_ioctl_iface;
}

uint32_t ap6256_cyw43_port_last_ioctl_len(void)
{
    return s_cyw43_last_ioctl_len;
}

uint32_t ap6256_cyw43_port_last_ioctl_id(void)
{
    return s_cyw43_last_ioctl_id;
}

int32_t ap6256_cyw43_port_last_ioctl_status(void)
{
    return s_cyw43_last_ioctl_status;
}

int32_t ap6256_cyw43_port_last_ioctl_poll(void)
{
    return s_cyw43_last_ioctl_poll;
}

uint32_t ap6256_cyw43_port_last_ioctl_phase(void)
{
    return s_cyw43_last_ioctl_phase;
}

uint32_t ap6256_cyw43_port_last_completed_ioctl_kind(void)
{
    return s_cyw43_last_completed_ioctl_kind;
}

uint32_t ap6256_cyw43_port_last_completed_ioctl_cmd(void)
{
    return s_cyw43_last_completed_ioctl_cmd;
}

uint32_t ap6256_cyw43_port_last_completed_ioctl_iface(void)
{
    return s_cyw43_last_completed_ioctl_iface;
}

uint32_t ap6256_cyw43_port_last_completed_ioctl_id(void)
{
    return s_cyw43_last_completed_ioctl_id;
}

int32_t ap6256_cyw43_port_last_completed_ioctl_status(void)
{
    return s_cyw43_last_completed_ioctl_status;
}

int32_t ap6256_cyw43_port_last_completed_ioctl_poll(void)
{
    return s_cyw43_last_completed_ioctl_poll;
}

void ap6256_cyw43_port_record_packet_pending(uint8_t packet_pending,
                                             uint8_t pending_source,
                                             uint8_t dat1_level,
                                             uint8_t cccr_int_pending,
                                             uint32_t f1_int_status,
                                             int32_t status)
{
    s_cyw43_packet_pending = (packet_pending != 0U) ? 1U : 0U;
    s_cyw43_packet_pending_source = pending_source;
    s_cyw43_dat1_level = dat1_level;
    s_cyw43_cccr_int_pending = cccr_int_pending;
    s_cyw43_f1_int_status = f1_int_status;
    s_cyw43_packet_pending_status = status;
    ap6256_cyw43_port_persist_pre_reset_diag(0U);
}

uint8_t ap6256_cyw43_port_packet_pending(void)
{
    return s_cyw43_packet_pending;
}

uint8_t ap6256_cyw43_port_packet_pending_source(void)
{
    return s_cyw43_packet_pending_source;
}

uint8_t ap6256_cyw43_port_dat1_level(void)
{
    return s_cyw43_dat1_level;
}

uint8_t ap6256_cyw43_port_cccr_int_pending(void)
{
    return s_cyw43_cccr_int_pending;
}

uint32_t ap6256_cyw43_port_f1_int_status(void)
{
    return s_cyw43_f1_int_status;
}

int32_t ap6256_cyw43_port_packet_pending_status(void)
{
    return s_cyw43_packet_pending_status;
}

void ap6256_cyw43_port_set_kso_status(int32_t status)
{
    s_cyw43_kso_status = status;
    ap6256_cyw43_port_persist_pre_reset_diag(0U);
}

int32_t ap6256_cyw43_port_kso_status(void)
{
    return s_cyw43_kso_status;
}

void ap6256_cyw43_port_set_scan_wake_step(uint32_t step)
{
    s_cyw43_scan_wake_step = step;
}

uint32_t ap6256_cyw43_port_scan_wake_step(void)
{
    return s_cyw43_scan_wake_step;
}

void ap6256_cyw43_port_record_send_credit(uint8_t flow_control,
                                          uint8_t tx_seq,
                                          uint8_t credit,
                                          uint8_t synthetic_credit,
                                          int32_t status)
{
    s_cyw43_send_flow_control = flow_control;
    s_cyw43_send_tx_seq = tx_seq;
    s_cyw43_send_credit = credit;
    s_cyw43_send_synthetic_credit = synthetic_credit;
    s_cyw43_send_credit_status = status;
    ap6256_cyw43_port_persist_pre_reset_diag(0U);
}

uint8_t ap6256_cyw43_port_send_flow_control(void)
{
    return s_cyw43_send_flow_control;
}

uint8_t ap6256_cyw43_port_send_tx_seq(void)
{
    return s_cyw43_send_tx_seq;
}

uint8_t ap6256_cyw43_port_send_credit(void)
{
    return s_cyw43_send_credit;
}

uint8_t ap6256_cyw43_port_send_synthetic_credit(void)
{
    return s_cyw43_send_synthetic_credit;
}

int32_t ap6256_cyw43_port_send_credit_status(void)
{
    return s_cyw43_send_credit_status;
}

void ap6256_cyw43_port_note_wait_no_packet(void)
{
    s_cyw43_wait_no_packet_count++;
    ap6256_cyw43_port_persist_pre_reset_diag(0U);
}

void ap6256_cyw43_port_note_wait_recovery(void)
{
    s_cyw43_wait_recovery_count++;
    ap6256_cyw43_port_persist_pre_reset_diag(0U);
}

void ap6256_cyw43_port_note_wait_forced_probe(void)
{
    s_cyw43_wait_forced_probe_count++;
    ap6256_cyw43_port_persist_pre_reset_diag(0U);
}

void ap6256_cyw43_port_note_wait_resend(void)
{
    s_cyw43_wait_resend_count++;
    ap6256_cyw43_port_persist_pre_reset_diag(0U);
}

uint32_t ap6256_cyw43_port_wait_no_packet_count(void)
{
    return s_cyw43_wait_no_packet_count;
}

uint32_t ap6256_cyw43_port_wait_recovery_count(void)
{
    return s_cyw43_wait_recovery_count;
}

uint32_t ap6256_cyw43_port_wait_forced_probe_count(void)
{
    return s_cyw43_wait_forced_probe_count;
}

uint32_t ap6256_cyw43_port_wait_resend_count(void)
{
    return s_cyw43_wait_resend_count;
}

void ap6256_cyw43_port_set_ioctl_attempt_flags(uint8_t recovery_attempted,
                                               uint8_t forced_probe_attempted,
                                               uint8_t resend_attempted)
{
    s_cyw43_ioctl_recovery_attempted = (recovery_attempted != 0U) ? 1U : 0U;
    s_cyw43_ioctl_forced_probe_attempted = (forced_probe_attempted != 0U) ? 1U : 0U;
    s_cyw43_ioctl_resend_attempted = (resend_attempted != 0U) ? 1U : 0U;
    ap6256_cyw43_port_persist_pre_reset_diag(0U);
}

uint8_t ap6256_cyw43_port_ioctl_recovery_attempted(void)
{
    return s_cyw43_ioctl_recovery_attempted;
}

uint8_t ap6256_cyw43_port_ioctl_forced_probe_attempted(void)
{
    return s_cyw43_ioctl_forced_probe_attempted;
}

uint8_t ap6256_cyw43_port_ioctl_resend_attempted(void)
{
    return s_cyw43_ioctl_resend_attempted;
}

void ap6256_cyw43_port_record_control_tx_frame(uint32_t kind,
                                               uint32_t cmd,
                                               uint32_t iface,
                                               uint32_t len,
                                               uint32_t id,
                                               uint32_t sdpcm_len,
                                               uint32_t transfer_len,
                                               uint32_t block_size,
                                               const uint8_t *frame,
                                               uint32_t frame_len)
{
    uint32_t copy_len = frame_len;
    uint32_t checksum = 2166136261UL;

    if (copy_len > sizeof(s_cyw43_control_tx_diag.first64)) {
        copy_len = sizeof(s_cyw43_control_tx_diag.first64);
    }

    memset(&s_cyw43_control_tx_diag, 0, sizeof(s_cyw43_control_tx_diag));
    s_cyw43_control_tx_diag.valid = 1U;
    s_cyw43_control_tx_diag.kind = kind;
    s_cyw43_control_tx_diag.cmd = cmd;
    s_cyw43_control_tx_diag.iface = iface;
    s_cyw43_control_tx_diag.len = len;
    s_cyw43_control_tx_diag.id = id;
    s_cyw43_control_tx_diag.sdpcm_len = sdpcm_len;
    s_cyw43_control_tx_diag.transfer_len = transfer_len;
    s_cyw43_control_tx_diag.block_size = block_size;

    if ((frame != NULL) && (copy_len != 0U)) {
        memcpy(s_cyw43_control_tx_diag.first64, frame, copy_len);
        for (uint32_t i = 0U; i < frame_len; ++i) {
            checksum ^= frame[i];
            checksum *= 16777619UL;
        }
    }
    s_cyw43_control_tx_diag.checksum = checksum;
    ap6256_cyw43_port_persist_pre_reset_diag(0U);
}

void ap6256_cyw43_port_record_assoc_target(const uint8_t bssid[6],
                                           uint16_t channel,
                                           uint8_t selected_5g,
                                           uint16_t chanspec,
                                           uint16_t rsn_cap,
                                           uint8_t mfp,
                                           uint32_t auth_type,
                                           uint8_t candidate_index,
                                           uint8_t candidate_count)
{
    if (bssid != NULL) {
        s_cyw43_assoc_target_valid = 1U;
        for (uint32_t i = 0U; i < 6U; ++i) {
            s_cyw43_assoc_target_bssid[i] = bssid[i];
        }
    } else {
        s_cyw43_assoc_target_valid = 0U;
        memset((void *)s_cyw43_assoc_target_bssid, 0, sizeof(s_cyw43_assoc_target_bssid));
    }
    s_cyw43_assoc_target_channel = (uint8_t)((channel <= 255U) ? channel : 0U);
    s_cyw43_assoc_target_5g = (selected_5g != 0U) ? 1U : 0U;
    s_cyw43_assoc_target_chanspec = chanspec;
    s_cyw43_assoc_target_rsn_cap = rsn_cap;
    s_cyw43_assoc_target_mfp = mfp;
    s_cyw43_assoc_candidate_index = candidate_index;
    s_cyw43_assoc_candidate_count = candidate_count;
    /*
     * Persist the low auth byte. For CYW43 auth constants this is the useful
     * wsec/auth selector: 0=open, 2=WPA/TKIP, 4=WPA2/AES, 6=WPA/WPA2 mixed.
     */
    s_cyw43_assoc_target_auth_code = (uint8_t)(auth_type & 0xFFU);
    ap6256_cyw43_port_persist_pre_reset_diag(1U);
}

uint8_t ap6256_cyw43_port_assoc_target_is_5g(void)
{
    return s_cyw43_assoc_target_5g;
}

uint16_t ap6256_cyw43_port_assoc_target_rsn_cap(void)
{
    return s_cyw43_assoc_target_rsn_cap;
}

uint8_t ap6256_cyw43_port_assoc_target_mfp(void)
{
    return s_cyw43_assoc_target_mfp;
}

void ap6256_cyw43_port_record_rx_frame(uint8_t rx_class,
                                       uint8_t channel,
                                       uint16_t sdpcm_len,
                                       uint16_t payload_len,
                                       const uint8_t *payload)
{
    uint32_t first_word = 0U;
    uint16_t ethertype = 0U;

    s_cyw43_last_rx_class = rx_class;
    s_cyw43_last_rx_channel = channel;
    s_cyw43_last_rx_sdpcm_len = sdpcm_len;
    s_cyw43_last_rx_payload_len = payload_len;
    s_cyw43_last_rx_ethertype = 0U;
    s_cyw43_last_rx_ip_proto = 0U;
    s_cyw43_last_rx_src_port = 0U;
    s_cyw43_last_rx_dst_port = 0U;

    if (payload != NULL) {
        first_word = payload[0];
        if (payload_len > 1U) {
            first_word |= ((uint32_t)payload[1] << 8U);
        }
        if (payload_len > 2U) {
            first_word |= ((uint32_t)payload[2] << 16U);
        }
        if (payload_len > 3U) {
            first_word |= ((uint32_t)payload[3] << 24U);
        }
        if (payload_len > 13U) {
            ethertype = ((uint16_t)payload[12] << 8U) | payload[13];
            s_cyw43_last_rx_ethertype = ethertype;
        }
        if ((ethertype == 0x0800U) && (payload_len > 23U)) {
            uint8_t ip_header_len = (uint8_t)((payload[14] & 0x0FU) * 4U);
            uint16_t port_offset = (uint16_t)(14U + ip_header_len);

            s_cyw43_last_rx_ip_proto = payload[23];
            if (((payload[23] == 6U) || (payload[23] == 17U)) &&
                (ip_header_len >= 20U) &&
                (payload_len >= (uint16_t)(port_offset + 4U))) {
                s_cyw43_last_rx_src_port = ((uint16_t)payload[port_offset] << 8U) |
                                           payload[port_offset + 1U];
                s_cyw43_last_rx_dst_port = ((uint16_t)payload[port_offset + 2U] << 8U) |
                                           payload[port_offset + 3U];
            }
        }
    }
    s_cyw43_last_rx_first_word = first_word;
    ap6256_cyw43_port_persist_pre_reset_diag(0U);
}

void ap6256_cyw43_port_record_tx_frame(uint8_t itf,
                                       uint16_t payload_len,
                                       uint16_t ethertype,
                                       uint8_t ip_proto,
                                       uint16_t src_port,
                                       uint16_t dst_port,
                                       uint32_t src_mac_hi,
                                       uint16_t src_mac_lo,
                                       uint32_t dhcp_chaddr_hi,
                                       uint16_t dhcp_chaddr_lo,
                                       int32_t status)
{
    s_cyw43_last_tx_itf = itf;
    s_cyw43_last_tx_payload_len = payload_len;
    s_cyw43_last_tx_ethertype = ethertype;
    s_cyw43_last_tx_ip_proto = ip_proto;
    s_cyw43_last_tx_src_port = src_port;
    s_cyw43_last_tx_dst_port = dst_port;
    s_cyw43_last_tx_src_mac_hi = src_mac_hi;
    s_cyw43_last_tx_src_mac_lo = src_mac_lo;
    s_cyw43_last_tx_dhcp_chaddr_hi = dhcp_chaddr_hi;
    s_cyw43_last_tx_dhcp_chaddr_lo = dhcp_chaddr_lo;
    s_cyw43_last_tx_status = status;
    ap6256_cyw43_port_persist_pre_reset_diag(0U);
}

uint8_t ap6256_cyw43_port_last_rx_class(void)
{
    return s_cyw43_last_rx_class;
}

uint8_t ap6256_cyw43_port_last_rx_channel(void)
{
    return s_cyw43_last_rx_channel;
}

uint16_t ap6256_cyw43_port_last_rx_sdpcm_len(void)
{
    return s_cyw43_last_rx_sdpcm_len;
}

uint16_t ap6256_cyw43_port_last_rx_payload_len(void)
{
    return s_cyw43_last_rx_payload_len;
}

uint32_t ap6256_cyw43_port_last_rx_first_word(void)
{
    return s_cyw43_last_rx_first_word;
}

uint16_t ap6256_cyw43_port_last_rx_ethertype(void)
{
    return s_cyw43_last_rx_ethertype;
}

uint8_t ap6256_cyw43_port_last_rx_ip_proto(void)
{
    return s_cyw43_last_rx_ip_proto;
}

uint16_t ap6256_cyw43_port_last_rx_src_port(void)
{
    return s_cyw43_last_rx_src_port;
}

uint16_t ap6256_cyw43_port_last_rx_dst_port(void)
{
    return s_cyw43_last_rx_dst_port;
}

uint8_t ap6256_cyw43_port_last_tx_itf(void)
{
    return s_cyw43_last_tx_itf;
}

uint16_t ap6256_cyw43_port_last_tx_payload_len(void)
{
    return s_cyw43_last_tx_payload_len;
}

uint16_t ap6256_cyw43_port_last_tx_ethertype(void)
{
    return s_cyw43_last_tx_ethertype;
}

uint8_t ap6256_cyw43_port_last_tx_ip_proto(void)
{
    return s_cyw43_last_tx_ip_proto;
}

uint16_t ap6256_cyw43_port_last_tx_src_port(void)
{
    return s_cyw43_last_tx_src_port;
}

uint16_t ap6256_cyw43_port_last_tx_dst_port(void)
{
    return s_cyw43_last_tx_dst_port;
}

uint32_t ap6256_cyw43_port_last_tx_src_mac_hi(void)
{
    return s_cyw43_last_tx_src_mac_hi;
}

uint16_t ap6256_cyw43_port_last_tx_src_mac_lo(void)
{
    return s_cyw43_last_tx_src_mac_lo;
}

uint32_t ap6256_cyw43_port_last_tx_dhcp_chaddr_hi(void)
{
    return s_cyw43_last_tx_dhcp_chaddr_hi;
}

uint16_t ap6256_cyw43_port_last_tx_dhcp_chaddr_lo(void)
{
    return s_cyw43_last_tx_dhcp_chaddr_lo;
}

int32_t ap6256_cyw43_port_last_tx_status(void)
{
    return s_cyw43_last_tx_status;
}

void ap6256_cyw43_port_get_control_tx_diag(ap6256_cyw43_control_tx_diag_t *diag)
{
    if (diag == NULL) {
        return;
    }
    *diag = s_cyw43_control_tx_diag;
}

void ap6256_cyw43_port_get_pre_reset_diag(ap6256_cyw43_pre_reset_diag_t *diag)
{
    ap6256_cyw43_port_read_pre_reset_diag(diag);
}

uint8_t ap6256_cyw43_port_pre_reset_diag_valid(void)
{
    return ap6256_cyw43_pre_reset_valid_raw();
}

void ap6256_cyw43_port_set_reference_nvram_enabled(uint8_t enable)
{
    s_cyw43_reference_nvram_enabled = (enable != 0U) ? 1U : 0U;
}

uint8_t ap6256_cyw43_port_reference_nvram_enabled(void)
{
    return s_cyw43_reference_nvram_enabled;
}

void ap6256_cyw43_port_set_last_bus_init_ret(int32_t value)
{
    s_cyw43_last_bus_init_ret = value;
}

int32_t ap6256_cyw43_port_last_bus_init_ret(void)
{
    return s_cyw43_last_bus_init_ret;
}

void ap6256_cyw43_port_set_bus_stage(uint32_t value)
{
    s_cyw43_bus_stage = value;
}

uint32_t ap6256_cyw43_port_bus_stage(void)
{
    return s_cyw43_bus_stage;
}

void ap6256_cyw43_port_set_poll_diag(int32_t header_read_status,
                                     int32_t payload_read_status,
                                     uint16_t hdr0,
                                     uint16_t hdr1)
{
    s_cyw43_poll_header_read_status = header_read_status;
    s_cyw43_poll_payload_read_status = payload_read_status;
    s_cyw43_poll_hdr0 = hdr0;
    s_cyw43_poll_hdr1 = hdr1;
}

int32_t ap6256_cyw43_port_poll_header_read_status(void)
{
    return s_cyw43_poll_header_read_status;
}

int32_t ap6256_cyw43_port_poll_payload_read_status(void)
{
    return s_cyw43_poll_payload_read_status;
}

uint16_t ap6256_cyw43_port_poll_hdr0(void)
{
    return s_cyw43_poll_hdr0;
}

uint16_t ap6256_cyw43_port_poll_hdr1(void)
{
    return s_cyw43_poll_hdr1;
}

uint8_t ap6256_cyw43_port_last_cmd53_write(void)
{
    return s_cyw43_last_cmd53_write;
}

uint8_t ap6256_cyw43_port_last_cmd53_function(void)
{
    return s_cyw43_last_cmd53_function;
}

uint8_t ap6256_cyw43_port_last_cmd53_block_mode(void)
{
    return s_cyw43_last_cmd53_block_mode;
}

uint32_t ap6256_cyw43_port_last_cmd53_block_size(void)
{
    return s_cyw43_last_cmd53_block_size;
}

uint32_t ap6256_cyw43_port_last_cmd53_length(void)
{
    return s_cyw43_last_cmd53_length;
}

int32_t ap6256_cyw43_port_last_cmd53_status(void)
{
    return s_cyw43_last_cmd53_status;
}

uint16_t ap6256_cyw43_port_last_cmd53_frame_size(void)
{
    return s_cyw43_last_cmd53_frame_size;
}

uint32_t ap6256_cyw43_port_last_cmd53_count(void)
{
    return s_cyw43_last_cmd53_count;
}

uint32_t ap6256_cyw43_port_runtime_f2_block_size(void)
{
    if (s_cyw43_runtime_f2_block_size == 0U) {
        return AP6256_CYW43_SDIO_F2_BLOCK_SIZE;
    }
    return s_cyw43_runtime_f2_block_size;
}

void ap6256_cyw43_port_set_runtime_f2_block_size(uint32_t block_size)
{
    if ((block_size == 64U) || (block_size == 128U) ||
        (block_size == 256U) || (block_size == 512U)) {
        s_cyw43_runtime_f2_block_size = block_size;
    }
}

int32_t ap6256_cyw43_port_apply_runtime_f2_block_size(void)
{
    ap6256_status_t st;
    uint32_t block_size = ap6256_cyw43_port_runtime_f2_block_size();

    st = ap6256_sdio_set_block_size(2U, (uint16_t)block_size);
    if (st != AP6256_STATUS_OK) {
        return ap6256_cyw43_status_to_errno(st);
    }

    return 0;
}
