#ifndef AP6256_CYW43_PORT_H
#define AP6256_CYW43_PORT_H

#include <stddef.h>
#include <stdint.h>

#define AP6256_CYW43_PACKET_SRC_NONE       0U
#define AP6256_CYW43_PACKET_SRC_DAT1       1U
#define AP6256_CYW43_PACKET_SRC_CCCR_F2    2U
#define AP6256_CYW43_PACKET_SRC_F1_MAILBOX 3U
#define AP6256_CYW43_PACKET_SRC_ERROR      4U

#define AP6256_CYW43_IOCTL_PHASE_NONE           0U
#define AP6256_CYW43_IOCTL_PHASE_SCAN_WAKE      1U
#define AP6256_CYW43_IOCTL_PHASE_SEND_FAIL      2U
#define AP6256_CYW43_IOCTL_PHASE_WAIT_NO_PACKET 3U
#define AP6256_CYW43_IOCTL_PHASE_WAIT_CMD53     4U
#define AP6256_CYW43_IOCTL_PHASE_WRONG_ID       5U
#define AP6256_CYW43_IOCTL_PHASE_MALFORMED      6U
#define AP6256_CYW43_IOCTL_PHASE_CONTROL_STATUS 7U
#define AP6256_CYW43_IOCTL_PHASE_OK             8U
#define AP6256_CYW43_IOCTL_PHASE_SEND_CREDIT    9U

#define AP6256_CYW43_SCAN_WAKE_NONE            0U
#define AP6256_CYW43_SCAN_WAKE_WAKE_CTRL_READ  1U
#define AP6256_CYW43_SCAN_WAKE_WAKE_CTRL_WRITE 2U
#define AP6256_CYW43_SCAN_WAKE_KSO             3U
#define AP6256_CYW43_SCAN_WAKE_SLEEP_VERIFY    4U
#define AP6256_CYW43_SCAN_WAKE_HT_VERIFY       5U
#define AP6256_CYW43_SCAN_WAKE_IORDY_VERIFY    6U
#define AP6256_CYW43_SCAN_WAKE_OK              7U

#define AP6256_CYW43_BREADCRUMB_NONE           0U
#define AP6256_CYW43_BREADCRUMB_SETUP_ENTER    1U
#define AP6256_CYW43_BREADCRUMB_WIFI_ON        2U
#define AP6256_CYW43_BREADCRUMB_BUS_INIT       3U
#define AP6256_CYW43_BREADCRUMB_LL_WIFI_ON     4U
#define AP6256_CYW43_BREADCRUMB_PM             5U
#define AP6256_CYW43_BREADCRUMB_TCPIP_INIT     6U
#define AP6256_CYW43_BREADCRUMB_SETUP_EXIT     7U
#define AP6256_CYW43_BREADCRUMB_SCAN_START     8U
#define AP6256_CYW43_BREADCRUMB_SCAN_ACCEPTED  9U
#define AP6256_CYW43_BREADCRUMB_SCAN_WAIT      10U
#define AP6256_CYW43_BREADCRUMB_SCAN_COMPLETE  11U
#define AP6256_CYW43_BREADCRUMB_PROMPT_SSID    12U
#define AP6256_CYW43_BREADCRUMB_SCAN_TIMEOUT   13U
#define AP6256_CYW43_BREADCRUMB_RELEASE        14U
#define AP6256_CYW43_BREADCRUMB_SUSPEND        15U
#define AP6256_CYW43_BREADCRUMB_SCAN_WAKE_ENTER 16U
#define AP6256_CYW43_BREADCRUMB_SCAN_WAKE_EXIT  17U
#define AP6256_CYW43_BREADCRUMB_SCAN_DRAIN_ENTER 18U
#define AP6256_CYW43_BREADCRUMB_SCAN_DRAIN_EXIT  19U
#define AP6256_CYW43_BREADCRUMB_ESCAN_IOVAR      20U
#define AP6256_CYW43_BREADCRUMB_IOCTL_SEND       21U
#define AP6256_CYW43_BREADCRUMB_IOCTL_WAIT       22U
#define AP6256_CYW43_BREADCRUMB_SDPCM_CREDIT_WAIT 23U
#define AP6256_CYW43_BREADCRUMB_SDPCM_TX         24U
#define AP6256_CYW43_BREADCRUMB_CMD53_READ       25U
#define AP6256_CYW43_BREADCRUMB_CMD53_WRITE      26U
#define AP6256_CYW43_BREADCRUMB_CMD52            27U
#define AP6256_CYW43_BREADCRUMB_SCAN_RETURN      28U
#define AP6256_CYW43_BREADCRUMB_HARDFAULT      100U
#define AP6256_CYW43_BREADCRUMB_MEMMANAGE      101U
#define AP6256_CYW43_BREADCRUMB_BUSFAULT       102U
#define AP6256_CYW43_BREADCRUMB_USAGEFAULT     103U
#define AP6256_CYW43_BREADCRUMB_STACK_OVERFLOW 104U
#define AP6256_CYW43_BREADCRUMB_MALLOC_FAILED  105U

void ap6256_cyw43_port_init(void);
void ap6256_cyw43_port_deinit(void);
void ap6256_cyw43_port_poll(void);
uint8_t ap6256_cyw43_port_pending_poll(void);
void ap6256_cyw43_port_record_breadcrumb(uint32_t stage, int32_t detail);
void ap6256_cyw43_port_capture_boot_reset_flags(uint32_t flags);
uint8_t ap6256_cyw43_port_breadcrumb_valid(void);
uint32_t ap6256_cyw43_port_breadcrumb_stage(void);
int32_t ap6256_cyw43_port_breadcrumb_detail(void);
uint32_t ap6256_cyw43_port_breadcrumb_tick_ms(void);
uint32_t ap6256_cyw43_port_breadcrumb_reset_flags(void);
uint32_t ap6256_cyw43_port_boot_reset_flags(void);
const char *ap6256_cyw43_port_breadcrumb_name(uint32_t stage);
void ap6256_cyw43_port_format_reset_flags(uint32_t flags, char *buffer, size_t buffer_len);
void ap6256_cyw43_port_set_setup_status(int32_t status);
int32_t ap6256_cyw43_port_setup_status(void);
uint32_t ap6256_cyw43_port_sched_calls(void);
uintptr_t ap6256_cyw43_port_last_sched_func(void);
uint32_t ap6256_cyw43_port_sdio_init_calls(void);
uint32_t ap6256_cyw43_port_sdio_reinit_calls(void);
void ap6256_cyw43_port_set_last_bus_init_ret(int32_t value);
int32_t ap6256_cyw43_port_last_bus_init_ret(void);
void ap6256_cyw43_port_set_bus_stage(uint32_t value);
uint32_t ap6256_cyw43_port_bus_stage(void);
void ap6256_cyw43_port_set_poll_diag(int32_t header_read_status,
                                     int32_t payload_read_status,
                                     uint16_t hdr0,
                                     uint16_t hdr1);
int32_t ap6256_cyw43_port_poll_header_read_status(void);
int32_t ap6256_cyw43_port_poll_payload_read_status(void);
uint16_t ap6256_cyw43_port_poll_hdr0(void);
uint16_t ap6256_cyw43_port_poll_hdr1(void);
uint8_t ap6256_cyw43_port_last_cmd53_write(void);
uint8_t ap6256_cyw43_port_last_cmd53_function(void);
uint8_t ap6256_cyw43_port_last_cmd53_block_mode(void);
uint32_t ap6256_cyw43_port_last_cmd53_block_size(void);
uint32_t ap6256_cyw43_port_last_cmd53_length(void);
int32_t ap6256_cyw43_port_last_cmd53_status(void);
uint16_t ap6256_cyw43_port_last_cmd53_frame_size(void);
uint32_t ap6256_cyw43_port_last_cmd53_count(void);
void ap6256_cyw43_port_record_last_cmd(uint32_t cmd, uint32_t arg, int32_t status, uint32_t response);
uint32_t ap6256_cyw43_port_last_cmd(void);
uint32_t ap6256_cyw43_port_last_cmd_arg(void);
int32_t ap6256_cyw43_port_last_cmd_status(void);
uint32_t ap6256_cyw43_port_last_cmd_response(void);
void ap6256_cyw43_port_set_chip_id_raw(uint32_t value);
uint32_t ap6256_cyw43_port_chip_id_raw(void);
void ap6256_cyw43_port_set_ram_size_bytes(uint32_t value);
uint32_t ap6256_cyw43_port_ram_size_bytes(void);
void ap6256_cyw43_port_set_ram_base_addr(uint32_t value);
uint32_t ap6256_cyw43_port_ram_base_addr(void);
void ap6256_cyw43_port_set_boot_mode(uint32_t value);
uint32_t ap6256_cyw43_port_boot_mode(void);
void ap6256_cyw43_port_set_boot_descriptor(uint32_t cpu_wrapper,
                                           uint32_t ram_wrapper,
                                           uint32_t firmware_addr,
                                           uint32_t nvram_addr,
                                           uint32_t footer_addr);
void ap6256_cyw43_port_set_boot_topology(uint32_t cpu_core_id,
                                         uint32_t ram_core_id,
                                         uint32_t reset_vector_addr,
                                         uint32_t reset_vector_value);
uint32_t ap6256_cyw43_port_cpu_wrapper_addr(void);
uint32_t ap6256_cyw43_port_ram_wrapper_addr(void);
uint32_t ap6256_cyw43_port_firmware_addr(void);
uint32_t ap6256_cyw43_port_nvram_addr(void);
uint32_t ap6256_cyw43_port_footer_addr(void);
uint32_t ap6256_cyw43_port_cpu_core_id(void);
uint32_t ap6256_cyw43_port_ram_core_id(void);
uint32_t ap6256_cyw43_port_reset_vector_addr(void);
uint32_t ap6256_cyw43_port_reset_vector_value(void);
void ap6256_cyw43_port_set_verify_mismatch(uint32_t address,
                                           uint32_t expected,
                                           uint32_t actual);
uint32_t ap6256_cyw43_port_verify_mismatch_addr(void);
uint32_t ap6256_cyw43_port_verify_expected(void);
uint32_t ap6256_cyw43_port_verify_actual(void);
void ap6256_cyw43_port_set_nvram_metrics(uint32_t packed_len,
                                         uint32_t padded_len,
                                         uint32_t footer_word,
                                         uint8_t using_reference);
uint32_t ap6256_cyw43_port_nvram_packed_len(void);
uint32_t ap6256_cyw43_port_nvram_padded_len(void);
uint32_t ap6256_cyw43_port_nvram_footer_word(void);
uint8_t ap6256_cyw43_port_nvram_using_reference(void);
void ap6256_cyw43_port_set_core_diag(uint8_t chip_clock_csr,
                                     uint32_t sr_control1,
                                     uint32_t wlan_ioctrl,
                                     uint32_t wlan_resetctrl,
                                     uint32_t socram_ioctrl,
                                     uint32_t socram_resetctrl);
uint8_t ap6256_cyw43_port_chip_clock_csr_diag(void);
uint32_t ap6256_cyw43_port_sr_control1_diag(void);
uint32_t ap6256_cyw43_port_wlan_ioctrl_diag(void);
uint32_t ap6256_cyw43_port_wlan_resetctrl_diag(void);
uint32_t ap6256_cyw43_port_socram_ioctrl_diag(void);
uint32_t ap6256_cyw43_port_socram_resetctrl_diag(void);
void ap6256_cyw43_port_set_checkpoint(uint32_t checkpoint,
                                      uint32_t checkpoint_result,
                                      uint8_t function,
                                      uint32_t address,
                                      uint32_t write_value,
                                      uint32_t readback_value,
                                      int32_t status);
uint32_t ap6256_cyw43_port_checkpoint(void);
uint32_t ap6256_cyw43_port_checkpoint_result(void);
uint32_t ap6256_cyw43_port_last_success_checkpoint(void);
uint8_t ap6256_cyw43_port_checkpoint_function(void);
uint32_t ap6256_cyw43_port_checkpoint_address(void);
uint32_t ap6256_cyw43_port_checkpoint_write_value(void);
uint32_t ap6256_cyw43_port_checkpoint_readback_value(void);
int32_t ap6256_cyw43_port_checkpoint_status(void);
void ap6256_cyw43_port_set_profile(uint32_t profile);
uint32_t ap6256_cyw43_port_profile(void);
void ap6256_cyw43_port_set_stage5_snapshot(uint8_t wakeup_ctrl,
                                           uint8_t sleep_csr,
                                           uint8_t cardcap,
                                           uint8_t io_ready);
uint8_t ap6256_cyw43_port_wakeup_ctrl_diag(void);
uint8_t ap6256_cyw43_port_sleep_csr_diag(void);
uint8_t ap6256_cyw43_port_cardcap_diag(void);
uint8_t ap6256_cyw43_port_io_ready_diag(void);
void ap6256_cyw43_port_record_backplane_access(uint8_t is_write,
                                               uint32_t address,
                                               uint8_t width_bytes,
                                               int32_t status);
uint8_t ap6256_cyw43_port_backplane_is_write(void);
uint32_t ap6256_cyw43_port_backplane_address(void);
uint8_t ap6256_cyw43_port_backplane_width_bytes(void);
int32_t ap6256_cyw43_port_backplane_status(void);
void ap6256_cyw43_port_record_async_event(uint32_t event_type,
                                          uint32_t status,
                                          uint32_t reason,
                                          uint32_t flags);
uint32_t ap6256_cyw43_port_last_async_event_type(void);
uint32_t ap6256_cyw43_port_last_async_event_status(void);
uint32_t ap6256_cyw43_port_last_async_event_reason(void);
uint32_t ap6256_cyw43_port_last_async_event_flags(void);
uint32_t ap6256_cyw43_port_async_event_count(void);
void ap6256_cyw43_port_record_ioctl(uint32_t kind,
                                    uint32_t cmd,
                                    uint32_t iface,
                                    uint32_t len,
                                    uint32_t id,
                                    int32_t status,
                                    int32_t last_poll);
void ap6256_cyw43_port_set_ioctl_phase(uint32_t phase);
uint32_t ap6256_cyw43_port_last_ioctl_kind(void);
uint32_t ap6256_cyw43_port_last_ioctl_cmd(void);
uint32_t ap6256_cyw43_port_last_ioctl_iface(void);
uint32_t ap6256_cyw43_port_last_ioctl_len(void);
uint32_t ap6256_cyw43_port_last_ioctl_id(void);
int32_t ap6256_cyw43_port_last_ioctl_status(void);
int32_t ap6256_cyw43_port_last_ioctl_poll(void);
uint32_t ap6256_cyw43_port_last_ioctl_phase(void);
void ap6256_cyw43_port_record_packet_pending(uint8_t packet_pending,
                                             uint8_t pending_source,
                                             uint8_t dat1_level,
                                             uint8_t cccr_int_pending,
                                             uint32_t f1_int_status,
                                             int32_t status);
uint8_t ap6256_cyw43_port_packet_pending(void);
uint8_t ap6256_cyw43_port_packet_pending_source(void);
uint8_t ap6256_cyw43_port_dat1_level(void);
uint8_t ap6256_cyw43_port_cccr_int_pending(void);
uint32_t ap6256_cyw43_port_f1_int_status(void);
int32_t ap6256_cyw43_port_packet_pending_status(void);
void ap6256_cyw43_port_set_kso_status(int32_t status);
int32_t ap6256_cyw43_port_kso_status(void);
void ap6256_cyw43_port_set_scan_wake_step(uint32_t step);
uint32_t ap6256_cyw43_port_scan_wake_step(void);
void ap6256_cyw43_port_record_send_credit(uint8_t flow_control,
                                          uint8_t tx_seq,
                                          uint8_t credit,
                                          uint8_t synthetic_credit,
                                          int32_t status);
uint8_t ap6256_cyw43_port_send_flow_control(void);
uint8_t ap6256_cyw43_port_send_tx_seq(void);
uint8_t ap6256_cyw43_port_send_credit(void);
uint8_t ap6256_cyw43_port_send_synthetic_credit(void);
int32_t ap6256_cyw43_port_send_credit_status(void);
void ap6256_cyw43_port_set_reference_nvram_enabled(uint8_t enable);
uint8_t ap6256_cyw43_port_reference_nvram_enabled(void);

#endif /* AP6256_CYW43_PORT_H */
