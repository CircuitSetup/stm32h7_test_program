#ifndef AP6256_CYW43_PORT_H
#define AP6256_CYW43_PORT_H

#include <stdint.h>

void ap6256_cyw43_port_init(void);
void ap6256_cyw43_port_deinit(void);
void ap6256_cyw43_port_poll(void);
uint8_t ap6256_cyw43_port_pending_poll(void);
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
void ap6256_cyw43_port_set_reference_nvram_enabled(uint8_t enable);
uint8_t ap6256_cyw43_port_reference_nvram_enabled(void);

#endif /* AP6256_CYW43_PORT_H */
