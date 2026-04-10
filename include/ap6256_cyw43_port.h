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
void ap6256_cyw43_port_set_reference_nvram_enabled(uint8_t enable);
uint8_t ap6256_cyw43_port_reference_nvram_enabled(void);

#endif /* AP6256_CYW43_PORT_H */
