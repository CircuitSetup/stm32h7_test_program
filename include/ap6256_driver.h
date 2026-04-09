#ifndef AP6256_DRIVER_H
#define AP6256_DRIVER_H

#include "main.h"

#include <stdint.h>

typedef enum {
    AP6256_STATUS_OK = 0,
    AP6256_STATUS_TIMEOUT,
    AP6256_STATUS_IO_ERROR,
    AP6256_STATUS_PROTOCOL_ERROR,
    AP6256_STATUS_BAD_PARAM
} ap6256_status_t;

typedef struct {
    uint8_t wl_reg_on;
    uint8_t bt_reg_on;
    uint8_t wl_pin_level;
    uint8_t bt_pin_level;
    uint8_t disabled_cmd5_response;
    uint8_t cmd5_ready;
    uint8_t cmd5_attempts;
    uint8_t cmd3_ok;
    uint8_t cmd7_ok;
    uint16_t rca;
    uint8_t cccr_read_ok;
    uint8_t cccr_read_attempts;
    uint32_t ocr;
    uint8_t cccr_rev;
    uint8_t sdio_rev;
    uint8_t io_enable;
    uint8_t io_ready;
    uint32_t cis_ptr;
    uint8_t function1_ready;
    uint8_t function2_ready;
    uint16_t function1_block_size;
    uint16_t function2_block_size;
    uint8_t chip_clock_csr;
    uint32_t chip_id_raw;
} ap6256_wifi_diag_t;

typedef struct {
    uint8_t wl_reg_on;
    uint8_t bt_reg_on;
    uint8_t reset_event_seen;
    uint8_t reset_status;
    uint8_t version_event_seen;
    uint8_t version_status;
    uint8_t hci_version;
    uint16_t hci_revision;
    uint8_t lmp_version;
    uint16_t manufacturer;
    uint16_t lmp_subversion;
    uint32_t event_frames_seen;
} ap6256_bt_diag_t;

typedef struct {
    uint8_t wl_reg_on;
    uint8_t bt_reg_on;
    uint8_t selected;
    uint8_t cmd5_ready;
    uint16_t rca;
    uint32_t ocr;
} ap6256_sdio_session_t;

void ap6256_init(void);
void ap6256_set_enables(uint8_t wl_on, uint8_t bt_on);
void ap6256_power_down(void);

ap6256_status_t ap6256_sdio_open(ap6256_sdio_session_t *session, uint8_t wl_on, uint8_t bt_on);
void ap6256_sdio_close(void);
ap6256_status_t ap6256_sdio_cmd52_read_u8(uint8_t function, uint32_t address, uint8_t *value_out);
ap6256_status_t ap6256_sdio_cmd52_write_u8(uint8_t function, uint32_t address, uint8_t value);
ap6256_status_t ap6256_sdio_cmd53_read(uint8_t function,
                                       uint32_t address,
                                       uint8_t *data,
                                       uint32_t len,
                                       uint8_t block_mode,
                                       uint8_t op_code);
ap6256_status_t ap6256_sdio_cmd53_write(uint8_t function,
                                        uint32_t address,
                                        const uint8_t *data,
                                        uint32_t len,
                                        uint8_t block_mode,
                                        uint8_t op_code);
ap6256_status_t ap6256_sdio_enable_function(uint8_t function);
ap6256_status_t ap6256_sdio_set_block_size(uint8_t function, uint16_t block_size);
ap6256_status_t ap6256_bcm_prepare_control_bus(uint8_t *chip_clock_csr_out);
ap6256_status_t ap6256_bcm_set_backplane_window(uint32_t address);
ap6256_status_t ap6256_bcm_backplane_read32(uint32_t address, uint32_t *value_out);
ap6256_status_t ap6256_bcm_backplane_write32(uint32_t address, uint32_t value);
ap6256_status_t ap6256_bcm_backplane_read(uint32_t address, uint8_t *data, uint32_t len);
ap6256_status_t ap6256_bcm_backplane_write(uint32_t address, const uint8_t *data, uint32_t len);
uint16_t ap6256_bcm_chip_id_from_raw(uint32_t chip_id_raw);

ap6256_status_t ap6256_bt_open(uint8_t wl_on);
void ap6256_bt_close(void);
ap6256_status_t ap6256_bt_uart_write(const uint8_t *data, uint16_t len);
ap6256_status_t ap6256_bt_uart_read(uint8_t *data, uint16_t len, uint32_t timeout_ms);
ap6256_status_t ap6256_bt_hci_command(uint16_t opcode,
                                      const uint8_t *params,
                                      uint8_t params_len,
                                      uint8_t *ret_params,
                                      uint8_t *ret_len);
ap6256_status_t ap6256_bt_patchram_download(void);

ap6256_status_t ap6256_wifi_transport_probe(ap6256_wifi_diag_t *diag);
ap6256_status_t ap6256_bt_hci_probe(ap6256_bt_diag_t *diag);
const char *ap6256_status_to_string(ap6256_status_t status);

#endif
