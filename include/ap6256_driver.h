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

void ap6256_init(void);
void ap6256_set_enables(uint8_t wl_on, uint8_t bt_on);
void ap6256_power_down(void);

ap6256_status_t ap6256_wifi_transport_probe(ap6256_wifi_diag_t *diag);
ap6256_status_t ap6256_bt_hci_probe(ap6256_bt_diag_t *diag);
const char *ap6256_status_to_string(ap6256_status_t status);

#endif
