#ifndef AP6256_ASSETS_H
#define AP6256_ASSETS_H

#include <stdint.h>

typedef enum {
    AP6256_ASSET_WIFI_FIRMWARE = 0,
    AP6256_ASSET_WIFI_CLM_BLOB,
    AP6256_ASSET_WIFI_NVRAM,
    AP6256_ASSET_BT_PATCHRAM,
    AP6256_ASSET_WIFI_NVRAM_REFERENCE,
    AP6256_ASSET_COUNT
} ap6256_asset_id_t;

typedef struct {
    ap6256_asset_id_t id;
    const char *tag;
    const char *filename;
    const char *sha256;
    uint32_t size;
    const uint8_t *data;
} ap6256_embedded_asset_t;

const ap6256_embedded_asset_t *ap6256_assets_get(ap6256_asset_id_t id);
const ap6256_embedded_asset_t *ap6256_assets_wifi_firmware(void);
const ap6256_embedded_asset_t *ap6256_assets_wifi_clm_blob(void);
const ap6256_embedded_asset_t *ap6256_assets_wifi_nvram(void);
const ap6256_embedded_asset_t *ap6256_assets_bt_patchram(void);
const ap6256_embedded_asset_t *ap6256_assets_reference_nvram(void);
uint8_t ap6256_assets_ready(void);

#endif /* AP6256_ASSETS_H */
