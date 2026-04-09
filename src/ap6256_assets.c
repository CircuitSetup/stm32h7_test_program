#include "ap6256_assets.h"

#include "generated/ap6256_assets_manifest.h"

#include <stddef.h>

static const ap6256_embedded_asset_t s_assets[AP6256_ASSET_COUNT] = {
    {
        AP6256_ASSET_WIFI_FIRMWARE,
        "wifi_fw",
        AP6256_ASSET_WIFI_FIRMWARE_FILENAME,
        AP6256_ASSET_WIFI_FIRMWARE_SHA256,
        AP6256_ASSET_WIFI_FIRMWARE_SIZE,
        ap6256_asset_wifi_firmware_start
    },
    {
        AP6256_ASSET_WIFI_CLM_BLOB,
        "wifi_clm",
        AP6256_ASSET_WIFI_CLM_BLOB_FILENAME,
        AP6256_ASSET_WIFI_CLM_BLOB_SHA256,
        AP6256_ASSET_WIFI_CLM_BLOB_SIZE,
        ap6256_asset_wifi_clm_blob_start
    },
    {
        AP6256_ASSET_WIFI_NVRAM,
        "wifi_nvram",
        AP6256_ASSET_WIFI_NVRAM_FILENAME,
        AP6256_ASSET_WIFI_NVRAM_SHA256,
        AP6256_ASSET_WIFI_NVRAM_SIZE,
        ap6256_asset_wifi_nvram_start
    },
    {
        AP6256_ASSET_BT_PATCHRAM,
        "bt_patchram",
        AP6256_ASSET_BT_PATCHRAM_FILENAME,
        AP6256_ASSET_BT_PATCHRAM_SHA256,
        AP6256_ASSET_BT_PATCHRAM_SIZE,
        ap6256_asset_bt_patchram_start
    },
    {
        AP6256_ASSET_WIFI_NVRAM_REFERENCE,
        "wifi_nvram_reference",
        AP6256_ASSET_WIFI_NVRAM_REFERENCE_FILENAME,
        AP6256_ASSET_WIFI_NVRAM_REFERENCE_SHA256,
        AP6256_ASSET_WIFI_NVRAM_REFERENCE_SIZE,
        ap6256_asset_wifi_nvram_reference_start
    }
};

const ap6256_embedded_asset_t *ap6256_assets_get(ap6256_asset_id_t id)
{
    if ((uint32_t)id >= AP6256_ASSET_COUNT) {
        return NULL;
    }

    return &s_assets[(uint32_t)id];
}

const ap6256_embedded_asset_t *ap6256_assets_wifi_firmware(void)
{
    return ap6256_assets_get(AP6256_ASSET_WIFI_FIRMWARE);
}

const ap6256_embedded_asset_t *ap6256_assets_wifi_clm_blob(void)
{
    return ap6256_assets_get(AP6256_ASSET_WIFI_CLM_BLOB);
}

const ap6256_embedded_asset_t *ap6256_assets_wifi_nvram(void)
{
    return ap6256_assets_get(AP6256_ASSET_WIFI_NVRAM);
}

const ap6256_embedded_asset_t *ap6256_assets_bt_patchram(void)
{
    return ap6256_assets_get(AP6256_ASSET_BT_PATCHRAM);
}

const ap6256_embedded_asset_t *ap6256_assets_reference_nvram(void)
{
    return ap6256_assets_get(AP6256_ASSET_WIFI_NVRAM_REFERENCE);
}

uint8_t ap6256_assets_ready(void)
{
    size_t i;

    for (i = 0U; i < AP6256_ASSET_COUNT; ++i) {
        if ((s_assets[i].data == NULL) || (s_assets[i].size == 0U) ||
            (s_assets[i].filename == NULL) || (s_assets[i].filename[0] == '\0')) {
            return 0U;
        }
    }

    return 1U;
}
