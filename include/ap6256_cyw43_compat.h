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
#define AP6256_CYW43_RAM_BASE               0x00198000UL
#define AP6256_CYW43_RAM_SIZE_BYTES          0x000C8000UL

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

#endif /* AP6256_CYW43_COMPAT_H */
