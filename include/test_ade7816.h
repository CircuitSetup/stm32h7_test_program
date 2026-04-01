#ifndef TEST_ADE7816_H
#define TEST_ADE7816_H

#include <stdbool.h>
#include <stdint.h>

bool ade7816_init_bus(void);
bool ade7816_read_reg(uint8_t index, uint16_t reg, uint8_t size_bytes, uint32_t *value);
bool ade7816_write_reg(uint8_t index, uint16_t reg, uint8_t size_bytes, uint32_t value);
bool ade7816_soft_reset(uint8_t index);

#endif
