#ifndef BOARD_MAP_H
#define BOARD_MAP_H

#include "main.h"
#include "board_types.h"

typedef enum {
    GPIO_CLASS_SAFE_OUTPUT = 0,
    GPIO_CLASS_SAFE_INPUT,
    GPIO_CLASS_BIDIRECTIONAL_BUS,
    GPIO_CLASS_DO_NOT_TOGGLE,
    GPIO_CLASS_BOOT_OR_DEBUG_RESERVED,
    GPIO_CLASS_ANALOG_ONLY,
    GPIO_CLASS_DANGEROUS
} gpio_class_t;

typedef struct {
    const char *mcu_pin;
    GPIO_TypeDef *port;
    uint16_t pin;
    const char *net;
    const char *peripheral;
    const char *external_component;
    test_mode_t test_mode;
    const char *test_method;
    const char *pass_fail_criteria;
} board_signal_map_t;

typedef struct {
    const char *mcu_pin;
    GPIO_TypeDef *port;
    uint16_t pin;
    const char *net;
    gpio_class_t gpio_class;
    const char *note;
} board_gpio_policy_t;

typedef struct {
    uint8_t index;
    const char *refdes;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    const char *cs_net;
    GPIO_TypeDef *irq_port;
    uint16_t irq_pin;
    const char *irq_net;
    const char *clk_net;
    const char *vp_net;
    const char *vn_net;
    const char *fixture_voltage_source;
    const char *fixture_current_connector;
} board_ade_device_map_t;

typedef struct {
    uint8_t channel_index;
    uint8_t ade_index;
    char phase_name;
    const char *ip_net;
    const char *in_net;
    const char *connector;
    const char *connector_pin;
} board_current_channel_map_t;

typedef struct {
    const char *rail_name;
    const char *observation_method;
    const char *details;
} board_rail_observability_t;

const board_signal_map_t *board_map_get_signals(size_t *count);
const board_gpio_policy_t *board_map_get_gpio_policy(size_t *count);
const board_ade_device_map_t *board_map_get_ade_devices(size_t *count);
const board_current_channel_map_t *board_map_get_current_channels(size_t *count);
const board_rail_observability_t *board_map_get_rails(size_t *count);

const board_ade_device_map_t *board_map_find_ade_device(uint8_t index);
const char *board_map_status_to_string(test_status_t status);
const char *board_map_mode_to_string(test_mode_t mode);
const char *board_map_gpio_class_to_string(gpio_class_t gpio_class);

#endif
