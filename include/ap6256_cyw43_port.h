#ifndef AP6256_CYW43_PORT_H
#define AP6256_CYW43_PORT_H

#include <stdint.h>

void ap6256_cyw43_port_init(void);
void ap6256_cyw43_port_deinit(void);
void ap6256_cyw43_port_poll(void);
uint8_t ap6256_cyw43_port_pending_poll(void);

#endif /* AP6256_CYW43_PORT_H */
