#ifndef NETWORK_ETHERNET_H
#define NETWORK_ETHERNET_H

#include "board_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void network_ethernet_boot_start(void);
void network_ethernet_poll(void);
void network_ethernet_suspend(void);
void network_ethernet_resume(void);

void network_ethernet_test_phy(board_test_result_t *result);
void network_ethernet_test_link(board_test_result_t *result);
void network_ethernet_print_info(void);

#ifdef __cplusplus
}
#endif

#endif /* NETWORK_ETHERNET_H */
