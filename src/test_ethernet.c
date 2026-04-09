#include "board_test.h"
#include "network_ethernet.h"

void test_ethernet_phy(board_test_result_t *result)
{
    network_ethernet_test_phy(result);
}

void test_ethernet_link(board_test_result_t *result)
{
    network_ethernet_test_link(result);
}

void test_ethernet_print_info(void)
{
    network_ethernet_print_info();
}
