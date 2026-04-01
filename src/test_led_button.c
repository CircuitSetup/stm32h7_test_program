#include "board_test.h"

void test_led_button(board_test_result_t *result)
{
    board_test_set_result(result,
                          "human.led_button",
                          "LED/Button",
                          TEST_STATUS_SKIP,
                          TEST_MODE_SANITY,
                          "none present",
                          "No MCU-controlled user LED or user button is present on this revision. S1 is reset-only.",
                          "Observe power LED D3 and reset switch S1 manually if needed.");
}
