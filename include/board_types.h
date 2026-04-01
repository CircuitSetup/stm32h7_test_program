#ifndef BOARD_TYPES_H
#define BOARD_TYPES_H

#include <stdint.h>

typedef enum {
    TEST_STATUS_PASS = 0,
    TEST_STATUS_FAIL,
    TEST_STATUS_WARN,
    TEST_STATUS_SKIP,
    TEST_STATUS_REQUIRES_FIXTURE
} test_status_t;

typedef enum {
    TEST_MODE_AUTOMATIC = 0,
    TEST_MODE_INTERACTIVE,
    TEST_MODE_FIXTURE,
    TEST_MODE_SANITY
} test_mode_t;

typedef struct {
    char test_name[40];
    char component[40];
    test_status_t status;
    test_mode_t mode;
    char measured_value_text[128];
    char detail[192];
    char operator_action_required[128];
    uint32_t timestamp_ms;
} board_test_result_t;

#endif
