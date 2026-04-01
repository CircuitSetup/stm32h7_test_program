#include "board_test.h"

#include <stdio.h>

static void drive_safe_output(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    HAL_GPIO_WritePin(port, pin, state);
    HAL_Delay(2U);
}

void test_gpio_walk_outputs(board_test_result_t *result)
{
    const board_gpio_policy_t *policy;
    size_t count = 0U;
    size_t i;
    uint32_t toggled = 0U;
    char measured[64];

    policy = board_map_get_gpio_policy(&count);

    for (i = 0U; i < count; ++i) {
        if (policy[i].gpio_class != GPIO_CLASS_SAFE_OUTPUT) {
            continue;
        }

        drive_safe_output(policy[i].port, policy[i].pin, GPIO_PIN_SET);
        drive_safe_output(policy[i].port, policy[i].pin, GPIO_PIN_RESET);
        toggled++;
    }

    (void)snprintf(measured, sizeof(measured), "safe_outputs_toggled=%lu", (unsigned long)toggled);

    if (toggled == 0U) {
        board_test_set_result(result,
                              "gpio.safe_outputs",
                              "GPIO",
                              TEST_STATUS_SKIP,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "No safe outputs are defined in GPIO policy map.",
                              "");
    } else {
        board_test_set_result(result,
                              "gpio.safe_outputs",
                              "GPIO",
                              TEST_STATUS_PASS,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "Walking-output pattern executed only on explicitly safe outputs.",
                              "");
    }
}

void test_gpio_log_inputs(board_test_result_t *result)
{
    const board_gpio_policy_t *policy;
    size_t count = 0U;
    size_t i;
    uint32_t high_count = 0U;
    uint32_t input_count = 0U;
    char measured[80];

    policy = board_map_get_gpio_policy(&count);

    for (i = 0U; i < count; ++i) {
        if (policy[i].gpio_class != GPIO_CLASS_SAFE_INPUT) {
            continue;
        }

        input_count++;
        if (HAL_GPIO_ReadPin(policy[i].port, policy[i].pin) == GPIO_PIN_SET) {
            high_count++;
        }
    }

    (void)snprintf(measured,
                   sizeof(measured),
                   "safe_inputs=%lu,high=%lu,low=%lu",
                   (unsigned long)input_count,
                   (unsigned long)high_count,
                   (unsigned long)(input_count - high_count));

    if (input_count == 0U) {
        board_test_set_result(result,
                              "gpio.safe_inputs",
                              "GPIO",
                              TEST_STATUS_SKIP,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "No safe inputs defined in GPIO policy map.",
                              "");
    } else {
        board_test_set_result(result,
                              "gpio.safe_inputs",
                              "GPIO",
                              TEST_STATUS_PASS,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "Input levels sampled without toggling reserved or dangerous pins.",
                              "");
    }
}
