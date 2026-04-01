#include "board_test.h"

#include "test_ade7816.h"
#include "test_uart.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define ADE_REG_PERIOD 0xE607U
#define ADE_REG_VRMS   0x43C0U
#define ADE_REG_IARMS  0x43C1U
#define ADE_REG_IBRMS  0x43C2U
#define ADE_REG_ICRMS  0x43C3U
#define ADE_REG_IDRMS  0x43C4U
#define ADE_REG_IERMS  0x43C5U
#define ADE_REG_IFRMS  0x43C6U

static uint8_t s_analog_fixture_explicit = 0U;

void test_analog_fixture_set_explicit(uint8_t explicit_request)
{
    s_analog_fixture_explicit = (explicit_request != 0U) ? 1U : 0U;
}

static bool read_device_rms(uint8_t index,
                            uint32_t *period,
                            uint32_t *vrms,
                            uint32_t *irms_a,
                            uint32_t *irms_b,
                            uint32_t *irms_c,
                            uint32_t *irms_d,
                            uint32_t *irms_e,
                            uint32_t *irms_f)
{
    return ade7816_read_reg(index, ADE_REG_PERIOD, 2U, period) &&
           ade7816_read_reg(index, ADE_REG_VRMS, 4U, vrms) &&
           ade7816_read_reg(index, ADE_REG_IARMS, 4U, irms_a) &&
           ade7816_read_reg(index, ADE_REG_IBRMS, 4U, irms_b) &&
           ade7816_read_reg(index, ADE_REG_ICRMS, 4U, irms_c) &&
           ade7816_read_reg(index, ADE_REG_IDRMS, 4U, irms_d) &&
           ade7816_read_reg(index, ADE_REG_IERMS, 4U, irms_e) &&
           ade7816_read_reg(index, ADE_REG_IFRMS, 4U, irms_f);
}

void test_analog_sanity(board_test_result_t *result)
{
    const board_ade_device_map_t *devs;
    size_t count = 0U;
    size_t i;
    uint32_t ok = 0U;
    uint32_t nonzero = 0U;
    char measured[96];

    devs = board_map_get_ade_devices(&count);

    for (i = 0U; i < count; ++i) {
        uint32_t period = 0U;
        uint32_t vrms = 0U;
        uint32_t ia = 0U;
        uint32_t ib = 0U;
        uint32_t ic = 0U;
        uint32_t id = 0U;
        uint32_t ie = 0U;
        uint32_t ifr = 0U;

        if (!read_device_rms(devs[i].index, &period, &vrms, &ia, &ib, &ic, &id, &ie, &ifr)) {
            continue;
        }

        ok++;
        if ((vrms > 0U) || (ia > 0U) || (ib > 0U) || (ic > 0U) || (id > 0U) || (ie > 0U) || (ifr > 0U)) {
            nonzero++;
        }
    }

    (void)snprintf(measured,
                   sizeof(measured),
                   "devices_read=%lu/%lu,nonzero=%lu",
                   (unsigned long)ok,
                   (unsigned long)count,
                   (unsigned long)nonzero);

    if (ok != count) {
        board_test_set_result(result,
                              "analog.sanity",
                              "Metering AFE",
                              TEST_STATUS_WARN,
                              TEST_MODE_SANITY,
                              measured,
                              "Could not read one or more RMS register sets.",
                              "Run 'run ade' and inspect affected device register dumps.");
    } else {
        board_test_set_result(result,
                              "analog.sanity",
                              "Metering AFE",
                              TEST_STATUS_PASS,
                              TEST_MODE_SANITY,
                              measured,
                              "RMS/period register read path is alive on all ADE devices.",
                              "Absolute metrology accuracy requires fixture stimulus.");
    }
}

void test_analog_fixture(board_test_result_t *result)
{
    const board_ade_device_map_t *devs;
    const board_current_channel_map_t *channels;
    size_t dev_count = 0U;
    size_t ch_count = 0U;
    size_t i;
    uint32_t active_currents = 0U;
    uint32_t active_voltages = 0U;
    char measured[96];

    if (s_analog_fixture_explicit == 0U) {
        board_test_set_result(result,
                              "analog.fixture",
                              "Metering AFE fixture",
                              TEST_STATUS_REQUIRES_FIXTURE,
                              TEST_MODE_FIXTURE,
                              "not_run_in_auto=1",
                              "Fixture mode not explicitly requested during automatic sequence.",
                              "Run 'run analog_fixture' when low-voltage isolated stimulus fixture is connected.");
        return;
    }

    devs = board_map_get_ade_devices(&dev_count);
    channels = board_map_get_current_channels(&ch_count);

    test_uart_write_str("\r\n[ INFO ] analog.fixture: raw RMS/period report\r\n");
    test_uart_write_str("[ INFO ] analog.fixture: Apply isolated low-voltage AC stimulus only. Do NOT use live mains for this step.\r\n");

    for (i = 0U; i < dev_count; ++i) {
        uint32_t period = 0U;
        uint32_t vrms = 0U;
        uint32_t ia = 0U;
        uint32_t ib = 0U;
        uint32_t ic = 0U;
        uint32_t id = 0U;
        uint32_t ie = 0U;
        uint32_t ifr = 0U;

        if (!read_device_rms(devs[i].index, &period, &vrms, &ia, &ib, &ic, &id, &ie, &ifr)) {
            test_uart_printf("[ INFO ] analog.fixture: ADE[%u] read failed\r\n", devs[i].index);
            continue;
        }

        if (vrms > 1000U) {
            active_voltages++;
        }
        if ((ia > 1000U) || (ib > 1000U) || (ic > 1000U) || (id > 1000U) || (ie > 1000U) || (ifr > 1000U)) {
            active_currents++;
        }

        test_uart_printf("[ INFO ] analog.fixture: ADE[%u]/%s VP=%s VN=%s SRC=%s PERIOD=%lu VRMS=%lu IA=%lu IB=%lu IC=%lu ID=%lu IE=%lu IF=%lu\r\n",
                         devs[i].index,
                         devs[i].refdes,
                         devs[i].vp_net,
                         devs[i].vn_net,
                         devs[i].fixture_voltage_source,
                         (unsigned long)period,
                         (unsigned long)vrms,
                         (unsigned long)ia,
                         (unsigned long)ib,
                         (unsigned long)ic,
                         (unsigned long)id,
                         (unsigned long)ie,
                         (unsigned long)ifr);
    }

    test_uart_write_str("[ INFO ] analog.fixture: channel map (for swap detection)\r\n");
    for (i = 0U; i < ch_count; ++i) {
        test_uart_printf("[ INFO ] analog.fixture: CH%02u ADE%u%c IP=%s IN=%s CONN=%s.%s\r\n",
                         channels[i].channel_index,
                         channels[i].ade_index,
                         channels[i].phase_name,
                         channels[i].ip_net,
                         channels[i].in_net,
                         channels[i].connector,
                         channels[i].connector_pin);
    }

    (void)snprintf(measured,
                   sizeof(measured),
                   "active_voltage_devices=%lu,active_current_devices=%lu",
                   (unsigned long)active_voltages,
                   (unsigned long)active_currents);

    if ((active_voltages == 0U) && (active_currents == 0U)) {
        board_test_set_result(result,
                              "analog.fixture",
                              "Metering AFE fixture",
                              TEST_STATUS_REQUIRES_FIXTURE,
                              TEST_MODE_FIXTURE,
                              measured,
                              "No significant RMS activity detected; likely no injected fixture stimulus.",
                              "Inject low-voltage isolated AC stimulus and rerun 'run analog_fixture'.");
    } else {
        board_test_set_result(result,
                              "analog.fixture",
                              "Metering AFE fixture",
                              TEST_STATUS_PASS,
                              TEST_MODE_FIXTURE,
                              measured,
                              "RMS activity detected under fixture mode.",
                              "Compare measured values against fixture expected tolerances.");
    }
}
