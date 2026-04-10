#include "board_test.h"
#include "ap6256_bt_runtime.h"
#include "ap6256_wifi_runtime.h"

#include "ap6256_connectivity.h"
#include "ap6256_driver.h"
#include "test_uart.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static void format_wifi_transport_summary(char *buffer, size_t buffer_len,
                                          ap6256_status_t st,
                                          const ap6256_wifi_diag_t *diag)
{
    if ((buffer == NULL) || (buffer_len == 0U) || (diag == NULL)) {
        return;
    }

    (void)snprintf(buffer,
                   buffer_len,
                   "st=%s,wp=%u,bp=%u,ocr=%08lX,rca=%04X,c3=%u,c7=%u,cccr=%02X,sdio=%02X,clk=%02X,chip=%04X",
                   ap6256_status_to_string(st),
                   diag->wl_pin_level,
                   diag->bt_pin_level,
                   (unsigned long)diag->ocr,
                   diag->rca,
                   diag->cmd3_ok,
                   diag->cmd7_ok,
                   diag->cccr_rev,
                   diag->sdio_rev,
                   diag->chip_clock_csr,
                   ap6256_bcm_chip_id_from_raw(diag->chip_id_raw));
}

static void format_bt_transport_summary(char *buffer, size_t buffer_len,
                                        ap6256_status_t st,
                                        const ap6256_bt_diag_t *diag)
{
    if ((buffer == NULL) || (buffer_len == 0U) || (diag == NULL)) {
        return;
    }

    (void)snprintf(buffer,
                   buffer_len,
                   "st=%s,wl=%u,reset=%u/s%02X,ver=%u/s%02X,hci=0x%02X,mfg=0x%04X",
                   ap6256_status_to_string(st),
                   diag->wl_reg_on,
                   diag->reset_event_seen,
                   diag->reset_status,
                   diag->version_event_seen,
                   diag->version_status,
                   diag->hci_version,
                   diag->manufacturer);
}

void test_wifi_connect(board_test_result_t *result)
{
    char measured[160];
    char detail[160];
    ap6256_wifi_runtime_summary_t summary;
    ap6256_status_t st;
    ap6256_wifi_diag_t diag;

    memset(&summary, 0, sizeof(summary));
    memset(&diag, 0, sizeof(diag));
    memset(detail, 0, sizeof(detail));

    test_uart_write_str("[ INFO ] wifi.connect stage: transport preflight\r\n");
    st = ap6256_connectivity_probe_wifi_transport(&diag);
    format_wifi_transport_summary(measured, sizeof(measured), st, &diag);

    if ((st != AP6256_STATUS_OK) || (diag.cmd5_ready == 0U)) {
        board_test_set_result(result,
                              "wifi.connect",
                              "AP6256 Wi-Fi",
                              TEST_STATUS_FAIL,
                              TEST_MODE_INTERACTIVE,
                              measured,
                              "AP6256 Wi-Fi transport preflight failed before any scan/join attempt.",
                              "Check WL_REG_ON path, SDMMC1 routing, and 3V3_WIFI rail.");
        return;
    }

    test_uart_write_str("[ INFO ] wifi.connect stage: runtime interactive start\r\n");
    st = ap6256_wifi_runtime_run_interactive(&summary, detail, sizeof(detail));
    (void)snprintf(measured,
                   sizeof(measured),
                   "st=%s,scan=%u,rssi=%d,ssid=%s,ip=%s",
                   ap6256_status_to_string(st),
                   summary.scan_results_count,
                   (int)summary.selected_rssi,
                   (summary.selected_ssid[0] != '\0') ? summary.selected_ssid : "n/a",
                   (summary.leased_ip[0] != '\0') ? summary.leased_ip : "n/a");

    if (st == AP6256_STATUS_OK) {
        board_test_set_result(result,
                              "wifi.connect",
                              "AP6256 Wi-Fi",
                              TEST_STATUS_PASS,
                              TEST_MODE_INTERACTIVE,
                              measured,
                              detail,
                              "");
        return;
    }

    board_test_set_result(result,
                          "wifi.connect",
                          "AP6256 Wi-Fi",
                          TEST_STATUS_FAIL,
                          TEST_MODE_INTERACTIVE,
                          measured,
                          detail,
                          "Check AP6256 Wi-Fi transport, firmware assets, SSID selection, credentials, and DHCP availability.");
}

void test_wifi_sdio_presence(board_test_result_t *result)
{
    char measured[128];
    ap6256_status_t st;
    ap6256_wifi_diag_t diag;
    const ap6256_wifi_state_t *state;

    memset(&diag, 0, sizeof(diag));
    st = ap6256_connectivity_probe_wifi_transport(&diag);
    state = ap6256_connectivity_get_wifi_state();

    if ((st == AP6256_STATUS_OK) && (diag.cmd5_ready != 0U)) {
        format_wifi_transport_summary(measured, sizeof(measured), st, &diag);

        board_test_set_result(result,
                              "wifi.sdio",
                              "AP6256 Wi-Fi SDIO",
                              TEST_STATUS_PASS,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "AP6256 SDIO transport driver probe passed (CMD5 ready + CMD52 reads).",
                              "Diagnostic only. End-to-end qualification lives in wifi.connect.");
    } else {
        (void)snprintf(measured,
                       sizeof(measured),
                       "st=%s,wp=%u,bp=%u,rdy=%u,dc5=%u,rca=%04X,c3=%u,c7=%u,c5=%u,c52ok=%u,c52=%u",
                       ap6256_status_to_string(st),
                       diag.wl_pin_level,
                       diag.bt_pin_level,
                       diag.cmd5_ready,
                       diag.disabled_cmd5_response,
                       diag.rca,
                       diag.cmd3_ok,
                       diag.cmd7_ok,
                       diag.cmd5_attempts,
                       diag.cccr_read_ok,
                       diag.cccr_read_attempts);
        board_test_set_result(result,
                              "wifi.sdio",
                              "AP6256 Wi-Fi SDIO",
                              TEST_STATUS_WARN,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "AP6256 SDIO transport probe did not reach CMD5-ready + required CMD52 CCCR read.",
                              "Diagnostic only. Check WL_REG_ON path, 3V3_WIFI rail, SDMMC1 routing, and command line integrity.");
    }

    if ((state != NULL) && (state->last_error[0] != '\0')) {
        test_uart_printf("[ INFO ] wifi.sdio note: %s\r\n", state->last_error);
    }
}

void test_bt_ble_link(board_test_result_t *result)
{
    char measured[128];
    char detail[160];
    ap6256_bt_runtime_summary_t summary;
    ap6256_status_t st;
    ap6256_bt_diag_t diag;

    memset(&summary, 0, sizeof(summary));
    memset(&diag, 0, sizeof(diag));
    memset(detail, 0, sizeof(detail));
    st = ap6256_connectivity_probe_bt_transport(&diag);
    format_bt_transport_summary(measured, sizeof(measured), st, &diag);

    if ((st != AP6256_STATUS_OK) ||
        (diag.reset_event_seen == 0U) ||
        (diag.version_event_seen == 0U) ||
        (diag.reset_status != 0x00U) ||
        (diag.version_status != 0x00U)) {
        board_test_set_result(result,
                              "bt.ble_link",
                              "AP6256 Bluetooth",
                              TEST_STATUS_FAIL,
                              TEST_MODE_INTERACTIVE,
                              measured,
                              "AP6256 Bluetooth transport preflight failed before BLE scan/connect.",
                              "Check BT_REG_ON path, UART3 routing, CTS/RTS behavior, and 3V3_WIFI rail.");
        return;
    }

    st = ap6256_bt_runtime_run_interactive(&summary, detail, sizeof(detail));
    (void)snprintf(measured,
                   sizeof(measured),
                   "st=%s,dev=%u,svc=%u,patch=%u,addr=%s",
                   ap6256_status_to_string(st),
                   summary.device_count,
                   summary.service_count,
                   summary.patchram_loaded,
                   (summary.selected_address[0] != '\0') ? summary.selected_address : "n/a");

    if (st == AP6256_STATUS_OK) {
        board_test_set_result(result,
                              "bt.ble_link",
                              "AP6256 Bluetooth",
                              TEST_STATUS_PASS,
                              TEST_MODE_INTERACTIVE,
                              measured,
                              detail,
                              "");
        return;
    }

    board_test_set_result(result,
                          "bt.ble_link",
                          "AP6256 Bluetooth",
                          TEST_STATUS_FAIL,
                          TEST_MODE_INTERACTIVE,
                          measured,
                          detail,
                          "Check AP6256 BT power, UART3 routing, PatchRAM asset, and BLE target behavior.");
}

void test_bt_uart_hci(board_test_result_t *result)
{
    char measured[128];
    ap6256_status_t st;
    ap6256_bt_diag_t diag;
    const ap6256_bt_state_t *state;
    uint8_t bt_hci_ok;

    memset(&diag, 0, sizeof(diag));
    st = ap6256_connectivity_probe_bt_transport(&diag);
    state = ap6256_connectivity_get_bt_state();
    bt_hci_ok = ((st == AP6256_STATUS_OK) &&
                 (diag.reset_event_seen != 0U) &&
                 (diag.version_event_seen != 0U) &&
                 (diag.reset_status == 0x00U) &&
                 (diag.version_status == 0x00U)) ? 1U : 0U;

    format_bt_transport_summary(measured, sizeof(measured), st, &diag);

    if (bt_hci_ok != 0U) {
        board_test_set_result(result,
                              "bt.hci",
                              "AP6256 Bluetooth UART",
                              TEST_STATUS_PASS,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "AP6256 HCI transport probe passed (reset + read-version command complete).",
                              "Diagnostic only. End-to-end qualification lives in bt.ble_link.");
    } else {
        board_test_set_result(result,
                              "bt.hci",
                              "AP6256 Bluetooth UART",
                              TEST_STATUS_WARN,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "AP6256 HCI transport probe did not receive expected command-complete sequence.",
                              "Diagnostic only. Check BT_REG_ON path, 3V3_WIFI rail, UART3 routing, and CTS/RTS line behavior.");
    }

    if ((state != NULL) && (state->last_error[0] != '\0')) {
        test_uart_printf("[ INFO ] bt.hci note: %s\r\n", state->last_error);
    }
}

void test_wifi_print_info(void)
{
    ap6256_connectivity_print_wifi_info();
}

void test_bt_print_info(void)
{
    ap6256_connectivity_print_bt_info();
    test_uart_write_str("AP6256 control lines: WL_REG_ON=PC7, BT_REG_ON=PD13 (driver-managed sequencing)\r\n");
}
