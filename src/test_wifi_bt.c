#include "board_test.h"

#include "ap6256_driver.h"
#include "test_uart.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint8_t s_wifi_present = 0U;
static uint8_t s_bt_hci_ok = 0U;
static ap6256_status_t s_wifi_status = AP6256_STATUS_TIMEOUT;
static ap6256_status_t s_bt_status = AP6256_STATUS_TIMEOUT;
static ap6256_wifi_diag_t s_wifi_diag;
static ap6256_bt_diag_t s_bt_diag;

void test_wifi_sdio_presence(board_test_result_t *result)
{
    char measured[128];
    ap6256_status_t st;

    memset(&s_wifi_diag, 0, sizeof(s_wifi_diag));
    st = ap6256_wifi_transport_probe(&s_wifi_diag);
    s_wifi_status = st;

    if ((st == AP6256_STATUS_OK) && (s_wifi_diag.cmd5_ready != 0U)) {
        s_wifi_present = 1U;
        (void)snprintf(measured,
                       sizeof(measured),
                       "st=%s,wp=%u,bp=%u,ocr=%08lX,rca=%04X,c3=%u,c7=%u,cccr=%02X,sdio=%02X,c5=%u,c52=%u",
                       ap6256_status_to_string(st),
                       s_wifi_diag.wl_pin_level,
                       s_wifi_diag.bt_pin_level,
                       (unsigned long)s_wifi_diag.ocr,
                       s_wifi_diag.rca,
                       s_wifi_diag.cmd3_ok,
                       s_wifi_diag.cmd7_ok,
                       s_wifi_diag.cccr_rev,
                       s_wifi_diag.sdio_rev,
                       s_wifi_diag.cmd5_attempts,
                       s_wifi_diag.cccr_read_attempts);

        board_test_set_result(result,
                              "wifi.sdio",
                              "AP6256 Wi-Fi SDIO",
                              TEST_STATUS_PASS,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "AP6256 SDIO transport driver probe passed (CMD5 ready + CMD52 reads).",
                              "Full Wi-Fi networking requires driver stack + firmware download.");
    } else {
        s_wifi_present = 0U;
        (void)snprintf(measured,
                       sizeof(measured),
                       "st=%s,wp=%u,bp=%u,rdy=%u,dc5=%u,rca=%04X,c3=%u,c7=%u,c5=%u,c52ok=%u,c52=%u",
                       ap6256_status_to_string(st),
                       s_wifi_diag.wl_pin_level,
                       s_wifi_diag.bt_pin_level,
                       s_wifi_diag.cmd5_ready,
                       s_wifi_diag.disabled_cmd5_response,
                       s_wifi_diag.rca,
                       s_wifi_diag.cmd3_ok,
                       s_wifi_diag.cmd7_ok,
                       s_wifi_diag.cmd5_attempts,
                       s_wifi_diag.cccr_read_ok,
                       s_wifi_diag.cccr_read_attempts);
        board_test_set_result(result,
                              "wifi.sdio",
                              "AP6256 Wi-Fi SDIO",
                              TEST_STATUS_WARN,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "AP6256 SDIO transport probe did not reach CMD5-ready + required CMD52 CCCR read.",
                              "Check WL_REG_ON path, 3V3_WIFI rail, SDMMC1 routing, and command line integrity.");
    }
}

void test_bt_uart_hci(board_test_result_t *result)
{
    char measured[128];
    ap6256_status_t st;

    memset(&s_bt_diag, 0, sizeof(s_bt_diag));
    st = ap6256_bt_hci_probe(&s_bt_diag);
    s_bt_status = st;
    s_bt_hci_ok = ((st == AP6256_STATUS_OK) &&
                   (s_bt_diag.reset_event_seen != 0U) &&
                   (s_bt_diag.version_event_seen != 0U) &&
                   (s_bt_diag.reset_status == 0x00U) &&
                   (s_bt_diag.version_status == 0x00U)) ? 1U : 0U;

    (void)snprintf(measured,
                    sizeof(measured),
                    "st=%s,wl=%u,reset=%u/s%02X,ver=%u/s%02X,hci=0x%02X,mfg=0x%04X",
                    ap6256_status_to_string(st),
                    s_bt_diag.wl_reg_on,
                    s_bt_diag.reset_event_seen,
                    s_bt_diag.reset_status,
                    s_bt_diag.version_event_seen,
                    s_bt_diag.version_status,
                    s_bt_diag.hci_version,
                    s_bt_diag.manufacturer);

    if (s_bt_hci_ok != 0U) {
        board_test_set_result(result,
                              "bt.hci",
                              "AP6256 Bluetooth UART",
                              TEST_STATUS_PASS,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "AP6256 HCI transport probe passed (reset + read-version command complete).",
                              "Full BT functionality requires host driver stack.");
    } else {
        board_test_set_result(result,
                              "bt.hci",
                              "AP6256 Bluetooth UART",
                              TEST_STATUS_WARN,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "AP6256 HCI transport probe did not receive expected command-complete sequence.",
                              "Check BT_REG_ON path, 3V3_WIFI rail, UART3 routing, and CTS/RTS line behavior.");
    }
}

void test_wifi_print_info(void)
{
    test_uart_printf("Wi-Fi SDIO: %s (status=%s) OCR=0x%08lX CMD5_ready=%u tries=%u\r\n",
                     (s_wifi_present != 0U) ? "detected" : "not detected",
                     ap6256_status_to_string(s_wifi_status),
                     (unsigned long)s_wifi_diag.ocr,
                     s_wifi_diag.cmd5_ready,
                     s_wifi_diag.cmd5_attempts);
    test_uart_printf("  CCCR=0x%02X SDIO_REV=0x%02X IOE=0x%02X IOR=0x%02X CIS=0x%06lX disabled_cmd5=%u\r\n",
                     s_wifi_diag.cccr_rev,
                     s_wifi_diag.sdio_rev,
                     s_wifi_diag.io_enable,
                     s_wifi_diag.io_ready,
                     (unsigned long)s_wifi_diag.cis_ptr,
                     s_wifi_diag.disabled_cmd5_response);
    test_uart_printf("  WL/BT requested=%u/%u pin_level=%u/%u CMD52_CCCR ok=%u tries=%u\r\n",
                     s_wifi_diag.wl_reg_on,
                     s_wifi_diag.bt_reg_on,
                     s_wifi_diag.wl_pin_level,
                     s_wifi_diag.bt_pin_level,
                     s_wifi_diag.cccr_read_ok,
                     s_wifi_diag.cccr_read_attempts);
    test_uart_printf("  CMD3=%u CMD7=%u RCA=0x%04X\r\n",
                     s_wifi_diag.cmd3_ok,
                     s_wifi_diag.cmd7_ok,
                     s_wifi_diag.rca);

    test_uart_printf("BT HCI: %s (status=%s) reset=%u/s%02X ver=%u/s%02X frames=%lu\r\n",
                     (s_bt_hci_ok != 0U) ? "responsive" : "not responsive",
                     ap6256_status_to_string(s_bt_status),
                     s_bt_diag.reset_event_seen,
                     s_bt_diag.reset_status,
                     s_bt_diag.version_event_seen,
                     s_bt_diag.version_status,
                     (unsigned long)s_bt_diag.event_frames_seen);
    test_uart_printf("  HCI_VER=0x%02X HCI_REV=0x%04X LMP_VER=0x%02X MFG=0x%04X LMP_SUB=0x%04X\r\n",
                     s_bt_diag.hci_version,
                     s_bt_diag.hci_revision,
                     s_bt_diag.lmp_version,
                     s_bt_diag.manufacturer,
                     s_bt_diag.lmp_subversion);
    test_uart_write_str("AP6256 control lines: WL_REG_ON=PC7, BT_REG_ON=PD13 (driver-managed sequencing)\r\n");
}
