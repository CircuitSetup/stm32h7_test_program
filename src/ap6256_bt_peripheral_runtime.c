#include "ap6256_bt_peripheral_runtime.h"

#include "ap6256_assets.h"
#include "ap6256_connectivity.h"
#include "ble/att_db.h"
#include "ble/att_server.h"
#include "bluetooth_data_types.h"
#include "btstack_chipset_bcm.h"
#include "btstack_defines.h"
#include "btstack_event.h"
#include "btstack_memory.h"
#include "btstack_run_loop_embedded.h"
#include "btstack_uart.h"
#include "btstack_uart_block.h"
#include "btstack_util.h"
#include "gap.h"
#include "generated/ap6256_bt_peripheral_gatt.h"
#include "hal_uart_dma.h"
#include "hci.h"
#include "hci_transport.h"
#include "hci_transport_h4.h"
#include "l2cap.h"
#include "network_manager.h"
#include "test_uart.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define AP6256_BT_PERIPHERAL_NAME                "Karios48 BLE Test"
#define AP6256_BT_PERIPHERAL_SERVICE_UUID        "7C4E0001-2D5B-4A6B-8F3C-9B4A3E280001"
#define AP6256_BT_PERIPHERAL_VALUE               "FW=karios48_board_test;BT=OK"
#define AP6256_BT_PERIPHERAL_STACK_READY_TIMEOUT_MS 30000U
#define AP6256_BT_PERIPHERAL_CONNECT_TIMEOUT_MS  60000U
#define AP6256_BT_PERIPHERAL_READ_TIMEOUT_MS     15000U

typedef struct {
    volatile uint8_t active;
    volatile uint8_t stack_ready;
    volatile uint8_t patchram_loaded;
    volatile uint8_t advertising_started;
    volatile uint8_t connected;
    volatile uint8_t read_observed;
    volatile uint8_t command_polling;
    volatile uint8_t connection_complete;
    volatile uint8_t disconnect_complete;
    volatile uint8_t connect_status;
    volatile hci_con_handle_t connection_handle;
    volatile uint32_t read_count;
    volatile uint32_t command_complete_count;
    volatile uint32_t command_status_count;
    volatile uint16_t last_command_complete_opcode;
    volatile uint16_t last_command_status_opcode;
    volatile uint8_t last_command_complete_status;
    volatile uint8_t last_command_status_value;
    volatile uint8_t poweron_failed;
    char connection_state[24];
    char peer_address[24];
    btstack_packet_callback_registration_t hci_event_registration;
} ap6256_bt_peripheral_runtime_state_t;

static ap6256_bt_peripheral_runtime_state_t s_bt_peripheral_runtime;

static const hci_transport_config_uart_t s_bt_uart_config = {
    HCI_TRANSPORT_CONFIG_UART,
    115200U,
    115200U,
    BTSTACK_UART_FLOWCONTROL_ON,
    NULL,
    BTSTACK_UART_PARITY_OFF
};

static const uint8_t s_adv_data[] = {
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, 0x06,
    0x11, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS,
    0x01, 0x00, 0x28, 0x3e, 0x4a, 0x9b, 0x3c, 0x8f, 0x6b, 0x4a, 0x5b, 0x2d, 0x01, 0x00, 0x4e, 0x7c
};

static const uint8_t s_scan_response_data[] = {
    0x12, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME,
    'K', 'a', 'r', 'i', 'o', 's', '4', '8', ' ', 'B', 'L', 'E', ' ', 'T', 'e', 's', 't'
};

static void bt_peripheral_copy_text(char *dst, size_t dst_len, const char *src)
{
    if ((dst == NULL) || (dst_len == 0U)) {
        return;
    }

    if (src == NULL) {
        dst[0] = '\0';
        return;
    }

    (void)snprintf(dst, dst_len, "%s", src);
}

static void bt_peripheral_set_detail(char *detail, size_t detail_len, const char *text)
{
    if ((detail != NULL) && (detail_len > 0U)) {
        bt_peripheral_copy_text(detail, detail_len, text);
    }
}

static void bt_peripheral_set_connection_state(const char *state)
{
    bt_peripheral_copy_text(s_bt_peripheral_runtime.connection_state,
                            sizeof(s_bt_peripheral_runtime.connection_state),
                            state);
}

static void bt_peripheral_update_runtime_state(void)
{
    hal_uart_dma_diag_t uart_diag;

    ap6256_connectivity_set_bt_runtime(s_bt_peripheral_runtime.stack_ready,
                                       (network_manager_get_owner() == NETWORK_OWNER_BLUETOOTH) ? 1U : 0U,
                                       s_bt_peripheral_runtime.patchram_loaded,
                                       0U,
                                       0U,
                                       s_bt_peripheral_runtime.connected,
                                       s_bt_peripheral_runtime.connection_state);
    ap6256_connectivity_set_bt_selection(s_bt_peripheral_runtime.peer_address,
                                         AP6256_BT_PERIPHERAL_NAME,
                                         0,
                                         AP6256_BT_PERIPHERAL_SERVICE_UUID);
    ap6256_connectivity_set_bt_read_count(s_bt_peripheral_runtime.read_count);
    hal_uart_dma_get_diag(&uart_diag);
    ap6256_connectivity_set_bt_uart_diag(uart_diag.tx_blocks,
                                         uart_diag.tx_bytes,
                                         uart_diag.rx_irq_bytes,
                                         uart_diag.rx_blocks_complete,
                                         uart_diag.rx_errors,
                                         uart_diag.rx_overruns,
                                         uart_diag.pending_len,
                                         uart_diag.pending_offset,
                                         uart_diag.ring_count,
                                         uart_diag.irq_active,
                                         uart_diag.rx_active);
}

static void bt_peripheral_print_uart_diag(const char *stage)
{
    hal_uart_dma_diag_t diag;

    hal_uart_dma_get_diag(&diag);
    ap6256_connectivity_set_bt_uart_diag(diag.tx_blocks,
                                         diag.tx_bytes,
                                         diag.rx_irq_bytes,
                                         diag.rx_blocks_complete,
                                         diag.rx_errors,
                                         diag.rx_overruns,
                                         diag.pending_len,
                                         diag.pending_offset,
                                         diag.ring_count,
                                         diag.irq_active,
                                         diag.rx_active);
    test_uart_printf("[ INFO ] bt.ble_peripheral uart %s: tx=%lu/%lu rx_irq=%lu rx_blk=%lu err=%lu ov=%lu pend=%u/%u ring=%u irq=%u active=%u\r\n",
                     (stage != NULL) ? stage : "diag",
                     (unsigned long)diag.tx_blocks,
                     (unsigned long)diag.tx_bytes,
                     (unsigned long)diag.rx_irq_bytes,
                     (unsigned long)diag.rx_blocks_complete,
                     (unsigned long)diag.rx_errors,
                     (unsigned long)diag.rx_overruns,
                     (unsigned)diag.pending_offset,
                     (unsigned)diag.pending_len,
                     (unsigned)diag.ring_count,
                     (unsigned)diag.irq_active,
                     (unsigned)diag.rx_active);
}

static void bt_peripheral_fill_summary(ap6256_bt_peripheral_runtime_summary_t *summary)
{
    if (summary == NULL) {
        return;
    }

    memset(summary, 0, sizeof(*summary));
    summary->patchram_loaded = s_bt_peripheral_runtime.patchram_loaded;
    summary->advertising_started = s_bt_peripheral_runtime.advertising_started;
    summary->connected = s_bt_peripheral_runtime.connected;
    summary->read_count = s_bt_peripheral_runtime.read_count;
    bt_peripheral_copy_text(summary->peer_address,
                            sizeof(summary->peer_address),
                            s_bt_peripheral_runtime.peer_address);
    bt_peripheral_copy_text(summary->local_name,
                            sizeof(summary->local_name),
                            AP6256_BT_PERIPHERAL_NAME);
    bt_peripheral_copy_text(summary->service_uuid,
                            sizeof(summary->service_uuid),
                            AP6256_BT_PERIPHERAL_SERVICE_UUID);
}

static void bt_peripheral_reset_session_state(void)
{
    s_bt_peripheral_runtime.stack_ready = 0U;
    s_bt_peripheral_runtime.patchram_loaded = 0U;
    s_bt_peripheral_runtime.advertising_started = 0U;
    s_bt_peripheral_runtime.connected = 0U;
    s_bt_peripheral_runtime.read_observed = 0U;
    s_bt_peripheral_runtime.connection_complete = 0U;
    s_bt_peripheral_runtime.disconnect_complete = 0U;
    s_bt_peripheral_runtime.connect_status = 0xFFU;
    s_bt_peripheral_runtime.connection_handle = HCI_CON_HANDLE_INVALID;
    s_bt_peripheral_runtime.read_count = 0U;
    s_bt_peripheral_runtime.command_complete_count = 0U;
    s_bt_peripheral_runtime.command_status_count = 0U;
    s_bt_peripheral_runtime.last_command_complete_opcode = 0U;
    s_bt_peripheral_runtime.last_command_status_opcode = 0U;
    s_bt_peripheral_runtime.last_command_complete_status = 0xFFU;
    s_bt_peripheral_runtime.last_command_status_value = 0xFFU;
    s_bt_peripheral_runtime.poweron_failed = 0U;
    s_bt_peripheral_runtime.peer_address[0] = '\0';
    bt_peripheral_set_connection_state("idle");
    ap6256_connectivity_set_bt_selection("",
                                         AP6256_BT_PERIPHERAL_NAME,
                                         0,
                                         AP6256_BT_PERIPHERAL_SERVICE_UUID);
    ap6256_connectivity_set_bt_read_count(0U);
}

static bool bt_peripheral_wait_until(volatile uint8_t *flag, uint32_t timeout_ms)
{
    uint32_t start_ms = HAL_GetTick();

    while ((HAL_GetTick() - start_ms) < timeout_ms) {
        (void)hal_uart_dma_poll();
        btstack_run_loop_embedded_execute_once();
        (void)hal_uart_dma_poll();
        bt_peripheral_update_runtime_state();
        if (*flag != 0U) {
            return true;
        }
    }

    return false;
}

static bool bt_peripheral_wait_for_hci_state(HCI_STATE target_state, uint32_t timeout_ms)
{
    uint32_t start_ms = HAL_GetTick();

    while ((HAL_GetTick() - start_ms) < timeout_ms) {
        (void)hal_uart_dma_poll();
        btstack_run_loop_embedded_execute_once();
        (void)hal_uart_dma_poll();
        bt_peripheral_update_runtime_state();
        if (hci_get_state() == target_state) {
            return true;
        }
    }

    return hci_get_state() == target_state;
}

static bool bt_peripheral_power_off_stack(uint32_t timeout_ms)
{
    if (hci_get_state() == HCI_STATE_OFF) {
        return true;
    }

    if (hci_power_control(HCI_POWER_OFF) != 0) {
        return false;
    }

    return bt_peripheral_wait_for_hci_state(HCI_STATE_OFF, timeout_ms);
}

static void bt_peripheral_hci_event_handler(uint8_t packet_type,
                                            uint16_t channel,
                                            uint8_t *packet,
                                            uint16_t size)
{
    bd_addr_t address;

    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }

    switch (hci_event_packet_get_type(packet)) {
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
            s_bt_peripheral_runtime.stack_ready = 1U;
            s_bt_peripheral_runtime.patchram_loaded = 1U;
            bt_peripheral_set_connection_state("ready");
            ap6256_connectivity_set_bt_note("BTstack BLE peripheral runtime is ready on the AP6256 controller.");
            bt_peripheral_update_runtime_state();
        }
        break;

    case BTSTACK_EVENT_POWERON_FAILED:
        s_bt_peripheral_runtime.poweron_failed = 1U;
        bt_peripheral_set_connection_state("poweron_failed");
        bt_peripheral_update_runtime_state();
        break;

    case HCI_EVENT_COMMAND_COMPLETE:
        s_bt_peripheral_runtime.command_complete_count++;
        s_bt_peripheral_runtime.last_command_complete_opcode =
            hci_event_command_complete_get_command_opcode(packet);
        s_bt_peripheral_runtime.last_command_complete_status =
            hci_event_command_complete_get_return_parameters(packet)[0];
        break;

    case HCI_EVENT_COMMAND_STATUS:
        s_bt_peripheral_runtime.command_status_count++;
        s_bt_peripheral_runtime.last_command_status_opcode =
            hci_event_command_status_get_command_opcode(packet);
        s_bt_peripheral_runtime.last_command_status_value =
            hci_event_command_status_get_status(packet);
        break;

    case HCI_EVENT_META_GAP:
        if (hci_event_gap_meta_get_subevent_code(packet) == GAP_SUBEVENT_LE_CONNECTION_COMPLETE) {
            s_bt_peripheral_runtime.connection_complete = 1U;
            s_bt_peripheral_runtime.connect_status = gap_subevent_le_connection_complete_get_status(packet);
            if (s_bt_peripheral_runtime.connect_status == ERROR_CODE_SUCCESS) {
                s_bt_peripheral_runtime.connected = 1U;
                s_bt_peripheral_runtime.connection_handle =
                    gap_subevent_le_connection_complete_get_connection_handle(packet);
                gap_subevent_le_connection_complete_get_peer_address(packet, address);
                bt_peripheral_copy_text(s_bt_peripheral_runtime.peer_address,
                                        sizeof(s_bt_peripheral_runtime.peer_address),
                                        bd_addr_to_str(address));
                gap_advertisements_enable(0);
                bt_peripheral_set_connection_state("connected");
                ap6256_connectivity_set_bt_note("Peer connected to the AP6256 BLE peripheral.");
                test_uart_printf("[ INFO ] bt.ble_peripheral stage: peer connected %s\r\n",
                                 s_bt_peripheral_runtime.peer_address);
            } else {
                s_bt_peripheral_runtime.connected = 0U;
                s_bt_peripheral_runtime.connection_handle = HCI_CON_HANDLE_INVALID;
                bt_peripheral_set_connection_state("connect_failed");
            }
            bt_peripheral_update_runtime_state();
        }
        break;

    case HCI_EVENT_DISCONNECTION_COMPLETE:
        s_bt_peripheral_runtime.disconnect_complete = 1U;
        s_bt_peripheral_runtime.connected = 0U;
        s_bt_peripheral_runtime.connection_handle = HCI_CON_HANDLE_INVALID;
        bt_peripheral_set_connection_state("disconnected");
        bt_peripheral_update_runtime_state();
        break;

    default:
        break;
    }
}

static uint16_t bt_peripheral_att_read_callback(hci_con_handle_t connection_handle,
                                                uint16_t att_handle,
                                                uint16_t offset,
                                                uint8_t *buffer,
                                                uint16_t buffer_size)
{
    UNUSED(connection_handle);

    if (att_handle != ATT_CHARACTERISTIC_7C4E0002_2D5B_4A6B_8F3C_9B4A3E280001_01_VALUE_HANDLE) {
        return 0;
    }

    if ((buffer != NULL) && (offset == 0U)) {
        s_bt_peripheral_runtime.read_count++;
        s_bt_peripheral_runtime.read_observed = 1U;
        bt_peripheral_set_connection_state("read_observed");
        ap6256_connectivity_set_bt_note("Peer read the AP6256 BLE qualification characteristic.");
        bt_peripheral_update_runtime_state();
        test_uart_printf("[ INFO ] bt.ble_peripheral stage: characteristic read count=%lu peer=%s\r\n",
                         (unsigned long)s_bt_peripheral_runtime.read_count,
                         (s_bt_peripheral_runtime.peer_address[0] != '\0')
                             ? s_bt_peripheral_runtime.peer_address
                             : "n/a");
    }

    return att_read_callback_handle_blob((const uint8_t *)AP6256_BT_PERIPHERAL_VALUE,
                                         (uint16_t)(sizeof(AP6256_BT_PERIPHERAL_VALUE) - 1U),
                                         offset,
                                         buffer,
                                         buffer_size);
}

static int bt_peripheral_att_write_callback(hci_con_handle_t connection_handle,
                                            uint16_t att_handle,
                                            uint16_t transaction_mode,
                                            uint16_t offset,
                                            uint8_t *buffer,
                                            uint16_t buffer_size)
{
    UNUSED(connection_handle);
    UNUSED(att_handle);
    UNUSED(transaction_mode);
    UNUSED(offset);
    UNUSED(buffer);
    UNUSED(buffer_size);
    return 0;
}

static void bt_peripheral_control_init(const void *transport_config)
{
    UNUSED(transport_config);
}

static int bt_peripheral_control_on(void)
{
    ap6256_status_t st;

    st = ap6256_bt_open(0U);
    if (st != AP6256_STATUS_OK) {
        return -1;
    }

    s_bt_peripheral_runtime.patchram_loaded = 0U;
    test_uart_printf("[ INFO ] bt.ble_peripheral stage: controller power on, BCM PatchRAM init pending\r\n");
    test_uart_flush_uart_rx();
    return 0;
}

static int bt_peripheral_control_off(void)
{
    ap6256_bt_close();
    return 0;
}

static int bt_peripheral_control_sleep(void)
{
    return 0;
}

static int bt_peripheral_control_wake(void)
{
    return 0;
}

static void bt_peripheral_control_register_notifications(void (*cb)(POWER_NOTIFICATION_t event))
{
    UNUSED(cb);
}

static const btstack_control_t s_bt_peripheral_control = {
    &bt_peripheral_control_init,
    &bt_peripheral_control_on,
    &bt_peripheral_control_off,
    &bt_peripheral_control_sleep,
    &bt_peripheral_control_wake,
    &bt_peripheral_control_register_notifications
};

static bool bt_peripheral_start_stack(char *detail, size_t detail_len)
{
    const ap6256_embedded_asset_t *patch_asset = ap6256_assets_bt_patchram();
    int rc;

    if ((patch_asset == NULL) || (patch_asset->data == NULL) || (patch_asset->size == 0U)) {
        bt_peripheral_set_detail(detail, detail_len, "Bluetooth PatchRAM asset is missing or empty.");
        ap6256_connectivity_set_bt_note(detail);
        return false;
    }

    bt_peripheral_reset_session_state();
    memset(&s_bt_peripheral_runtime.hci_event_registration, 0, sizeof(s_bt_peripheral_runtime.hci_event_registration));
    s_bt_peripheral_runtime.active = 1U;
    bt_peripheral_set_connection_state("starting");
    bt_peripheral_update_runtime_state();

    test_uart_printf("[ INFO ] bt.ble_peripheral stage: btstack init\r\n");
    btstack_memory_init();
    btstack_run_loop_init(btstack_run_loop_embedded_get_instance());
    hci_init(hci_transport_h4_instance(btstack_uart_block_embedded_instance()), (void *)&s_bt_uart_config);
    hci_set_chipset(btstack_chipset_bcm_instance());
    hci_set_control(&s_bt_peripheral_control);
    l2cap_init();
    att_server_init(profile_data, bt_peripheral_att_read_callback, bt_peripheral_att_write_callback);

    s_bt_peripheral_runtime.hci_event_registration.callback = &bt_peripheral_hci_event_handler;
    hci_add_event_handler(&s_bt_peripheral_runtime.hci_event_registration);

    ap6256_connectivity_set_bt_note("Starting BTstack and loading Broadcom PatchRAM for BLE peripheral qualification.");
    test_uart_printf("[ INFO ] bt.ble_peripheral stage: hci power on\r\n");
    rc = hci_power_control(HCI_POWER_ON);
    if (rc != 0) {
        bt_peripheral_set_detail(detail, detail_len, "BTstack power-on failed while opening the AP6256 BLE peripheral runtime.");
        ap6256_connectivity_set_bt_note(detail);
        hci_remove_event_handler(&s_bt_peripheral_runtime.hci_event_registration);
        att_server_deinit();
        hci_deinit();
        s_bt_peripheral_runtime.active = 0U;
        bt_peripheral_update_runtime_state();
        return false;
    }

    test_uart_printf("[ INFO ] bt.ble_peripheral stage: wait HCI working\r\n");
    if (!bt_peripheral_wait_until(&s_bt_peripheral_runtime.stack_ready, AP6256_BT_PERIPHERAL_STACK_READY_TIMEOUT_MS)) {
        bt_peripheral_print_uart_diag("hci_timeout");
        test_uart_printf("[ INFO ] bt.ble_peripheral hci trace: cc=%lu last_cc=0x%04X/s%02X cs=%lu last_cs=0x%04X/s%02X pof=%u state=%u\r\n",
                         (unsigned long)s_bt_peripheral_runtime.command_complete_count,
                         (unsigned)s_bt_peripheral_runtime.last_command_complete_opcode,
                         (unsigned)s_bt_peripheral_runtime.last_command_complete_status,
                         (unsigned long)s_bt_peripheral_runtime.command_status_count,
                         (unsigned)s_bt_peripheral_runtime.last_command_status_opcode,
                         (unsigned)s_bt_peripheral_runtime.last_command_status_value,
                         (unsigned)s_bt_peripheral_runtime.poweron_failed,
                         (unsigned)hci_get_state());
        bt_peripheral_set_detail(detail, detail_len, "Timed out waiting for BTstack HCI working state in BLE peripheral runtime.");
        ap6256_connectivity_set_bt_note(detail);
        hci_remove_event_handler(&s_bt_peripheral_runtime.hci_event_registration);
        att_server_deinit();
        (void)bt_peripheral_power_off_stack(2000U);
        hci_deinit();
        s_bt_peripheral_runtime.active = 0U;
        bt_peripheral_update_runtime_state();
        return false;
    }

    return true;
}

static bool bt_peripheral_start_advertising(char *detail, size_t detail_len)
{
    bd_addr_t null_addr;

    memset(null_addr, 0, sizeof(null_addr));
    test_uart_printf("[ INFO ] bt.ble_peripheral stage: advertising start\r\n");
    hci_le_advertisements_set_params(0x0030U, 0x0030U, 0U, 0U, null_addr, 0x07U, 0x00U);
    gap_advertisements_set_data((uint8_t)sizeof(s_adv_data), (uint8_t *)s_adv_data);
    gap_scan_response_set_data((uint8_t)sizeof(s_scan_response_data), (uint8_t *)s_scan_response_data);
    gap_advertisements_enable(1);
    s_bt_peripheral_runtime.advertising_started = 1U;
    bt_peripheral_set_connection_state("advertising");
    ap6256_connectivity_set_bt_note("Advertising AP6256 BLE qualification service; connect from a phone and read the test characteristic.");
    bt_peripheral_update_runtime_state();

    if (s_bt_peripheral_runtime.advertising_started == 0U) {
        bt_peripheral_set_detail(detail, detail_len, "Failed to start BLE advertising for AP6256 peripheral qualification.");
        ap6256_connectivity_set_bt_note(detail);
        return false;
    }

    return true;
}

static bool bt_peripheral_disconnect(uint32_t timeout_ms)
{
    if (s_bt_peripheral_runtime.connected == 0U) {
        return true;
    }

    s_bt_peripheral_runtime.disconnect_complete = 0U;
    bt_peripheral_set_connection_state("disconnecting");
    bt_peripheral_update_runtime_state();
    (void)gap_disconnect(s_bt_peripheral_runtime.connection_handle);
    return bt_peripheral_wait_until(&s_bt_peripheral_runtime.disconnect_complete, timeout_ms);
}

static bool bt_peripheral_stop_stack(void)
{
    bool powered_off = true;

    if (s_bt_peripheral_runtime.active == 0U) {
        bt_peripheral_set_connection_state("idle");
        bt_peripheral_update_runtime_state();
        return true;
    }

    if (s_bt_peripheral_runtime.stack_ready != 0U) {
        hci_remove_event_handler(&s_bt_peripheral_runtime.hci_event_registration);
        att_server_deinit();
    }

    gap_advertisements_enable(0);
    powered_off = bt_peripheral_power_off_stack(2000U);
    hci_deinit();

    s_bt_peripheral_runtime.active = 0U;
    s_bt_peripheral_runtime.stack_ready = 0U;
    s_bt_peripheral_runtime.patchram_loaded = 0U;
    s_bt_peripheral_runtime.connected = 0U;
    s_bt_peripheral_runtime.connection_handle = HCI_CON_HANDLE_INVALID;
    bt_peripheral_set_connection_state("idle");
    bt_peripheral_update_runtime_state();
    return powered_off;
}

void ap6256_bt_peripheral_runtime_suspend(void)
{
    (void)bt_peripheral_disconnect(2000U);
    (void)bt_peripheral_stop_stack();
    ap6256_connectivity_set_bt_note("BLE peripheral runtime suspended.");
}

ap6256_status_t ap6256_bt_peripheral_runtime_run_interactive(ap6256_bt_peripheral_runtime_summary_t *summary,
                                                             char *detail,
                                                             size_t detail_len)
{
    ap6256_status_t st = AP6256_STATUS_OK;
    bool cleanup_ok = true;

    if ((detail == NULL) || (detail_len == 0U)) {
        return AP6256_STATUS_BAD_PARAM;
    }

    if (!network_manager_acquire(NETWORK_OWNER_BLUETOOTH, 30000U)) {
        bt_peripheral_set_detail(detail, detail_len, "Timed out waiting for Bluetooth radio ownership.");
        return AP6256_STATUS_TIMEOUT;
    }

    s_bt_peripheral_runtime.command_polling = 1U;
    test_uart_printf("[ INFO ] bt.ble_peripheral stage: acquired bluetooth owner\r\n");

    if (!bt_peripheral_start_stack(detail, detail_len)) {
        st = AP6256_STATUS_IO_ERROR;
        goto exit;
    }

    if (!bt_peripheral_start_advertising(detail, detail_len)) {
        st = AP6256_STATUS_IO_ERROR;
        goto exit;
    }

    if (!bt_peripheral_wait_until(&s_bt_peripheral_runtime.connection_complete, AP6256_BT_PERIPHERAL_CONNECT_TIMEOUT_MS)) {
        bt_peripheral_set_detail(detail, detail_len, "Advertising started but no inbound BLE connection arrived before timeout.");
        ap6256_connectivity_set_bt_note(detail);
        bt_peripheral_set_connection_state("connect_timeout");
        bt_peripheral_update_runtime_state();
        st = AP6256_STATUS_TIMEOUT;
        goto exit;
    }

    if ((s_bt_peripheral_runtime.connect_status != ERROR_CODE_SUCCESS) ||
        (s_bt_peripheral_runtime.connected == 0U)) {
        bt_peripheral_set_detail(detail, detail_len, "Inbound BLE connection attempt failed after advertising started.");
        ap6256_connectivity_set_bt_note(detail);
        bt_peripheral_set_connection_state("connect_failed");
        bt_peripheral_update_runtime_state();
        st = AP6256_STATUS_IO_ERROR;
        goto exit;
    }

    test_uart_printf("[ INFO ] bt.ble_peripheral stage: wait for GATT read\r\n");
    if (!bt_peripheral_wait_until(&s_bt_peripheral_runtime.read_observed, AP6256_BT_PERIPHERAL_READ_TIMEOUT_MS)) {
        bt_peripheral_set_detail(detail, detail_len, "Peer connected but no GATT read was observed before timeout.");
        ap6256_connectivity_set_bt_note(detail);
        bt_peripheral_set_connection_state("read_timeout");
        bt_peripheral_update_runtime_state();
        st = AP6256_STATUS_TIMEOUT;
        goto exit;
    }

    bt_peripheral_fill_summary(summary);
    (void)snprintf(detail,
                   detail_len,
                   "BLE peripheral accepted peer %s and served %lu GATT read(s).",
                   (s_bt_peripheral_runtime.peer_address[0] != '\0')
                       ? s_bt_peripheral_runtime.peer_address
                       : "n/a",
                   (unsigned long)s_bt_peripheral_runtime.read_count);
    ap6256_connectivity_set_bt_note("BLE peripheral qualification succeeded with an inbound connection and characteristic read.");

exit:
    bt_peripheral_fill_summary(summary);

    if (!bt_peripheral_disconnect(2000U)) {
        cleanup_ok = false;
    }
    if (!bt_peripheral_stop_stack()) {
        cleanup_ok = false;
    }
    s_bt_peripheral_runtime.command_polling = 0U;
    network_manager_release(NETWORK_OWNER_BLUETOOTH);
    bt_peripheral_update_runtime_state();

    if (!cleanup_ok) {
        bt_peripheral_set_detail(detail, detail_len, "BLE peripheral qualification reached cleanup, but disconnect/teardown did not complete cleanly.");
        ap6256_connectivity_set_bt_note(detail);
        return AP6256_STATUS_IO_ERROR;
    }

    return st;
}
