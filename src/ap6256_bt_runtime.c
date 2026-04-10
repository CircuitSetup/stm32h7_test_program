#include "ap6256_bt_runtime.h"

#include "ap6256_assets.h"
#include "ap6256_connectivity.h"
#include "btstack_run_loop_embedded.h"
#include "btstack_chipset_bcm.h"
#include "btstack_defines.h"
#include "btstack_event.h"
#include "btstack_memory.h"
#include "btstack_uart.h"
#include "btstack_uart_block.h"
#include "btstack_util.h"
#include "gap.h"
#include "hci.h"
#include "hci_transport.h"
#include "hci_transport_h4.h"
#include "network_manager.h"
#include "test_uart.h"

#include "cmsis_os2.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define AP6256_BT_MAX_DEVICES     12U
#define AP6256_BT_MAX_SERVICES    16U

typedef struct {
    uint8_t address_type;
    bd_addr_t address;
    int8_t rssi;
    char address_text[24];
    char name[32];
} ap6256_bt_device_t;

typedef struct {
    uint16_t start_handle;
    uint16_t end_handle;
    char uuid[40];
} ap6256_bt_service_t;

typedef struct {
    volatile uint8_t active;
    volatile uint8_t stack_ready;
    volatile uint8_t patchram_loaded;
    volatile uint8_t connected;
    volatile uint8_t connection_complete;
    volatile uint8_t disconnect_complete;
    volatile uint8_t discovery_complete;
    volatile uint8_t connect_status;
    volatile uint8_t discovery_status;
    volatile hci_con_handle_t connection_handle;
    volatile uint8_t device_count;
    volatile uint8_t service_count;
    volatile uint8_t cached_valid;
    char connection_state[24];
    ap6256_bt_device_t devices[AP6256_BT_MAX_DEVICES];
    ap6256_bt_service_t services[AP6256_BT_MAX_SERVICES];
    ap6256_bt_device_t cached_device;
    ap6256_bt_service_t cached_service;
    btstack_packet_callback_registration_t hci_event_registration;
} ap6256_bt_runtime_state_t;

static ap6256_bt_runtime_state_t s_bt_runtime;

static const hci_transport_config_uart_t s_bt_uart_config = {
    HCI_TRANSPORT_CONFIG_UART,
    115200U,
    115200U,
    0,
    NULL,
    BTSTACK_UART_PARITY_OFF
};

static void bt_copy_text(char *dst, size_t dst_len, const char *src)
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

static void bt_set_detail(char *detail, size_t detail_len, const char *text)
{
    if ((detail != NULL) && (detail_len > 0U)) {
        bt_copy_text(detail, detail_len, text);
    }
}

static void bt_parse_name(const uint8_t *adv_data, uint8_t adv_len, char *name_out, size_t name_len)
{
    uint8_t offset = 0U;

    if ((name_out == NULL) || (name_len == 0U)) {
        return;
    }

    name_out[0] = '\0';
    while ((offset + 1U) < adv_len) {
        uint8_t field_len = adv_data[offset];
        uint8_t field_type;
        uint8_t copy_len;

        if ((field_len == 0U) || ((offset + 1U + field_len) > adv_len)) {
            break;
        }

        field_type = adv_data[offset + 1U];
        if ((field_type == 0x08U) || (field_type == 0x09U)) {
            copy_len = (uint8_t)(field_len - 1U);
            if (copy_len >= name_len) {
                copy_len = (uint8_t)(name_len - 1U);
            }
            memcpy(name_out, &adv_data[offset + 2U], copy_len);
            name_out[copy_len] = '\0';
            return;
        }

        offset = (uint8_t)(offset + field_len + 1U);
    }
}

static void bt_format_service_uuid(char *dst, size_t dst_len, const gatt_client_service_t *service)
{
    if ((dst == NULL) || (dst_len == 0U) || (service == NULL)) {
        return;
    }

    if (service->uuid16 != 0U) {
        (void)snprintf(dst, dst_len, "0x%04X", service->uuid16);
        return;
    }

    bt_copy_text(dst, dst_len, uuid128_to_str(service->uuid128));
}

static void bt_fill_summary(ap6256_bt_runtime_summary_t *summary,
                            const ap6256_bt_device_t *device,
                            const ap6256_bt_service_t *service)
{
    if (summary == NULL) {
        return;
    }

    memset(summary, 0, sizeof(*summary));
    summary->patchram_loaded = s_bt_runtime.patchram_loaded;
    summary->device_count = s_bt_runtime.device_count;
    summary->service_count = s_bt_runtime.service_count;
    summary->connected = s_bt_runtime.connected;

    if (device != NULL) {
        summary->selected_rssi = device->rssi;
        bt_copy_text(summary->selected_address, sizeof(summary->selected_address), device->address_text);
        bt_copy_text(summary->selected_name, sizeof(summary->selected_name), device->name);
    }

    if (service != NULL) {
        bt_copy_text(summary->selected_service_uuid, sizeof(summary->selected_service_uuid), service->uuid);
    }
}

static void bt_set_connection_state(const char *state)
{
    bt_copy_text(s_bt_runtime.connection_state, sizeof(s_bt_runtime.connection_state), state);
}

static void bt_update_runtime_state(void)
{
    ap6256_connectivity_set_bt_runtime(s_bt_runtime.stack_ready,
                                       (network_manager_get_owner() == NETWORK_OWNER_BLUETOOTH) ? 1U : 0U,
                                       s_bt_runtime.patchram_loaded,
                                       s_bt_runtime.device_count,
                                       s_bt_runtime.service_count,
                                       s_bt_runtime.connected,
                                       s_bt_runtime.connection_state);
}

static int bt_find_device(const ap6256_bt_device_t *devices,
                          uint8_t count,
                          const bd_addr_t address,
                          uint8_t address_type)
{
    uint8_t i;

    for (i = 0U; i < count; ++i) {
        if ((devices[i].address_type == address_type) &&
            (memcmp(devices[i].address, address, sizeof(bd_addr_t)) == 0)) {
            return (int)i;
        }
    }

    return -1;
}

static void bt_record_device(uint8_t address_type,
                             const bd_addr_t address,
                             int8_t rssi,
                             const char *name)
{
    int slot = bt_find_device(s_bt_runtime.devices,
                              s_bt_runtime.device_count,
                              address,
                              address_type);
    ap6256_bt_device_t *device;

    if (slot < 0) {
        if (s_bt_runtime.device_count >= AP6256_BT_MAX_DEVICES) {
            return;
        }
        slot = (int)s_bt_runtime.device_count++;
    }

    device = &s_bt_runtime.devices[(uint32_t)slot];
    device->address_type = address_type;
    memcpy(device->address, address, sizeof(bd_addr_t));
    device->rssi = rssi;
    bt_copy_text(device->address_text,
                 sizeof(device->address_text),
                 bd_addr_to_str(address));
    if ((name != NULL) && (name[0] != '\0')) {
        bt_copy_text(device->name, sizeof(device->name), name);
    } else if (device->name[0] == '\0') {
        bt_copy_text(device->name, sizeof(device->name), "(unnamed)");
    }
}

static void bt_record_service(const gatt_client_service_t *service)
{
    ap6256_bt_service_t *dst;

    if ((service == NULL) || (s_bt_runtime.service_count >= AP6256_BT_MAX_SERVICES)) {
        return;
    }

    dst = &s_bt_runtime.services[s_bt_runtime.service_count++];
    memset(dst, 0, sizeof(*dst));
    dst->start_handle = service->start_group_handle;
    dst->end_handle = service->end_group_handle;
    bt_format_service_uuid(dst->uuid, sizeof(dst->uuid), service);
}

static int bt_prompt_index(const char *prompt, uint8_t count, uint32_t timeout_ms)
{
    char line[16];
    char *endptr = NULL;
    unsigned long value;
    int line_len;

    if (count == 0U) {
        return -1;
    }

    test_uart_write_str(prompt);
    line_len = test_uart_read_line(line, sizeof(line), timeout_ms);
    if (line_len <= 0) {
        return -1;
    }

    value = strtoul(line, &endptr, 10);
    if ((endptr == line) || (*endptr != '\0') || (value == 0UL) || (value > count)) {
        return -1;
    }

    return (int)(value - 1UL);
}

static void bt_print_devices(void)
{
    uint8_t i;

    test_uart_write_str("\r\nNearby BLE devices:\r\n");
    for (i = 0U; i < s_bt_runtime.device_count; ++i) {
        test_uart_printf("  %u. %s RSSI=%d type=%u name='%s'\r\n",
                         (unsigned int)(i + 1U),
                         s_bt_runtime.devices[i].address_text,
                         (int)s_bt_runtime.devices[i].rssi,
                         s_bt_runtime.devices[i].address_type,
                         s_bt_runtime.devices[i].name);
    }
}

static void bt_print_services(void)
{
    uint8_t i;

    test_uart_write_str("Primary services:\r\n");
    for (i = 0U; i < s_bt_runtime.service_count; ++i) {
        test_uart_printf("  %u. %s [0x%04X-0x%04X]\r\n",
                         (unsigned int)(i + 1U),
                         s_bt_runtime.services[i].uuid,
                         s_bt_runtime.services[i].start_handle,
                         s_bt_runtime.services[i].end_handle);
    }
}

static void bt_runtime_hci_event_handler(uint8_t packet_type,
                                         uint16_t channel,
                                         uint8_t *packet,
                                         uint16_t size)
{
    char name[32];
    bd_addr_t address;

    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }

    switch (hci_event_packet_get_type(packet)) {
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
            s_bt_runtime.stack_ready = 1U;
            s_bt_runtime.patchram_loaded = 1U;
            bt_set_connection_state("ready");
            ap6256_connectivity_set_bt_note("BTstack is running and the AP6256 controller is ready.");
            bt_update_runtime_state();
        }
        break;

    case GAP_EVENT_ADVERTISING_REPORT:
        memset(name, 0, sizeof(name));
        gap_event_advertising_report_get_address(packet, address);
        bt_parse_name(gap_event_advertising_report_get_data(packet),
                      gap_event_advertising_report_get_data_length(packet),
                      name,
                      sizeof(name));
        bt_record_device(gap_event_advertising_report_get_address_type(packet),
                         address,
                         gap_event_advertising_report_get_rssi(packet),
                         name);
        bt_update_runtime_state();
        break;

    case HCI_EVENT_META_GAP:
        if (hci_event_gap_meta_get_subevent_code(packet) == GAP_SUBEVENT_LE_CONNECTION_COMPLETE) {
            s_bt_runtime.connection_complete = 1U;
            s_bt_runtime.connect_status = gap_subevent_le_connection_complete_get_status(packet);
            if (s_bt_runtime.connect_status == ERROR_CODE_SUCCESS) {
                s_bt_runtime.connected = 1U;
                s_bt_runtime.connection_handle = gap_subevent_le_connection_complete_get_connection_handle(packet);
                bt_set_connection_state("connected");
            } else {
                s_bt_runtime.connected = 0U;
                s_bt_runtime.connection_handle = HCI_CON_HANDLE_INVALID;
                bt_set_connection_state("connect_failed");
            }
            bt_update_runtime_state();
        }
        break;

    case HCI_EVENT_DISCONNECTION_COMPLETE:
        s_bt_runtime.disconnect_complete = 1U;
        s_bt_runtime.connected = 0U;
        s_bt_runtime.connection_handle = HCI_CON_HANDLE_INVALID;
        bt_set_connection_state("disconnected");
        bt_update_runtime_state();
        break;

    default:
        break;
    }
}

static void bt_runtime_gatt_event_handler(uint8_t packet_type,
                                          uint16_t channel,
                                          uint8_t *packet,
                                          uint16_t size)
{
    gatt_client_service_t service;

    UNUSED(packet_type);
    UNUSED(channel);
    UNUSED(size);

    switch (hci_event_packet_get_type(packet)) {
    case GATT_EVENT_SERVICE_QUERY_RESULT:
        gatt_event_service_query_result_get_service(packet, &service);
        bt_record_service(&service);
        bt_update_runtime_state();
        break;

    case GATT_EVENT_QUERY_COMPLETE:
        s_bt_runtime.discovery_complete = 1U;
        s_bt_runtime.discovery_status = gatt_event_query_complete_get_att_status(packet);
        bt_set_connection_state((s_bt_runtime.discovery_status == ATT_ERROR_SUCCESS) ? "services_ready"
                                                                                     : "service_query_failed");
        bt_update_runtime_state();
        break;

    default:
        break;
    }
}

static int bt_control_on(void)
{
    return (ap6256_bt_open(0U) == AP6256_STATUS_OK) ? 0 : -1;
}

static int bt_control_off(void)
{
    ap6256_bt_close();
    return 0;
}

static int bt_control_sleep(void)
{
    return 0;
}

static int bt_control_wake(void)
{
    return 0;
}

static void bt_control_register_notifications(void (*cb)(POWER_NOTIFICATION_t event))
{
    UNUSED(cb);
}

static const btstack_control_t s_bt_control = {
    NULL,
    &bt_control_on,
    &bt_control_off,
    &bt_control_sleep,
    &bt_control_wake,
    &bt_control_register_notifications
};

static void bt_runtime_reset_session_state(void)
{
    memset(&s_bt_runtime.devices, 0, sizeof(s_bt_runtime.devices));
    memset(&s_bt_runtime.services, 0, sizeof(s_bt_runtime.services));
    s_bt_runtime.stack_ready = 0U;
    s_bt_runtime.patchram_loaded = 0U;
    s_bt_runtime.connected = 0U;
    s_bt_runtime.connection_complete = 0U;
    s_bt_runtime.disconnect_complete = 0U;
    s_bt_runtime.discovery_complete = 0U;
    s_bt_runtime.connect_status = 0xFFU;
    s_bt_runtime.discovery_status = 0xFFU;
    s_bt_runtime.connection_handle = HCI_CON_HANDLE_INVALID;
    s_bt_runtime.device_count = 0U;
    s_bt_runtime.service_count = 0U;
    bt_set_connection_state("idle");
}

static bool bt_runtime_wait_until(volatile uint8_t *flag, uint32_t timeout_ms)
{
    uint32_t start_ms = HAL_GetTick();

    while ((HAL_GetTick() - start_ms) < timeout_ms) {
        if (*flag != 0U) {
            return true;
        }
        osDelay(10U);
    }

    return false;
}

static bool bt_runtime_start_stack(char *detail, size_t detail_len)
{
    const ap6256_embedded_asset_t *patch_asset = ap6256_assets_bt_patchram();
    int rc;

    if ((patch_asset == NULL) || (patch_asset->data == NULL) || (patch_asset->size == 0U)) {
        bt_set_detail(detail, detail_len, "Bluetooth PatchRAM asset is missing or empty.");
        ap6256_connectivity_set_bt_note(detail);
        return false;
    }

    bt_runtime_reset_session_state();
    memset(&s_bt_runtime.hci_event_registration, 0, sizeof(s_bt_runtime.hci_event_registration));
    s_bt_runtime.active = 1U;
    bt_set_connection_state("starting");
    bt_update_runtime_state();

    btstack_memory_init();
    btstack_run_loop_init(btstack_run_loop_embedded_get_instance());

    hci_init(hci_transport_h4_instance(btstack_uart_block_embedded_instance()), (void *)&s_bt_uart_config);
    hci_set_control(&s_bt_control);
    hci_set_chipset(btstack_chipset_bcm_instance());
    l2cap_init();
    gatt_client_init();

    s_bt_runtime.hci_event_registration.callback = &bt_runtime_hci_event_handler;
    hci_add_event_handler(&s_bt_runtime.hci_event_registration);

    ap6256_connectivity_set_bt_note("Starting BTstack and loading Broadcom PatchRAM.");
    rc = hci_power_control(HCI_POWER_ON);
    if (rc != 0) {
        bt_set_detail(detail, detail_len, "BTstack power-on failed while opening the AP6256 controller.");
        ap6256_connectivity_set_bt_note(detail);
        s_bt_runtime.active = 0U;
        bt_update_runtime_state();
        hci_remove_event_handler(&s_bt_runtime.hci_event_registration);
        hci_deinit();
        return false;
    }

    if (!bt_runtime_wait_until(&s_bt_runtime.stack_ready, AP6256_BT_CONNECT_TIMEOUT_MS)) {
        bt_set_detail(detail, detail_len, "Timed out waiting for BTstack to reach HCI_STATE_WORKING.");
        ap6256_connectivity_set_bt_note(detail);
        hci_remove_event_handler(&s_bt_runtime.hci_event_registration);
        hci_close();
        hci_deinit();
        s_bt_runtime.active = 0U;
        bt_update_runtime_state();
        return false;
    }

    return true;
}

static void bt_runtime_stop_stack(void)
{
    if (s_bt_runtime.active == 0U) {
        bt_set_connection_state("idle");
        bt_update_runtime_state();
        return;
    }

    if (s_bt_runtime.stack_ready != 0U) {
        hci_remove_event_handler(&s_bt_runtime.hci_event_registration);
        hci_close();
        hci_deinit();
    }

    s_bt_runtime.active = 0U;
    s_bt_runtime.stack_ready = 0U;
    s_bt_runtime.patchram_loaded = 0U;
    s_bt_runtime.connected = 0U;
    s_bt_runtime.connection_handle = HCI_CON_HANDLE_INVALID;
    bt_set_connection_state("idle");
    bt_update_runtime_state();
}

static ap6256_status_t bt_runtime_scan(char *detail, size_t detail_len)
{
    bt_runtime_reset_session_state();
    s_bt_runtime.stack_ready = 1U;
    s_bt_runtime.patchram_loaded = 1U;
    bt_set_connection_state("scanning");
    bt_update_runtime_state();

    gap_set_scan_parameters(1U, 0x0030U, 0x0030U);
    gap_start_scan();
    ap6256_connectivity_set_bt_note("Scanning for nearby BLE devices.");
    osDelay(AP6256_BT_SCAN_TIMEOUT_MS);
    gap_stop_scan();
    osDelay(100U);

    if (s_bt_runtime.device_count == 0U) {
        bt_set_detail(detail, detail_len, "No BLE advertising devices were discovered before scan timeout.");
        ap6256_connectivity_set_bt_note(detail);
        bt_set_connection_state("scan_timeout");
        bt_update_runtime_state();
        return AP6256_STATUS_TIMEOUT;
    }

    bt_set_connection_state("scan_complete");
    bt_update_runtime_state();
    return AP6256_STATUS_OK;
}

static ap6256_status_t bt_runtime_connect(const ap6256_bt_device_t *device,
                                          char *detail,
                                          size_t detail_len)
{
    if (device == NULL) {
        bt_set_detail(detail, detail_len, "No BLE device is selected.");
        return AP6256_STATUS_BAD_PARAM;
    }

    s_bt_runtime.connection_complete = 0U;
    s_bt_runtime.connect_status = 0xFFU;
    bt_set_connection_state("connecting");
    bt_update_runtime_state();
    ap6256_connectivity_set_bt_selection(device->address_text, device->name, device->rssi, NULL);
    ap6256_connectivity_set_bt_note("Connecting to selected BLE device.");

    if (gap_connect(device->address, (bd_addr_type_t)device->address_type) != ERROR_CODE_SUCCESS) {
        bt_set_detail(detail, detail_len, "BTstack rejected the BLE connect request.");
        ap6256_connectivity_set_bt_note(detail);
        bt_set_connection_state("connect_failed");
        bt_update_runtime_state();
        return AP6256_STATUS_IO_ERROR;
    }

    if (!bt_runtime_wait_until(&s_bt_runtime.connection_complete, AP6256_BT_CONNECT_TIMEOUT_MS)) {
        bt_set_detail(detail, detail_len, "Timed out waiting for BLE connection complete.");
        ap6256_connectivity_set_bt_note(detail);
        bt_set_connection_state("connect_timeout");
        bt_update_runtime_state();
        return AP6256_STATUS_TIMEOUT;
    }

    if ((s_bt_runtime.connect_status != ERROR_CODE_SUCCESS) || (s_bt_runtime.connected == 0U)) {
        bt_set_detail(detail, detail_len, "BLE connection attempt failed.");
        ap6256_connectivity_set_bt_note(detail);
        bt_set_connection_state("connect_failed");
        bt_update_runtime_state();
        return AP6256_STATUS_IO_ERROR;
    }

    bt_set_connection_state("connected");
    bt_update_runtime_state();
    return AP6256_STATUS_OK;
}

static ap6256_status_t bt_runtime_discover_services(char *detail, size_t detail_len)
{
    uint8_t status;

    s_bt_runtime.service_count = 0U;
    s_bt_runtime.discovery_complete = 0U;
    s_bt_runtime.discovery_status = 0xFFU;
    bt_set_connection_state("discovering_services");
    bt_update_runtime_state();
    ap6256_connectivity_set_bt_note("Discovering primary services on selected BLE device.");

    status = gatt_client_discover_primary_services(&bt_runtime_gatt_event_handler,
                                                   s_bt_runtime.connection_handle);
    if (status != ERROR_CODE_SUCCESS) {
        bt_set_detail(detail, detail_len, "BTstack refused to start primary-service discovery.");
        ap6256_connectivity_set_bt_note(detail);
        bt_set_connection_state("service_query_failed");
        bt_update_runtime_state();
        return AP6256_STATUS_IO_ERROR;
    }

    if (!bt_runtime_wait_until(&s_bt_runtime.discovery_complete, AP6256_BT_DISCOVERY_TIMEOUT_MS)) {
        bt_set_detail(detail, detail_len, "Timed out waiting for BLE primary-service discovery.");
        ap6256_connectivity_set_bt_note(detail);
        bt_set_connection_state("service_query_timeout");
        bt_update_runtime_state();
        return AP6256_STATUS_TIMEOUT;
    }

    if (s_bt_runtime.discovery_status != ATT_ERROR_SUCCESS) {
        bt_set_detail(detail, detail_len, "BTstack reported a GATT primary-service discovery error.");
        ap6256_connectivity_set_bt_note(detail);
        bt_set_connection_state("service_query_failed");
        bt_update_runtime_state();
        return AP6256_STATUS_PROTOCOL_ERROR;
    }

    if (s_bt_runtime.service_count == 0U) {
        bt_set_detail(detail, detail_len, "Connected BLE device exposed no primary services.");
        ap6256_connectivity_set_bt_note(detail);
        bt_set_connection_state("no_services");
        bt_update_runtime_state();
        return AP6256_STATUS_PROTOCOL_ERROR;
    }

    bt_set_connection_state("services_ready");
    bt_update_runtime_state();
    return AP6256_STATUS_OK;
}

static void bt_runtime_disconnect(uint32_t timeout_ms)
{
    if (s_bt_runtime.connected == 0U) {
        return;
    }

    s_bt_runtime.disconnect_complete = 0U;
    bt_set_connection_state("disconnecting");
    bt_update_runtime_state();
    (void)gap_disconnect(s_bt_runtime.connection_handle);
    (void)bt_runtime_wait_until(&s_bt_runtime.disconnect_complete, timeout_ms);
}

static ap6256_status_t bt_runtime_execute(uint8_t reuse_cached,
                                          ap6256_bt_runtime_summary_t *summary,
                                          char *detail,
                                          size_t detail_len)
{
    ap6256_bt_device_t *selected_device = NULL;
    ap6256_bt_service_t *selected_service = NULL;
    int selection;
    int cached_index;
    ap6256_status_t st = AP6256_STATUS_OK;

    if ((detail == NULL) || (detail_len == 0U)) {
        return AP6256_STATUS_BAD_PARAM;
    }

    if ((reuse_cached != 0U) && (s_bt_runtime.cached_valid == 0U)) {
        bt_set_detail(detail, detail_len, "No cached BLE device/service selection is available.");
        return AP6256_STATUS_BAD_PARAM;
    }

    if (!network_manager_acquire(NETWORK_OWNER_BLUETOOTH, 30000U)) {
        bt_set_detail(detail, detail_len, "Timed out waiting for Bluetooth radio ownership.");
        return AP6256_STATUS_TIMEOUT;
    }

    if (!bt_runtime_start_stack(detail, detail_len)) {
        st = AP6256_STATUS_IO_ERROR;
        goto exit;
    }

    st = bt_runtime_scan(detail, detail_len);
    if (st != AP6256_STATUS_OK) {
        goto exit;
    }

    bt_print_devices();
    if (reuse_cached != 0U) {
        cached_index = bt_find_device(s_bt_runtime.devices,
                                      s_bt_runtime.device_count,
                                      s_bt_runtime.cached_device.address,
                                      s_bt_runtime.cached_device.address_type);
        if (cached_index >= 0) {
            selected_device = &s_bt_runtime.devices[(uint32_t)cached_index];
        }
    }

    if (selected_device == NULL) {
        selection = bt_prompt_index("Select BLE device index: ",
                                    s_bt_runtime.device_count,
                                    AP6256_WIFI_PROMPT_TIMEOUT_MS);
        if (selection < 0) {
            bt_set_detail(detail, detail_len, "Invalid or timed-out BLE device selection.");
            st = AP6256_STATUS_TIMEOUT;
            goto exit;
        }
        selected_device = &s_bt_runtime.devices[(uint32_t)selection];
    }

    st = bt_runtime_connect(selected_device, detail, detail_len);
    if (st != AP6256_STATUS_OK) {
        goto exit;
    }

    st = bt_runtime_discover_services(detail, detail_len);
    if (st != AP6256_STATUS_OK) {
        goto exit;
    }

    bt_print_services();
    if ((reuse_cached != 0U) && (s_bt_runtime.cached_valid != 0U)) {
        uint8_t i;
        for (i = 0U; i < s_bt_runtime.service_count; ++i) {
            if (strcmp(s_bt_runtime.services[i].uuid, s_bt_runtime.cached_service.uuid) == 0) {
                selected_service = &s_bt_runtime.services[i];
                break;
            }
        }
    }

    if (selected_service == NULL) {
        selection = bt_prompt_index("Select primary service index to validate: ",
                                    s_bt_runtime.service_count,
                                    AP6256_WIFI_PROMPT_TIMEOUT_MS);
        if (selection < 0) {
            bt_set_detail(detail, detail_len, "Invalid or timed-out BLE service selection.");
            st = AP6256_STATUS_TIMEOUT;
            goto exit;
        }
        selected_service = &s_bt_runtime.services[(uint32_t)selection];
    }

    memcpy(&s_bt_runtime.cached_device, selected_device, sizeof(s_bt_runtime.cached_device));
    memcpy(&s_bt_runtime.cached_service, selected_service, sizeof(s_bt_runtime.cached_service));
    s_bt_runtime.cached_valid = 1U;

    ap6256_connectivity_set_bt_selection(selected_device->address_text,
                                         selected_device->name,
                                         selected_device->rssi,
                                         selected_service->uuid);
    bt_set_connection_state("qualified");
    bt_update_runtime_state();
    ap6256_connectivity_set_bt_note("BLE device connected and selected primary service discovered via BTstack.");
    bt_fill_summary(summary, selected_device, selected_service);
    (void)snprintf(detail,
                   detail_len,
                   "BLE device connected and selected primary service %s discovered.",
                   selected_service->uuid);

exit:
    bt_runtime_disconnect(2000U);
    bt_runtime_stop_stack();
    if (st != AP6256_STATUS_OK) {
        ap6256_connectivity_set_bt_note(detail);
    }
    network_manager_release(NETWORK_OWNER_BLUETOOTH);
    return st;
}

void ap6256_bt_runtime_poll(void)
{
    if (s_bt_runtime.active == 0U) {
        return;
    }

    btstack_run_loop_embedded_execute_once();
}

void ap6256_bt_runtime_suspend(void)
{
    bt_runtime_disconnect(2000U);
    bt_runtime_stop_stack();
    ap6256_connectivity_set_bt_note("BTstack runtime suspended.");
}

uint8_t ap6256_bt_runtime_has_cached_selection(void)
{
    return s_bt_runtime.cached_valid;
}

ap6256_status_t ap6256_bt_runtime_run_interactive(ap6256_bt_runtime_summary_t *summary,
                                                  char *detail,
                                                  size_t detail_len)
{
    return bt_runtime_execute(0U, summary, detail, detail_len);
}

ap6256_status_t ap6256_bt_runtime_run_cached(ap6256_bt_runtime_summary_t *summary,
                                             char *detail,
                                             size_t detail_len)
{
    return bt_runtime_execute(1U, summary, detail, detail_len);
}
