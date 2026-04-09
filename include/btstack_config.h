#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

#define HAVE_EMBEDDED_TIME_MS

#define ENABLE_BLE
#define ENABLE_LE_CENTRAL
#define ENABLE_LOG_ERROR
#define ENABLE_LOG_INFO

#define HCI_ACL_PAYLOAD_SIZE            (512 + 4)
#define MAX_NR_GATT_CLIENTS             1
#define MAX_NR_HCI_CONNECTIONS          1
#define MAX_NR_HCI_EVENT_CALLBACKS      2
#define MAX_NR_L2CAP_CHANNELS           3
#define MAX_NR_L2CAP_SERVICES           1
#define MAX_NR_BTSTACK_TIMERS           4
#define MAX_ATT_DB_SIZE                 512

#endif /* BTSTACK_CONFIG_H */
