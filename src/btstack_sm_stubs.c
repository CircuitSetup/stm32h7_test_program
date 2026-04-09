#include "../third_party/btstack/src/btstack_defines.h"
#include "../third_party/btstack/src/ble/sm.h"
#include "../third_party/btstack/src/gap.h"

void sm_add_event_handler(btstack_packet_callback_registration_t *callback_handler)
{
    UNUSED(callback_handler);
}

void sm_remove_event_handler(btstack_packet_callback_registration_t *callback_handler)
{
    UNUSED(callback_handler);
}

void sm_request_pairing(hci_con_handle_t con_handle)
{
    UNUSED(con_handle);
}

bool gap_reconnect_security_setup_active(hci_con_handle_t con_handle)
{
    UNUSED(con_handle);
    return false;
}
