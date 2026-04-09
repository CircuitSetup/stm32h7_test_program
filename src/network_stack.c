#include "network_stack.h"

#include "cmsis_os2.h"
#include "lwip/tcpip.h"

static volatile uint8_t s_network_stack_ready = 0U;

static void network_stack_ready_callback(void *arg)
{
    osSemaphoreId_t ready_sem = (osSemaphoreId_t)arg;

    s_network_stack_ready = 1U;
    if (ready_sem != NULL) {
        (void)osSemaphoreRelease(ready_sem);
    }
}

bool network_stack_init(void)
{
    osSemaphoreId_t ready_sem;
    static const osSemaphoreAttr_t ready_sem_attr = {
        .name = "tcpip_ready"
    };

    if (s_network_stack_ready != 0U) {
        return true;
    }

    ready_sem = osSemaphoreNew(1U, 0U, &ready_sem_attr);
    if (ready_sem == NULL) {
        return false;
    }

    tcpip_init(network_stack_ready_callback, ready_sem);
    if (osSemaphoreAcquire(ready_sem, 5000U) != osOK) {
        (void)osSemaphoreDelete(ready_sem);
        return false;
    }

    (void)osSemaphoreDelete(ready_sem);
    return true;
}

bool network_stack_is_ready(void)
{
    return (s_network_stack_ready != 0U);
}
