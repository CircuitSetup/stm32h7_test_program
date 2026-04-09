#ifndef __SYS_ARCH_H__
#define __SYS_ARCH_H__

#include "lwip/opt.h"

#if (NO_SYS != 0)
#error "NO_SYS needs to be set to 0 for the RTOS lwIP port"
#endif

#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SYS_MBOX_NULL ((osMessageQueueId_t)0)
#define SYS_SEM_NULL  ((osSemaphoreId_t)0)

typedef osSemaphoreId_t    sys_sem_t;
typedef osMutexId_t        sys_mutex_t;
typedef osMessageQueueId_t sys_mbox_t;
typedef osThreadId_t       sys_thread_t;
typedef uint32_t           sys_prot_t;

#ifdef __cplusplus
}
#endif

#endif /* __SYS_ARCH_H__ */
