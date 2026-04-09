/*
 * Minimal NO_SYS port hooks for lwIP on this project.
 *
 * The macro renames keep the public lwIP names available to later code
 * while letting this header provide a tiny built-in implementation.
 */
#ifndef LWIP_ARCH_SYS_ARCH_H
#define LWIP_ARCH_SYS_ARCH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t HAL_GetTick(void);

#define sys_init     lwip_sys_init
#define sys_now      lwip_sys_now
#define sys_jiffies  lwip_sys_jiffies

static inline void lwip_sys_init(void)
{
}

static inline uint32_t lwip_sys_now(void)
{
    return HAL_GetTick();
}

static inline uint32_t lwip_sys_jiffies(void)
{
    return HAL_GetTick();
}

#ifdef __cplusplus
}
#endif

#endif /* LWIP_ARCH_SYS_ARCH_H */
