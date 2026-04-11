#ifndef CYW43_INCLUDED_CONFIGPORT_H
#define CYW43_INCLUDED_CONFIGPORT_H

#include "cmsis_os2.h"

#include <stddef.h>
#include <stdint.h>

#ifndef static_assert
#define static_assert(expr, msg) typedef int static_assert_##__LINE__[(expr) ? 1 : -1]
#endif

#define MIN(a, b)                           ((a) <= (b) ? (a) : (b))
#define CYW43_ARRAY_SIZE(a)                 (sizeof(a) / sizeof((a)[0]))

#define CYW43_USE_SPI                       (0)
#define CYW43_LWIP                          (1)
#define CYW43_NETUTILS                      (0)
#define CYW43_ENABLE_BLUETOOTH              (0)
#define CYW43_ENABLE_BLUETOOTH_OVER_UART    (0)
#define CYW43_USE_OTP_MAC                   (0)
#define CYW43_RESOURCE_VERIFY_DOWNLOAD      (0)

#define CYW43_PIN_WL_REG_ON                 (0)
#define CYW43_PIN_WL_SDIO_1                 (1)

#define CYW43_EPERM                         (1)
#define CYW43_EIO                           (5)
#define CYW43_EINVAL                        (22)
#define CYW43_ETIMEDOUT                     (110)

#define CYW43_THREAD_ENTER                  ap6256_cyw43_thread_enter()
#define CYW43_THREAD_EXIT                   ap6256_cyw43_thread_exit()
#define CYW43_THREAD_LOCK_CHECK             do { } while (0)

#define CYW43_SDPCM_SEND_COMMON_WAIT        do { ap6256_cyw43_port_wait_step(); } while (0)
#define CYW43_DO_IOCTL_WAIT                 do { ap6256_cyw43_port_wait_step(); } while (0)
#define CYW43_EVENT_POLL_HOOK               do { ap6256_cyw43_port_wait_step(); } while (0)

/*
 * Broadcom DCMD responses can legitimately take longer than the upstream
 * MCU-default 500 ms when the BCM43456 firmware is starting a scan. Match the
 * common brcmfmac-class timeout so scan start failures separate real missing
 * responses from slow accepted commands.
 */
#define CYW43_IOCTL_TIMEOUT_US              (2500000U)

#define CYW43_HAL_PIN_MODE_INPUT            (0)
#define CYW43_HAL_PIN_MODE_OUTPUT           (1)
#define CYW43_HAL_PIN_PULL_NONE             (0)
#define CYW43_HAL_MAC_WLAN0                 (0)

#define CYW43_HOST_NAME                     "karios48-wifi"

unsigned int cyw43_hal_ticks_us(void);
unsigned int cyw43_hal_ticks_ms(void);
void cyw43_delay_us(unsigned int us);
void cyw43_delay_ms(unsigned int ms);
void cyw43_hal_get_mac(int interface, uint8_t mac[6]);
void cyw43_hal_pin_config(int pin, int mode, int pull, int alt);
void cyw43_hal_pin_config_irq_falling(int pin, int enable);
int cyw43_hal_pin_read(int pin);
void cyw43_hal_pin_low(int pin);
void cyw43_hal_pin_high(int pin);
void cyw43_schedule_internal_poll_dispatch(void (*func)(void));

void ap6256_cyw43_thread_enter(void);
void ap6256_cyw43_thread_exit(void);
void ap6256_cyw43_port_wait_step(void);

#endif /* CYW43_INCLUDED_CONFIGPORT_H */
