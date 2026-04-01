#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FW_NAME "karios48_board_test"
#define FW_VERSION "1.0.0"
#define FW_BOARD_REV "karios_48 v257"

#define BOARD_HAS_ETHERNET 1
#define BOARD_HAS_AP6256 1
#define BOARD_HAS_ADE7816 1
#define BOARD_ADE_COUNT 8
#define BOARD_HAS_MICROSD 1
#define BOARD_HAS_USB_C 1
#define BOARD_HAS_USER_LED 0
#define BOARD_HAS_USER_BUTTON 0
#define BOARD_ENABLE_IWDG_HOOK 0
#define BOARD_ENABLE_SEGGER_RTT 1
#define BOARD_LOG_MIRROR_UART 0
#define BOARD_AUTORUN_ON_BOOT 0
#define BOARD_ENABLE_CSV_LOG 0

#define BOARD_RTT_UP_BUFFER_SIZE 32768U
#define BOARD_RTT_DOWN_BUFFER_SIZE 1024U

extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi4;
extern ADC_HandleTypeDef hadc3;
extern CRC_HandleTypeDef hcrc;

extern volatile uint8_t g_clock_fallback_hsi;

void SystemClock_Config(void);
void Error_Handler(void);

void MX_GPIO_Init(void);
void MX_USART3_UART_Init(void);
void MX_SPI4_Init(void);
void MX_ADC3_Init(void);
void MX_CRC_Init(void);

#ifdef __cplusplus
}
#endif

#endif
