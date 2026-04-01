#include "board_test.h"

#include <stdio.h>
#include <string.h>

#if BOARD_HAS_USB_C
static PCD_HandleTypeDef s_hpcd_usb_fs;
static uint8_t s_usb_ready = 0U;

static volatile uint32_t s_usb_evt_reset = 0U;
static volatile uint32_t s_usb_evt_sof = 0U;
static volatile uint32_t s_usb_evt_connect = 0U;
static volatile uint32_t s_usb_evt_disconnect = 0U;
static volatile uint32_t s_usb_evt_suspend = 0U;
static volatile uint32_t s_usb_evt_resume = 0U;

static void usb_event_reset(void)
{
    s_usb_evt_reset = 0U;
    s_usb_evt_sof = 0U;
    s_usb_evt_connect = 0U;
    s_usb_evt_disconnect = 0U;
    s_usb_evt_suspend = 0U;
    s_usb_evt_resume = 0U;
}

static HAL_StatusTypeDef usb_fs_prepare_clock(void)
{
    RCC_PeriphCLKInitTypeDef periph;

    memset(&periph, 0, sizeof(periph));
    periph.PeriphClockSelection = RCC_PERIPHCLK_USB;
#if defined(RCC_USBCLKSOURCE_HSI48)
    periph.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
#else
    periph.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
#endif
    return HAL_RCCEx_PeriphCLKConfig(&periph);
}

static HAL_StatusTypeDef usb_fs_init_device(void)
{
    if (usb_fs_prepare_clock() != HAL_OK) {
        return HAL_ERROR;
    }

    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

    memset(&s_hpcd_usb_fs, 0, sizeof(s_hpcd_usb_fs));
    s_hpcd_usb_fs.Instance = USB_OTG_FS;
    s_hpcd_usb_fs.Init.dev_endpoints = 4U;
    s_hpcd_usb_fs.Init.Host_channels = 0U;
    s_hpcd_usb_fs.Init.speed = PCD_SPEED_FULL;
    s_hpcd_usb_fs.Init.phy_itface = PCD_PHY_EMBEDDED;
    s_hpcd_usb_fs.Init.Sof_enable = ENABLE;
    s_hpcd_usb_fs.Init.low_power_enable = DISABLE;
    s_hpcd_usb_fs.Init.lpm_enable = DISABLE;
    s_hpcd_usb_fs.Init.battery_charging_enable = DISABLE;
    s_hpcd_usb_fs.Init.vbus_sensing_enable = DISABLE;
    s_hpcd_usb_fs.Init.use_dedicated_ep1 = DISABLE;
    s_hpcd_usb_fs.Init.use_external_vbus = DISABLE;
    s_hpcd_usb_fs.Init.dma_enable = DISABLE;

    if (HAL_PCD_Init(&s_hpcd_usb_fs) != HAL_OK) {
        return HAL_ERROR;
    }

    HAL_NVIC_SetPriority(OTG_FS_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);

    if (HAL_PCD_Start(&s_hpcd_usb_fs) != HAL_OK) {
        return HAL_ERROR;
    }

    s_usb_ready = 1U;
    return HAL_OK;
}

static void usb_fs_stop_device(void)
{
    if (s_usb_ready != 0U) {
        (void)HAL_PCD_Stop(&s_hpcd_usb_fs);
        (void)HAL_PCD_DeInit(&s_hpcd_usb_fs);
        s_usb_ready = 0U;
    }

    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
}
#endif

void test_usb_irq_handler(void)
{
#if BOARD_HAS_USB_C
    if ((s_usb_ready != 0U) && (s_hpcd_usb_fs.Instance == USB_OTG_FS)) {
        HAL_PCD_IRQHandler(&s_hpcd_usb_fs);
    }
#endif
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
#if BOARD_HAS_USB_C
    if ((hpcd != NULL) && (hpcd->Instance == USB_OTG_FS)) {
        s_usb_evt_reset++;
    }
#else
    (void)hpcd;
#endif
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
#if BOARD_HAS_USB_C
    if ((hpcd != NULL) && (hpcd->Instance == USB_OTG_FS)) {
        s_usb_evt_sof++;
    }
#else
    (void)hpcd;
#endif
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
#if BOARD_HAS_USB_C
    if ((hpcd != NULL) && (hpcd->Instance == USB_OTG_FS)) {
        s_usb_evt_connect++;
    }
#else
    (void)hpcd;
#endif
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
#if BOARD_HAS_USB_C
    if ((hpcd != NULL) && (hpcd->Instance == USB_OTG_FS)) {
        s_usb_evt_disconnect++;
    }
#else
    (void)hpcd;
#endif
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
#if BOARD_HAS_USB_C
    if ((hpcd != NULL) && (hpcd->Instance == USB_OTG_FS)) {
        s_usb_evt_suspend++;
    }
#else
    (void)hpcd;
#endif
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
#if BOARD_HAS_USB_C
    if ((hpcd != NULL) && (hpcd->Instance == USB_OTG_FS)) {
        s_usb_evt_resume++;
    }
#else
    (void)hpcd;
#endif
}

void test_usb_fs_device(board_test_result_t *result)
{
#if BOARD_HAS_USB_C
    uint32_t start_ms;
    uint32_t timeout_ms = 4000U;
    char measured[128];

    usb_event_reset();

    if (usb_fs_init_device() != HAL_OK) {
        board_test_set_result(result,
                              "usb.fs",
                              "USB-C OTG FS",
                              TEST_STATUS_FAIL,
                              TEST_MODE_INTERACTIVE,
                              "pcd_init_failed",
                              "USB OTG FS PCD init/start failed.",
                              "Check USB clock source, OTG_FS pin mux on PA11/PA12, and hardware assembly.");
        return;
    }

    start_ms = HAL_GetTick();
    while ((HAL_GetTick() - start_ms) < timeout_ms) {
        if ((s_usb_evt_reset > 0U) || (s_usb_evt_sof >= 16U)) {
            break;
        }
        HAL_Delay(5U);
    }

    (void)snprintf(measured,
                   sizeof(measured),
                   "rst=%lu,sof=%lu,conn=%lu,disc=%lu,susp=%lu,res=%lu",
                   (unsigned long)s_usb_evt_reset,
                   (unsigned long)s_usb_evt_sof,
                   (unsigned long)s_usb_evt_connect,
                   (unsigned long)s_usb_evt_disconnect,
                   (unsigned long)s_usb_evt_suspend,
                   (unsigned long)s_usb_evt_resume);

    if ((s_usb_evt_reset > 0U) || (s_usb_evt_sof >= 16U)) {
        board_test_set_result(result,
                              "usb.fs",
                              "USB-C OTG FS",
                              TEST_STATUS_PASS,
                              TEST_MODE_INTERACTIVE,
                              measured,
                              "USB FS bus activity detected (host reset/SOF observed).",
                              "For payload-level validation, run a dedicated USB CDC/DFU class firmware.");
    } else if (s_usb_evt_connect > 0U) {
        board_test_set_result(result,
                              "usb.fs",
                              "USB-C OTG FS",
                              TEST_STATUS_WARN,
                              TEST_MODE_INTERACTIVE,
                              measured,
                              "USB connect event seen but no reset/SOF from host during timeout.",
                              "Use a data-capable USB-C cable to a host port and retry.");
    } else {
        board_test_set_result(result,
                              "usb.fs",
                              "USB-C OTG FS",
                              TEST_STATUS_REQUIRES_FIXTURE,
                              TEST_MODE_INTERACTIVE,
                              measured,
                              "No host USB bus activity observed during the test window.",
                              "Connect USB-C to a host (data cable) and rerun 'run usb'.");
    }

    usb_fs_stop_device();
#else
    board_test_set_result(result,
                          "usb.fs",
                          "USB-C OTG FS",
                          TEST_STATUS_SKIP,
                          TEST_MODE_INTERACTIVE,
                          "not_fitted",
                          "USB-C interface is not present on this board.",
                          "");
#endif
}
