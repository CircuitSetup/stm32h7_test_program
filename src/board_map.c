#include "board_map.h"

static const board_signal_map_t s_signals[] = {
    {
        "PA13", GPIOA, GPIO_PIN_13, "SWDIO", "SWDIO",
        "J4.02, U$1.TMS", TEST_MODE_SANITY,
        "Read-only sanity: keep in default SWD function.",
        "Debug probe can attach; pin is never repurposed by test firmware."
    },
    {
        "PA14", GPIOA, GPIO_PIN_14, "SWCLK", "SWCLK",
        "J4.04, U$1.TCK", TEST_MODE_SANITY,
        "Read-only sanity: keep in default SWD function.",
        "Debug probe can attach; pin is never repurposed by test firmware."
    },
    {
        "PA15", GPIOA, GPIO_PIN_15, "JTDI", "SWD/JTAG auxiliary",
        "J4.08, U$1.TDI", TEST_MODE_SANITY,
        "Read-only sanity: keep in debug-compatible state.",
        "Pin remains reserved and is not toggled during GPIO tests."
    },
    {
        "PB10", GPIOB, GPIO_PIN_10, "!MAN_RESET (via R23)", "Reset monitor input",
        "U8.!MR, S1 manual reset switch, J4.10", TEST_MODE_SANITY,
        "Sample idle level only; no forced toggle.",
        "Line idles high and reset source flags match expected reboot path."
    },
    {
        "PC7", GPIOC, GPIO_PIN_7, "WL_REG_ON", "AP6256 Wi-Fi enable",
        "U1.WL_REG_ON via R4", TEST_MODE_AUTOMATIC,
        "Drive low/high with timing guards and verify SDIO bus response changes.",
        "AP6256 SDIO presence only when enabled; disabled state remains quiet."
    },
    {
        "PD13", GPIOD, GPIO_PIN_13, "BT_REG_ON", "AP6256 BT enable",
        "U1.BT_REG_ON via R1", TEST_MODE_AUTOMATIC,
        "Drive low/high with timing guards and run UART HCI probe.",
        "HCI reset/event response when enabled; no response when disabled."
    },
    {
        "PC8", GPIOC, GPIO_PIN_8, "SDMMC1_D0", "SDMMC1 Wi-Fi data",
        "U1.SDIO_DATA_0 via R9", TEST_MODE_AUTOMATIC,
        "Configured for SDMMC1 CMD5/CMD52 presence checks.",
        "IO-card response received on SDMMC1 when WL_REG_ON asserted."
    },
    {
        "PC9", GPIOC, GPIO_PIN_9, "SDMMC1_D1", "SDMMC1 Wi-Fi data",
        "U1.SDIO_DATA_1 via R9", TEST_MODE_AUTOMATIC,
        "Configured as SDMMC1 4-bit bus.",
        "Bus stays stable without stuck-low/stuck-high condition during probe."
    },
    {
        "PC10", GPIOC, GPIO_PIN_10, "SDMMC1_D2", "SDMMC1 Wi-Fi data",
        "U1.SDIO_DATA_2 via R9", TEST_MODE_AUTOMATIC,
        "Configured as SDMMC1 4-bit bus.",
        "Bus stays stable without line errors during probe commands."
    },
    {
        "PC11", GPIOC, GPIO_PIN_11, "SDMMC1_D3", "SDMMC1 Wi-Fi data",
        "U1.SDIO_DATA_3 via R9", TEST_MODE_AUTOMATIC,
        "Configured as SDMMC1 4-bit bus.",
        "Bus stays stable without line errors during probe commands."
    },
    {
        "PC12", GPIOC, GPIO_PIN_12, "SDMMC1_CK (via R10)", "SDMMC1 Wi-Fi clock",
        "U1.SDIO_DATA_CLK", TEST_MODE_AUTOMATIC,
        "Generate low-speed SDIO clock during presence enumeration.",
        "CMD5/CMD52 sequence returns valid response frame."
    },
    {
        "PD2", GPIOD, GPIO_PIN_2, "SDMMC1_CMD", "SDMMC1 Wi-Fi command",
        "U1.SDIO_DATA_CMD via R12", TEST_MODE_AUTOMATIC,
        "Issue CMD0/CMD5/CMD52 to SDIO device.",
        "Valid command response from AP6256 SDIO function 0."
    },
    {
        "PD8", GPIOD, GPIO_PIN_8, "USART3_TX", "AP6256 BT UART host TX",
        "U1.UART_RXD (TP-BT_TX)", TEST_MODE_AUTOMATIC,
        "Transmit HCI reset/version commands.",
        "Expected HCI event bytes returned on USART3_RX."
    },
    {
        "PD9", GPIOD, GPIO_PIN_9, "USART3_RX", "AP6256 BT UART host RX",
        "U1.UART_TXD (TP-BT_RX)", TEST_MODE_AUTOMATIC,
        "Receive and parse HCI event packets.",
        "UART receives non-garbage HCI event with matching opcode status."
    },
    {
        "PD11", GPIOD, GPIO_PIN_11, "USART3_CTS", "AP6256 UART flow control",
        "U1.UART_RTS_N", TEST_MODE_SANITY,
        "Sample module RTS_N level during BT probe.",
        "Line does not block low-speed HCI sanity transactions."
    },
    {
        "PD12", GPIOD, GPIO_PIN_12, "USART3_RTS", "AP6256 UART flow control",
        "U1.UART_CTS_N", TEST_MODE_SANITY,
        "Drive host RTS active-low during BT HCI probe when flow-control is unmanaged.",
        "Line state remains consistent across BT enable transitions."
    },
    {
        "PE2", GPIOE, GPIO_PIN_2, "SPI4_SCK (via R14)", "ADE7816 SPI clock",
        "IC2..IC9 SCLK/SCL", TEST_MODE_AUTOMATIC,
        "SPI mode-3 transactions to each CS line with timeout.",
        "Per-device register readback succeeds without bus contention."
    },
    {
        "PE5", GPIOE, GPIO_PIN_5, "SPI4_MISO", "ADE7816 SPI MISO",
        "IC2..IC9 MISO/HSD", TEST_MODE_AUTOMATIC,
        "Read status/config/version/checksum registers.",
        "Responses are not all-0x00/all-0xFF and are stable per-device."
    },
    {
        "PE6", GPIOE, GPIO_PIN_6, "SPI4_MOSI (via R15)", "ADE7816 SPI MOSI",
        "IC2..IC9 MOSI/SDA", TEST_MODE_AUTOMATIC,
        "Write/readback safe control bits and clear status flags.",
        "Readback reflects writes where register is defined writable."
    },
    {
        "PE7", GPIOE, GPIO_PIN_7, "CS1", "ADE7816 chip select",
        "IC2.!SS/HDA", TEST_MODE_AUTOMATIC,
        "Assert CS1 only and probe IC2 registers.",
        "Only IC2 responds with valid register values."
    },
    {
        "PE8", GPIOE, GPIO_PIN_8, "CS2", "ADE7816 chip select",
        "IC4.!SS/HDA", TEST_MODE_AUTOMATIC,
        "Assert CS2 only and probe IC4 registers.",
        "Only IC4 responds with valid register values."
    },
    {
        "PE9", GPIOE, GPIO_PIN_9, "CS3", "ADE7816 chip select",
        "IC6.!SS/HDA", TEST_MODE_AUTOMATIC,
        "Assert CS3 only and probe IC6 registers.",
        "Only IC6 responds with valid register values."
    },
    {
        "PE10", GPIOE, GPIO_PIN_10, "CS4", "ADE7816 chip select",
        "IC9.!SS/HDA", TEST_MODE_AUTOMATIC,
        "Assert CS4 only and probe IC9 registers.",
        "Only IC9 responds with valid register values."
    },
    {
        "PE11", GPIOE, GPIO_PIN_11, "CS5", "ADE7816 chip select",
        "IC8.!SS/HDA", TEST_MODE_AUTOMATIC,
        "Assert CS5 only and probe IC8 registers.",
        "Only IC8 responds with valid register values."
    },
    {
        "PE12", GPIOE, GPIO_PIN_12, "CS6", "ADE7816 chip select",
        "IC7.!SS/HDA", TEST_MODE_AUTOMATIC,
        "Assert CS6 only and probe IC7 registers.",
        "Only IC7 responds with valid register values."
    },
    {
        "PE13", GPIOE, GPIO_PIN_13, "CS7", "ADE7816 chip select",
        "IC5.!SS/HDA", TEST_MODE_AUTOMATIC,
        "Assert CS7 only and probe IC5 registers.",
        "Only IC5 responds with valid register values."
    },
    {
        "PE14", GPIOE, GPIO_PIN_14, "CS8", "ADE7816 chip select",
        "IC3.!SS/HDA", TEST_MODE_AUTOMATIC,
        "Assert CS8 only and probe IC3 registers.",
        "Only IC3 responds with valid register values."
    },
    {
        "PA3", GPIOA, GPIO_PIN_3, "INTRPT1", "ADE7816 IRQ0 input",
        "IC2.!IRQ0", TEST_MODE_SANITY,
        "Sample IRQ idle level before/after register clear.",
        "Line remains in expected inactive state without stimulus."
    },
    {
        "PA4", GPIOA, GPIO_PIN_4, "INTRPT2", "ADE7816 IRQ0 input",
        "IC4.!IRQ0", TEST_MODE_SANITY,
        "Sample IRQ idle level before/after register clear.",
        "Line remains in expected inactive state without stimulus."
    },
    {
        "PA5", GPIOA, GPIO_PIN_5, "INTRPT3", "ADE7816 IRQ0 input",
        "IC6.!IRQ0", TEST_MODE_SANITY,
        "Sample IRQ idle level before/after register clear.",
        "Line remains in expected inactive state without stimulus."
    },
    {
        "PA6", GPIOA, GPIO_PIN_6, "INTRPT4", "ADE7816 IRQ0 input",
        "IC9.!IRQ0", TEST_MODE_SANITY,
        "Sample IRQ idle level before/after register clear.",
        "Line remains in expected inactive state without stimulus."
    },
    {
        "PB7", GPIOB, GPIO_PIN_7, "INTRPT5", "ADE7816 IRQ0 input",
        "IC8.!IRQ0", TEST_MODE_SANITY,
        "Sample IRQ idle level before/after register clear.",
        "Line remains in expected inactive state without stimulus."
    },
    {
        "PB8", GPIOB, GPIO_PIN_8, "INTRPT6", "ADE7816 IRQ0 input",
        "IC7.!IRQ0", TEST_MODE_SANITY,
        "Sample IRQ idle level before/after register clear.",
        "Line remains in expected inactive state without stimulus."
    },
    {
        "PA9", GPIOA, GPIO_PIN_9, "INTRPT7", "ADE7816 IRQ0 input",
        "IC5.!IRQ0", TEST_MODE_SANITY,
        "Sample IRQ idle level before/after register clear.",
        "Line remains in expected inactive state without stimulus."
    },
    {
        "PA10", GPIOA, GPIO_PIN_10, "INTRPT8", "ADE7816 IRQ0 input",
        "IC3.!IRQ0", TEST_MODE_SANITY,
        "Sample IRQ idle level before/after register clear.",
        "Line remains in expected inactive state without stimulus."
    },
    {
        "PC1", GPIOC, GPIO_PIN_1, "ETH_MDC", "PHY management clock",
        "U6.MDC", TEST_MODE_AUTOMATIC,
        "DP83640 management clock used for address-1 validation and network qualification.",
        "PHY state is validated by the network_ethernet module before DHCP and ping tests."
    },
    {
        "PA2", GPIOA, GPIO_PIN_2, "ETH_MDIO", "PHY management data",
        "U6.MDIO via R38/R39", TEST_MODE_AUTOMATIC,
        "DP83640 management data for address-1 validation and live status reporting.",
        "PHY configuration is expected to match RMII master mode before DHCP and ping qualification."
    },
    {
        "PA1", GPIOA, GPIO_PIN_1, "RMII_REF_CLK", "RMII 50 MHz ref clock",
        "U6.TX_CLK via R27", TEST_MODE_SANITY,
        "PHY-provided 50 MHz reference clock required for RMII master mode.",
        "The Ethernet test only proceeds when the DP83640 is valid and link-qualified."
    },
    {
        "PA7", GPIOA, GPIO_PIN_7, "RMII_CRS_DV", "RMII RX control",
        "U6.CRS/CRS_DV via R34", TEST_MODE_SANITY,
        "No forced traffic generation in base test.",
        "Pin remains configured for RMII and no bus fault occurs."
    },
    {
        "PC4", GPIOC, GPIO_PIN_4, "RMII_RXD0", "RMII RX data",
        "U6.RXD_O via R31", TEST_MODE_SANITY,
        "No forced traffic generation in base test.",
        "Pin remains configured for RMII and no bus fault occurs."
    },
    {
        "PC5", GPIOC, GPIO_PIN_5, "RMII_RXD1", "RMII RX data",
        "U6.RXD_1 via R33", TEST_MODE_SANITY,
        "No forced traffic generation in base test.",
        "Pin remains configured for RMII and no bus fault occurs."
    },
    {
        "PB11", GPIOB, GPIO_PIN_11, "RMII_TX_EN", "RMII TX control",
        "U6.TX_EN", TEST_MODE_SANITY,
        "Configured for RMII master-mode Ethernet qualification.",
        "PHY loopback or forced-link states are not acceptable for pass/fail Ethernet testing."
    },
    {
        "PB12", GPIOB, GPIO_PIN_12, "RMII_TXD0", "RMII TX data",
        "U6.TXD_O", TEST_MODE_SANITY,
        "Configured for RMII only.",
        "No unexpected pin contention while PHY is active."
    },
    {
        "PB13", GPIOB, GPIO_PIN_13, "RMII_TXD1", "RMII TX data",
        "U6.TXD_1", TEST_MODE_SANITY,
        "Configured for RMII only.",
        "No unexpected pin contention while PHY is active."
    },
    {
        "PA11", GPIOA, GPIO_PIN_11, "OTG-FS-D_N", "USB FS D-",
        "J5.D- through D1 ESD", TEST_MODE_SANITY,
        "Electrical sanity only unless USB host cable is attached.",
        "Pin config is valid and no short-to-ground symptom observed."
    },
    {
        "PA12", GPIOA, GPIO_PIN_12, "OTG-FS-D_P", "USB FS D+",
        "J5.D+ through D1 ESD", TEST_MODE_SANITY,
        "Electrical sanity only unless USB host cable is attached.",
        "Pin config is valid and no short-to-ground symptom observed."
    },
    {
        "PD6", GPIOD, GPIO_PIN_6, "SDMMC2_CK", "microSD clock",
        "J1.CLK via R11", TEST_MODE_INTERACTIVE,
        "Issue SD card identification commands when card inserted.",
        "Card responds to CMD0/CMD8/ACMD41 sequence."
    },
    {
        "PD7", GPIOD, GPIO_PIN_7, "SDMMC2_CMD", "microSD command",
        "J1.CMD via R8", TEST_MODE_INTERACTIVE,
        "Issue SD card identification commands when card inserted.",
        "Card command responses have valid framing and no timeout."
    },
    {
        "PB14", GPIOB, GPIO_PIN_14, "SDMMC2_D0", "microSD data0",
        "J1.DAT0 via R7", TEST_MODE_INTERACTIVE,
        "Read CSD/CID in 1-bit mode.",
        "Card data path returns sane non-0xFFFF words."
    },
    {
        "PB15", GPIOB, GPIO_PIN_15, "SDMMC2_D1", "microSD data1",
        "J1.DAT1 via R7", TEST_MODE_SANITY,
        "Configured for 4-bit capability; optional.",
        "No stuck line detected if card supports 4-bit mode."
    },
    {
        "PB3", GPIOB, GPIO_PIN_3, "SDMMC2_D2", "microSD data2",
        "J1.DAT2 via R7", TEST_MODE_SANITY,
        "Configured for 4-bit capability; optional.",
        "No stuck line detected if card supports 4-bit mode."
    },
    {
        "PB4", GPIOB, GPIO_PIN_4, "SDMMC2_D3", "microSD data3/CD",
        "J1.CD_DAT3 via R7", TEST_MODE_SANITY,
        "Used as DAT3; no dedicated card-detect switch wired.",
        "Card detection inferred from command response only."
    },
    {
        "PC14-OSC32_IN", GPIOC, GPIO_PIN_14, "32.768KHZ", "LSE bypass input",
        "Y2.OUT, U1.LPO", TEST_MODE_AUTOMATIC,
        "Check LSE ready flag (bypass mode) and RTC domain clocking.",
        "LSERDY asserted or fallback explicitly reported."
    },
    {
        "PH0-OSC_IN", GPIOH, GPIO_PIN_0, "25MHZ_2", "HSE bypass input",
        "Y3.OUT via R26", TEST_MODE_AUTOMATIC,
        "Check HSE ready flag (bypass mode).", 
        "HSERDY asserted or HSI fallback explicitly reported."
    }
};

static const board_gpio_policy_t s_gpio_policy[] = {
    { "PC7", GPIOC, GPIO_PIN_7, "WL_REG_ON", GPIO_CLASS_SAFE_OUTPUT, "AP6256 Wi-Fi regulator enable" },
    { "PD13", GPIOD, GPIO_PIN_13, "BT_REG_ON", GPIO_CLASS_SAFE_OUTPUT, "AP6256 Bluetooth regulator enable" },
    { "PE7", GPIOE, GPIO_PIN_7, "CS1", GPIO_CLASS_SAFE_OUTPUT, "ADE7816 chip-select" },
    { "PE8", GPIOE, GPIO_PIN_8, "CS2", GPIO_CLASS_SAFE_OUTPUT, "ADE7816 chip-select" },
    { "PE9", GPIOE, GPIO_PIN_9, "CS3", GPIO_CLASS_SAFE_OUTPUT, "ADE7816 chip-select" },
    { "PE10", GPIOE, GPIO_PIN_10, "CS4", GPIO_CLASS_SAFE_OUTPUT, "ADE7816 chip-select" },
    { "PE11", GPIOE, GPIO_PIN_11, "CS5", GPIO_CLASS_SAFE_OUTPUT, "ADE7816 chip-select" },
    { "PE12", GPIOE, GPIO_PIN_12, "CS6", GPIO_CLASS_SAFE_OUTPUT, "ADE7816 chip-select" },
    { "PE13", GPIOE, GPIO_PIN_13, "CS7", GPIO_CLASS_SAFE_OUTPUT, "ADE7816 chip-select" },
    { "PE14", GPIOE, GPIO_PIN_14, "CS8", GPIO_CLASS_SAFE_OUTPUT, "ADE7816 chip-select" },

    { "PA3", GPIOA, GPIO_PIN_3, "INTRPT1", GPIO_CLASS_SAFE_INPUT, "ADE IRQ0" },
    { "PA4", GPIOA, GPIO_PIN_4, "INTRPT2", GPIO_CLASS_SAFE_INPUT, "ADE IRQ0" },
    { "PA5", GPIOA, GPIO_PIN_5, "INTRPT3", GPIO_CLASS_SAFE_INPUT, "ADE IRQ0" },
    { "PA6", GPIOA, GPIO_PIN_6, "INTRPT4", GPIO_CLASS_SAFE_INPUT, "ADE IRQ0" },
    { "PB7", GPIOB, GPIO_PIN_7, "INTRPT5", GPIO_CLASS_SAFE_INPUT, "ADE IRQ0" },
    { "PB8", GPIOB, GPIO_PIN_8, "INTRPT6", GPIO_CLASS_SAFE_INPUT, "ADE IRQ0" },
    { "PA9", GPIOA, GPIO_PIN_9, "INTRPT7", GPIO_CLASS_SAFE_INPUT, "ADE IRQ0" },
    { "PA10", GPIOA, GPIO_PIN_10, "INTRPT8", GPIO_CLASS_SAFE_INPUT, "ADE IRQ0" },
    { "PB10", GPIOB, GPIO_PIN_10, "!MAN_RESET", GPIO_CLASS_SAFE_INPUT, "Manual reset monitor only" },

    { "PE2", GPIOE, GPIO_PIN_2, "SPI4_SCK", GPIO_CLASS_BIDIRECTIONAL_BUS, "ADE SPI bus" },
    { "PE5", GPIOE, GPIO_PIN_5, "SPI4_MISO", GPIO_CLASS_BIDIRECTIONAL_BUS, "ADE SPI bus" },
    { "PE6", GPIOE, GPIO_PIN_6, "SPI4_MOSI", GPIO_CLASS_BIDIRECTIONAL_BUS, "ADE SPI bus" },

    { "PA1", GPIOA, GPIO_PIN_1, "RMII_REF_CLK", GPIO_CLASS_DO_NOT_TOGGLE, "PHY-provided 50 MHz reference clock for RMII master mode" },
    { "PA2", GPIOA, GPIO_PIN_2, "ETH_MDIO", GPIO_CLASS_BIDIRECTIONAL_BUS, "DP83640 address-1 management data bus" },
    { "PC1", GPIOC, GPIO_PIN_1, "ETH_MDC", GPIO_CLASS_BIDIRECTIONAL_BUS, "DP83640 management clock for DHCP/ping qualification" },
    { "PA7", GPIOA, GPIO_PIN_7, "RMII_CRS_DV", GPIO_CLASS_DO_NOT_TOGGLE, "RMII receive control" },
    { "PC4", GPIOC, GPIO_PIN_4, "RMII_RXD0", GPIO_CLASS_DO_NOT_TOGGLE, "RMII receive data" },
    { "PC5", GPIOC, GPIO_PIN_5, "RMII_RXD1", GPIO_CLASS_DO_NOT_TOGGLE, "RMII receive data" },
    { "PB11", GPIOB, GPIO_PIN_11, "RMII_TX_EN", GPIO_CLASS_DO_NOT_TOGGLE, "RMII transmit control used during DHCP and ping qualification" },
    { "PB12", GPIOB, GPIO_PIN_12, "RMII_TXD0", GPIO_CLASS_DO_NOT_TOGGLE, "RMII transmit data" },
    { "PB13", GPIOB, GPIO_PIN_13, "RMII_TXD1", GPIO_CLASS_DO_NOT_TOGGLE, "RMII transmit data" },

    { "PD8", GPIOD, GPIO_PIN_8, "USART3_TX", GPIO_CLASS_BIDIRECTIONAL_BUS, "Console and BT HCI host TX" },
    { "PD9", GPIOD, GPIO_PIN_9, "USART3_RX", GPIO_CLASS_BIDIRECTIONAL_BUS, "Console and BT HCI host RX" },
    { "PD11", GPIOD, GPIO_PIN_11, "USART3_CTS", GPIO_CLASS_BIDIRECTIONAL_BUS, "BT flow-control" },
    { "PD12", GPIOD, GPIO_PIN_12, "USART3_RTS", GPIO_CLASS_BIDIRECTIONAL_BUS, "BT flow-control" },

    { "PC12", GPIOC, GPIO_PIN_12, "SDMMC1_CK", GPIO_CLASS_DO_NOT_TOGGLE, "Wi-Fi SDIO clock" },
    { "PD2", GPIOD, GPIO_PIN_2, "SDMMC1_CMD", GPIO_CLASS_DO_NOT_TOGGLE, "Wi-Fi SDIO command" },
    { "PC8", GPIOC, GPIO_PIN_8, "SDMMC1_D0", GPIO_CLASS_DO_NOT_TOGGLE, "Wi-Fi SDIO data" },
    { "PC9", GPIOC, GPIO_PIN_9, "SDMMC1_D1", GPIO_CLASS_DO_NOT_TOGGLE, "Wi-Fi SDIO data" },
    { "PC10", GPIOC, GPIO_PIN_10, "SDMMC1_D2", GPIO_CLASS_DO_NOT_TOGGLE, "Wi-Fi SDIO data" },
    { "PC11", GPIOC, GPIO_PIN_11, "SDMMC1_D3", GPIO_CLASS_DO_NOT_TOGGLE, "Wi-Fi SDIO data" },

    { "PD6", GPIOD, GPIO_PIN_6, "SDMMC2_CK", GPIO_CLASS_DO_NOT_TOGGLE, "microSD clock" },
    { "PD7", GPIOD, GPIO_PIN_7, "SDMMC2_CMD", GPIO_CLASS_DO_NOT_TOGGLE, "microSD command" },
    { "PB14", GPIOB, GPIO_PIN_14, "SDMMC2_D0", GPIO_CLASS_DO_NOT_TOGGLE, "microSD data" },
    { "PB15", GPIOB, GPIO_PIN_15, "SDMMC2_D1", GPIO_CLASS_DO_NOT_TOGGLE, "microSD data" },
    { "PB3", GPIOB, GPIO_PIN_3, "SDMMC2_D2", GPIO_CLASS_DO_NOT_TOGGLE, "microSD data" },
    { "PB4", GPIOB, GPIO_PIN_4, "SDMMC2_D3", GPIO_CLASS_DO_NOT_TOGGLE, "microSD data" },

    { "PA11", GPIOA, GPIO_PIN_11, "OTG-FS-D_N", GPIO_CLASS_DO_NOT_TOGGLE, "USB FS data line" },
    { "PA12", GPIOA, GPIO_PIN_12, "OTG-FS-D_P", GPIO_CLASS_DO_NOT_TOGGLE, "USB FS data line" },

    { "PA13", GPIOA, GPIO_PIN_13, "SWDIO", GPIO_CLASS_BOOT_OR_DEBUG_RESERVED, "Debug reserved" },
    { "PA14", GPIOA, GPIO_PIN_14, "SWCLK", GPIO_CLASS_BOOT_OR_DEBUG_RESERVED, "Debug reserved" },
    { "PA15", GPIOA, GPIO_PIN_15, "JTDI", GPIO_CLASS_BOOT_OR_DEBUG_RESERVED, "Debug reserved" },

    { "PC14", GPIOC, GPIO_PIN_14, "32.768KHZ", GPIO_CLASS_ANALOG_ONLY, "Clock input (LSE bypass)" },
    { "PH0", GPIOH, GPIO_PIN_0, "25MHZ_2", GPIO_CLASS_ANALOG_ONLY, "Clock input (HSE bypass)" }
};

static const board_ade_device_map_t s_ade_devices[] = {
    { 1, "IC2", GPIOE, GPIO_PIN_7,  "CS1", GPIOA, GPIO_PIN_3,  "INTRPT1", "ADE_CLK1", "V1_P", "V1_N", "T1.V1P", "J7" },
    { 2, "IC4", GPIOE, GPIO_PIN_8,  "CS2", GPIOA, GPIO_PIN_4,  "INTRPT2", "ADE_CLK2", "V2_P", "V2_N", "T1.V1P", "J9" },
    { 3, "IC6", GPIOE, GPIO_PIN_9,  "CS3", GPIOA, GPIO_PIN_5,  "INTRPT3", "ADE_CLK3", "V3_P", "V3_N", "T1.V1P", "J11" },
    { 4, "IC9", GPIOE, GPIO_PIN_10, "CS4", GPIOA, GPIO_PIN_6,  "INTRPT4", "ADE_CLK4", "V4_P", "V4_N", "T2.V2P", "J13" },
    { 5, "IC8", GPIOE, GPIO_PIN_11, "CS5", GPIOB, GPIO_PIN_7,  "INTRPT5", "ADE_CLK5", "V5_P", "V5_N", "T2.V2P", "J14" },
    { 6, "IC7", GPIOE, GPIO_PIN_12, "CS6", GPIOB, GPIO_PIN_8,  "INTRPT6", "ADE_CLK6", "V6_P", "V6_N", "T2.V2P", "J12" },
    { 7, "IC5", GPIOE, GPIO_PIN_13, "CS7", GPIOA, GPIO_PIN_9,  "INTRPT7", "ADE_CLK7", "V7_P", "V7_N", "T3.V3P", "J10" },
    { 8, "IC3", GPIOE, GPIO_PIN_14, "CS8", GPIOA, GPIO_PIN_10, "INTRPT8", "ADE_CLK8", "V8_P", "V8_N", "T3.V3P", "J8" }
};

static const board_current_channel_map_t s_current_channels[] = {
    { 1, 1, 'A', "IP1", "IN1", "J7", "12" },
    { 2, 1, 'B', "IP2", "IN2", "J7", "10" },
    { 3, 1, 'C', "IP3", "IN3", "J7", "8" },
    { 4, 1, 'D', "IP4", "IN4-6", "J7", "6" },
    { 5, 1, 'E', "IP5", "IN4-6", "J7", "4" },
    { 6, 1, 'F', "IP6", "IN4-6", "J7", "2" },

    { 7, 2, 'A', "IP7", "IN7", "J9", "12" },
    { 8, 2, 'B', "IP8", "IN8", "J9", "10" },
    { 9, 2, 'C', "IP9", "IN9", "J9", "8" },
    { 10, 2, 'D', "IP10", "IN10-12", "J9", "6" },
    { 11, 2, 'E', "IP11", "IN10-12", "J9", "4" },
    { 12, 2, 'F', "IP12", "IN10-12", "J9", "2" },

    { 13, 3, 'A', "IP13", "IN13", "J11", "12" },
    { 14, 3, 'B', "IP14", "IN14", "J11", "10" },
    { 15, 3, 'C', "IP15", "IN15", "J11", "8" },
    { 16, 3, 'D', "IP16", "IN16-18", "J11", "6" },
    { 17, 3, 'E', "IP17", "IN16-18", "J11", "4" },
    { 18, 3, 'F', "IP18", "IN16-18", "J11", "2" },

    { 19, 4, 'A', "IP19", "IN19", "J13", "12" },
    { 20, 4, 'B', "IP20", "IN20", "J13", "10" },
    { 21, 4, 'C', "IP21", "IN21", "J13", "8" },
    { 22, 4, 'D', "IP22", "IN22-24", "J13", "6" },
    { 23, 4, 'E', "IP23", "IN22-24", "J13", "4" },
    { 24, 4, 'F', "IP24", "IN22-24", "J13", "2" },

    { 25, 5, 'A', "IP25", "IN25", "J14", "12" },
    { 26, 5, 'B', "IP26", "IN26", "J14", "10" },
    { 27, 5, 'C', "IP27", "IN27", "J14", "8" },
    { 28, 5, 'D', "IP28", "IN28-30", "J14", "6" },
    { 29, 5, 'E', "IP29", "IN28-30", "J14", "4" },
    { 30, 5, 'F', "IP30", "IN28-30", "J14", "2" },

    { 31, 6, 'A', "IP31", "IN31", "J12", "12" },
    { 32, 6, 'B', "IP32", "IN32", "J12", "10" },
    { 33, 6, 'C', "IP33", "IN33", "J12", "8" },
    { 34, 6, 'D', "IP34", "IN34-36", "J12", "6" },
    { 35, 6, 'E', "IP35", "IN34-36", "J12", "4" },
    { 36, 6, 'F', "IP36", "IN34-36", "J12", "2" },

    { 37, 7, 'A', "IP37", "IN37", "J10", "12" },
    { 38, 7, 'B', "IP38", "IN38", "J10", "10" },
    { 39, 7, 'C', "IP39", "IN39", "J10", "8" },
    { 40, 7, 'D', "IP40", "IN40-42", "J10", "6" },
    { 41, 7, 'E', "IP41", "IN40-42", "J10", "4" },
    { 42, 7, 'F', "IP42", "IN40-42", "J10", "2" },

    { 43, 8, 'A', "IP43", "IN43", "J8", "12" },
    { 44, 8, 'B', "IP44", "IN44", "J8", "10" },
    { 45, 8, 'C', "IP45", "IN45", "J8", "8" },
    { 46, 8, 'D', "IP46", "IN46-48", "J8", "6" },
    { 47, 8, 'E', "IP47", "IN46-48", "J8", "4" },
    { 48, 8, 'F', "IP48", "IN46-48", "J8", "2" }
};

static const board_rail_observability_t s_rails[] = {
    { "3V3", "indirect", "Inferred from MCU run state + U8 supervisor + downstream bus enumeration." },
    { "VDD_MCU", "direct_internal", "Estimated via ADC VREFINT conversion (VDDA estimation)." },
    { "VDDA", "direct_internal", "Estimated via ADC VREFINT conversion; reports mV." },
    { "VBAT", "sanity_only", "Tied to VDD_MCU through R17; no independent ADC sense path." },
    { "3V3_WIFI", "indirect", "Inferred by AP6256 SDIO/UART response when WL_REG_ON/BT_REG_ON asserted." },
    { "VDD_ETH", "indirect", "Inferred by DP83640 address-1 access, RMII master-mode validation, and DHCP/ping qualification." },
    { "3V3_ADE", "indirect", "Inferred by ADE7816 SPI response across all eight devices." },
    { "3V3_LDO", "not_observable", "No MCU ADC divider/sense path." },
    { "3V3_HLK", "not_observable", "No MCU ADC divider/sense path." },
    { "USB_5V/VBUS", "not_observable", "No MCU VBUS sense pin routed in this revision." }
};

const board_signal_map_t *board_map_get_signals(size_t *count)
{
    if (count != NULL) {
        *count = sizeof(s_signals) / sizeof(s_signals[0]);
    }
    return s_signals;
}

const board_gpio_policy_t *board_map_get_gpio_policy(size_t *count)
{
    if (count != NULL) {
        *count = sizeof(s_gpio_policy) / sizeof(s_gpio_policy[0]);
    }
    return s_gpio_policy;
}

const board_ade_device_map_t *board_map_get_ade_devices(size_t *count)
{
    if (count != NULL) {
        *count = sizeof(s_ade_devices) / sizeof(s_ade_devices[0]);
    }
    return s_ade_devices;
}

const board_current_channel_map_t *board_map_get_current_channels(size_t *count)
{
    if (count != NULL) {
        *count = sizeof(s_current_channels) / sizeof(s_current_channels[0]);
    }
    return s_current_channels;
}

const board_rail_observability_t *board_map_get_rails(size_t *count)
{
    if (count != NULL) {
        *count = sizeof(s_rails) / sizeof(s_rails[0]);
    }
    return s_rails;
}

const board_ade_device_map_t *board_map_find_ade_device(uint8_t index)
{
    size_t i;

    for (i = 0U; i < (sizeof(s_ade_devices) / sizeof(s_ade_devices[0])); ++i) {
        if (s_ade_devices[i].index == index) {
            return &s_ade_devices[i];
        }
    }

    return NULL;
}

const char *board_map_status_to_string(test_status_t status)
{
    switch (status) {
    case TEST_STATUS_PASS:
        return "PASS";
    case TEST_STATUS_FAIL:
        return "FAIL";
    case TEST_STATUS_WARN:
        return "WARN";
    case TEST_STATUS_SKIP:
        return "SKIP";
    case TEST_STATUS_REQUIRES_FIXTURE:
        return "REQUIRES_FIXTURE";
    default:
        return "UNKNOWN";
    }
}

const char *board_map_mode_to_string(test_mode_t mode)
{
    switch (mode) {
    case TEST_MODE_AUTOMATIC:
        return "automatic";
    case TEST_MODE_INTERACTIVE:
        return "interactive";
    case TEST_MODE_FIXTURE:
        return "fixture";
    case TEST_MODE_SANITY:
        return "sanity";
    default:
        return "unknown";
    }
}

const char *board_map_gpio_class_to_string(gpio_class_t gpio_class)
{
    switch (gpio_class) {
    case GPIO_CLASS_SAFE_OUTPUT:
        return "safe_output";
    case GPIO_CLASS_SAFE_INPUT:
        return "safe_input";
    case GPIO_CLASS_BIDIRECTIONAL_BUS:
        return "bidirectional_bus";
    case GPIO_CLASS_DO_NOT_TOGGLE:
        return "do_not_toggle";
    case GPIO_CLASS_BOOT_OR_DEBUG_RESERVED:
        return "boot_debug_reserved";
    case GPIO_CLASS_ANALOG_ONLY:
        return "analog_only";
    case GPIO_CLASS_DANGEROUS:
        return "dangerous";
    default:
        return "unknown";
    }
}
