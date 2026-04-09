# STM32H743 Board Test Firmware

This directory is the PlatformIO project for a board bring-up and manufacturing test firmware targeting the on-board `STM32H743VIT6`. The firmware boots the board, exposes an interactive test console over J-Link RTT channel 0, and runs board-level diagnostics for the major fitted peripherals.

## Hardware Under Test

- MCU: `STM32H743VIT6`
- Wi-Fi / Bluetooth module: `AP6256`
- Ethernet PHY: `DP83640TVV/NOPB`
- Ethernet connector / magnetics: `HR911130A`
- Energy metering ICs: `ADE7816ACPZ` x8 (`IC2..IC9`)
- Oscillators: `Y2` (32.768 kHz), `Y3` (25 MHz), `Y4` (16.384 MHz fanout to ADE via `U9`)
- Reset supervisor: `TPS3705-33DR` (`U8`)
- microSD socket: `J1`
- USB-C connector: `J5`

## Repo Structure

- `platformio.ini`: PlatformIO build, upload, and J-Link/SWD configuration
- `src/`: main firmware sources and test modules
- `include/`: shared headers, board configuration, and public test interfaces

## Build And Flash

Run these commands from this directory (`stm32h7_test_program`):

```powershell
pio run -e h743vitx -j1
pio run -e h743vitx -t upload
```

The current PlatformIO environment is configured for:

- Framework: `stm32cube`
- Target MCU: `stm32h743vit6`
- Upload / debug tool: J-Link
- Interface: SWD

## Viewing Output With J-Link RTT

The primary live output path in the current build is SEGGER RTT channel 0. Console commands can also be entered through RTT channel 0.

Recommended tools:

- `JLinkRTTViewer.exe`
- `JLinkRTTClient.exe`
- `JLinkRTTLogger.exe` for log capture

Typical workflow:

1. Flash the firmware with J-Link over SWD.
2. If the target is left halted after programming, start it once with `g` from J-Link Commander.
3. Open RTT Viewer or RTT Client and connect to the target.
4. Wait for the startup banner, then enter commands such as `help`, `run_all`, or `summary`.

## Common Commands

- `help`
- `run_all`
- `run <group|test_name>`
- `run ade <index>`
- `run analog_fixture`
- `summary`
- `dump_board_map`
- `dump_gpio`
- `dump_registers ade <index>`
- `set verbose 0|1`
- `reboot`
- Aliases: `list`, `ade_scan`, `eth_info`, `wifi_info`

Common groups:

- `run core`
- `run power`
- `run gpio`
- `run eth`
- `run wifi`
- `run bt`
- `run usb`
- `run ade`
- `run memory`
- `run human`

## Test Coverage

Automatic tests:

- Core reset source, UID, SysTick, and clock status
- SRAM march sanity and flash CRC
- Internal ADC-based MCU rail sanity
- ADE7816 SPI scan and register sanity
- AP6256 SDIO presence probe
- AP6256 Bluetooth HCI transport sanity
- DP83640 at MDIO address 1, RMII master mode validation, DHCP lease qualification, and external ping readiness
- Safe GPIO output and input checks

Interactive tests:

- Reset path confirmation with `S1`
- Ethernet DHCP + ping qualification through the DP83640 at MDIO address 1
- microSD detect / identify
- USB-C full-speed bus activity with host cable attached

Fixture tests:

- `run analog_fixture` for low-voltage isolated metering stimulus

Sanity-only checks:

- SWD/debug pins kept reserved
- BOOT0 strap logging
- RJ45 LEDs treated as operator-observable only
- External rails without direct MCU ADC measurement inferred indirectly

## Notes

- RTT is enabled by default in this build.
- UART log mirroring is disabled by default (`BOARD_LOG_MIRROR_UART = 0`), so a serial console is not the primary operator path.
- If no RTT output appears after programming, run the target once with `g` and reconnect RTT Viewer.
- ADE reset is shared on the board reset tree, and several external rails are inferred rather than directly measured by the MCU.
