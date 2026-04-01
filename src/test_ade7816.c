#include "board_test.h"

#include "test_ade7816.h"
#include "test_uart.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ADE_SPI_TIMEOUT_MS 100U

#define ADE_REG_RUN       0xE228U
#define ADE_REG_STATUS0   0xE502U
#define ADE_REG_STATUS1   0xE503U
#define ADE_REG_MASK0     0xE50AU
#define ADE_REG_MASK1     0xE50BU
#define ADE_REG_CHECKSUM  0xE51FU
#define ADE_REG_PERIOD    0xE607U
#define ADE_REG_CONFIG    0xE618U
#define ADE_REG_MMODE     0xE700U
#define ADE_REG_ACCMODE   0xE701U
#define ADE_REG_LCYCMODE  0xE702U
#define ADE_REG_HSDC_CFG  0xE706U
#define ADE_REG_VERSION   0xE707U
#define ADE_REG_VRMS      0x43C0U
#define ADE_REG_IARMS     0x43C1U
#define ADE_REG_IBRMS     0x43C2U
#define ADE_REG_ICRMS     0x43C3U
#define ADE_REG_IDRMS     0x43C4U
#define ADE_REG_IERMS     0x43C5U
#define ADE_REG_IFRMS     0x43C6U

static void ade_select(const board_ade_device_map_t *dev, bool active)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, active ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static bool ade_spi_tx(const uint8_t *data, uint16_t len)
{
    return (HAL_SPI_Transmit(&hspi4, (uint8_t *)data, len, ADE_SPI_TIMEOUT_MS) == HAL_OK);
}

static bool ade_spi_rx(uint8_t *data, uint16_t len)
{
    return (HAL_SPI_Receive(&hspi4, data, len, ADE_SPI_TIMEOUT_MS) == HAL_OK);
}

bool ade7816_init_bus(void)
{
    const board_ade_device_map_t *devs;
    size_t count = 0U;
    size_t i;

    devs = board_map_get_ade_devices(&count);
    for (i = 0U; i < count; ++i) {
        HAL_GPIO_WritePin(devs[i].cs_port, devs[i].cs_pin, GPIO_PIN_SET);
    }

    return true;
}

bool ade7816_read_reg(uint8_t index, uint16_t reg, uint8_t size_bytes, uint32_t *value)
{
    const board_ade_device_map_t *dev = board_map_find_ade_device(index);
    uint8_t header[3];
    uint8_t data[4] = {0};
    uint8_t i;

    if ((dev == NULL) || (value == NULL)) {
        return false;
    }
    if ((size_bytes == 0U) || (size_bytes > 4U)) {
        return false;
    }

    header[0] = 0x01U; /* SPI read */
    header[1] = (uint8_t)((reg >> 8U) & 0xFFU);
    header[2] = (uint8_t)(reg & 0xFFU);

    ade_select(dev, true);
    if (!ade_spi_tx(header, sizeof(header))) {
        ade_select(dev, false);
        return false;
    }

    if (!ade_spi_rx(data, size_bytes)) {
        ade_select(dev, false);
        return false;
    }
    ade_select(dev, false);

    *value = 0U;
    for (i = 0U; i < size_bytes; ++i) {
        *value = (*value << 8U) | data[i];
    }

    return true;
}

bool ade7816_write_reg(uint8_t index, uint16_t reg, uint8_t size_bytes, uint32_t value)
{
    const board_ade_device_map_t *dev = board_map_find_ade_device(index);
    uint8_t packet[7];
    uint8_t i;

    if (dev == NULL) {
        return false;
    }
    if ((size_bytes == 0U) || (size_bytes > 4U)) {
        return false;
    }

    packet[0] = 0x00U; /* SPI write */
    packet[1] = (uint8_t)((reg >> 8U) & 0xFFU);
    packet[2] = (uint8_t)(reg & 0xFFU);

    for (i = 0U; i < size_bytes; ++i) {
        uint8_t shift = (uint8_t)((size_bytes - 1U - i) * 8U);
        packet[3U + i] = (uint8_t)((value >> shift) & 0xFFU);
    }

    ade_select(dev, true);
    if (!ade_spi_tx(packet, (uint16_t)(3U + size_bytes))) {
        ade_select(dev, false);
        return false;
    }
    ade_select(dev, false);

    return true;
}

bool ade7816_soft_reset(uint8_t index)
{
    uint32_t cfg = 0U;

    if (!ade7816_read_reg(index, ADE_REG_CONFIG, 1U, &cfg)) {
        return false;
    }

    cfg |= 0x80U;

    if (!ade7816_write_reg(index, ADE_REG_CONFIG, 1U, cfg)) {
        return false;
    }

    HAL_Delay(20U);
    return true;
}

void test_ade_single_device(uint8_t index, board_test_result_t *result)
{
    uint32_t version = 0U;
    uint32_t status0 = 0U;
    uint32_t status1 = 0U;
    uint32_t checksum = 0U;
    uint32_t run_before = 0U;
    uint32_t run_after = 0U;
    uint32_t mmode = 0U;
    char measured[160];

    (void)ade7816_init_bus();

    if (!ade7816_read_reg(index, ADE_REG_VERSION, 1U, &version) ||
        !ade7816_read_reg(index, ADE_REG_STATUS0, 4U, &status0) ||
        !ade7816_read_reg(index, ADE_REG_STATUS1, 4U, &status1) ||
        !ade7816_read_reg(index, ADE_REG_CHECKSUM, 4U, &checksum) ||
        !ade7816_read_reg(index, ADE_REG_RUN, 2U, &run_before) ||
        !ade7816_read_reg(index, ADE_REG_MMODE, 1U, &mmode)) {
        board_test_set_result(result,
                              "ade.single",
                              "ADE7816",
                              TEST_STATUS_FAIL,
                              TEST_MODE_AUTOMATIC,
                              "register read timeout",
                              "Could not read one or more ADE7816 core registers.",
                              "Check SPI4 mode/pins, CS routing, ADE clock, and ADE rails.");
        return;
    }

    if (!ade7816_write_reg(index, ADE_REG_RUN, 2U, 0x0001U) ||
        !ade7816_read_reg(index, ADE_REG_RUN, 2U, &run_after)) {
        board_test_set_result(result,
                              "ade.single",
                              "ADE7816",
                              TEST_STATUS_WARN,
                              TEST_MODE_AUTOMATIC,
                              "RUN write/readback issue",
                              "Core register access works, but RUN write/readback did not verify.",
                              "Review ADE startup and RUN control handling.");
        return;
    }

    (void)snprintf(measured,
                   sizeof(measured),
                   "idx=%u,ver=0x%02lX,chk=0x%08lX,run=0x%04lX->0x%04lX,status0=0x%08lX,status1=0x%08lX",
                   index,
                   (unsigned long)version,
                   (unsigned long)checksum,
                   (unsigned long)run_before,
                   (unsigned long)run_after,
                   (unsigned long)status0,
                   (unsigned long)status1);

    /* VERSION (0xE707) is a die revision code; 0x00 can be valid on some lots/revisions. */
    if ((version == 0xFFU) || (checksum == 0x00000000U) || (checksum == 0xFFFFFFFFU)) {
        board_test_set_result(result,
                              "ade.single",
                              "ADE7816",
                              TEST_STATUS_WARN,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "Device responds but one or more identity/integrity fields look suspicious.",
                              "Inspect ADE clock distribution (Y4/U9/R103..R110) and supply rails.");
        return;
    }

    board_test_set_result(result,
                          "ade.single",
                          "ADE7816",
                          TEST_STATUS_PASS,
                          TEST_MODE_AUTOMATIC,
                          measured,
                          "Per-device communication and core register sanity passed.",
                          "");
}

void test_ade_scan_all(board_test_result_t *result)
{
    size_t count = 0U;
    const board_ade_device_map_t *devs = board_map_get_ade_devices(&count);
    size_t i;
    uint8_t verbose = board_test_get_verbose();
    uint32_t pass = 0U;
    uint32_t fail = 0U;
    uint32_t warn = 0U;
    char measured[80];

    if (verbose != 0U) {
        test_uart_printf("[ INFO ] ade.scan: probing %lu device(s)...\r\n", (unsigned long)count);
    }

    for (i = 0U; i < count; ++i) {
        board_test_result_t one;

        if (verbose != 0U) {
            test_uart_printf("[ INFO ] ade.scan: idx=%u ref=%s\r\n",
                             devs[i].index,
                             devs[i].refdes);
        }

        test_ade_single_device(devs[i].index, &one);

        if (one.status == TEST_STATUS_PASS) {
            pass++;
        } else if (one.status == TEST_STATUS_FAIL) {
            fail++;
        } else {
            warn++;
        }

        if (verbose != 0U) {
            test_uart_printf("[ INFO ] ade.scan: idx=%u status=%s measured=%s\r\n",
                             devs[i].index,
                             board_map_status_to_string(one.status),
                             one.measured_value_text);
        }
    }

    (void)snprintf(measured,
                   sizeof(measured),
                   "spi_probe_devices=%lu,pass=%lu,warn=%lu,fail=%lu",
                   (unsigned long)count,
                   (unsigned long)pass,
                   (unsigned long)warn,
                   (unsigned long)fail);

    if (fail > 0U) {
        board_test_set_result(result,
                              "ade.scan",
                              "ADE7816 x8",
                              TEST_STATUS_FAIL,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "One or more ADE devices failed communication sanity.",
                              "Inspect failing CS/IRQ nets and SPI4 shared bus integrity.");
    } else if (warn > 0U) {
        board_test_set_result(result,
                              "ade.scan",
                              "ADE7816 x8",
                              TEST_STATUS_WARN,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "All devices responded, but one or more had warning-level register anomalies.",
                              "Review per-device snapshots via 'dump_registers ade <index>'.");
    } else {
        board_test_set_result(result,
                              "ade.scan",
                              "ADE7816 x8",
                              TEST_STATUS_PASS,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "All ADE7816 devices passed communication sanity.",
                              "");
    }
}

void test_ade_irq_lines(board_test_result_t *result)
{
    const board_ade_device_map_t *devs;
    size_t count = 0U;
    size_t i;
    uint32_t low_count = 0U;
    char measured[64];

    devs = board_map_get_ade_devices(&count);

    for (i = 0U; i < count; ++i) {
        if (HAL_GPIO_ReadPin(devs[i].irq_port, devs[i].irq_pin) == GPIO_PIN_RESET) {
            low_count++;
        }
    }

    (void)snprintf(measured,
                   sizeof(measured),
                   "irq_low=%lu/%lu",
                   (unsigned long)low_count,
                   (unsigned long)count);

    if (low_count == 0U) {
        board_test_set_result(result,
                              "ade.irq_idle",
                              "ADE7816 IRQ0 lines",
                              TEST_STATUS_PASS,
                              TEST_MODE_SANITY,
                              measured,
                              "All wired IRQ0 lines are inactive at idle.",
                              "!IRQ1 is not connected on this board revision.");
    } else {
        board_test_set_result(result,
                              "ade.irq_idle",
                              "ADE7816 IRQ0 lines",
                              TEST_STATUS_WARN,
                              TEST_MODE_SANITY,
                              measured,
                              "One or more IRQ0 lines are asserted low at idle.",
                              "Read STATUS0/STATUS1 and MASK0/MASK1 for affected device(s)." );
    }
}

void test_ade_register_snapshot(board_test_result_t *result)
{
    size_t count = 0U;
    const board_ade_device_map_t *devs = board_map_get_ade_devices(&count);
    size_t i;
    uint8_t verbose = board_test_get_verbose();
    uint32_t ok = 0U;
    uint32_t period;
    uint32_t vrms;
    uint32_t iarms;
    char measured[80];

    if (verbose != 0U) {
        test_uart_printf("[ INFO ] ade.registers: reading PERIOD/VRMS/IARMS for %lu device(s)...\r\n",
                         (unsigned long)count);
    }

    for (i = 0U; i < count; ++i) {
        if (ade7816_read_reg(devs[i].index, ADE_REG_PERIOD, 2U, &period) &&
            ade7816_read_reg(devs[i].index, ADE_REG_VRMS, 4U, &vrms) &&
            ade7816_read_reg(devs[i].index, ADE_REG_IARMS, 4U, &iarms)) {
            ok++;
        }
    }

    (void)snprintf(measured,
                   sizeof(measured),
                   "snapshot_ok=%lu/%lu",
                   (unsigned long)ok,
                   (unsigned long)count);

    if (ok == count) {
        board_test_set_result(result,
                              "ade.registers",
                              "ADE7816 snapshots",
                              TEST_STATUS_PASS,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "PERIOD/VRMS/IARMS snapshot reads succeeded for all devices.",
                              "");
    } else {
        board_test_set_result(result,
                              "ade.registers",
                              "ADE7816 snapshots",
                              TEST_STATUS_WARN,
                              TEST_MODE_AUTOMATIC,
                              measured,
                              "Snapshot read failed on one or more devices.",
                              "Use 'dump_registers ade <index>' for detail.");
    }
}

void test_ade_dump_registers(uint8_t index)
{
    uint32_t value = 0U;

    test_uart_printf("[ INFO ] ade.dump: ADE[%u] register dump\r\n", index);

    if (!ade7816_read_reg(index, ADE_REG_VERSION, 1U, &value)) {
        test_uart_write_str("  VERSION read failed\r\n");
        return;
    }
    test_uart_printf("  VERSION   (0x%04X) = 0x%02lX\r\n", ADE_REG_VERSION, (unsigned long)value);

    if (ade7816_read_reg(index, ADE_REG_CHECKSUM, 4U, &value)) {
        test_uart_printf("  CHECKSUM  (0x%04X) = 0x%08lX\r\n", ADE_REG_CHECKSUM, (unsigned long)value);
    }
    if (ade7816_read_reg(index, ADE_REG_STATUS0, 4U, &value)) {
        test_uart_printf("  STATUS0   (0x%04X) = 0x%08lX\r\n", ADE_REG_STATUS0, (unsigned long)value);
    }
    if (ade7816_read_reg(index, ADE_REG_STATUS1, 4U, &value)) {
        test_uart_printf("  STATUS1   (0x%04X) = 0x%08lX\r\n", ADE_REG_STATUS1, (unsigned long)value);
    }
    if (ade7816_read_reg(index, ADE_REG_MASK0, 4U, &value)) {
        test_uart_printf("  MASK0     (0x%04X) = 0x%08lX\r\n", ADE_REG_MASK0, (unsigned long)value);
    }
    if (ade7816_read_reg(index, ADE_REG_MASK1, 4U, &value)) {
        test_uart_printf("  MASK1     (0x%04X) = 0x%08lX\r\n", ADE_REG_MASK1, (unsigned long)value);
    }
    if (ade7816_read_reg(index, ADE_REG_CONFIG, 1U, &value)) {
        test_uart_printf("  CONFIG    (0x%04X) = 0x%02lX\r\n", ADE_REG_CONFIG, (unsigned long)value);
    }
    if (ade7816_read_reg(index, ADE_REG_MMODE, 1U, &value)) {
        test_uart_printf("  MMODE     (0x%04X) = 0x%02lX\r\n", ADE_REG_MMODE, (unsigned long)value);
    }
    if (ade7816_read_reg(index, ADE_REG_ACCMODE, 1U, &value)) {
        test_uart_printf("  ACCMODE   (0x%04X) = 0x%02lX\r\n", ADE_REG_ACCMODE, (unsigned long)value);
    }
    if (ade7816_read_reg(index, ADE_REG_LCYCMODE, 1U, &value)) {
        test_uart_printf("  LCYCMODE  (0x%04X) = 0x%02lX\r\n", ADE_REG_LCYCMODE, (unsigned long)value);
    }
    if (ade7816_read_reg(index, ADE_REG_HSDC_CFG, 1U, &value)) {
        test_uart_printf("  HSDC_CFG  (0x%04X) = 0x%02lX\r\n", ADE_REG_HSDC_CFG, (unsigned long)value);
    }
    if (ade7816_read_reg(index, ADE_REG_PERIOD, 2U, &value)) {
        test_uart_printf("  PERIOD    (0x%04X) = 0x%04lX\r\n", ADE_REG_PERIOD, (unsigned long)value);
    }
    if (ade7816_read_reg(index, ADE_REG_VRMS, 4U, &value)) {
        test_uart_printf("  VRMS      (0x%04X) = 0x%08lX\r\n", ADE_REG_VRMS, (unsigned long)value);
    }
    if (ade7816_read_reg(index, ADE_REG_IARMS, 4U, &value)) {
        test_uart_printf("  IARMS     (0x%04X) = 0x%08lX\r\n", ADE_REG_IARMS, (unsigned long)value);
    }
    if (ade7816_read_reg(index, ADE_REG_IBRMS, 4U, &value)) {
        test_uart_printf("  IBRMS     (0x%04X) = 0x%08lX\r\n", ADE_REG_IBRMS, (unsigned long)value);
    }
    if (ade7816_read_reg(index, ADE_REG_ICRMS, 4U, &value)) {
        test_uart_printf("  ICRMS     (0x%04X) = 0x%08lX\r\n", ADE_REG_ICRMS, (unsigned long)value);
    }
    if (ade7816_read_reg(index, ADE_REG_IDRMS, 4U, &value)) {
        test_uart_printf("  IDRMS     (0x%04X) = 0x%08lX\r\n", ADE_REG_IDRMS, (unsigned long)value);
    }
    if (ade7816_read_reg(index, ADE_REG_IERMS, 4U, &value)) {
        test_uart_printf("  IERMS     (0x%04X) = 0x%08lX\r\n", ADE_REG_IERMS, (unsigned long)value);
    }
    if (ade7816_read_reg(index, ADE_REG_IFRMS, 4U, &value)) {
        test_uart_printf("  IFRMS     (0x%04X) = 0x%08lX\r\n", ADE_REG_IFRMS, (unsigned long)value);
    }
}
