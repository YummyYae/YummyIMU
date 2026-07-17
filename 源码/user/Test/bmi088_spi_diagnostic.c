#include "bmi088_spi_diagnostic.h"

#include "ti_msp_dl_config.h"

#include <stdint.h>

#define DIAG_ACCEL_EXPECTED_ID     0x1EU
#define DIAG_GYRO_EXPECTED_ID      0x0FU
#define DIAG_SAMPLE_COUNT          16U
#define DIAG_SPI_TIMEOUT_CYCLES    (CPUCLK_FREQ / 500U)
#define DIAG_UART_TIMEOUT_CYCLES   (CPUCLK_FREQ / 100U)
#define DIAG_LED_PORT              GPIOB
#define DIAG_LED_PIN               DL_GPIO_PIN_18
#define DIAG_LED_IOMUX             IOMUX_PINCM44

typedef struct {
    uint8_t address_rx;
    uint8_t dummy_rx;
    uint8_t data_rx;
    uint8_t transfer_ok;
} DiagAccelRead_t;

typedef struct {
    uint8_t address_rx;
    uint8_t data_rx;
    uint8_t next_rx;
    uint8_t transfer_ok;
} DiagGyroRead_t;

typedef struct {
    uint32_t frequency_hz;
    uint16_t divider;
} DiagSpiRate_t;

static const DiagSpiRate_t gDiagSpiRates[] = {
    {500000U, 79U},
    {1000000U, 39U},
    {5000000U, 7U},
    {10000000U, 3U},
};

static void diag_delay_ms(uint32_t milliseconds)
{
    while (milliseconds-- != 0U) {
        DL_Common_delayCycles(CPUCLK_FREQ / 1000U);
    }
}

static void diag_uart_putc(char value)
{
    uint32_t timeout = DIAG_UART_TIMEOUT_CYCLES;

    while (DL_UART_Main_isTXFIFOFull(UART_OUT_INST)) {
        if (timeout-- == 0U) {
            return;
        }
    }
    DL_UART_Main_transmitData(UART_OUT_INST, (uint8_t)value);
}

static void diag_uart_write(const char *text)
{
    while (*text != '\0') {
        diag_uart_putc(*text++);
    }
}

static void diag_uart_hex8(uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";

    diag_uart_putc(hex[(value >> 4U) & 0x0FU]);
    diag_uart_putc(hex[value & 0x0FU]);
}

static void diag_uart_u32(uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    if (value == 0U) {
        diag_uart_putc('0');
        return;
    }

    while ((value != 0U) && (count < sizeof(digits))) {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    }
    while (count != 0U) {
        diag_uart_putc(digits[--count]);
    }
}

static void diag_uart_flush(void)
{
    uint32_t timeout = DIAG_UART_TIMEOUT_CYCLES;

    while (DL_UART_Main_isBusy(UART_OUT_INST)) {
        if (timeout-- == 0U) {
            return;
        }
    }
}

static uint8_t diag_spi_wait_idle(void)
{
    uint32_t timeout = DIAG_SPI_TIMEOUT_CYCLES;

    while (DL_SPI_isBusy(SPI_BMI088_INST)) {
        if (timeout-- == 0U) {
            return 0U;
        }
    }
    return 1U;
}

static void diag_spi_drain_rx(void)
{
    while (!DL_SPI_isRXFIFOEmpty(SPI_BMI088_INST)) {
        (void)DL_SPI_receiveData8(SPI_BMI088_INST);
    }
}

static uint8_t diag_spi_transfer(uint8_t tx, uint8_t *rx)
{
    uint32_t timeout = DIAG_SPI_TIMEOUT_CYCLES;

    diag_spi_drain_rx();
    while (DL_SPI_isTXFIFOFull(SPI_BMI088_INST)) {
        if (timeout-- == 0U) {
            return 0U;
        }
    }
    DL_SPI_transmitData8(SPI_BMI088_INST, tx);

    timeout = DIAG_SPI_TIMEOUT_CYCLES;
    while (DL_SPI_isRXFIFOEmpty(SPI_BMI088_INST)) {
        if (timeout-- == 0U) {
            return 0U;
        }
    }
    *rx = DL_SPI_receiveData8(SPI_BMI088_INST);
    return 1U;
}

static void diag_set_chip_selects_high(void)
{
    DL_GPIO_setPins(BMI088_PORT, BMI088_CSB1_PIN | BMI088_CSB2_PIN);
}

static DiagAccelRead_t diag_read_accel_id(void)
{
    DiagAccelRead_t result = {0U, 0U, 0U, 1U};

    DL_GPIO_clearPins(BMI088_PORT, BMI088_CSB1_PIN);
    result.transfer_ok &= diag_spi_transfer(0x80U, &result.address_rx);
    result.transfer_ok &= diag_spi_transfer(0x00U, &result.dummy_rx);
    result.transfer_ok &= diag_spi_transfer(0x00U, &result.data_rx);
    result.transfer_ok &= diag_spi_wait_idle();
    DL_GPIO_setPins(BMI088_PORT, BMI088_CSB1_PIN);
    DL_Common_delayCycles(CPUCLK_FREQ / 100000U);

    return result;
}

static DiagGyroRead_t diag_read_gyro_id(void)
{
    DiagGyroRead_t result = {0U, 0U, 0U, 1U};

    DL_GPIO_clearPins(BMI088_PORT, BMI088_CSB2_PIN);
    result.transfer_ok &= diag_spi_transfer(0x80U, &result.address_rx);
    result.transfer_ok &= diag_spi_transfer(0x00U, &result.data_rx);
    result.transfer_ok &= diag_spi_transfer(0x00U, &result.next_rx);
    result.transfer_ok &= diag_spi_wait_idle();
    DL_GPIO_setPins(BMI088_PORT, BMI088_CSB2_PIN);
    DL_Common_delayCycles(CPUCLK_FREQ / 100000U);

    return result;
}

static void diag_configure_spi(uint16_t divider, uint8_t mode3)
{
    (void)diag_spi_wait_idle();
    diag_set_chip_selects_high();
    DL_SPI_disable(SPI_BMI088_INST);
    DL_SPI_setFrameFormat(SPI_BMI088_INST,
                          (mode3 != 0U) ? DL_SPI_FRAME_FORMAT_MOTO3_POL1_PHA1
                                        : DL_SPI_FRAME_FORMAT_MOTO3_POL0_PHA0);
    DL_SPI_setBitRateSerialClockDivider(SPI_BMI088_INST, divider);
    diag_spi_drain_rx();
    DL_SPI_enable(SPI_BMI088_INST);
    diag_delay_ms(2U);
}

static void diag_print_result(uint32_t sequence,
                              const DiagSpiRate_t *rate,
                              uint8_t mode3)
{
    DiagAccelRead_t accel_first = {0U, 0U, 0U, 0U};
    DiagGyroRead_t gyro_first = {0U, 0U, 0U, 0U};
    uint32_t accel_ok = 0U;
    uint32_t gyro_ok = 0U;
    uint32_t gyro_zero = 0U;
    uint32_t gyro_ff = 0U;
    uint32_t gyro_other = 0U;
    uint32_t timeout_count = 0U;

    diag_configure_spi(rate->divider, mode3);

    for (uint32_t sample = 0U; sample < DIAG_SAMPLE_COUNT; sample++) {
        DiagAccelRead_t accel = diag_read_accel_id();
        DiagGyroRead_t gyro = diag_read_gyro_id();

        if (sample == 0U) {
            accel_first = accel;
            gyro_first = gyro;
        }
        if ((accel.transfer_ok != 0U) && (accel.data_rx == DIAG_ACCEL_EXPECTED_ID)) {
            accel_ok++;
        }
        if (gyro.transfer_ok == 0U) {
            timeout_count++;
        } else if (gyro.data_rx == DIAG_GYRO_EXPECTED_ID) {
            gyro_ok++;
        } else if (gyro.data_rx == 0x00U) {
            gyro_zero++;
        } else if (gyro.data_rx == 0xFFU) {
            gyro_ff++;
        } else {
            gyro_other++;
        }
    }

    diag_uart_write("DIAG:SEQ=");
    diag_uart_u32(sequence);
    diag_uart_write(",MODE=");
    diag_uart_u32((mode3 != 0U) ? 3U : 0U);
    diag_uart_write(",SPI_HZ=");
    diag_uart_u32(rate->frequency_hz);
    diag_uart_write(",A_RX=");
    diag_uart_hex8(accel_first.address_rx);
    diag_uart_putc('/');
    diag_uart_hex8(accel_first.dummy_rx);
    diag_uart_putc('/');
    diag_uart_hex8(accel_first.data_rx);
    diag_uart_write(",G_RX=");
    diag_uart_hex8(gyro_first.address_rx);
    diag_uart_putc('/');
    diag_uart_hex8(gyro_first.data_rx);
    diag_uart_putc('/');
    diag_uart_hex8(gyro_first.next_rx);
    diag_uart_write(",A_OK=");
    diag_uart_u32(accel_ok);
    diag_uart_write(",G_OK=");
    diag_uart_u32(gyro_ok);
    diag_uart_write(",G_00=");
    diag_uart_u32(gyro_zero);
    diag_uart_write(",G_FF=");
    diag_uart_u32(gyro_ff);
    diag_uart_write(",G_OTHER=");
    diag_uart_u32(gyro_other);
    diag_uart_write(",TIMEOUT=");
    diag_uart_u32(timeout_count);
    diag_uart_write(",RESULT=");

    if (gyro_ok == DIAG_SAMPLE_COUNT) {
        diag_uart_write("GYRO_OK");
        DL_GPIO_clearPins(DIAG_LED_PORT, DIAG_LED_PIN);
    } else if (gyro_zero == DIAG_SAMPLE_COUNT) {
        diag_uart_write("GYRO_NO_RESPONSE_LOW");
        DL_GPIO_togglePins(DIAG_LED_PORT, DIAG_LED_PIN);
    } else if (gyro_ff == DIAG_SAMPLE_COUNT) {
        diag_uart_write("GYRO_NO_RESPONSE_HIGH");
        DL_GPIO_togglePins(DIAG_LED_PORT, DIAG_LED_PIN);
    } else if (timeout_count != 0U) {
        diag_uart_write("SPI_CONTROLLER_TIMEOUT");
        DL_GPIO_togglePins(DIAG_LED_PORT, DIAG_LED_PIN);
    } else {
        diag_uart_write("GYRO_UNSTABLE_OR_WRONG_FRAME");
        DL_GPIO_togglePins(DIAG_LED_PORT, DIAG_LED_PIN);
    }
    diag_uart_putc('\n');
    diag_uart_flush();
}

void BMI088_SpiDiagnostic_Run(void)
{
    uint32_t sequence = 0U;

    __disable_irq();

    /* 只初始化本测试需要的电源、引脚、时钟、UART2 与 SPI0。 */
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_SYSCTL_CLK_init();
    SYSCFG_DL_UART_OUT_init();
    SYSCFG_DL_SPI_BMI088_init();

    DL_UART_Main_disableInterrupt(UART_OUT_INST, 0xFFFFFFFFUL);
    DL_GPIO_initDigitalOutput(DIAG_LED_IOMUX);
    DL_GPIO_setPins(DIAG_LED_PORT, DIAG_LED_PIN);
    DL_GPIO_enableOutput(DIAG_LED_PORT, DIAG_LED_PIN);
    diag_set_chip_selects_high();
    diag_delay_ms(100U);

    diag_uart_write("\nBMI088_SPI_DIAGNOSTIC\n");
    diag_uart_write("UART:921600,8N1\n");
    diag_uart_write("PINS:SCK=PA12,MOSI=PA14,MISO=PA13,CSB1=PA15,CSB2=PA16\n");
    diag_uart_write("EXPECTED:A=0x1E,G=0x0F\n");
    diag_uart_write("A_RX=ADDR/DUMMY/ID,G_RX=ADDR/ID/NEXT\n");
    diag_uart_write("TEST:MODE0_AND_MODE3,500KHZ_TO_10MHZ\n");
    diag_uart_flush();

    while (1) {
        for (uint8_t mode3 = 0U; mode3 <= 1U; mode3++) {
            for (uint32_t rate_index = 0U;
                 rate_index < (sizeof(gDiagSpiRates) / sizeof(gDiagSpiRates[0]));
                 rate_index++) {
                diag_print_result(++sequence, &gDiagSpiRates[rate_index], mode3);
                diag_delay_ms(250U);
            }
        }
    }
}
