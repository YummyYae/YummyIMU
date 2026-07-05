#include "bmi088_mspm0.h"

#include "BMI088reg.h"
#include "ti_msp_dl_config.h"
#include <math.h>
#include <stdbool.h>

extern volatile uint32_t gSystemTickMs;

BMI088_Data_t BMI088Sensor;

static const uint8_t accel_init_table[][3] = {
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
    {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_HIGH, BMI088_INT1_IO_CTRL_ERROR},
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR},
};

static const uint8_t gyro_init_table[][3] = {
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_HIGH, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR},
};

static void delay_ms(uint32_t ms)
{
    uint32_t start = gSystemTickMs;
    uint32_t last_tick = start;
    uint32_t stagnant_loops = 0U;

    if ((__get_IPSR() != 0U) || (ms == 0U)) {
        DL_Common_delayCycles((CPUCLK_FREQ / 1000U) * ms);
        return;
    }

    while ((uint32_t)(gSystemTickMs - start) < ms) {
        if (gSystemTickMs != last_tick) {
            last_tick = gSystemTickMs;
            stagnant_loops = 0U;
        } else if (++stagnant_loops > 20000U) {
            uint32_t elapsed = gSystemTickMs - start;

            if (elapsed < ms) {
                DL_Common_delayCycles((CPUCLK_FREQ / 1000U) * (ms - elapsed));
            }
            return;
        }
        __WFI();
    }
}

static uint8_t spi_transfer(uint8_t tx)
{
    while (!DL_SPI_isRXFIFOEmpty(SPI_BMI088_INST)) {
        (void) DL_SPI_receiveData8(SPI_BMI088_INST);
    }

    DL_SPI_transmitDataBlocking8(SPI_BMI088_INST, tx);
    return DL_SPI_receiveDataBlocking8(SPI_BMI088_INST);
}

static void cs_accel_low(void)
{
    DL_GPIO_clearPins(BMI088_PORT, BMI088_CSB1_PIN);
}

static void cs_accel_high(void)
{
    while (DL_SPI_isBusy(SPI_BMI088_INST)) {
    }
    DL_GPIO_setPins(BMI088_PORT, BMI088_CSB1_PIN);
}

static void cs_gyro_low(void)
{
    DL_GPIO_clearPins(BMI088_PORT, BMI088_CSB2_PIN);
}

static void cs_gyro_high(void)
{
    while (DL_SPI_isBusy(SPI_BMI088_INST)) {
    }
    DL_GPIO_setPins(BMI088_PORT, BMI088_CSB2_PIN);
}

static void write_single_reg(uint8_t reg, uint8_t data)
{
    (void) spi_transfer(reg);
    (void) spi_transfer(data);
}

static uint8_t read_single_reg(uint8_t reg)
{
    (void) spi_transfer(reg | 0x80U);
    return spi_transfer(0x55U);
}

static void read_multi_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    (void) spi_transfer(reg | 0x80U);
    while (len-- != 0U) {
        *buf++ = spi_transfer(0x55U);
    }
}

static void accel_write_single_reg(uint8_t reg, uint8_t data)
{
    cs_accel_low();
    write_single_reg(reg, data);
    cs_accel_high();
}

static uint8_t accel_read_single_reg(uint8_t reg)
{
    uint8_t data;

    cs_accel_low();
    (void) spi_transfer(reg | 0x80U);
    (void) spi_transfer(0x55U);
    data = spi_transfer(0x55U);
    cs_accel_high();

    return data;
}

static void accel_read_multi_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    cs_accel_low();
    (void) spi_transfer(reg | 0x80U);
    (void) spi_transfer(0x55U);
    while (len-- != 0U) {
        *buf++ = spi_transfer(0x55U);
    }
    cs_accel_high();
}

static void gyro_write_single_reg(uint8_t reg, uint8_t data)
{
    cs_gyro_low();
    write_single_reg(reg, data);
    cs_gyro_high();
}

static uint8_t gyro_read_single_reg(uint8_t reg)
{
    uint8_t data;

    cs_gyro_low();
    data = read_single_reg(reg);
    cs_gyro_high();

    return data;
}

static void gyro_read_multi_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    cs_gyro_low();
    read_multi_reg(reg, buf, len);
    cs_gyro_high();
}

uint8_t BMI088_ReadAccelChipId(void)
{
    return accel_read_single_reg(BMI088_ACC_CHIP_ID);
}

uint8_t BMI088_ReadGyroChipId(void)
{
    return gyro_read_single_reg(BMI088_GYRO_CHIP_ID);
}

static uint8_t bmi088_accel_init(void)
{
    uint8_t error = BMI088_NO_ERROR;

    (void) accel_read_single_reg(BMI088_ACC_CHIP_ID);
    delay_ms(1);
    if (accel_read_single_reg(BMI088_ACC_CHIP_ID) != BMI088_ACC_CHIP_ID_VALUE) {
        delay_ms(1);
        if (accel_read_single_reg(BMI088_ACC_CHIP_ID) != BMI088_ACC_CHIP_ID_VALUE) {
            return BMI088_NO_SENSOR;
        }
    }

    accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME_MS);

    (void) accel_read_single_reg(BMI088_ACC_CHIP_ID);
    delay_ms(1);
    if (accel_read_single_reg(BMI088_ACC_CHIP_ID) != BMI088_ACC_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    for (uint32_t i = 0; i < (sizeof(accel_init_table) / sizeof(accel_init_table[0])); i++) {
        accel_write_single_reg(accel_init_table[i][0], accel_init_table[i][1]);
        delay_ms(1);

        if (accel_read_single_reg(accel_init_table[i][0]) != accel_init_table[i][1]) {
            error |= accel_init_table[i][2];
        }
    }

    return error;
}

static uint8_t bmi088_gyro_init(void)
{
    uint8_t error = BMI088_NO_ERROR;

    (void) gyro_read_single_reg(BMI088_GYRO_CHIP_ID);
    delay_ms(1);
    if (gyro_read_single_reg(BMI088_GYRO_CHIP_ID) != BMI088_GYRO_CHIP_ID_VALUE) {
        delay_ms(1);
        if (gyro_read_single_reg(BMI088_GYRO_CHIP_ID) != BMI088_GYRO_CHIP_ID_VALUE) {
            return BMI088_NO_SENSOR;
        }
    }

    gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME_MS);

    (void) gyro_read_single_reg(BMI088_GYRO_CHIP_ID);
    delay_ms(1);
    if (gyro_read_single_reg(BMI088_GYRO_CHIP_ID) != BMI088_GYRO_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    for (uint32_t i = 0; i < (sizeof(gyro_init_table) / sizeof(gyro_init_table[0])); i++) {
        gyro_write_single_reg(gyro_init_table[i][0], gyro_init_table[i][1]);
        delay_ms(1);

        if (gyro_read_single_reg(gyro_init_table[i][0]) != gyro_init_table[i][1]) {
            error |= gyro_init_table[i][2];
        }
    }

    return error;
}

static void use_saved_offsets(BMI088_Data_t *BMI088Sensor)
{
    BMI088Sensor->GyroOffset[0] = BMI088_GX_OFFSET;
    BMI088Sensor->GyroOffset[1] = BMI088_GY_OFFSET;
    BMI088Sensor->GyroOffset[2] = BMI088_GZ_OFFSET;
    BMI088Sensor->AccelOffset[0] = BMI088_AX_OFFSET;
    BMI088Sensor->AccelOffset[1] = BMI088_AY_OFFSET;
    BMI088Sensor->AccelOffset[2] = BMI088_AZ_OFFSET;
    BMI088Sensor->gNorm = BMI088_G_NORM;
    BMI088Sensor->AccelScale = 9.81f / BMI088Sensor->gNorm;
    BMI088Sensor->TempWhenCali = 40.0f;
}

#define BMI088_STARTUP_CALIB_SAMPLES 1200U
#define BMI088_STARTUP_CALIB_RETRIES 3U
#define BMI088_STARTUP_G_NORM_LIMIT 0.80f
#define BMI088_STARTUP_GYRO_DIFF_LIMIT 0.18f
#define BMI088_RAW_SATURATION_LIMIT 32000

static float absf_local(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void calibrate_gyro_offset(BMI088_Data_t *BMI088Sensor)
{
    uint8_t buf[8];
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    const uint16_t samples = 1000U;

    use_saved_offsets(BMI088Sensor);

    for (uint16_t i = 0; i < samples; i++) {
        gyro_read_multi_reg(BMI088_GYRO_CHIP_ID, buf, 8);
        if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
            gyro_sum[0] += (float) ((int16_t) ((buf[3] << 8) | buf[2])) * BMI088_GYRO_2000_SEN;
            gyro_sum[1] += (float) ((int16_t) ((buf[5] << 8) | buf[4])) * BMI088_GYRO_2000_SEN;
            gyro_sum[2] += (float) ((int16_t) ((buf[7] << 8) | buf[6])) * BMI088_GYRO_2000_SEN;
        }
        delay_ms(1);
    }

    BMI088Sensor->GyroOffset[0] = gyro_sum[0] / (float) samples;
    BMI088Sensor->GyroOffset[1] = gyro_sum[1] / (float) samples;
    BMI088Sensor->GyroOffset[2] = gyro_sum[2] / (float) samples;
}

uint8_t BMI088_StartupCalibrate(void)
{
    uint8_t buf[8];
    uint8_t calibrated = 0U;

    for (uint8_t retry = 0U; retry < BMI088_STARTUP_CALIB_RETRIES; retry++) {
        float accel_sum[3] = {0.0f, 0.0f, 0.0f};
        float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
        float gyro_min[3] = {0.0f, 0.0f, 0.0f};
        float gyro_max[3] = {0.0f, 0.0f, 0.0f};
        float g_norm_min = 0.0f;
        float g_norm_max = 0.0f;
        uint16_t valid_samples = 0U;

        for (uint16_t sample = 0U; sample < BMI088_STARTUP_CALIB_SAMPLES; sample++) {
            float accel[3];
            float gyro[3];
            float g_norm;
            int16_t raw;

            accel_read_multi_reg(BMI088_ACCEL_XOUT_L, buf, 6);
            raw = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8U));
            accel[0] = (float)raw * BMI088_ACCEL_6G_SEN;
            raw = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8U));
            accel[1] = (float)raw * BMI088_ACCEL_6G_SEN;
            raw = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8U));
            accel[2] = (float)raw * BMI088_ACCEL_6G_SEN;

            gyro_read_multi_reg(BMI088_GYRO_CHIP_ID, buf, 8);
            if (buf[0] != BMI088_GYRO_CHIP_ID_VALUE) {
                delay_ms(1U);
                continue;
            }

            raw = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8U));
            gyro[0] = (float)raw * BMI088_GYRO_2000_SEN;
            raw = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8U));
            gyro[1] = (float)raw * BMI088_GYRO_2000_SEN;
            raw = (int16_t)((uint16_t)buf[6] | ((uint16_t)buf[7] << 8U));
            gyro[2] = (float)raw * BMI088_GYRO_2000_SEN;

            g_norm = sqrtf((accel[0] * accel[0]) + (accel[1] * accel[1]) + (accel[2] * accel[2]));

            if (valid_samples == 0U) {
                g_norm_min = g_norm;
                g_norm_max = g_norm;
                for (uint8_t axis = 0U; axis < 3U; axis++) {
                    gyro_min[axis] = gyro[axis];
                    gyro_max[axis] = gyro[axis];
                }
            } else {
                if (g_norm < g_norm_min) {
                    g_norm_min = g_norm;
                }
                if (g_norm > g_norm_max) {
                    g_norm_max = g_norm;
                }
                for (uint8_t axis = 0U; axis < 3U; axis++) {
                    if (gyro[axis] < gyro_min[axis]) {
                        gyro_min[axis] = gyro[axis];
                    }
                    if (gyro[axis] > gyro_max[axis]) {
                        gyro_max[axis] = gyro[axis];
                    }
                }
            }

            for (uint8_t axis = 0U; axis < 3U; axis++) {
                accel_sum[axis] += accel[axis];
                gyro_sum[axis] += gyro[axis];
            }
            valid_samples++;
            delay_ms(1U);
        }

        if (valid_samples < (BMI088_STARTUP_CALIB_SAMPLES / 2U)) {
            continue;
        }

        if ((g_norm_max - g_norm_min) > BMI088_STARTUP_G_NORM_LIMIT) {
            continue;
        }

        if (((gyro_max[0] - gyro_min[0]) > BMI088_STARTUP_GYRO_DIFF_LIMIT) ||
            ((gyro_max[1] - gyro_min[1]) > BMI088_STARTUP_GYRO_DIFF_LIMIT) ||
            ((gyro_max[2] - gyro_min[2]) > BMI088_STARTUP_GYRO_DIFF_LIMIT)) {
            continue;
        }

        BMI088Sensor.gNorm = 0.5f * (g_norm_min + g_norm_max);
        if (absf_local(BMI088Sensor.gNorm - 9.80665f) > 1.5f) {
            BMI088Sensor.gNorm = 9.80665f;
        }

        BMI088Sensor.AccelScale = 9.80665f / BMI088Sensor.gNorm;
        for (uint8_t axis = 0U; axis < 3U; axis++) {
            BMI088Sensor.GyroOffset[axis] = gyro_sum[axis] / (float)valid_samples;
            BMI088Sensor.AccelOffset[axis] = 0.0f;
        }
        BMI088Sensor.AccelOffset[0] = BMI088_AX_OFFSET;
        BMI088Sensor.AccelOffset[1] = BMI088_AY_OFFSET;
        BMI088Sensor.AccelOffset[2] = BMI088_AZ_OFFSET;
        BMI088Sensor.TempWhenCali = 40.0f;
        calibrated = 1U;
        break;
    }

    return calibrated;
}

uint8_t BMI088_Init(uint8_t calibrate)
{
    uint8_t error = BMI088_NO_SENSOR;

    BMI088Sensor.GyroSaturationCount = 0U;
    BMI088Sensor.AccelSaturationCount = 0U;
    BMI088Sensor.LastGyroSaturated = 0U;
    BMI088Sensor.LastAccelSaturated = 0U;

    DL_GPIO_setPins(BMI088_PORT, BMI088_CSB1_PIN | BMI088_CSB2_PIN);
    delay_ms(2);

    for (uint32_t i = 0; i < 3U; i++) {
        error = bmi088_accel_init();
        error |= bmi088_gyro_init();
        if (error != BMI088_NO_SENSOR) {
            break;
        }
        delay_ms(10);
    }

    if (calibrate != 0U) {
        if (BMI088_StartupCalibrate() == 0U) {
            calibrate_gyro_offset(&BMI088Sensor);
        }
    } else {
        use_saved_offsets(&BMI088Sensor);
    }

    return error;
}

void BMI088_Read(BMI088_Data_t *BMI088Sensor)
{
    uint8_t buf[8];
    int16_t raw;
    uint8_t gyro_saturated = 0U;
    uint8_t accel_saturated = 0U;

    accel_read_multi_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    raw = (int16_t) ((buf[1] << 8) | buf[0]);
    if ((raw >= BMI088_RAW_SATURATION_LIMIT) || (raw <= -BMI088_RAW_SATURATION_LIMIT)) {
        accel_saturated = 1U;
    }
    BMI088Sensor->Accel[0] = ((float) raw * BMI088_ACCEL_6G_SEN * BMI088Sensor->AccelScale) - BMI088Sensor->AccelOffset[0];
    raw = (int16_t) ((buf[3] << 8) | buf[2]);
    if ((raw >= BMI088_RAW_SATURATION_LIMIT) || (raw <= -BMI088_RAW_SATURATION_LIMIT)) {
        accel_saturated = 1U;
    }
    BMI088Sensor->Accel[1] = ((float) raw * BMI088_ACCEL_6G_SEN * BMI088Sensor->AccelScale) - BMI088Sensor->AccelOffset[1];
    raw = (int16_t) ((buf[5] << 8) | buf[4]);
    if ((raw >= BMI088_RAW_SATURATION_LIMIT) || (raw <= -BMI088_RAW_SATURATION_LIMIT)) {
        accel_saturated = 1U;
    }
    BMI088Sensor->Accel[2] = (float) raw * BMI088_ACCEL_6G_SEN * BMI088Sensor->AccelScale;

    gyro_read_multi_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
        raw = (int16_t) ((buf[3] << 8) | buf[2]);
        if ((raw >= BMI088_RAW_SATURATION_LIMIT) || (raw <= -BMI088_RAW_SATURATION_LIMIT)) {
            gyro_saturated = 1U;
        }
        BMI088Sensor->Gyro[0] = ((float) raw * BMI088_GYRO_2000_SEN) - BMI088Sensor->GyroOffset[0];
        raw = (int16_t) ((buf[5] << 8) | buf[4]);
        if ((raw >= BMI088_RAW_SATURATION_LIMIT) || (raw <= -BMI088_RAW_SATURATION_LIMIT)) {
            gyro_saturated = 1U;
        }
        BMI088Sensor->Gyro[1] = ((float) raw * BMI088_GYRO_2000_SEN) - BMI088Sensor->GyroOffset[1];
        raw = (int16_t) ((buf[7] << 8) | buf[6]);
        if ((raw >= BMI088_RAW_SATURATION_LIMIT) || (raw <= -BMI088_RAW_SATURATION_LIMIT)) {
            gyro_saturated = 1U;
        }
        BMI088Sensor->Gyro[2] = ((float) raw * BMI088_GYRO_2000_SEN) - BMI088Sensor->GyroOffset[2];
    }

    BMI088Sensor->LastGyroSaturated = gyro_saturated;
    BMI088Sensor->LastAccelSaturated = accel_saturated;
    if (gyro_saturated != 0U) {
        BMI088Sensor->GyroSaturationCount++;
    }
    if (accel_saturated != 0U) {
        BMI088Sensor->AccelSaturationCount++;
    }

    (void) raw;
}

void BMI088_ReadTemperatureRaw(uint8_t *temp_m, uint8_t *temp_l, int16_t *raw)
{
    uint8_t buf[2];
    int16_t raw_value;

    accel_read_multi_reg(BMI088_TEMP_M, buf, 2U);
    raw_value = (int16_t)(((uint16_t)buf[0] << 3U) | ((uint16_t)buf[1] >> 5U));
    if (raw_value > 1023) {
        raw_value -= 2048;
    }

    if (temp_m != 0) {
        *temp_m = buf[0];
    }
    if (temp_l != 0) {
        *temp_l = buf[1];
    }
    if (raw != 0) {
        *raw = raw_value;
    }
}

float BMI088_ReadTemperature(void)
{
    int16_t raw;

    BMI088_ReadTemperatureRaw(0, 0, &raw);
    BMI088Sensor.Temperature = ((float)raw * BMI088_TEMP_FACTOR) + BMI088_TEMP_OFFSET;
    return BMI088Sensor.Temperature;
}
