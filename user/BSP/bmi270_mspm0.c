#include "bmi270_mspm0.h"

#include "bmi220_config_file.h"
#include "bmi270_config_file.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>

extern volatile uint32_t gSystemTickMs;

BMI270_Data_t BMI270Sensor;
BMI270_Debug_t BMI270_Debug;
uint8_t BMI270_RegDumpAfterReset[128];
uint8_t BMI270_RegDumpAfterConfig[128];
uint8_t BMI270_RegDumpAfterPower[128];

#define BMI270_REG_CHIP_ID         0x00U
#define BMI270_REG_ERR_REG         0x02U
#define BMI270_REG_ACC_X_LSB       0x0CU
#define BMI270_REG_INTERNAL_STATUS 0x21U
#define BMI270_REG_TEMPERATURE_0   0x22U
#define BMI270_REG_ACC_CONF        0x40U
#define BMI270_REG_ACC_RANGE       0x41U
#define BMI270_REG_GYR_CONF        0x42U
#define BMI270_REG_GYR_RANGE       0x43U
#define BMI270_REG_STATUS          0x1BU
#define BMI270_REG_INIT_CTRL       0x59U
#define BMI270_REG_INIT_ADDR_0     0x5BU
#define BMI270_REG_INIT_DATA       0x5EU
#define BMI270_REG_PWR_CONF        0x7CU
#define BMI270_REG_PWR_CTRL        0x7DU
#define BMI270_REG_CMD             0x7EU

#define BMI270_CMD_SOFT_RESET      0xB6U
#define BMI270_INIT_OK             0x01U
#define BMI270_INIT_STATUS_MASK    0x0FU

#define BMI270_ACC_CONF_200HZ_PERF 0xA9U
#define BMI270_GYR_CONF_3200HZ_PERF 0xEDU
#define BMI270_ACC_RANGE_8G        0x02U
#define BMI270_GYR_RANGE_2000DPS   0x00U
#define BMI270_PWR_CTRL_ACC_GYR    0x06U
#define BMI220_PWR_CONF_FIFO_WAKE  0x02U
#define BMI220_PWR_CTRL_ALL_ON     0x0EU
#define BMI270_RAW_SATURATION_LIMIT 32000

#define BMI270_CONFIG_CHUNK_LEN    256U
#define BMI270_CONFIG_STATUS_POLLS 20U
#define BMI270_CONFIG_STATUS_DELAY 10U
#define BMI220_CONFIG_LOAD_DELAY   140U
#define BMI270_SOFT_RESET_DELAY    50U
#define BMI270_GYRO_OFFSET_SAMPLES 1000U

#define BMI270_DEBUG_VARIANT_BMI270 0U
#define BMI270_DEBUG_VARIANT_BMI220 1U

static void delay_ms(uint32_t ms);
static uint8_t read_reg(uint8_t reg);
static uint8_t init_bmi220(uint8_t error);
static uint8_t init_bmi270(uint8_t error);
static void calibrate_gyro_offset(void);

static bool is_valid_chip_id(uint8_t chip_id)
{
    return (chip_id == BMI270_CHIP_ID_VALUE) || (chip_id == BMI220_CHIP_ID_VALUE);
}

static bool read_chip_id_retry(uint8_t *chip_id)
{
    for (uint8_t i = 0U; i < 10U; i++) {
        (void) read_reg(BMI270_REG_CHIP_ID);
        delay_ms(2U);
        *chip_id = read_reg(BMI270_REG_CHIP_ID);
        if (is_valid_chip_id(*chip_id)) {
            return true;
        }
    }

    return false;
}

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
    while (!DL_SPI_isRXFIFOEmpty(SPI_Bmi270_INST)) {
        (void) DL_SPI_receiveData8(SPI_Bmi270_INST);
    }

    DL_SPI_transmitDataBlocking8(SPI_Bmi270_INST, tx);
    return DL_SPI_receiveDataBlocking8(SPI_Bmi270_INST);
}

static void cs_low(void)
{
    DL_GPIO_clearPins(BMI270_PORT, BMI270_CS_PIN);
}

static void cs_high(void)
{
    while (DL_SPI_isBusy(SPI_Bmi270_INST)) {
    }
    DL_GPIO_setPins(BMI270_PORT, BMI270_CS_PIN);
}

static void write_regs(uint8_t reg, const uint8_t *data, uint16_t len)
{
    cs_low();
    (void) spi_transfer(reg & 0x7FU);
    while (len-- != 0U) {
        (void) spi_transfer(*data++);
    }
    cs_high();
}

static void write_reg(uint8_t reg, uint8_t data)
{
    write_regs(reg, &data, 1U);
}

static void read_regs(uint8_t reg, uint8_t *data, uint16_t len)
{
    cs_low();
    (void) spi_transfer(reg | 0x80U);
    (void) spi_transfer(0x00U);
    while (len-- != 0U) {
        *data++ = spi_transfer(0x00U);
    }
    cs_high();
}

static uint8_t read_reg(uint8_t reg)
{
    uint8_t data;

    read_regs(reg, &data, 1U);
    return data;
}

static void upload_config_file(const uint8_t *config, uint16_t size)
{
    uint16_t index;

    for (index = 0U; index < size; index += BMI270_CONFIG_CHUNK_LEN) {
        uint8_t addr[2];
        uint16_t remaining = (uint16_t)(size - index);
        uint16_t len = (remaining > BMI270_CONFIG_CHUNK_LEN) ? BMI270_CONFIG_CHUNK_LEN : remaining;

        addr[0] = (uint8_t)((index / 2U) & 0x0FU);
        addr[1] = (uint8_t)(((index / 2U) >> 4U) & 0xFFU);
        write_regs(BMI270_REG_INIT_ADDR_0, addr, 2U);
        delay_ms(1U);
        write_regs(BMI270_REG_INIT_DATA, &config[index], len);
        delay_ms(1U);
    }
}

static void dump_registers(uint8_t *dump)
{
    for (uint8_t reg = 0U; reg < 128U; reg++) {
        dump[reg] = read_reg(reg);
        delay_ms(1U);
    }
}

static void clear_debug_state(void)
{
    for (uint8_t axis = 0U; axis < 3U; axis++) {
        BMI270Sensor.GyroOffset[axis] = 0.0f;
    }

    BMI270Sensor.GyroSaturationCount = 0U;
    BMI270Sensor.AccelSaturationCount = 0U;
    BMI270Sensor.LastGyroSaturated = 0U;
    BMI270Sensor.LastAccelSaturated = 0U;

    BMI270_Debug.ChipIdBeforeReset = 0U;
    BMI270_Debug.ChipIdAfterReset = 0U;
    BMI270_Debug.PwrConfAfterSuspend = 0U;
    BMI270_Debug.InitCtrlBeforeLoad = 0U;
    BMI270_Debug.InitCtrlAfterLoad = 0U;
    BMI270_Debug.InternalStatus = 0U;
    BMI270_Debug.ErrorReg = 0U;
    BMI270_Debug.PwrCtrlAfterEnable = 0U;
    BMI270_Debug.AccConfAfterWrite = 0U;
    BMI270_Debug.AccRangeAfterWrite = 0U;
    BMI270_Debug.GyrConfAfterWrite = 0U;
    BMI270_Debug.GyrRangeAfterWrite = 0U;
    BMI270_Debug.CompatVariant = BMI270_DEBUG_VARIANT_BMI270;
    BMI270_Debug.CompatDataNonZero = 0U;
    BMI270_Debug.StatusAfterEnable = 0U;
    BMI270_Debug.Reserved = 0U;
}

static void enter_config_load_mode(void)
{
    write_reg(BMI270_REG_PWR_CONF, 0x00U);
    delay_ms(1U);
    BMI270_Debug.PwrConfAfterSuspend = read_reg(BMI270_REG_PWR_CONF);

    write_reg(BMI270_REG_INIT_CTRL, 0x00U);
    delay_ms(1U);
    BMI270_Debug.InitCtrlBeforeLoad = read_reg(BMI270_REG_INIT_CTRL);
}

static uint8_t capture_config_status(uint8_t error)
{
    BMI270Sensor.InternalStatus = read_reg(BMI270_REG_INTERNAL_STATUS);
    BMI270_Debug.InternalStatus = BMI270Sensor.InternalStatus;
    BMI270_Debug.ErrorReg = read_reg(BMI270_REG_ERR_REG);
    dump_registers(BMI270_RegDumpAfterConfig);

    if ((BMI270Sensor.InternalStatus & BMI270_INIT_STATUS_MASK) != BMI270_INIT_OK) {
        error |= BMI270_CONFIG_LOAD_ERROR;
    }

    return error;
}

static void wait_bmi270_config_ready(void)
{
    for (uint8_t i = 0U; i < BMI270_CONFIG_STATUS_POLLS; i++) {
        delay_ms(BMI270_CONFIG_STATUS_DELAY);
        if ((read_reg(BMI270_REG_INTERNAL_STATUS) & BMI270_INIT_STATUS_MASK) == BMI270_INIT_OK) {
            break;
        }
    }
}

uint8_t BMI270_ReadChipId(void)
{
    return read_reg(BMI270_REG_CHIP_ID);
}

uint8_t BMI270_ReadInternalStatus(void)
{
    return read_reg(BMI270_REG_INTERNAL_STATUS);
}

void BMI270_UpdateDebugSnapshot(void)
{
    BMI270_Debug.InternalStatus = read_reg(BMI270_REG_INTERNAL_STATUS);
    BMI270_Debug.ErrorReg = read_reg(BMI270_REG_ERR_REG);
    BMI270_Debug.PwrCtrlAfterEnable = read_reg(BMI270_REG_PWR_CTRL);
    BMI270_Debug.AccConfAfterWrite = read_reg(BMI270_REG_ACC_CONF);
    BMI270_Debug.AccRangeAfterWrite = read_reg(BMI270_REG_ACC_RANGE);
    BMI270_Debug.GyrConfAfterWrite = read_reg(BMI270_REG_GYR_CONF);
    BMI270_Debug.GyrRangeAfterWrite = read_reg(BMI270_REG_GYR_RANGE);
    BMI270_Debug.StatusAfterEnable = read_reg(BMI270_REG_STATUS);
}

static uint8_t init_bmi220(uint8_t error)
{
    BMI270_Debug.CompatVariant = BMI270_DEBUG_VARIANT_BMI220;

    enter_config_load_mode();
    upload_config_file(gBmi220ConfigFile, BMI220_CONFIG_FILE_SIZE);
    write_reg(BMI270_REG_INIT_CTRL, 0x01U);
    delay_ms(BMI220_CONFIG_LOAD_DELAY);
    BMI270_Debug.InitCtrlAfterLoad = read_reg(BMI270_REG_INIT_CTRL);
    error = capture_config_status(error);

    write_reg(BMI270_REG_PWR_CONF, BMI220_PWR_CONF_FIFO_WAKE);
    delay_ms(1U);
    write_reg(BMI270_REG_GYR_CONF, BMI270_GYR_CONF_3200HZ_PERF);
    delay_ms(1U);
    BMI270_Debug.GyrConfAfterWrite = read_reg(BMI270_REG_GYR_CONF);
    write_reg(BMI270_REG_GYR_RANGE, BMI270_GYR_RANGE_2000DPS);
    delay_ms(1U);
    BMI270_Debug.GyrRangeAfterWrite = read_reg(BMI270_REG_GYR_RANGE);
    write_reg(BMI270_REG_ACC_CONF, 0xA8U);
    delay_ms(1U);
    BMI270_Debug.AccConfAfterWrite = read_reg(BMI270_REG_ACC_CONF);
    write_reg(BMI270_REG_ACC_RANGE, BMI270_ACC_RANGE_8G);
    delay_ms(1U);
    BMI270_Debug.AccRangeAfterWrite = read_reg(BMI270_REG_ACC_RANGE);
    write_reg(BMI270_REG_PWR_CTRL, BMI220_PWR_CTRL_ALL_ON);
    delay_ms(100U);

    BMI270_Debug.PwrCtrlAfterEnable = read_reg(BMI270_REG_PWR_CTRL);
    BMI270_Debug.StatusAfterEnable = read_reg(BMI270_REG_STATUS);
    BMI270_Debug.CompatDataNonZero = 0U;

    if ((BMI270_Debug.PwrCtrlAfterEnable & BMI220_PWR_CTRL_ALL_ON) != BMI220_PWR_CTRL_ALL_ON) {
        error |= BMI270_POWER_CONFIG_ERROR;
    }
    if (((BMI270_Debug.AccRangeAfterWrite & 0x03U) != BMI270_ACC_RANGE_8G) ||
        ((BMI270_Debug.GyrRangeAfterWrite & 0x07U) != BMI270_GYR_RANGE_2000DPS)) {
        error |= BMI270_SENSOR_CONFIG_ERROR;
    }

    dump_registers(BMI270_RegDumpAfterPower);
    calibrate_gyro_offset();
    BMI270Sensor.InitError = error;
    return error;
}

static uint8_t init_bmi270(uint8_t error)
{
    enter_config_load_mode();
    upload_config_file(gBmi270ConfigFile, BMI270_CONFIG_FILE_SIZE);
    write_reg(BMI270_REG_INIT_CTRL, 0x01U);
    delay_ms(1U);
    BMI270_Debug.InitCtrlAfterLoad = read_reg(BMI270_REG_INIT_CTRL);
    wait_bmi270_config_ready();
    error = capture_config_status(error);

    write_reg(BMI270_REG_PWR_CONF, 0x01U);
    delay_ms(1U);
    write_reg(BMI270_REG_ACC_CONF, BMI270_ACC_CONF_200HZ_PERF);
    delay_ms(1U);
    BMI270_Debug.AccConfAfterWrite = read_reg(BMI270_REG_ACC_CONF);
    write_reg(BMI270_REG_ACC_RANGE, BMI270_ACC_RANGE_8G);
    delay_ms(1U);
    BMI270_Debug.AccRangeAfterWrite = read_reg(BMI270_REG_ACC_RANGE);
    write_reg(BMI270_REG_GYR_CONF, BMI270_GYR_CONF_3200HZ_PERF);
    delay_ms(1U);
    BMI270_Debug.GyrConfAfterWrite = read_reg(BMI270_REG_GYR_CONF);
    write_reg(BMI270_REG_GYR_RANGE, BMI270_GYR_RANGE_2000DPS);
    delay_ms(1U);
    BMI270_Debug.GyrRangeAfterWrite = read_reg(BMI270_REG_GYR_RANGE);
    write_reg(BMI270_REG_PWR_CTRL, BMI270_PWR_CTRL_ACC_GYR);
    delay_ms(50U);

    BMI270_Debug.PwrCtrlAfterEnable = read_reg(BMI270_REG_PWR_CTRL);
    if ((BMI270_Debug.PwrCtrlAfterEnable & BMI270_PWR_CTRL_ACC_GYR) != BMI270_PWR_CTRL_ACC_GYR) {
        error |= BMI270_POWER_CONFIG_ERROR;
    }
    if ((BMI270_Debug.AccRangeAfterWrite & 0x03U) != BMI270_ACC_RANGE_8G) {
        error |= BMI270_SENSOR_CONFIG_ERROR;
    }
    dump_registers(BMI270_RegDumpAfterPower);
    calibrate_gyro_offset();

    BMI270Sensor.InitError = error;
    return error;
}

uint8_t BMI270_Init(void)
{
    uint8_t error = BMI270_NO_ERROR;
    bool chip_id_valid_before_reset;
    bool chip_id_valid_after_reset;

    clear_debug_state();

    DL_GPIO_setPins(BMI270_PORT, BMI270_CS_PIN);
    delay_ms(3U);

    chip_id_valid_before_reset = read_chip_id_retry(&BMI270Sensor.ChipId);
    BMI270_Debug.ChipIdBeforeReset = BMI270Sensor.ChipId;

    write_reg(BMI270_REG_CMD, BMI270_CMD_SOFT_RESET);
    delay_ms(BMI270_SOFT_RESET_DELAY);

    chip_id_valid_after_reset = read_chip_id_retry(&BMI270Sensor.ChipId);
    BMI270_Debug.ChipIdAfterReset = BMI270Sensor.ChipId;
    dump_registers(BMI270_RegDumpAfterReset);
    if ((!chip_id_valid_before_reset) && (!chip_id_valid_after_reset)) {
        error |= BMI270_NO_SENSOR;
    }

    if (BMI270Sensor.ChipId == BMI220_CHIP_ID_VALUE) {
        return init_bmi220(error);
    }

    return init_bmi270(error);
}

void BMI270_Read(BMI270_Data_t *BMI270Sensor)
{
    uint8_t buf[12];
    int16_t raw;
    uint8_t gyro_saturated = 0U;
    uint8_t accel_saturated = 0U;

    read_regs(BMI270_REG_ACC_X_LSB, buf, sizeof(buf));

    raw = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8U));
    if ((raw >= BMI270_RAW_SATURATION_LIMIT) || (raw <= -BMI270_RAW_SATURATION_LIMIT)) {
        accel_saturated = 1U;
    }
    BMI270Sensor->Accel[0] = (float)raw * BMI270_ACCEL_8G_SEN;
    raw = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8U));
    if ((raw >= BMI270_RAW_SATURATION_LIMIT) || (raw <= -BMI270_RAW_SATURATION_LIMIT)) {
        accel_saturated = 1U;
    }
    BMI270Sensor->Accel[1] = (float)raw * BMI270_ACCEL_8G_SEN;
    raw = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8U));
    if ((raw >= BMI270_RAW_SATURATION_LIMIT) || (raw <= -BMI270_RAW_SATURATION_LIMIT)) {
        accel_saturated = 1U;
    }
    BMI270Sensor->Accel[2] = (float)raw * BMI270_ACCEL_8G_SEN;

    raw = (int16_t)((uint16_t)buf[6] | ((uint16_t)buf[7] << 8U));
    if ((raw >= BMI270_RAW_SATURATION_LIMIT) || (raw <= -BMI270_RAW_SATURATION_LIMIT)) {
        gyro_saturated = 1U;
    }
    BMI270Sensor->Gyro[0] = ((float)raw * BMI270_GYRO_2000_SEN) - BMI270Sensor->GyroOffset[0];
    raw = (int16_t)((uint16_t)buf[8] | ((uint16_t)buf[9] << 8U));
    if ((raw >= BMI270_RAW_SATURATION_LIMIT) || (raw <= -BMI270_RAW_SATURATION_LIMIT)) {
        gyro_saturated = 1U;
    }
    BMI270Sensor->Gyro[1] = ((float)raw * BMI270_GYRO_2000_SEN) - BMI270Sensor->GyroOffset[1];
    raw = (int16_t)((uint16_t)buf[10] | ((uint16_t)buf[11] << 8U));
    if ((raw >= BMI270_RAW_SATURATION_LIMIT) || (raw <= -BMI270_RAW_SATURATION_LIMIT)) {
        gyro_saturated = 1U;
    }
    BMI270Sensor->Gyro[2] = ((float)raw * BMI270_GYRO_2000_SEN) - BMI270Sensor->GyroOffset[2];

    BMI270Sensor->LastGyroSaturated = gyro_saturated;
    BMI270Sensor->LastAccelSaturated = accel_saturated;
    if (gyro_saturated != 0U) {
        BMI270Sensor->GyroSaturationCount++;
    }
    if (accel_saturated != 0U) {
        BMI270Sensor->AccelSaturationCount++;
    }

    (void) raw;
}

float BMI270_ReadTemperature(void)
{
    uint8_t buf[2];
    int16_t raw;

    read_regs(BMI270_REG_TEMPERATURE_0, buf, sizeof(buf));
    raw = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8U));
    BMI270Sensor.Temperature = 23.0f + ((float)raw / 512.0f);
    return BMI270Sensor.Temperature;
}

static void calibrate_gyro_offset(void)
{
    uint8_t buf[12];
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};

    for (uint16_t sample = 0U; sample < BMI270_GYRO_OFFSET_SAMPLES; sample++) {
        int16_t raw;

        read_regs(BMI270_REG_ACC_X_LSB, buf, sizeof(buf));
        raw = (int16_t)((uint16_t)buf[6] | ((uint16_t)buf[7] << 8U));
        gyro_sum[0] += (float)raw * BMI270_GYRO_2000_SEN;
        raw = (int16_t)((uint16_t)buf[8] | ((uint16_t)buf[9] << 8U));
        gyro_sum[1] += (float)raw * BMI270_GYRO_2000_SEN;
        raw = (int16_t)((uint16_t)buf[10] | ((uint16_t)buf[11] << 8U));
        gyro_sum[2] += (float)raw * BMI270_GYRO_2000_SEN;
        delay_ms(1U);
    }

    for (uint8_t axis = 0U; axis < 3U; axis++) {
        BMI270Sensor.GyroOffset[axis] = gyro_sum[axis] / (float)BMI270_GYRO_OFFSET_SAMPLES;
    }
}
