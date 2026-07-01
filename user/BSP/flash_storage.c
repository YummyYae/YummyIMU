#include "flash_storage.h"

#include "bmi088_mspm0.h"
#include "bmi270_mspm0.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>
#include <stdint.h>

#define CONFIG_FLASH_ADDRESS 0x0001FC00UL
#define CONFIG_FLASH_SIZE    1024U
#define CONFIG_SLOT_SIZE     64U
#define CONFIG_SLOT_COUNT    (CONFIG_FLASH_SIZE / CONFIG_SLOT_SIZE)
#define CONFIG_MAGIC         0x30434647UL
#define CONFIG_VERSION       3U
#define CONFIG_VERSION_V2    2U
#define CONFIG_SAMPLE_MS     1U
#define CONFIG_SLOT_WORDS    (CONFIG_SLOT_SIZE / 4U)

extern volatile uint32_t gSystemTickMs;
extern void GyroBias_CalibrationProgress(uint32_t elapsed_ms, uint32_t total_ms);

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t length;
    uint32_t sequence;
    RuntimeConfig_t config;
    uint32_t crc32;
} ConfigRecord_t;

typedef struct {
    uint32_t baud_rate;
    uint32_t report_rate_hz;
    uint32_t cal_wait_ms;
    uint32_t cal_record_ms;
    float target_temperature_c;
    uint8_t gyro_bias_valid;
    uint8_t reserved[3];
    GyroBiasData_t gyro_bias;
} RuntimeConfigV2_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t length;
    uint32_t sequence;
    RuntimeConfigV2_t config;
    uint32_t crc32;
} ConfigRecordV2_t;

typedef char ConfigRecordSizeCheck[(sizeof(ConfigRecord_t) <= CONFIG_SLOT_SIZE) ? 1 : -1];
typedef char ConfigRecordV2SizeCheck[(sizeof(ConfigRecordV2_t) <= CONFIG_SLOT_SIZE) ? 1 : -1];

static void delay_ms(uint32_t ms)
{
    uint32_t start = gSystemTickMs;

    while ((uint32_t)(gSystemTickMs - start) < ms) {
        __WFI();
    }
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    while (len-- != 0U) {
        crc ^= (uint32_t)(*data++);
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((crc & 1U) != 0U) {
                crc = (crc >> 1U) ^ 0xEDB88320UL;
            } else {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static uint32_t record_crc(const ConfigRecord_t *record)
{
    return crc32_update(0xFFFFFFFFUL, (const uint8_t *)record, offsetof(ConfigRecord_t, crc32)) ^ 0xFFFFFFFFUL;
}

static uint32_t record_v2_crc(const ConfigRecordV2_t *record)
{
    return crc32_update(0xFFFFFFFFUL, (const uint8_t *)record, offsetof(ConfigRecordV2_t, crc32)) ^ 0xFFFFFFFFUL;
}

static uint8_t bias_values_are_reasonable(const GyroBiasData_t *bias)
{
    for (uint8_t axis = 0U; axis < 3U; axis++) {
        if ((bias->bmi088[axis] < -0.35f) || (bias->bmi088[axis] > 0.35f) ||
            (bias->bmi270[axis] < -0.35f) || (bias->bmi270[axis] > 0.35f)) {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t baud_rate_is_supported(uint32_t baud_rate)
{
    return ((baud_rate == 115200U) ||
            (baud_rate == 230400U) ||
            (baud_rate == 460800U) ||
            (baud_rate == 921600U));
}

static uint8_t output_mode_is_supported(uint8_t output_mode)
{
    return ((output_mode == RUNTIME_OUTPUT_MODE_USE) ||
            (output_mode == RUNTIME_OUTPUT_MODE_DEBUG));
}

static void config_sanitize(RuntimeConfig_t *config)
{
    if (baud_rate_is_supported(config->baud_rate) == 0U) {
        config->baud_rate = RUNTIME_CONFIG_DEFAULT_BAUD_RATE;
    }

    if ((config->report_rate_hz < 1U) || (config->report_rate_hz > 500U)) {
        config->report_rate_hz = RUNTIME_CONFIG_DEFAULT_REPORT_RATE_HZ;
    }

    if ((config->target_temperature_c < 20.0f) || (config->target_temperature_c > 85.0f)) {
        config->target_temperature_c = RUNTIME_CONFIG_DEFAULT_TARGET_TEMP_C;
    }

    if (output_mode_is_supported(config->output_mode) == 0U) {
        config->output_mode = RUNTIME_CONFIG_DEFAULT_OUTPUT_MODE;
    }

    if ((config->gyro_bias_valid == 0U) || (bias_values_are_reasonable(&config->gyro_bias) == 0U)) {
        config->gyro_bias_valid = 0U;
        for (uint8_t axis = 0U; axis < 3U; axis++) {
            config->gyro_bias.bmi088[axis] = 0.0f;
            config->gyro_bias.bmi270[axis] = 0.0f;
        }
    } else {
        config->gyro_bias_valid = 1U;
    }
}

static const ConfigRecord_t *slot_record(uint32_t slot)
{
    return (const ConfigRecord_t *)(CONFIG_FLASH_ADDRESS + (slot * CONFIG_SLOT_SIZE));
}

static const ConfigRecordV2_t *slot_record_v2(uint32_t slot)
{
    return (const ConfigRecordV2_t *)(CONFIG_FLASH_ADDRESS + (slot * CONFIG_SLOT_SIZE));
}

static uint8_t record_is_valid(const ConfigRecord_t *record)
{
    RuntimeConfig_t config;
    const ConfigRecordV2_t *record_v2;

    if (record->magic != CONFIG_MAGIC) {
        return 0U;
    }

    if (record->version == CONFIG_VERSION) {
        if ((record->length != sizeof(ConfigRecord_t)) ||
            (record->crc32 != record_crc(record))) {
            return 0U;
        }

        config = record->config;
        config_sanitize(&config);
        if ((config.baud_rate != record->config.baud_rate) ||
            (config.report_rate_hz != record->config.report_rate_hz) ||
            (config.target_temperature_c != record->config.target_temperature_c) ||
            (config.gyro_bias_valid != record->config.gyro_bias_valid)) {
            return 0U;
        }

        if ((output_mode_is_supported(record->config.output_mode) != 0U) &&
            (config.output_mode != record->config.output_mode)) {
            return 0U;
        }

        return 1U;
    }

    if (record->version != CONFIG_VERSION_V2) {
        return 0U;
    }

    record_v2 = (const ConfigRecordV2_t *)record;
    if ((record_v2->length != sizeof(ConfigRecordV2_t)) ||
        (record_v2->crc32 != record_v2_crc(record_v2))) {
        return 0U;
    }

    config.baud_rate = record_v2->config.baud_rate;
    config.report_rate_hz = record_v2->config.report_rate_hz;
    config.target_temperature_c = record_v2->config.target_temperature_c;
    config.gyro_bias_valid = record_v2->config.gyro_bias_valid;
    config.output_mode = RUNTIME_CONFIG_DEFAULT_OUTPUT_MODE;
    config.reserved[0] = 0U;
    config.reserved[1] = 0U;
    config.gyro_bias = record_v2->config.gyro_bias;
    config_sanitize(&config);

    return ((config.baud_rate == record_v2->config.baud_rate) &&
            (config.report_rate_hz == record_v2->config.report_rate_hz) &&
            (config.target_temperature_c == record_v2->config.target_temperature_c) &&
            (config.gyro_bias_valid == record_v2->config.gyro_bias_valid)) ? 1U : 0U;
}

static uint8_t slot_is_blank(uint32_t slot)
{
    const uint32_t *words = (const uint32_t *)(CONFIG_FLASH_ADDRESS + (slot * CONFIG_SLOT_SIZE));

    for (uint32_t i = 0U; i < CONFIG_SLOT_WORDS; i++) {
        if (words[i] != 0xFFFFFFFFUL) {
            return 0U;
        }
    }

    return 1U;
}

static uint32_t latest_valid_slot(uint8_t *found)
{
    uint32_t latest_slot = 0U;
    uint32_t latest_sequence = 0U;

    *found = 0U;
    for (uint32_t slot = 0U; slot < CONFIG_SLOT_COUNT; slot++) {
        const ConfigRecord_t *record = slot_record(slot);

        if (record_is_valid(record) != 0U) {
            if ((*found == 0U) || (record->sequence >= latest_sequence)) {
                latest_slot = slot;
                latest_sequence = record->sequence;
                *found = 1U;
            }
        }
    }

    return latest_slot;
}

void RuntimeConfig_Default(RuntimeConfig_t *config)
{
    config->baud_rate = RUNTIME_CONFIG_DEFAULT_BAUD_RATE;
    config->report_rate_hz = RUNTIME_CONFIG_DEFAULT_REPORT_RATE_HZ;
    config->target_temperature_c = RUNTIME_CONFIG_DEFAULT_TARGET_TEMP_C;
    config->gyro_bias_valid = 0U;
    config->output_mode = RUNTIME_CONFIG_DEFAULT_OUTPUT_MODE;
    config->reserved[0] = 0U;
    config->reserved[1] = 0U;

    for (uint8_t axis = 0U; axis < 3U; axis++) {
        config->gyro_bias.bmi088[axis] = 0.0f;
        config->gyro_bias.bmi270[axis] = 0.0f;
    }
}

uint8_t RuntimeConfig_Load(RuntimeConfig_t *config)
{
    uint8_t found;
    uint32_t slot;
    const ConfigRecord_t *record;
    const ConfigRecordV2_t *record_v2;

    RuntimeConfig_Default(config);
    slot = latest_valid_slot(&found);
    if (found == 0U) {
        return 0U;
    }

    record = slot_record(slot);
    if (record->version == CONFIG_VERSION) {
        *config = record->config;
    } else {
        record_v2 = slot_record_v2(slot);
        config->baud_rate = record_v2->config.baud_rate;
        config->report_rate_hz = record_v2->config.report_rate_hz;
        config->target_temperature_c = record_v2->config.target_temperature_c;
        config->gyro_bias_valid = record_v2->config.gyro_bias_valid;
        config->output_mode = RUNTIME_CONFIG_DEFAULT_OUTPUT_MODE;
        config->reserved[0] = 0U;
        config->reserved[1] = 0U;
        config->gyro_bias = record_v2->config.gyro_bias;
    }
    config_sanitize(config);
    return 1U;
}

uint8_t RuntimeConfig_Save(const RuntimeConfig_t *config)
{
    ConfigRecord_t record;
    RuntimeConfig_t sanitized;
    uint32_t words[CONFIG_SLOT_WORDS];
    uint8_t found;
    uint32_t latest_slot;
    uint32_t latest_sequence;
    uint32_t next_slot;
    DL_FLASHCTL_COMMAND_STATUS status;

    sanitized = *config;
    config_sanitize(&sanitized);
    sanitized.reserved[0] = 0U;
    sanitized.reserved[1] = 0U;

    latest_slot = latest_valid_slot(&found);
    latest_sequence = (found != 0U) ? slot_record(latest_slot)->sequence : 0U;
    next_slot = (found != 0U) ? (latest_slot + 1U) : 0U;
    if ((next_slot >= CONFIG_SLOT_COUNT) || (slot_is_blank(next_slot) == 0U)) {
        DL_FlashCTL_unprotectSector(FLASHCTL, CONFIG_FLASH_ADDRESS, DL_FLASHCTL_REGION_SELECT_MAIN);
        __disable_irq();
        status = DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL, CONFIG_FLASH_ADDRESS, DL_FLASHCTL_COMMAND_SIZE_SECTOR);
        __enable_irq();
        DL_FlashCTL_protectMainMemory(FLASHCTL);
        if (status != DL_FLASHCTL_COMMAND_STATUS_PASSED) {
            return 0U;
        }

        next_slot = 0U;
    }

    record.magic = CONFIG_MAGIC;
    record.version = CONFIG_VERSION;
    record.length = sizeof(ConfigRecord_t);
    record.sequence = latest_sequence + 1U;
    record.config = sanitized;
    record.crc32 = record_crc(&record);

    for (uint32_t i = 0U; i < CONFIG_SLOT_WORDS; i++) {
        words[i] = 0xFFFFFFFFUL;
    }
    for (uint32_t i = 0U; i < sizeof(record); i++) {
        ((uint8_t *)words)[i] = ((const uint8_t *)&record)[i];
    }

    DL_FlashCTL_unprotectSector(FLASHCTL, CONFIG_FLASH_ADDRESS, DL_FLASHCTL_REGION_SELECT_MAIN);
    __disable_irq();
    status = DL_FlashCTL_programMemoryBlockingFromRAM64WithECCGenerated(
        FLASHCTL, CONFIG_FLASH_ADDRESS + (next_slot * CONFIG_SLOT_SIZE),
        words, CONFIG_SLOT_WORDS, DL_FLASHCTL_REGION_SELECT_MAIN);
    __enable_irq();
    DL_FlashCTL_protectMainMemory(FLASHCTL);
    if (status != DL_FLASHCTL_COMMAND_STATUS_PASSED) {
        return 0U;
    }

    if (record_is_valid(slot_record(next_slot)) == 0U) {
        return 0U;
    }

    return 1U;
}

void GyroBias_Apply(const GyroBiasData_t *bias)
{
    for (uint8_t axis = 0U; axis < 3U; axis++) {
        BMI088Sensor.GyroOffset[axis] = bias->bmi088[axis];
        BMI270Sensor.GyroOffset[axis] = bias->bmi270[axis];
    }
}

uint8_t GyroBias_Calibrate(GyroBiasData_t *bias, uint32_t wait_ms, uint32_t record_ms)
{
    double bmi088_sum[3] = {0.0, 0.0, 0.0};
    double bmi270_sum[3] = {0.0, 0.0, 0.0};
    uint32_t accepted_samples = 0U;
    uint32_t total_ms = wait_ms + record_ms;

    if ((wait_ms > 600000U) || (record_ms == 0U) || (record_ms > 600000U) || (total_ms < wait_ms)) {
        return 0U;
    }

    for (uint8_t axis = 0U; axis < 3U; axis++) {
        BMI088Sensor.GyroOffset[axis] = 0.0f;
        BMI270Sensor.GyroOffset[axis] = 0.0f;
    }

    for (uint32_t elapsed = 0U; elapsed < total_ms; elapsed += CONFIG_SAMPLE_MS) {
        BMI088_Read(&BMI088Sensor);
        BMI270_Read(&BMI270Sensor);
        GyroBias_CalibrationProgress(elapsed, total_ms);

        if (elapsed >= wait_ms) {
            for (uint8_t axis = 0U; axis < 3U; axis++) {
                bmi088_sum[axis] += (double)BMI088Sensor.Gyro[axis];
                bmi270_sum[axis] += (double)BMI270Sensor.Gyro[axis];
            }
            accepted_samples++;
        }

        delay_ms(CONFIG_SAMPLE_MS);
    }

    GyroBias_CalibrationProgress(total_ms, total_ms);
    if (accepted_samples == 0U) {
        return 0U;
    }

    for (uint8_t axis = 0U; axis < 3U; axis++) {
        bias->bmi088[axis] = (float)(bmi088_sum[axis] / (double)accepted_samples);
        bias->bmi270[axis] = (float)(bmi270_sum[axis] / (double)accepted_samples);
    }

    if (bias_values_are_reasonable(bias) == 0U) {
        return 0U;
    }

    GyroBias_Apply(bias);
    return 1U;
}

