#include "flash_storage.h"

#include "ti_msp_dl_config.h"

#include <stddef.h>
#include <stdint.h>

#define CONFIG_FLASH_ADDRESS 0x0001FC00UL
#define CONFIG_FLASH_SIZE    1024U
#define CONFIG_SLOT_SIZE     128U
#define CONFIG_SLOT_COUNT    (CONFIG_FLASH_SIZE / CONFIG_SLOT_SIZE)
#define CONFIG_SLOT_WORDS    (CONFIG_SLOT_SIZE / 4U)
#define CONFIG_MAGIC         0x30434647UL
#define CONFIG_VERSION       8U

#define GYRO_BIAS_LIMIT_RADPS        0.35f
#define ACCEL_BIAS_LIMIT_MPS2        3.0f
#define ACCEL_SCALE_MIN              0.8f
#define ACCEL_SCALE_MAX              1.2f
#define YAW_ERROR_LIMIT_DEG_PER_TURN 30.0f
/* 兼容旧版 0-100 分记录；加载后会按新 0-90 标准自动重算并保存。 */
#define GYRO_SCORE_LEGACY_MAX        100U

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t length;
    uint32_t sequence;
    RuntimeConfig_t config;
    uint32_t crc32;
} ConfigRecord_t;

/* Flash 槽与记录必须严格等长，避免结构体变化后遗留字节或跨槽写入。 */
typedef char ConfigRecordSizeCheck[(sizeof(ConfigRecord_t) == CONFIG_SLOT_SIZE) ? 1 : -1];

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
    return crc32_update(0xFFFFFFFFUL,
                        (const uint8_t *)record,
                        offsetof(ConfigRecord_t, crc32)) ^ 0xFFFFFFFFUL;
}

static uint8_t gyro_bias_values_are_reasonable(const GyroBiasData_t *bias)
{
    for (uint8_t axis = 0U; axis < 3U; axis++) {
        if ((bias->bmi088[axis] != bias->bmi088[axis]) ||
            (bias->bmi270[axis] != bias->bmi270[axis]) ||
            (bias->bmi088[axis] < -GYRO_BIAS_LIMIT_RADPS) ||
            (bias->bmi088[axis] > GYRO_BIAS_LIMIT_RADPS) ||
            (bias->bmi270[axis] < -GYRO_BIAS_LIMIT_RADPS) ||
            (bias->bmi270[axis] > GYRO_BIAS_LIMIT_RADPS)) {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t accel_bias_values_are_reasonable(const AccelBiasData_t *bias)
{
    for (uint8_t axis = 0U; axis < 3U; axis++) {
        if ((bias->bmi088[axis] != bias->bmi088[axis]) ||
            (bias->bmi270[axis] != bias->bmi270[axis]) ||
            (bias->bmi088[axis] < -ACCEL_BIAS_LIMIT_MPS2) ||
            (bias->bmi088[axis] > ACCEL_BIAS_LIMIT_MPS2) ||
            (bias->bmi270[axis] < -ACCEL_BIAS_LIMIT_MPS2) ||
            (bias->bmi270[axis] > ACCEL_BIAS_LIMIT_MPS2)) {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t accel_scale_values_are_reasonable(const AccelScaleData_t *scale)
{
    for (uint8_t axis = 0U; axis < 3U; axis++) {
        if ((scale->bmi088[axis] != scale->bmi088[axis]) ||
            (scale->bmi270[axis] != scale->bmi270[axis]) ||
            (scale->bmi088[axis] < ACCEL_SCALE_MIN) ||
            (scale->bmi088[axis] > ACCEL_SCALE_MAX) ||
            (scale->bmi270[axis] < ACCEL_SCALE_MIN) ||
            (scale->bmi270[axis] > ACCEL_SCALE_MAX)) {
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
    if ((output_mode == RUNTIME_OUTPUT_MODE_USE) ||
        (output_mode == RUNTIME_OUTPUT_MODE_DEBUG) ||
        (output_mode == RUNTIME_OUTPUT_MODE_BINARY)) {
        return 1U;
    }

#if RUNTIME_FEATURE_INS_ENABLED
    if (output_mode == RUNTIME_OUTPUT_MODE_INS) {
        return 1U;
    }
#endif

    return 0U;
}

static uint8_t imu_source_is_supported(uint8_t imu_source)
{
    return ((imu_source == RUNTIME_IMU_SOURCE_DUAL) ||
            (imu_source == RUNTIME_IMU_SOURCE_BMI088) ||
            (imu_source == RUNTIME_IMU_SOURCE_BMI270) ||
            (imu_source == RUNTIME_IMU_SOURCE_AUTO));
}

static uint8_t yaw_error_is_supported(float error_deg_per_turn)
{
    return ((error_deg_per_turn == error_deg_per_turn) &&
            (error_deg_per_turn >= -YAW_ERROR_LIMIT_DEG_PER_TURN) &&
            (error_deg_per_turn <= YAW_ERROR_LIMIT_DEG_PER_TURN)) ? 1U : 0U;
}

static void config_sanitize(RuntimeConfig_t *config)
{
    uint32_t rate_limit;

    if (baud_rate_is_supported(config->baud_rate) == 0U) {
        config->baud_rate = RUNTIME_CONFIG_DEFAULT_BAUD_RATE;
    }

    if ((config->target_temperature_c != config->target_temperature_c) ||
        (config->target_temperature_c < 20.0f) ||
        (config->target_temperature_c > 85.0f)) {
        config->target_temperature_c = RUNTIME_CONFIG_DEFAULT_TARGET_TEMP_C;
    }

    if (output_mode_is_supported(config->output_mode) == 0U) {
        config->output_mode = RUNTIME_CONFIG_DEFAULT_OUTPUT_MODE;
    }

    rate_limit = RuntimeConfig_GetReportRateLimit(config->output_mode);
    if ((config->report_rate_hz == 0U) ||
        ((RUNTIME_REPORT_RATE_BASE_HZ % config->report_rate_hz) != 0U)) {
        config->report_rate_hz = RUNTIME_CONFIG_DEFAULT_REPORT_RATE_HZ;
    } else if (config->report_rate_hz > rate_limit) {
        config->report_rate_hz = rate_limit;
    }

    if (imu_source_is_supported(config->imu_source) == 0U) {
        config->imu_source = RUNTIME_CONFIG_DEFAULT_IMU_SOURCE;
    }

    if (yaw_error_is_supported(config->bmi088_yaw_error_deg_per_turn) == 0U) {
        config->bmi088_yaw_error_deg_per_turn = RUNTIME_CONFIG_DEFAULT_YAW_ERR_DEG;
    }
    if (yaw_error_is_supported(config->bmi270_yaw_error_deg_per_turn) == 0U) {
        config->bmi270_yaw_error_deg_per_turn = RUNTIME_CONFIG_DEFAULT_YAW_ERR_DEG;
    }

    config->gyro_calibrated_mask &= RUNTIME_GYRO_CAL_DUAL;
    if ((config->gyro_bias_valid == 0U) ||
        (config->gyro_calibrated_mask == 0U) ||
        (gyro_bias_values_are_reasonable(&config->gyro_bias) == 0U)) {
        config->gyro_bias_valid = 0U;
        config->gyro_calibrated_mask = 0U;
        config->bmi088_gyro_score = RUNTIME_GYRO_SCORE_INVALID;
        config->bmi270_gyro_score = RUNTIME_GYRO_SCORE_INVALID;
        for (uint8_t axis = 0U; axis < 3U; axis++) {
            config->gyro_bias.bmi088[axis] = 0.0f;
            config->gyro_bias.bmi270[axis] = 0.0f;
            config->gyro_variance.bmi088[axis] = 0U;
            config->gyro_variance.bmi270[axis] = 0U;
        }
    } else {
        config->gyro_bias_valid = 1U;
        if (((config->gyro_calibrated_mask & RUNTIME_GYRO_CAL_BMI088) == 0U) ||
            (config->bmi088_gyro_score > GYRO_SCORE_LEGACY_MAX)) {
            config->bmi088_gyro_score = RUNTIME_GYRO_SCORE_INVALID;
            for (uint8_t axis = 0U; axis < 3U; axis++) {
                config->gyro_variance.bmi088[axis] = 0U;
            }
        }
        if (((config->gyro_calibrated_mask & RUNTIME_GYRO_CAL_BMI270) == 0U) ||
            (config->bmi270_gyro_score > GYRO_SCORE_LEGACY_MAX)) {
            config->bmi270_gyro_score = RUNTIME_GYRO_SCORE_INVALID;
            for (uint8_t axis = 0U; axis < 3U; axis++) {
                config->gyro_variance.bmi270[axis] = 0U;
            }
        }
    }

    config->accel_calibrated_mask &= RUNTIME_ACCEL_CAL_DUAL;
    if ((config->accel_bias_valid == 0U) ||
        (config->accel_calibrated_mask == 0U) ||
        (accel_bias_values_are_reasonable(&config->accel_bias) == 0U) ||
        (accel_scale_values_are_reasonable(&config->accel_scale) == 0U)) {
        config->accel_bias_valid = 0U;
        config->accel_calibrated_mask = 0U;
        for (uint8_t axis = 0U; axis < 3U; axis++) {
            config->accel_bias.bmi088[axis] = 0.0f;
            config->accel_bias.bmi270[axis] = 0.0f;
            config->accel_scale.bmi088[axis] = 1.0f;
            config->accel_scale.bmi270[axis] = 1.0f;
        }
    } else {
        config->accel_bias_valid = 1U;
    }
}

static const ConfigRecord_t *slot_record(uint32_t slot)
{
    return (const ConfigRecord_t *)(CONFIG_FLASH_ADDRESS + (slot * CONFIG_SLOT_SIZE));
}

static uint8_t record_is_valid(const ConfigRecord_t *record)
{
    if ((record->magic != CONFIG_MAGIC) ||
        (record->version != CONFIG_VERSION) ||
        (record->length != sizeof(ConfigRecord_t))) {
        return 0U;
    }

    return (record->crc32 == record_crc(record)) ? 1U : 0U;
}

static uint8_t slot_is_blank(uint32_t slot)
{
    const uint32_t *words =
        (const uint32_t *)(CONFIG_FLASH_ADDRESS + (slot * CONFIG_SLOT_SIZE));

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
            if ((*found == 0U) ||
                ((int32_t)(record->sequence - latest_sequence) > 0)) {
                latest_slot = slot;
                latest_sequence = record->sequence;
                *found = 1U;
            }
        }
    }

    return latest_slot;
}

uint8_t RuntimeConfig_Load(RuntimeConfig_t *config)
{
    uint8_t found;
    uint32_t slot;

    if (config == NULL) {
        return 0U;
    }

    RuntimeConfig_Default(config);
    slot = latest_valid_slot(&found);
    if (found == 0U) {
        return 0U;
    }

    *config = slot_record(slot)->config;
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

    if (config == NULL) {
        return 0U;
    }

    sanitized = *config;
    config_sanitize(&sanitized);

    latest_slot = latest_valid_slot(&found);
    latest_sequence = (found != 0U) ? slot_record(latest_slot)->sequence : 0U;
    next_slot = (found != 0U) ? (latest_slot + 1U) : 0U;
    if ((next_slot >= CONFIG_SLOT_COUNT) || (slot_is_blank(next_slot) == 0U)) {
        DL_FlashCTL_unprotectSector(FLASHCTL,
                                   CONFIG_FLASH_ADDRESS,
                                   DL_FLASHCTL_REGION_SELECT_MAIN);
        __disable_irq();
        status = DL_FlashCTL_eraseMemoryFromRAM(
            FLASHCTL, CONFIG_FLASH_ADDRESS, DL_FLASHCTL_COMMAND_SIZE_SECTOR);
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

    DL_FlashCTL_unprotectSector(FLASHCTL,
                               CONFIG_FLASH_ADDRESS,
                               DL_FLASHCTL_REGION_SELECT_MAIN);
    __disable_irq();
    status = DL_FlashCTL_programMemoryBlockingFromRAM64WithECCGenerated(
        FLASHCTL,
        CONFIG_FLASH_ADDRESS + (next_slot * CONFIG_SLOT_SIZE),
        words,
        CONFIG_SLOT_WORDS,
        DL_FLASHCTL_REGION_SELECT_MAIN);
    __enable_irq();
    DL_FlashCTL_protectMainMemory(FLASHCTL);
    if (status != DL_FLASHCTL_COMMAND_STATUS_PASSED) {
        return 0U;
    }

    return record_is_valid(slot_record(next_slot));
}
