#include "runtime_config.h"

#include <stddef.h>

/*
 * 运行配置公共逻辑：
 * 1. 只维护配置默认值、回传频率约束和方差编码。
 * 2. 不访问 Flash，不引用任务状态，也不触碰硬件寄存器。
 * 3. Flash 存储层负责持久化和数据清洗，其他层只依赖这里的稳定数据契约。
 */

uint16_t RuntimeConfig_EncodeGyroVariance(float variance_rad2_s2)
{
    float encoded;

    if ((variance_rad2_s2 != variance_rad2_s2) || (variance_rad2_s2 <= 0.0f)) {
        return 0U;
    }

    encoded = variance_rad2_s2 * RUNTIME_GYRO_VARIANCE_SCALE;
    if (encoded >= 65535.0f) {
        return 0xFFFFU;
    }

    return (uint16_t)(encoded + 0.5f);
}

float RuntimeConfig_DecodeGyroVariance(uint16_t encoded_variance)
{
    return (float)encoded_variance / RUNTIME_GYRO_VARIANCE_SCALE;
}

uint32_t RuntimeConfig_GetReportRateLimit(uint8_t output_mode)
{
    if (output_mode == RUNTIME_OUTPUT_MODE_DEBUG) {
        return RUNTIME_REPORT_RATE_DEBUG_MAX_HZ;
    }
    if (output_mode == RUNTIME_OUTPUT_MODE_BINARY) {
        return RUNTIME_REPORT_RATE_BINARY_MAX_HZ;
    }
#if RUNTIME_FEATURE_INS_ENABLED
    if (output_mode == RUNTIME_OUTPUT_MODE_INS) {
        return RUNTIME_REPORT_RATE_INS_MAX_HZ;
    }
#endif

    return RUNTIME_REPORT_RATE_USE_MAX_HZ;
}

uint8_t RuntimeConfig_ReportRateIsSupported(uint8_t output_mode,
                                            uint32_t report_rate_hz)
{
    uint32_t rate_limit = RuntimeConfig_GetReportRateLimit(output_mode);

    if ((report_rate_hz == 0U) || (report_rate_hz > rate_limit)) {
        return 0U;
    }

    return ((RUNTIME_REPORT_RATE_BASE_HZ % report_rate_hz) == 0U) ? 1U : 0U;
}

void RuntimeConfig_Default(RuntimeConfig_t *config)
{
    if (config == NULL) {
        return;
    }

    config->baud_rate = RUNTIME_CONFIG_DEFAULT_BAUD_RATE;
    config->report_rate_hz = RUNTIME_CONFIG_DEFAULT_REPORT_RATE_HZ;
    config->target_temperature_c = RUNTIME_CONFIG_DEFAULT_TARGET_TEMP_C;
    config->gyro_bias_valid = 0U;
    config->accel_bias_valid = 0U;
    config->gyro_calibrated_mask = 0U;
    config->accel_calibrated_mask = 0U;
    config->output_mode = RUNTIME_CONFIG_DEFAULT_OUTPUT_MODE;
    config->imu_source = RUNTIME_CONFIG_DEFAULT_IMU_SOURCE;
    config->bmi088_gyro_score = RUNTIME_GYRO_SCORE_INVALID;
    config->bmi270_gyro_score = RUNTIME_GYRO_SCORE_INVALID;
    config->bmi088_yaw_error_deg_per_turn = RUNTIME_CONFIG_DEFAULT_YAW_ERR_DEG;
    config->bmi270_yaw_error_deg_per_turn = RUNTIME_CONFIG_DEFAULT_YAW_ERR_DEG;

    for (uint8_t axis = 0U; axis < 3U; axis++) {
        config->gyro_bias.bmi088[axis] = 0.0f;
        config->gyro_bias.bmi270[axis] = 0.0f;
        config->gyro_variance.bmi088[axis] = 0U;
        config->gyro_variance.bmi270[axis] = 0U;
        config->accel_bias.bmi088[axis] = 0.0f;
        config->accel_bias.bmi270[axis] = 0.0f;
        config->accel_scale.bmi088[axis] = 1.0f;
        config->accel_scale.bmi270[axis] = 1.0f;
    }
}
