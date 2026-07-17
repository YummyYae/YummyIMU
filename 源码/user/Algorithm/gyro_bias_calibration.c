#include "gyro_bias_calibration.h"

#include "bmi088_mspm0.h"
#include "bmi270_mspm0.h"
#include "board_mspm0.h"

#include <stddef.h>

#define GYRO_BIAS_SAMPLE_MS       1U
#define GYRO_BIAS_MAX_TIME_MS     600000U
#define GYRO_BIAS_REASONABLE_RAD  0.35f
#define GYRO_SCORE_BIAS_WEIGHT       0.20f
#define GYRO_SCORE_VARIANCE_WEIGHT   0.80f
#define GYRO_SCORE_SENSITIVITY       0.50f

/*
 * 四组实测样本的逐轴均值。BMI270 通道同时覆盖当前兼容的 BMI260/BMI270/BMI220。
 * 当某轴的零漂绝对值和方差均等于对应参考值时，该轴得到 20/30 分。
 */
static const float gBmi088BiasReference[3] = {
    0.00165325f, 0.00495300f, 0.00347275f
};
static const float gBmi088VarianceReference[3] = {
    0.000165750f, 0.000200750f, 0.000190750f
};
static const float gBmi270BiasReference[3] = {
    0.00255475f, 0.00333750f, 0.00275350f
};
static const float gBmi270VarianceReference[3] = {
    0.000036200f, 0.000036075f, 0.000027700f
};

/*
 * 本模块负责陀螺仪静态质量评估：等待阶段丢弃数据，记录阶段统计两颗传感器的三轴均值与方差。
 * 采样前临时清零旧 GyroOffset，因此这里的均值是未扣零漂的原始角速度，单位为 rad/s。
 * 每颗 IMU 的三个轴分别按 0-30 分评价，单颗最高 90 分，双 IMU 合计最高 180 分。
 * 零漂占 20%，噪声方差占 80%；综合劣化量平方后再映射分数，以提高对样本差异的敏感性。
 * 加速度偏置和比例必须由独立六面标定求解，避免用单一静止姿态错误估计三轴参数。
 */

static uint8_t gyro_bias_values_are_reasonable(const GyroBiasData_t *bias,
                                               uint8_t sensor_mask)
{
    for (uint8_t axis = 0U; axis < 3U; axis++) {
        if (((sensor_mask & IMU_BIAS_SENSOR_BMI088) != 0U) &&
            ((bias->bmi088[axis] != bias->bmi088[axis]) ||
             (bias->bmi088[axis] < -GYRO_BIAS_REASONABLE_RAD) ||
             (bias->bmi088[axis] > GYRO_BIAS_REASONABLE_RAD))) {
            return 0U;
        }
        if (((sensor_mask & IMU_BIAS_SENSOR_BMI270) != 0U) &&
            ((bias->bmi270[axis] != bias->bmi270[axis]) ||
             (bias->bmi270[axis] < -GYRO_BIAS_REASONABLE_RAD) ||
             (bias->bmi270[axis] > GYRO_BIAS_REASONABLE_RAD))) {
            return 0U;
        }
    }

    return 1U;
}

static float gyro_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t gyro_axis_quality_score(float bias,
                                       float variance,
                                       float bias_reference,
                                       float variance_reference)
{
    float normalized_quality;
    float score;

    if ((bias != bias) || (variance != variance) || (variance < 0.0f) ||
        (bias_reference <= 0.0f) || (variance_reference <= 0.0f)) {
        return 0U;
    }

    normalized_quality =
        (GYRO_SCORE_BIAS_WEIGHT * gyro_abs(bias) / bias_reference) +
        (GYRO_SCORE_VARIANCE_WEIGHT * variance / variance_reference);
    score = (float)RUNTIME_GYRO_AXIS_SCORE_MAX /
        (1.0f + (GYRO_SCORE_SENSITIVITY * normalized_quality * normalized_quality));

    if ((score != score) || (score <= 0.0f)) {
        return 0U;
    }
    if (score >= (float)RUNTIME_GYRO_AXIS_SCORE_MAX) {
        return RUNTIME_GYRO_AXIS_SCORE_MAX;
    }

    return (uint8_t)(score + 0.5f);
}

static uint8_t gyro_quality_score(const float bias[3],
                                  const float variance[3],
                                  const float bias_reference[3],
                                  const float variance_reference[3])
{
    uint16_t total_score = 0U;

    for (uint8_t axis = 0U; axis < 3U; axis++) {
        total_score += gyro_axis_quality_score(bias[axis], variance[axis],
                                                bias_reference[axis],
                                                variance_reference[axis]);
    }

    if (total_score > RUNTIME_GYRO_SENSOR_SCORE_MAX) {
        total_score = RUNTIME_GYRO_SENSOR_SCORE_MAX;
    }
    return (uint8_t)total_score;
}

uint8_t GyroBias_RecalculateScores(RuntimeConfig_t *config)
{
    uint8_t changed = 0U;
    float variance[3];

    if ((config == NULL) || (config->gyro_bias_valid == 0U)) {
        return 0U;
    }

    if ((config->gyro_calibrated_mask & IMU_BIAS_SENSOR_BMI088) != 0U) {
        if ((config->bmi088_gyro_score != RUNTIME_GYRO_SCORE_INVALID) ||
            (config->gyro_variance.bmi088[0] != 0U) ||
            (config->gyro_variance.bmi088[1] != 0U) ||
            (config->gyro_variance.bmi088[2] != 0U)) {
            for (uint8_t axis = 0U; axis < 3U; axis++) {
                variance[axis] = RuntimeConfig_DecodeGyroVariance(
                    config->gyro_variance.bmi088[axis]);
            }
            uint8_t score = gyro_quality_score(config->gyro_bias.bmi088, variance,
                                                gBmi088BiasReference,
                                                gBmi088VarianceReference);
            if (config->bmi088_gyro_score != score) {
                config->bmi088_gyro_score = score;
                changed = 1U;
            }
        }
    }

    if ((config->gyro_calibrated_mask & IMU_BIAS_SENSOR_BMI270) != 0U) {
        if ((config->bmi270_gyro_score != RUNTIME_GYRO_SCORE_INVALID) ||
            (config->gyro_variance.bmi270[0] != 0U) ||
            (config->gyro_variance.bmi270[1] != 0U) ||
            (config->gyro_variance.bmi270[2] != 0U)) {
            for (uint8_t axis = 0U; axis < 3U; axis++) {
                variance[axis] = RuntimeConfig_DecodeGyroVariance(
                    config->gyro_variance.bmi270[axis]);
            }
            uint8_t score = gyro_quality_score(config->gyro_bias.bmi270, variance,
                                                gBmi270BiasReference,
                                                gBmi270VarianceReference);
            if (config->bmi270_gyro_score != score) {
                config->bmi270_gyro_score = score;
                changed = 1U;
            }
        }
    }

    return changed;
}

void GyroBias_Apply(const GyroBiasData_t *bias)
{
    for (uint8_t axis = 0U; axis < 3U; axis++) {
        BMI088Sensor.GyroOffset[axis] = bias->bmi088[axis];
        BMI270Sensor.GyroOffset[axis] = bias->bmi270[axis];
    }
}

void GyroBias_StoreQuality(RuntimeConfig_t *config,
                           const GyroQualityData_t *quality,
                           uint8_t sensor_mask)
{
    if ((config == NULL) || (quality == NULL)) {
        return;
    }

    sensor_mask &= IMU_BIAS_SENSOR_DUAL;
    for (uint8_t axis = 0U; axis < 3U; axis++) {
        if ((sensor_mask & IMU_BIAS_SENSOR_BMI088) != 0U) {
            config->gyro_variance.bmi088[axis] =
                RuntimeConfig_EncodeGyroVariance(quality->bmi088_variance[axis]);
        }
        if ((sensor_mask & IMU_BIAS_SENSOR_BMI270) != 0U) {
            config->gyro_variance.bmi270[axis] =
                RuntimeConfig_EncodeGyroVariance(quality->bmi270_variance[axis]);
        }
    }
    if ((sensor_mask & IMU_BIAS_SENSOR_BMI088) != 0U) {
        config->bmi088_gyro_score = quality->bmi088_score;
    }
    if ((sensor_mask & IMU_BIAS_SENSOR_BMI270) != 0U) {
        config->bmi270_gyro_score = quality->bmi270_score;
    }
}

uint8_t GyroBias_CalibrateWithService(GyroBiasData_t *bias,
                                      GyroQualityData_t *quality,
                                      uint8_t sensor_mask,
                                      uint32_t wait_ms,
                                      uint32_t record_ms,
                                      GyroBiasServiceHook_t service,
                                      void *service_context,
                                      GyroBiasProgressHook_t progress)
{
    double bmi088_sum[3] = {0.0, 0.0, 0.0};
    double bmi270_sum[3] = {0.0, 0.0, 0.0};
    double bmi088_sum_square[3] = {0.0, 0.0, 0.0};
    double bmi270_sum_square[3] = {0.0, 0.0, 0.0};
    GyroBiasData_t calibrated_gyro_bias;
    GyroQualityData_t calibrated_quality = {0};
    float previous_bmi088_gyro_bias[3];
    float previous_bmi270_gyro_bias[3];
    uint32_t accepted_samples = 0U;
    uint32_t total_ms = wait_ms + record_ms;
    uint32_t start_tick;
    uint32_t last_sample_tick;
    uint32_t elapsed;

    sensor_mask &= IMU_BIAS_SENSOR_DUAL;
    if ((bias == NULL) || (quality == NULL) || (sensor_mask == 0U) ||
        (wait_ms > GYRO_BIAS_MAX_TIME_MS) ||
        (record_ms == 0U) ||
        (record_ms > GYRO_BIAS_MAX_TIME_MS) ||
        (total_ms < wait_ms)) {
        return 0U;
    }

    calibrated_gyro_bias = *bias;

    for (uint8_t axis = 0U; axis < 3U; axis++) {
        previous_bmi088_gyro_bias[axis] = BMI088Sensor.GyroOffset[axis];
        previous_bmi270_gyro_bias[axis] = BMI270Sensor.GyroOffset[axis];
        BMI088Sensor.GyroOffset[axis] = 0.0f;
        BMI270Sensor.GyroOffset[axis] = 0.0f;
    }

    /*
     * 使用真实系统毫秒时钟控制采样。旧实现每轮完成 SPI 读取和温控服务后
     * 又固定延时 1ms，导致这些执行时间被叠加到采样周期和校准总时长中。
     */
    start_tick = Board_GetSystemTickMs();
    last_sample_tick = start_tick - GYRO_BIAS_SAMPLE_MS;
    while (1) {
        uint32_t now_tick = Board_GetSystemTickMs();

        elapsed = now_tick - start_tick;
        if (elapsed >= total_ms) {
            break;
        }
        if ((uint32_t)(now_tick - last_sample_tick) < GYRO_BIAS_SAMPLE_MS) {
            continue;
        }
        last_sample_tick = now_tick;

        if ((sensor_mask & IMU_BIAS_SENSOR_BMI088) != 0U) {
            BMI088_Read(&BMI088Sensor);
        }
        if ((sensor_mask & IMU_BIAS_SENSOR_BMI270) != 0U) {
            BMI270_Read(&BMI270Sensor);
        }

        if (progress != NULL) {
            progress(elapsed, total_ms);
        }

        if (elapsed >= wait_ms) {
            for (uint8_t axis = 0U; axis < 3U; axis++) {
                if ((sensor_mask & IMU_BIAS_SENSOR_BMI088) != 0U) {
                    double sample = (double)BMI088Sensor.Gyro[axis];

                    bmi088_sum[axis] += sample;
                    bmi088_sum_square[axis] += sample * sample;
                }
                if ((sensor_mask & IMU_BIAS_SENSOR_BMI270) != 0U) {
                    double sample = (double)BMI270Sensor.Gyro[axis];

                    bmi270_sum[axis] += sample;
                    bmi270_sum_square[axis] += sample * sample;
                }
            }
            accepted_samples++;
        }

        if (service != NULL) {
            service(service_context);
        }
    }

    if (progress != NULL) {
        progress(total_ms, total_ms);
    }

    if (accepted_samples < 2U) {
        goto calibration_failed;
    }

    for (uint8_t axis = 0U; axis < 3U; axis++) {
        if ((sensor_mask & IMU_BIAS_SENSOR_BMI088) != 0U) {
            double variance_numerator;

            calibrated_gyro_bias.bmi088[axis] =
                (float)(bmi088_sum[axis] / (double)accepted_samples);
            variance_numerator = bmi088_sum_square[axis] -
                ((bmi088_sum[axis] * bmi088_sum[axis]) / (double)accepted_samples);
            if (variance_numerator < 0.0) {
                variance_numerator = 0.0;
            }
            calibrated_quality.bmi088_variance[axis] =
                (float)(variance_numerator / (double)(accepted_samples - 1U));
        }
        if ((sensor_mask & IMU_BIAS_SENSOR_BMI270) != 0U) {
            double variance_numerator;

            calibrated_gyro_bias.bmi270[axis] =
                (float)(bmi270_sum[axis] / (double)accepted_samples);
            variance_numerator = bmi270_sum_square[axis] -
                ((bmi270_sum[axis] * bmi270_sum[axis]) / (double)accepted_samples);
            if (variance_numerator < 0.0) {
                variance_numerator = 0.0;
            }
            calibrated_quality.bmi270_variance[axis] =
                (float)(variance_numerator / (double)(accepted_samples - 1U));
        }
    }

    if (gyro_bias_values_are_reasonable(&calibrated_gyro_bias, sensor_mask) == 0U) {
        goto calibration_failed;
    }
    if ((sensor_mask & IMU_BIAS_SENSOR_BMI088) != 0U) {
        calibrated_quality.bmi088_score =
            gyro_quality_score(calibrated_gyro_bias.bmi088,
                               calibrated_quality.bmi088_variance,
                               gBmi088BiasReference,
                               gBmi088VarianceReference);
    }
    if ((sensor_mask & IMU_BIAS_SENSOR_BMI270) != 0U) {
        calibrated_quality.bmi270_score =
            gyro_quality_score(calibrated_gyro_bias.bmi270,
                               calibrated_quality.bmi270_variance,
                               gBmi270BiasReference,
                               gBmi270VarianceReference);
    }
    *bias = calibrated_gyro_bias;
    *quality = calibrated_quality;
    GyroBias_Apply(bias);
    return 1U;

calibration_failed:
    for (uint8_t axis = 0U; axis < 3U; axis++) {
        BMI088Sensor.GyroOffset[axis] = previous_bmi088_gyro_bias[axis];
        BMI270Sensor.GyroOffset[axis] = previous_bmi270_gyro_bias[axis];
    }
    return 0U;
}

uint8_t GyroBias_Calibrate(GyroBiasData_t *bias,
                           GyroQualityData_t *quality,
                           uint8_t sensor_mask,
                           uint32_t wait_ms,
                           uint32_t record_ms,
                           GyroBiasProgressHook_t progress)
{
    return GyroBias_CalibrateWithService(bias, quality, sensor_mask,
                                         wait_ms, record_ms, NULL, NULL, progress);
}
