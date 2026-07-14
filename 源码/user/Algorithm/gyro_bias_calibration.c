#include "gyro_bias_calibration.h"

#include "bmi088_mspm0.h"
#include "bmi270_mspm0.h"
#include "board_mspm0.h"

#include <stddef.h>

#define GYRO_BIAS_SAMPLE_MS       1U
#define GYRO_BIAS_MAX_TIME_MS     600000U
#define GYRO_BIAS_REASONABLE_RAD  0.35f

/*
 * 本模块只负责陀螺仪静态零偏：等待阶段丢弃数据，记录阶段平均两颗传感器的三轴角速度。
 * 加速度偏置和比例必须由独立六面标定求解，避免用单一静止姿态错误估计三轴参数。
 */

static uint8_t gyro_bias_values_are_reasonable(const GyroBiasData_t *bias)
{
    for (uint8_t axis = 0U; axis < 3U; axis++) {
        if ((bias->bmi088[axis] != bias->bmi088[axis]) ||
            (bias->bmi270[axis] != bias->bmi270[axis]) ||
            (bias->bmi088[axis] < -GYRO_BIAS_REASONABLE_RAD) ||
            (bias->bmi088[axis] > GYRO_BIAS_REASONABLE_RAD) ||
            (bias->bmi270[axis] < -GYRO_BIAS_REASONABLE_RAD) ||
            (bias->bmi270[axis] > GYRO_BIAS_REASONABLE_RAD)) {
            return 0U;
        }
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

uint8_t GyroBias_CalibrateWithService(GyroBiasData_t *bias,
                                      uint8_t sensor_mask,
                                      uint32_t wait_ms,
                                      uint32_t record_ms,
                                      GyroBiasServiceHook_t service,
                                      void *service_context,
                                      GyroBiasProgressHook_t progress)
{
    double bmi088_sum[3] = {0.0, 0.0, 0.0};
    double bmi270_sum[3] = {0.0, 0.0, 0.0};
    GyroBiasData_t calibrated_gyro_bias;
    float previous_bmi088_gyro_bias[3];
    float previous_bmi270_gyro_bias[3];
    uint32_t accepted_samples = 0U;
    uint32_t total_ms = wait_ms + record_ms;

    sensor_mask &= IMU_BIAS_SENSOR_DUAL;
    if ((bias == NULL) || (sensor_mask == 0U) ||
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

    for (uint32_t elapsed = 0U; elapsed < total_ms; elapsed += GYRO_BIAS_SAMPLE_MS) {
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
                    bmi088_sum[axis] += (double)BMI088Sensor.Gyro[axis];
                }
                if ((sensor_mask & IMU_BIAS_SENSOR_BMI270) != 0U) {
                    bmi270_sum[axis] += (double)BMI270Sensor.Gyro[axis];
                }
            }
            accepted_samples++;
        }

        if (service != NULL) {
            service(service_context);
        }

        Board_DelayMs(GYRO_BIAS_SAMPLE_MS);
    }

    if (progress != NULL) {
        progress(total_ms, total_ms);
    }

    if (accepted_samples == 0U) {
        goto calibration_failed;
    }

    for (uint8_t axis = 0U; axis < 3U; axis++) {
        if ((sensor_mask & IMU_BIAS_SENSOR_BMI088) != 0U) {
            calibrated_gyro_bias.bmi088[axis] =
                (float)(bmi088_sum[axis] / (double)accepted_samples);
        }
        if ((sensor_mask & IMU_BIAS_SENSOR_BMI270) != 0U) {
            calibrated_gyro_bias.bmi270[axis] =
                (float)(bmi270_sum[axis] / (double)accepted_samples);
        }
    }

    if (gyro_bias_values_are_reasonable(&calibrated_gyro_bias) == 0U) {
        goto calibration_failed;
    }
    *bias = calibrated_gyro_bias;
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
                           uint8_t sensor_mask,
                           uint32_t wait_ms,
                           uint32_t record_ms,
                           GyroBiasProgressHook_t progress)
{
    return GyroBias_CalibrateWithService(bias, sensor_mask,
                                         wait_ms, record_ms, NULL, NULL, progress);
}
