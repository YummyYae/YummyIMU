#include "gyro_bias_calibration.h"

#include "bmi088_mspm0.h"
#include "bmi270_mspm0.h"
#include "board_mspm0.h"

#include <stddef.h>

#define GYRO_BIAS_SAMPLE_MS       1U
#define GYRO_BIAS_MAX_TIME_MS     600000U
#define GYRO_BIAS_REASONABLE_RAD  0.35f

static uint8_t gyro_bias_values_are_reasonable(const GyroBiasData_t *bias)
{
    for (uint8_t axis = 0U; axis < 3U; axis++) {
        if ((bias->bmi088[axis] < -GYRO_BIAS_REASONABLE_RAD) ||
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
                                      uint32_t wait_ms,
                                      uint32_t record_ms,
                                      GyroBiasServiceHook_t service,
                                      void *service_context,
                                      GyroBiasProgressHook_t progress)
{
    double bmi088_sum[3] = {0.0, 0.0, 0.0};
    double bmi270_sum[3] = {0.0, 0.0, 0.0};
    uint32_t accepted_samples = 0U;
    uint32_t total_ms = wait_ms + record_ms;

    if ((bias == NULL) ||
        (wait_ms > GYRO_BIAS_MAX_TIME_MS) ||
        (record_ms == 0U) ||
        (record_ms > GYRO_BIAS_MAX_TIME_MS) ||
        (total_ms < wait_ms)) {
        return 0U;
    }

    for (uint8_t axis = 0U; axis < 3U; axis++) {
        BMI088Sensor.GyroOffset[axis] = 0.0f;
        BMI270Sensor.GyroOffset[axis] = 0.0f;
    }

    for (uint32_t elapsed = 0U; elapsed < total_ms; elapsed += GYRO_BIAS_SAMPLE_MS) {
        BMI088_Read(&BMI088Sensor);
        BMI270_Read(&BMI270Sensor);

        if (progress != NULL) {
            progress(elapsed, total_ms);
        }

        if (elapsed >= wait_ms) {
            for (uint8_t axis = 0U; axis < 3U; axis++) {
                bmi088_sum[axis] += (double)BMI088Sensor.Gyro[axis];
                bmi270_sum[axis] += (double)BMI270Sensor.Gyro[axis];
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
        return 0U;
    }

    for (uint8_t axis = 0U; axis < 3U; axis++) {
        bias->bmi088[axis] = (float)(bmi088_sum[axis] / (double)accepted_samples);
        bias->bmi270[axis] = (float)(bmi270_sum[axis] / (double)accepted_samples);
    }

    if (gyro_bias_values_are_reasonable(bias) == 0U) {
        return 0U;
    }

    GyroBias_Apply(bias);
    return 1U;
}

uint8_t GyroBias_Calibrate(GyroBiasData_t *bias,
                           uint32_t wait_ms,
                           uint32_t record_ms,
                           GyroBiasProgressHook_t progress)
{
    return GyroBias_CalibrateWithService(bias, wait_ms, record_ms, NULL, NULL, progress);
}

