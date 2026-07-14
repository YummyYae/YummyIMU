#include "accel_six_calibration.h"

#include "bmi088_mspm0.h"
#include "bmi270_mspm0.h"
#include "board_mspm0.h"
#include "imu_attitude.h"

#include <stddef.h>
#include <stdint.h>

#define ACCEL_SIX_GRAVITY_MPS2          9.80665f
#define ACCEL_SIX_SAMPLE_MS             1U
#define ACCEL_SIX_WARMUP_SAMPLES        200U
#define ACCEL_SIX_RECORD_SAMPLES        1000U
#define ACCEL_SIX_TOTAL_SAMPLES         (ACCEL_SIX_WARMUP_SAMPLES + ACCEL_SIX_RECORD_SAMPLES)
#define ACCEL_SIX_GYRO_MOVING_LIMIT     0.10f
#define ACCEL_SIX_ACCEL_NORM_MIN        7.5f
#define ACCEL_SIX_ACCEL_NORM_MAX        12.5f
#define ACCEL_SIX_FACE_AXIS_MIN         7.0f
#define ACCEL_SIX_FACE_CROSS_MAX        4.0f
#define ACCEL_SIX_VARIANCE_MAX          0.20f
#define ACCEL_SIX_BAD_SAMPLE_MAX        50U
#define ACCEL_SIX_OFFSET_MAX            3.0f
#define ACCEL_SIX_SCALE_MIN             0.8f
#define ACCEL_SIX_SCALE_MAX             1.2f

typedef struct {
    float bmi088_mean[ACCEL_SIX_FACE_COUNT][3];
    float bmi270_mean[ACCEL_SIX_FACE_COUNT][3];
    uint8_t sensor_mask;
    uint8_t next_face;
    uint8_t active;
} AccelSixContext_t;

typedef struct {
    float sum[3];
    float sum_sq[3];
    uint16_t moving_samples;
    uint16_t invalid_samples;
} AccelCaptureAccumulator_t;

static AccelSixContext_t gAccelSix;

static uint8_t accel_value_is_valid(float value)
{
    return ((value == value) && (value >= -100.0f) && (value <= 100.0f)) ? 1U : 0U;
}

static float vector_norm_sq(Axis3f value)
{
    return (value.x * value.x) + (value.y * value.y) + (value.z * value.z);
}

static void accumulator_add(AccelCaptureAccumulator_t *accumulator,
                            Axis3f accel,
                            Axis3f gyro,
                            uint8_t saturated)
{
    float accel_norm_sq;

    if ((saturated != 0U) ||
        (accel_value_is_valid(accel.x) == 0U) ||
        (accel_value_is_valid(accel.y) == 0U) ||
        (accel_value_is_valid(accel.z) == 0U) ||
        (accel_value_is_valid(gyro.x) == 0U) ||
        (accel_value_is_valid(gyro.y) == 0U) ||
        (accel_value_is_valid(gyro.z) == 0U)) {
        accumulator->invalid_samples++;
        return;
    }

    accel_norm_sq = vector_norm_sq(accel);
    if ((vector_norm_sq(gyro) >
         (ACCEL_SIX_GYRO_MOVING_LIMIT * ACCEL_SIX_GYRO_MOVING_LIMIT)) ||
        (accel_norm_sq < (ACCEL_SIX_ACCEL_NORM_MIN * ACCEL_SIX_ACCEL_NORM_MIN)) ||
        (accel_norm_sq > (ACCEL_SIX_ACCEL_NORM_MAX * ACCEL_SIX_ACCEL_NORM_MAX))) {
        accumulator->moving_samples++;
    }

    accumulator->sum[0] += accel.x;
    accumulator->sum[1] += accel.y;
    accumulator->sum[2] += accel.z;
    accumulator->sum_sq[0] += accel.x * accel.x;
    accumulator->sum_sq[1] += accel.y * accel.y;
    accumulator->sum_sq[2] += accel.z * accel.z;
}

static AccelSixResult_t accumulator_finish(const AccelCaptureAccumulator_t *accumulator,
                                           uint8_t face,
                                           float mean[3])
{
    uint8_t face_axis = face / 2U;
    uint8_t positive_face = ((face & 1U) == 0U) ? 1U : 0U;

    if (accumulator->invalid_samples != 0U) {
        return ACCEL_SIX_RESULT_INVALID_SENSOR;
    }
    if (accumulator->moving_samples > ACCEL_SIX_BAD_SAMPLE_MAX) {
        return ACCEL_SIX_RESULT_MOVING;
    }

    for (uint8_t axis = 0U; axis < 3U; axis++) {
        float variance;

        mean[axis] = accumulator->sum[axis] / (float)ACCEL_SIX_RECORD_SAMPLES;
        variance = (accumulator->sum_sq[axis] / (float)ACCEL_SIX_RECORD_SAMPLES) -
                   (mean[axis] * mean[axis]);
        if ((variance != variance) || (variance > ACCEL_SIX_VARIANCE_MAX)) {
            return ACCEL_SIX_RESULT_MOVING;
        }
    }

    if (((positive_face != 0U) && (mean[face_axis] < ACCEL_SIX_FACE_AXIS_MIN)) ||
        ((positive_face == 0U) && (mean[face_axis] > -ACCEL_SIX_FACE_AXIS_MIN))) {
        return ACCEL_SIX_RESULT_WRONG_FACE;
    }
    for (uint8_t axis = 0U; axis < 3U; axis++) {
        if ((axis != face_axis) &&
            ((mean[axis] > ACCEL_SIX_FACE_CROSS_MAX) ||
             (mean[axis] < -ACCEL_SIX_FACE_CROSS_MAX))) {
            return ACCEL_SIX_RESULT_WRONG_FACE;
        }
    }

    return ACCEL_SIX_RESULT_CAPTURED;
}

static uint8_t solve_sensor(const float means[ACCEL_SIX_FACE_COUNT][3],
                            float offset[3],
                            float scale[3])
{
    for (uint8_t axis = 0U; axis < 3U; axis++) {
        float plus = means[axis * 2U][axis];
        float minus = means[(axis * 2U) + 1U][axis];
        float span = plus - minus;
        float solved_offset;
        float solved_scale;

        if ((span != span) || (span <= 0.0f)) {
            return 0U;
        }

        solved_offset = 0.5f * (plus + minus);
        solved_scale = (2.0f * ACCEL_SIX_GRAVITY_MPS2) / span;
        if ((solved_offset != solved_offset) ||
            (solved_scale != solved_scale) ||
            (solved_offset < -ACCEL_SIX_OFFSET_MAX) ||
            (solved_offset > ACCEL_SIX_OFFSET_MAX) ||
            (solved_scale < ACCEL_SIX_SCALE_MIN) ||
            (solved_scale > ACCEL_SIX_SCALE_MAX)) {
            return 0U;
        }

        offset[axis] = solved_offset;
        scale[axis] = solved_scale;
    }

    return 1U;
}

void AccelSixCalibration_Reset(void)
{
    gAccelSix.sensor_mask = 0U;
    gAccelSix.next_face = ACCEL_SIX_FACE_POSITIVE_X;
    gAccelSix.active = 0U;
}

/* 开始一次事务式六面标定；只有第六面求解成功后，调用者才会收到新参数。 */
uint8_t AccelSixCalibration_Start(uint8_t sensor_mask)
{
    sensor_mask &= ACCEL_SIX_SENSOR_DUAL;
    if (sensor_mask == 0U) {
        return 0U;
    }

    for (uint8_t face = 0U; face < ACCEL_SIX_FACE_COUNT; face++) {
        for (uint8_t axis = 0U; axis < 3U; axis++) {
            gAccelSix.bmi088_mean[face][axis] = 0.0f;
            gAccelSix.bmi270_mean[face][axis] = 0.0f;
        }
    }
    gAccelSix.sensor_mask = sensor_mask;
    gAccelSix.next_face = ACCEL_SIX_FACE_POSITIVE_X;
    gAccelSix.active = 1U;
    return 1U;
}

uint8_t AccelSixCalibration_IsActive(void)
{
    return gAccelSix.active;
}

uint8_t AccelSixCalibration_GetExpectedFace(void)
{
    return gAccelSix.next_face;
}

/*
 * 采集当前一面：前 200 ms 丢弃，随后记录 1000 ms，并同时检查饱和、运动、方差和摆放方向。
 * 失败不会推进面序号，用户保持或重新摆放后可以再次发送 ACCAL CAPTURE。
 */
AccelSixResult_t AccelSixCalibration_Capture(AccelBiasData_t *offset,
                                             AccelScaleData_t *scale,
                                             AccelSixServiceHook_t service,
                                             void *service_context,
                                             AccelSixProgressHook_t progress)
{
    AccelCaptureAccumulator_t bmi088 = {{0.0f, 0.0f, 0.0f},
                                        {0.0f, 0.0f, 0.0f}, 0U, 0U};
    AccelCaptureAccumulator_t bmi270 = {{0.0f, 0.0f, 0.0f},
                                        {0.0f, 0.0f, 0.0f}, 0U, 0U};
    AccelBiasData_t solved_offset;
    AccelScaleData_t solved_scale;
    AccelSixResult_t result;
    uint8_t face;

    if ((gAccelSix.active == 0U) || (offset == NULL) || (scale == NULL)) {
        return ACCEL_SIX_RESULT_NOT_ACTIVE;
    }

    face = gAccelSix.next_face;
    for (uint32_t sample = 0U; sample < ACCEL_SIX_TOTAL_SAMPLES; sample++) {
        if ((gAccelSix.sensor_mask & ACCEL_SIX_SENSOR_BMI088) != 0U) {
            BMI088_Read(&BMI088Sensor);
        }
        if ((gAccelSix.sensor_mask & ACCEL_SIX_SENSOR_BMI270) != 0U) {
            BMI270_Read(&BMI270Sensor);
        }

        if (sample >= ACCEL_SIX_WARMUP_SAMPLES) {
            if ((gAccelSix.sensor_mask & ACCEL_SIX_SENSOR_BMI088) != 0U) {
                accumulator_add(&bmi088,
                                ImuAttitude_BMI088RawAccelToBoard(),
                                ImuAttitude_BMI088GyroToBoard(),
                                (uint8_t)(BMI088Sensor.LastAccelSaturated |
                                          BMI088Sensor.LastGyroSaturated));
            }
            if ((gAccelSix.sensor_mask & ACCEL_SIX_SENSOR_BMI270) != 0U) {
                accumulator_add(&bmi270,
                                ImuAttitude_BMI270RawAccelToBoard(),
                                ImuAttitude_BMI270GyroToBoard(),
                                (uint8_t)(BMI270Sensor.LastAccelSaturated |
                                          BMI270Sensor.LastGyroSaturated));
            }
        }

        if (service != NULL) {
            service(service_context);
        }
        if (progress != NULL) {
            progress(sample, ACCEL_SIX_TOTAL_SAMPLES);
        }
        Board_DelayMs(ACCEL_SIX_SAMPLE_MS);
    }

    if (progress != NULL) {
        progress(ACCEL_SIX_TOTAL_SAMPLES, ACCEL_SIX_TOTAL_SAMPLES);
    }

    if ((gAccelSix.sensor_mask & ACCEL_SIX_SENSOR_BMI088) != 0U) {
        result = accumulator_finish(&bmi088, face, gAccelSix.bmi088_mean[face]);
        if (result != ACCEL_SIX_RESULT_CAPTURED) {
            return result;
        }
    }
    if ((gAccelSix.sensor_mask & ACCEL_SIX_SENSOR_BMI270) != 0U) {
        result = accumulator_finish(&bmi270, face, gAccelSix.bmi270_mean[face]);
        if (result != ACCEL_SIX_RESULT_CAPTURED) {
            return result;
        }
    }

    gAccelSix.next_face++;
    if (gAccelSix.next_face < ACCEL_SIX_FACE_COUNT) {
        return ACCEL_SIX_RESULT_CAPTURED;
    }

    solved_offset = *offset;
    solved_scale = *scale;
    if (((gAccelSix.sensor_mask & ACCEL_SIX_SENSOR_BMI088) != 0U) &&
        (solve_sensor(gAccelSix.bmi088_mean,
                      solved_offset.bmi088,
                      solved_scale.bmi088) == 0U)) {
        gAccelSix.active = 0U;
        return ACCEL_SIX_RESULT_SOLVE_FAILED;
    }
    if (((gAccelSix.sensor_mask & ACCEL_SIX_SENSOR_BMI270) != 0U) &&
        (solve_sensor(gAccelSix.bmi270_mean,
                      solved_offset.bmi270,
                      solved_scale.bmi270) == 0U)) {
        gAccelSix.active = 0U;
        return ACCEL_SIX_RESULT_SOLVE_FAILED;
    }

    *offset = solved_offset;
    *scale = solved_scale;
    gAccelSix.active = 0U;
    return ACCEL_SIX_RESULT_COMPLETE;
}
