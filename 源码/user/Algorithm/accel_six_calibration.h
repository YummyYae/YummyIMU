#ifndef ACCEL_SIX_CALIBRATION_H
#define ACCEL_SIX_CALIBRATION_H

#include "flash_storage.h"

#include <stdint.h>

#define ACCEL_SIX_SENSOR_BMI088 0x01U
#define ACCEL_SIX_SENSOR_BMI270 0x02U
#define ACCEL_SIX_SENSOR_DUAL   (ACCEL_SIX_SENSOR_BMI088 | ACCEL_SIX_SENSOR_BMI270)

typedef enum {
    ACCEL_SIX_FACE_POSITIVE_X = 0U,
    ACCEL_SIX_FACE_NEGATIVE_X = 1U,
    ACCEL_SIX_FACE_POSITIVE_Y = 2U,
    ACCEL_SIX_FACE_NEGATIVE_Y = 3U,
    ACCEL_SIX_FACE_POSITIVE_Z = 4U,
    ACCEL_SIX_FACE_NEGATIVE_Z = 5U,
    ACCEL_SIX_FACE_COUNT = 6U
} AccelSixFace_t;

typedef enum {
    ACCEL_SIX_RESULT_CAPTURED = 0U,
    ACCEL_SIX_RESULT_COMPLETE = 1U,
    ACCEL_SIX_RESULT_NOT_ACTIVE = 2U,
    ACCEL_SIX_RESULT_INVALID_SENSOR = 3U,
    ACCEL_SIX_RESULT_MOVING = 4U,
    ACCEL_SIX_RESULT_WRONG_FACE = 5U,
    ACCEL_SIX_RESULT_SOLVE_FAILED = 6U
} AccelSixResult_t;

typedef void (*AccelSixServiceHook_t)(void *context);
typedef void (*AccelSixProgressHook_t)(uint32_t elapsed_ms, uint32_t total_ms);

void AccelSixCalibration_Reset(void);
uint8_t AccelSixCalibration_Start(uint8_t sensor_mask);
uint8_t AccelSixCalibration_IsActive(void);
uint8_t AccelSixCalibration_GetExpectedFace(void);
AccelSixResult_t AccelSixCalibration_Capture(AccelBiasData_t *offset,
                                             AccelScaleData_t *scale,
                                             AccelSixServiceHook_t service,
                                             void *service_context,
                                             AccelSixProgressHook_t progress);

#endif
