#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include <stdint.h>

#define GYRO_BIAS_DEFAULT_WAIT_MS   (10000U)
#define GYRO_BIAS_DEFAULT_RECORD_MS (30000U)
#define RUNTIME_CONFIG_DEFAULT_BAUD_RATE      (921600U)
#define RUNTIME_CONFIG_DEFAULT_REPORT_RATE_HZ (50U)
#define RUNTIME_CONFIG_DEFAULT_TARGET_TEMP_C  (40.0f)
#define RUNTIME_CONFIG_DEFAULT_YAW_ERR_DEG    (0.0f)

#define RUNTIME_REPORT_RATE_BASE_HZ       (1000U)
#define RUNTIME_REPORT_RATE_USE_MAX_HZ    (500U)
#define RUNTIME_REPORT_RATE_DEBUG_MAX_HZ  (100U)
#define RUNTIME_REPORT_RATE_BINARY_MAX_HZ (1000U)
#define RUNTIME_REPORT_RATE_INS_MAX_HZ    (100U)

/* 当前发布版开放简化惯导，但不把六面加速度标定作为启动条件。 */
#define RUNTIME_FEATURE_INS_ENABLED              1U
#define RUNTIME_INS_REQUIRE_ACCEL_CALIBRATION    0U

#define RUNTIME_OUTPUT_MODE_USE    0U
#define RUNTIME_OUTPUT_MODE_DEBUG  1U
#define RUNTIME_OUTPUT_MODE_BINARY 2U
#define RUNTIME_OUTPUT_MODE_INS    3U
#define RUNTIME_CONFIG_DEFAULT_OUTPUT_MODE RUNTIME_OUTPUT_MODE_USE

#define RUNTIME_IMU_SOURCE_DUAL   0U
#define RUNTIME_IMU_SOURCE_BMI088 1U
#define RUNTIME_IMU_SOURCE_BMI270 2U
#define RUNTIME_IMU_SOURCE_AUTO   3U
#define RUNTIME_CONFIG_DEFAULT_IMU_SOURCE RUNTIME_IMU_SOURCE_DUAL

#define RUNTIME_ACCEL_CAL_BMI088 0x01U
#define RUNTIME_ACCEL_CAL_BMI270 0x02U
#define RUNTIME_ACCEL_CAL_DUAL   (RUNTIME_ACCEL_CAL_BMI088 | RUNTIME_ACCEL_CAL_BMI270)
#define RUNTIME_GYRO_CAL_BMI088  RUNTIME_ACCEL_CAL_BMI088
#define RUNTIME_GYRO_CAL_BMI270  RUNTIME_ACCEL_CAL_BMI270
#define RUNTIME_GYRO_CAL_DUAL    RUNTIME_ACCEL_CAL_DUAL

typedef struct {
    float bmi088[3];
    float bmi270[3];
} GyroBiasData_t;

typedef struct {
    float bmi088[3];
    float bmi270[3];
} AccelBiasData_t;

typedef struct {
    float bmi088[3];
    float bmi270[3];
} AccelScaleData_t;

typedef struct {
    uint32_t baud_rate;
    uint32_t report_rate_hz;
    float target_temperature_c;
    uint8_t gyro_bias_valid;
    uint8_t accel_bias_valid;
    uint8_t gyro_calibrated_mask;
    uint8_t accel_calibrated_mask;
    uint8_t output_mode;
    uint8_t imu_source;
    uint8_t reserved[2];
    float bmi088_yaw_error_deg_per_turn;
    float bmi270_yaw_error_deg_per_turn;
    GyroBiasData_t gyro_bias;
    AccelBiasData_t accel_bias;
    AccelScaleData_t accel_scale;
} RuntimeConfig_t;

void RuntimeConfig_Default(RuntimeConfig_t *config);
uint8_t RuntimeConfig_Load(RuntimeConfig_t *config);
uint8_t RuntimeConfig_Save(const RuntimeConfig_t *config);
uint32_t RuntimeConfig_GetReportRateLimit(uint8_t output_mode);
uint8_t RuntimeConfig_ReportRateIsSupported(uint8_t output_mode, uint32_t report_rate_hz);

#endif
