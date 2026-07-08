#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include <stdint.h>

#define GYRO_BIAS_DEFAULT_WAIT_MS   (10000U)
#define GYRO_BIAS_DEFAULT_RECORD_MS (30000U)
#define RUNTIME_CONFIG_DEFAULT_BAUD_RATE      (921600U)
#define RUNTIME_CONFIG_DEFAULT_REPORT_RATE_HZ (50U)
#define RUNTIME_CONFIG_DEFAULT_TARGET_TEMP_C  (40.0f)
#define RUNTIME_CONFIG_DEFAULT_YAW_ERR_DEG    (0.0f)

#define RUNTIME_OUTPUT_MODE_USE   0U
#define RUNTIME_OUTPUT_MODE_DEBUG 1U
#define RUNTIME_CONFIG_DEFAULT_OUTPUT_MODE RUNTIME_OUTPUT_MODE_USE

#define RUNTIME_IMU_SOURCE_DUAL   0U
#define RUNTIME_IMU_SOURCE_BMI088 1U
#define RUNTIME_IMU_SOURCE_BMI270 2U
#define RUNTIME_IMU_SOURCE_AUTO   3U
#define RUNTIME_CONFIG_DEFAULT_IMU_SOURCE RUNTIME_IMU_SOURCE_DUAL

typedef struct {
    float bmi088[3];
    float bmi270[3];
} GyroBiasData_t;

typedef struct {
    uint32_t baud_rate;
    uint32_t report_rate_hz;
    float target_temperature_c;
    uint8_t gyro_bias_valid;
    uint8_t output_mode;
    uint8_t imu_source;
    uint8_t reserved[1];
    float bmi088_yaw_error_deg_per_turn;
    float bmi270_yaw_error_deg_per_turn;
    GyroBiasData_t gyro_bias;
} RuntimeConfig_t;

void RuntimeConfig_Default(RuntimeConfig_t *config);
uint8_t RuntimeConfig_Load(RuntimeConfig_t *config);
uint8_t RuntimeConfig_Save(const RuntimeConfig_t *config);

#endif
