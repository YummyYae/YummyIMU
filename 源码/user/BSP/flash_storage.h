#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include <stdint.h>

#define GYRO_BIAS_DEFAULT_WAIT_MS   (10000U)
#define GYRO_BIAS_DEFAULT_RECORD_MS (30000U)
#define RUNTIME_CONFIG_DEFAULT_BAUD_RATE      (921600U)
#define RUNTIME_CONFIG_DEFAULT_REPORT_RATE_HZ (50U)
#define RUNTIME_CONFIG_DEFAULT_TARGET_TEMP_C  (40.0f)

#define RUNTIME_OUTPUT_MODE_USE   0U
#define RUNTIME_OUTPUT_MODE_DEBUG 1U
#define RUNTIME_CONFIG_DEFAULT_OUTPUT_MODE RUNTIME_OUTPUT_MODE_USE

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
    uint8_t reserved[2];
    GyroBiasData_t gyro_bias;
} RuntimeConfig_t;

typedef void (*GyroBiasServiceHook_t)(void *context);

void RuntimeConfig_Default(RuntimeConfig_t *config);
uint8_t RuntimeConfig_Load(RuntimeConfig_t *config);
uint8_t RuntimeConfig_Save(const RuntimeConfig_t *config);

uint8_t GyroBias_Calibrate(GyroBiasData_t *bias, uint32_t wait_ms, uint32_t record_ms);
uint8_t GyroBias_CalibrateWithService(GyroBiasData_t *bias,
                                      uint32_t wait_ms,
                                      uint32_t record_ms,
                                      GyroBiasServiceHook_t service,
                                      void *service_context);
void GyroBias_Apply(const GyroBiasData_t *bias);

#endif

