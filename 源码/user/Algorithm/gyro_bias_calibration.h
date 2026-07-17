#ifndef GYRO_BIAS_CALIBRATION_H
#define GYRO_BIAS_CALIBRATION_H

#include "runtime_config.h"

#include <stdint.h>

#define IMU_BIAS_SENSOR_BMI088 0x01U
#define IMU_BIAS_SENSOR_BMI270 0x02U
#define IMU_BIAS_SENSOR_DUAL   (IMU_BIAS_SENSOR_BMI088 | IMU_BIAS_SENSOR_BMI270)

typedef void (*GyroBiasServiceHook_t)(void *context);
typedef void (*GyroBiasProgressHook_t)(uint32_t elapsed_ms, uint32_t total_ms);

typedef struct {
    float bmi088_variance[3];
    float bmi270_variance[3];
    uint8_t bmi088_score;
    uint8_t bmi270_score;
} GyroQualityData_t;

void GyroBias_Apply(const GyroBiasData_t *bias);
void GyroBias_StoreQuality(RuntimeConfig_t *config,
                           const GyroQualityData_t *quality,
                           uint8_t sensor_mask);
/* 按当前评分参考标准重算 Flash 中已有的质量分数。 */
uint8_t GyroBias_RecalculateScores(RuntimeConfig_t *config);
uint8_t GyroBias_Calibrate(GyroBiasData_t *bias,
                           GyroQualityData_t *quality,
                           uint8_t sensor_mask,
                           uint32_t wait_ms,
                           uint32_t record_ms,
                           GyroBiasProgressHook_t progress);
uint8_t GyroBias_CalibrateWithService(GyroBiasData_t *bias,
                                      GyroQualityData_t *quality,
                                      uint8_t sensor_mask,
                                      uint32_t wait_ms,
                                      uint32_t record_ms,
                                      GyroBiasServiceHook_t service,
                                      void *service_context,
                                      GyroBiasProgressHook_t progress);

#endif
