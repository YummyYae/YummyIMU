#ifndef GYRO_BIAS_CALIBRATION_H
#define GYRO_BIAS_CALIBRATION_H

#include "flash_storage.h"

#include <stdint.h>

typedef void (*GyroBiasServiceHook_t)(void *context);
typedef void (*GyroBiasProgressHook_t)(uint32_t elapsed_ms, uint32_t total_ms);

void GyroBias_Apply(const GyroBiasData_t *bias);
uint8_t GyroBias_Calibrate(GyroBiasData_t *bias,
                           uint32_t wait_ms,
                           uint32_t record_ms,
                           GyroBiasProgressHook_t progress);
uint8_t GyroBias_CalibrateWithService(GyroBiasData_t *bias,
                                      uint32_t wait_ms,
                                      uint32_t record_ms,
                                      GyroBiasServiceHook_t service,
                                      void *service_context,
                                      GyroBiasProgressHook_t progress);

#endif

