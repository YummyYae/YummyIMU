#ifndef TASK_TEMPERATURE_H
#define TASK_TEMPERATURE_H

#include "runtime_state.h"

#define TEMPERATURE_DRIFT_DEFAULT_RATE_C_PER_MIN 1.0f
#define TEMPERATURE_DRIFT_DEFAULT_REPORT_HZ      10U

typedef enum {
    TEMPERATURE_DRIFT_START_OK = 0,
    TEMPERATURE_DRIFT_START_ALREADY_ACTIVE,
    TEMPERATURE_DRIFT_START_INVALID_PARAM,
    TEMPERATURE_DRIFT_START_REQUIRES_DUAL_IMU,
    TEMPERATURE_DRIFT_START_TEMPERATURE_INVALID,
    TEMPERATURE_DRIFT_START_TEMPERATURE_TOO_HIGH
} TemperatureDriftStartResult_t;

void TaskTemperature_Init(RuntimeState_t *state);
void TaskTemperature_SampleSensorsFromInterrupt(RuntimeState_t *state);
/* 1kHz 中断采样入口，仅在 TDRIFT 运行时执行轻量级累加。 */
void TaskTemperature_AccumulateDriftFromInterrupt(const RuntimeState_t *state);
void TaskTemperature_Update100Hz(RuntimeState_t *state);
void TaskTemperature_CalibrationService(void *context);
uint8_t TaskTemperature_HaveFault(const RuntimeState_t *state);
uint8_t TaskTemperature_IsReady(const RuntimeState_t *state, float tolerance_c);
/* 连续温漂测试控制接口；测试参数仅驻留 RAM，不写入 Flash。 */
TemperatureDriftStartResult_t TaskTemperature_StartDriftTest(RuntimeState_t *state,
                                                              float rate_c_per_min,
                                                              uint32_t report_hz);
uint8_t TaskTemperature_StopDriftTest(RuntimeState_t *state);
void TaskTemperature_WriteDriftTestStatus(const RuntimeState_t *state);
uint8_t TaskTemperature_IsDriftTestActive(void);

#endif

