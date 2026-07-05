#ifndef TASK_TEMPERATURE_H
#define TASK_TEMPERATURE_H

#include "runtime_state.h"

void TaskTemperature_Init(RuntimeState_t *state);
void TaskTemperature_SampleSensorsFromInterrupt(RuntimeState_t *state);
void TaskTemperature_Update100Hz(RuntimeState_t *state);
void TaskTemperature_CalibrationService(void *context);
uint8_t TaskTemperature_HaveFault(const RuntimeState_t *state);
uint8_t TaskTemperature_IsReady(const RuntimeState_t *state, float tolerance_c);

#endif

