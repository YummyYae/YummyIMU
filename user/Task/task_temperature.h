#ifndef TASK_TEMPERATURE_H
#define TASK_TEMPERATURE_H

#include "runtime_state.h"

void TaskTemperature_Init(RuntimeState_t *state);
void TaskTemperature_SampleSensorsFromInterrupt(RuntimeState_t *state);
void TaskTemperature_Update100Hz(RuntimeState_t *state);

#endif

