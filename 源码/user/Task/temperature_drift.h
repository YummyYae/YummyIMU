#ifndef TEMPERATURE_DRIFT_H
#define TEMPERATURE_DRIFT_H

#include "runtime_state.h"

/* 温漂扫描是温度任务的内部子状态机，不参与 main 的独立任务调度。 */
void TemperatureDrift_Init(void);
void TemperatureDrift_Service100Hz(RuntimeState_t *state);

#endif
