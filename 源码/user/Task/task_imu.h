#ifndef TASK_IMU_H
#define TASK_IMU_H

#include "runtime_state.h"

#include <stdint.h>

void TaskIMU_Init(RuntimeState_t *state);
uint8_t TaskIMU_HaveFault(const RuntimeState_t *state);
uint8_t TaskIMU_HaveAlarm(const RuntimeState_t *state);
void TaskIMU_Update1kHz(RuntimeState_t *state);
void TaskIMU_EnableInterruptUpdate(uint8_t enable);
void TaskIMU_UpdateFromInterrupt(RuntimeState_t *state);
void TaskIMU_ServiceReport(RuntimeState_t *state);
void TaskIMU_AlignInitialAttitude(void);
void TaskIMU_UpdateOutputAngles(void);
void TaskIMU_WriteAngles(const RuntimeState_t *state);

#endif

