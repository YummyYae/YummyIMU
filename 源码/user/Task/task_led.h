#ifndef TASK_LED_H
#define TASK_LED_H

#include <stdint.h>

void TaskLED_Init(void);
void TaskLED_Set(uint8_t on);
void TaskLED_UpdateCalibrationStatus(void);
void TaskLED_UpdateSystemStatus(uint8_t started, uint8_t alarm, uint8_t ready, uint8_t output_active);

#endif
