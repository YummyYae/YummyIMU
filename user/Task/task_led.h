#ifndef TASK_LED_H
#define TASK_LED_H

#include <stdint.h>

void TaskLED_Init(void);
void TaskLED_Set(uint8_t on);
void TaskLED_Refresh(uint8_t sensors_ok, uint8_t output_active);

#endif

