#ifndef APP_BOARD_H
#define APP_BOARD_H

#include <stdint.h>

void Board_InitHardware(void);
void Board_EnableInterrupts(void);
void Board_EnterCritical(void);
void Board_ExitCritical(void);
void Board_DelayMs(uint32_t ms);
void Board_StartImuTimer(void);
uint32_t Board_GetImuTimerCount(void);
uint32_t Board_GetImuTimerElapsedTicks(uint32_t start_count, uint32_t end_count);
uint32_t Board_GetImuInterruptBudgetTicks(void);
void Board_ResetAfterSave(void);
void Board_HandleGroup1Irq(void);
void Board_HandleNmi(void);

#endif

