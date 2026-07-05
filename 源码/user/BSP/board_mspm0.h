#ifndef APP_BOARD_H
#define APP_BOARD_H

void Board_InitHardware(void);
void Board_EnableInterrupts(void);
void Board_ResetAfterSave(void);
void Board_HandleGroup1Irq(void);
void Board_HandleNmi(void);

#endif

