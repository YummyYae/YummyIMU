#ifndef TASK_SERIAL_H
#define TASK_SERIAL_H

#include <stdint.h>
#include "runtime_state.h"

#define UART_RX_LINE_SIZE   48U
#define UART_RX_QUEUE_DEPTH 4U

void TaskSerial_ApplyBaud(uint32_t baud_rate);
void TaskSerial_Write(const char *text);
void TaskSerial_CollectRx(void);
uint8_t TaskSerial_TakeCommand(char line[UART_RX_LINE_SIZE]);
uint8_t TaskSerial_TakeOverflow(void);
uint8_t TaskSerial_PollCommand(RuntimeState_t *ctx);
void TaskSerial_PauseReport(RuntimeState_t *ctx);
void GyroBias_CalibrationProgress(uint32_t elapsed_ms, uint32_t total_ms);

#endif
