#ifndef SERIAL_COMMAND_H
#define SERIAL_COMMAND_H

#include "runtime_state.h"

#include <stdint.h>

uint8_t SerialCommand_Poll(RuntimeState_t *state);
void SerialCommand_PauseReport(RuntimeState_t *state);
void SerialCommand_CalibrationProgress(uint32_t elapsed_ms, uint32_t total_ms);

#endif
