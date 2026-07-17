#ifndef TASK_SERIAL_H
#define TASK_SERIAL_H

#include "runtime_state.h"
#include "serial_transport.h"

uint8_t TaskSerial_PollCommand(RuntimeState_t *ctx);

#endif
