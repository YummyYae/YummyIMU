#ifndef SERIAL_STATUS_H
#define SERIAL_STATUS_H

#include "runtime_state.h"

/* 输出 STATUS 及其子命令快照；args 指向 STATUS 后的剩余参数。 */
void SerialStatus_WriteSnapshot(RuntimeState_t *state, char *args);

#endif
