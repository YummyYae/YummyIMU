#include "task_serial.h"

#include "serial_command.h"

/*
 * 串口任务只保留前台调度入口。
 * UART/DMA 收发由 serial_transport 负责，协议解析与业务编排由 serial_command 负责。
 */
uint8_t TaskSerial_PollCommand(RuntimeState_t *state)
{
    return SerialCommand_Poll(state);
}
