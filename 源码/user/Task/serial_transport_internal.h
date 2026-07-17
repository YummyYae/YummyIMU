#ifndef SERIAL_TRANSPORT_INTERNAL_H
#define SERIAL_TRANSPORT_INTERNAL_H

#include <stdint.h>

#define UART_RX_LINE_SIZE   48U
#define UART_RX_QUEUE_DEPTH 4U

/* 仅供串口任务内部的命令解释器消费完整行，不属于产品级公开接口。 */
uint8_t SerialTransport_TakeCommand(char line[UART_RX_LINE_SIZE]);
uint8_t SerialTransport_TakeOverflow(void);

#endif
