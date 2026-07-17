#ifndef SERIAL_TRANSPORT_H
#define SERIAL_TRANSPORT_H

#include <stdint.h>

/* UART/DMA 传输支撑层：不解析协议，也不参与任务调度。 */
void TaskSerial_ApplyBaud(uint32_t baud_rate);
void TaskSerial_Write(const char *text);
void TaskSerial_WriteBytes(const uint8_t *data, uint16_t length);
void TaskSerial_CollectRx(void);

#endif
