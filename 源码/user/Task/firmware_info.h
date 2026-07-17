#ifndef FIRMWARE_INFO_H
#define FIRMWARE_INFO_H

/* 固件版本字符串由 main.c 顶部宏统一定义，STATUS 只通过此接口读取。 */
const char *Firmware_GetVersion(void);

#endif
