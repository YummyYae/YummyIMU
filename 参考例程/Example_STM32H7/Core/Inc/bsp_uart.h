#ifndef BSP_UART_H
#define BSP_UART_H

#include "main.h"
#include <stdint.h>

#define BSP_UART_ATT_AXIS_COUNT      3U
#define BSP_UART_USE_VALUE_COUNT     3U

/* 当前只使用 pitch, roll, yaw 三值姿态帧。 */
typedef enum {
  BSP_UART_FRAME_NONE = 0,
  BSP_UART_FRAME_USE = 3
} BSP_UART_FrameMode_t;

typedef struct {
  /* 解包后的姿态角：0=pitch，1=roll，2=yaw。 */
  float virtual_angle[BSP_UART_ATT_AXIS_COUNT];
  /* 保存原始三值，方便调试或上层直接读取。 */
  float raw[BSP_UART_USE_VALUE_COUNT];
  uint32_t last_update_ms;
  uint32_t rx_byte_count;
  uint32_t rx_line_count;
  uint32_t valid_frame_count;
  uint32_t bad_frame_count;
  uint8_t last_byte;
  BSP_UART_FrameMode_t mode;
  /* updated=1 表示有新数据未取走，valid=1 表示至少成功解包过一次。 */
  uint8_t updated;
  uint8_t valid;
} BSP_UART_Attitude_t;

extern BSP_UART_Attitude_t g_bsp_uart_attitude;

/* Init 会清空状态并注册第一个串口，Add 用于继续注册其它串口。 */
void BSP_UART_Init(UART_HandleTypeDef *huart);
uint8_t BSP_UART_Add(UART_HandleTypeDef *huart);
void BSP_UART_Poll(void);
void BSP_UART_PushByte(uint8_t byte);
uint8_t BSP_UART_ParseLine(char *line, BSP_UART_Attitude_t *out);
uint8_t BSP_UART_TakeUpdated(BSP_UART_Attitude_t *out);

#endif
