#include "bsp_uart.h"
#include <string.h>

#define BSP_UART_LINE_SIZE 64U
#define BSP_UART_MAX_PORTS 4U
#define BSP_UART_RX_BUDGET 32U
#define IS_DIGIT(ch) (((ch) >= '0') && ((ch) <= '9'))

enum {
  ATT_PITCH = 0,
  ATT_ROLL,
  ATT_YAW
};

/* 每个串口独立保存一行接收缓存，避免多路输入互相串包。 */
typedef struct {
  UART_HandleTypeDef *uart;
  char line[BSP_UART_LINE_SIZE];
  uint8_t len;
  uint8_t overflow;
} BSP_UART_Rx_t;

/* 最新一次成功解包的姿态数据。 */
BSP_UART_Attitude_t g_bsp_uart_attitude;

static BSP_UART_Rx_t s_rx[BSP_UART_MAX_PORTS];
static uint8_t s_rx_count;

static void reset_rx(BSP_UART_Rx_t *rx)
{
  rx->len = 0U;
  rx->overflow = 0U;
}

static void clear_uart_errors(UART_HandleTypeDef *huart)
{
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET) {
    __HAL_UART_CLEAR_OREFLAG(huart);
  }
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE) != RESET) {
    __HAL_UART_CLEAR_FEFLAG(huart);
  }
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE) != RESET) {
    __HAL_UART_CLEAR_NEFLAG(huart);
  }
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_PE) != RESET) {
    __HAL_UART_CLEAR_PEFLAG(huart);
  }
}

static void skip_spaces(char **p)
{
  while ((**p == ' ') || (**p == '\t')) {
    (*p)++;
  }
}

/* 解析一个普通小数：支持 +12、-12.34、.56，不支持科学计数法。 */
static uint8_t parse_number(char **p, float *value)
{
  float num = 0.0f;
  float scale = 1.0f;
  int8_t sign = 1;
  uint8_t has_digit = 0U;

  skip_spaces(p);

  if ((**p == '-') || (**p == '+')) {
    if (*(*p)++ == '-') {
      sign = -1;
    }
  }

  while (IS_DIGIT(**p)) {
    num = (num * 10.0f) + (float)(*(*p)++ - '0');
    has_digit = 1U;
  }

  if (**p == '.') {
    (*p)++;
    while (IS_DIGIT(**p)) {
      scale *= 10.0f;
      num += (float)(*(*p)++ - '0') / scale;
      has_digit = 1U;
    }
  }

  *value = num * (float)sign;
  return has_digit;
}

static uint8_t parse_comma(char **p)
{
  skip_spaces(p);
  if (**p != ',') {
    return 0U;
  }
  (*p)++;
  return 1U;
}

/* 将一整行 CSV 解包为 pitch,roll,yaw 三个角度。 */
uint8_t BSP_UART_ParseLine(char *line, BSP_UART_Attitude_t *out)
{
  float value[BSP_UART_USE_VALUE_COUNT];
  char *p = line;

  if ((line == 0) || (out == 0)) {
    return 0U;
  }

  if ((parse_number(&p, &value[ATT_PITCH]) == 0U) || (parse_comma(&p) == 0U)) {
    return 0U;
  }
  if ((parse_number(&p, &value[ATT_ROLL]) == 0U) || (parse_comma(&p) == 0U)) {
    return 0U;
  }
  if (parse_number(&p, &value[ATT_YAW]) == 0U) {
    return 0U;
  }

  skip_spaces(&p);
  if (*p != '\0') {
    return 0U;
  }

  (void)memset(out, 0, sizeof(*out));
  for (uint8_t i = 0U; i < BSP_UART_USE_VALUE_COUNT; i++) {
    out->virtual_angle[i] = value[i];
    out->raw[i] = value[i];
  }

  out->mode = BSP_UART_FRAME_USE;
  out->last_update_ms = HAL_GetTick();
  out->valid = 1U;
  out->updated = 1U;
  return 1U;
}

/* 遇到换行后提交一整行；坏包只累计错误计数。 */
static void finish_line(BSP_UART_Rx_t *rx)
{
  BSP_UART_Attitude_t parsed;

  if ((rx->overflow == 0U) && (rx->len != 0U)) {
    rx->line[rx->len] = '\0';

    if (BSP_UART_ParseLine(rx->line, &parsed) != 0U) {
      parsed.rx_byte_count = g_bsp_uart_attitude.rx_byte_count;
      parsed.rx_line_count = g_bsp_uart_attitude.rx_line_count + 1U;
      parsed.valid_frame_count = g_bsp_uart_attitude.valid_frame_count + 1U;
      parsed.bad_frame_count = g_bsp_uart_attitude.bad_frame_count;
      parsed.last_byte = g_bsp_uart_attitude.last_byte;
      g_bsp_uart_attitude = parsed;
    } else {
      g_bsp_uart_attitude.rx_line_count++;
      g_bsp_uart_attitude.bad_frame_count++;
    }
  }

  reset_rx(rx);
}

/* 按字节拼接一行，协议用 '\r' 或 '\n' 作为帧结束。 */
static void push_byte(BSP_UART_Rx_t *rx, uint8_t byte)
{
  g_bsp_uart_attitude.rx_byte_count++;
  g_bsp_uart_attitude.last_byte = byte;

  if ((byte == '\n') || (byte == '\r')) {
    finish_line(rx);
  } else if ((byte >= 0x20U) && (byte <= 0x7EU) &&
             (rx->len < (BSP_UART_LINE_SIZE - 1U))) {
    rx->line[rx->len++] = (char)byte;
  } else if ((byte < 0x20U) || (byte > 0x7EU)) {
    reset_rx(rx);
  } else {
    rx->overflow = 1U;
  }
}

void BSP_UART_Init(UART_HandleTypeDef *huart)
{
  (void)memset(&g_bsp_uart_attitude, 0, sizeof(g_bsp_uart_attitude));
  (void)memset(s_rx, 0, sizeof(s_rx));
  s_rx_count = 0U;
  (void)BSP_UART_Add(huart);
}

/* 注册一个需要参与解包的串口。 */
uint8_t BSP_UART_Add(UART_HandleTypeDef *huart)
{
  if ((huart == 0) || (s_rx_count >= BSP_UART_MAX_PORTS)) {
    return 0U;
  }

  s_rx[s_rx_count].uart = huart;
  reset_rx(&s_rx[s_rx_count++]);

  clear_uart_errors(huart);
  for (uint8_t i = 0U; (i < BSP_UART_RX_BUDGET) &&
       (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET); i++) {
    (void)(uint8_t)(huart->Instance->RDR & 0xFFU);
  }

  return 1U;
}

void BSP_UART_PushByte(uint8_t byte)
{
  if (s_rx_count != 0U) {
    push_byte(&s_rx[0], byte);
  }
}

/* 轮询所有已注册串口，任意一路收到完整行都会尝试解包。 */
void BSP_UART_Poll(void)
{
  for (uint8_t i = 0U; i < s_rx_count; i++) {
    clear_uart_errors(s_rx[i].uart);
    for (uint8_t n = 0U; (n < BSP_UART_RX_BUDGET) &&
         (__HAL_UART_GET_FLAG(s_rx[i].uart, UART_FLAG_RXNE) != RESET); n++) {
      push_byte(&s_rx[i], (uint8_t)(s_rx[i].uart->Instance->RDR & 0xFFU));
    }
  }
}

uint8_t BSP_UART_TakeUpdated(BSP_UART_Attitude_t *out)
{
  if (g_bsp_uart_attitude.updated == 0U) {
    return 0U;
  }

  if (out != 0) {
    *out = g_bsp_uart_attitude;
  }
  g_bsp_uart_attitude.updated = 0U;
  return 1U;
}
