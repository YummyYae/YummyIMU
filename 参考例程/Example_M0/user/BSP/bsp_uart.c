/* 串口轮询接收姿态 CSV：pitch,roll,yaw\n，例如 1.234,-0.056,89.120 */
#include "ti_msp_dl_config.h"
#include <stdint.h>

#define BSP_UART_LINE_SIZE 64U
#define ATT_PITCH 0U
#define ATT_ROLL  1U
#define ATT_YAW   2U

/* 解包后的姿态数组：0=pitch，1=roll，2=yaw */
float g_bsp_uart_attitude[3];
static char s_line[BSP_UART_LINE_SIZE];
static uint8_t s_len;
static uint8_t s_overflow;

/* 解析一个普通小数：支持 +12、-12.34、0.56，不处理科学计数法 */
static uint8_t parse_number(char **p, float *value)
{
    float num = 0.0f;
    float scale = 1.0f;
    int8_t sign = 1;
    uint8_t has_digit = 0U;
    if (**p == '-') {
        sign = -1;
        (*p)++;
    } else if (**p == '+') {
        (*p)++;
    }
    while ((**p >= '0') && (**p <= '9')) {
        num = num * 10.0f + (float)(*(*p)++ - '0');
        has_digit = 1U;
    }
    if (**p == '.') {
        (*p)++;
        while ((**p >= '0') && (**p <= '9')) {
            scale *= 10.0f;
            num += (float)(*(*p)++ - '0') / scale;
            has_digit = 1U;
        }
    }
    *value = num * (float)sign;
    return has_digit;
}

/* 将一整行 CSV 解包到数组：out[0]=pitch，out[1]=roll，out[2]=yaw */
static uint8_t unpack_csv(char *line, float out[3])
{
    char *p = line;
    if ((parse_number(&p, &out[ATT_PITCH]) == 0U) || (*p++ != ',')) return 0U;
    if ((parse_number(&p, &out[ATT_ROLL]) == 0U) || (*p++ != ',')) return 0U;
    if (parse_number(&p, &out[ATT_YAW]) == 0U) return 0U;
    while ((*p == ' ') || (*p == '\t')) p++;
    return (*p == '\0') ? 1U : 0U;
}

static void reset_line(void) { s_len = 0U; s_overflow = 0U; }

static void finish_line(void)
{
    float value[3];
    if ((s_overflow == 0U) && (s_len != 0U)) {
        s_line[s_len] = '\0';
        if (unpack_csv(s_line, value) != 0U) {
            g_bsp_uart_attitude[ATT_PITCH] = value[ATT_PITCH];
            g_bsp_uart_attitude[ATT_ROLL] = value[ATT_ROLL];
            g_bsp_uart_attitude[ATT_YAW] = value[ATT_YAW];
        }
    }
    reset_line();
}

static void push_byte(uint8_t byte)
{
    if ((byte == '\n') || (byte == '\r')) finish_line();
    else if (s_len < (BSP_UART_LINE_SIZE - 1U)) s_line[s_len++] = (char)byte;
    else s_overflow = 1U;
}

void BSP_UART_Init(uint32_t baud_rate)
{
    DL_UART_Main_configBaudRate(UART_OUT_INST, UART_OUT_INST_FREQUENCY, baud_rate);
    DL_UART_Main_disableInterrupt(UART_OUT_INST, DL_UART_MAIN_INTERRUPT_RX);
    g_bsp_uart_attitude[ATT_PITCH] = 0.0f;
    g_bsp_uart_attitude[ATT_ROLL] = 0.0f;
    g_bsp_uart_attitude[ATT_YAW] = 0.0f;
    reset_line();
}

void BSP_UART_Poll(void)
{
    uint8_t byte;
    while (DL_UART_Main_receiveDataCheck(UART_OUT_INST, &byte)) {
        push_byte(byte);
    }
}
