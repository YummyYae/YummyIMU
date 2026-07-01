#include "task_led.h"

#include "ti_msp_dl_config.h"

#define STATUS_LED_PORT GPIOB
#define STATUS_LED_PIN DL_GPIO_PIN_18
#define STATUS_LED_IOMUX IOMUX_PINCM44

/*
 * LED 任务流程：
 * 1. TaskLED_Init() 配置 PB18 为输出，并默认关闭 LED。
 * 2. TaskLED_Set() 封装 PB18 低电平点亮的硬件细节。
 * 3. TaskLED_Refresh() 根据“传感器正常”和“当前允许回传”两个条件决定是否点亮。
 * 4. 串口命令回复等待期间、初始化异常或暂停输出时，其他任务会直接关闭 LED。
 */

/* 初始化 PB18 状态灯 GPIO，默认关闭后再使能输出。 */
void TaskLED_Init(void)
{
    DL_GPIO_initDigitalOutput(STATUS_LED_IOMUX);
    TaskLED_Set(0U);
    DL_GPIO_enableOutput(STATUS_LED_PORT, STATUS_LED_PIN);
}

/* 设置 LED 状态；on 非 0 点亮，on 为 0 熄灭。PB18 是低电平点亮。 */
void TaskLED_Set(uint8_t on)
{
    if (on != 0U) {
        DL_GPIO_clearPins(STATUS_LED_PORT, STATUS_LED_PIN);
    } else {
        DL_GPIO_setPins(STATUS_LED_PORT, STATUS_LED_PIN);
    }
}

/* 根据传感器状态和输出状态刷新 LED，作为系统是否正常回传的外部指示。 */
void TaskLED_Refresh(uint8_t sensors_ok, uint8_t output_active)
{
    TaskLED_Set(((sensors_ok != 0U) && (output_active != 0U)) ? 1U : 0U);
}

