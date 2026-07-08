#include "task_led.h"

#include "ti_msp_dl_config.h"
#include "runtime_state.h"

#define STATUS_LED_PORT GPIOB
#define STATUS_LED_PIN DL_GPIO_PIN_18
#define STATUS_LED_IOMUX IOMUX_PINCM44
#define STATUS_LED_ALARM_BLINK_MS 100U
#define STATUS_LED_WAIT_BLINK_MS 600U
#define STATUS_LED_CAL_BLINK_MS 3000U

/*
 * LED 任务流程：
 * 1. TaskLED_Init() 配置 PB18 为输出，并默认关闭 LED。
 * 2. TaskLED_Set() 封装 PB18 低电平点亮的硬件细节。
 * 3. TaskLED_UpdateSystemStatus() 负责错误快闪、等待加热慢闪、温度到位常亮。
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

void TaskLED_UpdateCalibrationStatus(void)
{
    uint8_t blink_on = (((gSystemTickMs / STATUS_LED_CAL_BLINK_MS) & 1U) == 0U) ? 1U : 0U;

    TaskLED_Set(blink_on);
}

/*
 * 刷新系统状态灯：
 * - 出现 IMU/温度异常：快闪。
 * - 无错误但两颗 IMU 尚未进入目标温度 +-2 摄氏度：慢闪。
 * - 两颗 IMU 均进入目标温度 +-2 摄氏度且一切正常：常亮。
 * - 命令回复暂停期间或尚未启动完成：熄灭。
 */
void TaskLED_UpdateSystemStatus(uint8_t started, uint8_t alarm, uint8_t ready, uint8_t output_active)
{
    uint8_t blink_on;

    if ((started == 0U) || (output_active == 0U)) {
        TaskLED_Set(0U);
        return;
    }

    if (alarm != 0U) {
        blink_on = (((gSystemTickMs / STATUS_LED_ALARM_BLINK_MS) & 1U) == 0U) ? 1U : 0U;
        TaskLED_Set(blink_on);
        return;
    }

    if (ready != 0U) {
        TaskLED_Set(1U);
    } else {
        blink_on = (((gSystemTickMs / STATUS_LED_WAIT_BLINK_MS) & 1U) == 0U) ? 1U : 0U;
        TaskLED_Set(blink_on);
    }
}

