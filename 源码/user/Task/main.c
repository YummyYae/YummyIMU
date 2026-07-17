#define FIRMWARE_VERSION "1.5"
#include "firmware_info.h"
#include "board_mspm0.h"
#include "task_temperature.h"
#include "task_imu.h"
#include "runtime_state.h"
#include "task_led.h"
#include "task_serial.h"

#include "ti_msp_dl_config.h"

const char *Firmware_GetVersion(void)
{
    return FIRMWARE_VERSION;
}

/*
 * 主流程说明：
 * 1. 先完成板级硬件、LED、Flash 参数、串口、温控定时器等基础初始化。
 * 2. 开启中断后，SysTick/UART/TIMERG8 只负责投递事件，不在中断里执行重任务。
 * 3. IMU 初始化放在基础通信可用之后，便于零漂整定或异常时通过串口反馈。
 * 4. while(1) 是前台协作式调度器：
 *    - IMU 读取与解算已经放入 TIMERG6 1kHz 最高优先级中断。
 *    - 前台只处理串口命令、串口回传和 100Hz 温控事件。
 *    - 没有待处理事件时进入 WFI，等待下一次中断唤醒。
 */
int main(void)
{
    /* 先初始化板级硬件和状态灯，保证后续异常有基本指示。 */
    Board_InitHardware();
    TaskLED_Init();

    /* 运行参数必须先从 Flash 读出，再按配置启动串口和温控。 */
    RuntimeState_LoadConfig(&gRuntimeState);
    TaskSerial_ApplyBaud(gRuntimeState.runtime_config.baud_rate);
    if (Board_HasExternalClockFault() != 0U) {
        TaskSerial_Write("ERROR:HFXT_STARTUP_TIMEOUT\n"
                         "CLOCK_SOURCE:SYSOSC_PLL\n");
    }
    TaskTemperature_Init(&gRuntimeState);

    /* 从这里开始，SysTick、串口、温控定时器中断可以投递调度事件。 */
    Board_EnableInterrupts();

    /* IMU 初始化可能触发零漂整定，并在完成后进行初始姿态对齐。 */
    TaskIMU_Init(&gRuntimeState);
    if (RuntimeState_SavePendingConfig(&gRuntimeState) == 0U) {
        TaskSerial_Write("ERROR:SAVE_FAILED\n");
    }
    RuntimeState_ClearImuRuntimeStats();

    TaskTemperature_Update100Hz(&gRuntimeState);

    while (1) {
        uint8_t handled = 0U;
        uint8_t output_active;

        /* 第一优先级：处理串口命令，命令期间只暂停回传，不阻塞 1kHz IMU 中断。 */
        TaskSerial_CollectRx();
        if (TaskSerial_PollCommand(&gRuntimeState) != 0U) {
            handled = 1U;
        }

        /* 第二优先级：消费 IMU 中断置位的回传请求，串口阻塞发送不影响 IMU 积分节拍。 */
        TaskIMU_ServiceReport(&gRuntimeState);

        /* 第三优先级：温控任务由 100Hz 定时器标志触发。 */
        if (RuntimeState_TakeTemperatureUpdate() != 0U) {
            TaskTemperature_Update100Hz(&gRuntimeState);
            handled = 1U;
        }

        /* LED 状态独立刷新：错误快闪，等待加热慢闪，温度到位且正常常亮。 */
        output_active = ((int32_t)(gSystemTickMs - gRuntimeState.uart_report_resume_tick) >= 0) ? 1U : 0U;
        TaskLED_UpdateSystemStatus(1U,
                                   TaskIMU_HaveAlarm(&gRuntimeState),
                                   TaskTemperature_IsReady(&gRuntimeState, 2.0f),
                                   output_active);

        /* 没有待处理任务时进入 WFI，等待下一次中断唤醒。 */
        if (handled == 0U) {
            RuntimeState_WaitForInterrupt();
        }
    }
}
