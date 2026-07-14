#include "runtime_state.h"

#include "ti_msp_dl_config.h"

RuntimeState_t gRuntimeState;
volatile uint32_t gSystemTickMs;
volatile ImuRuntimeStats_t gImuRuntimeStats;

/*
 * 运行状态模块流程：
 * 1. 保存全局运行状态 gRuntimeState，包括配置参数、错误状态、温控状态和输出分频。
 * 2. SysTick 中断调用 RuntimeState_OnSysTick()，只维护系统毫秒时间。
 * 3. TIMERG6 1kHz 中断直接执行 IMU 读取与解算，保证积分周期不受前台任务影响。
 * 4. TIMERG8 中断调用 RuntimeState_OnTemperatureTick()，置位 100Hz 温控标志。
 * 5. main 负责串口命令、串口输出和温控等低优先级工作。
 */
static volatile uint8_t gPendingTemperatureUpdate;

/* 从 Flash 加载运行配置，并根据配置计算串口回传分频。 */
void RuntimeState_LoadConfig(RuntimeState_t *ctx)
{
    (void) RuntimeConfig_Load(&ctx->runtime_config);
    ctx->gyro_bias_valid = ctx->runtime_config.gyro_bias_valid;
    RuntimeState_UpdateReportRate(ctx);
}

/* 根据模式和 report_rate_hz 计算 IMU 1kHz 更新到串口输出之间的整数分频。 */
void RuntimeState_UpdateReportRate(RuntimeState_t *ctx)
{
    uint32_t rate = ctx->runtime_config.report_rate_hz;

    if (RuntimeConfig_ReportRateIsSupported(ctx->runtime_config.output_mode, rate) == 0U) {
        rate = RUNTIME_CONFIG_DEFAULT_REPORT_RATE_HZ;
        ctx->runtime_config.report_rate_hz = rate;
    }

    ctx->uart_report_ticks = IMU_UPDATE_RATE_HZ / rate;
}

/* 限制可用波特率，避免命令设置到当前时钟配置无法稳定支持的数值。 */
uint8_t RuntimeState_BaudRateIsSupported(uint32_t baud_rate)
{
    return ((baud_rate == 115200U) ||
            (baud_rate == 230400U) ||
            (baud_rate == 460800U) ||
            (baud_rate == 921600U));
}

/* SysTick 1ms 中断入口调用：只记录系统毫秒时间，IMU 更新由 TIMERG6 最高优先级中断完成。 */
void RuntimeState_OnSysTick(void)
{
    gSystemTickMs++;
}

/* 100Hz 温控定时器中断入口调用：只置位温控待处理标志。 */
void RuntimeState_OnTemperatureTick(void)
{
    gPendingTemperatureUpdate = 1U;
}

/* 前台取走温控更新标志；返回 1 表示需要执行一次温控任务。 */
uint8_t RuntimeState_TakeTemperatureUpdate(void)
{
    if (gPendingTemperatureUpdate == 0U) {
        return 0U;
    }

    gPendingTemperatureUpdate = 0U;
    return 1U;
}

/* 清空启动或整定期间的 IMU 统计兼容入口。 */
void RuntimeState_ClearImuUpdates(void)
{
    RuntimeState_ClearImuRuntimeStats();
}

/* IMU 初始化完成后清空运行统计，让 STATUS 只反映稳定运行阶段的负载情况。 */
void RuntimeState_ClearImuRuntimeStats(void)
{
    __disable_irq();
    gImuRuntimeStats.update_count = 0U;
    gImuRuntimeStats.overrun_count = 0U;
    gImuRuntimeStats.reserved[0] = 0U;
    gImuRuntimeStats.reserved[1] = 0U;
    __enable_irq();
}

/* 前台空闲时进入低功耗等待，直到下一次中断唤醒。 */
void RuntimeState_WaitForInterrupt(void)
{
    __WFI();
}

