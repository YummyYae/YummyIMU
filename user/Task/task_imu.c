#include "task_imu.h"

#include "task_led.h"
#include "task_serial.h"
#include "task_temperature.h"
#include "bmi088_mspm0.h"
#include "bmi270_mspm0.h"
#include "flash_storage.h"
#include "imu_attitude.h"
#include "ti_msp_dl_config.h"

#include <stdio.h>

#define STARTUP_ALIGN_SAMPLES 300U
#define STARTUP_ALIGN_SAMPLE_MS 1U
#define STARTUP_SETTLE_SAMPLES 1000U

/*
 * IMU 任务流程：
 * 1. TaskIMU_Init() 初始化 BMI088 与 BMI270/BMI220，读取芯片 ID，并加载或整定零漂。
 * 2. 初始化姿态算法后，TaskIMU_AlignInitialAttitude() 采样静止加速度，建立初始姿态。
 * 3. 运行阶段由 TIMERG6 1kHz 中断直接调用 TaskIMU_UpdateFromInterrupt()，在最高优先级里完成 SPI 读取和姿态积分。
 * 4. 中断只置位“需要回传”的标志，串口格式化和阻塞发送仍由 main 中的 TaskIMU_ServiceReport() 完成。
 * 5. 串口命令、温控和 Flash 操作都不会再改变 IMU 的积分节拍。
 */

static volatile uint8_t gImuInterruptUpdateEnabled;
static volatile uint8_t gImuInterruptBusy;
static volatile uint8_t gImuReportPending;
static uint16_t gUartReportDivider;

static void load_or_calibrate_bias(RuntimeState_t *state)
{
    if (state->gyro_bias_valid != 0U) {
        GyroBias_Apply(&state->runtime_config.gyro_bias);
        state->gyro_bias_valid = 1U;
        return;
    }

    TaskSerial_Write("0,0,0,0,0,0,0,0,0\n");
    if (GyroBias_Calibrate(&state->runtime_config.gyro_bias,
                           GYRO_BIAS_DEFAULT_WAIT_MS,
                           GYRO_BIAS_DEFAULT_RECORD_MS) != 0U) {
        state->runtime_config.gyro_bias_valid = 1U;
        state->gyro_bias_valid = 1U;
        state->gyro_bias_calibrated = 1U;
        (void) RuntimeConfig_Save(&state->runtime_config);
    }
}

/*
 * 初始化两颗 IMU 和姿态算法。
 * 如果 Flash 中已有有效零漂，则直接应用；否则执行默认零漂整定。
 */
void TaskIMU_Init(RuntimeState_t *state)
{
    TaskIMU_EnableInterruptUpdate(0U);
    state->bmi088_init_error = BMI088_Init((state->gyro_bias_valid != 0U) ? 0U : 1U);
    state->accel_chip_id = BMI088_ReadAccelChipId();
    state->gyro_chip_id = BMI088_ReadGyroChipId();
    state->bmi270_init_error = BMI270_Init();
    state->bmi270_chip_id = BMI270_ReadChipId();
    load_or_calibrate_bias(state);
    ImuAttitude_SetDebugMode(
        (state->runtime_config.output_mode == RUNTIME_OUTPUT_MODE_DEBUG) ? 1U : 0U);
    ImuAttitude_Init();
    TaskIMU_AlignInitialAttitude();
    for (uint8_t i = 0U; i < 10U; i++) {
        TaskTemperature_SampleSensorsFromInterrupt(state);
    }
    DL_TimerG_startCounter(TIMERG6_1000hz_INST);
    TaskIMU_EnableInterruptUpdate(1U);
}

/* 检查 IMU 初始化结果和芯片 ID，供 LED 状态和异常输出判断使用。 */
uint8_t TaskIMU_HaveFault(const RuntimeState_t *state)
{
    if ((state->bmi088_init_error != 0U) || (state->bmi270_init_error != 0U)) {
        return 1U;
    }

    if ((state->accel_chip_id == 0U) || (state->gyro_chip_id == 0U) || (state->bmi270_chip_id == 0U)) {
        return 1U;
    }

    return 0U;
}

/*
 * 上电和手动整定后调用，用静止重力方向给姿态解算设置初始角。
 * 这里先平均采样加速度，再让姿态算法跑一段 settle，使初始输出更平稳。
 */
void TaskIMU_AlignInitialAttitude(void)
{
    Axis3f bmi088_accel = {0.0f, 0.0f, 0.0f};
    Axis3f bmi270_accel = {0.0f, 0.0f, 0.0f};

    for (uint32_t sample = 0U; sample < STARTUP_ALIGN_SAMPLES; sample++) {
        Axis3f bmi088_sample;
        Axis3f bmi270_sample;

        BMI088_Read(&BMI088Sensor);
        BMI270_Read(&BMI270Sensor);
        bmi270_sample = ImuAttitude_BMI270AccelToBoard();

        bmi088_sample = ImuAttitude_BMI088AccelToBoard();
        bmi088_accel.x += bmi088_sample.x;
        bmi088_accel.y += bmi088_sample.y;
        bmi088_accel.z += bmi088_sample.z;
        bmi270_accel.x += bmi270_sample.x;
        bmi270_accel.y += bmi270_sample.y;
        bmi270_accel.z += bmi270_sample.z;
        DL_Common_delayCycles(CPUCLK_FREQ / 1000U * STARTUP_ALIGN_SAMPLE_MS);
    }

    bmi088_accel.x /= (float)STARTUP_ALIGN_SAMPLES;
    bmi088_accel.y /= (float)STARTUP_ALIGN_SAMPLES;
    bmi088_accel.z /= (float)STARTUP_ALIGN_SAMPLES;
    bmi270_accel.x /= (float)STARTUP_ALIGN_SAMPLES;
    bmi270_accel.y /= (float)STARTUP_ALIGN_SAMPLES;
    bmi270_accel.z /= (float)STARTUP_ALIGN_SAMPLES;

    ImuAttitude_SetInitialAccel(bmi088_accel, bmi270_accel);

    for (uint32_t sample = 0U; sample < STARTUP_SETTLE_SAMPLES; sample++) {
        ImuAttitude_Update(IMU_UPDATE_DT_SECONDS);
        DL_Common_delayCycles(CPUCLK_FREQ / 1000U);
    }

    TaskIMU_UpdateOutputAngles();
}

void TaskIMU_UpdateOutputAngles(void)
{
    ImuAttitude_UpdateOutput();
}

/* 按历史上位机顺序输出九个角度：BMI088、BMI270/BMI220、VirtualIMU 各自的 Pitch、Roll、Yaw。 */
void TaskIMU_WriteAngles(const RuntimeState_t *state)
{
    char line[192];
    float bmi088_pitch;
    float bmi088_roll;
    float bmi088_yaw;
    float bmi270_pitch;
    float bmi270_roll;
    float bmi270_yaw;
    float virtual_pitch;
    float virtual_roll;
    float virtual_yaw;

    __disable_irq();
    bmi088_pitch = BMI088.Pitch;
    bmi088_roll = BMI088.Roll;
    bmi088_yaw = BMI088.Yaw;
    bmi270_pitch = BMI270.Pitch;
    bmi270_roll = BMI270.Roll;
    bmi270_yaw = BMI270.Yaw;
    virtual_pitch = VirtualIMU.Pitch;
    virtual_roll = VirtualIMU.Roll;
    virtual_yaw = VirtualIMU.Yaw;
    __enable_irq();

    if (state->runtime_config.output_mode == RUNTIME_OUTPUT_MODE_DEBUG) {
        (void) snprintf(line, sizeof(line),
                        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                        bmi088_pitch * RAD2DEG,
                        bmi088_roll * RAD2DEG,
                        bmi088_yaw * RAD2DEG,
                        bmi270_pitch * RAD2DEG,
                        bmi270_roll * RAD2DEG,
                        bmi270_yaw * RAD2DEG,
                        virtual_pitch * RAD2DEG,
                        virtual_roll * RAD2DEG,
                        virtual_yaw * RAD2DEG);
    } else {
        (void) snprintf(line, sizeof(line),
                        "%.3f,%.3f,%.3f\n",
                        virtual_pitch * RAD2DEG,
                        virtual_roll * RAD2DEG,
                        virtual_yaw * RAD2DEG);
    }
    TaskSerial_Write(line);
}

/*
 * 允许或禁止 1kHz 中断直接更新 IMU。
 * 初始化、零漂整定和 STATUS 等前台 SPI 操作期间必须先关闭，避免中断与前台同时访问两路 SPI。
 */
void TaskIMU_EnableInterruptUpdate(uint8_t enable)
{
    __disable_irq();
    gImuInterruptUpdateEnabled = (enable != 0U) ? 1U : 0U;
    gImuInterruptBusy = 0U;
    gImuReportPending = 0U;
    gUartReportDivider = 0U;
    __enable_irq();
}

/*
 * 最高优先级 1kHz 中断入口。
 * 这里只做确定周期的工作：双 IMU SPI 读取、真实/虚拟姿态积分、输出分频标志。
 * 禁止在这里做 printf、Flash、串口阻塞发送或温控 SPI 读取。
 */
void TaskIMU_UpdateFromInterrupt(RuntimeState_t *state)
{
    if (gImuInterruptUpdateEnabled == 0U) {
        return;
    }
    if (gImuInterruptBusy != 0U) {
        gImuRuntimeStats.overrun_count++;
        return;
    }
    gImuInterruptBusy = 1U;

    gImuRuntimeStats.update_count++;

    ImuAttitude_Update(IMU_UPDATE_DT_SECONDS);
    TaskTemperature_SampleSensorsFromInterrupt(state);

    gUartReportDivider++;
    if (gUartReportDivider >= state->uart_report_ticks) {
        gUartReportDivider = 0U;
        ImuAttitude_UpdateOutput();
        gImuReportPending = 1U;
    }

    gImuInterruptBusy = 0U;
}

/*
 * 前台兼容入口。
 * 当前 IMU 已迁移到中断更新，本函数只保留给旧调度路径兜底，不在 main 中主动调用。
 */
void TaskIMU_Update1kHz(RuntimeState_t *state)
{
    TaskIMU_UpdateFromInterrupt(state);
}

/*
 * 前台回传服务。
 * main 循环调用该函数消费中断置位的回传标志，完成 LED 更新和串口发送。
 */
void TaskIMU_ServiceReport(RuntimeState_t *state)
{
    uint8_t report_pending;

    __disable_irq();
    report_pending = gImuReportPending;
    gImuReportPending = 0U;
    __enable_irq();

    if (report_pending == 0U) {
        return;
    }

    if ((int32_t)(gSystemTickMs - state->uart_report_resume_tick) >= 0) {
        TaskLED_Refresh(TaskIMU_HaveFault(state) == 0U, 1U);
        TaskIMU_WriteAngles(state);
    } else {
        TaskLED_Set(0U);
    }
}

