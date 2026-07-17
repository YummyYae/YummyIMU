#include "task_imu.h"

#include "task_led.h"
#include "serial_transport.h"
#include "task_temperature.h"
#include "imu_report.h"
#include "bmi088_mspm0.h"
#include "bmi270_mspm0.h"
#include "board_mspm0.h"
#include "flash_storage.h"
#include "accel_six_calibration.h"
#include "gyro_bias_calibration.h"
#if RUNTIME_FEATURE_INS_ENABLED
#include "inertial_navigation.h"
#endif
#include "imu_attitude.h"

#include <stdio.h>

#define STARTUP_ALIGN_SAMPLES 300U
#define STARTUP_ALIGN_SAMPLE_MS 1U
#define STARTUP_SETTLE_SAMPLES 1000U
#define BMI088_ACCEL_CHIP_ID 0x1EU
#define BMI088_GYRO_CHIP_ID  0x0FU
#define AUTO_CAL_TEMP_TOLERANCE_C 2.0f
#define AUTO_CAL_LOOP_MS 10U
#define AUTO_CAL_MAX_WAIT_MS 300000U
#define IMU_FAULT_RETRY_PERIOD_MS 1000U

/*
 * IMU 任务流程：
 * 1. TaskIMU_Init() 初始化 BMI088 与 BMI270/BMI260/BMI220，读取芯片 ID，并加载或整定陀螺零漂。
 * 2. 从 Flash 加载六面加速度参数后，TaskIMU_AlignInitialAttitude() 使用同一份校正数据建立初始姿态。
 * 3. 运行阶段由 TIMERG6 1kHz 中断直接调用 TaskIMU_UpdateFromInterrupt()，在最高优先级里完成 SPI 读取和姿态积分。
 * 4. INS 模式在同一中断中完成融合采样的二维坐标变换、速度积分和位置积分。
 * 5. 中断只置位“需要回传”的标志，串口格式化和阻塞发送仍由 main 中的 TaskIMU_ServiceReport() 完成。
 * 6. 串口命令、温控和 Flash 操作都不会改变 IMU 的积分节拍。
 */

static volatile uint8_t gImuInterruptUpdateEnabled;
static volatile uint8_t gImuReportPending;
static uint16_t gUartReportDivider;
static uint8_t gTaskImuActiveMask;
static uint32_t gImuFaultRetryTick;

static uint8_t imu_source_required_mask(uint8_t source)
{
    if (source == RUNTIME_IMU_SOURCE_BMI088) {
        return IMU_MASK_BMI088;
    }

    if (source == RUNTIME_IMU_SOURCE_BMI270) {
        return IMU_MASK_BMI270;
    }

    if (source == RUNTIME_IMU_SOURCE_AUTO) {
        return 0U;
    }

    return IMU_MASK_DUAL;
}

static uint8_t imu_present_mask(const RuntimeState_t *state)
{
    uint8_t mask = 0U;

    if ((state->bmi088_init_error == 0U) &&
        (state->accel_chip_id == BMI088_ACCEL_CHIP_ID) &&
        (state->gyro_chip_id == BMI088_GYRO_CHIP_ID)) {
        mask |= IMU_MASK_BMI088;
    }

    if ((state->bmi270_init_error == 0U) &&
        ((state->bmi270_chip_id == BMI270_CHIP_ID_VALUE) ||
         (state->bmi270_chip_id == BMI220_CHIP_ID_VALUE) ||
         (state->bmi270_chip_id == BMI260_CHIP_ID_VALUE))) {
        mask |= IMU_MASK_BMI270;
    }

    return mask;
}

static void imu_update_selection(RuntimeState_t *state)
{
    uint8_t present_mask = imu_present_mask(state);
    uint8_t required_mask = imu_source_required_mask(state->runtime_config.imu_source);
    uint8_t active_mask;

    state->bmi088_present = ((present_mask & IMU_MASK_BMI088) != 0U) ? 1U : 0U;
    state->bmi270_present = ((present_mask & IMU_MASK_BMI270) != 0U) ? 1U : 0U;
    state->required_imu_mask = required_mask;

    if (state->runtime_config.imu_source == RUNTIME_IMU_SOURCE_AUTO) {
        active_mask = present_mask;
    } else {
        active_mask = ((present_mask & required_mask) == required_mask) ? required_mask : 0U;
    }

    state->active_imu_mask = active_mask;
    gTaskImuActiveMask = active_mask;
    if (active_mask != 0U) {
        ImuAttitude_SetInputMask(active_mask);
    }
}

static uint8_t imu_identity_has_fault(const RuntimeState_t *state)
{
    if ((state->required_imu_mask != 0U) &&
        ((state->active_imu_mask & state->required_imu_mask) != state->required_imu_mask)) {
        return 1U;
    }

    if (state->active_imu_mask == 0U) {
        return 1U;
    }

    return 0U;
}


static void task_imu_delay_ms(uint32_t ms)
{
    Board_DelayMs(ms);
}

/* 自动零漂整定期间每秒回传剩余时间，并维持校准灯态。 */
static void task_imu_calibration_progress(uint32_t elapsed_ms, uint32_t total_ms)
{
    static uint32_t last_remaining_s;
    uint32_t remaining_s = (total_ms - elapsed_ms + 999U) / 1000U;

    TaskLED_UpdateCalibrationStatus();

    if ((elapsed_ms == 0U) || (remaining_s != last_remaining_s)) {
        char line[64];

        last_remaining_s = remaining_s;
        (void) snprintf(line, sizeof(line), "CAL_REMAIN:%u\n", remaining_s);
        TaskSerial_Write(line);
    }
}

static uint8_t auto_calibration_wait_ready(RuntimeState_t *state)
{
    uint32_t elapsed_ms = 0U;
    char line[128];

    if (imu_identity_has_fault(state) != 0U) {
        TaskSerial_Write("ERROR:IMU_INIT_FAILED\n");
        return 0U;
    }

    TaskSerial_Write("CAL_WAIT:AUTO_TEMP\n");

    while (TaskTemperature_IsReady(state, AUTO_CAL_TEMP_TOLERANCE_C) == 0U) {
        for (uint8_t i = 0U; i < AUTO_CAL_LOOP_MS; i++) {
            TaskTemperature_CalibrationService(state);
            TaskLED_UpdateCalibrationStatus();
            task_imu_delay_ms(1U);
        }

        elapsed_ms += AUTO_CAL_LOOP_MS;
        if (elapsed_ms >= AUTO_CAL_MAX_WAIT_MS) {
            TaskSerial_Write("ERROR:TEMP_WAIT_TIMEOUT\n");
            return 0U;
        }
        if ((elapsed_ms % 1000U) == 0U) {
            (void) snprintf(line, sizeof(line),
                            "CAL_WAIT:T088=%.3f,T270=%.3f\n",
                            (state->bmi088_temperature_filter_valid != 0U) ?
                                state->bmi088_temperature_filtered : BMI088Sensor.Temperature,
                            (state->bmi270_temperature_filter_valid != 0U) ?
                                state->bmi270_temperature_filtered : BMI270Sensor.Temperature);
            TaskSerial_Write(line);
        }
    }

    TaskSerial_Write("CAL_WAIT:DONE\n");
    return 1U;
}

static void task_imu_write_calibration_scores(const RuntimeConfig_t *config,
                                              uint8_t sensor_mask)
{
    char line[64];

    if ((sensor_mask & IMU_BIAS_SENSOR_BMI088) != 0U) {
        (void) snprintf(line, sizeof(line), "BMI088_GYRO_SCORE:%u\n",
                        config->bmi088_gyro_score);
        TaskSerial_Write(line);
    }
    if ((sensor_mask & IMU_BIAS_SENSOR_BMI270) != 0U) {
        (void) snprintf(line, sizeof(line), "BMI270_GYRO_SCORE:%u\n",
                        config->bmi270_gyro_score);
        TaskSerial_Write(line);
    }
}

static void load_or_calibrate_bias(RuntimeState_t *state)
{
    uint8_t calibration_mask = imu_present_mask(state);
    GyroQualityData_t quality;

    if ((state->runtime_config.gyro_bias_valid != 0U) &&
        ((state->runtime_config.gyro_calibrated_mask & state->active_imu_mask) ==
         state->active_imu_mask)) {
        GyroBias_Apply(&state->runtime_config.gyro_bias);
        state->gyro_bias_valid = 1U;
        return;
    }

#if TEMP_TUNING_FIRMWARE
    /*
     * 温控调试临时固件：Keil 全片擦除后 Flash 零漂会丢失。
     * 此版本只用于循环观察温度/PWM，不等待 40 秒零漂整定，避免上电后长时间不输出温度。
     */
    for (uint8_t axis = 0U; axis < 3U; axis++) {
        state->runtime_config.gyro_bias.bmi088[axis] = 0.0f;
        state->runtime_config.gyro_bias.bmi270[axis] = 0.0f;
    }
    GyroBias_Apply(&state->runtime_config.gyro_bias);
    return;
#else
    if (auto_calibration_wait_ready(state) == 0U) {
        return;
    }
    TaskSerial_Write("CAL:START\n");
    TaskLED_UpdateCalibrationStatus();
    if (GyroBias_CalibrateWithService(&state->runtime_config.gyro_bias,
                                      &quality,
                                      calibration_mask,
                                      0U,
                                      GYRO_BIAS_DEFAULT_RECORD_MS,
                                      TaskTemperature_CalibrationService,
                                      state,
                                      task_imu_calibration_progress) != 0U) {
        GyroBias_StoreQuality(&state->runtime_config, &quality, calibration_mask);
        state->runtime_config.gyro_bias_valid = 1U;
        state->runtime_config.gyro_calibrated_mask |= calibration_mask;
        state->gyro_bias_valid = 1U;
        state->gyro_bias_calibrated = 1U;
        TaskSerial_Write("CAL:DONE\n");
        task_imu_write_calibration_scores(&state->runtime_config, calibration_mask);
        if (RuntimeConfig_Save(&state->runtime_config) == 0U) {
            TaskSerial_Write("ERROR:SAVE_FAILED\n");
        }
    } else {
        TaskSerial_Write("ERROR:CAL_FAILED\n");
    }
#endif
}

/*
 * 按当前配置重新建立姿态算法状态。
 * 上电初始化和故障恢复共用该入口，确保量程补偿、安装方向和初始姿态完全一致。
 */
static void task_imu_initialize_algorithms(RuntimeState_t *state)
{
    ImuAttitude_SetAccelCalibration(state->runtime_config.accel_bias.bmi088,
                                    state->runtime_config.accel_bias.bmi270,
                                    state->runtime_config.accel_scale.bmi088,
                                    state->runtime_config.accel_scale.bmi270,
                                    state->runtime_config.accel_bias_valid);
    ImuAttitude_SetYawErrorDegPerTurn(state->runtime_config.bmi088_yaw_error_deg_per_turn,
                                      state->runtime_config.bmi270_yaw_error_deg_per_turn);
    ImuAttitude_SetDebugMode(
        (state->runtime_config.output_mode == RUNTIME_OUTPUT_MODE_DEBUG) ? 1U : 0U);
    ImuAttitude_Init();
    AccelSixCalibration_Reset();
    if (TaskIMU_HaveFault(state) == 0U) {
        TaskIMU_AlignInitialAttitude();
    }
#if RUNTIME_FEATURE_INS_ENABLED
    InertialNavigation_Init();
#endif
}

/* 返回当前故障状态下需要重新初始化的 IMU，只重试必需且尚未可用的通道。 */
static uint8_t task_imu_fault_retry_mask(const RuntimeState_t *state)
{
    uint8_t required_mask = state->required_imu_mask;

    /* AUTO 模式只有在两颗 IMU 都不可用时才会进入故障状态，此时同时重新探测两路。 */
    if (required_mask == 0U) {
        required_mask = IMU_MASK_DUAL;
    }

    return required_mask & (uint8_t)(~imu_present_mask(state));
}

/*
 * 故障恢复流程：
 * 1. 每次只重新初始化异常通道，并刷新用于串口诊断的芯片 ID；
 * 2. 仍有故障时保持 1kHz 更新关闭，下一秒继续尝试；
 * 3. 全部通道恢复后重新应用零漂、重建姿态初值并恢复中断采样。
 */
static uint8_t task_imu_retry_faulted_devices(RuntimeState_t *state)
{
    uint8_t probe_mask = state->required_imu_mask;
    uint8_t retry_mask;

    TaskIMU_EnableInterruptUpdate(0U);

    if (probe_mask == 0U) {
        probe_mask = IMU_MASK_DUAL;
    }

    /* 每轮先刷新所有必需通道的 ID，防止把等待期间新出现的掉线当成仍然在线。 */
    if ((probe_mask & IMU_MASK_BMI088) != 0U) {
        state->accel_chip_id = BMI088_ReadAccelChipId();
        state->gyro_chip_id = BMI088_ReadGyroChipId();
    }
    if ((probe_mask & IMU_MASK_BMI270) != 0U) {
        state->bmi270_chip_id = BMI270_ReadChipId();
    }

    retry_mask = task_imu_fault_retry_mask(state);
    if ((retry_mask & IMU_MASK_BMI088) != 0U) {
        /* 故障重试不执行 BMI088 驱动内部的临时零漂整定。 */
        state->bmi088_init_error = BMI088_Init(0U);
        state->accel_chip_id = BMI088_ReadAccelChipId();
        state->gyro_chip_id = BMI088_ReadGyroChipId();
    }

    if ((retry_mask & IMU_MASK_BMI270) != 0U) {
        state->bmi270_init_error = BMI270_Init();
        state->bmi270_chip_id = BMI270_ReadChipId();
    }

    imu_update_selection(state);
    if (TaskIMU_HaveFault(state) != 0U) {
        return 0U;
    }

    load_or_calibrate_bias(state);
    task_imu_initialize_algorithms(state);
    for (uint8_t i = 0U; i < 10U; i++) {
        TaskTemperature_SampleSensorsFromInterrupt(state);
    }
    TaskIMU_EnableInterruptUpdate(1U);
    TaskSerial_Write("IMU:RECOVERED\n");
    return 1U;
}

/* 在前台按 1 秒周期触发故障恢复，避免故障输出循环占满 SPI 和 CPU。 */
static uint8_t task_imu_service_fault_recovery(RuntimeState_t *state)
{
    uint32_t now = gSystemTickMs;
    uint8_t recovered;

    if ((uint32_t)(now - gImuFaultRetryTick) < IMU_FAULT_RETRY_PERIOD_MS) {
        return 0U;
    }

    recovered = task_imu_retry_faulted_devices(state);
    /* 从本次尝试结束时重新计时，给串口、温控和 LED 前台任务留出运行窗口。 */
    gImuFaultRetryTick = gSystemTickMs;
    return recovered;
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
    imu_update_selection(state);
    if (TaskIMU_HaveFault(state) == 0U) {
        load_or_calibrate_bias(state);
    }
    task_imu_initialize_algorithms(state);
    for (uint8_t i = 0U; i < 10U; i++) {
        TaskTemperature_SampleSensorsFromInterrupt(state);
    }
    Board_StartImuTimer();
    TaskIMU_EnableInterruptUpdate((TaskIMU_HaveFault(state) == 0U) ? 1U : 0U);
    gImuFaultRetryTick = gSystemTickMs;
}

/* 检查 IMU 初始化结果和芯片 ID，供 LED 状态和异常输出判断使用。 */
uint8_t TaskIMU_HaveFault(const RuntimeState_t *state)
{
    return (imu_identity_has_fault(state) != 0U) ? 1U : 0U;
}

/* 判断芯片已启动后的传感器报警状态：IMU 初始化失败、ID 读不到或温度异常都需要闪灯。 */
uint8_t TaskIMU_HaveAlarm(const RuntimeState_t *state)
{
    if (TaskIMU_HaveFault(state) != 0U) {
        return 1U;
    }

    if (TaskTemperature_HaveFault(state) != 0U) {
        return 1U;
    }

#if RUNTIME_FEATURE_INS_ENABLED
    if ((state->runtime_config.output_mode == RUNTIME_OUTPUT_MODE_INS) &&
        (InertialNavigation_HaveFault() != 0U)) {
        return 1U;
    }
#endif

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

        if ((gTaskImuActiveMask & IMU_MASK_BMI088) != 0U) {
            BMI088_Read(&BMI088Sensor);
            bmi088_sample = ImuAttitude_BMI088AccelToBoard();
        } else {
            bmi088_sample.x = 0.0f;
            bmi088_sample.y = 0.0f;
            bmi088_sample.z = 9.80665f;
        }

        if ((gTaskImuActiveMask & IMU_MASK_BMI270) != 0U) {
            BMI270_Read(&BMI270Sensor);
            bmi270_sample = ImuAttitude_BMI270AccelToBoard();
        } else {
            bmi270_sample.x = 0.0f;
            bmi270_sample.y = 0.0f;
            bmi270_sample.z = 9.80665f;
        }

        bmi088_accel.x += bmi088_sample.x;
        bmi088_accel.y += bmi088_sample.y;
        bmi088_accel.z += bmi088_sample.z;
        bmi270_accel.x += bmi270_sample.x;
        bmi270_accel.y += bmi270_sample.y;
        bmi270_accel.z += bmi270_sample.z;
        Board_DelayMs(STARTUP_ALIGN_SAMPLE_MS);
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
        Board_DelayMs(1U);
    }

    ImuAttitude_UpdateOutput();
}

/*
 * 允许或禁止 1kHz 中断直接更新 IMU。
 * 初始化、零漂整定和 STATUS 等前台 SPI 操作期间必须先关闭，避免中断与前台同时访问两路 SPI。
 */
void TaskIMU_EnableInterruptUpdate(uint8_t enable)
{
    Board_EnterCritical();
    gImuInterruptUpdateEnabled = (enable != 0U) ? 1U : 0U;
    gImuReportPending = 0U;
    gUartReportDivider = 0U;
    Board_ExitCritical();
}

/*
 * 最高优先级 1kHz 中断入口。
 * 这里只做确定周期的工作：双 IMU SPI 读取、姿态/二维惯导积分和输出分频标志。
 * 禁止在这里做 printf、Flash、串口阻塞发送或温控 SPI 读取。
 */
void TaskIMU_UpdateFromInterrupt(RuntimeState_t *state)
{
#if RUNTIME_FEATURE_INS_ENABLED
    ImuFusedSample_t fused_sample;
#endif
    uint32_t start_count;
    uint32_t end_count;
    uint32_t elapsed_ticks;

    if (gImuInterruptUpdateEnabled == 0U) {
        return;
    }
    if (state->active_imu_mask == 0U) {
        return;
    }
    start_count = Board_GetImuTimerCount();

    gImuRuntimeStats.update_count++;

    ImuAttitude_Update(IMU_UPDATE_DT_SECONDS);
#if RUNTIME_FEATURE_INS_ENABLED
    if (state->runtime_config.output_mode == RUNTIME_OUTPUT_MODE_INS) {
        ImuAttitude_GetFusedSample(&fused_sample);
        InertialNavigation_Update(&fused_sample,
                                  IMU_UPDATE_DT_SECONDS,
                                  gImuRuntimeStats.update_count);
    }
#endif
    TaskTemperature_SampleSensorsFromInterrupt(state);
    TaskTemperature_AccumulateDriftFromInterrupt(state);

    gUartReportDivider++;
    if (gUartReportDivider >= state->uart_report_ticks) {
        gUartReportDivider = 0U;
#if RUNTIME_FEATURE_INS_ENABLED
        if (state->runtime_config.output_mode != RUNTIME_OUTPUT_MODE_INS) {
            ImuAttitude_UpdateOutput();
        }
#else
        ImuAttitude_UpdateOutput();
#endif
        gImuReportPending = 1U;
    }

    end_count = Board_GetImuTimerCount();
    elapsed_ticks = Board_GetImuTimerElapsedTicks(start_count, end_count);
    if (elapsed_ticks > Board_GetImuInterruptBudgetTicks()) {
        gImuRuntimeStats.overrun_count++;
    }

}

/*
 * 前台回传服务。
 * main 循环调用该函数消费中断置位的回传标志，完成 LED 更新和串口发送。
 */
void TaskIMU_ServiceReport(RuntimeState_t *state)
{
    uint8_t report_pending;
    uint8_t imu_fault;

    /* 温漂扫描期间串口只保留 TDRIFT 协议，避免姿态数据混入曲线采集流。 */
    if (TaskTemperature_IsDriftTestActive() != 0U) {
        Board_EnterCritical();
        gImuReportPending = 0U;
        Board_ExitCritical();
        return;
    }

    imu_fault = TaskIMU_HaveFault(state);
    if (imu_fault != 0U) {
        if (task_imu_service_fault_recovery(state) != 0U) {
            return;
        }
        ImuReport_Service(state, 0U, 1U, 1U);
        return;
    }

    Board_EnterCritical();
    report_pending = gImuReportPending;
    gImuReportPending = 0U;
    Board_ExitCritical();

    ImuReport_Service(state,
                      report_pending,
                      0U,
                      TaskIMU_HaveAlarm(state));
}

#if RUNTIME_FEATURE_INS_ENABLED
/* INS START 的前台入口：只投递请求，真正清零由下一次 1kHz 中断完成。 */
uint8_t TaskIMU_RequestNavigationStart(const RuntimeState_t *state)
{
    if ((state == NULL) ||
        (state->runtime_config.output_mode != RUNTIME_OUTPUT_MODE_INS) ||
        (TaskIMU_HaveFault(state) != 0U)) {
        return 0U;
    }

    InertialNavigation_RequestStart();
    return 1U;
}

/* 主循环读取惯导结果时短暂屏蔽中断，避免复制到一半被 1kHz 更新打断。 */
void TaskIMU_GetNavigationSnapshot(InertialNavigationSnapshot_t *out)
{
    if (out == NULL) {
        return;
    }

    Board_EnterCritical();
    InertialNavigation_GetSnapshot(out);
    Board_ExitCritical();
}
#endif

