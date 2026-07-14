#include "task_imu.h"

#include "task_led.h"
#include "task_serial.h"
#include "task_temperature.h"
#include "bmi088_mspm0.h"
#include "bmi270_mspm0.h"
#include "board_mspm0.h"
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
#define TEMP_TUNING_FIRMWARE 0U
#define AUTO_CAL_TEMP_TOLERANCE_C 2.0f
#define AUTO_CAL_LOOP_MS 10U
#define AUTO_CAL_MAX_WAIT_MS 300000U
#define IMU_ERROR_REPORT_PERIOD_MS 1000U
#define BINARY_ATTITUDE_HEADER 0xA5U
#define BINARY_ATTITUDE_PACKET_SIZE 8U
#define BINARY_ANGLE_SCALE 100.0f

/*
 * IMU 任务流程：
 * 1. TaskIMU_Init() 初始化 BMI088 与 BMI270/BMI220，读取芯片 ID，并加载或整定陀螺零漂。
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

#if RUNTIME_FEATURE_INS_ENABLED
static void task_imu_service_navigation_start_event(void)
{
    InertialNavigationStartEvent_t event;
    uint8_t have_event;
    char line[48];

    if (InertialNavigation_HaveStartEvent() == 0U) {
        return;
    }

    Board_EnterCritical();
    have_event = InertialNavigation_TakeStartEvent(&event);
    Board_ExitCritical();
    if (have_event == 0U) {
        return;
    }

    (void) snprintf(line, sizeof(line), "INS:STARTED:%lu\n",
                    (unsigned long) event.update_count);
    TaskSerial_Write(line);
    if (event.moving != 0U) {
        TaskSerial_Write("WARN:INS_START_MOVING\n");
    }
    if (event.bias_valid == 0U) {
        TaskSerial_Write("WARN:INS_ACCEL_BIAS_INVALID\n");
    }
    if (event.heading_valid == 0U) {
        TaskSerial_Write("WARN:INS_HEADING_INVALID\n");
    }
}
#endif

/* 将弧度角归一化到 [-180, 180)，并编码为 0.01 度分辨率的 int16。 */
static int16_t binary_encode_angle(float angle_rad)
{
    float angle_deg = angle_rad * RAD2DEG;
    float scaled;

    if ((angle_deg != angle_deg) || (angle_deg > 720.0f) || (angle_deg < -720.0f)) {
        return 0;
    }
    while (angle_deg >= 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }

    scaled = angle_deg * BINARY_ANGLE_SCALE;
    return (scaled >= 0.0f) ? (int16_t)(scaled + 0.5f) : (int16_t)(scaled - 0.5f);
}

/* 按小端顺序写入一个有符号 16 位角度。 */
static void binary_write_i16_le(uint8_t packet[BINARY_ATTITUDE_PACKET_SIZE],
                                uint8_t offset,
                                int16_t value)
{
    uint16_t encoded = (uint16_t)value;

    packet[offset] = (uint8_t)(encoded & 0xFFU);
    packet[offset + 1U] = (uint8_t)(encoded >> 8U);
}

/* 输出 A5 + Pitch/Roll/Yaw(int16, 0.01度, 小端) + 累加和校验。 */
static void task_imu_write_binary_attitude(const IMU_Attitude_t *attitude)
{
    uint8_t packet[BINARY_ATTITUDE_PACKET_SIZE];
    uint8_t checksum = 0U;

    packet[0] = BINARY_ATTITUDE_HEADER;
    binary_write_i16_le(packet, 1U, binary_encode_angle(attitude->Pitch));
    binary_write_i16_le(packet, 3U, binary_encode_angle(attitude->Roll));
    binary_write_i16_le(packet, 5U, binary_encode_angle(attitude->Yaw));

    for (uint8_t i = 0U; i < (BINARY_ATTITUDE_PACKET_SIZE - 1U); i++) {
        checksum = (uint8_t)(checksum + packet[i]);
    }
    packet[BINARY_ATTITUDE_PACKET_SIZE - 1U] = checksum;
    TaskSerial_WriteBytes(packet, BINARY_ATTITUDE_PACKET_SIZE);
}

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
         (state->bmi270_chip_id == BMI220_CHIP_ID_VALUE))) {
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

static void task_imu_write_fault_report(const RuntimeState_t *state)
{
    if ((state->required_imu_mask & IMU_MASK_BMI088) != 0U) {
        if (state->bmi088_present == 0U) {
            TaskSerial_Write("ERROR:BMI088_MISSING\n");
        }
    }

    if ((state->required_imu_mask & IMU_MASK_BMI270) != 0U) {
        if (state->bmi270_present == 0U) {
            TaskSerial_Write("ERROR:BMI270_MISSING\n");
        }
    }

    if ((state->bmi088_present == 0U) && (state->bmi270_present == 0U)) {
        TaskSerial_Write("ERROR:NO_ACTIVE_IMU\n");
    }
}

static void task_imu_delay_ms(uint32_t ms)
{
    Board_DelayMs(ms);
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

static void load_or_calibrate_bias(RuntimeState_t *state)
{
    uint8_t calibration_mask = imu_present_mask(state);

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
                                      calibration_mask,
                                      0U,
                                      GYRO_BIAS_DEFAULT_RECORD_MS,
                                      TaskTemperature_CalibrationService,
                                      state,
                                      GyroBias_CalibrationProgress) != 0U) {
        state->runtime_config.gyro_bias_valid = 1U;
        state->runtime_config.gyro_calibrated_mask |= calibration_mask;
        state->gyro_bias_valid = 1U;
        state->gyro_bias_calibrated = 1U;
        (void) RuntimeConfig_Save(&state->runtime_config);
        TaskSerial_Write("CAL:DONE\n");
    } else {
        TaskSerial_Write("ERROR:CAL_FAILED\n");
    }
#endif
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
    for (uint8_t i = 0U; i < 10U; i++) {
        TaskTemperature_SampleSensorsFromInterrupt(state);
    }
    Board_StartImuTimer();
    TaskIMU_EnableInterruptUpdate((TaskIMU_HaveFault(state) == 0U) ? 1U : 0U);
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

    TaskIMU_UpdateOutputAngles();
}

void TaskIMU_UpdateOutputAngles(void)
{
    ImuAttitude_UpdateOutput();
}

/* 根据模式输出融合三角、调试九角、二进制姿态包或二维惯导位置与速度。 */
void TaskIMU_WriteAngles(const RuntimeState_t *state)
{
    char line[192];
#if TEMP_TUNING_FIRMWARE
    (void) snprintf(line, sizeof(line),
                    "%.3f,%.3f,%.3f,%.3f,%.3f\n",
                    (state->bmi088_temperature_filter_valid != 0U) ?
                        state->bmi088_temperature_filtered : BMI088Sensor.Temperature,
                    BMI270Sensor.Temperature,
                    state->bmi088_heater_duty,
                    state->bmi270_heater_duty,
                    state->runtime_config.target_temperature_c);
    TaskSerial_Write(line);
#else
    INS_t bmi088_result;
    IMU_Attitude_t bmi270_result;
    IMU_Attitude_t virtual_result;

#if RUNTIME_FEATURE_INS_ENABLED
    if (state->runtime_config.output_mode == RUNTIME_OUTPUT_MODE_INS) {
        InertialNavigationSnapshot_t navigation;

        TaskIMU_GetNavigationSnapshot(&navigation);
        if (navigation.state == INERTIAL_NAVIGATION_RUNNING) {
            (void) snprintf(line, sizeof(line),
                            "%.3f,%.3f,%.3f,%.3f\n",
                            navigation.position_m[0],
                            navigation.position_m[1],
                            navigation.velocity_mps[0],
                            navigation.velocity_mps[1]);
            TaskSerial_Write(line);
        }
        return;
    }
#endif

    if (state->runtime_config.output_mode == RUNTIME_OUTPUT_MODE_BINARY) {
        Board_EnterCritical();
        ImuAttitude_GetFusedResult(&virtual_result);
        Board_ExitCritical();
        task_imu_write_binary_attitude(&virtual_result);
        return;
    }

    Board_EnterCritical();
    ImuAttitude_GetBMI088Result(&bmi088_result);
    ImuAttitude_GetBMI270Result(&bmi270_result);
    ImuAttitude_GetFusedResult(&virtual_result);
    Board_ExitCritical();

    if (state->runtime_config.output_mode == RUNTIME_OUTPUT_MODE_DEBUG) {
        (void) snprintf(line, sizeof(line),
                        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                        bmi088_result.Pitch * RAD2DEG,
                        bmi088_result.Roll * RAD2DEG,
                        bmi088_result.Yaw * RAD2DEG,
                        bmi270_result.Pitch * RAD2DEG,
                        bmi270_result.Roll * RAD2DEG,
                        bmi270_result.Yaw * RAD2DEG,
                        virtual_result.Pitch * RAD2DEG,
                        virtual_result.Roll * RAD2DEG,
                        virtual_result.Yaw * RAD2DEG);
    } else {
        (void) snprintf(line, sizeof(line),
                        "%.3f,%.3f,%.3f\n",
                        virtual_result.Pitch * RAD2DEG,
                        virtual_result.Roll * RAD2DEG,
                        virtual_result.Yaw * RAD2DEG);
    }
    TaskSerial_Write(line);
#endif
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
    static uint32_t last_error_report_tick;
#if RUNTIME_FEATURE_INS_ENABLED
    static uint32_t last_navigation_notice_tick;

    task_imu_service_navigation_start_event();
#endif

    if (TaskIMU_HaveFault(state) != 0U) {
        TaskLED_UpdateSystemStatus(1U, 1U, 0U, 1U);
        if ((int32_t)(gSystemTickMs - state->uart_report_resume_tick) >= 0) {
            if ((uint32_t)(gSystemTickMs - last_error_report_tick) >= IMU_ERROR_REPORT_PERIOD_MS) {
                last_error_report_tick = gSystemTickMs;
                task_imu_write_fault_report(state);
            }
        }
        return;
    }

    Board_EnterCritical();
    report_pending = gImuReportPending;
    gImuReportPending = 0U;
    Board_ExitCritical();

    if (report_pending == 0U) {
        return;
    }

#if RUNTIME_FEATURE_INS_ENABLED
    if (state->runtime_config.output_mode == RUNTIME_OUTPUT_MODE_INS) {
        InertialNavigationSnapshot_t navigation;

        TaskIMU_GetNavigationSnapshot(&navigation);
        if (navigation.state != INERTIAL_NAVIGATION_RUNNING) {
            if (((int32_t)(gSystemTickMs - state->uart_report_resume_tick) >= 0) &&
                ((uint32_t)(gSystemTickMs - last_navigation_notice_tick) >=
                 IMU_ERROR_REPORT_PERIOD_MS)) {
                last_navigation_notice_tick = gSystemTickMs;
                if (navigation.state == INERTIAL_NAVIGATION_FAULT) {
                    TaskSerial_Write("ERROR:INS_DATA_INVALID\n");
                } else if (navigation.state == INERTIAL_NAVIGATION_ALIGNING) {
                    char line[48];

                    (void) snprintf(line, sizeof(line), "INS:ALIGNING:%u/1000\n",
                                    navigation.alignment_samples);
                    TaskSerial_Write(line);
                } else {
                    TaskSerial_Write("INS:WAIT_START\n");
                }
            }
            return;
        }
    }
#endif

    if ((int32_t)(gSystemTickMs - state->uart_report_resume_tick) >= 0) {
        TaskLED_UpdateSystemStatus(1U,
                                   TaskIMU_HaveAlarm(state),
                                   TaskTemperature_IsReady(state, 2.0f),
                                   1U);
        TaskIMU_WriteAngles(state);
    } else {
        TaskLED_Set(0U);
    }
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

