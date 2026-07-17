#include "temperature_drift.h"

#include "task_temperature.h"
#include "serial_transport.h"
#include "bmi088_mspm0.h"
#include "bmi270_mspm0.h"
#include "board_mspm0.h"

#include <stddef.h>
#include <stdio.h>

#define TEMPERATURE_DRIFT_END_C                  70.0f
#define TEMPERATURE_DRIFT_MIN_RATE_C_PER_MIN     0.1f
#define TEMPERATURE_DRIFT_MAX_RATE_C_PER_MIN     60.0f
#define TEMPERATURE_DRIFT_MAX_REPORT_HZ          10U
#define TEMPERATURE_DRIFT_END_TOLERANCE_C        0.5f
#define TEMPERATURE_DRIFT_END_STABLE_MS          3000U
#define TEMPERATURE_DRIFT_END_TIMEOUT_MS         300000U
#define TEMPERATURE_DRIFT_PHASE_RAMP             1U
#define TEMPERATURE_DRIFT_PHASE_HOLD             2U

typedef struct {
    uint8_t phase;
    uint32_t start_tick;
    uint32_t last_report_tick;
    uint32_t phase_start_tick;
    uint32_t end_stable_start_tick;
    uint32_t ramp_duration_ms;
    uint32_t report_period_ms;
    uint32_t sequence;
    uint32_t total_samples;
    float saved_target_c;
    float start_temperature_c;
    float rate_c_per_min;
} TemperatureDriftTestContext_t;

typedef struct {
    volatile float bmi088_gyro_sum[3];
    volatile float bmi270_gyro_sum[3];
    volatile float bmi088_temperature_sum;
    volatile float bmi270_temperature_sum;
    volatile uint32_t samples;
} TemperatureDriftAccumulator_t;

static TemperatureDriftTestContext_t gTemperatureDriftTest;
static TemperatureDriftAccumulator_t gTemperatureDriftAccumulator;
static volatile uint8_t gTemperatureDriftTestActive;

/*
 * 温漂扫描子模块流程：
 * 1. 1kHz IMU 中断只累加两路原始角速度和对应温度，不进行格式化。
 * 2. 温控任务每 10ms 调用状态机，线性修改 RAM 中的目标温度。
 * 3. 到达回传周期后原子换出采集窗口，并输出固定列顺序的 TDRIFT 数据。
 * 4. STOP、完成或异常退出时恢复测试前目标温度并清空两路 PID 状态。
 * 5. 本模块不写 Flash，不改变永久温控配置。
 */

static void temperature_drift_reset_heater_state(RuntimeState_t *state)
{
    state->bmi088_heater_pid_integral = 0.0f;
    state->bmi088_heater_pid_last_error = 0.0f;
    state->bmi270_heater_pid_integral = 0.0f;
    state->bmi270_heater_pid_last_error = 0.0f;
}

/* 清空温漂测试的 1kHz 窗口累加器；调用者必须保证当前处于临界区。 */
static void temperature_drift_clear_accumulator(void)
{
    for (uint8_t axis = 0U; axis < 3U; axis++) {
        gTemperatureDriftAccumulator.bmi088_gyro_sum[axis] = 0.0f;
        gTemperatureDriftAccumulator.bmi270_gyro_sum[axis] = 0.0f;
    }
    gTemperatureDriftAccumulator.bmi088_temperature_sum = 0.0f;
    gTemperatureDriftAccumulator.bmi270_temperature_sum = 0.0f;
    gTemperatureDriftAccumulator.samples = 0U;
}

/*
 * 原子地取走一个采集窗口并换算为均值。
 * deactivate 非零时先关闭中断侧累积，用于 STOP、DONE 和异常退出时取得最后一段数据。
 */
static uint32_t temperature_drift_take_window(float bmi088_gyro[3],
                                              float bmi270_gyro[3],
                                              float *bmi088_temperature,
                                              float *bmi270_temperature,
                                              uint8_t deactivate)
{
    float bmi088_sum[3];
    float bmi270_sum[3];
    float bmi088_temperature_sum;
    float bmi270_temperature_sum;
    uint32_t samples;

    Board_EnterCritical();
    if (deactivate != 0U) {
        gTemperatureDriftTestActive = 0U;
    }
    samples = gTemperatureDriftAccumulator.samples;
    for (uint8_t axis = 0U; axis < 3U; axis++) {
        bmi088_sum[axis] = gTemperatureDriftAccumulator.bmi088_gyro_sum[axis];
        bmi270_sum[axis] = gTemperatureDriftAccumulator.bmi270_gyro_sum[axis];
    }
    bmi088_temperature_sum = gTemperatureDriftAccumulator.bmi088_temperature_sum;
    bmi270_temperature_sum = gTemperatureDriftAccumulator.bmi270_temperature_sum;
    temperature_drift_clear_accumulator();
    Board_ExitCritical();

    if (samples == 0U) {
        return 0U;
    }

    for (uint8_t axis = 0U; axis < 3U; axis++) {
        bmi088_gyro[axis] = bmi088_sum[axis] / (float)samples;
        bmi270_gyro[axis] = bmi270_sum[axis] / (float)samples;
    }
    *bmi088_temperature = bmi088_temperature_sum / (float)samples;
    *bmi270_temperature = bmi270_temperature_sum / (float)samples;
    return samples;
}

/* 输出一个固定列顺序的 CSV 数据行，供上位机直接按温度绘制六条零漂曲线。 */
static void temperature_drift_write_data(const RuntimeState_t *state,
                                         uint32_t now,
                                         uint8_t deactivate)
{
    float bmi088_gyro[3];
    float bmi270_gyro[3];
    float bmi088_temperature;
    float bmi270_temperature;
    uint32_t samples;
    char line[256];

    samples = temperature_drift_take_window(bmi088_gyro,
                                            bmi270_gyro,
                                            &bmi088_temperature,
                                            &bmi270_temperature,
                                            deactivate);
    if (samples == 0U) {
        return;
    }

    gTemperatureDriftTest.total_samples += samples;
    (void) snprintf(line, sizeof(line),
                    "TDRIFT:DATA,%lu,%lu,%.3f,%.3f,%.3f,"
                    "%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%lu\n",
                    (unsigned long)gTemperatureDriftTest.sequence,
                    (unsigned long)(now - gTemperatureDriftTest.start_tick),
                    state->runtime_config.target_temperature_c,
                    bmi088_temperature,
                    bmi270_temperature,
                    bmi088_gyro[0],
                    bmi088_gyro[1],
                    bmi088_gyro[2],
                    bmi270_gyro[0],
                    bmi270_gyro[1],
                    bmi270_gyro[2],
                    (unsigned long)samples);
    gTemperatureDriftTest.sequence++;
    TaskSerial_Write(line);
}

/* 测试结束后只恢复 RAM 中原目标温度，不触碰 Flash 配置。 */
static void temperature_drift_restore_control(RuntimeState_t *state)
{
    state->runtime_config.target_temperature_c = gTemperatureDriftTest.saved_target_c;
    temperature_drift_reset_heater_state(state);
    state->uart_report_resume_tick = gSystemTickMs + COMMAND_REPORT_PAUSE_MS;
}

static void temperature_drift_finish(RuntimeState_t *state, uint32_t now)
{
    char line[96];

    temperature_drift_write_data(state, now, 1U);
    temperature_drift_restore_control(state);
    (void) snprintf(line, sizeof(line),
                    "TDRIFT:DONE,ELAPSED_MS=%lu,TOTAL_SAMPLES=%lu\n",
                    (unsigned long)(now - gTemperatureDriftTest.start_tick),
                    (unsigned long)gTemperatureDriftTest.total_samples);
    TaskSerial_Write(line);
}

static void temperature_drift_abort(RuntimeState_t *state,
                                    uint32_t now,
                                    const char *reason)
{
    char line[112];

    temperature_drift_write_data(state, now, 1U);
    temperature_drift_restore_control(state);
    (void) snprintf(line, sizeof(line),
                    "TDRIFT:ERROR,REASON=%s,ELAPSED_MS=%lu\n",
                    reason,
                    (unsigned long)(now - gTemperatureDriftTest.start_tick));
    TaskSerial_Write(line);
}

/* 100Hz 前台状态机：生成线性温度设定值、定期输出窗口均值，并在 70°C 稳定后结束。 */
void TemperatureDrift_Service100Hz(RuntimeState_t *state)
{
    uint32_t now;
    uint32_t elapsed_ms;

    if (gTemperatureDriftTestActive == 0U) {
        return;
    }

    now = gSystemTickMs;
    if (state->active_imu_mask != IMU_MASK_DUAL) {
        temperature_drift_abort(state, now, "DUAL_IMU_LOST");
        return;
    }
    if (TaskTemperature_HaveFault(state) != 0U) {
        temperature_drift_abort(state, now, "TEMPERATURE_INVALID");
        return;
    }

    elapsed_ms = now - gTemperatureDriftTest.start_tick;
    if (gTemperatureDriftTest.phase == TEMPERATURE_DRIFT_PHASE_RAMP) {
        if (elapsed_ms >= gTemperatureDriftTest.ramp_duration_ms) {
            state->runtime_config.target_temperature_c = TEMPERATURE_DRIFT_END_C;
            gTemperatureDriftTest.phase = TEMPERATURE_DRIFT_PHASE_HOLD;
            gTemperatureDriftTest.phase_start_tick = now;
            gTemperatureDriftTest.end_stable_start_tick = 0U;
            TaskSerial_Write("TDRIFT:HOLD,TARGET_C=70.000\n");
        } else {
            state->runtime_config.target_temperature_c =
                gTemperatureDriftTest.start_temperature_c +
                (gTemperatureDriftTest.rate_c_per_min * (float)elapsed_ms / 60000.0f);
        }
    }

    if (((int32_t)(now - state->uart_report_resume_tick) >= 0) &&
        ((uint32_t)(now - gTemperatureDriftTest.last_report_tick) >=
         gTemperatureDriftTest.report_period_ms)) {
        gTemperatureDriftTest.last_report_tick = now;
        temperature_drift_write_data(state, now, 0U);
    }

    if (gTemperatureDriftTest.phase != TEMPERATURE_DRIFT_PHASE_HOLD) {
        return;
    }

    if (TaskTemperature_IsReady(state, TEMPERATURE_DRIFT_END_TOLERANCE_C) != 0U) {
        if (gTemperatureDriftTest.end_stable_start_tick == 0U) {
            gTemperatureDriftTest.end_stable_start_tick = now;
        } else if ((uint32_t)(now - gTemperatureDriftTest.end_stable_start_tick) >=
                   TEMPERATURE_DRIFT_END_STABLE_MS) {
            temperature_drift_finish(state, now);
            return;
        }
    } else {
        gTemperatureDriftTest.end_stable_start_tick = 0U;
    }

    if ((uint32_t)(now - gTemperatureDriftTest.phase_start_tick) >=
        TEMPERATURE_DRIFT_END_TIMEOUT_MS) {
        temperature_drift_abort(state, now, "END_TEMPERATURE_TIMEOUT");
    }
}

/*
 * 启动连续温漂测试：保存原目标温度，以当前较高的 IMU 温度作为斜坡起点，并清空采集窗口。
 * 只允许双 IMU、有效温度且起始温度低于 70°C 的状态进入测试，所有参数仅保存在 RAM。
 */
TemperatureDriftStartResult_t TaskTemperature_StartDriftTest(RuntimeState_t *state,
                                                              float rate_c_per_min,
                                                              uint32_t report_hz)
{
    float start_temperature_c;
    float ramp_duration_ms;
    char line[160];

    if (gTemperatureDriftTestActive != 0U) {
        return TEMPERATURE_DRIFT_START_ALREADY_ACTIVE;
    }
    if ((state == NULL) || (rate_c_per_min != rate_c_per_min) ||
        (rate_c_per_min < TEMPERATURE_DRIFT_MIN_RATE_C_PER_MIN) ||
        (rate_c_per_min > TEMPERATURE_DRIFT_MAX_RATE_C_PER_MIN) ||
        (report_hz == 0U) || (report_hz > TEMPERATURE_DRIFT_MAX_REPORT_HZ) ||
        ((1000U % report_hz) != 0U)) {
        return TEMPERATURE_DRIFT_START_INVALID_PARAM;
    }
    if (state->active_imu_mask != IMU_MASK_DUAL) {
        return TEMPERATURE_DRIFT_START_REQUIRES_DUAL_IMU;
    }
    if ((state->bmi088_temperature_valid == 0U) ||
        (state->bmi270_temperature_valid == 0U) ||
        (state->bmi088_temperature_filter_valid == 0U) ||
        (state->bmi270_temperature_filter_valid == 0U)) {
        return TEMPERATURE_DRIFT_START_TEMPERATURE_INVALID;
    }

    start_temperature_c = state->bmi088_temperature_filtered;
    if (state->bmi270_temperature_filtered > start_temperature_c) {
        start_temperature_c = state->bmi270_temperature_filtered;
    }
    if ((start_temperature_c != start_temperature_c) ||
        (start_temperature_c >= TEMPERATURE_DRIFT_END_C)) {
        return TEMPERATURE_DRIFT_START_TEMPERATURE_TOO_HIGH;
    }

    gTemperatureDriftTest.phase = TEMPERATURE_DRIFT_PHASE_RAMP;
    gTemperatureDriftTest.start_tick = gSystemTickMs;
    gTemperatureDriftTest.last_report_tick = gSystemTickMs;
    gTemperatureDriftTest.phase_start_tick = gSystemTickMs;
    gTemperatureDriftTest.end_stable_start_tick = 0U;
    gTemperatureDriftTest.report_period_ms = 1000U / report_hz;
    gTemperatureDriftTest.sequence = 0U;
    gTemperatureDriftTest.total_samples = 0U;
    gTemperatureDriftTest.saved_target_c = state->runtime_config.target_temperature_c;
    gTemperatureDriftTest.start_temperature_c = start_temperature_c;
    gTemperatureDriftTest.rate_c_per_min = rate_c_per_min;
    ramp_duration_ms =
        ((TEMPERATURE_DRIFT_END_C - start_temperature_c) * 60000.0f) /
        rate_c_per_min;
    gTemperatureDriftTest.ramp_duration_ms = (uint32_t)(ramp_duration_ms + 0.5f);

    state->runtime_config.target_temperature_c = start_temperature_c;
    temperature_drift_reset_heater_state(state);

    Board_EnterCritical();
    temperature_drift_clear_accumulator();
    gTemperatureDriftTestActive = 1U;
    Board_ExitCritical();

    TaskSerial_Write("TDRIFT:START\n");
    (void) snprintf(line, sizeof(line),
                    "TDRIFT:CONFIG,START_C=%.3f,END_C=70.000,"
                    "RATE_C_PER_MIN=%.3f,REPORT_HZ=%lu\n",
                    start_temperature_c,
                    rate_c_per_min,
                    (unsigned long)report_hz);
    TaskSerial_Write(line);
    TaskSerial_Write(
        "TDRIFT:COLUMNS,SEQ,ELAPSED_MS,TARGET_C,BMI088_TEMP_C,BMI270_TEMP_C,"
        "BMI088_GX_RADPS,BMI088_GY_RADPS,BMI088_GZ_RADPS,"
        "BMI270_GX_RADPS,BMI270_GY_RADPS,BMI270_GZ_RADPS,SAMPLES\n");
    return TEMPERATURE_DRIFT_START_OK;
}

/* 用户主动停止时先输出最后一个不完整窗口，再恢复测试前的目标温度。 */
uint8_t TaskTemperature_StopDriftTest(RuntimeState_t *state)
{
    uint32_t now;
    char line[96];

    if ((state == NULL) || (gTemperatureDriftTestActive == 0U)) {
        return 0U;
    }

    now = gSystemTickMs;
    temperature_drift_write_data(state, now, 1U);
    temperature_drift_restore_control(state);
    (void) snprintf(line, sizeof(line),
                    "TDRIFT:STOPPED,ELAPSED_MS=%lu,TOTAL_SAMPLES=%lu\n",
                    (unsigned long)(now - gTemperatureDriftTest.start_tick),
                    (unsigned long)gTemperatureDriftTest.total_samples);
    TaskSerial_Write(line);
    return 1U;
}

/* 输出当前测试阶段与温度，不改变斜坡进度，也不清空正在累计的数据窗口。 */
void TaskTemperature_WriteDriftTestStatus(const RuntimeState_t *state)
{
    char line[224];
    const char *phase;

    if ((state == NULL) || (gTemperatureDriftTestActive == 0U)) {
        TaskSerial_Write("TDRIFT:STATUS,ACTIVE=0\n");
        return;
    }

    phase = (gTemperatureDriftTest.phase == TEMPERATURE_DRIFT_PHASE_HOLD) ?
                "HOLD" : "RAMP";
    (void) snprintf(line, sizeof(line),
                    "TDRIFT:STATUS,ACTIVE=1,PHASE=%s,ELAPSED_MS=%lu,"
                    "TARGET_C=%.3f,BMI088_TEMP_C=%.3f,BMI270_TEMP_C=%.3f,"
                    "RATE_C_PER_MIN=%.3f,REPORT_HZ=%lu,SEQ=%lu\n",
                    phase,
                    (unsigned long)(gSystemTickMs - gTemperatureDriftTest.start_tick),
                    state->runtime_config.target_temperature_c,
                    state->bmi088_temperature_filtered,
                    state->bmi270_temperature_filtered,
                    gTemperatureDriftTest.rate_c_per_min,
                    (unsigned long)(1000U / gTemperatureDriftTest.report_period_ms),
                    (unsigned long)gTemperatureDriftTest.sequence);
    TaskSerial_Write(line);
}

/* 供串口和 IMU 任务判断是否应拒绝配置命令、暂停普通姿态回传。 */
uint8_t TaskTemperature_IsDriftTestActive(void)
{
    return gTemperatureDriftTestActive;
}

/*
 * 由 1kHz IMU 中断调用。Gyro 已扣除保存零漂，因此加回 GyroOffset 后得到传感器原始物理角速度。
 * 每个串口数据点是本窗口全部 1kHz 样本的均值，上位机无需处理高频随机噪声。
 */
void TaskTemperature_AccumulateDriftFromInterrupt(const RuntimeState_t *state)
{
    if ((state == NULL) || (gTemperatureDriftTestActive == 0U)) {
        return;
    }

    for (uint8_t axis = 0U; axis < 3U; axis++) {
        gTemperatureDriftAccumulator.bmi088_gyro_sum[axis] +=
            BMI088Sensor.Gyro[axis] + BMI088Sensor.GyroOffset[axis];
        gTemperatureDriftAccumulator.bmi270_gyro_sum[axis] +=
            BMI270Sensor.Gyro[axis] + BMI270Sensor.GyroOffset[axis];
    }
    gTemperatureDriftAccumulator.bmi088_temperature_sum +=
        state->bmi088_temperature_filtered;
    gTemperatureDriftAccumulator.bmi270_temperature_sum +=
        state->bmi270_temperature_filtered;
    if (gTemperatureDriftAccumulator.samples != 0xFFFFFFFFUL) {
        gTemperatureDriftAccumulator.samples++;
    }
}

void TemperatureDrift_Init(void)
{
    gTemperatureDriftTestActive = 0U;
    temperature_drift_clear_accumulator();
}
