#include "task_temperature.h"

#include "bmi088_mspm0.h"
#include "bmi270_mspm0.h"
#include "board_mspm0.h"
#include "temperature_drift.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>

#define HEATER_PWM_PERIOD 1001U
#define HEATER_PWM_LOAD_VALUE (HEATER_PWM_PERIOD - 1U)
#define HEATER_PID_DT 0.10f
#define BMI088_HEATER_PID_KP 32.0f
#define BMI088_HEATER_PID_KI 0.50f
#define BMI088_HEATER_PID_KD 14.0f
#define BMI088_HEATER_FAST_BAND_C 3.0f
#define BMI088_TEMPERATURE_FILTER_ALPHA 0.12f

#define BMI270_HEATER_PID_KP 56.0f
#define BMI270_HEATER_PID_KI 0.50f
#define BMI270_HEATER_PID_KD 10.0f
#define BMI270_HEATER_FAST_BAND_C 3.0f
#define BMI270_TEMPERATURE_FILTER_ALPHA 0.10f

#define HEATER_CONTROL_DIVIDER 10U
#define IMU_TEMPERATURE_MIN_C (-20.0f)
#define IMU_TEMPERATURE_READ_MAX_C 100.0f
#define IMU_TEMPERATURE_SAFE_MAX_C 85.0f

/*
 * 温度任务流程：
 * 1. TaskTemperature_Init() 先关闭两路加热输出，再启动 PWM 和 TIMERG8 100Hz 定时器。
 * 2. TIMERG8 中断只投递温控更新标志，不在中断里读 SPI 或计算 PID。
 * 3. main 消费 100Hz 标志后调用 TaskTemperature_Update100Hz()。
 * 4. 温度采样由 1kHz IMU 中断每 10 次读取一次，保证 SPI 访问统一调度。
 * 5. BMI088 和 BMI270/BMI220 使用完全独立的 PID 状态与 PWM 通道。
 * 6. 采样温度先做一阶低通，PID 只以 10Hz 更新，避免 BMI088 0.125 摄氏度量化台阶扰动输出。
 * 7. BMI088 与 BMI270 使用独立 PID 参数；两路都在低于目标 3 摄氏度以上时全开升温。
 * 8. 任一路关断时直接切回 GPIO 低电平，避免 PWM 比较值或极性错误导致误加热。
 * 9. TDRIFT 测试临时生成线性升温目标，并输出两颗 IMU 的同步温度与原始角速度均值；结束后恢复原目标。
 */

/* 将 0.0-1.0 的占空比转换为 down-count edge PWM 比较值。 */
static uint32_t heater_duty_to_compare(float duty)
{
    uint32_t compare;

    if (duty < 0.0f) {
        duty = 0.0f;
    } else if (duty > 1.0f) {
        duty = 1.0f;
    }

    compare = HEATER_PWM_LOAD_VALUE - (uint32_t)((duty * (float)HEATER_PWM_LOAD_VALUE) + 0.5f);
    if (compare > HEATER_PWM_LOAD_VALUE) {
        compare = HEATER_PWM_LOAD_VALUE;
    }

    return compare;
}

/* 约束占空比到硬件允许范围，同时保证运行状态里记录的 duty 也是最终值。 */
static float heater_clamp_duty(float duty)
{
    if (duty < 0.0f) {
        return 0.0f;
    }
    if (duty > 1.0f) {
        return 1.0f;
    }

    return duty;
}

/* 将 BMI088 加热引脚切成 GPIO 低电平，作为硬件关断态。 */
static void bmi088_heater_force_off(void)
{
    DL_GPIO_clearPins(GPIO_PWM_Motors_C1_PORT, GPIO_PWM_Motors_C1_PIN);
    DL_GPIO_initDigitalOutput(GPIO_PWM_Motors_C1_IOMUX);
    DL_GPIO_enableOutput(GPIO_PWM_Motors_C1_PORT, GPIO_PWM_Motors_C1_PIN);
}

/* 将 BMI270/BMI220 加热引脚切成 GPIO 低电平，作为硬件关断态。 */
static void bmi270_heater_force_off(void)
{
    DL_GPIO_clearPins(GPIO_PWM_Motors_C3_PORT, GPIO_PWM_Motors_C3_PIN);
    DL_GPIO_initDigitalOutput(GPIO_PWM_Motors_C3_IOMUX);
    DL_GPIO_enableOutput(GPIO_PWM_Motors_C3_PORT, GPIO_PWM_Motors_C3_PIN);
}

/* 设置 BMI088 加热 PWM，占用 TIMA0_CCP1。 */
static void bmi088_heater_set_duty(RuntimeState_t *state, float duty)
{
    duty = heater_clamp_duty(duty);
    state->bmi088_heater_duty = duty;
    if (duty <= 0.0f) {
        DL_TimerA_setCaptureCompareValue(PWM_Motors_INST,
                                         HEATER_PWM_LOAD_VALUE,
                                         DL_TIMER_CC_1_INDEX);
        bmi088_heater_force_off();
        return;
    }

    DL_TimerA_setCaptureCompareValue(PWM_Motors_INST,
                                     heater_duty_to_compare(duty),
                                     DL_TIMER_CC_1_INDEX);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_Motors_C1_IOMUX,
                                         GPIO_PWM_Motors_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_Motors_C1_PORT, GPIO_PWM_Motors_C1_PIN);
}

/* 设置 BMI270/BMI220 加热 PWM，占用 TIMA0_CCP3。 */
static void bmi270_heater_set_duty(RuntimeState_t *state, float duty)
{
    duty = heater_clamp_duty(duty);
    state->bmi270_heater_duty = duty;
    if (duty <= 0.0f) {
        DL_TimerA_setCaptureCompareValue(PWM_Motors_INST,
                                         HEATER_PWM_LOAD_VALUE,
                                         DL_TIMER_CC_3_INDEX);
        bmi270_heater_force_off();
        return;
    }

    DL_TimerA_setCaptureCompareValue(PWM_Motors_INST,
                                     heater_duty_to_compare(duty),
                                     DL_TIMER_CC_3_INDEX);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_Motors_C3_IOMUX,
                                         GPIO_PWM_Motors_C3_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_Motors_C3_PORT, GPIO_PWM_Motors_C3_PIN);
}

/* 清空单路 PID 状态，通常在温度无效、异常保护或快开快关切换时调用。 */
static void heater_pid_reset(float *integral, float *last_error)
{
    *integral = 0.0f;
    *last_error = 0.0f;
}

/* 更新单路温度低通滤波；首次有效采样直接装载，避免上电从 0 慢慢爬升。 */
static void temperature_filter_update(float sample, float alpha, float *filtered, uint8_t *filter_valid)
{
    if (*filter_valid == 0U) {
        *filtered = sample;
        *filter_valid = 1U;
    } else {
        *filtered += alpha * (sample - *filtered);
    }
}

/* 单路温控闭环：远低于目标时快开，接近目标后按各自参数 PID 细调。 */
static float heater_pid_update(float target_temp,
                               float current_temp,
                               float kp,
                               float ki,
                               float kd,
                               float fast_band_c,
                               float *integral,
                               float *last_error)
{
    float error = target_temp - current_temp;
    float derivative;
    float output;

    if ((current_temp <= IMU_TEMPERATURE_MIN_C) || (current_temp >= IMU_TEMPERATURE_SAFE_MAX_C)) {
        heater_pid_reset(integral, last_error);
        return 0.0f;
    }

    if (error > fast_band_c) {
        heater_pid_reset(integral, last_error);
        return 1.0f;
    }

    if (error < -fast_band_c) {
        if (*last_error > 0.0f) {
            *integral = 0.0f;
        }
        *last_error = error;
        output = kp * error;
        return output / 100.0f;
    }

    if ((error <= 0.0f) && (*last_error > 0.0f)) {
        *integral = 0.0f;
    }

    *integral += error * HEATER_PID_DT;

    if (*integral > 20.0f) {
        *integral = 20.0f;
    } else if (*integral < -20.0f) {
        *integral = -20.0f;
    }

    derivative = (error - *last_error) / HEATER_PID_DT;
    *last_error = error;

    output = (kp * error) +
             (ki * (*integral)) +
             (kd * derivative);

    return output / 100.0f;
}


/* 初始化加热 PWM 与 100Hz 温控定时器。 */
void TaskTemperature_Init(RuntimeState_t *state)
{
    TemperatureDrift_Init();
    heater_pid_reset(&state->bmi088_heater_pid_integral, &state->bmi088_heater_pid_last_error);
    heater_pid_reset(&state->bmi270_heater_pid_integral, &state->bmi270_heater_pid_last_error);
    state->bmi088_temperature_filter_valid = 0U;
    state->bmi270_temperature_filter_valid = 0U;
    state->bmi088_temperature_filtered = 0.0f;
    state->bmi270_temperature_filtered = 0.0f;
    bmi088_heater_force_off();
    bmi270_heater_force_off();
    bmi088_heater_set_duty(state, 0.0f);
    bmi270_heater_set_duty(state, 0.0f);
    DL_TimerA_startCounter(PWM_Motors_INST);
    NVIC_EnableIRQ(TIMERG8_100hz_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMERG8_100hz_INST);
}

/*
 * 1kHz IMU 中断里的温度采样入口。
 * 该函数每 10 次调用读取一次两颗 IMU 温度，保证所有 SPI 访问都集中在最高优先级 IMU 中断里。
 */
void TaskTemperature_SampleSensorsFromInterrupt(RuntimeState_t *state)
{
    static uint8_t divider;
    float bmi088_temp;
    float bmi270_temp;
    uint8_t bmi088_valid;
    uint8_t bmi270_valid;

    divider++;
    if (divider < 10U) {
        return;
    }
    divider = 0U;

    if ((state->active_imu_mask & IMU_MASK_BMI270) != 0U) {
        bmi270_temp = BMI270_ReadTemperature();
        bmi270_valid = ((bmi270_temp > IMU_TEMPERATURE_MIN_C) &&
                        (bmi270_temp < IMU_TEMPERATURE_READ_MAX_C)) ? 1U : 0U;
    } else {
        bmi270_valid = 0U;
    }

    if ((state->active_imu_mask & IMU_MASK_BMI088) != 0U) {
        BMI088_ReadTemperatureRaw(&state->bmi088_temp_msb, &state->bmi088_temp_lsb, &state->bmi088_temp_raw);
        bmi088_temp = ((float)state->bmi088_temp_raw * BMI088_TEMP_FACTOR) + BMI088_TEMP_OFFSET;
        BMI088Sensor.Temperature = bmi088_temp;
        state->accel_chip_id = BMI088_ReadAccelChipId();
        bmi088_valid = ((bmi088_temp > IMU_TEMPERATURE_MIN_C) &&
                        (bmi088_temp < IMU_TEMPERATURE_READ_MAX_C) &&
                        (state->accel_chip_id == 0x1EU)) ? 1U : 0U;
    } else {
        bmi088_valid = 0U;
    }

    state->bmi088_temperature_valid = bmi088_valid;
    state->bmi270_temperature_valid = bmi270_valid;

    if (bmi088_valid != 0U) {
        temperature_filter_update(bmi088_temp,
                                  BMI088_TEMPERATURE_FILTER_ALPHA,
                                  &state->bmi088_temperature_filtered,
                                  &state->bmi088_temperature_filter_valid);
    } else {
        state->bmi088_temperature_filter_valid = 0U;
    }

    if (bmi270_valid != 0U) {
        temperature_filter_update(bmi270_temp,
                                  BMI270_TEMPERATURE_FILTER_ALPHA,
                                  &state->bmi270_temperature_filtered,
                                  &state->bmi270_temperature_filter_valid);
    } else {
        state->bmi270_temperature_filter_valid = 0U;
    }
}

/*
 * 温控 100Hz 更新入口。
 * 该函数只使用中断里更新好的温度缓存，完成有效性判断、安全关断、PID 计算和 PWM 更新。
 */
void TaskTemperature_Update100Hz(RuntimeState_t *state)
{
    float bmi088_temp = BMI088Sensor.Temperature;
    float bmi270_temp = BMI270Sensor.Temperature;
    uint8_t bmi088_valid = state->bmi088_temperature_valid;
    uint8_t bmi270_valid = state->bmi270_temperature_valid;
    static uint8_t control_divider;

    TemperatureDrift_Service100Hz(state);

    if (state->active_imu_mask == 0U) {
        heater_pid_reset(&state->bmi088_heater_pid_integral, &state->bmi088_heater_pid_last_error);
        heater_pid_reset(&state->bmi270_heater_pid_integral, &state->bmi270_heater_pid_last_error);
        bmi088_heater_set_duty(state, 0.0f);
        bmi270_heater_set_duty(state, 0.0f);
        return;
    }

    if (((state->active_imu_mask & IMU_MASK_BMI088) == 0U) ||
        (bmi088_valid == 0U) || (bmi088_temp >= IMU_TEMPERATURE_SAFE_MAX_C)) {
        heater_pid_reset(&state->bmi088_heater_pid_integral, &state->bmi088_heater_pid_last_error);
        bmi088_heater_set_duty(state, 0.0f);
    }

    if (((state->active_imu_mask & IMU_MASK_BMI270) == 0U) ||
        (bmi270_valid == 0U) || (bmi270_temp >= IMU_TEMPERATURE_SAFE_MAX_C)) {
        heater_pid_reset(&state->bmi270_heater_pid_integral, &state->bmi270_heater_pid_last_error);
        bmi270_heater_set_duty(state, 0.0f);
    }

    control_divider++;
    if (control_divider < HEATER_CONTROL_DIVIDER) {
        return;
    }
    control_divider = 0U;

    if (((state->active_imu_mask & IMU_MASK_BMI088) != 0U) &&
        (bmi088_valid != 0U) && (state->bmi088_temperature_filter_valid != 0U)) {
        bmi088_heater_set_duty(state,
            heater_pid_update(state->runtime_config.target_temperature_c,
                              state->bmi088_temperature_filtered,
                              BMI088_HEATER_PID_KP,
                              BMI088_HEATER_PID_KI,
                              BMI088_HEATER_PID_KD,
                              BMI088_HEATER_FAST_BAND_C,
                              &state->bmi088_heater_pid_integral,
                              &state->bmi088_heater_pid_last_error));
    } else {
        heater_pid_reset(&state->bmi088_heater_pid_integral, &state->bmi088_heater_pid_last_error);
        bmi088_heater_set_duty(state, 0.0f);
    }

    if (((state->active_imu_mask & IMU_MASK_BMI270) != 0U) &&
        (bmi270_valid != 0U) && (state->bmi270_temperature_filter_valid != 0U)) {
        bmi270_heater_set_duty(state,
            heater_pid_update(state->runtime_config.target_temperature_c,
                              state->bmi270_temperature_filtered,
                              BMI270_HEATER_PID_KP,
                              BMI270_HEATER_PID_KI,
                              BMI270_HEATER_PID_KD,
                              BMI270_HEATER_FAST_BAND_C,
                              &state->bmi270_heater_pid_integral,
                              &state->bmi270_heater_pid_last_error));
    } else {
        heater_pid_reset(&state->bmi270_heater_pid_integral, &state->bmi270_heater_pid_last_error);
        bmi270_heater_set_duty(state, 0.0f);
    }
}

/* 判断温度链路是否异常：读不到温度、温度过高或过低都会触发报警。 */
void TaskTemperature_CalibrationService(void *context)
{
    RuntimeState_t *state = (RuntimeState_t *)context;
    static uint8_t update_divider;

    if (state == NULL) {
        return;
    }

    TaskTemperature_SampleSensorsFromInterrupt(state);
    update_divider++;
    if (update_divider >= 10U) {
        update_divider = 0U;
        TaskTemperature_Update100Hz(state);
    }
}

uint8_t TaskTemperature_HaveFault(const RuntimeState_t *state)
{
    if (state == NULL) {
        return 1U;
    }

    if (state->active_imu_mask == 0U) {
        return 1U;
    }

    if (((state->active_imu_mask & IMU_MASK_BMI088) != 0U) &&
        ((state->bmi088_temperature_valid == 0U) ||
         (BMI088Sensor.Temperature <= IMU_TEMPERATURE_MIN_C) ||
         (BMI088Sensor.Temperature >= IMU_TEMPERATURE_SAFE_MAX_C))) {
        return 1U;
    }

    if (((state->active_imu_mask & IMU_MASK_BMI270) != 0U) &&
        ((state->bmi270_temperature_valid == 0U) ||
         (BMI270Sensor.Temperature <= IMU_TEMPERATURE_MIN_C) ||
         (BMI270Sensor.Temperature >= IMU_TEMPERATURE_SAFE_MAX_C))) {
        return 1U;
    }

    return 0U;
}

/* 判断两颗 IMU 是否都进入目标温度附近。用于 LED 慢闪/常亮状态切换。 */
uint8_t TaskTemperature_IsReady(const RuntimeState_t *state, float tolerance_c)
{
    float bmi088_temp;
    float bmi270_temp;
    float bmi088_error;
    float bmi270_error;

    if (state->active_imu_mask == 0U) {
        return 0U;
    }

    if ((state->active_imu_mask & IMU_MASK_BMI088) != 0U) {
        if ((state->bmi088_temperature_valid == 0U) ||
            (state->bmi088_temperature_filter_valid == 0U)) {
            return 0U;
        }
        bmi088_temp = state->bmi088_temperature_filtered;
        bmi088_error = bmi088_temp - state->runtime_config.target_temperature_c;
        if (bmi088_error < 0.0f) {
            bmi088_error = -bmi088_error;
        }
        if (bmi088_error >= tolerance_c) {
            return 0U;
        }
    }

    if ((state->active_imu_mask & IMU_MASK_BMI270) != 0U) {
        if ((state->bmi270_temperature_valid == 0U) ||
            (state->bmi270_temperature_filter_valid == 0U)) {
            return 0U;
        }
        bmi270_temp = state->bmi270_temperature_filtered;
        bmi270_error = bmi270_temp - state->runtime_config.target_temperature_c;
        if (bmi270_error < 0.0f) {
            bmi270_error = -bmi270_error;
        }
        if (bmi270_error >= tolerance_c) {
            return 0U;
        }
    }

    return 1U;
}

