#include "task_temperature.h"

#include "bmi088_mspm0.h"
#include "bmi270_mspm0.h"
#include "ti_msp_dl_config.h"

#define HEATER_PWM_PERIOD 1001U
#define HEATER_PID_DT 0.01f
#define HEATER_PID_KP 45.0f
#define HEATER_PID_KI 1.5f
#define HEATER_PID_KD 0.0f
#define HEATER_FAST_BAND_C 5.0f

/*
 * 温度任务流程：
 * 1. TaskTemperature_Init() 先关闭两路加热输出，再启动 PWM 和 TIMERG8 100Hz 定时器。
 * 2. TIMERG8 中断只投递温控更新标志，不在中断里读 SPI 或计算 PID。
 * 3. main 消费 100Hz 标志后调用 TaskTemperature_Update100Hz()。
 * 4. 温度采样由 1kHz IMU 中断每 10 次读取一次，保证 SPI 访问统一调度。
 * 5. BMI088 和 BMI270/BMI220 使用完全独立的 PID 状态与 PWM 通道。
 * 6. 温差大于 5 摄氏度时直接全开或全关，进入目标附近后再交给 PID 细调。
 */

/* 将 0.0-1.0 的占空比转换为当前 PWM 极性的比较值。 */
static uint32_t heater_duty_to_compare(float duty)
{
    if (duty < 0.0f) {
        duty = 0.0f;
    } else if (duty > 1.0f) {
        duty = 1.0f;
    }

    uint32_t compare = HEATER_PWM_PERIOD - (uint32_t)((duty * (float)HEATER_PWM_PERIOD) + 0.5f);
    if (compare > HEATER_PWM_PERIOD) {
        compare = HEATER_PWM_PERIOD;
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

/* 设置 BMI088 加热 PWM，占用 TIMA0_CCP1。 */
static void bmi088_heater_set_duty(RuntimeState_t *state, float duty)
{
    duty = heater_clamp_duty(duty);
    state->bmi088_heater_duty = duty;
    DL_TimerA_setCaptureCompareValue(PWM_Motors_INST,
                                     heater_duty_to_compare(duty),
                                     DL_TIMER_CC_1_INDEX);
}

/* 设置 BMI270/BMI220 加热 PWM，占用 TIMA0_CCP3。 */
static void bmi270_heater_set_duty(RuntimeState_t *state, float duty)
{
    duty = heater_clamp_duty(duty);
    state->bmi270_heater_duty = duty;
    DL_TimerA_setCaptureCompareValue(PWM_Motors_INST,
                                     heater_duty_to_compare(duty),
                                     DL_TIMER_CC_3_INDEX);
}

/* 清空单路 PID 状态，通常在温度无效、异常保护或快开快关切换时调用。 */
static void heater_pid_reset(float *integral, float *last_error)
{
    *integral = 0.0f;
    *last_error = 0.0f;
}

/* 单路温控闭环：5 度外快开快关，5 度内 PID 细调。 */
static float heater_pid_update(float target_temp, float current_temp, float *integral, float *last_error)
{
    float error = target_temp - current_temp;
    float derivative;
    float output;

    if (error > HEATER_FAST_BAND_C) {
        heater_pid_reset(integral, last_error);
        return 1.0f;
    }

    if (error < -HEATER_FAST_BAND_C) {
        heater_pid_reset(integral, last_error);
        return 0.0f;
    }

    *integral += error * HEATER_PID_DT;
    if (*integral > 50.0f) {
        *integral = 50.0f;
    } else if (*integral < -50.0f) {
        *integral = -50.0f;
    }

    derivative = (error - *last_error) / HEATER_PID_DT;
    *last_error = error;

    output = (HEATER_PID_KP * error) +
             (HEATER_PID_KI * (*integral)) +
             (HEATER_PID_KD * derivative);

    return output / 100.0f;
}

/* 初始化加热 PWM 与 100Hz 温控定时器。 */
void TaskTemperature_Init(RuntimeState_t *state)
{
    heater_pid_reset(&state->bmi088_heater_pid_integral, &state->bmi088_heater_pid_last_error);
    heater_pid_reset(&state->bmi270_heater_pid_integral, &state->bmi270_heater_pid_last_error);
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

    bmi270_temp = BMI270_ReadTemperature();
    bmi270_valid = ((bmi270_temp >= -20.0f) && (bmi270_temp <= 100.0f)) ? 1U : 0U;

    BMI088_ReadTemperatureRaw(&state->bmi088_temp_msb, &state->bmi088_temp_lsb, &state->bmi088_temp_raw);
    bmi088_temp = ((float)state->bmi088_temp_raw * BMI088_TEMP_FACTOR) + BMI088_TEMP_OFFSET;
    BMI088Sensor.Temperature = bmi088_temp;
    state->accel_chip_id = BMI088_ReadAccelChipId();
    bmi088_valid = ((bmi088_temp >= -20.0f) && (bmi088_temp <= 100.0f) &&
                    (state->accel_chip_id == 0x1EU)) ? 1U : 0U;

    state->bmi088_temperature_valid = bmi088_valid;
    state->bmi270_temperature_valid = bmi270_valid;
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

    if ((state->bmi088_init_error != 0U) || (state->bmi270_init_error != 0U) ||
        (state->accel_chip_id == 0U) || (state->gyro_chip_id == 0U) || (state->bmi270_chip_id == 0U)) {
        heater_pid_reset(&state->bmi088_heater_pid_integral, &state->bmi088_heater_pid_last_error);
        heater_pid_reset(&state->bmi270_heater_pid_integral, &state->bmi270_heater_pid_last_error);
        bmi088_heater_set_duty(state, 0.0f);
        bmi270_heater_set_duty(state, 0.0f);
        return;
    }

    if (bmi088_valid != 0U) {
        bmi088_heater_set_duty(state,
            heater_pid_update(state->runtime_config.target_temperature_c,
                              bmi088_temp,
                              &state->bmi088_heater_pid_integral,
                              &state->bmi088_heater_pid_last_error));
    } else {
        heater_pid_reset(&state->bmi088_heater_pid_integral, &state->bmi088_heater_pid_last_error);
        bmi088_heater_set_duty(state, 0.0f);
    }

    if (bmi270_valid != 0U) {
        bmi270_heater_set_duty(state,
            heater_pid_update(state->runtime_config.target_temperature_c,
                              bmi270_temp,
                              &state->bmi270_heater_pid_integral,
                              &state->bmi270_heater_pid_last_error));
    } else {
        heater_pid_reset(&state->bmi270_heater_pid_integral, &state->bmi270_heater_pid_last_error);
        bmi270_heater_set_duty(state, 0.0f);
    }
}

