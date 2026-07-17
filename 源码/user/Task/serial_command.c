#include "serial_command.h"

#include "accel_six_calibration.h"
#include "bmi088_mspm0.h"
#include "board_mspm0.h"
#include "flash_storage.h"
#include "gyro_bias_calibration.h"
#include "serial_status.h"
#include "serial_transport_internal.h"
#include "serial_transport.h"
#include "task_imu.h"
#include "task_led.h"
#include "task_temperature.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

/*
 * 串口命令处理流程：
 * 1. 仅从串口传输层取得已经由换行符结束的完整命令，噪声字节不会直接触发业务。
 * 2. 先寻找合法命令起点，再严格解析参数与行尾；解析失败时不暂停姿态回传。
 * 3. 有效命令统一回复 OK 和 RECV；普通查询暂停回传三秒，配置命令保存 Flash 后复位。
 * 4. 校准、温漂、STATUS 等业务只在此处编排，UART/DMA 细节不进入命令层。
 */

static char *skip_spaces(char *text)
{
    while ((*text == ' ') || (*text == '\t') || (*text == ',')) {
        text++;
    }

    return text;
}

/* 解析无符号整数，并显式检查十进制累加溢出。 */
static uint8_t parse_u32_arg(char **text, uint32_t *value)
{
    uint32_t result = 0U;
    uint32_t digit;
    char *p = skip_spaces(*text);

    if ((*p < '0') || (*p > '9')) {
        return 0U;
    }

    while ((*p >= '0') && (*p <= '9')) {
        digit = (uint32_t)(*p - '0');
        if (result > ((0xFFFFFFFFUL - digit) / 10UL)) {
            return 0U;
        }
        result = (result * 10U) + digit;
        p++;
    }

    *text = p;
    *value = result;
    return 1U;
}

/* 解析温度和标定误差等有限精度浮点参数，避免引入 sscanf。 */
static uint8_t parse_float_arg(char **text, float *value)
{
    int32_t sign = 1;
    int32_t integer = 0;
    int32_t fraction = 0;
    int32_t fraction_scale = 1;
    char *p = skip_spaces(*text);

    if (*p == '-') {
        sign = -1;
        p++;
    }

    if ((*p < '0') || (*p > '9')) {
        return 0U;
    }

    while ((*p >= '0') && (*p <= '9')) {
        int32_t digit = (int32_t)(*p - '0');

        if (integer > ((1000 - digit) / 10)) {
            return 0U;
        }
        integer = (integer * 10) + digit;
        p++;
    }

    if (*p == '.') {
        p++;
        while ((*p >= '0') && (*p <= '9') && (fraction_scale < 1000)) {
            fraction = (fraction * 10) + (int32_t)(*p - '0');
            fraction_scale *= 10;
            p++;
        }
    }

    *text = p;
    *value = (float)sign * ((float)integer + ((float)fraction / (float)fraction_scale));
    return 1U;
}

static uint8_t command_is_separator(char ch)
{
    return ((ch == '\0') || (ch == ' ') || (ch == '\t') || (ch == ',')) ? 1U : 0U;
}

static uint8_t command_is_end(char *text)
{
    return (*skip_spaces(text) == '\0') ? 1U : 0U;
}

/* 命令关键字大小写不敏感，并要求关键字后必须是参数分隔符或行尾。 */
static uint8_t command_match(const char *text, const char *keyword, uint8_t length)
{
    for (uint8_t i = 0U; i < length; i++) {
        char ch = text[i];

        if ((ch >= 'a') && (ch <= 'z')) {
            ch = (char)(ch - 'a' + 'A');
        }
        if (ch != keyword[i]) {
            return 0U;
        }
    }

    return command_is_separator(text[length]);
}

/*
 * 上电早期 RX 可能先收到少量噪声，再收到真实命令。
 * 在完整行中寻找首个合法命令头，使噪声前缀不会破坏第一条有效命令。
 */
static char *command_find_start(char *line)
{
    char *p = line;

    while (*p != '\0') {
        if ((command_match(p, "STATUS", 6U) != 0U) ||
            (command_match(p, "REBOOT", 6U) != 0U) ||
            (command_match(p, "TDRIFT", 6U) != 0U) ||
            (command_match(p, "ACCAL", 5U) != 0U) ||
            (command_match(p, "CAL", 3U) != 0U) ||
            (command_match(p, "TEMP", 4U) != 0U) ||
            (command_match(p, "BAUD", 4U) != 0U) ||
            (command_match(p, "RATE", 4U) != 0U) ||
            (command_match(p, "MODE", 4U) != 0U) ||
            (command_match(p, "YAWCAL", 6U) != 0U) ||
            (command_match(p, "LIST", 4U) != 0U) ||
            (command_match(p, "INS", 3U) != 0U) ||
            (command_match(p, "IMU", 3U) != 0U)) {
            return p;
        }
        p++;
    }

    return NULL;
}

static void command_ack_echo(const char *line)
{
    char echo[UART_RX_LINE_SIZE + 16U];

    TaskSerial_Write("OK\n");
    (void) snprintf(echo, sizeof(echo), "RECV:%s\n", line);
    TaskSerial_Write(echo);
}

void SerialCommand_PauseReport(RuntimeState_t *state)
{
    state->uart_report_resume_tick = gSystemTickMs + COMMAND_REPORT_PAUSE_MS;
    TaskLED_Set(0U);
}

/* 保存当前 RAM 配置；成功后复位使所有依赖参数按统一上电流程生效。 */
static void command_save_and_reset(RuntimeState_t *state, const char *reason)
{
    char line[64];

    (void) snprintf(line, sizeof(line), "%s:SAVE_START\n", reason);
    TaskSerial_Write(line);
    TaskIMU_EnableInterruptUpdate(0U);
    RuntimeState_ClearImuRuntimeStats();
    if (RuntimeConfig_Save(&state->runtime_config) != 0U) {
        (void) snprintf(line, sizeof(line), "%s:SAVE_DONE\n", reason);
        TaskSerial_Write(line);
        Board_ResetAfterSave();
    } else {
        TaskSerial_Write("ERROR:SAVE_FAILED\n");
        TaskIMU_EnableInterruptUpdate((TaskIMU_HaveFault(state) == 0U) ? 1U : 0U);
    }
}

static void serial_write_calibration_scores(const RuntimeConfig_t *config,
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

static const char *accel_six_face_name(uint8_t face)
{
    static const char *const names[ACCEL_SIX_FACE_COUNT] = {
        "+X", "-X", "+Y", "-Y", "+Z", "-Z"
    };

    if (face >= ACCEL_SIX_FACE_COUNT) {
        return "DONE";
    }
    return names[face];
}

static void serial_write_accel_six_prompt(void)
{
    char line[32];

    (void) snprintf(line, sizeof(line), "ACCAL:PLACE:%s\n",
                    accel_six_face_name(AccelSixCalibration_GetExpectedFace()));
    TaskSerial_Write(line);
}

/* 六面采集阻塞前台时仍刷新校准灯态；温控由校准服务回调维持。 */
static void accel_six_progress(uint32_t elapsed_ms, uint32_t total_ms)
{
    (void) total_ms;
    if ((elapsed_ms % 100U) == 0U) {
        TaskLED_UpdateCalibrationStatus();
    }
}

static void serial_write_accel_six_error(AccelSixResult_t result)
{
    if (result == ACCEL_SIX_RESULT_NOT_ACTIVE) {
        TaskSerial_Write("ERROR:ACCAL_NOT_ACTIVE\n");
    } else if (result == ACCEL_SIX_RESULT_INVALID_SENSOR) {
        TaskSerial_Write("ERROR:ACCAL_SENSOR_INVALID\n");
    } else if (result == ACCEL_SIX_RESULT_MOVING) {
        TaskSerial_Write("ERROR:ACCAL_MOVING\n");
    } else if (result == ACCEL_SIX_RESULT_WRONG_FACE) {
        TaskSerial_Write("ERROR:ACCAL_WRONG_FACE\n");
    } else {
        TaskSerial_Write("ERROR:ACCAL_SOLVE_FAILED\n");
    }
}

static void serial_write_temperature_drift_start_error(TemperatureDriftStartResult_t result)
{
    if (result == TEMPERATURE_DRIFT_START_ALREADY_ACTIVE) {
        TaskSerial_Write("ERROR:TDRIFT_ALREADY_ACTIVE\n");
    } else if (result == TEMPERATURE_DRIFT_START_INVALID_PARAM) {
        TaskSerial_Write("ERROR:TDRIFT_PARAM\n");
    } else if (result == TEMPERATURE_DRIFT_START_REQUIRES_DUAL_IMU) {
        TaskSerial_Write("ERROR:TDRIFT_REQUIRES_DUAL_IMU\n");
    } else if (result == TEMPERATURE_DRIFT_START_TEMPERATURE_INVALID) {
        TaskSerial_Write("ERROR:TDRIFT_TEMPERATURE_INVALID\n");
    } else if (result == TEMPERATURE_DRIFT_START_TEMPERATURE_TOO_HIGH) {
        TaskSerial_Write("ERROR:TDRIFT_START_TEMPERATURE\n");
    }
}

static void serial_write_command_list(void)
{
    TaskSerial_Write("COMMAND:LIST\n");
    TaskSerial_Write("REBOOT\n");
    TaskSerial_Write("CAL <WAIT_S> <RECORD_S>\n");
    TaskSerial_Write("ACCAL START\n");
    TaskSerial_Write("ACCAL CAPTURE  ORDER:+X,-X,+Y,-Y,+Z,-Z\n");
    TaskSerial_Write("TDRIFT START [0.1-60.0C/MIN] [1|2|4|5|8|10HZ]\n");
    TaskSerial_Write("TDRIFT STATUS|STOP\n");
    TaskSerial_Write("TEMP <20-85C>\n");
    TaskSerial_Write("BAUD <115200|230400|460800|921600>\n");
    TaskSerial_Write("RATE <1|2|4|5|8|10|20|25|40|50|100|125|200|250|500|1000>\n");
#if RUNTIME_FEATURE_INS_ENABLED
    TaskSerial_Write("RATE_LIMIT USE:500 DEBUG:100 BINARY:1000 INS:100\n");
    TaskSerial_Write("MODE <USE|DEBUG|BINARY|INS>\n");
#else
    TaskSerial_Write("RATE_LIMIT USE:500 DEBUG:100 BINARY:1000\n");
    TaskSerial_Write("MODE <USE|DEBUG|BINARY>\n");
#endif
    TaskSerial_Write("IMU <DUAL|BMI088|BMI270|AUTO>\n");
    TaskSerial_Write("YAWCAL <BMI088_ERR> <BMI270_ERR>\n");
#if RUNTIME_FEATURE_INS_ENABLED
    TaskSerial_Write("INS START\n");
    TaskSerial_Write("STATUS [CONFIG|IMU|BIAS|QUALITY|HEAT|DIAG|INS]\n");
#else
    TaskSerial_Write("STATUS [CONFIG|IMU|BIAS|QUALITY|HEAT|DIAG]\n");
#endif
    TaskSerial_Write("LIST\n");
}

/* 处理单条完整命令；返回 1 表示命令已被识别并消费。 */
static uint8_t command_handle(RuntimeState_t *state, char *line)
{
    char *cmd = command_find_start(line);
    char *p = cmd;

    if (cmd == NULL) {
        return 0U;
    }

    if (command_match(p, "REBOOT", 6U) != 0U) {
        if (command_is_end(p + 6) == 0U) {
            return 0U;
        }

        SerialCommand_PauseReport(state);
        command_ack_echo(cmd);
        TaskIMU_EnableInterruptUpdate(0U);
        TaskSerial_Write("REBOOTING\n");
        Board_SystemReset();
        return 1U;
    } else if (command_match(p, "TDRIFT", 6U) != 0U) {
        TemperatureDriftStartResult_t result;
        float rate_c_per_min = TEMPERATURE_DRIFT_DEFAULT_RATE_C_PER_MIN;
        uint32_t report_hz = TEMPERATURE_DRIFT_DEFAULT_REPORT_HZ;

        p = skip_spaces(p + 6);
        if (command_match(p, "START", 5U) != 0U) {
            p = skip_spaces(p + 5);
            if (*p != '\0') {
                if (parse_float_arg(&p, &rate_c_per_min) == 0U) {
                    return 0U;
                }
                if (command_is_end(p) == 0U) {
                    if ((parse_u32_arg(&p, &report_hz) == 0U) ||
                        (command_is_end(p) == 0U)) {
                        return 0U;
                    }
                }
            }

            SerialCommand_PauseReport(state);
            command_ack_echo(cmd);
            result = TaskTemperature_StartDriftTest(state, rate_c_per_min, report_hz);
            if (result != TEMPERATURE_DRIFT_START_OK) {
                serial_write_temperature_drift_start_error(result);
            }
            return 1U;
        }

        if ((command_match(p, "STOP", 4U) != 0U) &&
            (command_is_end(p + 4) != 0U)) {
            SerialCommand_PauseReport(state);
            command_ack_echo(cmd);
            if (TaskTemperature_StopDriftTest(state) == 0U) {
                TaskSerial_Write("ERROR:TDRIFT_NOT_ACTIVE\n");
            }
            return 1U;
        }

        if ((command_match(p, "STATUS", 6U) != 0U) &&
            (command_is_end(p + 6) != 0U)) {
            SerialCommand_PauseReport(state);
            command_ack_echo(cmd);
            TaskTemperature_WriteDriftTestStatus(state);
            return 1U;
        }

        return 0U;
    } else if ((TaskTemperature_IsDriftTestActive() != 0U) &&
               (command_match(p, "STATUS", 6U) == 0U) &&
               (command_match(p, "LIST", 4U) == 0U)) {
        SerialCommand_PauseReport(state);
        command_ack_echo(cmd);
        TaskSerial_Write("ERROR:TDRIFT_BUSY\n");
        return 1U;
    } else if (command_match(p, "ACCAL", 5U) != 0U) {
        AccelSixResult_t result;

        p = skip_spaces(p + 5);
        if ((command_match(p, "START", 5U) != 0U) &&
            (command_is_end(p + 5) != 0U)) {
            if (TaskIMU_HaveFault(state) != 0U) {
                TaskSerial_Write("ERROR:IMU_INIT_FAILED\n");
                return 1U;
            }

            SerialCommand_PauseReport(state);
            command_ack_echo(cmd);
            if (AccelSixCalibration_Start(state->active_imu_mask) == 0U) {
                TaskSerial_Write("ERROR:ACCAL_START_FAILED\n");
                return 1U;
            }
            TaskSerial_Write("ACCAL:START\n");
            serial_write_accel_six_prompt();
            return 1U;
        }

        if ((command_match(p, "CAPTURE", 7U) == 0U) ||
            (command_is_end(p + 7) == 0U)) {
            return 0U;
        }

        SerialCommand_PauseReport(state);
        command_ack_echo(cmd);
        if (AccelSixCalibration_IsActive() == 0U) {
            TaskSerial_Write("ERROR:ACCAL_NOT_ACTIVE\n");
            return 1U;
        }

        TaskSerial_Write("ACCAL:CAPTURE\n");
        TaskIMU_EnableInterruptUpdate(0U);
        result = AccelSixCalibration_Capture(&state->runtime_config.accel_bias,
                                             &state->runtime_config.accel_scale,
                                             TaskTemperature_CalibrationService,
                                             state,
                                             accel_six_progress);
        TaskIMU_EnableInterruptUpdate(1U);
        if (result == ACCEL_SIX_RESULT_CAPTURED) {
            TaskSerial_Write("ACCAL:CAPTURED\n");
            serial_write_accel_six_prompt();
        } else if (result == ACCEL_SIX_RESULT_COMPLETE) {
            state->runtime_config.accel_bias_valid = 1U;
            state->runtime_config.accel_calibrated_mask |= state->active_imu_mask;
            TaskSerial_Write("ACCAL:DONE\n");
            command_save_and_reset(state, "ACCAL");
        } else {
            serial_write_accel_six_error(result);
            if (AccelSixCalibration_IsActive() != 0U) {
                serial_write_accel_six_prompt();
            }
        }
        return 1U;
    } else if (command_match(p, "CAL", 3U) != 0U) {
        uint32_t wait_s;
        uint32_t record_s;
        uint8_t calibration_mask;
        GyroQualityData_t quality;

        p += 3;
        if ((parse_u32_arg(&p, &wait_s) == 0U) ||
            (parse_u32_arg(&p, &record_s) == 0U) ||
            (wait_s > 600U) || (record_s == 0U) || (record_s > 600U) ||
            (command_is_end(p) == 0U)) {
            return 0U;
        }

        SerialCommand_PauseReport(state);
        command_ack_echo(cmd);
        TaskSerial_Write("CAL:START\n");
        TaskIMU_EnableInterruptUpdate(0U);
        calibration_mask =
            (uint8_t)(((state->bmi088_present != 0U) ? IMU_BIAS_SENSOR_BMI088 : 0U) |
                      ((state->bmi270_present != 0U) ? IMU_BIAS_SENSOR_BMI270 : 0U));
        if (GyroBias_CalibrateWithService(&state->runtime_config.gyro_bias,
                                          &quality,
                                          calibration_mask,
                                          wait_s * 1000U,
                                          record_s * 1000U,
                                          TaskTemperature_CalibrationService,
                                          state,
                                          SerialCommand_CalibrationProgress) != 0U) {
            GyroBias_StoreQuality(&state->runtime_config, &quality, calibration_mask);
            state->runtime_config.gyro_bias_valid = 1U;
            state->runtime_config.gyro_calibrated_mask |= calibration_mask;
            state->gyro_bias_valid = 1U;
            state->gyro_bias_calibrated = 1U;
            TaskIMU_AlignInitialAttitude();
            TaskIMU_EnableInterruptUpdate(1U);
            SerialCommand_PauseReport(state);
            TaskSerial_Write("CAL:DONE\n");
            serial_write_calibration_scores(&state->runtime_config, calibration_mask);
            command_save_and_reset(state, "CAL");
        } else {
            TaskIMU_EnableInterruptUpdate(1U);
            TaskSerial_Write("ERROR:CAL_FAILED\n");
        }
        return 1U;
    } else if (command_match(p, "TEMP", 4U) != 0U) {
        float target_temp;

        p += 4;
        if ((parse_float_arg(&p, &target_temp) == 0U) ||
            (target_temp < 20.0f) || (target_temp > 85.0f) ||
            (command_is_end(p) == 0U)) {
            return 0U;
        }

        SerialCommand_PauseReport(state);
        command_ack_echo(cmd);
        state->runtime_config.target_temperature_c = target_temp;
        BMI088Sensor.TempWhenCali = target_temp;
        state->bmi088_heater_pid_integral = 0.0f;
        state->bmi088_heater_pid_last_error = 0.0f;
        state->bmi270_heater_pid_integral = 0.0f;
        state->bmi270_heater_pid_last_error = 0.0f;
        TaskSerial_Write("TEMP:UPDATED\n");
        command_save_and_reset(state, "TEMP");
        return 1U;
    } else if (command_match(p, "BAUD", 4U) != 0U) {
        uint32_t baud_rate;

        p += 4;
        if ((parse_u32_arg(&p, &baud_rate) == 0U) ||
            (RuntimeState_BaudRateIsSupported(baud_rate) == 0U) ||
            (command_is_end(p) == 0U)) {
            return 0U;
        }

        SerialCommand_PauseReport(state);
        command_ack_echo(cmd);
        state->runtime_config.baud_rate = baud_rate;
        TaskSerial_Write("BAUD:UPDATED\n");
        command_save_and_reset(state, "BAUD");
        return 1U;
    } else if (command_match(p, "RATE", 4U) != 0U) {
        uint32_t report_rate;

        p += 4;
        if ((parse_u32_arg(&p, &report_rate) == 0U) ||
            (RuntimeConfig_ReportRateIsSupported(state->runtime_config.output_mode,
                                                 report_rate) == 0U) ||
            (command_is_end(p) == 0U)) {
            return 0U;
        }

        SerialCommand_PauseReport(state);
        command_ack_echo(cmd);
        state->runtime_config.report_rate_hz = report_rate;
        RuntimeState_UpdateReportRate(state);
        TaskSerial_Write("RATE:UPDATED\n");
        command_save_and_reset(state, "RATE");
        return 1U;
    } else if (command_match(p, "MODE", 4U) != 0U) {
        uint32_t rate_limit;
        uint8_t rate_clamped = 0U;

        p = skip_spaces(p + 4);
        if ((strncmp(p, "DEBUG", 5U) == 0) && (command_is_end(p + 5) != 0U)) {
            state->runtime_config.output_mode = RUNTIME_OUTPUT_MODE_DEBUG;
        } else if ((strncmp(p, "BINARY", 6U) == 0) &&
                   (command_is_end(p + 6) != 0U)) {
            state->runtime_config.output_mode = RUNTIME_OUTPUT_MODE_BINARY;
        } else if ((strncmp(p, "INS", 3U) == 0) &&
                   (command_is_end(p + 3) != 0U)) {
#if RUNTIME_FEATURE_INS_ENABLED
            state->runtime_config.output_mode = RUNTIME_OUTPUT_MODE_INS;
#else
            TaskSerial_Write("ERROR:INS_DISABLED\n");
            return 1U;
#endif
        } else if ((strncmp(p, "USE", 3U) == 0) &&
                   (command_is_end(p + 3) != 0U)) {
            state->runtime_config.output_mode = RUNTIME_OUTPUT_MODE_USE;
        } else {
            return 0U;
        }

        rate_limit = RuntimeConfig_GetReportRateLimit(state->runtime_config.output_mode);
        if (state->runtime_config.report_rate_hz > rate_limit) {
            state->runtime_config.report_rate_hz = rate_limit;
            rate_clamped = 1U;
        }
        RuntimeState_UpdateReportRate(state);

        SerialCommand_PauseReport(state);
        command_ack_echo(cmd);
        if (rate_clamped != 0U) {
            char rate_line[32];

            (void) snprintf(rate_line, sizeof(rate_line), "RATE:CLAMPED:%lu\n",
                            (unsigned long) state->runtime_config.report_rate_hz);
            TaskSerial_Write(rate_line);
        }
        TaskSerial_Write("MODE:UPDATED\n");
        command_save_and_reset(state, "MODE");
        return 1U;
    } else if (command_match(p, "IMU", 3U) != 0U) {
        p = skip_spaces(p + 3);
        if ((strncmp(p, "DUAL", 4U) == 0) && (command_is_end(p + 4) != 0U)) {
            state->runtime_config.imu_source = RUNTIME_IMU_SOURCE_DUAL;
        } else if ((strncmp(p, "BMI088", 6U) == 0) &&
                   (command_is_end(p + 6) != 0U)) {
            state->runtime_config.imu_source = RUNTIME_IMU_SOURCE_BMI088;
        } else if ((strncmp(p, "BMI270", 6U) == 0) &&
                   (command_is_end(p + 6) != 0U)) {
            state->runtime_config.imu_source = RUNTIME_IMU_SOURCE_BMI270;
        } else if ((strncmp(p, "AUTO", 4U) == 0) &&
                   (command_is_end(p + 4) != 0U)) {
            state->runtime_config.imu_source = RUNTIME_IMU_SOURCE_AUTO;
        } else {
            return 0U;
        }

        SerialCommand_PauseReport(state);
        command_ack_echo(cmd);
        TaskSerial_Write("IMU:UPDATED\n");
        command_save_and_reset(state, "IMU");
        return 1U;
    } else if (command_match(p, "YAWCAL", 6U) != 0U) {
        float bmi088_error;
        float bmi270_error;

        p += 6;
        if ((parse_float_arg(&p, &bmi088_error) == 0U) ||
            (parse_float_arg(&p, &bmi270_error) == 0U) ||
            (bmi088_error < -30.0f) || (bmi088_error > 30.0f) ||
            (bmi270_error < -30.0f) || (bmi270_error > 30.0f) ||
            (command_is_end(p) == 0U)) {
            return 0U;
        }

        SerialCommand_PauseReport(state);
        command_ack_echo(cmd);
        state->runtime_config.bmi088_yaw_error_deg_per_turn = bmi088_error;
        state->runtime_config.bmi270_yaw_error_deg_per_turn = bmi270_error;
        TaskSerial_Write("YAWCAL:UPDATED\n");
        command_save_and_reset(state, "YAWCAL");
        return 1U;
    } else if (command_match(p, "INS", 3U) != 0U) {
#if RUNTIME_FEATURE_INS_ENABLED
        p = skip_spaces(p + 3);
        if ((command_match(p, "START", 5U) == 0U) ||
            (command_is_end(p + 5) == 0U)) {
            return 0U;
        }

        if (state->runtime_config.output_mode != RUNTIME_OUTPUT_MODE_INS) {
            TaskSerial_Write("ERROR:INS_MODE_REQUIRED\n");
            return 1U;
        }
#if RUNTIME_INS_REQUIRE_ACCEL_CALIBRATION
        if ((state->runtime_config.accel_bias_valid == 0U) ||
            ((state->runtime_config.accel_calibrated_mask & state->active_imu_mask) !=
             state->active_imu_mask)) {
            TaskSerial_Write("ERROR:ACCAL_REQUIRED\n");
            return 1U;
        }
#endif
        if (TaskIMU_RequestNavigationStart(state) == 0U) {
            TaskSerial_Write("ERROR:IMU_INIT_FAILED\n");
            return 1U;
        }

        /* 实时原点标记不暂停回传、不写 Flash；中断在下一个采样点确认生效。 */
        command_ack_echo(cmd);
        TaskSerial_Write("INS:START_QUEUED\n");
        return 1U;
#else
        TaskSerial_Write("ERROR:INS_DISABLED\n");
        return 1U;
#endif
    } else if (command_match(p, "STATUS", 6U) != 0U) {
        SerialCommand_PauseReport(state);
        command_ack_echo(cmd);
        SerialStatus_WriteSnapshot(state, p + 6);
        return 1U;
    } else if (command_match(p, "LIST", 4U) != 0U) {
        SerialCommand_PauseReport(state);
        command_ack_echo(cmd);
        serial_write_command_list();
        return 1U;
    }

    return 0U;
}

uint8_t SerialCommand_Poll(RuntimeState_t *state)
{
    char line[UART_RX_LINE_SIZE];

    /* 溢出的噪声或超长行只被丢弃，不算作有效命令，也不暂停回传。 */
    (void) SerialTransport_TakeOverflow();
    if (SerialTransport_TakeCommand(line) == 0U) {
        return 0U;
    }

    return command_handle(state, line);
}

/* 零漂整定期间每秒回传剩余时间，并维持三秒明灭的校准灯态。 */
void SerialCommand_CalibrationProgress(uint32_t elapsed_ms, uint32_t total_ms)
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
