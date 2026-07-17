#include "serial_status.h"

#include "firmware_info.h"
#include "serial_transport.h"
#include "task_imu.h"
#include "bmi088_mspm0.h"
#include "bmi270_mspm0.h"
#include "board_mspm0.h"
#include "flash_storage.h"

#include <stdio.h>
#include <string.h>

#define STATUS_BMI088_ACCEL_CHIP_ID 0x1EU
#define STATUS_BMI088_GYRO_CHIP_ID  0x0FU

/*
 * STATUS 报表流程：
 * 1. 每次查询先读取一份独立的 Flash 配置快照，避免把未保存的 RAM 参数冒充永久配置。
 * 2. 无子命令时只输出版本、基础配置和 IMU 概要；详细信息按子命令拆分。
 * 3. 本模块只格式化只读快照，不修改运行参数，也不触发 Flash 写入。
 */

static char *status_skip_spaces(char *text)
{
    while ((*text == ' ') || (*text == '\t') || (*text == ',')) {
        text++;
    }
    return text;
}

static uint8_t status_args_end(char *text)
{
    return (*status_skip_spaces(text) == '\0') ? 1U : 0U;
}

/* 输出 STATUS 快照：当前运行参数、Flash 中的零漂值以及两颗 IMU 温度。 */
static const char *status_bmi088_model_name(const RuntimeState_t *state)
{
    if ((state->accel_chip_id == STATUS_BMI088_ACCEL_CHIP_ID) &&
        (state->gyro_chip_id == STATUS_BMI088_GYRO_CHIP_ID)) {
        return "BMI088";
    }

    return "UNKNOWN";
}

static const char *status_bmi270_model_name(const RuntimeState_t *state)
{
    if (state->bmi270_chip_id == BMI270_CHIP_ID_VALUE) {
        return "BMI270";
    }

    if (state->bmi270_chip_id == BMI220_CHIP_ID_VALUE) {
        return "BMI220";
    }

    if (state->bmi270_chip_id == BMI260_CHIP_ID_VALUE) {
        return "BMI260";
    }

    return "UNKNOWN";
}

static const char *status_output_mode_name(uint8_t output_mode)
{
    if (output_mode == RUNTIME_OUTPUT_MODE_DEBUG) {
        return "DEBUG";
    }
    if (output_mode == RUNTIME_OUTPUT_MODE_BINARY) {
        return "BINARY";
    }
#if RUNTIME_FEATURE_INS_ENABLED
    if (output_mode == RUNTIME_OUTPUT_MODE_INS) {
        return "INS";
    }
#endif

    return "USE";
}

static const char *status_imu_source_name(uint8_t imu_source)
{
    if (imu_source == RUNTIME_IMU_SOURCE_AUTO) {
        return "AUTO";
    }
    if (imu_source == RUNTIME_IMU_SOURCE_BMI088) {
        return "BMI088";
    }
    if (imu_source == RUNTIME_IMU_SOURCE_BMI270) {
        return "BMI270";
    }

    return "DUAL";
}

static const char *status_active_imu_name(uint8_t active_mask)
{
    if (active_mask == IMU_MASK_DUAL) {
        return "DUAL";
    }
    if (active_mask == IMU_MASK_BMI088) {
        return "BMI088";
    }
    if (active_mask == IMU_MASK_BMI270) {
        return "BMI270";
    }

    return "NONE";
}

static uint8_t status_load_flash_config(RuntimeConfig_t *flash_config)
{
    return RuntimeConfig_Load(flash_config);
}

static void serial_write_status_version(void)
{
    char line[32];

    (void) snprintf(line, sizeof(line), "FW:%s\n", Firmware_GetVersion());
    TaskSerial_Write(line);
}

static void serial_write_status_config(const RuntimeConfig_t *flash_config, uint8_t flash_config_valid)
{
    char line[256];

    if (flash_config_valid != 0U) {
        (void) snprintf(line, sizeof(line),
                        "BAUD:%lu\n"
                        "RATE:%lu\n"
                        "MODE:%s\n"
                        "IMU_SOURCE:%s\n"
                        "TARGET_TEMP:%.3f\n"
                        "BMI088_YAW_ERR:%.3f\n"
                        "BMI270_YAW_ERR:%.3f\n",
                        (unsigned long) flash_config->baud_rate,
                        (unsigned long) flash_config->report_rate_hz,
                        status_output_mode_name(flash_config->output_mode),
                        status_imu_source_name(flash_config->imu_source),
                        flash_config->target_temperature_c,
                        flash_config->bmi088_yaw_error_deg_per_turn,
                        flash_config->bmi270_yaw_error_deg_per_turn);
    } else {
        (void) snprintf(line, sizeof(line),
                        "BAUD:INVALID\n"
                        "RATE:INVALID\n"
                        "MODE:INVALID\n"
                        "IMU_SOURCE:INVALID\n"
                        "TARGET_TEMP:INVALID\n"
                        "BMI088_YAW_ERR:INVALID\n"
                        "BMI270_YAW_ERR:INVALID\n");
    }
    TaskSerial_Write(line);
}

static void serial_write_status_imu(RuntimeState_t *state)
{
    char line[256];

    (void) snprintf(line, sizeof(line),
                    "IMU_ACTIVE:%s\n"
                    "BMI088_MODEL:%s\n"
                    "BMI088_PRESENT:%u\n"
                    "BMI088_ACCEL_ID:0x%02X\n"
                    "BMI088_GYRO_ID:0x%02X\n"
                    "BMI088_INIT_ERROR:0x%02X\n"
                    "BMI088_ACCEL_INIT_ERROR:0x%02X\n"
                    "BMI088_GYRO_INIT_ERROR:0x%02X\n"
                    "BMI270_MODEL:%s\n",
                    status_active_imu_name(state->active_imu_mask),
                    status_bmi088_model_name(state),
                    state->bmi088_present,
                    state->accel_chip_id,
                    state->gyro_chip_id,
                    state->bmi088_init_error,
                    BMI088_Debug.AccelInitError,
                    BMI088_Debug.GyroInitError,
                    status_bmi270_model_name(state));
    TaskSerial_Write(line);

    (void) snprintf(line, sizeof(line),
                    "BMI270_PRESENT:%u\n"
                    "BMI270_CHIP_ID:0x%02X\n"
                    "BMI270_ID_BEFORE_RESET:0x%02X\n"
                    "BMI270_ID_AFTER_RESET:0x%02X\n"
                    "BMI270_INIT_ERROR:0x%02X\n",
                    state->bmi270_present,
                    state->bmi270_chip_id,
                    BMI270_Debug.ChipIdBeforeReset,
                    BMI270_Debug.ChipIdAfterReset,
                    state->bmi270_init_error);
    TaskSerial_Write(line);

    (void) snprintf(line, sizeof(line), "BMI088_TEMP:%.3f\n", BMI088Sensor.Temperature);
    TaskSerial_Write(line);

    if (state->bmi270_temperature_valid != 0U) {
        (void) snprintf(line, sizeof(line), "BMI270_TEMP:%.3f\n", BMI270Sensor.Temperature);
    } else {
        (void) snprintf(line, sizeof(line), "BMI270_TEMP:INVALID\n");
    }
    TaskSerial_Write(line);
}

static void serial_write_status_bias(const RuntimeConfig_t *flash_config, uint8_t flash_config_valid)
{
    char line[256];

    if ((flash_config_valid != 0U) && (flash_config->gyro_bias_valid != 0U)) {
        (void) snprintf(line, sizeof(line), "GYRO_CAL_IMU:%s\n",
                        status_active_imu_name(flash_config->gyro_calibrated_mask));
    } else {
        (void) snprintf(line, sizeof(line), "GYRO_CAL_IMU:NONE\n");
    }
    TaskSerial_Write(line);

    if ((flash_config_valid != 0U) && (flash_config->accel_bias_valid != 0U)) {
        (void) snprintf(line, sizeof(line), "ACCEL_CAL_IMU:%s\n",
                        status_active_imu_name(flash_config->accel_calibrated_mask));
    } else {
        (void) snprintf(line, sizeof(line), "ACCEL_CAL_IMU:NONE\n");
    }
    TaskSerial_Write(line);

    if ((flash_config_valid != 0U) && (flash_config->gyro_bias_valid != 0U)) {
        (void) snprintf(line, sizeof(line),
                        "BMI088_BIAS_X:%.6f\n"
                        "BMI088_BIAS_Y:%.6f\n"
                        "BMI088_BIAS_Z:%.6f\n"
                        "BMI270_BIAS_X:%.6f\n"
                        "BMI270_BIAS_Y:%.6f\n"
                        "BMI270_BIAS_Z:%.6f\n",
                        flash_config->gyro_bias.bmi088[0],
                        flash_config->gyro_bias.bmi088[1],
                        flash_config->gyro_bias.bmi088[2],
                        flash_config->gyro_bias.bmi270[0],
                        flash_config->gyro_bias.bmi270[1],
                        flash_config->gyro_bias.bmi270[2]);
    } else {
        (void) snprintf(line, sizeof(line),
                        "BMI088_BIAS_X:INVALID\n"
                        "BMI088_BIAS_Y:INVALID\n"
                        "BMI088_BIAS_Z:INVALID\n"
                        "BMI270_BIAS_X:INVALID\n"
                        "BMI270_BIAS_Y:INVALID\n"
                        "BMI270_BIAS_Z:INVALID\n");
    }
    TaskSerial_Write(line);

    if ((flash_config_valid != 0U) && (flash_config->accel_bias_valid != 0U)) {
        (void) snprintf(line, sizeof(line),
                        "BMI088_ACCEL_OFFSET_X:%.6f\n"
                        "BMI088_ACCEL_OFFSET_Y:%.6f\n"
                        "BMI088_ACCEL_OFFSET_Z:%.6f\n"
                        "BMI270_ACCEL_OFFSET_X:%.6f\n"
                        "BMI270_ACCEL_OFFSET_Y:%.6f\n"
                        "BMI270_ACCEL_OFFSET_Z:%.6f\n",
                        flash_config->accel_bias.bmi088[0],
                        flash_config->accel_bias.bmi088[1],
                        flash_config->accel_bias.bmi088[2],
                        flash_config->accel_bias.bmi270[0],
                        flash_config->accel_bias.bmi270[1],
                        flash_config->accel_bias.bmi270[2]);
    } else {
        (void) snprintf(line, sizeof(line),
                        "BMI088_ACCEL_OFFSET_X:INVALID\n"
                        "BMI088_ACCEL_OFFSET_Y:INVALID\n"
                        "BMI088_ACCEL_OFFSET_Z:INVALID\n"
                        "BMI270_ACCEL_OFFSET_X:INVALID\n"
                        "BMI270_ACCEL_OFFSET_Y:INVALID\n"
                        "BMI270_ACCEL_OFFSET_Z:INVALID\n");
    }
    TaskSerial_Write(line);

    if ((flash_config_valid != 0U) && (flash_config->accel_bias_valid != 0U)) {
        (void) snprintf(line, sizeof(line),
                        "BMI088_ACCEL_SCALE_X:%.6f\n"
                        "BMI088_ACCEL_SCALE_Y:%.6f\n"
                        "BMI088_ACCEL_SCALE_Z:%.6f\n"
                        "BMI270_ACCEL_SCALE_X:%.6f\n"
                        "BMI270_ACCEL_SCALE_Y:%.6f\n"
                        "BMI270_ACCEL_SCALE_Z:%.6f\n",
                        flash_config->accel_scale.bmi088[0],
                        flash_config->accel_scale.bmi088[1],
                        flash_config->accel_scale.bmi088[2],
                        flash_config->accel_scale.bmi270[0],
                        flash_config->accel_scale.bmi270[1],
                        flash_config->accel_scale.bmi270[2]);
    } else {
        (void) snprintf(line, sizeof(line),
                        "BMI088_ACCEL_SCALE_X:INVALID\n"
                        "BMI088_ACCEL_SCALE_Y:INVALID\n"
                        "BMI088_ACCEL_SCALE_Z:INVALID\n"
                        "BMI270_ACCEL_SCALE_X:INVALID\n"
                        "BMI270_ACCEL_SCALE_Y:INVALID\n"
                        "BMI270_ACCEL_SCALE_Z:INVALID\n");
    }
    TaskSerial_Write(line);
}

static uint8_t status_gyro_quality_is_valid(const RuntimeConfig_t *flash_config,
                                            uint8_t flash_config_valid,
                                            uint8_t sensor_mask,
                                            uint8_t score)
{
    return ((flash_config_valid != 0U) &&
            (flash_config->gyro_bias_valid != 0U) &&
            ((flash_config->gyro_calibrated_mask & sensor_mask) != 0U) &&
            (score <= RUNTIME_GYRO_SENSOR_SCORE_MAX)) ? 1U : 0U;
}

static void serial_write_status_quality(const RuntimeConfig_t *flash_config,
                                        uint8_t flash_config_valid)
{
    char line[224];

    if (status_gyro_quality_is_valid(flash_config, flash_config_valid,
                                     RUNTIME_GYRO_CAL_BMI088,
                                     flash_config->bmi088_gyro_score) != 0U) {
        (void) snprintf(line, sizeof(line),
                        "BMI088_GYRO_VARIANCE_X:%.9f\n"
                        "BMI088_GYRO_VARIANCE_Y:%.9f\n"
                        "BMI088_GYRO_VARIANCE_Z:%.9f\n"
                        "BMI088_GYRO_SCORE:%u\n",
                        RuntimeConfig_DecodeGyroVariance(flash_config->gyro_variance.bmi088[0]),
                        RuntimeConfig_DecodeGyroVariance(flash_config->gyro_variance.bmi088[1]),
                        RuntimeConfig_DecodeGyroVariance(flash_config->gyro_variance.bmi088[2]),
                        flash_config->bmi088_gyro_score);
    } else {
        (void) snprintf(line, sizeof(line),
                        "BMI088_GYRO_VARIANCE_X:INVALID\n"
                        "BMI088_GYRO_VARIANCE_Y:INVALID\n"
                        "BMI088_GYRO_VARIANCE_Z:INVALID\n"
                        "BMI088_GYRO_SCORE:INVALID\n");
    }
    TaskSerial_Write(line);

    if (status_gyro_quality_is_valid(flash_config, flash_config_valid,
                                     RUNTIME_GYRO_CAL_BMI270,
                                     flash_config->bmi270_gyro_score) != 0U) {
        (void) snprintf(line, sizeof(line),
                        "BMI270_GYRO_VARIANCE_X:%.9f\n"
                        "BMI270_GYRO_VARIANCE_Y:%.9f\n"
                        "BMI270_GYRO_VARIANCE_Z:%.9f\n"
                        "BMI270_GYRO_SCORE:%u\n",
                        RuntimeConfig_DecodeGyroVariance(flash_config->gyro_variance.bmi270[0]),
                        RuntimeConfig_DecodeGyroVariance(flash_config->gyro_variance.bmi270[1]),
                        RuntimeConfig_DecodeGyroVariance(flash_config->gyro_variance.bmi270[2]),
                        flash_config->bmi270_gyro_score);
    } else {
        (void) snprintf(line, sizeof(line),
                        "BMI270_GYRO_VARIANCE_X:INVALID\n"
                        "BMI270_GYRO_VARIANCE_Y:INVALID\n"
                        "BMI270_GYRO_VARIANCE_Z:INVALID\n"
                        "BMI270_GYRO_SCORE:INVALID\n");
    }
    TaskSerial_Write(line);
}

static void serial_write_status_heat(RuntimeState_t *state, const RuntimeConfig_t *flash_config,
                                     uint8_t flash_config_valid)
{
    char line[160];

    if (flash_config_valid != 0U) {
        (void) snprintf(line, sizeof(line), "TARGET_TEMP:%.3f\n", flash_config->target_temperature_c);
    } else {
        (void) snprintf(line, sizeof(line), "TARGET_TEMP:INVALID\n");
    }
    TaskSerial_Write(line);

    (void) snprintf(line, sizeof(line), "BMI088_TEMP:%.3f\n", BMI088Sensor.Temperature);
    TaskSerial_Write(line);
    if (state->bmi270_temperature_valid != 0U) {
        (void) snprintf(line, sizeof(line), "BMI270_TEMP:%.3f\n", BMI270Sensor.Temperature);
    } else {
        (void) snprintf(line, sizeof(line), "BMI270_TEMP:INVALID\n");
    }
    TaskSerial_Write(line);

    (void) snprintf(line, sizeof(line),
                    "BMI088_HEAT:%.3f\n"
                    "BMI270_HEAT:%.3f\n",
                    state->bmi088_heater_duty,
                    state->bmi270_heater_duty);
    TaskSerial_Write(line);
}

static void serial_write_status_diag(void)
{
    char line[192];

    (void) snprintf(line, sizeof(line),
                    "BMI088_GYRO_SAT:%lu\n"
                    "BMI088_ACCEL_SAT:%lu\n"
                    "BMI270_GYRO_SAT:%lu\n"
                    "BMI270_ACCEL_SAT:%lu\n"
                    "IMU_UPDATE:%lu\n"
                    "IMU_OVERRUN:%lu\n",
                    (unsigned long) BMI088Sensor.GyroSaturationCount,
                    (unsigned long) BMI088Sensor.AccelSaturationCount,
                    (unsigned long) BMI270Sensor.GyroSaturationCount,
                    (unsigned long) BMI270Sensor.AccelSaturationCount,
                    (unsigned long) gImuRuntimeStats.update_count,
                    (unsigned long) gImuRuntimeStats.overrun_count);
    TaskSerial_Write(line);
    TaskSerial_Write((Board_HasExternalClockFault() != 0U) ?
                         "CLOCK_SOURCE:SYSOSC_PLL\n" :
                         "CLOCK_SOURCE:HFXT_PLL\n");
}

#if RUNTIME_FEATURE_INS_ENABLED
static const char *status_navigation_state_name(uint8_t state)
{
    if (state == INERTIAL_NAVIGATION_RUNNING) {
        return "RUNNING";
    }
    if (state == INERTIAL_NAVIGATION_FAULT) {
        return "FAULT";
    }
    if (state == INERTIAL_NAVIGATION_ALIGNING) {
        return "ALIGNING";
    }

    return "WAIT_START";
}

static uint32_t status_navigation_elapsed_ms(uint32_t updates)
{
    uint32_t seconds = updates / IMU_UPDATE_RATE_HZ;
    uint32_t remainder = updates % IMU_UPDATE_RATE_HZ;

    return (seconds * 1000U) + ((remainder * 1000U) / IMU_UPDATE_RATE_HZ);
}

/* 输出二维惯导的运行状态；位置单位为 m，速度单位为 m/s。 */
static void serial_write_status_ins(void)
{
    InertialNavigationSnapshot_t navigation;
    char line[320];

    TaskIMU_GetNavigationSnapshot(&navigation);
    (void) snprintf(line, sizeof(line),
                    "INS_STATE:%s\n"
                    "INS_ALIGN_SAMPLES:%u\n"
                    "INS_TIME_MS:%lu\n"
                    "INS_STATIONARY:%u\n"
                    "INS_BIAS_VALID:%u\n"
                    "INS_FAULT:%u\n"
                    "INS_X:%.3f\n"
                    "INS_Y:%.3f\n"
                    "INS_VX:%.3f\n"
                    "INS_VY:%.3f\n"
                    "INS_BIAS_X:%.6f\n"
                    "INS_BIAS_Y:%.6f\n",
                    status_navigation_state_name(navigation.state),
                    navigation.alignment_samples,
                    (unsigned long) status_navigation_elapsed_ms(navigation.elapsed_updates),
                    navigation.stationary,
                    navigation.bias_valid,
                    navigation.fault,
                    navigation.position_m[0],
                    navigation.position_m[1],
                    navigation.velocity_mps[0],
                    navigation.velocity_mps[1],
                    navigation.accel_bias_earth_mps2[0],
                    navigation.accel_bias_earth_mps2[1]);
    TaskSerial_Write(line);
}
#endif

void SerialStatus_WriteSnapshot(RuntimeState_t *state, char *args)
{
    RuntimeConfig_t flash_config;
    uint8_t flash_config_valid = status_load_flash_config(&flash_config);

    args = status_skip_spaces(args);
    serial_write_status_version();

    if (*args == '\0') {
        serial_write_status_config(&flash_config, flash_config_valid);
        serial_write_status_imu(state);
        return;
    }

    if ((strncmp(args, "CONFIG", 6U) == 0) && (status_args_end(args + 6) != 0U)) {
        serial_write_status_config(&flash_config, flash_config_valid);
    } else if ((strncmp(args, "IMU", 3U) == 0) && (status_args_end(args + 3) != 0U)) {
        serial_write_status_imu(state);
    } else if ((strncmp(args, "BIAS", 4U) == 0) && (status_args_end(args + 4) != 0U)) {
        serial_write_status_bias(&flash_config, flash_config_valid);
    } else if ((strncmp(args, "QUALITY", 7U) == 0) && (status_args_end(args + 7) != 0U)) {
        serial_write_status_quality(&flash_config, flash_config_valid);
    } else if ((strncmp(args, "HEAT", 4U) == 0) && (status_args_end(args + 4) != 0U)) {
        serial_write_status_heat(state, &flash_config, flash_config_valid);
    } else if ((strncmp(args, "DIAG", 4U) == 0) && (status_args_end(args + 4) != 0U)) {
        serial_write_status_diag();
    } else if ((strncmp(args, "INS", 3U) == 0) && (status_args_end(args + 3) != 0U)) {
#if RUNTIME_FEATURE_INS_ENABLED
        serial_write_status_ins();
#else
        TaskSerial_Write("ERROR:INS_DISABLED\n");
#endif
    } else {
        TaskSerial_Write("ERROR:STATUS_ARG\n");
    }
}
