#include "imu_diagnostics.h"

#include "bmi088_mspm0.h"
#include "bmi270_mspm0.h"
#include "serial_transport.h"

#include <stdio.h>

#define BMI088_ACCEL_CHIP_ID 0x1EU
#define BMI088_GYRO_CHIP_ID  0x0FU

/*
 * IMU 硬件诊断流程：
 * 1. 使用初始化阶段保存的多次 Chip ID、复位前后 ID 和寄存器回读快照。
 * 2. 先输出兼容旧上位机的 MISSING/INIT/ID_MISMATCH 概要行。
 * 3. 再以 CAUSE 字段区分无响应、器件或片选错误、链路不稳定和配置失败。
 * 4. 这里只解释已经采集到的诊断信息，不执行 SPI 访问，也不改变 IMU 运行状态。
 */

static uint8_t chip_id_looks_disconnected(uint8_t chip_id)
{
    return ((chip_id == 0x00U) || (chip_id == 0xFFU)) ? 1U : 0U;
}

static uint8_t chip_id_is_supported_bmi2xx(uint8_t chip_id)
{
    return ((chip_id == BMI270_CHIP_ID_VALUE) ||
            (chip_id == BMI220_CHIP_ID_VALUE) ||
            (chip_id == BMI260_CHIP_ID_VALUE)) ? 1U : 0U;
}

static const char *chip_id_model_name(uint8_t chip_id)
{
    if (chip_id == BMI088_ACCEL_CHIP_ID) {
        return "BMI088_ACCEL";
    }
    if (chip_id == BMI088_GYRO_CHIP_ID) {
        return "BMI088_GYRO";
    }
    if (chip_id == BMI270_CHIP_ID_VALUE) {
        return "BMI270";
    }
    if (chip_id == BMI220_CHIP_ID_VALUE) {
        return "BMI220";
    }
    if (chip_id == BMI260_CHIP_ID_VALUE) {
        return "BMI260";
    }
    return "UNKNOWN";
}

static uint8_t id_history_is_unstable(uint8_t valid_reads,
                                      uint8_t no_response_reads,
                                      uint8_t other_reads,
                                      uint8_t transitions)
{
    /* 仅出现 0x00/0xFF 时仍归类为无响应；二者跳变通常只是 MISO 浮空。 */
    if ((valid_reads == 0U) && (other_reads == 0U)) {
        return 0U;
    }
    if (transitions != 0U) {
        return 1U;
    }
    if ((valid_reads != 0U) && ((no_response_reads != 0U) || (other_reads != 0U))) {
        return 1U;
    }
    if ((no_response_reads != 0U) && (other_reads != 0U)) {
        return 1U;
    }
    return 0U;
}

static uint8_t write_bmi088_channel_fault(const char *channel,
                                          uint8_t chip_id,
                                          uint8_t expected_id,
                                          uint8_t init_error,
                                          uint8_t first_fail_reg,
                                          uint8_t first_fail_expected,
                                          uint8_t first_fail_read,
                                          uint8_t valid_reads,
                                          uint8_t no_response_reads,
                                          uint8_t other_reads,
                                          uint8_t transitions)
{
    char line[176];
    uint8_t unstable = id_history_is_unstable(valid_reads,
                                               no_response_reads,
                                               other_reads,
                                               transitions);

    if ((chip_id != expected_id) || (init_error == BMI088_NO_SENSOR)) {
        if (unstable != 0U) {
            (void) snprintf(line, sizeof(line),
                            "ERROR:%s:CAUSE=UNSTABLE_LINK,ID=0x%02X,V=%u,N=%u,O=%u,T=%u\n",
                            channel,
                            chip_id,
                            valid_reads,
                            no_response_reads,
                            other_reads,
                            transitions);
        } else if (chip_id_looks_disconnected(chip_id) != 0U) {
            (void) snprintf(line, sizeof(line),
                            "ERROR:%s:CAUSE=NO_RESPONSE,ID=0x%02X,HINT=CHECK_POWER_SOLDER_CS\n",
                            channel,
                            chip_id);
        } else {
            (void) snprintf(line, sizeof(line),
                            "ERROR:%s:CAUSE=WRONG_PART_OR_CS,ID=0x%02X,EXPECTED=0x%02X,DETECTED=%s\n",
                            channel,
                            chip_id,
                            expected_id,
                            chip_id_model_name(chip_id));
        }
        TaskSerial_Write(line);
        return 1U;
    }

    if (init_error != BMI088_NO_ERROR) {
        (void) snprintf(line, sizeof(line),
                        "ERROR:%s:CAUSE=INIT_CONFIG,CODE=0x%02X,REG=0x%02X,EXP=0x%02X,READ=0x%02X\n",
                        channel,
                        init_error,
                        first_fail_reg,
                        first_fail_expected,
                        first_fail_read);
        TaskSerial_Write(line);
        return 1U;
    }

    return 0U;
}

static uint8_t bmi270_id_history_is_unstable(const RuntimeState_t *state)
{
    uint8_t model_mask = BMI270_Debug.IdValidModelMask;

    if ((state->bmi270_init_error & BMI270_LINK_UNSTABLE_ERROR) != 0U) {
        return 1U;
    }
    if ((model_mask != 0U) && ((model_mask & (uint8_t)(model_mask - 1U)) != 0U)) {
        return 1U;
    }
    if ((chip_id_is_supported_bmi2xx(BMI270_Debug.ChipIdBeforeReset) != 0U) &&
        (chip_id_is_supported_bmi2xx(BMI270_Debug.ChipIdAfterReset) != 0U) &&
        (BMI270_Debug.ChipIdBeforeReset != BMI270_Debug.ChipIdAfterReset)) {
        return 1U;
    }
    return id_history_is_unstable(BMI270_Debug.IdValidReads,
                                  BMI270_Debug.IdNoResponseReads,
                                  BMI270_Debug.IdOtherReads,
                                  BMI270_Debug.IdTransitions);
}

static void write_bmi270_fault_detail(const RuntimeState_t *state)
{
    char line[192];
    const char *model = chip_id_model_name(state->bmi270_chip_id);

    if (bmi270_id_history_is_unstable(state) != 0U) {
        (void) snprintf(line, sizeof(line),
                        "ERROR:BMI270:CAUSE=UNSTABLE_LINK,ID0=0x%02X,ID1=0x%02X,ID=0x%02X,V=%u,N=%u,O=%u,T=%u\n",
                        BMI270_Debug.ChipIdBeforeReset,
                        BMI270_Debug.ChipIdAfterReset,
                        state->bmi270_chip_id,
                        BMI270_Debug.IdValidReads,
                        BMI270_Debug.IdNoResponseReads,
                        BMI270_Debug.IdOtherReads,
                        BMI270_Debug.IdTransitions);
        TaskSerial_Write(line);
        return;
    }

    if (chip_id_looks_disconnected(state->bmi270_chip_id) != 0U) {
        (void) snprintf(line, sizeof(line),
                        "ERROR:BMI270:CAUSE=NO_RESPONSE,ID=0x%02X,HINT=CHECK_POWER_SOLDER_CS\n",
                        state->bmi270_chip_id);
        TaskSerial_Write(line);
        return;
    }

    if (chip_id_is_supported_bmi2xx(state->bmi270_chip_id) == 0U) {
        (void) snprintf(line, sizeof(line),
                        "ERROR:BMI270:CAUSE=WRONG_PART_OR_CS,ID=0x%02X,EXPECTED=0x24|0x26|0x27,DETECTED=%s\n",
                        state->bmi270_chip_id,
                        model);
        TaskSerial_Write(line);
        return;
    }

    if ((state->bmi270_init_error & BMI270_CONFIG_LOAD_ERROR) != 0U) {
        (void) snprintf(line, sizeof(line),
                        "ERROR:BMI270:CAUSE=INIT_CONFIG,MODEL=%s,STAGE=CONFIG_LOAD,INTERNAL=0x%02X,ERR=0x%02X\n",
                        model,
                        BMI270_Debug.InternalStatus,
                        BMI270_Debug.ErrorReg);
        TaskSerial_Write(line);
    }
    if ((state->bmi270_init_error & BMI270_POWER_CONFIG_ERROR) != 0U) {
        (void) snprintf(line, sizeof(line),
                        "ERROR:BMI270:CAUSE=INIT_CONFIG,MODEL=%s,STAGE=POWER,PWR=0x%02X\n",
                        model,
                        BMI270_Debug.PwrCtrlAfterEnable);
        TaskSerial_Write(line);
    }
    if ((state->bmi270_init_error & BMI270_SENSOR_CONFIG_ERROR) != 0U) {
        (void) snprintf(line, sizeof(line),
                        "ERROR:BMI270:CAUSE=INIT_CONFIG,MODEL=%s,STAGE=SENSOR_REG,AC=0x%02X,AR=0x%02X,GC=0x%02X,GR=0x%02X\n",
                        model,
                        BMI270_Debug.AccConfAfterWrite,
                        BMI270_Debug.AccRangeAfterWrite,
                        BMI270_Debug.GyrConfAfterWrite,
                        BMI270_Debug.GyrRangeAfterWrite);
        TaskSerial_Write(line);
    }
    if ((state->bmi270_init_error & BMI270_UNSUPPORTED_ID_ERROR) != 0U) {
        (void) snprintf(line, sizeof(line),
                        "ERROR:BMI270:CAUSE=WRONG_PART_OR_CS,ID=0x%02X,EXPECTED=0x24|0x26|0x27,DETECTED=%s\n",
                        state->bmi270_chip_id,
                        model);
        TaskSerial_Write(line);
    }
    if ((state->bmi270_init_error & (BMI270_CONFIG_LOAD_ERROR |
                                     BMI270_POWER_CONFIG_ERROR |
                                     BMI270_SENSOR_CONFIG_ERROR |
                                     BMI270_UNSUPPORTED_ID_ERROR |
                                     BMI270_LINK_UNSTABLE_ERROR)) == 0U) {
        (void) snprintf(line, sizeof(line),
                        "ERROR:BMI270:CAUSE=INIT_UNKNOWN,MODEL=%s,FLAGS=0x%02X\n",
                        model,
                        state->bmi270_init_error);
        TaskSerial_Write(line);
    }
}

void ImuDiagnostics_WriteFaultReport(const RuntimeState_t *state)
{
    char line[80];
    uint8_t report_mask = state->required_imu_mask;

    /* AUTO 模式下仅在两路都不可用时进入本函数，此时仍给出两路诊断。 */
    if ((report_mask == 0U) && (state->active_imu_mask == 0U)) {
        report_mask = IMU_MASK_DUAL;
    }

    if ((report_mask & IMU_MASK_BMI088) != 0U) {
        if (state->bmi088_present == 0U) {
            if ((state->accel_chip_id == BMI088_ACCEL_CHIP_ID) &&
                (state->gyro_chip_id == BMI088_GYRO_CHIP_ID)) {
                (void) snprintf(line, sizeof(line), "ERROR:BMI088_INIT:0x%02X\n",
                                state->bmi088_init_error);
                TaskSerial_Write(line);
            } else if ((chip_id_looks_disconnected(state->accel_chip_id) != 0U) &&
                       (chip_id_looks_disconnected(state->gyro_chip_id) != 0U)) {
                TaskSerial_Write("ERROR:BMI088_MISSING\n");
            } else {
                (void) snprintf(line, sizeof(line),
                                "ERROR:BMI088_ID_MISMATCH:A=0x%02X,G=0x%02X\n",
                                state->accel_chip_id,
                                state->gyro_chip_id);
                TaskSerial_Write(line);
            }

            if ((state->accel_chip_id == BMI088_GYRO_CHIP_ID) &&
                (state->gyro_chip_id == BMI088_ACCEL_CHIP_ID)) {
                (void) snprintf(line, sizeof(line),
                                "ERROR:BMI088:CAUSE=CS_SWAPPED,A_ID=0x%02X,G_ID=0x%02X\n",
                                state->accel_chip_id,
                                state->gyro_chip_id);
                TaskSerial_Write(line);
            } else {
                uint8_t detail_count = 0U;

                detail_count += write_bmi088_channel_fault(
                    "BMI088_ACCEL",
                    state->accel_chip_id,
                    BMI088_ACCEL_CHIP_ID,
                    BMI088_Debug.AccelInitError,
                    BMI088_Debug.AccelFirstFailReg,
                    BMI088_Debug.AccelFirstFailExpected,
                    BMI088_Debug.AccelFirstFailRead,
                    BMI088_Debug.AccelIdValidReads,
                    BMI088_Debug.AccelIdNoResponseReads,
                    BMI088_Debug.AccelIdOtherReads,
                    BMI088_Debug.AccelIdTransitions);
                detail_count += write_bmi088_channel_fault(
                    "BMI088_GYRO",
                    state->gyro_chip_id,
                    BMI088_GYRO_CHIP_ID,
                    BMI088_Debug.GyroInitError,
                    BMI088_Debug.GyroFirstFailReg,
                    BMI088_Debug.GyroFirstFailExpected,
                    BMI088_Debug.GyroFirstFailRead,
                    BMI088_Debug.GyroIdValidReads,
                    BMI088_Debug.GyroIdNoResponseReads,
                    BMI088_Debug.GyroIdOtherReads,
                    BMI088_Debug.GyroIdTransitions);
                if (detail_count == 0U) {
                    (void) snprintf(line, sizeof(line),
                                    "ERROR:BMI088:CAUSE=INIT_UNKNOWN,FLAGS=0x%02X\n",
                                    state->bmi088_init_error);
                    TaskSerial_Write(line);
                }
            }
        }
    }

    if ((report_mask & IMU_MASK_BMI270) != 0U) {
        if (state->bmi270_present == 0U) {
            if ((state->bmi270_chip_id == BMI270_CHIP_ID_VALUE) ||
                (state->bmi270_chip_id == BMI220_CHIP_ID_VALUE) ||
                (state->bmi270_chip_id == BMI260_CHIP_ID_VALUE)) {
                const char *model = (state->bmi270_chip_id == BMI220_CHIP_ID_VALUE) ? "BMI220" :
                                    (state->bmi270_chip_id == BMI260_CHIP_ID_VALUE) ? "BMI260" :
                                                                                     "BMI270";
                (void) snprintf(line, sizeof(line), "ERROR:%s_INIT:0x%02X\n",
                                model,
                                state->bmi270_init_error);
                TaskSerial_Write(line);
            } else if (chip_id_looks_disconnected(state->bmi270_chip_id) != 0U) {
                TaskSerial_Write("ERROR:BMI270_MISSING\n");
            } else {
                (void) snprintf(line, sizeof(line), "ERROR:BMI270_ID_MISMATCH:0x%02X\n",
                                state->bmi270_chip_id);
                TaskSerial_Write(line);
            }
            write_bmi270_fault_detail(state);
        }
    }

    if (state->active_imu_mask == 0U) {
        TaskSerial_Write("ERROR:NO_ACTIVE_IMU\n");
    }
}
