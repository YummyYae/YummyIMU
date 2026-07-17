#include "imu_report.h"

#include "bmi088_mspm0.h"
#include "bmi270_mspm0.h"
#include "board_mspm0.h"
#include "imu_attitude.h"
#include "imu_diagnostics.h"
#if RUNTIME_FEATURE_INS_ENABLED
#include "inertial_navigation.h"
#endif
#include "serial_transport.h"
#include "task_led.h"
#include "task_temperature.h"

#include <stdio.h>

#define TEMP_TUNING_FIRMWARE        0U
#define IMU_ERROR_REPORT_PERIOD_MS  1000U
#define BINARY_ATTITUDE_HEADER      0xA5U
#define BINARY_ATTITUDE_PACKET_SIZE 8U
#define BINARY_ANGLE_SCALE          100.0f

/*
 * IMU 回传流程：
 * 1. 1kHz 中断只提交 report_pending，所有格式化与阻塞发送都留在前台。
 * 2. IMU 故障时停止姿态流，以 1 秒周期输出可机器解析的诊断信息。
 * 3. INS 未进入运行态时输出等待、对齐或数据异常状态，不混入位置数据。
 * 4. USE/DEBUG 输出文本，BINARY 输出固定 8 字节数据包，具体格式与原协议保持一致。
 */

#if RUNTIME_FEATURE_INS_ENABLED
static void imu_report_get_navigation_snapshot(InertialNavigationSnapshot_t *out)
{
    Board_EnterCritical();
    InertialNavigation_GetSnapshot(out);
    Board_ExitCritical();
}

static void imu_report_service_navigation_start_event(void)
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

static void binary_write_i16_le(uint8_t packet[BINARY_ATTITUDE_PACKET_SIZE],
                                uint8_t offset,
                                int16_t value)
{
    uint16_t encoded = (uint16_t)value;

    packet[offset] = (uint8_t)(encoded & 0xFFU);
    packet[offset + 1U] = (uint8_t)(encoded >> 8U);
}

/* 输出 A5 + Pitch/Roll/Yaw(int16, 0.01度, 小端) + 累加和校验。 */
static void imu_report_write_binary_attitude(const IMU_Attitude_t *attitude)
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

static void imu_report_write_output(const RuntimeState_t *state)
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

        imu_report_get_navigation_snapshot(&navigation);
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
        imu_report_write_binary_attitude(&virtual_result);
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

void ImuReport_Service(RuntimeState_t *state,
                       uint8_t report_pending,
                       uint8_t imu_fault,
                       uint8_t imu_alarm)
{
    static uint32_t last_error_report_tick;
#if RUNTIME_FEATURE_INS_ENABLED
    static uint32_t last_navigation_notice_tick;
#endif

#if RUNTIME_FEATURE_INS_ENABLED
    imu_report_service_navigation_start_event();
#endif

    if (imu_fault != 0U) {
        TaskLED_UpdateSystemStatus(1U, 1U, 0U, 1U);
        if (((int32_t)(gSystemTickMs - state->uart_report_resume_tick) >= 0) &&
            ((uint32_t)(gSystemTickMs - last_error_report_tick) >=
             IMU_ERROR_REPORT_PERIOD_MS)) {
            last_error_report_tick = gSystemTickMs;
            ImuDiagnostics_WriteFaultReport(state);
        }
        return;
    }

    if (report_pending == 0U) {
        return;
    }

#if RUNTIME_FEATURE_INS_ENABLED
    if (state->runtime_config.output_mode == RUNTIME_OUTPUT_MODE_INS) {
        InertialNavigationSnapshot_t navigation;

        imu_report_get_navigation_snapshot(&navigation);
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
                                   imu_alarm,
                                   TaskTemperature_IsReady(state, 2.0f),
                                   1U);
        imu_report_write_output(state);
    } else {
        TaskLED_Set(0U);
    }
}
