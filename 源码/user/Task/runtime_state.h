#ifndef RUNTIME_STATE_H
#define RUNTIME_STATE_H

#include "flash_storage.h"

#include <stdint.h>

#define IMU_UPDATE_RATE_HZ 1000U
#define IMU_UPDATE_DT_SECONDS 0.0010f
#define UART_DIAGNOSTIC_HEARTBEAT 0U
#define COMMAND_REPORT_PAUSE_MS 3000U

#define IMU_MASK_BMI088 0x01U
#define IMU_MASK_BMI270 0x02U
#define IMU_MASK_DUAL   (IMU_MASK_BMI088 | IMU_MASK_BMI270)

typedef struct {
    uint32_t update_count;
    uint32_t overrun_count;
    uint32_t reserved[2];
} ImuRuntimeStats_t;

typedef struct {
    uint8_t bmi088_init_error;
    uint8_t bmi270_init_error;
    uint8_t accel_chip_id;
    uint8_t gyro_chip_id;
    uint8_t bmi270_chip_id;
    uint8_t gyro_bias_valid;
    uint8_t gyro_bias_calibrated;
    uint8_t bmi088_temperature_valid;
    uint8_t bmi270_temperature_valid;
    uint8_t bmi088_temperature_filter_valid;
    uint8_t bmi270_temperature_filter_valid;
    uint8_t bmi088_temp_msb;
    uint8_t bmi088_temp_lsb;
    uint8_t bmi088_present;
    uint8_t bmi270_present;
    uint8_t active_imu_mask;
    uint8_t required_imu_mask;
    int16_t bmi088_temp_raw;
    float bmi088_temperature_filtered;
    float bmi270_temperature_filtered;
    uint32_t uart_report_ticks;
    uint32_t uart_report_resume_tick;
    float bmi088_heater_pid_integral;
    float bmi088_heater_pid_last_error;
    float bmi088_heater_duty;
    float bmi270_heater_pid_integral;
    float bmi270_heater_pid_last_error;
    float bmi270_heater_duty;
    RuntimeConfig_t runtime_config;
} RuntimeState_t;

extern RuntimeState_t gRuntimeState;
extern volatile uint32_t gSystemTickMs;
extern volatile ImuRuntimeStats_t gImuRuntimeStats;

void RuntimeState_LoadConfig(RuntimeState_t *ctx);
void RuntimeState_UpdateReportRate(RuntimeState_t *ctx);
uint8_t RuntimeState_BaudRateIsSupported(uint32_t baud_rate);
void RuntimeState_OnSysTick(void);
void RuntimeState_OnTemperatureTick(void);
uint8_t RuntimeState_TakeTemperatureUpdate(void);
void RuntimeState_ClearImuUpdates(void);
void RuntimeState_ClearImuRuntimeStats(void);
void RuntimeState_WaitForInterrupt(void);

#endif


