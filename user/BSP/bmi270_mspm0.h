#ifndef BMI270_MSPM0_H
#define BMI270_MSPM0_H

#include <stdint.h>

#define BMI270_CHIP_ID_VALUE 0x24U
#define BMI220_CHIP_ID_VALUE 0x26U
#define BMI270_ALT_CHIP_ID_VALUE BMI220_CHIP_ID_VALUE

#define BMI270_ACCEL_8G_SEN (9.80665f * 8.0f / 32768.0f)
#define BMI270_GYRO_2000_SEN (2000.0f * 0.01745329252f / 32768.0f)

typedef struct {
    float Accel[3];
    float Gyro[3];
    float GyroOffset[3];
    float Temperature;
    uint32_t GyroSaturationCount;
    uint32_t AccelSaturationCount;
    uint8_t ChipId;
    uint8_t InternalStatus;
    uint8_t InitError;
    uint8_t LastGyroSaturated;
    uint8_t LastAccelSaturated;
} BMI270_Data_t;

typedef struct {
    uint8_t ChipIdBeforeReset;
    uint8_t ChipIdAfterReset;
    uint8_t PwrConfAfterSuspend;
    uint8_t InitCtrlBeforeLoad;
    uint8_t InitCtrlAfterLoad;
    uint8_t InternalStatus;
    uint8_t ErrorReg;
    uint8_t PwrCtrlAfterEnable;
    uint8_t AccConfAfterWrite;
    uint8_t AccRangeAfterWrite;
    uint8_t GyrConfAfterWrite;
    uint8_t GyrRangeAfterWrite;
    uint8_t CompatVariant;
    uint8_t CompatDataNonZero;
    uint8_t StatusAfterEnable;
    uint8_t Reserved;
} BMI270_Debug_t;

typedef enum {
    BMI270_NO_ERROR = 0x00,
    BMI270_NO_SENSOR = 0x01,
    BMI270_CONFIG_LOAD_ERROR = 0x02,
    BMI270_POWER_CONFIG_ERROR = 0x04,
    BMI270_SENSOR_CONFIG_ERROR = 0x08,
} BMI270_Error_t;

extern BMI270_Data_t BMI270Sensor;
extern BMI270_Debug_t BMI270_Debug;
extern uint8_t BMI270_RegDumpAfterReset[128];
extern uint8_t BMI270_RegDumpAfterConfig[128];
extern uint8_t BMI270_RegDumpAfterPower[128];

uint8_t BMI270_Init(void);
void BMI270_Read(BMI270_Data_t *BMI270Sensor);
float BMI270_ReadTemperature(void);
uint8_t BMI270_ReadChipId(void);
uint8_t BMI270_ReadInternalStatus(void);
void BMI270_UpdateDebugSnapshot(void);

#endif
