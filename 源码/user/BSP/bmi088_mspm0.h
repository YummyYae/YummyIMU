#ifndef BMI088_MSPM0_H
#define BMI088_MSPM0_H

#include <stdint.h>

#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_EXPECTED_ACCEL_CHIP_ID 0x1EU
#define BMI088_EXPECTED_GYRO_CHIP_ID  0x0FU

/* BMI088_Init() 返回值用于标明失败的是哪一个独立芯片。 */
#define BMI088_INIT_ACCEL_FAILED 0x01U
#define BMI088_INIT_GYRO_FAILED  0x02U

#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_GYRO_2000_SEN 0.0010652644360316953f

#define BMI088_LONG_DELAY_TIME_MS 80U

#define BMI088_GX_OFFSET (-0.000681414269f)
#define BMI088_GY_OFFSET (-0.00134240754f)
#define BMI088_GZ_OFFSET (-0.00143384014f)
#define BMI088_AX_OFFSET (0.299675316f)
#define BMI088_AY_OFFSET (0.0720675737f)
#define BMI088_AZ_OFFSET (0.0f)
#define BMI088_G_NORM (9.84484291f)

typedef struct {
    float Accel[3];
    float Gyro[3];
    float TempWhenCali;
    float Temperature;
    float AccelScale;
    float GyroOffset[3];
    float AccelOffset[3];
    float gNorm;
    uint32_t GyroSaturationCount;
    uint32_t AccelSaturationCount;
    uint8_t LastGyroSaturated;
    uint8_t LastAccelSaturated;
} BMI088_Data_t;

/*
 * BMI088 实际由加速度计和陀螺仪两颗独立芯片组成。
 * 本结构保存初始化阶段的原始诊断信息，避免组合错误码掩盖具体故障通道。
 */
typedef struct {
    uint8_t AccelInitError;
    uint8_t GyroInitError;
    uint8_t AccelChipIdBeforeReset;
    uint8_t AccelChipIdAfterReset;
    uint8_t GyroChipIdBeforeReset;
    uint8_t GyroChipIdAfterReset;
    uint8_t AccelFinalChipId;
    uint8_t GyroFinalChipId;
    uint8_t AccelFirstFailReg;
    uint8_t AccelFirstFailExpected;
    uint8_t AccelFirstFailRead;
    uint8_t AccelConfigFailureMask;
    uint8_t GyroFirstFailReg;
    uint8_t GyroFirstFailExpected;
    uint8_t GyroFirstFailRead;
    uint8_t GyroConfigFailureMask;
    uint8_t AccelIdValidReads;
    uint8_t AccelIdNoResponseReads;
    uint8_t AccelIdOtherReads;
    uint8_t AccelIdTransitions;
    uint8_t GyroIdValidReads;
    uint8_t GyroIdNoResponseReads;
    uint8_t GyroIdOtherReads;
    uint8_t GyroIdTransitions;
    uint8_t AccelLastObservedId;
    uint8_t GyroLastObservedId;
    uint8_t AccelIdReadCount;
    uint8_t GyroIdReadCount;
} BMI088_Debug_t;

typedef enum {
    BMI088_NO_ERROR = 0x00,
    BMI088_ACC_PWR_CTRL_ERROR = 0x01,
    BMI088_ACC_PWR_CONF_ERROR = 0x02,
    BMI088_ACC_CONF_ERROR = 0x03,
    BMI088_ACC_RANGE_ERROR = 0x05,
    BMI088_INT1_IO_CTRL_ERROR = 0x06,
    BMI088_INT_MAP_DATA_ERROR = 0x07,
    BMI088_GYRO_RANGE_ERROR = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
    BMI088_GYRO_LPM1_ERROR = 0x0A,
    BMI088_GYRO_CTRL_ERROR = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,
    BMI088_NO_SENSOR = 0xFF,
} BMI088_Error_t;

extern BMI088_Data_t BMI088Sensor;
extern BMI088_Debug_t BMI088_Debug;

uint8_t BMI088_Init(uint8_t calibrate);
void BMI088_Read(BMI088_Data_t *BMI088Sensor);
float BMI088_ReadTemperature(void);
void BMI088_ReadTemperatureRaw(uint8_t *temp_m, uint8_t *temp_l, int16_t *raw);
uint8_t BMI088_StartupCalibrate(void);
uint8_t BMI088_ReadAccelChipId(void);
uint8_t BMI088_ReadGyroChipId(void);

#endif
