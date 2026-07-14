#ifndef IMU_ATTITUDE_H
#define IMU_ATTITUDE_H

#include "mahony_filter.h"

#include <stdint.h>

#define INS_AXIS_X 0
#define INS_AXIS_Y 1
#define INS_AXIS_Z 2

typedef struct {
    float q[4];
    float Gyro[3];
    float Accel[3];
    float MotionAccel_b[3];
    float MotionAccel_n[3];
    float AccelLPF;
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
    float YawAngleLast;
    float YawRoundCount;
    float v_n;
    float x_n;
    uint8_t ins_flag;
} INS_t;

typedef struct {
    float q[4];
    float Roll;
    float Pitch;
    float Yaw;
} IMU_Attitude_t;

/* 供高频算法直接使用的融合采样，不依赖低频欧拉角输出刷新。 */
typedef struct {
    float q[4];
    Axis3f gyro;
    Axis3f accel;
    uint8_t valid;
    uint8_t stationary;
} ImuFusedSample_t;

Axis3f ImuAttitude_BMI088RawAccelToBoard(void);
Axis3f ImuAttitude_BMI270RawAccelToBoard(void);
Axis3f ImuAttitude_BMI088AccelToBoard(void);
Axis3f ImuAttitude_BMI270AccelToBoard(void);
Axis3f ImuAttitude_BMI088GyroToBoard(void);
Axis3f ImuAttitude_BMI270GyroToBoard(void);
void ImuAttitude_GetBMI088Result(INS_t *out);
void ImuAttitude_GetBMI270Result(IMU_Attitude_t *out);
void ImuAttitude_GetFusedResult(IMU_Attitude_t *out);
void ImuAttitude_GetFusedSample(ImuFusedSample_t *out);
void ImuAttitude_Init(void);
void ImuAttitude_SetDebugMode(uint8_t enable);
void ImuAttitude_SetAccelCalibration(const float bmi088_offset[3],
                                     const float bmi270_offset[3],
                                     const float bmi088_scale[3],
                                     const float bmi270_scale[3],
                                     uint8_t valid);
void ImuAttitude_SetInputMask(uint8_t mask);
void ImuAttitude_SetYawErrorDegPerTurn(float bmi088_error, float bmi270_error);
void ImuAttitude_SetInitialAccel(Axis3f bmi088_accel, Axis3f bmi270_accel);
void ImuAttitude_Update(float dt);
void ImuAttitude_UpdateOutput(void);

#endif
