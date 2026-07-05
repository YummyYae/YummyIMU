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

extern INS_t BMI088;
extern IMU_Attitude_t BMI270;
extern IMU_Attitude_t VirtualIMU;
extern Axis3f DualGyroKalman_Omega;
extern Axis3f DualGyroKalman_Accel;
extern Axis3f BMI088BoardAccel;
extern Axis3f BMI088BoardGyro;
extern Axis3f BMI270BoardAccel;
extern Axis3f BMI270BoardGyro;

Axis3f ImuAttitude_BMI088AccelToBoard(void);
Axis3f ImuAttitude_BMI270AccelToBoard(void);
Axis3f ImuAttitude_BMI088GyroToBoard(void);
Axis3f ImuAttitude_BMI270GyroToBoard(void);
void ImuAttitude_Init(void);
void ImuAttitude_SetDebugMode(uint8_t enable);
void ImuAttitude_SetInitialAccel(Axis3f bmi088_accel, Axis3f bmi270_accel);
void ImuAttitude_Update(float dt);
void ImuAttitude_UpdateOutput(void);

#endif
