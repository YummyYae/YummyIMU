#ifndef MAHONY_FILTER_H
#define MAHONY_FILTER_H

#define DEG2RAD 0.01745329252f
#define RAD2DEG 57.295779513f

typedef struct {
    float x;
    float y;
    float z;
} Axis3f;

typedef struct {
    float Kp;
    float Ki;
    float dt;
    Axis3f gyro;
    Axis3f acc;
    float exInt;
    float eyInt;
    float ezInt;
    float q0;
    float q1;
    float q2;
    float q3;
    float rMat[3][3];
    float pitch;
    float roll;
    float yaw;
} MahonyFilter_t;

void Mahony_Init(MahonyFilter_t *filter, float kp, float ki, float dt);
void Mahony_SetFromAccel(MahonyFilter_t *filter, Axis3f acc);
void Mahony_Update(MahonyFilter_t *filter, Axis3f gyro, Axis3f acc);
void Mahony_UpdateFast(MahonyFilter_t *filter, Axis3f gyro, Axis3f acc);
void Mahony_UpdateGyroOnly(MahonyFilter_t *filter, Axis3f gyro);
void Mahony_UpdateGyroOnlyFast(MahonyFilter_t *filter, Axis3f gyro);
void Mahony_RotationMatrixUpdate(MahonyFilter_t *filter);
void Mahony_Output(MahonyFilter_t *filter);

#endif
