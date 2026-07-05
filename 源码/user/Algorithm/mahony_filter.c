#include "mahony_filter.h"

#include <math.h>
#include <stdint.h>

static float safe_inv_sqrt(float value)
{
    union {
        float f;
        uint32_t u;
    } convert;

    if (value <= 0.0f) {
        return 0.0f;
    }

    convert.f = value;
    convert.u = 0x5f3759dfU - (convert.u >> 1);
    convert.f = convert.f * (1.5f - (0.5f * value * convert.f * convert.f));
    convert.f = convert.f * (1.5f - (0.5f * value * convert.f * convert.f));

    return convert.f;
}

void Mahony_RotationMatrixUpdate(MahonyFilter_t *filter)
{
    float q1q1 = filter->q1 * filter->q1;
    float q2q2 = filter->q2 * filter->q2;
    float q3q3 = filter->q3 * filter->q3;
    float q0q1 = filter->q0 * filter->q1;
    float q0q2 = filter->q0 * filter->q2;
    float q0q3 = filter->q0 * filter->q3;
    float q1q2 = filter->q1 * filter->q2;
    float q1q3 = filter->q1 * filter->q3;
    float q2q3 = filter->q2 * filter->q3;

    filter->rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    filter->rMat[0][1] = 2.0f * (q1q2 - q0q3);
    filter->rMat[0][2] = 2.0f * (q1q3 + q0q2);
    filter->rMat[1][0] = 2.0f * (q1q2 + q0q3);
    filter->rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    filter->rMat[1][2] = 2.0f * (q2q3 - q0q1);
    filter->rMat[2][0] = 2.0f * (q1q3 - q0q2);
    filter->rMat[2][1] = 2.0f * (q2q3 + q0q1);
    filter->rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

void Mahony_Output(MahonyFilter_t *filter)
{
    filter->pitch = -asinf(filter->rMat[2][0]);
    filter->roll = atan2f(filter->rMat[2][1], filter->rMat[2][2]);
    filter->yaw = atan2f(filter->rMat[1][0], filter->rMat[0][0]);
}

void Mahony_SetFromAccel(MahonyFilter_t *filter, Axis3f acc)
{
    float inv_norm = safe_inv_sqrt((acc.x * acc.x) + (acc.y * acc.y) + (acc.z * acc.z));
    float roll;
    float pitch;
    float cr;
    float sr;
    float cp;
    float sp;

    if (inv_norm <= 0.0f) {
        return;
    }

    acc.x *= inv_norm;
    acc.y *= inv_norm;
    acc.z *= inv_norm;

    roll = atan2f(acc.y, acc.z);
    pitch = atan2f(-acc.x, sqrtf((acc.y * acc.y) + (acc.z * acc.z)));

    cr = cosf(roll * 0.5f);
    sr = sinf(roll * 0.5f);
    cp = cosf(pitch * 0.5f);
    sp = sinf(pitch * 0.5f);

    filter->q0 = cr * cp;
    filter->q1 = sr * cp;
    filter->q2 = cr * sp;
    filter->q3 = -sr * sp;
    filter->exInt = 0.0f;
    filter->eyInt = 0.0f;
    filter->ezInt = 0.0f;
    Mahony_RotationMatrixUpdate(filter);
    Mahony_Output(filter);
}

void Mahony_Init(MahonyFilter_t *filter, float kp, float ki, float dt)
{
    filter->Kp = kp;
    filter->Ki = ki;
    filter->dt = dt;
    filter->gyro = (Axis3f) {0.0f, 0.0f, 0.0f};
    filter->acc = (Axis3f) {0.0f, 0.0f, 0.0f};
    filter->exInt = 0.0f;
    filter->eyInt = 0.0f;
    filter->ezInt = 0.0f;
    filter->q0 = 1.0f;
    filter->q1 = 0.0f;
    filter->q2 = 0.0f;
    filter->q3 = 0.0f;
    Mahony_RotationMatrixUpdate(filter);
    Mahony_Output(filter);
}

static void mahony_update_internal(MahonyFilter_t *filter, Axis3f gyro, Axis3f acc, uint8_t update_output)
{
    float inv_norm;
    float ex;
    float ey;
    float ez;
    float q0_last;
    float q1_last;
    float q2_last;
    float q3_last;
    float half_t;

    filter->gyro = gyro;
    filter->acc = acc;

    inv_norm = safe_inv_sqrt((acc.x * acc.x) + (acc.y * acc.y) + (acc.z * acc.z));
    if (inv_norm <= 0.0f) {
        return;
    }

    filter->acc.x *= inv_norm;
    filter->acc.y *= inv_norm;
    filter->acc.z *= inv_norm;

    ex = (filter->acc.y * filter->rMat[2][2]) - (filter->acc.z * filter->rMat[2][1]);
    ey = (filter->acc.z * filter->rMat[2][0]) - (filter->acc.x * filter->rMat[2][2]);
    ez = (filter->acc.x * filter->rMat[2][1]) - (filter->acc.y * filter->rMat[2][0]);

    filter->exInt += filter->Ki * ex * filter->dt;
    filter->eyInt += filter->Ki * ey * filter->dt;
    filter->ezInt += filter->Ki * ez * filter->dt;

    filter->gyro.x += filter->Kp * ex + filter->exInt;
    filter->gyro.y += filter->Kp * ey + filter->eyInt;
    filter->gyro.z += filter->Kp * ez + filter->ezInt;

    q0_last = filter->q0;
    q1_last = filter->q1;
    q2_last = filter->q2;
    q3_last = filter->q3;
    half_t = filter->dt * 0.5f;

    filter->q0 += (-q1_last * filter->gyro.x - q2_last * filter->gyro.y - q3_last * filter->gyro.z) * half_t;
    filter->q1 += (q0_last * filter->gyro.x + q2_last * filter->gyro.z - q3_last * filter->gyro.y) * half_t;
    filter->q2 += (q0_last * filter->gyro.y - q1_last * filter->gyro.z + q3_last * filter->gyro.x) * half_t;
    filter->q3 += (q0_last * filter->gyro.z + q1_last * filter->gyro.y - q2_last * filter->gyro.x) * half_t;

    inv_norm = safe_inv_sqrt((filter->q0 * filter->q0) + (filter->q1 * filter->q1) +
                             (filter->q2 * filter->q2) + (filter->q3 * filter->q3));
    if (inv_norm <= 0.0f) {
        Mahony_Init(filter, filter->Kp, filter->Ki, filter->dt);
        return;
    }

    filter->q0 *= inv_norm;
    filter->q1 *= inv_norm;
    filter->q2 *= inv_norm;
    filter->q3 *= inv_norm;

    Mahony_RotationMatrixUpdate(filter);
    if (update_output != 0U) {
        Mahony_Output(filter);
    }
}

static void mahony_update_gyro_only_internal(MahonyFilter_t *filter, Axis3f gyro, uint8_t update_output)
{
    float inv_norm;
    float q0_last;
    float q1_last;
    float q2_last;
    float q3_last;
    float half_t;

    filter->gyro = gyro;

    q0_last = filter->q0;
    q1_last = filter->q1;
    q2_last = filter->q2;
    q3_last = filter->q3;
    half_t = filter->dt * 0.5f;

    filter->q0 += (-q1_last * gyro.x - q2_last * gyro.y - q3_last * gyro.z) * half_t;
    filter->q1 += (q0_last * gyro.x + q2_last * gyro.z - q3_last * gyro.y) * half_t;
    filter->q2 += (q0_last * gyro.y - q1_last * gyro.z + q3_last * gyro.x) * half_t;
    filter->q3 += (q0_last * gyro.z + q1_last * gyro.y - q2_last * gyro.x) * half_t;

    inv_norm = safe_inv_sqrt((filter->q0 * filter->q0) + (filter->q1 * filter->q1) +
                             (filter->q2 * filter->q2) + (filter->q3 * filter->q3));
    if (inv_norm <= 0.0f) {
        Mahony_Init(filter, filter->Kp, filter->Ki, filter->dt);
        return;
    }

    filter->q0 *= inv_norm;
    filter->q1 *= inv_norm;
    filter->q2 *= inv_norm;
    filter->q3 *= inv_norm;

    Mahony_RotationMatrixUpdate(filter);
    if (update_output != 0U) {
        Mahony_Output(filter);
    }
}

void Mahony_Update(MahonyFilter_t *filter, Axis3f gyro, Axis3f acc)
{
    mahony_update_internal(filter, gyro, acc, 1U);
}

void Mahony_UpdateFast(MahonyFilter_t *filter, Axis3f gyro, Axis3f acc)
{
    mahony_update_internal(filter, gyro, acc, 0U);
}

void Mahony_UpdateGyroOnly(MahonyFilter_t *filter, Axis3f gyro)
{
    mahony_update_gyro_only_internal(filter, gyro, 1U);
}

void Mahony_UpdateGyroOnlyFast(MahonyFilter_t *filter, Axis3f gyro)
{
    mahony_update_gyro_only_internal(filter, gyro, 0U);
}
