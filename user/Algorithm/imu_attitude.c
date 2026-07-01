#include "imu_attitude.h"

#include "bmi088_mspm0.h"
#include "bmi270_mspm0.h"

#include <math.h>
#include <stdint.h>

INS_t BMI088;
IMU_Attitude_t BMI270;

#define ENABLE_REAL_IMU_ATTITUDE_SOLVERS 1

static void fused_attitude_init(void);
static void fused_attitude_set_initial_accel(Axis3f accel);
static void fused_attitude_update(float dt);
static void fused_attitude_update_output(void);

/*
 * 旧的两颗真实 IMU 独立姿态解算代码先保留在这里，但当前固件不参与编译。
 * 以后需要恢复 BMI088/BMI270 独立角度输出时，将 ENABLE_REAL_IMU_ATTITUDE_SOLVERS 改为 1，
 * 再在 ImuAttitude_* 入口里重新打开对应调用即可。
 */
#if ENABLE_REAL_IMU_ATTITUDE_SOLVERS
#define BMI088_MAHONY_KP 0.0f

static MahonyFilter_t gMahony;
static MahonyFilter_t gBmi270Mahony;

static void bmi088_attitude_init(void);
static void bmi088_attitude_set_initial_accel(Axis3f accel);
static void bmi088_attitude_update(float dt);
static void bmi088_attitude_update_output(void);
static void bmi270_attitude_init(void);
static void bmi270_attitude_set_initial_accel(Axis3f accel);
static void bmi270_attitude_update(float dt);
static void bmi270_attitude_update_output(void);

static float wrap_pi(float angle)
{
    const float pi = 3.1415926f;
    const float two_pi = 6.2831852f;

    while (angle > pi) {
        angle -= two_pi;
    }
    while (angle < -pi) {
        angle += two_pi;
    }

    return angle;
}

static void bmi088_attitude_init(void)
{
    Mahony_Init(&gMahony, BMI088_MAHONY_KP, 0.0f, 0.0010f);
    BMI088.AccelLPF = 0.0089f;
    BMI088.q[0] = 1.0f;
    BMI088.q[1] = 0.0f;
    BMI088.q[2] = 0.0f;
    BMI088.q[3] = 0.0f;
}

static void bmi088_attitude_set_initial_accel(Axis3f accel)
{
    Mahony_SetFromAccel(&gMahony, accel);
    BMI088.q[0] = gMahony.q0;
    BMI088.q[1] = gMahony.q1;
    BMI088.q[2] = gMahony.q2;
    BMI088.q[3] = gMahony.q3;
    bmi088_attitude_update_output();
}

static void bmi088_attitude_update(float dt)
{
    Axis3f gyro;

    for (uint8_t i = 0; i < 3U; i++) {
        BMI088.Accel[i] = BMI088Sensor.Accel[i];
        BMI088.Gyro[i] = BMI088Sensor.Gyro[i];
    }

    gyro.x = BMI088Sensor.Gyro[0];
    gyro.y = BMI088Sensor.Gyro[1];
    gyro.z = BMI088Sensor.Gyro[2];

    if (BMI088Sensor.LastGyroSaturated != 0U) {
        return;
    }

    gMahony.dt = dt;
    Mahony_UpdateGyroOnlyFast(&gMahony, gyro);

    BMI088.q[0] = gMahony.q0;
    BMI088.q[1] = gMahony.q1;
    BMI088.q[2] = gMahony.q2;
    BMI088.q[3] = gMahony.q3;
    BMI088.ins_flag = 1U;

}

static void bmi088_attitude_update_output(void)
{
    Mahony_Output(&gMahony);

    BMI088.Pitch = wrap_pi(gMahony.roll + 0.0305f);
    BMI088.Roll = gMahony.pitch;
    BMI088.Yaw = gMahony.yaw;

    if ((BMI088.Yaw - BMI088.YawAngleLast) > 3.1415926f) {
        BMI088.YawRoundCount -= 1.0f;
    } else if ((BMI088.Yaw - BMI088.YawAngleLast) < -3.1415926f) {
        BMI088.YawRoundCount += 1.0f;
    }

    BMI088.YawTotalAngle = (6.2831852f * BMI088.YawRoundCount) + BMI088.Yaw;
    BMI088.YawAngleLast = BMI088.Yaw;
}

void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * (((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0]) +
                       ((q[1] * q[2] - q[0] * q[3]) * vecBF[1]) +
                       ((q[1] * q[3] + q[0] * q[2]) * vecBF[2]));
    vecEF[1] = 2.0f * (((q[1] * q[2] + q[0] * q[3]) * vecBF[0]) +
                       ((0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1]) +
                       ((q[2] * q[3] - q[0] * q[1]) * vecBF[2]));
    vecEF[2] = 2.0f * (((q[1] * q[3] - q[0] * q[2]) * vecBF[0]) +
                       ((q[2] * q[3] + q[0] * q[1]) * vecBF[1]) +
                       ((0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]));
}

void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * (((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0]) +
                       ((q[1] * q[2] + q[0] * q[3]) * vecEF[1]) +
                       ((q[1] * q[3] - q[0] * q[2]) * vecEF[2]));
    vecBF[1] = 2.0f * (((q[1] * q[2] - q[0] * q[3]) * vecEF[0]) +
                       ((0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1]) +
                       ((q[2] * q[3] + q[0] * q[1]) * vecEF[2]));
    vecBF[2] = 2.0f * (((q[1] * q[3] + q[0] * q[2]) * vecEF[0]) +
                       ((q[2] * q[3] - q[0] * q[1]) * vecEF[1]) +
                       ((0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]));
}

static void reset_attitude(IMU_Attitude_t *attitude)
{
    attitude->q[0] = 1.0f;
    attitude->q[1] = 0.0f;
    attitude->q[2] = 0.0f;
    attitude->q[3] = 0.0f;
    attitude->Roll = 0.0f;
    attitude->Pitch = 0.0f;
    attitude->Yaw = 0.0f;
}

/*
 * BMI220 与 BMI088 不是同向安装。
 * 实测静止重力方向后得到安装关系：
 *   BMI220 raw X -> 车体 Y
 *   BMI220 raw Y -> 车体 -X
 *   BMI220 raw Z -> 车体 Z
 * 即车体坐标 = {-raw.y, raw.x, raw.z}，加速度和陀螺仪必须使用同一个安装矩阵。
 */
static Axis3f bmi270_sensor_to_board(Axis3f sensor)
{
    Axis3f board;

    board.x = -sensor.y;
    board.y = sensor.x;
    board.z = sensor.z;

    return board;
}

static void update_attitude_from_filter(IMU_Attitude_t *attitude, const MahonyFilter_t *filter)
{
    attitude->q[0] = filter->q0;
    attitude->q[1] = filter->q1;
    attitude->q[2] = filter->q2;
    attitude->q[3] = filter->q3;
    attitude->Pitch = filter->roll;
    attitude->Roll = filter->pitch;
    attitude->Yaw = filter->yaw;
}

static void bmi270_attitude_init(void)
{
    Mahony_Init(&gBmi270Mahony, 0.0f, 0.0f, 0.0010f);
    reset_attitude(&BMI270);
}

static void bmi270_attitude_set_initial_accel(Axis3f accel)
{
    Mahony_SetFromAccel(&gBmi270Mahony, accel);
    bmi270_attitude_update_output();
}

static void bmi270_attitude_update(float dt)
{
    Axis3f gyro;

    gyro = ImuAttitude_BMI270GyroToBoard();

    if (BMI270Sensor.LastGyroSaturated != 0U) {
        return;
    }

    gBmi270Mahony.dt = dt;
    Mahony_UpdateGyroOnlyFast(&gBmi270Mahony, gyro);
}

static void bmi270_attitude_update_output(void)
{
    Mahony_Output(&gBmi270Mahony);
    update_attitude_from_filter(&BMI270, &gBmi270Mahony);
}

#endif

#define DUAL_GYRO_AXIS_COUNT 3U
#define DUAL_GYRO_STATE_SIZE 3U

#define DUAL_GYRO_Q_OMEGA_PER_SEC  0.0200f
#define DUAL_GYRO_Q_BIAS_PER_SEC   0.000002f
#define DUAL_GYRO_R_BMI088         0.000030f
#define DUAL_GYRO_R_BMI220         0.000045f
#define DUAL_GYRO_R_BMI220_GATED   0.250000f
#define DUAL_GYRO_R_BMI220_REJECTED 1000.000000f
#define DUAL_GYRO_R_STATIC         0.000001f

#define DUAL_GYRO_GRAVITY          9.80665f
#define DUAL_GYRO_ACC_STATIC_LIMIT 1.50f
#define DUAL_GYRO_ACC_MAHONY_LIMIT 1.00f
#define DUAL_GYRO_ACC_AGREE_LIMIT  2.00f
#define DUAL_GYRO_RATE_STATIC_LIMIT 0.040f
#define DUAL_GYRO_DISAGREE_LIMIT   0.350f
#define DUAL_GYRO_REJECT_LIMIT     1.800f
#define DUAL_GYRO_BMI088_WEIGHT    0.65f
#define DUAL_GYRO_BMI220_WEIGHT    0.35f
#define DUAL_GYRO_BMI220_LOW_WEIGHT 0.10f
#define DUAL_GYRO_REL_BIAS_ALPHA_PER_SEC 0.50f
#define DUAL_GYRO_GRAVITY_MIN_SQ   ((DUAL_GYRO_GRAVITY - DUAL_GYRO_ACC_STATIC_LIMIT) * \
                                    (DUAL_GYRO_GRAVITY - DUAL_GYRO_ACC_STATIC_LIMIT))
#define DUAL_GYRO_GRAVITY_MAX_SQ   ((DUAL_GYRO_GRAVITY + DUAL_GYRO_ACC_STATIC_LIMIT) * \
                                    (DUAL_GYRO_GRAVITY + DUAL_GYRO_ACC_STATIC_LIMIT))
#define DUAL_GYRO_MAHONY_GRAVITY_MIN_SQ ((DUAL_GYRO_GRAVITY - DUAL_GYRO_ACC_MAHONY_LIMIT) * \
                                         (DUAL_GYRO_GRAVITY - DUAL_GYRO_ACC_MAHONY_LIMIT))
#define DUAL_GYRO_MAHONY_GRAVITY_MAX_SQ ((DUAL_GYRO_GRAVITY + DUAL_GYRO_ACC_MAHONY_LIMIT) * \
                                         (DUAL_GYRO_GRAVITY + DUAL_GYRO_ACC_MAHONY_LIMIT))
#define DUAL_GYRO_RATE_STATIC_SQ   (DUAL_GYRO_RATE_STATIC_LIMIT * DUAL_GYRO_RATE_STATIC_LIMIT)
#define DUAL_GYRO_ACC_AGREE_SQ     (DUAL_GYRO_ACC_AGREE_LIMIT * DUAL_GYRO_ACC_AGREE_LIMIT)

typedef struct {
    float x[DUAL_GYRO_STATE_SIZE];
    float p[DUAL_GYRO_STATE_SIZE][DUAL_GYRO_STATE_SIZE];
    uint8_t initialized;
} DualGyroAxisKalman_t;

Axis3f DualGyroKalman_Omega;
Axis3f DualGyroKalman_Accel;
Axis3f BMI088BoardAccel;
Axis3f BMI088BoardGyro;
Axis3f BMI270BoardAccel;
Axis3f BMI270BoardGyro;
IMU_Attitude_t VirtualIMU;

static DualGyroAxisKalman_t gDualGyroKalman[DUAL_GYRO_AXIS_COUNT];
static MahonyFilter_t gFusedMahony;
static uint8_t gRealImuAttitudeSolversEnabled;

static float vector_norm_sq(Axis3f value)
{
    return (value.x * value.x) + (value.y * value.y) + (value.z * value.z);
}

Axis3f ImuAttitude_BMI270GyroToBoard(void)
{
    Axis3f sensor;

    sensor.x = BMI270Sensor.Gyro[0];
    sensor.y = BMI270Sensor.Gyro[1];
    sensor.z = BMI270Sensor.Gyro[2];
    return bmi270_sensor_to_board(sensor);
}

Axis3f ImuAttitude_BMI270AccelToBoard(void)
{
    Axis3f sensor;

    sensor.x = BMI270Sensor.Accel[0];
    sensor.y = BMI270Sensor.Accel[1];
    sensor.z = BMI270Sensor.Accel[2];
    return bmi270_sensor_to_board(sensor);
}

Axis3f ImuAttitude_BMI088GyroToBoard(void)
{
    Axis3f gyro;

    gyro.x = BMI088Sensor.Gyro[0];
    gyro.y = BMI088Sensor.Gyro[1];
    gyro.z = BMI088Sensor.Gyro[2];

    return gyro;
}

Axis3f ImuAttitude_BMI088AccelToBoard(void)
{
    Axis3f accel;

    accel.x = BMI088Sensor.Accel[0];
    accel.y = BMI088Sensor.Accel[1];
    accel.z = BMI088Sensor.Accel[2];

    return accel;
}

static uint8_t is_stationary(Axis3f gyro1, Axis3f gyro2, Axis3f accel1, Axis3f accel2,
                             uint8_t bmi088_usable, uint8_t bmi220_usable)
{
    float accel1_norm_sq = vector_norm_sq(accel1);
    float accel2_norm_sq = vector_norm_sq(accel2);

    if ((bmi088_usable == 0U) || (bmi220_usable == 0U)) {
        return 0U;
    }

    if ((accel1_norm_sq < DUAL_GYRO_GRAVITY_MIN_SQ) || (accel1_norm_sq > DUAL_GYRO_GRAVITY_MAX_SQ) ||
        (accel2_norm_sq < DUAL_GYRO_GRAVITY_MIN_SQ) || (accel2_norm_sq > DUAL_GYRO_GRAVITY_MAX_SQ)) {
        return 0U;
    }

    if ((vector_norm_sq(gyro1) > DUAL_GYRO_RATE_STATIC_SQ) ||
        (vector_norm_sq(gyro2) > DUAL_GYRO_RATE_STATIC_SQ)) {
        return 0U;
    }

    return 1U;
}

static Axis3f blend_accel(Axis3f accel1, Axis3f accel2, uint8_t bmi088_usable, uint8_t bmi220_usable)
{
    Axis3f accel;
    float gravity_sq = DUAL_GYRO_GRAVITY * DUAL_GYRO_GRAVITY;
    float accel1_norm_sq = vector_norm_sq(accel1);
    float accel2_norm_sq = vector_norm_sq(accel2);
    float accel1_error_sq = fabsf(accel1_norm_sq - gravity_sq);
    float accel2_error_sq = fabsf(accel2_norm_sq - gravity_sq);

    if ((bmi088_usable == 0U) && (bmi220_usable != 0U)) {
        accel = accel2;
    } else if ((bmi088_usable != 0U) && (bmi220_usable == 0U)) {
        accel = accel1;
    } else if ((bmi088_usable == 0U) && (bmi220_usable == 0U)) {
        accel.x = 0.0f;
        accel.y = 0.0f;
        accel.z = DUAL_GYRO_GRAVITY;
    } else if ((accel1_norm_sq >= DUAL_GYRO_GRAVITY_MIN_SQ) && (accel1_norm_sq <= DUAL_GYRO_GRAVITY_MAX_SQ) &&
        (accel2_norm_sq >= DUAL_GYRO_GRAVITY_MIN_SQ) && (accel2_norm_sq <= DUAL_GYRO_GRAVITY_MAX_SQ)) {
        accel.x = 0.5f * (accel1.x + accel2.x);
        accel.y = 0.5f * (accel1.y + accel2.y);
        accel.z = 0.5f * (accel1.z + accel2.z);
    } else if (accel1_error_sq <= accel2_error_sq) {
        accel = accel1;
    } else {
        accel = accel2;
    }

    return accel;
}

/* 判断融合后的加速度是否仍可信为重力；高速甩动时线加速度会污染 Mahony 修正。 */
static uint8_t accel_is_valid_for_mahony(Axis3f accel)
{
    float norm_sq = vector_norm_sq(accel);

    return ((norm_sq >= DUAL_GYRO_MAHONY_GRAVITY_MIN_SQ) &&
            (norm_sq <= DUAL_GYRO_MAHONY_GRAVITY_MAX_SQ)) ? 1U : 0U;
}

/* 两颗加速度计方向一致时，才认为融合加速度还能代表真实重力方向。 */
static uint8_t accel_vectors_agree(Axis3f accel1, Axis3f accel2)
{
    Axis3f diff;

    diff.x = accel1.x - accel2.x;
    diff.y = accel1.y - accel2.y;
    diff.z = accel1.z - accel2.z;

    return (vector_norm_sq(diff) <= DUAL_GYRO_ACC_AGREE_SQ) ? 1U : 0U;
}

static void reset_virtual_attitude(IMU_Attitude_t *attitude)
{
    attitude->q[0] = 1.0f;
    attitude->q[1] = 0.0f;
    attitude->q[2] = 0.0f;
    attitude->q[3] = 0.0f;
    attitude->Roll = 0.0f;
    attitude->Pitch = 0.0f;
    attitude->Yaw = 0.0f;
}

static void update_virtual_attitude_from_filter(IMU_Attitude_t *attitude, const MahonyFilter_t *filter)
{
    attitude->q[0] = filter->q0;
    attitude->q[1] = filter->q1;
    attitude->q[2] = filter->q2;
    attitude->q[3] = filter->q3;
    /* 串口和参考例程沿用平衡车车体角约定：Pitch 取 Mahony.roll，Roll 取 Mahony.pitch。 */
    attitude->Pitch = filter->roll;
    attitude->Roll = filter->pitch;
    attitude->Yaw = filter->yaw;
}

static void kalman_axis_init(DualGyroAxisKalman_t *filter, float gyro1, float gyro2)
{
    filter->x[0] = gyro1;
    filter->x[1] = 0.0f;
    filter->x[2] = gyro2 - gyro1;

    for (uint8_t row = 0U; row < DUAL_GYRO_STATE_SIZE; row++) {
        for (uint8_t col = 0U; col < DUAL_GYRO_STATE_SIZE; col++) {
            filter->p[row][col] = 0.0f;
        }
    }

    filter->p[0][0] = 0.0500f;
    filter->p[1][1] = 0.0100f;
    filter->p[2][2] = 0.0100f;
    filter->initialized = 1U;
}

static void kalman_predict(DualGyroAxisKalman_t *filter, float dt)
{
    filter->p[0][0] += DUAL_GYRO_Q_OMEGA_PER_SEC * dt;
    filter->p[1][1] += DUAL_GYRO_Q_BIAS_PER_SEC * dt;
    filter->p[2][2] += DUAL_GYRO_Q_BIAS_PER_SEC * dt;
}

static void kalman_update_scalar(DualGyroAxisKalman_t *filter, const float h[DUAL_GYRO_STATE_SIZE],
                                 float measurement, float measurement_noise)
{
    float ph_t[DUAL_GYRO_STATE_SIZE];
    float hp[DUAL_GYRO_STATE_SIZE];
    float innovation = measurement;
    float innovation_cov = measurement_noise;
    float gain[DUAL_GYRO_STATE_SIZE];

    for (uint8_t i = 0U; i < DUAL_GYRO_STATE_SIZE; i++) {
        innovation -= h[i] * filter->x[i];
    }

    for (uint8_t row = 0U; row < DUAL_GYRO_STATE_SIZE; row++) {
        ph_t[row] = 0.0f;
        hp[row] = 0.0f;
        for (uint8_t col = 0U; col < DUAL_GYRO_STATE_SIZE; col++) {
            ph_t[row] += filter->p[row][col] * h[col];
            hp[row] += h[col] * filter->p[col][row];
        }
        innovation_cov += h[row] * ph_t[row];
    }

    if (innovation_cov <= 0.0f) {
        return;
    }

    for (uint8_t i = 0U; i < DUAL_GYRO_STATE_SIZE; i++) {
        gain[i] = ph_t[i] / innovation_cov;
        filter->x[i] += gain[i] * innovation;
    }

    for (uint8_t row = 0U; row < DUAL_GYRO_STATE_SIZE; row++) {
        for (uint8_t col = 0U; col < DUAL_GYRO_STATE_SIZE; col++) {
            filter->p[row][col] -= gain[row] * hp[col];
        }
    }

    for (uint8_t row = 0U; row < DUAL_GYRO_STATE_SIZE; row++) {
        for (uint8_t col = (uint8_t)(row + 1U); col < DUAL_GYRO_STATE_SIZE; col++) {
            float symmetric = 0.5f * (filter->p[row][col] + filter->p[col][row]);

            filter->p[row][col] = symmetric;
            filter->p[col][row] = symmetric;
        }
    }
}

static float update_axis(DualGyroAxisKalman_t *filter, float gyro1, float gyro2, float dt,
                         uint8_t stationary, uint8_t gyro1_usable, uint8_t gyro2_usable)
{
    float relative_bias;
    float corrected_gyro2;
    float difference;
    float alpha;

    if (filter->initialized == 0U) {
        filter->x[0] = gyro1;
        filter->x[1] = 0.0f;
        filter->x[2] = gyro2 - gyro1;
        filter->initialized = 1U;
    }

    relative_bias = filter->x[2];

    if (stationary != 0U) {
        alpha = DUAL_GYRO_REL_BIAS_ALPHA_PER_SEC * dt;
        if (alpha > 0.05f) {
            alpha = 0.05f;
        }
        relative_bias += alpha * ((gyro2 - gyro1) - relative_bias);
        filter->x[2] = relative_bias;
        filter->x[0] = 0.0f;
        return 0.0f;
    }

    corrected_gyro2 = gyro2 - relative_bias;
    difference = fabsf(corrected_gyro2 - gyro1);

    if ((gyro1_usable == 0U) && (gyro2_usable != 0U)) {
        filter->x[0] = corrected_gyro2;
    } else if ((gyro1_usable != 0U) && (gyro2_usable == 0U)) {
        filter->x[0] = gyro1;
    } else if ((gyro1_usable == 0U) && (gyro2_usable == 0U)) {
        filter->x[0] = 0.0f;
    } else if (difference <= DUAL_GYRO_DISAGREE_LIMIT) {
        filter->x[0] = (DUAL_GYRO_BMI088_WEIGHT * gyro1) +
                       (DUAL_GYRO_BMI220_WEIGHT * corrected_gyro2);
    } else if (difference <= DUAL_GYRO_REJECT_LIMIT) {
        filter->x[0] = ((1.0f - DUAL_GYRO_BMI220_LOW_WEIGHT) * gyro1) +
                       (DUAL_GYRO_BMI220_LOW_WEIGHT * corrected_gyro2);
    } else {
        filter->x[0] = gyro1;
    }

    return filter->x[0];
}

static void fused_attitude_init(void)
{
    for (uint8_t axis = 0U; axis < DUAL_GYRO_AXIS_COUNT; axis++) {
        gDualGyroKalman[axis].initialized = 0U;
    }

    DualGyroKalman_Omega.x = 0.0f;
    DualGyroKalman_Omega.y = 0.0f;
    DualGyroKalman_Omega.z = 0.0f;
    DualGyroKalman_Accel.x = 0.0f;
    DualGyroKalman_Accel.y = 0.0f;
    DualGyroKalman_Accel.z = DUAL_GYRO_GRAVITY;
    Mahony_Init(&gFusedMahony, 0.20f, 0.0f, 0.0010f);
    reset_virtual_attitude(&VirtualIMU);
}

static void fused_attitude_set_initial_accel(Axis3f accel)
{
    Mahony_SetFromAccel(&gFusedMahony, accel);
    fused_attitude_update_output();
}

static void fused_attitude_update(float dt)
{
    Axis3f gyro1 = ImuAttitude_BMI088GyroToBoard();
    Axis3f gyro2 = ImuAttitude_BMI270GyroToBoard();
    Axis3f accel1 = ImuAttitude_BMI088AccelToBoard();
    Axis3f accel2 = ImuAttitude_BMI270AccelToBoard();
    uint8_t bmi088_gyro_usable = (BMI088Sensor.LastGyroSaturated == 0U) ? 1U : 0U;
    uint8_t bmi220_gyro_usable = (BMI270Sensor.LastGyroSaturated == 0U) ? 1U : 0U;
    uint8_t bmi088_accel_usable = (BMI088Sensor.LastAccelSaturated == 0U) ? 1U : 0U;
    uint8_t bmi220_accel_usable = (BMI270Sensor.LastAccelSaturated == 0U) ? 1U : 0U;
    uint8_t stationary = is_stationary(gyro1, gyro2, accel1, accel2,
                                       (bmi088_gyro_usable != 0U) && (bmi088_accel_usable != 0U),
                                       (bmi220_gyro_usable != 0U) && (bmi220_accel_usable != 0U));
    uint8_t accel_correction_usable;

    DualGyroKalman_Omega.x = update_axis(&gDualGyroKalman[0], gyro1.x, gyro2.x, dt, stationary,
                                          bmi088_gyro_usable, bmi220_gyro_usable);
    DualGyroKalman_Omega.y = update_axis(&gDualGyroKalman[1], gyro1.y, gyro2.y, dt, stationary,
                                          bmi088_gyro_usable, bmi220_gyro_usable);
    DualGyroKalman_Omega.z = update_axis(&gDualGyroKalman[2], gyro1.z, gyro2.z, dt, stationary,
                                          bmi088_gyro_usable, bmi220_gyro_usable);
    DualGyroKalman_Accel = blend_accel(accel1, accel2, bmi088_accel_usable, bmi220_accel_usable);
    BMI088BoardGyro = gyro1;
    BMI270BoardGyro = gyro2;
    BMI088BoardAccel = accel1;
    BMI270BoardAccel = accel2;

    gFusedMahony.dt = dt;
    accel_correction_usable = (stationary != 0U) &&
                              (bmi088_accel_usable != 0U) &&
                              (bmi220_accel_usable != 0U) &&
                              (accel_vectors_agree(accel1, accel2) != 0U) &&
                              (accel_is_valid_for_mahony(DualGyroKalman_Accel) != 0U);

    if (accel_correction_usable != 0U) {
        Mahony_UpdateFast(&gFusedMahony, DualGyroKalman_Omega, DualGyroKalman_Accel);
    } else {
        Mahony_UpdateGyroOnlyFast(&gFusedMahony, DualGyroKalman_Omega);
    }
}

static void fused_attitude_update_output(void)
{
    Mahony_Output(&gFusedMahony);
    update_virtual_attitude_from_filter(&VirtualIMU, &gFusedMahony);
}

void ImuAttitude_Init(void)
{
#if ENABLE_REAL_IMU_ATTITUDE_SOLVERS
    if (gRealImuAttitudeSolversEnabled != 0U) {
        bmi088_attitude_init();
        bmi270_attitude_init();
    } else {
        reset_attitude(&BMI270);
        BMI088.q[0] = 1.0f;
        BMI088.q[1] = 0.0f;
        BMI088.q[2] = 0.0f;
        BMI088.q[3] = 0.0f;
        BMI088.Pitch = 0.0f;
        BMI088.Roll = 0.0f;
        BMI088.Yaw = 0.0f;
        BMI088.YawTotalAngle = 0.0f;
        BMI088.YawAngleLast = 0.0f;
        BMI088.YawRoundCount = 0.0f;
        BMI088.ins_flag = 0U;
    }
#endif
    fused_attitude_init();
}

void ImuAttitude_SetDebugMode(uint8_t enable)
{
    gRealImuAttitudeSolversEnabled = (enable != 0U) ? 1U : 0U;
}

void ImuAttitude_SetInitialAccel(Axis3f bmi088_accel, Axis3f bmi270_accel)
{
    Axis3f fused_accel;

#if ENABLE_REAL_IMU_ATTITUDE_SOLVERS
    if (gRealImuAttitudeSolversEnabled != 0U) {
        bmi088_attitude_set_initial_accel(bmi088_accel);
        bmi270_attitude_set_initial_accel(bmi270_accel);
    }
#endif

    fused_accel.x = 0.5f * (bmi088_accel.x + bmi270_accel.x);
    fused_accel.y = 0.5f * (bmi088_accel.y + bmi270_accel.y);
    fused_accel.z = 0.5f * (bmi088_accel.z + bmi270_accel.z);
    fused_attitude_set_initial_accel(fused_accel);
}

void ImuAttitude_Update(float dt)
{
    BMI088_Read(&BMI088Sensor);
    BMI270_Read(&BMI270Sensor);
#if ENABLE_REAL_IMU_ATTITUDE_SOLVERS
    if (gRealImuAttitudeSolversEnabled != 0U) {
        bmi088_attitude_update(dt);
        bmi270_attitude_update(dt);
    }
#endif
    fused_attitude_update(dt);
}

void ImuAttitude_UpdateOutput(void)
{
#if ENABLE_REAL_IMU_ATTITUDE_SOLVERS
    if (gRealImuAttitudeSolversEnabled != 0U) {
        bmi088_attitude_update_output();
        bmi270_attitude_update_output();
    }
#endif
    fused_attitude_update_output();
}
