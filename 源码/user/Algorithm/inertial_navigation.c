#include "inertial_navigation.h"

#include <stddef.h>
#include <stdint.h>

#define NAV_ALIGNMENT_SAMPLES              1000U
#define NAV_START_ALIGNMENT_ENABLED        0U
#define NAV_STATIONARY_CONFIRM_SAMPLES     200U
#define NAV_INVALID_FAULT_SAMPLES          20U
#define NAV_ACCEL_FILTER_ALPHA             0.1181f
#define NAV_BIAS_ADAPT_ALPHA               0.0002f
#define NAV_STATIONARY_ACCEL_LIMIT_MPS2    0.35f
#define NAV_QUATERNION_NORM_MIN_SQ         0.80f
#define NAV_QUATERNION_NORM_MAX_SQ         1.20f
#define NAV_HEADING_NORM_MIN_SQ            0.01f
#define NAV_MAX_ACCEL_MPS2                 200.0f
#define NAV_MAX_ACCEL_BIAS_MPS2            3.0f
#define NAV_MAX_GYRO_RADPS                 100.0f
#define NAV_MAX_VELOCITY_MPS               1000.0f
#define NAV_MAX_POSITION_M                 1000000.0f

/*
 * 二维惯性导航流程：
 * 1. 姿态层每 1 ms 提供融合加速度、融合角速度和四元数；六面标定不是启动条件。
 * 2. 收到 INS START 后在下一帧直接记录航向原点，并清零二维位置和速度。
 * 3. 若启动帧静止且残差合理，则用该帧水平加速度做粗略零偏；否则按零偏为零直接运行。
 * 4. 运行时转入局部 X/Y 坐标并经过 20 Hz 一阶低通，然后进行梯形积分。
 * 5. 连续静止 200 ms 后执行零速更新，并缓慢跟踪温漂产生的水平残差；本模块不计算 Z 轴位置。
 * 6. 原 1000 ms 静止对齐流程由 NAV_START_ALIGNMENT_ENABLED 保留，后续需要时可以重新启用。
 */
typedef struct {
    float position_m[INERTIAL_NAVIGATION_AXIS_COUNT];
    float velocity_mps[INERTIAL_NAVIGATION_AXIS_COUNT];
    float acceleration_mps2[INERTIAL_NAVIGATION_AXIS_COUNT];
    float previous_accel_mps2[INERTIAL_NAVIGATION_AXIS_COUNT];
    float accel_bias_earth_mps2[INERTIAL_NAVIGATION_AXIS_COUNT];
    float alignment_sum_earth_mps2[INERTIAL_NAVIGATION_AXIS_COUNT];
    float origin_heading_cos;
    float origin_heading_sin;
    uint32_t start_update_count;
    uint32_t elapsed_updates;
    uint16_t alignment_samples;
    uint16_t stationary_samples;
    uint16_t invalid_samples;
    volatile uint8_t state;
    uint8_t stationary;
    uint8_t bias_valid;
    uint8_t fault;
    volatile uint8_t start_requested;
    volatile uint8_t start_event_pending;
    InertialNavigationStartEvent_t start_event;
} InertialNavigationContext_t;

static InertialNavigationContext_t gNavigation;

static uint8_t nav_float_is_valid(float value, float absolute_limit)
{
    return ((value == value) &&
            (value <= absolute_limit) &&
            (value >= -absolute_limit)) ? 1U : 0U;
}

static float nav_fast_inv_sqrt(float value)
{
    union {
        float f;
        uint32_t u;
    } convert;

    if (value <= 0.0f) {
        return 0.0f;
    }

    convert.f = value;
    convert.u = 0x5f3759dfU - (convert.u >> 1U);
    convert.f = convert.f * (1.5f - (0.5f * value * convert.f * convert.f));
    convert.f = convert.f * (1.5f - (0.5f * value * convert.f * convert.f));
    return convert.f;
}

static uint8_t nav_sample_is_valid(const ImuFusedSample_t *sample)
{
    float q_norm_sq;

    if ((sample == NULL) || (sample->valid == 0U)) {
        return 0U;
    }

    for (uint8_t i = 0U; i < 4U; i++) {
        if (nav_float_is_valid(sample->q[i], 2.0f) == 0U) {
            return 0U;
        }
    }

    if ((nav_float_is_valid(sample->accel.x, NAV_MAX_ACCEL_MPS2) == 0U) ||
        (nav_float_is_valid(sample->accel.y, NAV_MAX_ACCEL_MPS2) == 0U) ||
        (nav_float_is_valid(sample->accel.z, NAV_MAX_ACCEL_MPS2) == 0U) ||
        (nav_float_is_valid(sample->gyro.x, NAV_MAX_GYRO_RADPS) == 0U) ||
        (nav_float_is_valid(sample->gyro.y, NAV_MAX_GYRO_RADPS) == 0U) ||
        (nav_float_is_valid(sample->gyro.z, NAV_MAX_GYRO_RADPS) == 0U)) {
        return 0U;
    }

    q_norm_sq = (sample->q[0] * sample->q[0]) +
                (sample->q[1] * sample->q[1]) +
                (sample->q[2] * sample->q[2]) +
                (sample->q[3] * sample->q[3]);
    return ((q_norm_sq >= NAV_QUATERNION_NORM_MIN_SQ) &&
            (q_norm_sq <= NAV_QUATERNION_NORM_MAX_SQ)) ? 1U : 0U;
}

/* 只计算地理坐标系水平分量，避免为未使用的 Z 轴位置支付额外运算。 */
static void nav_body_accel_to_earth_xy(const ImuFusedSample_t *sample,
                                       float *earth_x,
                                       float *earth_y)
{
    float q0 = sample->q[0];
    float q1 = sample->q[1];
    float q2 = sample->q[2];
    float q3 = sample->q[3];
    float ax = sample->accel.x;
    float ay = sample->accel.y;
    float az = sample->accel.z;

    *earth_x = ((1.0f - (2.0f * ((q2 * q2) + (q3 * q3)))) * ax) +
               ((2.0f * ((q1 * q2) - (q0 * q3))) * ay) +
               ((2.0f * ((q1 * q3) + (q0 * q2))) * az);
    *earth_y = ((2.0f * ((q1 * q2) + (q0 * q3))) * ax) +
               ((1.0f - (2.0f * ((q1 * q1) + (q3 * q3)))) * ay) +
               ((2.0f * ((q2 * q3) - (q0 * q1))) * az);
}

static void nav_earth_to_local_xy(float earth_x, float earth_y,
                                  float *local_x, float *local_y)
{
    *local_x = (gNavigation.origin_heading_cos * earth_x) +
               (gNavigation.origin_heading_sin * earth_y);
    *local_y = (-gNavigation.origin_heading_sin * earth_x) +
               (gNavigation.origin_heading_cos * earth_y);
}

static uint8_t nav_capture_origin_heading(const ImuFusedSample_t *sample)
{
    float heading_x = 1.0f - (2.0f * ((sample->q[2] * sample->q[2]) +
                                      (sample->q[3] * sample->q[3])));
    float heading_y = 2.0f * ((sample->q[1] * sample->q[2]) +
                              (sample->q[0] * sample->q[3]));
    float norm_sq = (heading_x * heading_x) + (heading_y * heading_y);
    float inv_norm;

    if (norm_sq < NAV_HEADING_NORM_MIN_SQ) {
        gNavigation.origin_heading_cos = 1.0f;
        gNavigation.origin_heading_sin = 0.0f;
        return 0U;
    }

    inv_norm = nav_fast_inv_sqrt(norm_sq);
    gNavigation.origin_heading_cos = heading_x * inv_norm;
    gNavigation.origin_heading_sin = heading_y * inv_norm;
    return 1U;
}

static void nav_zero_motion_state(void)
{
    for (uint8_t axis = 0U; axis < INERTIAL_NAVIGATION_AXIS_COUNT; axis++) {
        gNavigation.position_m[axis] = 0.0f;
        gNavigation.velocity_mps[axis] = 0.0f;
        gNavigation.acceleration_mps2[axis] = 0.0f;
        gNavigation.previous_accel_mps2[axis] = 0.0f;
    }
}

static void nav_set_fault(uint8_t fault)
{
    gNavigation.state = INERTIAL_NAVIGATION_FAULT;
    gNavigation.fault = fault;
    gNavigation.stationary = 0U;
    gNavigation.acceleration_mps2[0] = 0.0f;
    gNavigation.acceleration_mps2[1] = 0.0f;
    gNavigation.previous_accel_mps2[0] = 0.0f;
    gNavigation.previous_accel_mps2[1] = 0.0f;
}

#if NAV_START_ALIGNMENT_ENABLED
/* 旧版精细启动：连续静止 1000 ms，估计当前温度下的水平加速度残差。 */
static void nav_begin_alignment(void)
{
    nav_zero_motion_state();
    gNavigation.accel_bias_earth_mps2[0] = 0.0f;
    gNavigation.accel_bias_earth_mps2[1] = 0.0f;
    gNavigation.alignment_sum_earth_mps2[0] = 0.0f;
    gNavigation.alignment_sum_earth_mps2[1] = 0.0f;
    gNavigation.alignment_samples = 0U;
    gNavigation.stationary_samples = 0U;
    gNavigation.invalid_samples = 0U;
    gNavigation.elapsed_updates = 0U;
    gNavigation.stationary = 0U;
    gNavigation.bias_valid = 0U;
    gNavigation.fault = INERTIAL_NAVIGATION_FAULT_NONE;
    gNavigation.start_event_pending = 0U;
    gNavigation.state = INERTIAL_NAVIGATION_ALIGNING;
}

static void nav_finish_alignment(const ImuFusedSample_t *sample,
                                 uint32_t update_count)
{
    uint8_t heading_valid;

    gNavigation.accel_bias_earth_mps2[0] =
        gNavigation.alignment_sum_earth_mps2[0] / (float)NAV_ALIGNMENT_SAMPLES;
    gNavigation.accel_bias_earth_mps2[1] =
        gNavigation.alignment_sum_earth_mps2[1] / (float)NAV_ALIGNMENT_SAMPLES;
    if ((nav_float_is_valid(gNavigation.accel_bias_earth_mps2[0],
                            NAV_MAX_ACCEL_BIAS_MPS2) == 0U) ||
        (nav_float_is_valid(gNavigation.accel_bias_earth_mps2[1],
                            NAV_MAX_ACCEL_BIAS_MPS2) == 0U)) {
        nav_set_fault(INERTIAL_NAVIGATION_FAULT_ALIGNMENT);
        return;
    }

    heading_valid = nav_capture_origin_heading(sample);
    nav_zero_motion_state();
    gNavigation.start_update_count = update_count;
    gNavigation.elapsed_updates = 0U;
    gNavigation.stationary_samples = NAV_STATIONARY_CONFIRM_SAMPLES;
    gNavigation.stationary = 1U;
    gNavigation.bias_valid = 1U;
    gNavigation.state = INERTIAL_NAVIGATION_RUNNING;

    gNavigation.start_event.update_count = update_count;
    gNavigation.start_event.moving = 0U;
    gNavigation.start_event.bias_valid = 1U;
    gNavigation.start_event.heading_valid = heading_valid;
    gNavigation.start_event_pending = 1U;
}

static void nav_update_alignment(const ImuFusedSample_t *sample,
                                 float earth_x,
                                 float earth_y,
                                 uint32_t update_count)
{
    if (sample->stationary == 0U) {
        gNavigation.alignment_sum_earth_mps2[0] = 0.0f;
        gNavigation.alignment_sum_earth_mps2[1] = 0.0f;
        gNavigation.alignment_samples = 0U;
        gNavigation.stationary = 0U;
        return;
    }

    gNavigation.stationary = 1U;
    gNavigation.alignment_sum_earth_mps2[0] += earth_x;
    gNavigation.alignment_sum_earth_mps2[1] += earth_y;
    if (gNavigation.alignment_samples < NAV_ALIGNMENT_SAMPLES) {
        gNavigation.alignment_samples++;
    }
    if (gNavigation.alignment_samples >= NAV_ALIGNMENT_SAMPLES) {
        nav_finish_alignment(sample, update_count);
    }
}
#endif

/* 简化启动：一帧内建立原点；静止时仅使用当前帧做粗略水平残差估计。 */
static void nav_start_immediately(const ImuFusedSample_t *sample,
                                  float earth_x,
                                  float earth_y,
                                  uint32_t update_count)
{
    uint8_t heading_valid = nav_capture_origin_heading(sample);
    uint8_t bias_valid = 0U;

    nav_zero_motion_state();
    gNavigation.accel_bias_earth_mps2[0] = 0.0f;
    gNavigation.accel_bias_earth_mps2[1] = 0.0f;
    gNavigation.alignment_sum_earth_mps2[0] = 0.0f;
    gNavigation.alignment_sum_earth_mps2[1] = 0.0f;
    gNavigation.alignment_samples = 0U;
    gNavigation.stationary_samples = 0U;
    gNavigation.invalid_samples = 0U;
    gNavigation.start_update_count = update_count;
    gNavigation.elapsed_updates = 0U;
    gNavigation.stationary = 0U;
    gNavigation.fault = INERTIAL_NAVIGATION_FAULT_NONE;
    gNavigation.start_event_pending = 0U;

    if ((sample->stationary != 0U) &&
        (nav_float_is_valid(earth_x, NAV_MAX_ACCEL_BIAS_MPS2) != 0U) &&
        (nav_float_is_valid(earth_y, NAV_MAX_ACCEL_BIAS_MPS2) != 0U)) {
        gNavigation.accel_bias_earth_mps2[0] = earth_x;
        gNavigation.accel_bias_earth_mps2[1] = earth_y;
        gNavigation.stationary_samples = NAV_STATIONARY_CONFIRM_SAMPLES;
        gNavigation.stationary = 1U;
        bias_valid = 1U;
    }

    gNavigation.bias_valid = bias_valid;
    gNavigation.state = INERTIAL_NAVIGATION_RUNNING;
    gNavigation.start_event.update_count = update_count;
    gNavigation.start_event.moving = (sample->stationary == 0U) ? 1U : 0U;
    gNavigation.start_event.bias_valid = bias_valid;
    gNavigation.start_event.heading_valid = heading_valid;
    gNavigation.start_event_pending = 1U;
}

void InertialNavigation_Init(void)
{
    for (uint8_t axis = 0U; axis < INERTIAL_NAVIGATION_AXIS_COUNT; axis++) {
        gNavigation.position_m[axis] = 0.0f;
        gNavigation.velocity_mps[axis] = 0.0f;
        gNavigation.acceleration_mps2[axis] = 0.0f;
        gNavigation.previous_accel_mps2[axis] = 0.0f;
        gNavigation.accel_bias_earth_mps2[axis] = 0.0f;
        gNavigation.alignment_sum_earth_mps2[axis] = 0.0f;
    }

    gNavigation.origin_heading_cos = 1.0f;
    gNavigation.origin_heading_sin = 0.0f;
    gNavigation.start_update_count = 0U;
    gNavigation.elapsed_updates = 0U;
    gNavigation.alignment_samples = 0U;
    gNavigation.stationary_samples = 0U;
    gNavigation.invalid_samples = 0U;
    gNavigation.state = INERTIAL_NAVIGATION_WAIT_START;
    gNavigation.stationary = 0U;
    gNavigation.bias_valid = 0U;
    gNavigation.fault = INERTIAL_NAVIGATION_FAULT_NONE;
    gNavigation.start_requested = 0U;
    gNavigation.start_event_pending = 0U;
    gNavigation.start_event.update_count = 0U;
    gNavigation.start_event.moving = 0U;
    gNavigation.start_event.bias_valid = 0U;
    gNavigation.start_event.heading_valid = 0U;
}

void InertialNavigation_RequestStart(void)
{
    gNavigation.start_requested = 1U;
}

void InertialNavigation_Update(const ImuFusedSample_t *sample,
                               float dt,
                               uint32_t update_count)
{
    float earth_x;
    float earth_y;
    float corrected_earth_x;
    float corrected_earth_y;
    float local_x;
    float local_y;
    float horizontal_sq;
    float previous_velocity;
    float new_velocity;
    uint8_t stationary_candidate;

    if ((dt <= 0.0f) || (dt > 0.1f) || (nav_sample_is_valid(sample) == 0U)) {
        if (gNavigation.invalid_samples < NAV_INVALID_FAULT_SAMPLES) {
            gNavigation.invalid_samples++;
        }
        if (((gNavigation.state == INERTIAL_NAVIGATION_RUNNING) ||
             (gNavigation.state == INERTIAL_NAVIGATION_ALIGNING)) &&
            (gNavigation.invalid_samples >= NAV_INVALID_FAULT_SAMPLES)) {
            nav_set_fault(INERTIAL_NAVIGATION_FAULT_SAMPLE_INVALID);
        }
        return;
    }

    gNavigation.invalid_samples = 0U;
    nav_body_accel_to_earth_xy(sample, &earth_x, &earth_y);

#if NAV_START_ALIGNMENT_ENABLED
    if (gNavigation.start_requested != 0U) {
        gNavigation.start_requested = 0U;
        nav_begin_alignment();
    }

    if (gNavigation.state == INERTIAL_NAVIGATION_ALIGNING) {
        nav_update_alignment(sample, earth_x, earth_y, update_count);
        return;
    }
#else
    if (gNavigation.start_requested != 0U) {
        gNavigation.start_requested = 0U;
        nav_start_immediately(sample, earth_x, earth_y, update_count);
        return;
    }
#endif
    if (gNavigation.state != INERTIAL_NAVIGATION_RUNNING) {
        return;
    }

    corrected_earth_x = earth_x - gNavigation.accel_bias_earth_mps2[0];
    corrected_earth_y = earth_y - gNavigation.accel_bias_earth_mps2[1];
    horizontal_sq = (corrected_earth_x * corrected_earth_x) +
                    (corrected_earth_y * corrected_earth_y);
    stationary_candidate = ((sample->stationary != 0U) &&
                            (horizontal_sq <=
                             (NAV_STATIONARY_ACCEL_LIMIT_MPS2 *
                              NAV_STATIONARY_ACCEL_LIMIT_MPS2))) ? 1U : 0U;

    if (stationary_candidate != 0U) {
        if (gNavigation.stationary_samples < NAV_STATIONARY_CONFIRM_SAMPLES) {
            gNavigation.stationary_samples++;
        }
    } else {
        gNavigation.stationary_samples = 0U;
    }
    gNavigation.stationary =
        (gNavigation.stationary_samples >= NAV_STATIONARY_CONFIRM_SAMPLES) ? 1U : 0U;

    if (gNavigation.stationary != 0U) {
        gNavigation.accel_bias_earth_mps2[0] +=
            NAV_BIAS_ADAPT_ALPHA * (earth_x - gNavigation.accel_bias_earth_mps2[0]);
        gNavigation.accel_bias_earth_mps2[1] +=
            NAV_BIAS_ADAPT_ALPHA * (earth_y - gNavigation.accel_bias_earth_mps2[1]);
        gNavigation.velocity_mps[0] = 0.0f;
        gNavigation.velocity_mps[1] = 0.0f;
        gNavigation.acceleration_mps2[0] = 0.0f;
        gNavigation.acceleration_mps2[1] = 0.0f;
        gNavigation.previous_accel_mps2[0] = 0.0f;
        gNavigation.previous_accel_mps2[1] = 0.0f;
    } else {
        nav_earth_to_local_xy(corrected_earth_x, corrected_earth_y, &local_x, &local_y);
        gNavigation.acceleration_mps2[0] +=
            NAV_ACCEL_FILTER_ALPHA * (local_x - gNavigation.acceleration_mps2[0]);
        gNavigation.acceleration_mps2[1] +=
            NAV_ACCEL_FILTER_ALPHA * (local_y - gNavigation.acceleration_mps2[1]);

        for (uint8_t axis = 0U; axis < INERTIAL_NAVIGATION_AXIS_COUNT; axis++) {
            previous_velocity = gNavigation.velocity_mps[axis];
            new_velocity = previous_velocity +
                           (0.5f * (gNavigation.previous_accel_mps2[axis] +
                                    gNavigation.acceleration_mps2[axis]) * dt);
            gNavigation.position_m[axis] += 0.5f * (previous_velocity + new_velocity) * dt;
            gNavigation.velocity_mps[axis] = new_velocity;
            gNavigation.previous_accel_mps2[axis] = gNavigation.acceleration_mps2[axis];
        }
    }

    gNavigation.elapsed_updates = update_count - gNavigation.start_update_count;
    if ((nav_float_is_valid(gNavigation.velocity_mps[0], NAV_MAX_VELOCITY_MPS) == 0U) ||
        (nav_float_is_valid(gNavigation.velocity_mps[1], NAV_MAX_VELOCITY_MPS) == 0U) ||
        (nav_float_is_valid(gNavigation.position_m[0], NAV_MAX_POSITION_M) == 0U) ||
        (nav_float_is_valid(gNavigation.position_m[1], NAV_MAX_POSITION_M) == 0U) ||
        (nav_float_is_valid(gNavigation.acceleration_mps2[0], NAV_MAX_ACCEL_MPS2) == 0U) ||
        (nav_float_is_valid(gNavigation.acceleration_mps2[1], NAV_MAX_ACCEL_MPS2) == 0U) ||
        (nav_float_is_valid(gNavigation.accel_bias_earth_mps2[0],
                            NAV_MAX_ACCEL_BIAS_MPS2) == 0U) ||
        (nav_float_is_valid(gNavigation.accel_bias_earth_mps2[1],
                            NAV_MAX_ACCEL_BIAS_MPS2) == 0U)) {
        nav_set_fault(INERTIAL_NAVIGATION_FAULT_NUMERIC);
    }
}

void InertialNavigation_GetSnapshot(InertialNavigationSnapshot_t *out)
{
    if (out == NULL) {
        return;
    }

    for (uint8_t axis = 0U; axis < INERTIAL_NAVIGATION_AXIS_COUNT; axis++) {
        out->position_m[axis] = gNavigation.position_m[axis];
        out->velocity_mps[axis] = gNavigation.velocity_mps[axis];
        out->acceleration_mps2[axis] = gNavigation.acceleration_mps2[axis];
        out->accel_bias_earth_mps2[axis] = gNavigation.accel_bias_earth_mps2[axis];
    }
    out->start_update_count = gNavigation.start_update_count;
    out->elapsed_updates = gNavigation.elapsed_updates;
    out->alignment_samples = gNavigation.alignment_samples;
    out->state = gNavigation.state;
    out->stationary = gNavigation.stationary;
    out->bias_valid = gNavigation.bias_valid;
    out->fault = gNavigation.fault;
}

uint8_t InertialNavigation_HaveStartEvent(void)
{
    return gNavigation.start_event_pending;
}

uint8_t InertialNavigation_TakeStartEvent(InertialNavigationStartEvent_t *out)
{
    if ((out == NULL) || (gNavigation.start_event_pending == 0U)) {
        return 0U;
    }

    *out = gNavigation.start_event;
    gNavigation.start_event_pending = 0U;
    return 1U;
}

uint8_t InertialNavigation_HaveFault(void)
{
    return (gNavigation.state == INERTIAL_NAVIGATION_FAULT) ? 1U : 0U;
}
