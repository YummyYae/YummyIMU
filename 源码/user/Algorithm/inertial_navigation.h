#ifndef INERTIAL_NAVIGATION_H
#define INERTIAL_NAVIGATION_H

#include "imu_attitude.h"

#include <stdint.h>

#define INERTIAL_NAVIGATION_AXIS_COUNT 2U

typedef enum {
    INERTIAL_NAVIGATION_WAIT_START = 0U,
    INERTIAL_NAVIGATION_ALIGNING = 1U,
    INERTIAL_NAVIGATION_RUNNING = 2U,
    INERTIAL_NAVIGATION_FAULT = 3U
} InertialNavigationState_t;

typedef enum {
    INERTIAL_NAVIGATION_FAULT_NONE = 0U,
    INERTIAL_NAVIGATION_FAULT_SAMPLE_INVALID = 1U,
    INERTIAL_NAVIGATION_FAULT_NUMERIC = 2U,
    INERTIAL_NAVIGATION_FAULT_ALIGNMENT = 3U
} InertialNavigationFault_t;

typedef struct {
    float position_m[INERTIAL_NAVIGATION_AXIS_COUNT];
    float velocity_mps[INERTIAL_NAVIGATION_AXIS_COUNT];
    float acceleration_mps2[INERTIAL_NAVIGATION_AXIS_COUNT];
    float accel_bias_earth_mps2[INERTIAL_NAVIGATION_AXIS_COUNT];
    uint32_t start_update_count;
    uint32_t elapsed_updates;
    uint16_t alignment_samples;
    uint8_t state;
    uint8_t stationary;
    uint8_t bias_valid;
    uint8_t fault;
} InertialNavigationSnapshot_t;

typedef struct {
    uint32_t update_count;
    uint8_t moving;
    uint8_t bias_valid;
    uint8_t heading_valid;
} InertialNavigationStartEvent_t;

void InertialNavigation_Init(void);
void InertialNavigation_RequestStart(void);
void InertialNavigation_Update(const ImuFusedSample_t *sample,
                               float dt,
                               uint32_t update_count);
void InertialNavigation_GetSnapshot(InertialNavigationSnapshot_t *out);
uint8_t InertialNavigation_HaveStartEvent(void);
uint8_t InertialNavigation_TakeStartEvent(InertialNavigationStartEvent_t *out);
uint8_t InertialNavigation_HaveFault(void);

#endif
