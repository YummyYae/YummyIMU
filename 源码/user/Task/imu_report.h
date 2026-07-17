#ifndef IMU_REPORT_H
#define IMU_REPORT_H

#include "runtime_state.h"

#include <stdint.h>

/* 前台 IMU 回传子模块：处理故障心跳、惯导状态和各输出模式的数据编码。 */
void ImuReport_Service(RuntimeState_t *state,
                       uint8_t report_pending,
                       uint8_t imu_fault,
                       uint8_t imu_alarm);

#endif
