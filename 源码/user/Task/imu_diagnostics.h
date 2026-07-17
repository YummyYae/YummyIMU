#ifndef IMU_DIAGNOSTICS_H
#define IMU_DIAGNOSTICS_H

#include "runtime_state.h"

/* 按当前 IMU 选择输出启动故障概要和可机器解析的硬件诊断明细。 */
void ImuDiagnostics_WriteFaultReport(const RuntimeState_t *state);

#endif
