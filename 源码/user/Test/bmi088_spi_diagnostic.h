#ifndef BMI088_SPI_DIAGNOSTIC_H
#define BMI088_SPI_DIAGNOSTIC_H

/*
 * 启动 BMI088 最小 SPI 诊断固件。
 * 本函数不会返回，也不会启动 IMU 解算、温控、Flash 或命令任务。
 */
void BMI088_SpiDiagnostic_Run(void);

#endif
