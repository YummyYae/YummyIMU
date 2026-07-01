/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define CPUCLK_FREQ                                                     80000000



/* Defines for PWM_Motors */
#define PWM_Motors_INST                                                    TIMA0
#define PWM_Motors_INST_IRQHandler                              TIMA0_IRQHandler
#define PWM_Motors_INST_INT_IRQN                                (TIMA0_INT_IRQn)
#define PWM_Motors_INST_CLK_FREQ                                        10000000
/* GPIO defines for channel 1 */
#define GPIO_PWM_Motors_C1_PORT                                            GPIOB
#define GPIO_PWM_Motors_C1_PIN                                    DL_GPIO_PIN_20
#define GPIO_PWM_Motors_C1_IOMUX                                 (IOMUX_PINCM48)
#define GPIO_PWM_Motors_C1_IOMUX_FUNC                IOMUX_PINCM48_PF_TIMA0_CCP1
#define GPIO_PWM_Motors_C1_IDX                               DL_TIMER_CC_1_INDEX
/* GPIO defines for channel 3 */
#define GPIO_PWM_Motors_C3_PORT                                            GPIOB
#define GPIO_PWM_Motors_C3_PIN                                    DL_GPIO_PIN_24
#define GPIO_PWM_Motors_C3_IOMUX                                 (IOMUX_PINCM52)
#define GPIO_PWM_Motors_C3_IOMUX_FUNC                IOMUX_PINCM52_PF_TIMA0_CCP3
#define GPIO_PWM_Motors_C3_IDX                               DL_TIMER_CC_3_INDEX

/* Defines for PWM_Buzzer */
#define PWM_Buzzer_INST                                                   TIMG12
#define PWM_Buzzer_INST_IRQHandler                             TIMG12_IRQHandler
#define PWM_Buzzer_INST_INT_IRQN                               (TIMG12_INT_IRQn)
#define PWM_Buzzer_INST_CLK_FREQ                                        10000000
/* GPIO defines for channel 1 */
#define GPIO_PWM_Buzzer_C1_PORT                                            GPIOA
#define GPIO_PWM_Buzzer_C1_PIN                                    DL_GPIO_PIN_25
#define GPIO_PWM_Buzzer_C1_IOMUX                                 (IOMUX_PINCM55)
#define GPIO_PWM_Buzzer_C1_IOMUX_FUNC               IOMUX_PINCM55_PF_TIMG12_CCP1
#define GPIO_PWM_Buzzer_C1_IDX                               DL_TIMER_CC_1_INDEX



/* Defines for TIMERA1_10hz */
#define TIMERA1_10hz_INST                                                (TIMA1)
#define TIMERA1_10hz_INST_IRQHandler                            TIMA1_IRQHandler
#define TIMERA1_10hz_INST_INT_IRQN                              (TIMA1_INT_IRQn)
#define TIMERA1_10hz_INST_LOAD_VALUE                                     (9999U)
/* Defines for TIMERG7_200hz */
#define TIMERG7_200hz_INST                                               (TIMG7)
#define TIMERG7_200hz_INST_IRQHandler                           TIMG7_IRQHandler
#define TIMERG7_200hz_INST_INT_IRQN                             (TIMG7_INT_IRQn)
#define TIMERG7_200hz_INST_LOAD_VALUE                                     (499U)
/* Defines for TIMERG8_100hz */
#define TIMERG8_100hz_INST                                               (TIMG8)
#define TIMERG8_100hz_INST_IRQHandler                           TIMG8_IRQHandler
#define TIMERG8_100hz_INST_INT_IRQN                             (TIMG8_INT_IRQn)
#define TIMERG8_100hz_INST_LOAD_VALUE                                     (499U)
/* Defines for TIMERG6_1000hz */
#define TIMERG6_1000hz_INST                                              (TIMG6)
#define TIMERG6_1000hz_INST_IRQHandler                          TIMG6_IRQHandler
#define TIMERG6_1000hz_INST_INT_IRQN                            (TIMG6_INT_IRQn)
#define TIMERG6_1000hz_INST_LOAD_VALUE                                   (9999U)



/* Defines for UART_OUT */
#define UART_OUT_INST                                                      UART2
#define UART_OUT_INST_FREQUENCY                                         40000000
#define UART_OUT_INST_IRQHandler                                UART2_IRQHandler
#define UART_OUT_INST_INT_IRQN                                    UART2_INT_IRQn
#define GPIO_UART_OUT_RX_PORT                                              GPIOA
#define GPIO_UART_OUT_TX_PORT                                              GPIOA
#define GPIO_UART_OUT_RX_PIN                                      DL_GPIO_PIN_22
#define GPIO_UART_OUT_TX_PIN                                      DL_GPIO_PIN_21
#define GPIO_UART_OUT_IOMUX_RX                                   (IOMUX_PINCM47)
#define GPIO_UART_OUT_IOMUX_TX                                   (IOMUX_PINCM46)
#define GPIO_UART_OUT_IOMUX_RX_FUNC                    IOMUX_PINCM47_PF_UART2_RX
#define GPIO_UART_OUT_IOMUX_TX_FUNC                    IOMUX_PINCM46_PF_UART2_TX
#define UART_OUT_BAUD_RATE                                              (921600)
#define UART_OUT_IBRD_40_MHZ_921600_BAUD                                     (2)
#define UART_OUT_FBRD_40_MHZ_921600_BAUD                                    (46)




/* Defines for SPI_Bmi270 */
#define SPI_Bmi270_INST                                                    SPI1
#define SPI_Bmi270_INST_IRQHandler                              SPI1_IRQHandler
#define SPI_Bmi270_INST_INT_IRQN                                  SPI1_INT_IRQn
#define GPIO_SPI_Bmi270_PICO_PORT                                         GPIOB
#define GPIO_SPI_Bmi270_PICO_PIN                                  DL_GPIO_PIN_8
#define GPIO_SPI_Bmi270_IOMUX_PICO                              (IOMUX_PINCM25)
#define GPIO_SPI_Bmi270_IOMUX_PICO_FUNC              IOMUX_PINCM25_PF_SPI1_PICO
#define GPIO_SPI_Bmi270_POCI_PORT                                         GPIOB
#define GPIO_SPI_Bmi270_POCI_PIN                                  DL_GPIO_PIN_7
#define GPIO_SPI_Bmi270_IOMUX_POCI                              (IOMUX_PINCM24)
#define GPIO_SPI_Bmi270_IOMUX_POCI_FUNC              IOMUX_PINCM24_PF_SPI1_POCI
/* GPIO configuration for SPI_Bmi270 */
#define GPIO_SPI_Bmi270_SCLK_PORT                                         GPIOB
#define GPIO_SPI_Bmi270_SCLK_PIN                                  DL_GPIO_PIN_9
#define GPIO_SPI_Bmi270_IOMUX_SCLK                              (IOMUX_PINCM26)
#define GPIO_SPI_Bmi270_IOMUX_SCLK_FUNC              IOMUX_PINCM26_PF_SPI1_SCLK
/* Defines for SPI_BMI088 */
#define SPI_BMI088_INST                                                    SPI0
#define SPI_BMI088_INST_IRQHandler                              SPI0_IRQHandler
#define SPI_BMI088_INST_INT_IRQN                                  SPI0_INT_IRQn
#define GPIO_SPI_BMI088_PICO_PORT                                         GPIOA
#define GPIO_SPI_BMI088_PICO_PIN                                 DL_GPIO_PIN_14
#define GPIO_SPI_BMI088_IOMUX_PICO                              (IOMUX_PINCM36)
#define GPIO_SPI_BMI088_IOMUX_PICO_FUNC              IOMUX_PINCM36_PF_SPI0_PICO
#define GPIO_SPI_BMI088_POCI_PORT                                         GPIOA
#define GPIO_SPI_BMI088_POCI_PIN                                 DL_GPIO_PIN_13
#define GPIO_SPI_BMI088_IOMUX_POCI                              (IOMUX_PINCM35)
#define GPIO_SPI_BMI088_IOMUX_POCI_FUNC              IOMUX_PINCM35_PF_SPI0_POCI
/* GPIO configuration for SPI_BMI088 */
#define GPIO_SPI_BMI088_SCLK_PORT                                         GPIOA
#define GPIO_SPI_BMI088_SCLK_PIN                                 DL_GPIO_PIN_12
#define GPIO_SPI_BMI088_IOMUX_SCLK                              (IOMUX_PINCM34)
#define GPIO_SPI_BMI088_IOMUX_SCLK_FUNC              IOMUX_PINCM34_PF_SPI0_SCLK



/* Port definition for Pin Group BMI088B */
#define BMI088B_PORT                                                     (GPIOB)

/* Defines for INT1: GPIOB.16 with pinCMx 33 on package pin 4 */
// groups represented: ["BMI270","BMI088B"]
// pins affected: ["INT","INT1"]
#define GPIO_MULTIPLE_GPIOB_INT_IRQN                            (GPIOB_INT_IRQn)
#define GPIO_MULTIPLE_GPIOB_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define BMI088B_INT1_IIDX                                   (DL_GPIO_IIDX_DIO16)
#define BMI088B_INT1_PIN                                        (DL_GPIO_PIN_16)
#define BMI088B_INT1_IOMUX                                       (IOMUX_PINCM33)
/* Port definition for Pin Group BMI088 */
#define BMI088_PORT                                                      (GPIOA)

/* Defines for CSB1: GPIOA.15 with pinCMx 37 on package pin 8 */
#define BMI088_CSB1_PIN                                         (DL_GPIO_PIN_15)
#define BMI088_CSB1_IOMUX                                        (IOMUX_PINCM37)
/* Defines for CSB2: GPIOA.16 with pinCMx 38 on package pin 9 */
#define BMI088_CSB2_PIN                                         (DL_GPIO_PIN_16)
#define BMI088_CSB2_IOMUX                                        (IOMUX_PINCM38)
/* Defines for PIN_0: GPIOA.17 with pinCMx 39 on package pin 10 */
// pins affected by this interrupt request:["PIN_0"]
#define BMI088_INT_IRQN                                         (GPIOA_INT_IRQn)
#define BMI088_INT_IIDX                         (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define BMI088_PIN_0_IIDX                                   (DL_GPIO_IIDX_DIO17)
#define BMI088_PIN_0_PIN                                        (DL_GPIO_PIN_17)
#define BMI088_PIN_0_IOMUX                                       (IOMUX_PINCM39)
/* Port definition for Pin Group BMI270 */
#define BMI270_PORT                                                      (GPIOB)

/* Defines for CS: GPIOB.6 with pinCMx 23 on package pin 58 */
#define BMI270_CS_PIN                                            (DL_GPIO_PIN_6)
#define BMI270_CS_IOMUX                                          (IOMUX_PINCM23)
/* Defines for INT: GPIOB.14 with pinCMx 31 on package pin 2 */
#define BMI270_INT_IIDX                                     (DL_GPIO_IIDX_DIO14)
#define BMI270_INT_PIN                                          (DL_GPIO_PIN_14)
#define BMI270_INT_IOMUX                                         (IOMUX_PINCM31)



/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_SYSCTL_CLK_init(void);
void SYSCFG_DL_PWM_Motors_init(void);
void SYSCFG_DL_PWM_Buzzer_init(void);
void SYSCFG_DL_TIMERA1_10hz_init(void);
void SYSCFG_DL_TIMERG7_200hz_init(void);
void SYSCFG_DL_TIMERG8_100hz_init(void);
void SYSCFG_DL_TIMERG6_1000hz_init(void);
void SYSCFG_DL_UART_OUT_init(void);
void SYSCFG_DL_SPI_Bmi270_init(void);
void SYSCFG_DL_SPI_BMI088_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
