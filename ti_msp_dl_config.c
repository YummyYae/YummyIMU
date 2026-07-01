/*
 * Copyright (c) 2023, Texas Instruments Incorporated
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
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gPWM_MotorsBackup;
DL_TimerA_backupConfig gTIMERA1_10hzBackup;
DL_TimerG_backupConfig gTIMERG7_200hzBackup;
DL_TimerG_backupConfig gTIMERG6_1000hzBackup;
DL_SPI_backupConfig gSPI_Bmi270Backup;
DL_SPI_backupConfig gSPI_BMI088Backup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_SYSCTL_CLK_init();
    SYSCFG_DL_PWM_Motors_init();
    SYSCFG_DL_PWM_Buzzer_init();
    SYSCFG_DL_TIMERA1_10hz_init();
    SYSCFG_DL_TIMERG7_200hz_init();
    SYSCFG_DL_TIMERG8_100hz_init();
    SYSCFG_DL_TIMERG6_1000hz_init();
    SYSCFG_DL_UART_OUT_init();
    SYSCFG_DL_SPI_Bmi270_init();
    SYSCFG_DL_SPI_BMI088_init();
    SYSCFG_DL_SYSTICK_init();
    /* Ensure backup structures have no valid state */
	gPWM_MotorsBackup.backupRdy 	= false;
	gTIMERA1_10hzBackup.backupRdy 	= false;
	gTIMERG7_200hzBackup.backupRdy 	= false;
	gTIMERG6_1000hzBackup.backupRdy 	= false;

	gSPI_Bmi270Backup.backupRdy 	= false;
	gSPI_BMI088Backup.backupRdy 	= false;

}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_saveConfiguration(PWM_Motors_INST, &gPWM_MotorsBackup);
	retStatus &= DL_TimerA_saveConfiguration(TIMERA1_10hz_INST, &gTIMERA1_10hzBackup);
	retStatus &= DL_TimerG_saveConfiguration(TIMERG7_200hz_INST, &gTIMERG7_200hzBackup);
	retStatus &= DL_TimerG_saveConfiguration(TIMERG6_1000hz_INST, &gTIMERG6_1000hzBackup);
	retStatus &= DL_SPI_saveConfiguration(SPI_Bmi270_INST, &gSPI_Bmi270Backup);
	retStatus &= DL_SPI_saveConfiguration(SPI_BMI088_INST, &gSPI_BMI088Backup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_restoreConfiguration(PWM_Motors_INST, &gPWM_MotorsBackup, false);
	retStatus &= DL_TimerA_restoreConfiguration(TIMERA1_10hz_INST, &gTIMERA1_10hzBackup, false);
	retStatus &= DL_TimerG_restoreConfiguration(TIMERG7_200hz_INST, &gTIMERG7_200hzBackup, false);
	retStatus &= DL_TimerG_restoreConfiguration(TIMERG6_1000hz_INST, &gTIMERG6_1000hzBackup, false);
	retStatus &= DL_SPI_restoreConfiguration(SPI_Bmi270_INST, &gSPI_Bmi270Backup);
	retStatus &= DL_SPI_restoreConfiguration(SPI_BMI088_INST, &gSPI_BMI088Backup);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(PWM_Motors_INST);
    DL_TimerG_reset(PWM_Buzzer_INST);
    DL_TimerA_reset(TIMERA1_10hz_INST);
    DL_TimerG_reset(TIMERG7_200hz_INST);
    DL_TimerG_reset(TIMERG8_100hz_INST);
    DL_TimerG_reset(TIMERG6_1000hz_INST);
    DL_UART_Main_reset(UART_OUT_INST);
    DL_SPI_reset(SPI_Bmi270_INST);
    DL_SPI_reset(SPI_BMI088_INST);


    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(PWM_Motors_INST);
    DL_TimerG_enablePower(PWM_Buzzer_INST);
    DL_TimerA_enablePower(TIMERA1_10hz_INST);
    DL_TimerG_enablePower(TIMERG7_200hz_INST);
    DL_TimerG_enablePower(TIMERG8_100hz_INST);
    DL_TimerG_enablePower(TIMERG6_1000hz_INST);
    DL_UART_Main_enablePower(UART_OUT_INST);
    DL_SPI_enablePower(SPI_Bmi270_INST);
    DL_SPI_enablePower(SPI_BMI088_INST);

    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXIN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXOUT_IOMUX);

    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_Motors_C1_IOMUX,GPIO_PWM_Motors_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_Motors_C1_PORT, GPIO_PWM_Motors_C1_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_Motors_C3_IOMUX,GPIO_PWM_Motors_C3_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_Motors_C3_PORT, GPIO_PWM_Motors_C3_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_Buzzer_C1_IOMUX,GPIO_PWM_Buzzer_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_Buzzer_C1_PORT, GPIO_PWM_Buzzer_C1_PIN);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_OUT_IOMUX_TX, GPIO_UART_OUT_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_OUT_IOMUX_RX, GPIO_UART_OUT_IOMUX_RX_FUNC);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_SPI_Bmi270_IOMUX_SCLK, GPIO_SPI_Bmi270_IOMUX_SCLK_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_SPI_Bmi270_IOMUX_PICO, GPIO_SPI_Bmi270_IOMUX_PICO_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_SPI_Bmi270_IOMUX_POCI, GPIO_SPI_Bmi270_IOMUX_POCI_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_SPI_BMI088_IOMUX_SCLK, GPIO_SPI_BMI088_IOMUX_SCLK_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_SPI_BMI088_IOMUX_PICO, GPIO_SPI_BMI088_IOMUX_PICO_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_SPI_BMI088_IOMUX_POCI, GPIO_SPI_BMI088_IOMUX_POCI_FUNC);

    DL_GPIO_initDigitalInputFeatures(BMI088B_INT1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(BMI088_CSB1_IOMUX);

    DL_GPIO_initDigitalOutput(BMI088_CSB2_IOMUX);

    DL_GPIO_initDigitalInputFeatures(BMI088_PIN_0_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(BMI270_CS_IOMUX);

    DL_GPIO_initDigitalInputFeatures(BMI270_INT_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_setPins(BMI088_PORT, BMI088_CSB1_PIN |
		BMI088_CSB2_PIN);
    DL_GPIO_enableOutput(BMI088_PORT, BMI088_CSB1_PIN |
		BMI088_CSB2_PIN);
    DL_GPIO_setUpperPinsPolarity(BMI088_PORT, DL_GPIO_PIN_17_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(BMI088_PORT, BMI088_PIN_0_PIN);
    DL_GPIO_enableInterrupt(BMI088_PORT, BMI088_PIN_0_PIN);
    DL_GPIO_setPins(GPIOB, BMI270_CS_PIN);
    DL_GPIO_enableOutput(GPIOB, BMI270_CS_PIN);
    DL_GPIO_setLowerPinsPolarity(GPIOB, DL_GPIO_PIN_14_EDGE_RISE);
    DL_GPIO_setUpperPinsPolarity(GPIOB, DL_GPIO_PIN_16_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(GPIOB, BMI088B_INT1_PIN |
		BMI270_INT_PIN);
    DL_GPIO_enableInterrupt(GPIOB, BMI088B_INT1_PIN |
		BMI270_INT_PIN);

}


static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfig = {
    .inputFreq              = DL_SYSCTL_SYSPLL_INPUT_FREQ_32_48_MHZ,
	.rDivClk2x              = 3,
	.rDivClk1               = 0,
	.rDivClk0               = 0,
	.enableCLK2x            = DL_SYSCTL_SYSPLL_CLK2X_DISABLE,
	.enableCLK1             = DL_SYSCTL_SYSPLL_CLK1_ENABLE,
	.enableCLK0             = DL_SYSCTL_SYSPLL_CLK0_ENABLE,
	.sysPLLMCLK             = DL_SYSCTL_SYSPLL_MCLK_CLK0,
	.sysPLLRef              = DL_SYSCTL_SYSPLL_REF_HFCLK,
	.qDiv                   = 3,
	.pDiv                   = DL_SYSCTL_SYSPLL_PDIV_1
};
SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_1);

    
	DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
	/* Set default configuration */
	DL_SYSCTL_disableHFXT();
	DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_setHFCLKSourceHFXTParams(DL_SYSCTL_HFXT_RANGE_32_48_MHZ,20, true);
    DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *) &gSYSPLLConfig);
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);
    DL_SYSCTL_setHFCLKDividerForMFPCLK(DL_SYSCTL_HFCLK_MFPCLK_DIVIDER_10);
    DL_SYSCTL_enableMFCLK();
    DL_SYSCTL_enableMFPCLK();
	DL_SYSCTL_setMFPCLKSource(DL_SYSCTL_MFPCLK_SOURCE_HFCLK);
    DL_SYSCTL_setMCLKSource(SYSOSC, HSCLK, DL_SYSCTL_HSCLK_SOURCE_SYSPLL);

}
SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_CLK_init(void) {
    while ((DL_SYSCTL_getClockStatus() & (DL_SYSCTL_CLK_STATUS_SYSPLL_GOOD
		 | DL_SYSCTL_CLK_STATUS_HFCLK_GOOD
		 | DL_SYSCTL_CLK_STATUS_HSCLK_GOOD))
	       != (DL_SYSCTL_CLK_STATUS_SYSPLL_GOOD
		 | DL_SYSCTL_CLK_STATUS_HFCLK_GOOD
		 | DL_SYSCTL_CLK_STATUS_HSCLK_GOOD))
	{
		/* Ensure that clocks are in default POR configuration before initialization.
		* Additionally once LFXT is enabled, the internal LFOSC is disabled, and cannot
		* be re-enabled other than by executing a BOOTRST. */
		;
	}
}



/*
 * Timer clock configuration to be sourced by  / 8 (10000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   10000000 Hz = 10000000 Hz / (8 * (0 + 1))
 */
static const DL_TimerA_ClockConfig gPWM_MotorsClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale = 0U
};

static const DL_TimerA_PWMConfig gPWM_MotorsConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 1001,
    .isTimerWithFourCC = true,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_Motors_init(void) {

    DL_TimerA_setClockConfig(
        PWM_Motors_INST, (DL_TimerA_ClockConfig *) &gPWM_MotorsClockConfig);

    DL_TimerA_initPWMMode(
        PWM_Motors_INST, (DL_TimerA_PWMConfig *) &gPWM_MotorsConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerA_setCounterControl(PWM_Motors_INST,DL_TIMER_CZC_CCCTL1_ZCOND,DL_TIMER_CAC_CCCTL1_ACOND,DL_TIMER_CLC_CCCTL1_LCOND);

    DL_TimerA_setCaptureCompareOutCtl(PWM_Motors_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_Motors_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_Motors_INST, 1001, DL_TIMER_CC_1_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(PWM_Motors_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_3_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_Motors_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_3_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_Motors_INST, 1001, DL_TIMER_CC_3_INDEX);

    DL_TimerA_enableClock(PWM_Motors_INST);


    
    DL_TimerA_setCCPDirection(PWM_Motors_INST , DL_TIMER_CC1_OUTPUT | DL_TIMER_CC3_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 8 (10000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   10000000 Hz = 10000000 Hz / (8 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gPWM_BuzzerClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale = 0U
};

static const DL_TimerG_PWMConfig gPWM_BuzzerConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 1000,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_Buzzer_init(void) {

    DL_TimerG_setClockConfig(
        PWM_Buzzer_INST, (DL_TimerG_ClockConfig *) &gPWM_BuzzerClockConfig);

    DL_TimerG_initPWMMode(
        PWM_Buzzer_INST, (DL_TimerG_PWMConfig *) &gPWM_BuzzerConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerG_setCounterControl(PWM_Buzzer_INST,DL_TIMER_CZC_CCCTL1_ZCOND,DL_TIMER_CAC_CCCTL1_ACOND,DL_TIMER_CLC_CCCTL1_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(PWM_Buzzer_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(PWM_Buzzer_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_Buzzer_INST, 1000, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(PWM_Buzzer_INST);


    
    DL_TimerG_setCCPDirection(PWM_Buzzer_INST , DL_TIMER_CC1_OUTPUT );


}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (10000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   100000 Hz = 10000000 Hz / (8 * (99 + 1))
 */
static const DL_TimerA_ClockConfig gTIMERA1_10hzClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 99U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMERA1_10hz_INST_LOAD_VALUE = (100 ms * 100000 Hz) - 1
 */
static const DL_TimerA_TimerConfig gTIMERA1_10hzTimerConfig = {
    .period     = TIMERA1_10hz_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMERA1_10hz_init(void) {

    DL_TimerA_setClockConfig(TIMERA1_10hz_INST,
        (DL_TimerA_ClockConfig *) &gTIMERA1_10hzClockConfig);

    DL_TimerA_initTimerMode(TIMERA1_10hz_INST,
        (DL_TimerA_TimerConfig *) &gTIMERA1_10hzTimerConfig);
    DL_TimerA_enableInterrupt(TIMERA1_10hz_INST , DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_enableClock(TIMERA1_10hz_INST);





}

/*
 * Timer clock configuration to be sourced by BUSCLK /  (10000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   100000 Hz = 10000000 Hz / (8 * (99 + 1))
 */
static const DL_TimerG_ClockConfig gTIMERG7_200hzClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 99U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMERG7_200hz_INST_LOAD_VALUE = (5 ms * 100000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gTIMERG7_200hzTimerConfig = {
    .period     = TIMERG7_200hz_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMERG7_200hz_init(void) {

    DL_TimerG_setClockConfig(TIMERG7_200hz_INST,
        (DL_TimerG_ClockConfig *) &gTIMERG7_200hzClockConfig);

    DL_TimerG_initTimerMode(TIMERG7_200hz_INST,
        (DL_TimerG_TimerConfig *) &gTIMERG7_200hzTimerConfig);
    DL_TimerG_enableInterrupt(TIMERG7_200hz_INST , DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(TIMERG7_200hz_INST);





}

/*
 * Timer clock configuration to be sourced by BUSCLK /  (5000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   50000 Hz = 5000000 Hz / (8 * (99 + 1))
 */
static const DL_TimerG_ClockConfig gTIMERG8_100hzClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 99U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMERG8_100hz_INST_LOAD_VALUE = (10 ms * 50000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gTIMERG8_100hzTimerConfig = {
    .period     = TIMERG8_100hz_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMERG8_100hz_init(void) {

    DL_TimerG_setClockConfig(TIMERG8_100hz_INST,
        (DL_TimerG_ClockConfig *) &gTIMERG8_100hzClockConfig);

    DL_TimerG_initTimerMode(TIMERG8_100hz_INST,
        (DL_TimerG_TimerConfig *) &gTIMERG8_100hzTimerConfig);
    DL_TimerG_enableInterrupt(TIMERG8_100hz_INST , DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(TIMERG8_100hz_INST);





}

/*
 * Timer clock configuration to be sourced by BUSCLK /  (10000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   10000000 Hz = 10000000 Hz / (8 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gTIMERG6_1000hzClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 0U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMERG6_1000hz_INST_LOAD_VALUE = (1 ms * 10000000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gTIMERG6_1000hzTimerConfig = {
    .period     = TIMERG6_1000hz_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMERG6_1000hz_init(void) {

    DL_TimerG_setClockConfig(TIMERG6_1000hz_INST,
        (DL_TimerG_ClockConfig *) &gTIMERG6_1000hzClockConfig);

    DL_TimerG_initTimerMode(TIMERG6_1000hz_INST,
        (DL_TimerG_TimerConfig *) &gTIMERG6_1000hzTimerConfig);
    DL_TimerG_enableInterrupt(TIMERG6_1000hz_INST , DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(TIMERG6_1000hz_INST);





}



static const DL_UART_Main_ClockConfig gUART_OUTClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_OUTConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_OUT_init(void)
{
    DL_UART_Main_setClockConfig(UART_OUT_INST, (DL_UART_Main_ClockConfig *) &gUART_OUTClockConfig);

    DL_UART_Main_init(UART_OUT_INST, (DL_UART_Main_Config *) &gUART_OUTConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 921600
     *  Actual baud rate: 920863.31
     */
    DL_UART_Main_setOversampling(UART_OUT_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_OUT_INST, UART_OUT_IBRD_40_MHZ_921600_BAUD, UART_OUT_FBRD_40_MHZ_921600_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART_OUT_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);


    DL_UART_Main_enable(UART_OUT_INST);
}

static const DL_SPI_Config gSPI_Bmi270_config = {
    .mode        = DL_SPI_MODE_CONTROLLER,
    .frameFormat = DL_SPI_FRAME_FORMAT_MOTO3_POL0_PHA0,
    .parity      = DL_SPI_PARITY_NONE,
    .dataSize    = DL_SPI_DATA_SIZE_8,
    .bitOrder    = DL_SPI_BIT_ORDER_MSB_FIRST,
};

static const DL_SPI_ClockConfig gSPI_Bmi270_clockConfig = {
    .clockSel    = DL_SPI_CLOCK_BUSCLK,
    .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1
};

SYSCONFIG_WEAK void SYSCFG_DL_SPI_Bmi270_init(void) {
    DL_SPI_setClockConfig(SPI_Bmi270_INST, (DL_SPI_ClockConfig *) &gSPI_Bmi270_clockConfig);

    DL_SPI_init(SPI_Bmi270_INST, (DL_SPI_Config *) &gSPI_Bmi270_config);

    /* Configure Controller mode */
    /*
     * Set the bit rate clock divider to generate the serial output clock
     *     outputBitRate = (spiInputClock) / ((1 + SCR) * 2)
     *     13333333 = (80000000)/((1 + 2) * 2)
     */
    DL_SPI_setBitRateSerialClockDivider(SPI_Bmi270_INST, 2);
    /* Set RX and TX FIFO threshold levels */
    DL_SPI_setFIFOThreshold(SPI_Bmi270_INST, DL_SPI_RX_FIFO_LEVEL_1_2_FULL, DL_SPI_TX_FIFO_LEVEL_1_2_EMPTY);

    /* Enable module */
    DL_SPI_enable(SPI_Bmi270_INST);
}
static const DL_SPI_Config gSPI_BMI088_config = {
    .mode        = DL_SPI_MODE_CONTROLLER,
    .frameFormat = DL_SPI_FRAME_FORMAT_MOTO3_POL0_PHA0,
    .parity      = DL_SPI_PARITY_NONE,
    .dataSize    = DL_SPI_DATA_SIZE_8,
    .bitOrder    = DL_SPI_BIT_ORDER_MSB_FIRST,
};

static const DL_SPI_ClockConfig gSPI_BMI088_clockConfig = {
    .clockSel    = DL_SPI_CLOCK_BUSCLK,
    .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1
};

SYSCONFIG_WEAK void SYSCFG_DL_SPI_BMI088_init(void) {
    DL_SPI_setClockConfig(SPI_BMI088_INST, (DL_SPI_ClockConfig *) &gSPI_BMI088_clockConfig);

    DL_SPI_init(SPI_BMI088_INST, (DL_SPI_Config *) &gSPI_BMI088_config);

    /* Configure Controller mode */
    /*
     * Set the bit rate clock divider to generate the serial output clock
     *     outputBitRate = (spiInputClock) / ((1 + SCR) * 2)
     *     13333333 = (80000000)/((1 + 2) * 2)
     */
    DL_SPI_setBitRateSerialClockDivider(SPI_BMI088_INST, 2);
    /* Set RX and TX FIFO threshold levels */
    DL_SPI_setFIFOThreshold(SPI_BMI088_INST, DL_SPI_RX_FIFO_LEVEL_1_2_FULL, DL_SPI_TX_FIFO_LEVEL_1_2_EMPTY);

    /* Enable module */
    DL_SPI_enable(SPI_BMI088_INST);
}

SYSCONFIG_WEAK void SYSCFG_DL_SYSTICK_init(void)
{
    /*
     * Initializes the SysTick period to 1.00 ms,
     * enables the interrupt, and starts the SysTick Timer
     */
    DL_SYSTICK_config(80000);
}

