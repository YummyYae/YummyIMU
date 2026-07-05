#include "ti_msp_dl_config.h"

static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfig = {
    .inputFreq = DL_SYSCTL_SYSPLL_INPUT_FREQ_32_48_MHZ,
    .rDivClk2x = 3,
    .rDivClk1 = 0,
    .rDivClk0 = 0,
    .enableCLK2x = DL_SYSCTL_SYSPLL_CLK2X_DISABLE,
    .enableCLK1 = DL_SYSCTL_SYSPLL_CLK1_ENABLE,
    .enableCLK0 = DL_SYSCTL_SYSPLL_CLK0_ENABLE,
    .sysPLLMCLK = DL_SYSCTL_SYSPLL_MCLK_CLK0,
    .sysPLLRef = DL_SYSCTL_SYSPLL_REF_HFCLK,
    .qDiv = 3,
    .pDiv = DL_SYSCTL_SYSPLL_PDIV_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_SYSCTL_CLK_init();
    SYSCFG_DL_UART_OUT_init();
}

SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    return true;
}

SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    return true;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_UART_Main_reset(UART_OUT_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_UART_Main_enablePower(UART_OUT_INST);

    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{
    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXIN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXOUT_IOMUX);
    DL_GPIO_initPeripheralOutputFunction(GPIO_UART_OUT_IOMUX_TX,
                                         GPIO_UART_OUT_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_UART_OUT_IOMUX_RX,
                                        GPIO_UART_OUT_IOMUX_RX_FUNC);
}

SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_1);

    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_setHFCLKSourceHFXTParams(DL_SYSCTL_HFXT_RANGE_32_48_MHZ, 20, true);
    DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *)&gSYSPLLConfig);
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);
    DL_SYSCTL_setHFCLKDividerForMFPCLK(DL_SYSCTL_HFCLK_MFPCLK_DIVIDER_10);
    DL_SYSCTL_enableMFCLK();
    DL_SYSCTL_enableMFPCLK();
    DL_SYSCTL_setMFPCLKSource(DL_SYSCTL_MFPCLK_SOURCE_HFCLK);
    DL_SYSCTL_setMCLKSource(SYSOSC, HSCLK, DL_SYSCTL_HSCLK_SOURCE_SYSPLL);
}

SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_CLK_init(void)
{
    while ((DL_SYSCTL_getClockStatus() &
               (DL_SYSCTL_CLK_STATUS_SYSPLL_GOOD |
                DL_SYSCTL_CLK_STATUS_HFCLK_GOOD |
                DL_SYSCTL_CLK_STATUS_HSCLK_GOOD)) !=
           (DL_SYSCTL_CLK_STATUS_SYSPLL_GOOD |
            DL_SYSCTL_CLK_STATUS_HFCLK_GOOD |
            DL_SYSCTL_CLK_STATUS_HSCLK_GOOD)) {
    }
}

SYSCONFIG_WEAK void SYSCFG_DL_UART_OUT_init(void)
{
    static const DL_UART_Main_ClockConfig clock_config = {
        .clockSel = DL_UART_MAIN_CLOCK_BUSCLK,
        .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1,
    };
    static const DL_UART_Main_Config uart_config = {
        .mode = DL_UART_MAIN_MODE_NORMAL,
        .direction = DL_UART_MAIN_DIRECTION_TX_RX,
        .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
        .parity = DL_UART_MAIN_PARITY_NONE,
        .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
        .stopBits = DL_UART_MAIN_STOP_BITS_ONE,
    };

    DL_UART_Main_setClockConfig(UART_OUT_INST,
                                (DL_UART_Main_ClockConfig *)&clock_config);
    DL_UART_Main_init(UART_OUT_INST, (DL_UART_Main_Config *)&uart_config);
    DL_UART_Main_setOversampling(UART_OUT_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_OUT_INST,
                                    UART_OUT_IBRD_40_MHZ_921600_BAUD,
                                    UART_OUT_FBRD_40_MHZ_921600_BAUD);
    DL_UART_Main_disableInterrupt(UART_OUT_INST, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enable(UART_OUT_INST);
}
