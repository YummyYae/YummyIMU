#include "board_mspm0.h"

#include "bmi088_mspm0.h"
#include "bmi270_mspm0.h"
#include "ti_msp_dl_config.h"

#define BOARD_SYSOSC_FREQUENCY_HZ       32000000U
#define BOARD_HFXT_STARTUP_TIMEOUT_MS   20U

extern volatile uint32_t gSystemTickMs;

static volatile uint8_t gBoardExternalClockFault;

static const DL_SYSCTL_SYSPLLConfig gBoardExternalClockPllConfig = {
    .inputFreq = DL_SYSCTL_SYSPLL_INPUT_FREQ_32_48_MHZ,
    .rDivClk2x = 3U,
    .rDivClk1 = 0U,
    .rDivClk0 = 0U,
    .enableCLK2x = DL_SYSCTL_SYSPLL_CLK2X_DISABLE,
    .enableCLK1 = DL_SYSCTL_SYSPLL_CLK1_ENABLE,
    .enableCLK0 = DL_SYSCTL_SYSPLL_CLK0_ENABLE,
    .sysPLLMCLK = DL_SYSCTL_SYSPLL_MCLK_CLK0,
    .sysPLLRef = DL_SYSCTL_SYSPLL_REF_HFCLK,
    .qDiv = 3U,
    .pDiv = DL_SYSCTL_SYSPLL_PDIV_1,
};

/* 32MHz SYSOSC x 5 / 2 = 80MHz，回退后保持所有外设时基不变。 */
static const DL_SYSCTL_SYSPLLConfig gBoardInternalClockPllConfig = {
    .inputFreq = DL_SYSCTL_SYSPLL_INPUT_FREQ_32_48_MHZ,
    .rDivClk2x = 3U,
    .rDivClk1 = 0U,
    .rDivClk0 = 0U,
    .enableCLK2x = DL_SYSCTL_SYSPLL_CLK2X_DISABLE,
    .enableCLK1 = DL_SYSCTL_SYSPLL_CLK1_ENABLE,
    .enableCLK0 = DL_SYSCTL_SYSPLL_CLK0_ENABLE,
    .sysPLLMCLK = DL_SYSCTL_SYSPLL_MCLK_CLK0,
    .sysPLLRef = DL_SYSCTL_SYSPLL_REF_SYSOSC,
    .qDiv = 4U,
    .pDiv = DL_SYSCTL_SYSPLL_PDIV_1,
};

static uint8_t board_wait_for_hfxt(void)
{
    for (uint32_t elapsed_ms = 0U;
         elapsed_ms < BOARD_HFXT_STARTUP_TIMEOUT_MS;
         elapsed_ms++) {
        if ((DL_SYSCTL_getClockStatus() & DL_SYSCTL_CLK_STATUS_HFCLK_GOOD) != 0U) {
            return 1U;
        }
        DL_Common_delayCycles(BOARD_SYSOSC_FREQUENCY_HZ / 1000U);
    }

    return ((DL_SYSCTL_getClockStatus() & DL_SYSCTL_CLK_STATUS_HFCLK_GOOD) != 0U) ? 1U : 0U;
}

/*
 * 覆盖 SysConfig 生成文件中的弱定义，避免 HFXT 缺失时卡在 DriverLib 的无限等待。
 * 外部 40MHz 晶振失败后改用内部 32MHz SYSOSC 驱动 PLL，MCLK 仍保持 80MHz。
 */
void SYSCFG_DL_SYSCTL_init(void)
{
    const DL_SYSCTL_SYSPLLConfig *pll_config;

    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_1);
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();

    gBoardExternalClockFault = 0U;
    DL_SYSCTL_setHFCLKSourceHFXTParams(
        DL_SYSCTL_HFXT_RANGE_32_48_MHZ, 20U, false);
    DL_SYSCTL_enableHFCLKStartupMonitor();

    if (board_wait_for_hfxt() != 0U) {
        pll_config = &gBoardExternalClockPllConfig;
        DL_SYSCTL_setHFCLKDividerForMFPCLK(DL_SYSCTL_HFCLK_MFPCLK_DIVIDER_10);
        DL_SYSCTL_setMFPCLKSource(DL_SYSCTL_MFPCLK_SOURCE_HFCLK);
    } else {
        gBoardExternalClockFault = 1U;
        DL_SYSCTL_disableHFCLKStartupMonitor();
        DL_SYSCTL_disableHFXT();
        pll_config = &gBoardInternalClockPllConfig;
        DL_SYSCTL_setMFPCLKSource(DL_SYSCTL_MFPCLK_SOURCE_SYSOSC);
    }

    DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *)pll_config);
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);
    DL_SYSCTL_enableMFCLK();
    DL_SYSCTL_enableMFPCLK();
    DL_SYSCTL_setMCLKSource(SYSOSC, HSCLK, DL_SYSCTL_HSCLK_SOURCE_SYSPLL);
}

/* PLL 和 MCLK 已在上述有限等待流程中确认，禁止再次检查失效的 HFCLK。 */
void SYSCFG_DL_SYSCTL_CLK_init(void)
{
}

void Board_InitHardware(void)
{
    SYSCFG_DL_init();
}

uint8_t Board_HasExternalClockFault(void)
{
    return gBoardExternalClockFault;
}

void Board_EnableInterrupts(void)
{
    NVIC_SetPriority(TIMERG6_1000hz_INST_INT_IRQN, 0U);
    NVIC_SetPriority(TIMERG8_100hz_INST_INT_IRQN, 2U);
    NVIC_SetPriority(GPIOA_INT_IRQn, 3U);

    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(TIMERG6_1000hz_INST_INT_IRQN);
}

void Board_EnterCritical(void)
{
    __disable_irq();
}

void Board_ExitCritical(void)
{
    __enable_irq();
}

void Board_DelayMs(uint32_t ms)
{
    DL_Common_delayCycles((CPUCLK_FREQ / 1000U) * ms);
}

uint32_t Board_GetSystemTickMs(void)
{
    return gSystemTickMs;
}

void Board_StartImuTimer(void)
{
    DL_TimerG_startCounter(TIMERG6_1000hz_INST);
}

uint32_t Board_GetImuTimerCount(void)
{
    return DL_TimerG_getTimerCount(TIMERG6_1000hz_INST);
}

uint32_t Board_GetImuTimerElapsedTicks(uint32_t start_count, uint32_t end_count)
{
    uint32_t period_ticks = TIMERG6_1000hz_INST_LOAD_VALUE + 1U;

    if (DL_TimerG_getCounterMode(TIMERG6_1000hz_INST) == DL_TIMER_COUNT_MODE_DOWN) {
        if (start_count >= end_count) {
            return start_count - end_count;
        }

        return start_count + period_ticks - end_count;
    }

    if (end_count >= start_count) {
        return end_count - start_count;
    }

    return end_count + period_ticks - start_count;
}

uint32_t Board_GetImuInterruptBudgetTicks(void)
{
    return (TIMERG6_1000hz_INST_LOAD_VALUE + 1U) * 8U / 10U;
}

void Board_SystemReset(void)
{
    for (uint32_t i = 0U; i < (CPUCLK_FREQ / 100U); i++) {
        __NOP();
    }

    NVIC_SystemReset();
}

void Board_ResetAfterSave(void)
{
    Board_SystemReset();
}

void Board_HandleGroup1Irq(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case DL_INTERRUPT_GROUP1_IIDX_GPIOA:
            if (DL_GPIO_getEnabledInterruptStatus(BMI088_PORT, BMI088_PIN_0_PIN) != 0U) {
                DL_GPIO_clearInterruptStatus(BMI088_PORT, BMI088_PIN_0_PIN);
            }
            break;

        case DL_INTERRUPT_GROUP1_IIDX_GPIOB:
            if (DL_GPIO_getEnabledInterruptStatus(GPIOB, BMI088B_INT1_PIN | BMI270_INT_PIN) != 0U) {
                DL_GPIO_clearInterruptStatus(GPIOB, BMI088B_INT1_PIN | BMI270_INT_PIN);
            }
            break;

        default:
            break;
    }
}

void Board_HandleNmi(void)
{
    uint32_t nmi_status = DL_SYSCTL_getRawNonMaskableInterruptStatus(
        DL_SYSCTL_NMI_FLASH_DED | DL_SYSCTL_NMI_SRAM_DED |
        DL_SYSCTL_NMI_LFCLK_FAIL | DL_SYSCTL_NMI_WWDT0_FAULT |
        DL_SYSCTL_NMI_WWDT1_FAULT | DL_SYSCTL_NMI_BORLVL);

    if ((nmi_status & (DL_SYSCTL_NMI_FLASH_DED | DL_SYSCTL_NMI_SRAM_DED)) != 0U) {
        DL_SYSCTL_clearECCErrorStatus();
    }

    if (nmi_status != 0U) {
        DL_SYSCTL_clearNonMaskableInterruptStatus(nmi_status);
    }
}

