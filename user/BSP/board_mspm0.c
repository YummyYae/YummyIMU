#include "board_mspm0.h"

#include "bmi088_mspm0.h"
#include "bmi270_mspm0.h"
#include "ti_msp_dl_config.h"

void Board_InitHardware(void)
{
    SYSCFG_DL_init();
}

void Board_EnableInterrupts(void)
{
    NVIC_SetPriority(TIMERG6_1000hz_INST_INT_IRQN, 0U);
    NVIC_SetPriority(TIMERG8_100hz_INST_INT_IRQN, 2U);
    NVIC_SetPriority(GPIOA_INT_IRQn, 3U);

    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(TIMERG6_1000hz_INST_INT_IRQN);
}

void Board_ResetAfterSave(void)
{
    for (uint32_t i = 0U; i < (CPUCLK_FREQ / 100U); i++) {
        __NOP();
    }

    NVIC_SystemReset();
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

