#include "board_mspm0.h"
#include "runtime_state.h"
#include "task_imu.h"

#include "ti_msp_dl_config.h"

void SysTick_Handler(void)
{
    RuntimeState_OnSysTick();
}

void GROUP1_IRQHandler(void)
{
    Board_HandleGroup1Irq();
}

void UART2_IRQHandler(void)
{
    uint8_t guard = 32U;

    while ((DL_UART_Main_getPendingInterrupt(UART_OUT_INST) != DL_UART_MAIN_IIDX_NO_INTERRUPT) &&
           (guard > 0U)) {
        while (!DL_UART_Main_isRXFIFOEmpty(UART_OUT_INST)) {
            (void) DL_UART_Main_receiveData(UART_OUT_INST);
        }
        DL_UART_Main_clearInterruptStatus(UART_OUT_INST, 0xFFFFFFFFUL);
        guard--;
    }

    if (guard == 0U) {
        while (!DL_UART_Main_isRXFIFOEmpty(UART_OUT_INST)) {
            (void) DL_UART_Main_receiveData(UART_OUT_INST);
        }
        DL_UART_Main_clearInterruptStatus(UART_OUT_INST, 0xFFFFFFFFUL);
    }
}

void TIMG8_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMERG8_100hz_INST) == DL_TIMER_IIDX_ZERO) {
        DL_TimerG_clearInterruptStatus(TIMERG8_100hz_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
        RuntimeState_OnTemperatureTick();
    }
}

void TIMG6_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMERG6_1000hz_INST) == DL_TIMER_IIDX_ZERO) {
        DL_TimerG_clearInterruptStatus(TIMERG6_1000hz_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
        TaskIMU_UpdateFromInterrupt(&gRuntimeState);
    }
}

void NMI_Handler(void)
{
    Board_HandleNmi();
}

