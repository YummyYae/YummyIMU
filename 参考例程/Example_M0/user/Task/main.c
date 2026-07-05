#include "ti_msp_dl_config.h"

void BSP_UART_Init(uint32_t baud_rate);
void BSP_UART_Poll(void);

int main(void)
{
    SYSCFG_DL_init();
    BSP_UART_Init(UART_OUT_BAUD_RATE);

    while (1) {
        BSP_UART_Poll();
    }
}
