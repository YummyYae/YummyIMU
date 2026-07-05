#ifndef TI_MSP_DL_CONFIG_H
#define TI_MSP_DL_CONFIG_H

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

#define POWER_STARTUP_DELAY (16)

#define GPIO_HFXT_PORT GPIOA
#define GPIO_HFXIN_PIN  DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX  (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX (IOMUX_PINCM11)

#define CPUCLK_FREQ 80000000

#define UART_OUT_INST UART2
#define UART_OUT_INST_FREQUENCY 40000000
#define UART_OUT_INST_IRQHandler UART2_IRQHandler
#define UART_OUT_INST_INT_IRQN UART2_INT_IRQn

#define GPIO_UART_OUT_RX_PORT GPIOA
#define GPIO_UART_OUT_TX_PORT GPIOA
#define GPIO_UART_OUT_RX_PIN DL_GPIO_PIN_22
#define GPIO_UART_OUT_TX_PIN DL_GPIO_PIN_21
#define GPIO_UART_OUT_IOMUX_RX (IOMUX_PINCM47)
#define GPIO_UART_OUT_IOMUX_TX (IOMUX_PINCM46)
#define GPIO_UART_OUT_IOMUX_RX_FUNC IOMUX_PINCM47_PF_UART2_RX
#define GPIO_UART_OUT_IOMUX_TX_FUNC IOMUX_PINCM46_PF_UART2_TX

#define UART_OUT_BAUD_RATE (921600U)
#define UART_OUT_IBRD_40_MHZ_921600_BAUD (2)
#define UART_OUT_FBRD_40_MHZ_921600_BAUD (46)

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_SYSCTL_CLK_init(void);
void SYSCFG_DL_UART_OUT_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif
