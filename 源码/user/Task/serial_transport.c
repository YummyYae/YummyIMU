#include "serial_transport.h"
#include "serial_transport_internal.h"

#include "ti_msp_dl_config.h"

#include <stddef.h>
#include <string.h>

/*
 * 串口任务流程：
 * 1. RX DMA 持续把 UART 字节搬入环形缓冲区，前台调用 TaskSerial_CollectRx() 消费新字节。
 * 2. 只有收到 CR/LF 才会形成完整命令，并送入内部命令队列。
 * 3. 完整命令只通过内部队列交给命令模块，具体协议不进入本文件。
 * 4. TX DMA 负责文本和二进制数据发送；上层任务只看到稳定的传输接口。
 */
#define UART_DMA_RX_CHANNEL      0U
#define UART_DMA_TX_CHANNEL      1U
#define UART_DMA_RX_BUFFER_SIZE  256U
#define UART_DMA_TX_CHUNK_SIZE   128U
#define UART_RX_DRAIN_SIZE       16U
#define UART_DMA_TX_TIMEOUT_MS   50U
#define UART_BUSY_TIMEOUT_CYCLES (CPUCLK_FREQ / 200U)
#define UART_RESET_DELAY_CYCLES  (CPUCLK_FREQ / 100000U)

static volatile uint8_t gCommandOverflow;
static volatile uint8_t gRxIndex;
static volatile uint8_t gCommandQueueHead;
static volatile uint8_t gCommandQueueTail;
static volatile uint8_t gCommandQueueCount;
static char gRxLine[UART_RX_LINE_SIZE];
static char gCommandQueue[UART_RX_QUEUE_DEPTH][UART_RX_LINE_SIZE];
static uint8_t gUartDmaRxBuffer[UART_DMA_RX_BUFFER_SIZE];
static uint8_t gUartDmaTxBuffer[UART_DMA_TX_CHUNK_SIZE];
static uint8_t gUartRxDrainBuffer[UART_RX_DRAIN_SIZE];
static uint16_t gUartDmaRxReadIndex;

static void serial_uart_apply_config(uint32_t baud_rate);
static uint8_t serial_uart_wait_idle(uint32_t timeout_cycles);
static uint8_t serial_dma_wait_tx_idle(uint32_t timeout_ms);
static void serial_dma_start_rx(void);
static void serial_collect_byte(uint8_t rx);
static void serial_reset_rx_parser(void);

static void serial_uart_apply_config(uint32_t baud_rate)
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

    DL_UART_Main_setClockConfig(UART_OUT_INST, (DL_UART_Main_ClockConfig *)&clock_config);
    DL_UART_Main_init(UART_OUT_INST, (DL_UART_Main_Config *)&uart_config);
    DL_UART_Main_configBaudRate(UART_OUT_INST, UART_OUT_INST_FREQUENCY, baud_rate);
    DL_UART_Main_enableFIFOs(UART_OUT_INST);
    DL_UART_Main_setRXFIFOThreshold(UART_OUT_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setTXFIFOThreshold(UART_OUT_INST, DL_UART_TX_FIFO_LEVEL_EMPTY);
    DL_UART_Main_disableInterrupt(UART_OUT_INST, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enable(UART_OUT_INST);
}

static uint8_t serial_uart_wait_idle(uint32_t timeout_cycles)
{
    while (DL_UART_Main_isBusy(UART_OUT_INST)) {
        if (timeout_cycles == 0U) {
            return 0U;
        }
        timeout_cycles--;
    }

    return 1U;
}

static uint8_t serial_dma_wait_tx_idle(uint32_t timeout_ms)
{
    uint32_t timeout_cycles = (CPUCLK_FREQ / 1000U) * timeout_ms;

    while (DL_DMA_isChannelEnabled(DMA, UART_DMA_TX_CHANNEL)) {
        if (timeout_cycles == 0U) {
            DL_DMA_disableChannel(DMA, UART_DMA_TX_CHANNEL);
            return 0U;
        }
        timeout_cycles--;
    }

    return 1U;
}

/* 配置 UART RX/TX DMA；RX 持续写入环形缓冲，前台只解析已经搬运好的字节。 */
static void serial_dma_init(void)
{
    DL_DMA_Config rx_config = {
        .trigger = DMA_UART2_RX_TRIG,
        .triggerType = DL_DMA_TRIGGER_TYPE_EXTERNAL,
        .transferMode = DL_DMA_FULL_CH_REPEAT_SINGLE_TRANSFER_MODE,
        .extendedMode = DL_DMA_NORMAL_MODE,
        .srcWidth = DL_DMA_WIDTH_BYTE,
        .destWidth = DL_DMA_WIDTH_BYTE,
        .srcIncrement = DL_DMA_ADDR_UNCHANGED,
        .destIncrement = DL_DMA_ADDR_INCREMENT,
    };
    DL_DMA_Config tx_config = {
        .trigger = DMA_UART2_TX_TRIG,
        .triggerType = DL_DMA_TRIGGER_TYPE_EXTERNAL,
        .transferMode = DL_DMA_SINGLE_TRANSFER_MODE,
        .extendedMode = DL_DMA_NORMAL_MODE,
        .srcWidth = DL_DMA_WIDTH_BYTE,
        .destWidth = DL_DMA_WIDTH_BYTE,
        .srcIncrement = DL_DMA_ADDR_INCREMENT,
        .destIncrement = DL_DMA_ADDR_UNCHANGED,
    };

    DL_DMA_disableChannel(DMA, UART_DMA_RX_CHANNEL);
    DL_DMA_disableChannel(DMA, UART_DMA_TX_CHANNEL);
    DL_DMA_initChannel(DMA, UART_DMA_RX_CHANNEL, &rx_config);
    DL_DMA_initChannel(DMA, UART_DMA_TX_CHANNEL, &tx_config);
    DL_DMA_enableRoundRobinPriority(DMA);
    DL_DMA_setBurstSize(DMA, DL_DMA_BURST_SIZE_8);

    gUartDmaRxReadIndex = 0U;
    (void) memset(gUartDmaRxBuffer, 0, sizeof(gUartDmaRxBuffer));
    serial_dma_start_rx();

    DL_UART_Main_enableDMAReceiveEvent(UART_OUT_INST, DL_UART_MAIN_DMA_INTERRUPT_RX);
    DL_UART_Main_enableDMATransmitEvent(UART_OUT_INST);
}

/* 重启 RX DMA；repeat-single 模式会在 256 字节后自动回到缓冲区头部。 */
static void serial_dma_start_rx(void)
{
    DL_DMA_setSrcAddr(DMA, UART_DMA_RX_CHANNEL, (uint32_t)&UART_OUT_INST->RXDATA);
    DL_DMA_setDestAddr(DMA, UART_DMA_RX_CHANNEL, (uint32_t)gUartDmaRxBuffer);
    DL_DMA_setTransferSize(DMA, UART_DMA_RX_CHANNEL, UART_DMA_RX_BUFFER_SIZE);
    DL_DMA_enableChannel(DMA, UART_DMA_RX_CHANNEL);
}

/* 按给定波特率重新配置 UART；忙状态异常时通过硬复位外设退出，避免初始化永久卡死。 */
void TaskSerial_ApplyBaud(uint32_t baud_rate)
{
    DL_DMA_disableChannel(DMA, UART_DMA_RX_CHANNEL);
    DL_DMA_disableChannel(DMA, UART_DMA_TX_CHANNEL);
    DL_UART_Main_disableDMAReceiveEvent(UART_OUT_INST, DL_UART_MAIN_DMA_INTERRUPT_RX);
    DL_UART_Main_disableDMATransmitEvent(UART_OUT_INST);
    DL_UART_Main_disableInterrupt(UART_OUT_INST, DL_UART_MAIN_INTERRUPT_RX);

    DL_UART_Main_disable(UART_OUT_INST);
    if (serial_uart_wait_idle(UART_BUSY_TIMEOUT_CYCLES) == 0U) {
        DL_UART_Main_reset(UART_OUT_INST);
        DL_UART_Main_enablePower(UART_OUT_INST);
        DL_Common_delayCycles(UART_RESET_DELAY_CYCLES);
    }

    serial_uart_apply_config(baud_rate);
    serial_reset_rx_parser();
    (void) DL_UART_drainRXFIFO(UART_OUT_INST, gUartRxDrainBuffer, UART_RX_DRAIN_SIZE);
    serial_dma_init();
}

/* 阻塞式写字符串，用于命令回复、STATUS 和姿态数据输出。 */
void TaskSerial_Write(const char *text)
{
    while (*text != '\0') {
        uint16_t chunk_len = 0U;

        while ((text[chunk_len] != '\0') && (chunk_len < UART_DMA_TX_CHUNK_SIZE)) {
            gUartDmaTxBuffer[chunk_len] = (uint8_t)text[chunk_len];
            chunk_len++;
        }

        (void) serial_dma_wait_tx_idle(UART_DMA_TX_TIMEOUT_MS);
        DL_DMA_setSrcAddr(DMA, UART_DMA_TX_CHANNEL, (uint32_t)gUartDmaTxBuffer);
        DL_DMA_setDestAddr(DMA, UART_DMA_TX_CHANNEL, (uint32_t)&UART_OUT_INST->TXDATA);
        DL_DMA_setTransferSize(DMA, UART_DMA_TX_CHANNEL, chunk_len);
        DL_DMA_enableChannel(DMA, UART_DMA_TX_CHANNEL);
        (void) serial_dma_wait_tx_idle(UART_DMA_TX_TIMEOUT_MS);
        (void) serial_uart_wait_idle(UART_BUSY_TIMEOUT_CYCLES);
        text += chunk_len;
    }
}

/* 发送指定长度的原始字节，用于固定长度的二进制姿态数据包。 */
void TaskSerial_WriteBytes(const uint8_t *data, uint16_t length)
{
    while ((data != NULL) && (length != 0U)) {
        uint16_t chunk_len = length;

        if (chunk_len > UART_DMA_TX_CHUNK_SIZE) {
            chunk_len = UART_DMA_TX_CHUNK_SIZE;
        }
        (void) memcpy(gUartDmaTxBuffer, data, chunk_len);

        (void) serial_dma_wait_tx_idle(UART_DMA_TX_TIMEOUT_MS);
        DL_DMA_setSrcAddr(DMA, UART_DMA_TX_CHANNEL, (uint32_t)gUartDmaTxBuffer);
        DL_DMA_setDestAddr(DMA, UART_DMA_TX_CHANNEL, (uint32_t)&UART_OUT_INST->TXDATA);
        DL_DMA_setTransferSize(DMA, UART_DMA_TX_CHANNEL, chunk_len);
        DL_DMA_enableChannel(DMA, UART_DMA_TX_CHANNEL);
        (void) serial_dma_wait_tx_idle(UART_DMA_TX_TIMEOUT_MS);
        (void) serial_uart_wait_idle(UART_BUSY_TIMEOUT_CYCLES);

        data += chunk_len;
        length -= chunk_len;
    }
}

/* 消费 RX DMA 新字节，并只在收到 CR/LF 后把完整命令放入队列。 */
void TaskSerial_CollectRx(void)
{
    uint16_t dma_remaining;
    uint16_t write_index;

    if (!DL_DMA_isChannelEnabled(DMA, UART_DMA_RX_CHANNEL)) {
        serial_dma_start_rx();
    }

    dma_remaining = DL_DMA_getTransferSize(DMA, UART_DMA_RX_CHANNEL);
    if (dma_remaining > UART_DMA_RX_BUFFER_SIZE) {
        dma_remaining = UART_DMA_RX_BUFFER_SIZE;
    }
    write_index = (uint16_t)(UART_DMA_RX_BUFFER_SIZE - dma_remaining);
    if (write_index >= UART_DMA_RX_BUFFER_SIZE) {
        write_index = 0U;
    }

    while (gUartDmaRxReadIndex != write_index) {
        serial_collect_byte(gUartDmaRxBuffer[gUartDmaRxReadIndex]);
        gUartDmaRxReadIndex++;
        if (gUartDmaRxReadIndex >= UART_DMA_RX_BUFFER_SIZE) {
            gUartDmaRxReadIndex = 0U;
        }
    }
}

/* 非可打印字节会清空当前行，避免上电噪声污染用户发送的第一条有效命令。 */
static void serial_collect_byte(uint8_t rx)
{
    if ((rx == '\r') || (rx == '\n')) {
        if (gRxIndex != 0U) {
            if (gCommandQueueCount < UART_RX_QUEUE_DEPTH) {
                gRxLine[gRxIndex] = '\0';
                for (uint8_t i = 0U; i <= gRxIndex; i++) {
                    gCommandQueue[gCommandQueueHead][i] = gRxLine[i];
                }
                gCommandQueueHead++;
                if (gCommandQueueHead >= UART_RX_QUEUE_DEPTH) {
                    gCommandQueueHead = 0U;
                }
                gCommandQueueCount++;
            } else {
                gCommandOverflow = 1U;
            }
            gRxIndex = 0U;
        }
    } else if ((rx >= 0x20U) && (rx <= 0x7EU) &&
               (gRxIndex < (UART_RX_LINE_SIZE - 1U))) {
        gRxLine[gRxIndex++] = (char)rx;
    } else if ((rx < 0x20U) || (rx > 0x7EU)) {
        if (gRxIndex != 0U) {
            gCommandOverflow = 1U;
        }
        gRxIndex = 0U;
    } else {
        gRxIndex = 0U;
        gCommandOverflow = 1U;
    }
}

static void serial_reset_rx_parser(void)
{
    __disable_irq();
    gCommandOverflow = 0U;
    gRxIndex = 0U;
    gCommandQueueHead = 0U;
    gCommandQueueTail = 0U;
    gCommandQueueCount = 0U;
    (void) memset(gRxLine, 0, sizeof(gRxLine));
    (void) memset(gCommandQueue, 0, sizeof(gCommandQueue));
    __enable_irq();
}

/* 从命令队列取出一条完整命令；该接口只对串口命令模块开放。 */
uint8_t SerialTransport_TakeCommand(char line[UART_RX_LINE_SIZE])
{
    if (gCommandQueueCount == 0U) {
        return 0U;
    }

    __disable_irq();
    for (uint8_t i = 0U; i < UART_RX_LINE_SIZE; i++) {
        line[i] = gCommandQueue[gCommandQueueTail][i];
        if (gCommandQueue[gCommandQueueTail][i] == '\0') {
            break;
        }
    }
    gCommandQueueTail++;
    if (gCommandQueueTail >= UART_RX_QUEUE_DEPTH) {
        gCommandQueueTail = 0U;
    }
    gCommandQueueCount--;
    __enable_irq();

    return 1U;
}

/* 读取并清除命令接收溢出标志。 */
uint8_t SerialTransport_TakeOverflow(void)
{
    if (gCommandOverflow == 0U) {
        return 0U;
    }

    gCommandOverflow = 0U;
    return 1U;
}
