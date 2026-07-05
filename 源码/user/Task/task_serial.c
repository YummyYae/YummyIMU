#include "task_serial.h"

#include "ti_msp_dl_config.h"
#include "board_mspm0.h"
#include "task_imu.h"
#include "task_led.h"
#include "task_temperature.h"
#include "bmi088_mspm0.h"
#include "bmi270_mspm0.h"
#include "flash_storage.h"
#include "imu_attitude.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

/*
 * 串口任务流程：
 * 1. UART 中断和主循环都会调用 TaskSerial_CollectRx()，把收到的字节拼成一行命令。
 * 2. 完整命令进入一个小型环形队列，主循环调用 TaskSerial_PollCommand() 逐条处理。
 * 3. 收到任何命令后，先暂停姿态回传 3 秒，关闭 LED，回复 OK 和 RECV 回显。
 * 4. CAL/TEMP/BAUD/RATE 成功后会立即写入 Flash 并复位。
 * 5. 姿态数据输出也在本任务中完成，但输出节拍由 IMU 任务的回传分频控制。
 */
#define UART_DMA_RX_CHANNEL    0U
#define UART_DMA_TX_CHANNEL    1U
#define UART_DMA_RX_BUFFER_SIZE 256U
#define UART_DMA_TX_CHUNK_SIZE  128U
#define UART_RX_DRAIN_SIZE      16U
#define STATUS_BMI088_ACCEL_CHIP_ID 0x1EU
#define STATUS_BMI088_GYRO_CHIP_ID  0x0FU
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

/* 配置 UART RX/TX DMA。RX 持续写入环形缓冲，前台只解析已经搬运好的字节。 */
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

/* 按给定波特率重新配置 UART，并打开 RX 中断和 FIFO。 */
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

        while (DL_DMA_isChannelEnabled(DMA, UART_DMA_TX_CHANNEL)) {
        }

        DL_DMA_setSrcAddr(DMA, UART_DMA_TX_CHANNEL, (uint32_t)gUartDmaTxBuffer);
        DL_DMA_setDestAddr(DMA, UART_DMA_TX_CHANNEL, (uint32_t)&UART_OUT_INST->TXDATA);
        DL_DMA_setTransferSize(DMA, UART_DMA_TX_CHANNEL, chunk_len);
        DL_DMA_enableChannel(DMA, UART_DMA_TX_CHANNEL);

        while (DL_DMA_isChannelEnabled(DMA, UART_DMA_TX_CHANNEL)) {
        }

        (void) serial_uart_wait_idle(UART_BUSY_TIMEOUT_CYCLES);
        text += chunk_len;
    }
}

/*
 * 收集 UART RX 字节。
 * 遇到 CR/LF 时认为一行命令结束，并放入命令队列；队列满或行过长会置溢出标志。
 */
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

/* 将单个接收字节送入原有行解析器。DMA 只改变搬运方式，不改变命令协议。 */
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
    } else if ((rx >= 0x20U) && (rx <= 0x7EU) && (gRxIndex < (UART_RX_LINE_SIZE - 1U))) {
        gRxLine[gRxIndex++] = (char)rx;
    } else if ((rx < 0x20U) || (rx > 0x7EU)) {
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

/* 从命令队列取出一条完整命令；没有命令时返回 0。 */
uint8_t TaskSerial_TakeCommand(char line[UART_RX_LINE_SIZE])
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
uint8_t TaskSerial_TakeOverflow(void)
{
    if (gCommandOverflow == 0U) {
        return 0U;
    }

    gCommandOverflow = 0U;
    return 1U;
}

/* 输出 STATUS 快照：当前运行参数、Flash 中的零漂值以及两颗 IMU 温度。 */
static const char *status_bmi088_model_name(const RuntimeState_t *state)
{
    if ((state->accel_chip_id == STATUS_BMI088_ACCEL_CHIP_ID) &&
        (state->gyro_chip_id == STATUS_BMI088_GYRO_CHIP_ID)) {
        return "BMI088";
    }

    return "UNKNOWN";
}

static const char *status_bmi270_model_name(const RuntimeState_t *state)
{
    if (state->bmi270_chip_id == BMI270_CHIP_ID_VALUE) {
        return "BMI270";
    }

    if (state->bmi270_chip_id == BMI220_CHIP_ID_VALUE) {
        return "BMI220";
    }

    return "UNKNOWN";
}

static void serial_write_status_snapshot(RuntimeState_t *state)
{
    char line[256];
    RuntimeConfig_t flash_config;
    uint8_t flash_config_valid;

    flash_config_valid = RuntimeConfig_Load(&flash_config);

    if (flash_config_valid != 0U) {
        (void) snprintf(line, sizeof(line),
                        "BAUD:%lu\n"
                        "RATE:%lu\n"
                        "MODE:%s\n"
                        "TARGET_TEMP:%.3f\n",
                        (unsigned long) flash_config.baud_rate,
                        (unsigned long) flash_config.report_rate_hz,
                        (flash_config.output_mode == RUNTIME_OUTPUT_MODE_DEBUG) ? "DEBUG" : "USE",
                        flash_config.target_temperature_c);
    } else {
        (void) snprintf(line, sizeof(line),
                        "BAUD:INVALID\n"
                        "RATE:INVALID\n"
                        "MODE:INVALID\n"
                        "TARGET_TEMP:INVALID\n");
    }
    TaskSerial_Write(line);

    (void) snprintf(line, sizeof(line),
                    "BMI088_MODEL:%s\n"
                    "BMI270_MODEL:%s\n",
                    status_bmi088_model_name(state),
                    status_bmi270_model_name(state));
    TaskSerial_Write(line);

    if ((flash_config_valid != 0U) && (flash_config.gyro_bias_valid != 0U)) {
        (void) snprintf(line, sizeof(line),
                        "BMI088_BIAS_X:%.6f\n"
                        "BMI088_BIAS_Y:%.6f\n"
                        "BMI088_BIAS_Z:%.6f\n"
                        "BMI270_BIAS_X:%.6f\n"
                        "BMI270_BIAS_Y:%.6f\n"
                        "BMI270_BIAS_Z:%.6f\n",
                        flash_config.gyro_bias.bmi088[0],
                        flash_config.gyro_bias.bmi088[1],
                        flash_config.gyro_bias.bmi088[2],
                        flash_config.gyro_bias.bmi270[0],
                        flash_config.gyro_bias.bmi270[1],
                        flash_config.gyro_bias.bmi270[2]);
    } else {
        (void) snprintf(line, sizeof(line),
                        "BMI088_BIAS_X:INVALID\n"
                        "BMI088_BIAS_Y:INVALID\n"
                        "BMI088_BIAS_Z:INVALID\n"
                        "BMI270_BIAS_X:INVALID\n"
                        "BMI270_BIAS_Y:INVALID\n"
                        "BMI270_BIAS_Z:INVALID\n");
    }
    TaskSerial_Write(line);

    (void) snprintf(line, sizeof(line), "BMI088_TEMP:%.3f\n", BMI088Sensor.Temperature);
    TaskSerial_Write(line);

    if (state->bmi270_temperature_valid != 0U) {
        (void) snprintf(line, sizeof(line), "BMI270_TEMP:%.3f\n", BMI270Sensor.Temperature);
    } else {
        (void) snprintf(line, sizeof(line), "BMI270_TEMP:INVALID\n");
    }
    TaskSerial_Write(line);

    (void) snprintf(line, sizeof(line),
                    "BMI088_HEAT:%.3f\n"
                    "BMI270_HEAT:%.3f\n",
                    state->bmi088_heater_duty,
                    state->bmi270_heater_duty);
    TaskSerial_Write(line);

    (void) snprintf(line, sizeof(line),
                    "BMI088_GYRO_SAT:%lu\n"
                    "BMI088_ACCEL_SAT:%lu\n"
                    "BMI270_GYRO_SAT:%lu\n"
                    "BMI270_ACCEL_SAT:%lu\n"
                    "IMU_UPDATE:%lu\n"
                    "IMU_OVERRUN:%lu\n",
                    (unsigned long) BMI088Sensor.GyroSaturationCount,
                    (unsigned long) BMI088Sensor.AccelSaturationCount,
                    (unsigned long) BMI270Sensor.GyroSaturationCount,
                    (unsigned long) BMI270Sensor.AccelSaturationCount,
                    (unsigned long) gImuRuntimeStats.update_count,
                    (unsigned long) gImuRuntimeStats.overrun_count);
    TaskSerial_Write(line);
}

/* 对有效命令统一回复 OK，并回显收到的原始命令。 */
static void command_ack_echo(const char *line)
{
    char echo[UART_RX_LINE_SIZE + 16U];

    TaskSerial_Write("OK\n");
    (void) snprintf(echo, sizeof(echo), "RECV:%s\n", line);
    TaskSerial_Write(echo);
}

/* 暂停姿态回传一段时间，并关闭 LED，用于给命令回复留出干净串口窗口。 */
void TaskSerial_PauseReport(RuntimeState_t *state)
{
    state->uart_report_resume_tick = gSystemTickMs + COMMAND_REPORT_PAUSE_MS;
    TaskLED_Set(0U);
}

/* 跳过命令参数前的空格、TAB 和逗号，支持命令参数用空格或逗号分隔。 */
static char *skip_spaces(char *text)
{
    while ((*text == ' ') || (*text == '\t') || (*text == ',')) {
        text++;
    }

    return text;
}

/* 解析无符号整数参数，用于 CAL/BAUD/RATE 等命令。 */
static uint8_t parse_u32_arg(char **text, uint32_t *value)
{
    uint32_t result = 0U;
    char *p = skip_spaces(*text);

    if ((*p < '0') || (*p > '9')) {
        return 0U;
    }

    while ((*p >= '0') && (*p <= '9')) {
        result = (result * 10U) + (uint32_t)(*p - '0');
        p++;
    }

    *text = p;
    *value = result;
    return 1U;
}

/* 解析温度浮点参数，避免引入 sscanf 造成额外开销。 */
static uint8_t parse_temp_arg(char **text, float *value)
{
    int32_t sign = 1;
    int32_t integer = 0;
    int32_t fraction = 0;
    int32_t fraction_scale = 1;
    char *p = skip_spaces(*text);

    if (*p == '-') {
        sign = -1;
        p++;
    }

    if ((*p < '0') || (*p > '9')) {
        return 0U;
    }

    while ((*p >= '0') && (*p <= '9')) {
        integer = (integer * 10) + (int32_t)(*p - '0');
        p++;
    }

    if (*p == '.') {
        p++;
        while ((*p >= '0') && (*p <= '9') && (fraction_scale < 1000)) {
            fraction = (fraction * 10) + (int32_t)(*p - '0');
            fraction_scale *= 10;
            p++;
        }
    }

    *text = p;
    *value = (float)sign * ((float)integer + ((float)fraction / (float)fraction_scale));
    return 1U;
}

static uint8_t command_is_separator(char ch)
{
    return ((ch == '\0') || (ch == ' ') || (ch == '\t') || (ch == ',')) ? 1U : 0U;
}

static uint8_t command_is_end(char *text)
{
    char *p = skip_spaces(text);

    return (*p == '\0') ? 1U : 0U;
}

static uint8_t command_match(const char *text, const char *keyword, uint8_t length)
{
    if (strncmp(text, keyword, length) != 0) {
        return 0U;
    }

    return command_is_separator(text[length]);
}

/*
 * 上电早期 RX 可能先收到几个脏字节，然后用户才发送真正命令。
 * 这里在整行中寻找第一个合法命令头，让 "xxSTATUS" 仍能按 STATUS 处理；
 * 没有合法命令头的噪声行会被静默丢弃，不暂停姿态输出。
 */
static char *command_find_start(char *line)
{
    char *p = line;

    while (*p != '\0') {
        if ((command_match(p, "STATUS", 6U) != 0U) ||
            (command_match(p, "CAL", 3U) != 0U) ||
            (command_match(p, "TEMP", 4U) != 0U) ||
            (command_match(p, "BAUD", 4U) != 0U) ||
            (command_match(p, "RATE", 4U) != 0U) ||
            (command_match(p, "MODE", 4U) != 0U)) {
            return p;
        }
        p++;
    }

    return NULL;
}

/* 保存当前 RAM 配置到 Flash，成功后延时复位，让新参数按上电流程重新生效。 */
static void command_save_and_reset(RuntimeState_t *state, const char *reason)
{
    char line[64];

    (void) snprintf(line, sizeof(line), "%s:SAVE_START\n", reason);
    TaskSerial_Write(line);
    TaskIMU_EnableInterruptUpdate(0U);
    RuntimeState_ClearImuUpdates();
    if (RuntimeConfig_Save(&state->runtime_config) != 0U) {
        (void) snprintf(line, sizeof(line), "%s:SAVE_DONE\n", reason);
        TaskSerial_Write(line);
        Board_ResetAfterSave();
    } else {
        TaskSerial_Write("ERROR:SAVE_FAILED\n");
    }
}

/*
 * 处理单条命令。
 * CAL/TEMP/BAUD/RATE 成功后自动保存并复位；STATUS 只查询，不写 Flash。
 */
static uint8_t command_handle(RuntimeState_t *state, char *line)
{
    char *cmd = command_find_start(line);
    char *p = cmd;

    if (cmd == NULL) {
        return 0U;
    }

    if (command_match(p, "CAL", 3U) != 0U) {
        uint32_t wait_s;
        uint32_t record_s;
        p += 3;
        if ((parse_u32_arg(&p, &wait_s) == 0U) ||
            (parse_u32_arg(&p, &record_s) == 0U) ||
            (wait_s > 600U) || (record_s == 0U) || (record_s > 600U) ||
            (command_is_end(p) == 0U)) {
            return 0U;
        }

        TaskSerial_PauseReport(state);
        command_ack_echo(cmd);
        TaskSerial_Write("CAL:START\n");
        TaskIMU_EnableInterruptUpdate(0U);
        if (GyroBias_CalibrateWithService(&state->runtime_config.gyro_bias,
                                          wait_s * 1000U,
                                          record_s * 1000U,
                                          TaskTemperature_CalibrationService,
                                          state) != 0U) {
            state->runtime_config.gyro_bias_valid = 1U;
            state->gyro_bias_valid = 1U;
            state->gyro_bias_calibrated = 1U;
            TaskIMU_AlignInitialAttitude();
            TaskIMU_EnableInterruptUpdate(1U);
            TaskSerial_PauseReport(state);
            TaskSerial_Write("CAL:DONE\n");
            command_save_and_reset(state, "CAL");
        } else {
            TaskIMU_EnableInterruptUpdate(1U);
            TaskSerial_Write("ERROR:CAL_FAILED\n");
        }
        return 1U;
    } else if (command_match(p, "TEMP", 4U) != 0U) {
        float target_temp;
        p += 4;
        if ((parse_temp_arg(&p, &target_temp) == 0U) ||
            (target_temp < 20.0f) || (target_temp > 85.0f) ||
            (command_is_end(p) == 0U)) {
            return 0U;
        }

        TaskSerial_PauseReport(state);
        command_ack_echo(cmd);
        state->runtime_config.target_temperature_c = target_temp;
        BMI088Sensor.TempWhenCali = target_temp;
        state->bmi088_heater_pid_integral = 0.0f;
        state->bmi088_heater_pid_last_error = 0.0f;
        state->bmi270_heater_pid_integral = 0.0f;
        state->bmi270_heater_pid_last_error = 0.0f;
        TaskSerial_Write("TEMP:UPDATED\n");
        command_save_and_reset(state, "TEMP");
        return 1U;
    } else if (command_match(p, "BAUD", 4U) != 0U) {
        uint32_t baud_rate;
        p += 4;
        if ((parse_u32_arg(&p, &baud_rate) == 0U) ||
            (RuntimeState_BaudRateIsSupported(baud_rate) == 0U) ||
            (command_is_end(p) == 0U)) {
            return 0U;
        }

        TaskSerial_PauseReport(state);
        command_ack_echo(cmd);
        state->runtime_config.baud_rate = baud_rate;
        TaskSerial_Write("BAUD:UPDATED\n");
        command_save_and_reset(state, "BAUD");
        return 1U;
    } else if (command_match(p, "RATE", 4U) != 0U) {
        uint32_t report_rate;
        p += 4;
        if ((parse_u32_arg(&p, &report_rate) == 0U) ||
            (report_rate < 1U) || (report_rate > 500U) ||
            (command_is_end(p) == 0U)) {
            return 0U;
        }

        TaskSerial_PauseReport(state);
        command_ack_echo(cmd);
        state->runtime_config.report_rate_hz = report_rate;
        RuntimeState_UpdateReportRate(state);
        TaskSerial_Write("RATE:UPDATED\n");
        command_save_and_reset(state, "RATE");
        return 1U;
    } else if (command_match(p, "MODE", 4U) != 0U) {
        p = skip_spaces(p + 4);
        if ((strncmp(p, "DEBUG", 5U) == 0) && (command_is_end(p + 5) != 0U)) {
            state->runtime_config.output_mode = RUNTIME_OUTPUT_MODE_DEBUG;
        } else if ((strncmp(p, "USE", 3U) == 0) && (command_is_end(p + 3) != 0U)) {
            state->runtime_config.output_mode = RUNTIME_OUTPUT_MODE_USE;
        } else {
            return 0U;
        }

        TaskSerial_PauseReport(state);
        command_ack_echo(cmd);
        TaskSerial_Write("MODE:UPDATED\n");
        command_save_and_reset(state, "MODE");
        return 1U;
    } else if ((command_match(p, "STATUS", 6U) != 0U) && (command_is_end(p + 6) != 0U)) {
        TaskSerial_PauseReport(state);
        command_ack_echo(cmd);
        serial_write_status_snapshot(state);
        return 1U;
    }

    return 0U;
}

/* 前台命令调度入口；返回 1 表示本轮处理了溢出或命令，低优先级任务应让出本轮。 */
uint8_t TaskSerial_PollCommand(RuntimeState_t *state)
{
    char line[UART_RX_LINE_SIZE];
    uint8_t handled = 0U;

    if (TaskSerial_TakeOverflow() != 0U) {
        handled = 0U;
    }

    if (TaskSerial_TakeCommand(line) == 0U) {
        return handled;
    }

    return command_handle(state, line);
}

/* 零漂整定过程中的进度回调，每秒输出剩余时间。 */
void GyroBias_CalibrationProgress(uint32_t elapsed_ms, uint32_t total_ms)
{
    if ((elapsed_ms % 1000U) == 0U) {
        char line[64];
        uint32_t remaining_s = (total_ms - elapsed_ms + 999U) / 1000U;

        (void) snprintf(line, sizeof(line), "CAL_REMAIN:%u\n", remaining_s);
        TaskSerial_Write(line);
    }
}


