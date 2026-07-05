/*
 * Copyright (c) 2020, Texas Instruments Incorporated
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

#include <ti/driverlib/dl_i2c.h>

#ifdef __MSPM0_HAS_I2C__

/**
 *  @brief I2C Controller APIs
 */

/**
 * @brief 设置I2C模块的时钟配置
 * @param i2c I2C外设寄存器结构体指针
 * @param config 时钟配置结构体指针，包含时钟源和分频系数
 * @details
 * - 更新CLKSEL寄存器选择总线时钟(BUSCLK)和主功能时钟(MFCLK)
 * - 设置CLKDIV寄存器的分频系数
 * - BUSCLK用于I2C总线时钟，MFCLK用于内部数字逻辑
 */
void DL_I2C_setClockConfig(I2C_Regs *i2c, DL_I2C_ClockConfig *config)
{
    // 组合总线时钟和主功能时钟选择，更新CLKSEL寄存器
    DL_Common_updateReg(&i2c->CLKSEL, (uint32_t) config->clockSel,
        I2C_CLKSEL_BUSCLK_SEL_MASK | I2C_CLKSEL_MFCLK_SEL_MASK);

    // 设置时钟分频系数，更新CLKDIV寄存器
    DL_Common_updateReg(
        &i2c->CLKDIV, (uint32_t) config->divideRatio, I2C_CLKDIV_RATIO_MASK);
}

/**
 * @brief 获取当前I2C时钟配置
 * @param i2c I2C外设寄存器结构体指针
 * @param config 用于存储配置的结构体指针
 * @details 从CLKSEL和CLKDIV寄存器读取当前配置
 */
void DL_I2C_getClockConfig(I2C_Regs *i2c, DL_I2C_ClockConfig *config)
{
    // 从CLKSEL寄存器提取时钟源配置
    uint32_t clockSel =
        i2c->CLKSEL & (I2C_CLKSEL_BUSCLK_SEL_MASK | I2C_CLKSEL_MFCLK_SEL_MASK);
    config->clockSel = (DL_I2C_CLOCK)(clockSel);

    // 从CLKDIV寄存器提取分频系数
    uint32_t divideRatio = i2c->CLKDIV & I2C_CLKDIV_RATIO_MASK;
    config->divideRatio  = (DL_I2C_CLOCK_DIVIDE)(divideRatio);
}

/**
 * @brief 填充控制器发送FIFO
 * @param i2c I2C外设寄存器结构体指针
 * @param buffer 待发送数据缓冲区指针
 * @param count 期望写入的数据个数
 * @return 实际成功写入FIFO的数据个数
 * @details
 * - 循环写入数据直到FIFO满或完成指定数量
 * - 使用DL_I2C_transmitControllerData函数写入单字节
 */
uint16_t DL_I2C_fillControllerTXFIFO(
    I2C_Regs *i2c, uint8_t *buffer, uint16_t count)
{
    uint16_t i;
    for (i = (uint16_t) 0; i < count; i++) {
        // 检查FIFO状态，未满时继续写入
        if (DL_I2C_isControllerTXFIFOFull(i2c) == false) {
            DL_I2C_transmitControllerData(i2c, buffer[i]);
        } else {  // FIFO已满则退出循环
            break;
        }
    }
    return i; // 返回实际写入数量
}


/**
 * @brief 刷新控制器发送FIFO
 * @param i2c I2C外设寄存器结构体指针
 * @details
 * - 启动刷新后等待直到FIFO完全清空
 * - 使用状态检查函数轮询FIFO空状态
 */
void DL_I2C_flushControllerTXFIFO(I2C_Regs *i2c)
{
    DL_I2C_startFlushControllerTXFIFO(i2c); // 启动刷新操作
    while (DL_I2C_isControllerTXFIFOEmpty(i2c) == false) {
        ; // 等待直到FIFO完全清空
    }
    DL_I2C_stopFlushControllerTXFIFO(i2c); // 停止刷新操作
}

/**
 * @brief 刷新控制器接收FIFO
 * @param i2c I2C外设寄存器结构体指针
 * @details 流程与TX FIFO刷新类似，但操作接收FIFO
 */
void DL_I2C_flushControllerRXFIFO(I2C_Regs *i2c)
{
    DL_I2C_startFlushControllerRXFIFO(i2c);
    while (DL_I2C_isControllerRXFIFOEmpty(i2c) == false) {
        ;
    }
    DL_I2C_stopFlushControllerRXFIFO(i2c);
}

/*------------------------ 目标模式相关API ------------------------*/

/**
 * @brief 填充目标模式发送FIFO
 * @param i2c I2C外设寄存器结构体指针
 * @param buffer 待发送数据缓冲区指针
 * @param count 期望写入的数据个数
 * @return 实际成功写入的字节数
 * @details 功能类似控制器模式，但针对目标设备TX FIFO
 */
uint8_t DL_I2C_fillTargetTXFIFO(I2C_Regs *i2c, uint8_t *buffer, uint8_t count)
{
    uint8_t i;
    for (i = (uint8_t) 0; i < count; i++) {
        if (DL_I2C_isTargetTXFIFOFull(i2c) == false) {
            DL_I2C_transmitTargetData(i2c, buffer[i]);
        } else {
            break;
        }
    }
    return i;
}

/**
 * @brief 刷新目标模式发送FIFO
 * @param i2c I2C外设寄存器结构体指针
 * @details
 * - 启动TX FIFO刷新操作，清空所有待发送数据
 * - 循环等待直到FIFO完全清空（空状态标志置位）
 * - 典型应用场景：通信异常恢复/重新初始化时清理残留数据
 * @warning 此操作会导致未发送数据丢失，需谨慎使用
 */
void DL_I2C_flushTargetTXFIFO(I2C_Regs *i2c)
{
    DL_I2C_startFlushTargetTXFIFO(i2c);  // 启动发送FIFO刷新
    /* 轮询等待FIFO空状态 */
    while (DL_I2C_isTargetTXFIFOEmpty(i2c) == false) {
        ; // 阻塞等待，直到FIFO计数器归零
    }
    DL_I2C_stopFlushTargetTXFIFO(i2c);   // 终止刷新操作
}

/**
 * @brief 刷新目标模式接收FIFO
 * @param i2c I2C外设寄存器结构体指针
 * @details
 * - 启动RX FIFO刷新操作，丢弃所有已接收数据
 * - 等待直到FIFO完全清空
 * - 用于清除错误状态下的无效数据或缓冲区复位
 * @note 与发送FIFO不同，接收FIFO刷新会丢弃未读取数据
 */
void DL_I2C_flushTargetRXFIFO(I2C_Regs *i2c)
{
    DL_I2C_startFlushTargetRXFIFO(i2c);  // 启动接收FIFO刷新
    /* 等待FIFO清空 */
    while (DL_I2C_isTargetRXFIFOEmpty(i2c) == false) {
        ; // 持续检查直到无剩余数据
    }
    DL_I2C_stopFlushTargetRXFIFO(i2c);   // 停止刷新操作
}
/**
 * @brief 阻塞式发送目标数据
 * @param i2c I2C外设寄存器结构体指针
 * @param data 待发送的1字节数据
 * @details
 * - 持续轮询传输请求状态位，直到主机发起读取操作
 * - 当检测到TRANSMIT_REQUEST状态时写入数据到FIFO
 * - 阻塞式设计，适用于实时性要求高的场景
 * @warning 长时间未收到主机请求会导致死锁，建议设置超时机制
 */
void DL_I2C_transmitTargetDataBlocking(I2C_Regs *i2c, uint8_t data)
{
    /* 轮询等待传输请求状态 */
    while ((DL_I2C_getTargetStatus(i2c) &
               DL_I2C_TARGET_STATUS_TRANSMIT_REQUEST) !=
           DL_I2C_TARGET_STATUS_TRANSMIT_REQUEST) {
        ; // 等待直到主机请求数据（SCL时钟被拉低）
    }
    /* 写入数据到目标TX FIFO */
    DL_I2C_transmitTargetData(i2c, data);
}

/**
 * @brief 非阻塞式发送目标数据检查
 * @param i2c I2C外设寄存器结构体指针
 * @param data 待发送的1字节数据
 * @return bool 发送结果
 * - true  : 数据成功写入FIFO
 * - false : FIFO已满，写入失败
 * @details
 * - 先检查目标TX FIFO是否满状态
 * - 若未满则立即写入数据并返回成功
 * - 非阻塞设计，适合在中断服务例程中使用
 * @note 调用方需处理发送失败的情况（如重试机制）
 */
bool DL_I2C_transmitTargetDataCheck(I2C_Regs *i2c, uint8_t data)
{
    bool status;
    /* 检查FIFO满状态 */
    if (DL_I2C_isTargetTXFIFOFull(i2c)) {
        status = false;  // FIFO满，无法写入
    } else {
        DL_I2C_transmitTargetData(i2c, data); // 写入数据
        status = true;   // 返回成功状态
    }
    return status;
}

/**
 * @brief 阻塞式接收目标数据
 * @param i2c I2C外设寄存器结构体指针
 * @return uint8_t 接收到的1字节数据
 * @details
 * - 持续轮询接收请求状态位，等待主机写入数据
 * - 当检测到RECEIVE_REQUEST状态时从FIFO读取数据
 * - 保证返回有效数据，适用于必须等待数据的场景
 * @warning 若主机未发送数据会永久阻塞，建议配合超时使用
 */
uint8_t DL_I2C_receiveTargetDataBlocking(I2C_Regs *i2c)
{
    /* 等待接收请求状态（主机发起写操作） */
    while (
        (DL_I2C_getTargetStatus(i2c) & DL_I2C_TARGET_STATUS_RECEIVE_REQUEST) !=
        DL_I2C_TARGET_STATUS_RECEIVE_REQUEST) {
        ; // 等待直到检测到接收请求
    }
    /* 从目标RX FIFO读取数据 */
    return DL_I2C_receiveTargetData(i2c);
}

/**
 * @brief 非阻塞式接收目标数据检查
 * @param i2c I2C外设寄存器结构体指针
 * @param buffer 数据存储缓冲区指针
 * @return bool 接收状态
 * - true  : 成功读取数据到buffer
 * - false : FIFO空，无数据可读
 * @details
 * - 检查RX FIFO是否为空
 * - 若有数据则立即读取，避免阻塞程序流程
 * - 适合在主循环中定期调用或中断服务例程处理
 * @note 多次调用可清空FIFO，需注意数据顺序
 */
bool DL_I2C_receiveTargetDataCheck(I2C_Regs *i2c, uint8_t *buffer)
{
    bool status;
    /* 检查FIFO空状态 */
    if (DL_I2C_isTargetRXFIFOEmpty(i2c)) {
        status = false; // 无数据可用
    } else {
        *buffer = DL_I2C_receiveTargetData(i2c); // 读取数据
        status  = true;  // 返回成功状态
    }
    return status;
}

#endif /* __MSPM0_HAS_I2C__ */
