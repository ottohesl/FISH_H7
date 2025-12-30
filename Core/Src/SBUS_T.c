#include "../Inc/SBUS_T.h"

/************************ 全局变量 ************************/
UART_HandleTypeDef *sbus_huart;          // SBUS串口句柄
UART_HandleTypeDef *sbus_debug_huart;    // 调试串口句柄
SBUS_Data_t sbus_data;                   // SBUS核心数据
uint8_t sbus_speed = 0;                  // 映射后的速度值

/************************ DMA接收缓冲区 ************************/
// STM32H7需迁址到DMA可访问区域（0x24000000开始）
#if SBUS_H_Vision==7
uint8_t SBUS_RX[SBUS_DMA_RX_SIZE] __attribute__((section(".ram")));
#else
uint8_t SBUS_RX[SBUS_DMA_RX_SIZE];
#endif

/************************ 帧解析静态变量（状态机） ************************/
static uint32_t dma_last_index = 0;      // 上一次解析的DMA索引
static uint8_t frame_buf[SBUS_PACKET_LENGTH]; // 单帧临时缓冲区
static uint8_t frame_pos = 0;            // 当前帧接收位置
static uint32_t frame_timeout = 0;       // 帧解析超时计数器
static SBUS_FrameState frame_state = SBUS_SEEK_START; // 解析状态机

/************************ 私有函数实现 ************************/
/**
 * @brief  SBUS通道值映射（速度转换）
 * @param  value: SBUS通道原始值
 * @retval 映射后的速度值（2-10）
 */
static int mapValue(int value) {
    // 输入范围限制
    if (value < sbus_ch3_center) value = sbus_ch3_center;
    if (value > sbus_ch3_max) value = sbus_ch3_max;
    
    // 线性映射：(输入-中值)*(最大-最小)/(最大-中值) + 最小值
    double mapped = speed_min + (value - sbus_ch3_center) * (speed_max - speed_min) / (sbus_ch3_max - sbus_ch3_center);
    return (int)(mapped + 0.5); // 四舍五入
}

/**
 * @brief  SBUS帧解码（原有逻辑保留）
 * @param  packet: 25字节完整SBUS帧
 */
static void SBUS_DecodePacket(uint8_t *packet) {
    // 帧头/帧尾校验
    if (packet[0] != SBUS_STARTBYTE || packet[24] != SBUS_ENDBYTE) {
#if SBUS_DEBUG_MODE
        ottohesl_uart(sbus_debug_huart, "SBUS帧格式错误\r\n");
#endif
        return;
    }

    // 解码16个通道（11位数据）
    sbus_data.channels[0]  = ((packet[1] | (packet[2] << 8)) & 0x07FF);
    sbus_data.channels[1]  = ((packet[2] >> 3 | (packet[3] << 5)) & 0x07FF);
    sbus_data.channels[2]  = ((packet[3] >> 6 | (packet[4] << 2) | (packet[5] << 10)) & 0x07FF);
    sbus_data.channels[3]  = ((packet[5] >> 1 | (packet[6] << 7)) & 0x07FF);
    sbus_data.channels[4]  = ((packet[6] >> 4 | (packet[7] << 4)) & 0x07FF);
    sbus_data.channels[5]  = ((packet[7] >> 7 | (packet[8] << 1) | (packet[9] << 9)) & 0x07FF);
    sbus_data.channels[6]  = ((packet[9] >> 2 | (packet[10] << 6)) & 0x07FF);
    sbus_data.channels[7]  = ((packet[10] >> 5 | (packet[11] << 3)) & 0x07FF);
    sbus_data.channels[8]  = ((packet[12] | (packet[13] << 8)) & 0x07FF);
    sbus_data.channels[9]  = ((packet[13] >> 3 | (packet[14] << 5)) & 0x07FF);
    sbus_data.channels[10] = ((packet[14] >> 6 | (packet[15] << 2) | (packet[16] << 10)) & 0x07FF);
    sbus_data.channels[11] = ((packet[16] >> 1 | (packet[17] << 7)) & 0x07FF);
    sbus_data.channels[12] = ((packet[17] >> 4 | (packet[18] << 4)) & 0x07FF);
    sbus_data.channels[13] = ((packet[18] >> 7 | (packet[19] << 1) | (packet[20] << 9)) & 0x07FF);
    sbus_data.channels[14] = ((packet[20] >> 2 | (packet[21] << 6)) & 0x07FF);
    sbus_data.channels[15] = ((packet[21] >> 5 | (packet[22] << 3)) & 0x07FF);

    // 解码标志位
    sbus_data.flags = packet[23];
    sbus_data.failsafe = (sbus_data.flags & 0x10) ? 1 : 0;  // bit4：失联标志
    sbus_data.frame_lost = (sbus_data.flags & 0x20) ? 1 : 0;// bit5：丢帧标志

    // 更新状态
    sbus_data.new_data_available = 1;
    sbus_data.last_update_time = HAL_GetTick();
}

/**
 * @brief  获取SBUS命令（原有逻辑保留）
 * @retval SBUS命令枚举值
 */
static SBUS_Command_t SBUS_GetCommand(void) {
    const uint16_t CH1_NEUTRAL = 874;  // 右摇杆左右中值
    const uint16_t CH3_NEUTRAL = 488;  // 左摇杆上下中值
    const uint16_t DEAD_ZONE = 50;     // 死区范围

    uint16_t left_stick_vertical = sbus_data.channels[2];  // 左摇杆上下（前进/停止）
    uint16_t right_stick_horizontal = sbus_data.channels[0]; // 右摇杆左右（转向）

    // 前进判断
    int is_forward = (left_stick_vertical > (CH3_NEUTRAL + DEAD_ZONE));
    if (is_forward) {
        sbus_speed = mapValue(left_stick_vertical); // 映射速度
        // 转向判断
        if (right_stick_horizontal < (CH1_NEUTRAL - DEAD_ZONE)) {
            return SBUS_CMD_TURN_LEFT;
        } else if (right_stick_horizontal > (CH1_NEUTRAL + DEAD_ZONE)) {
            return SBUS_CMD_TURN_RIGHT;
        } else {
            return SBUS_CMD_FORWARD;
        }
    }

    return SBUS_CMD_STOP;
}

/**
 * @brief  执行SBUS命令（原有逻辑保留）
 */
static void SBUS_ExecuteCommand(void) {
    // 失联保护
    if (sbus_data.failsafe) {
        Fish_ExecuteCommand(CMD_STOP);
#if SBUS_DEBUG_MODE
        ottohesl_uart(sbus_debug_huart, "SBUS失联，执行停止\r\n");
#endif
        return;
    }

    SBUS_Command_t cmd = SBUS_GetCommand();
    switch(cmd) {
        case SBUS_CMD_FORWARD:
            Fish_ExecuteCommand(CMD_FORWARD);
#if SBUS_DEBUG_MODE
            char speed_buf[32];
            sprintf(speed_buf, "前进，速度：%d\r\n", sbus_speed);
            ottohesl_uart(sbus_debug_huart, speed_buf);
#endif
            break;
        case SBUS_CMD_TURN_LEFT:
            Fish_ExecuteCommand(CMD_TURN_LEFT);
#if SBUS_DEBUG_MODE
            ottohesl_uart(sbus_debug_huart, "左转\r\n");
#endif
            break;
        case SBUS_CMD_TURN_RIGHT:
            Fish_ExecuteCommand(CMD_TURN_RIGHT);
#if SBUS_DEBUG_MODE
            ottohesl_uart(sbus_debug_huart, "右转\r\n");
#endif
            break;
        case SBUS_CMD_STOP:
            Fish_ExecuteCommand(CMD_STOP);
#if SBUS_DEBUG_MODE
            ottohesl_uart(sbus_debug_huart, "停止\r\n");
#endif
            break;
    }
}

/************************ 公开函数实现 ************************/
/**
 * @brief  SBUS初始化（启动DMA循环接收）
 * @param  h_sbus: SBUS串口句柄
 * @param  h_debug: 调试串口句柄
 * @note   需在CubeMX中开启UART DMA接收（循环模式）
 */
void SBUS_Init(UART_HandleTypeDef *h_sbus, UART_HandleTypeDef *h_debug) {
    // 初始化句柄
    sbus_huart = h_sbus;
    sbus_debug_huart = h_debug;
    
    // 初始化数据结构体
    memset(&sbus_data, 0, sizeof(SBUS_Data_t));
    
    // 启动DMA循环接收
    HAL_StatusTypeDef ret = HAL_UART_Receive_DMA(sbus_huart, SBUS_RX, SBUS_DMA_RX_SIZE);
    
    // 调试信息
#if SBUS_DEBUG_MODE
    if (ret != HAL_OK) {
        ottohesl_uart(sbus_debug_huart, "SBUS DMA初始化失败\r\n");
    } else if (sbus_huart->hdmarx->State != HAL_DMA_STATE_READY) {
        ottohesl_uart(sbus_debug_huart, "SBUS DMA未就绪\r\n");
    } else {
        ottohesl_uart(sbus_debug_huart, "SBUS DMA初始化成功\r\n");
    }
#endif
}

/**
 * @brief  SBUS数据处理（核心函数，需在主循环调用）
 * @retval true: 解析到有效数据；false: 无有效数据
 * @note   1. 处理DMA环形缓冲区数据 2. 状态机解析SBUS帧 3. 执行命令 4. 超时检测
 */
bool SBUS_Process(void) {
    bool has_new_data = false;
    uint32_t dma_curr_index = 0;
    uint32_t dma_data_len = 0;

    // 1. 关闭全局中断，防止DMA数据污染
    __disable_irq();
    
    // 2. 计算DMA已接收数据长度（环形缓冲区）
    dma_curr_index = SBUS_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(sbus_huart->hdmarx);
    if (dma_curr_index >= dma_last_index) {
        dma_data_len = dma_curr_index - dma_last_index;
    } else {
        dma_data_len = SBUS_DMA_RX_SIZE + dma_curr_index - dma_last_index;
    }
    
    // 3. 恢复全局中断
    __enable_irq();

    // 4. 解析DMA缓冲区数据（状态机）
    if (dma_data_len > 0) {
        for (uint32_t i = 0; i < dma_data_len; i++) {
            // 环形缓冲区取数
            uint8_t byte = SBUS_RX[(dma_last_index + i) % SBUS_DMA_RX_SIZE];
            frame_timeout++;

            // 超时保护：重置状态机
            if (frame_timeout > SBUS_FRAME_TIMEOUT) {
                frame_pos = 0;
                frame_timeout = 0;
                frame_state = SBUS_SEEK_START;
                continue;
            }

            // 状态机解析
            switch (frame_state) {
                case SBUS_SEEK_START:
                    // 找到起始字节，开始接收帧
                    if (byte == SBUS_STARTBYTE) {
                        frame_buf[frame_pos++] = byte;
                        frame_timeout = 0;
                        frame_state = SBUS_RECEIVE_DATA;
                    }
                    break;

                case SBUS_RECEIVE_DATA:
                    // 接收剩余24字节
                    frame_buf[frame_pos++] = byte;
                    // 帧接收完成
                    if (frame_pos >= SBUS_PACKET_LENGTH) {
                        // 解码并执行
                        SBUS_DecodePacket(frame_buf);
                        has_new_data = true;
                        
                        // 重置状态机
                        frame_pos = 0;
                        frame_timeout = 0;
                        frame_state = SBUS_SEEK_START;
                    }
                    break;
            }
        }
    }

    // 5. 更新DMA解析索引
    dma_last_index = dma_curr_index;

    // 6. 执行命令（有新数据时）
    if (sbus_data.new_data_available) {
        SBUS_ExecuteCommand();
        sbus_data.new_data_available = 0;
    }

    // 7. 通信超时检测（100ms失联则停止）
    if (HAL_GetTick() - sbus_data.last_update_time > SBUS_FAILSAFE_TIMEOUT) {
        sbus_data.failsafe = 1;
        Fish_ExecuteCommand(CMD_STOP);
#if SBUS_DEBUG_MODE
        ottohesl_uart(sbus_debug_huart, "SBUS超时，执行停止\r\n");
#endif
    }

    return has_new_data;
}