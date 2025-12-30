#include "sbus.h"
#include <stdio.h>
#include "steering.h"  // 使用您现有的机械鱼运动函数
#include <string.h>
#include <tgmath.h>

// 全局变量定义
SBUS_Data_t sbus_data;
UART_HandleTypeDef *sbus_huart;
uint8_t sbus_rx_byte;
uint8_t sbus_rx_buffer[SBUS_PACKET_LENGTH];
uint8_t sbus_rx_index = 0;
static uint8_t left_turn_executed = 0;  // 标记左转是否已执行
// SBUS控制初始化
void SBUS_Control_Init(UART_HandleTypeDef *huart)
{

    sbus_huart = huart;
    memset(&sbus_data, 0, sizeof(sbus_data));
    memset(sbus_rx_buffer, 0, sizeof(sbus_rx_buffer));
    // 启动SBUS接收中断
    HAL_StatusTypeDef status = HAL_UART_Receive_IT(sbus_huart, &sbus_rx_byte, 1);

    if (status == HAL_OK) {
    } else {
        printf("SBUS接收中断启动失败！错误代码: %d\r\n", status);
    }
}


int mapValue(int value) {
    // 输入范围检查
    //if (value < sbus_ch3_center) value = sbus_ch3_center;
    if (value > sbus_ch3_max) value = sbus_ch3_max;

    // 线性映射公式：y = 2 + (x-540) * (10-2) / (1200-540)
    double mapped = speed_min + (value - sbus_ch3_center) * (speed_max-speed_min) / (sbus_ch3_max-sbus_ch3_center);

    // 四舍五入到最近的整数
    return (int)(mapped + 0.5);  // 正数四舍五入
}

// SBUS主处理函数
void SBUS_Control_Process(void)
{
   // printf("SBUS处理中...\r\n");  // 添加调试信息
    if (sbus_data.new_data_available) {

        printf("SBUS数据 - ");
        printf("Ch1:%4d  Ch2:%4d   | ",
               sbus_data.channels[0],  // 通道1
               sbus_data.channels[2] // 通道2
             );

        // 显示当前命令
        SBUS_Command_t cmd = SBUS_GetCommand();
        switch(cmd) {
        case SBUS_CMD_FORWARD: printf("命令:前进"); break;
        case SBUS_CMD_TURN_LEFT: printf("命令:左转"); break;
        case SBUS_CMD_TURN_RIGHT: printf("命令:右转"); break;
        case SBUS_CMD_STOP: printf("命令:停止"); break;
        }
        printf("\r\n");

        SBUS_ExecuteCommand();
        sbus_data.new_data_available = 0;
    }

    // 通信超时检测（100ms超时进入安全模式）
    if (HAL_GetTick() - sbus_data.last_update_time > 100) {
        sbus_data.failsafe = 1;
        // 超时处理：停止机械鱼或进入安全模式
        Fish_ExecuteCommand(CMD_STOP); // 或您认为的安全状态
    }
}

// SBUS数据包解码
void SBUS_DecodePacket(uint8_t *packet)
{

    if (packet[0] != SBUS_STARTBYTE || packet[24] != SBUS_ENDBYTE) {
        printf("SBUS包格式错误！\r\n");
        return;
    }


    // 解码16个通道
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
    sbus_data.failsafe = (sbus_data.flags & 0x10) ? 1 : 0;   //bit4  0：遥控器信号正常,1:遥控器信号完全丢失，进入安全模式
    sbus_data.frame_lost = (sbus_data.flags & 0x20) ? 1 : 0;  //bit5   0：数据接收正常,1：丢失了一帧数据

    sbus_data.new_data_available = 1;    //告诉主循环有新数据需要处理
    sbus_data.last_update_time = HAL_GetTick();   //记录最后更新时间，用于超时检测
}

// 获取SBUS命令
SBUS_Command_t SBUS_GetCommand(void)
{
    const uint16_t CH1_NEUTRAL = 874;  // 右摇杆左右通道中值
    const uint16_t CH3_NEUTRAL = 488;  // 左摇杆上下通道中值
    const uint16_t DEAD_ZONE = 50;     // 死区范围

    uint16_t left_stick_vertical = sbus_data.channels[2];  // 通道2：左摇杆上下（前进/后退）
    uint16_t right_stick_horizontal = sbus_data.channels[0]; // 通道1：右摇杆左右（转向）

    // 首先检查是否在前进状态
    int is_forward = (left_stick_vertical > (CH3_NEUTRAL + DEAD_ZONE));

    if (is_forward) {
        // 在前进状态下检查转向
        if (right_stick_horizontal < (CH1_NEUTRAL - DEAD_ZONE)) {
            // 前进 + 左转
            return SBUS_CMD_TURN_LEFT;
        }
        else if (right_stick_horizontal > (CH1_NEUTRAL + DEAD_ZONE)) {
            // 前进 + 右转
            return SBUS_CMD_TURN_RIGHT;
        }
        else {
            // 前进但转向摇杆在中位 - 直行前进

          speed = mapValue(left_stick_vertical);
            printf("speed=%d\r\n", speed);
            return SBUS_CMD_FORWARD;
        }
    }


    return SBUS_CMD_STOP;
}

// 执行SBUS命令
void SBUS_ExecuteCommand(void)
{
    if (sbus_data.failsafe) {
        Fish_ExecuteCommand(CMD_FORWARD);
        return;
    }
    SBUS_Command_t cmd = SBUS_GetCommand();

    switch(cmd) {
        case SBUS_CMD_FORWARD:
            // 前进函数
            Fish_ExecuteCommand(CMD_FORWARD);
            break;

        case SBUS_CMD_TURN_LEFT:
            // 左转函数
            Fish_ExecuteCommand(CMD_TURN_LEFT);
            break;

        case SBUS_CMD_TURN_RIGHT:
            // 右转函数
            Fish_ExecuteCommand(CMD_TURN_RIGHT);
            break;

        case SBUS_CMD_STOP:
            // 停止或安全模式处理
             Fish_ExecuteCommand(CMD_STOP);
            // 可以根据需要调用您的停止函数或保持当前状态
            break;
    }
}

// SBUS接收完成回调
void SBUS_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == sbus_huart) {

        static uint8_t sbus_packet[SBUS_PACKET_LENGTH];  //用于存储完整的SBUS数据包25字节
        static uint16_t packet_index = 0;   //记录当前已经接收到的字节位置
        // 寻找起始字节
        if (packet_index == 0 && sbus_rx_byte != SBUS_STARTBYTE) {             // 如果不是起始字节，重置
            HAL_UART_Receive_IT(sbus_huart, &sbus_rx_byte, 1);
            return;
        }
            sbus_packet[packet_index++] = sbus_rx_byte;  //直接存入缓冲区相应位置
            //printf("接收数据包第%d字节: 0x%02X\r\n", sbus_rx_index, sbus_rx_byte);   //两个串口会串口中断处理中的时序冲突，不要写
            // 检查是否收到完整数据包
            if (packet_index >= SBUS_PACKET_LENGTH) {    //当packet_index达到25时，说明收到了完整数据包

                if (sbus_packet[SBUS_PACKET_LENGTH-1] == SBUS_ENDBYTE) {
                    SBUS_DecodePacket(sbus_packet);   //如果数据包完整且校验正确，调用解码函数  22个数据字节组成16个数据结构，处理16个通道
                }
                packet_index = 0;    //重置

            }


        // 重新启动接收
        HAL_UART_Receive_IT(sbus_huart, &sbus_rx_byte, 1);    //等待下一个字节的到来,不开就不能连续中断持续接收
    }
}

// 检查新命令可用性
uint8_t SBUS_IsNewCommandAvailable(void)
{
    return sbus_data.new_data_available;
}