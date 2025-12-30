//
// Created by 34969 on 25-12-26.
//

#ifndef SBUS_T_H
#define SBUS_T_H
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "steering.h"  // 机械鱼运动控制头文件

/************************ 预处理命令-芯片版本选择 ************************/
#define SBUS_H_Vision 7  // 根据实际芯片修改：1=F1,4=F4,7=H7
#if   (SBUS_H_Vision==1)
#include "stm32f1xx_hal.h"
#elif (SBUS_H_Vision==4)
#include "stm32f4xx_hal.h"
#elif (SBUS_H_Vision==7)
#include "stm32h7xx_hal.h"
#endif

/************************ 调试模式开关 ************************/
#define SBUS_DEBUG_MODE 1  // 1=开启调试，0=关闭

/************************ OTTOHELS文件包含管理 ************************/
#define SBUS_OTTOHELS 1
#if SBUS_OTTOHELS
#include "ottohesl.h"  // 若没有该文件，可注释并替换为自定义调试打印函数
#endif

/************************ SBUS协议常量 ************************/
#define SBUS_PACKET_LENGTH    25      // SBUS单帧长度（字节）
#define SBUS_STARTBYTE        0x0F    // 帧起始字节
#define SBUS_ENDBYTE          0x00    // 帧结束字节
#define SBUS_CHANNEL_COUNT    16      // SBUS通道数
#define SBUS_DMA_RX_SIZE      256     // DMA接收缓冲区大小（环形）
#define SBUS_FRAME_TIMEOUT    100     // 帧解析超时阈值（ms）
#define SBUS_FAILSAFE_TIMEOUT 100     // 通信超时阈值（ms）

/************************ SBUS控制参数 ************************/
#define sbus_ch3_max          1208
#define sbus_ch3_center       540
#define speed_max             10
#define speed_min             2

/************************ 枚举定义 ************************/
// SBUS帧解析状态机
typedef enum SBUS_FrameState {
    SBUS_SEEK_START = 0,    // 寻找帧起始字节
    SBUS_RECEIVE_DATA = 1,  // 接收帧数据
} SBUS_FrameState;

// SBUS命令映射（兼容原有机械鱼指令）
typedef enum {
    SBUS_CMD_FORWARD = CMD_FORWARD,      // 前进
    SBUS_CMD_TURN_LEFT = CMD_TURN_LEFT,  // 左转
    SBUS_CMD_TURN_RIGHT = CMD_TURN_RIGHT,// 右转
    SBUS_CMD_STOP = 0x04                 // 停止
} SBUS_Command_t;

/************************ 结构体定义 ************************/
// SBUS核心数据结构体
typedef struct {
    uint16_t channels[SBUS_CHANNEL_COUNT];  // 16通道原始值（0-2047）
    uint8_t flags;                          // 标志位（failsafe/frame_lost）
    uint8_t failsafe;                       // 1=失联，0=正常
    uint8_t frame_lost;                     // 1=丢帧，0=正常
    uint8_t new_data_available;             // 1=有新数据，0=无
    uint32_t last_update_time;              // 最后数据更新时间（ms）
    uint8_t raw_data[SBUS_PACKET_LENGTH];   // 原始帧数据
} SBUS_Data_t;

/************************ 函数声明 ************************/
// 初始化函数
void SBUS_Init(UART_HandleTypeDef *h_sbus, UART_HandleTypeDef *h_debug);
// 数据处理（解析DMA缓冲区+执行命令）
bool SBUS_Process(void);
// 私有函数（内部调用）
static void SBUS_DecodePacket(uint8_t *packet);
static SBUS_Command_t SBUS_GetCommand(void);
static void SBUS_ExecuteCommand(void);
static int mapValue(int value);

/************************ 全局变量声明 ************************/
extern SBUS_Data_t sbus_data;
extern UART_HandleTypeDef *sbus_huart;
extern UART_HandleTypeDef *sbus_debug_huart;

#endif //SBUS_T_H
