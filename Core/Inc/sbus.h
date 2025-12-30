#ifndef __SBUS_H__
#define __SBUS_H__

#include "main.h"
#include "usart.h"
#include "steering.h"  // 使用您现有的机械鱼运动控制

// SBUS协议常量
#define SBUS_PACKET_LENGTH      25
#define SBUS_STARTBYTE          0x0F
#define SBUS_ENDBYTE            0x00
#define SBUS_CHANNEL_COUNT      16

#define sbus_ch3_max            1208
#define sbus_ch3_center         540
#define speed_max               10
#define speed_min               2
// SBUS命令映射到机械鱼状态
typedef enum {
    SBUS_CMD_FORWARD = CMD_FORWARD,      // 映射到您的CMD_FORWARD
    SBUS_CMD_TURN_LEFT = CMD_TURN_LEFT,  // 映射到您的CMD_TURN_LEFT
    SBUS_CMD_TURN_RIGHT = CMD_TURN_RIGHT,// 映射到您的CMD_TURN_RIGHT
    SBUS_CMD_STOP = 0x04                 // 停止命令
} SBUS_Command_t;

// SBUS数据结构
typedef struct {
    uint8_t raw_data[SBUS_PACKET_LENGTH];
    uint16_t channels[SBUS_CHANNEL_COUNT];
    uint8_t flags;
    uint8_t failsafe;
    uint8_t frame_lost;
    uint8_t new_data_available;
    uint32_t last_update_time;
} SBUS_Data_t;

// 函数声明
void SBUS_Control_Init(UART_HandleTypeDef *huart);
void SBUS_Control_Process(void);
uint8_t SBUS_IsNewCommandAvailable(void);
void SBUS_ExecuteCommand(void);
void SBUS_RxCpltCallback(UART_HandleTypeDef *huart);
void SBUS_DecodePacket(uint8_t *packet);
SBUS_Command_t SBUS_GetCommand(void);

extern SBUS_Data_t sbus_data;
extern UART_HandleTypeDef *sbus_huart;

#endif