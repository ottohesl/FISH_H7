//
// Created by ypxl on 2025/11/11.
//

#ifndef FLSH_STM_STEERING_H
#define FLSH_STM_STEERING_H

#include "main.h"
#include "tim.h"


typedef enum {
    STATE_STOP,
    STATE_FORWARD,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT,
    STATE_RETURN_CENTER  // 新增：回中值状态
} FishState_t;

typedef enum {
    CMD_STOP = 0x00,    // 停止命令
    CMD_FORWARD = 0x01,    // 前进命令
    CMD_TURN_LEFT = 0x02,  // 左转命令
    CMD_TURN_RIGHT = 0x03,  // 右转命令
} Command_t;

extern  Command_t current_command;
extern FishState_t current_state ;
extern uint32_t state_start_time ;
extern float recovery_progress;  // 新增：恢复进度

extern uint32_t turn_cycle_count;  // 新增：记录左转摆动周期计数
extern uint16_t servo_angle_tail;
extern uint16_t servo_angle_body;
// 新增：中值平滑过渡相关变量
extern float current_body_median;    // 当前身体中值
extern float current_tail_median;    // 当前尾巴中值
extern float target_body_median;     // 目标身体中值
extern float target_tail_median;     // 目标尾巴中值

extern  uint8_t speed;

void Set_Servo_Angle(TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t angle);
void Fish_Stop(void);
void Fish_Forward(void);

void Fish_TurnLeft_Prepare(void);
void Fish_TurnLeft_Swing(void);
void Fish_TurnRight_Prepare(void);// 右转准备函数
void Fish_TurnRight_Swing(void);// 右转摆动函数


void Fish_StateMachine(void);

// 新增命令响应函数
void Fish_ExecuteCommand(Command_t cmd);

#endif //FLSH_STM_STEERING_H