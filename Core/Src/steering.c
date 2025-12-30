
#include "steering.h"
#include <math.h>
#include <stdio.h>

// 舵机脉冲宽度范围
#define SERVO_MIN_PULSE 50
#define SERVO_MAX_PULSE 250
// 舵机角度变量
uint16_t servo_angle_tail = 90;  // 鱼尾舵机角度
uint16_t servo_angle_body = 97;  // 鱼身舵机角度
// 运动参数
uint16_t swing_amplitude = 30;   // 摆动幅度
uint16_t swing_speed = 10;       // 摆动速度
uint32_t swing_counter = 0;      // 摆动计数器

// 转向参数
uint16_t turn_body_bias = 20;        // 身体转向偏置角度（左转为正，右转为负）
uint16_t turn_tail_bias = 10;        // 尾巴转向偏置角度（左转为正，右转为负）

FishState_t current_state = STATE_STOP;
Command_t current_command = CMD_STOP;  // 添加这行定义
uint32_t state_start_time = 0;

static float prepare_progress = 0.0f;
static uint32_t diff=0;
//标志位，记录转向状态
static uint32_t turn_start_counter = 0;
static uint32_t turn_swing_start_time = 0; // 修正：添加摆动阶段开始时间变量
// 标志是否处于左转准备阶段
static uint8_t is_turn_prepare_phase = 0;

static uint8_t is_turn_done = 0;
// 状态控制
const uint32_t STATE_DURATION = 3575; // 5秒

uint8_t speed=2;



// 新增：回中值状态变量
static uint32_t return_start_time = 0;
const uint32_t RETURN_DURATION = 500;  // 回中值过渡时间（500ms）
static uint16_t return_start_body_angle = 97;  // 回中值开始时的身体角度
static uint16_t return_start_tail_angle = 90;  // 回中值开始时的尾巴角度


// 记录身体历史角度，用于尾巴滞后
static uint16_t body_angle_history[10] = {97}; // 存储最近10个身体角度
static uint8_t history_index = 0;

// 新增：准备阶段进度控制

static uint32_t prepare_start_time = 0;
const uint32_t PREPARE_DURATION = 700; // 准备阶段持续时间

// 新增：保存前进时的最后角度
static uint16_t forward_final_body_angle = 97;
static uint16_t forward_final_tail_angle = 90;

// 转向执行标志
static uint8_t turn_executed = 0;
static Command_t pending_command = CMD_FORWARD;

void Set_Servo_Angle(TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t angle)
{
    uint16_t pulse_width;

    // 限制角度范围
    if (angle > 180) angle = 180;

    // 将角度转换为脉冲宽度 (线性映射)
    pulse_width = SERVO_MIN_PULSE + (angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / 180;

    // 设置PWM占空比
    __HAL_TIM_SET_COMPARE(htim, Channel, pulse_width);
}

void Set_Swing_Amplitude(uint16_t amplitude)
{
    swing_amplitude = amplitude;
}

void Set_Swing_Speed(uint16_t speed)
{
    swing_speed = speed;
}

void Set_Turn_Parameters(uint16_t body_bias, uint16_t tail_bias)
{
    turn_body_bias = body_bias;
    turn_tail_bias = tail_bias;
}


void Fish_Stop(void)
{
   // printf("机械鱼停止\n");
    // 设置舵机到中位位置
    servo_angle_body = 97;
    servo_angle_tail = 90;

    Set_Servo_Angle(&htim2, TIM_CHANNEL_2, servo_angle_tail);
    Set_Servo_Angle(&htim3, TIM_CHANNEL_1, servo_angle_body);

    // 重置摆动计数器
    swing_counter = 0;
    is_turn_prepare_phase=1;
}

void Fish_Forward(void)
{
    printf("机械鱼前进\n");
    // 计算正弦波角度
    float radian = swing_counter * 0.01f;

    // 鱼身舵机：较小的幅度，基础相位
    servo_angle_body = 97 + swing_amplitude * 0.6f * sinf(radian);

    // 鱼尾舵机：较大的幅度，滞后相位（身体先动，尾巴后动）
    servo_angle_tail = 90 + swing_amplitude * 0.8f * sinf(radian - 1.57f);

    // 限制角度范围
    if (servo_angle_body > 180) servo_angle_body = 180;
    if (servo_angle_body < 0) servo_angle_body = 0;
    if (servo_angle_tail > 180) servo_angle_tail = 180;
   if (servo_angle_tail < 0) servo_angle_tail = 0;

    // 设置舵机角度
    Set_Servo_Angle(&htim2, TIM_CHANNEL_2, servo_angle_tail);  // 鱼尾舵机
    Set_Servo_Angle(&htim3, TIM_CHANNEL_1, servo_angle_body);  // 鱼身舵机

    // 更新计数器
    swing_counter+=speed;
    if (swing_counter > 628) {
        swing_counter = 0;
    }

    // 准备转向标志位
    is_turn_prepare_phase=1;
}

void Fish_TurnLeft_Prepare(void)
{

    // 计算准备进度
    printf("机械鱼准备右转\n");
    uint32_t current_time = HAL_GetTick();
    prepare_progress = (float)(current_time - prepare_start_time) / PREPARE_DURATION;
    if (prepare_progress > 1.0f) prepare_progress = 1.0f;

    // 修改：使用缓动函数（ease-in-out）使过渡更平滑
    float ease_progress;
    if (prepare_progress < 0.5f) {
        ease_progress = 2.0f * prepare_progress * prepare_progress;
    } else {
        ease_progress = 1.0f - 2.0f * (1.0f - prepare_progress) * (1.0f - prepare_progress);
    }

    // 左转目标角度
    float body_bias = 20.0f;
    float tail_bias = 15.0f;
    float body_amplitude = swing_amplitude * 0.3f;
    float tail_amplitude = body_amplitude * 1.5f;

    float target_body_angle = 97 - body_bias - body_amplitude; // 左转最左侧角度
    float target_tail_angle = 90 - tail_bias - tail_amplitude; // 左转最左侧角度


    servo_angle_body = forward_final_body_angle + (target_body_angle - forward_final_body_angle) * ease_progress;
    servo_angle_tail = forward_final_tail_angle + (target_tail_angle - forward_final_tail_angle) * ease_progress;

    // 限制角度范围
    if (servo_angle_body > 180) servo_angle_body = 180;
    if (servo_angle_body < 0) servo_angle_body = 0;
    if (servo_angle_tail > 180) servo_angle_tail = 180;
    if (servo_angle_tail < 0) servo_angle_tail = 0;

    // 设置舵机角度
    Set_Servo_Angle(&htim2, TIM_CHANNEL_2, servo_angle_tail);
    Set_Servo_Angle(&htim3, TIM_CHANNEL_1, servo_angle_body);

    // 保存身体角度历史
    body_angle_history[history_index] = servo_angle_body;
    history_index = (history_index + 1) % 10;

    // 修改：减少延迟时间，使移动更流畅
    HAL_Delay(5);
}

void Fish_TurnRight_Prepare(void)
{
    printf("机械鱼准备左转\n");

    uint32_t current_time = HAL_GetTick();
    prepare_progress = (float)(current_time - prepare_start_time) / PREPARE_DURATION;
    if (prepare_progress > 1.0f) prepare_progress = 1.0f;

    float ease_progress;
    if (prepare_progress < 0.5f) {
        ease_progress = 2.0f * prepare_progress * prepare_progress;
    } else {
        ease_progress = 1.0f - 2.0f * (1.0f - prepare_progress) * (1.0f - prepare_progress);
    }

    float body_bias = 20.0f;
    float tail_bias = 15.0f;
    float body_amplitude = swing_amplitude * 0.3f;
    float tail_amplitude = body_amplitude * 1.5f;

    // 右转目标角度（向右偏）
    float target_body_angle = 97 + body_bias + body_amplitude;
    float target_tail_angle = 90 + tail_bias + tail_amplitude;


    // 修改：从前进时的最后角度平滑过渡到左转目标角度
     servo_angle_body = forward_final_body_angle + (target_body_angle - forward_final_body_angle) * ease_progress;
    servo_angle_tail = forward_final_tail_angle + (target_tail_angle - forward_final_tail_angle) * ease_progress;


    if (servo_angle_body > 180) servo_angle_body = 180;
    if (servo_angle_body < 0) servo_angle_body = 0;
    if (servo_angle_tail > 180) servo_angle_tail = 180;
    if (servo_angle_tail < 0) servo_angle_tail = 0;

    Set_Servo_Angle(&htim2, TIM_CHANNEL_2, servo_angle_tail);
    Set_Servo_Angle(&htim3, TIM_CHANNEL_1, servo_angle_body);

    body_angle_history[history_index] = servo_angle_body;
    history_index = (history_index + 1) % 10;
    HAL_Delay(5);
}



void Fish_TurnLeft_Swing(void)
{
    printf("机械鱼右转\n");
    uint32_t current_time = HAL_GetTick();


    // 使用更快的正弦波角度（快速摆动）
    float radian = swing_counter * 0.01f;  // 进一步增加系数使摆动更快

    // 左转参数 - 使用准备阶段的数据
    float body_bias = 20.0f;
    float tail_bias = 15.0f;

    // 修改：直接使用准备阶段结束时的角度作为起始点
    // 计算从准备阶段结束角度到摆动目标角度的过渡

    float transition_progress = (float)(current_time - turn_swing_start_time) / 200.0f; // 减少过渡时间到200ms
    if (transition_progress > 1.0f) transition_progress = 1.0f;

    // 修改：使用线性过渡，避免缓动函数的初始缓慢
    float ease_transition = transition_progress; // 线性过渡

    // 鱼身舵机：从准备阶段角度平滑过渡到摆动角度
    float body_sin = sinf(radian);
    float body_amplitude = swing_amplitude * 0.5f;    //身体摆动的最大角度偏移量,控制身体左右摆动的范围大小
    float body_offset = 97 - body_bias; // 左转中值

    // 修改：计算目标摆动角度
    float target_body_angle = body_offset + body_amplitude * body_sin;    //基准位置 + 摆动幅度 × 正弦值

    // 修改：从当前角度平滑过渡到目标摆动角度
    servo_angle_body = servo_angle_body + (target_body_angle - servo_angle_body) * ease_transition;  //为了平滑

    // 保存身体角度历史
    body_angle_history[history_index] = servo_angle_body;
    history_index = (history_index + 1) % 10;

    // 修改：直接计算尾巴角度，不使用历史角度
    float tail_radian = radian; ; // 数字越大相位滞后越大
    float tail_sin = sinf(tail_radian);
    float tail_offset = 90 - tail_bias;
    float tail_amplitude_factor = 1.8f;

    // 修改：计算目标尾巴角度
    float target_tail_angle = tail_offset + (tail_amplitude_factor * body_amplitude) * tail_sin;  //尾巴摆动幅度是身体的1.8倍.身体幅度15度 × 1.8 = 27度

    // 修改：增加尾巴的过渡延迟，使尾巴更滞后
    // 尾巴的过渡进度比身体慢一些
    float tail_transition_progress = transition_progress * 0.6f; // 尾巴过渡速度是身体的80%
    if (tail_transition_progress > 1.0f) tail_transition_progress = 1.0f;

    // // 修改：从当前尾巴角度平滑过渡到目标摆动角度
    // servo_angle_tail = servo_angle_tail + (target_tail_angle - servo_angle_tail) * ease_transition;
    // 从当前尾巴角度平滑过渡到目标摆动角度
    servo_angle_tail = servo_angle_tail + (target_tail_angle - servo_angle_tail) * tail_transition_progress;

    // 限制角度范围
    if (servo_angle_body > 180) servo_angle_body = 180;
    if (servo_angle_body < 0) servo_angle_body = 0;
    if (servo_angle_tail > 180) servo_angle_tail = 180;
    if (servo_angle_tail < 0) servo_angle_tail = 0;

    // 设置舵机角度
    Set_Servo_Angle(&htim3, TIM_CHANNEL_1, servo_angle_body);
    Set_Servo_Angle(&htim2, TIM_CHANNEL_2, servo_angle_tail);

    // 快速更新计数器（使摆动更快）
    static uint32_t last_counter = 0;
    swing_counter += 4;  // 增加步进值，使摆动更快
    printf("左转摆动: 前值=%d, 当前值=%d\n", last_counter, swing_counter);
    last_counter = swing_counter;
    if (swing_counter > 628) {
        swing_counter = 0;
    }

    // 控制舵机运动速度（摆动阶段较快）
    HAL_Delay(1);
}

void Fish_TurnRight_Swing (void)
{
    printf("机械鱼左转\n");
    uint32_t current_time = HAL_GetTick();

    float radian = swing_counter * 0.01f;    //对应就是4.71弧度=3π/2 ≈ 270度 O°  1.57弧度=π/2

    float body_bias = 20.0f;
    float tail_bias = 15.0f;


    float transition_progress = (float)(current_time - turn_swing_start_time) / 200.0f;
    if (transition_progress > 1.0f) transition_progress = 1.0f;

    float ease_transition = transition_progress;

   float body_sin = sinf(radian);   //3π/2 ≈ 270度 -->sin(4.71) = -1（正弦波最低点） radian = 0（sin = 0）  radian = 1.57（π/2，sin = 1）
    float body_amplitude = swing_amplitude * 0.5f;
    float body_offset = 97 + body_bias; // 右转中值（向右偏）


    float target_body_angle = body_offset + body_amplitude * body_sin;

    servo_angle_body = servo_angle_body + (target_body_angle - servo_angle_body) * ease_transition;

    body_angle_history[history_index] = servo_angle_body;
    history_index = (history_index + 1) % 10;

    float tail_radian = radian ;
    float tail_sin = sinf(tail_radian);
    float tail_offset = 90 + tail_bias; // 右转尾巴中值（向右偏）
    float tail_amplitude_factor = 1.8f;


    // 计算目标尾巴角度
    float target_tail_angle = tail_offset + tail_amplitude_factor * body_amplitude * tail_sin;

    // 修改：增加尾巴的过渡延迟，使尾巴更滞后
    // 尾巴的过渡进度比身体慢一些
    float tail_transition_progress = transition_progress * 0.6f; // 尾巴过渡速度是身体的80%
    if (tail_transition_progress > 1.0f) tail_transition_progress = 1.0f;

    // 从当前尾巴角度平滑过渡到目标摆动角度
    servo_angle_tail = servo_angle_tail + (target_tail_angle - servo_angle_tail) * tail_transition_progress;

    if (servo_angle_body > 180) servo_angle_body = 180;
    if (servo_angle_body < 0) servo_angle_body = 0;
    if (servo_angle_tail > 180) servo_angle_tail = 180;
    if (servo_angle_tail < 0) servo_angle_tail = 0;

    Set_Servo_Angle(&htim3, TIM_CHANNEL_1, servo_angle_body);
    Set_Servo_Angle(&htim2, TIM_CHANNEL_2, servo_angle_tail);



    static uint32_t last_counter = 0;
    swing_counter += 4;  // 增加步进值，使摆动更快
    printf("左转摆动: 前值=%d, 当前值=%d\n", last_counter, swing_counter);
    last_counter = swing_counter;

    if (swing_counter > 628)
    {
        printf("重置计数器: %d -> 0\n", swing_counter);
        swing_counter = 0;
    }
    // 增加适当的延迟使运动更平滑
   // HAL_Delay(5);
}

// 执行命令函数
void Fish_ExecuteCommand(Command_t cmd)
{
    uint32_t current_time = HAL_GetTick();
    current_command = cmd;

    switch (cmd)
    {
    case CMD_STOP:
        current_state = STATE_STOP;
        break;

    case CMD_FORWARD:
        current_state = STATE_FORWARD;
        break;

    case CMD_TURN_LEFT:
            // 初始化左转状态
        current_state = STATE_TURN_LEFT;
        printf("左\n");    //有
           break;

    case CMD_TURN_RIGHT:
        current_state = STATE_TURN_RIGHT;
        break;
    }
}


void Fish_StateMachine(void)
{
    uint32_t current_time = HAL_GetTick();


    switch (current_state) {
        case STATE_STOP:
            Fish_Stop();
            break;

        case STATE_FORWARD:
            Fish_Forward();
            break;

        case STATE_TURN_LEFT:
            if (is_turn_prepare_phase==1) {
                Fish_TurnLeft_Prepare();
                if (prepare_progress >= 1.0f) {
                    is_turn_prepare_phase = 2;
                    turn_start_counter = 471;
                    turn_swing_start_time = current_time;
                    swing_counter = 471;
                }
            } if (is_turn_prepare_phase==2) {
                Fish_TurnLeft_Swing();
                uint32_t diff;
                if (swing_counter >= turn_start_counter) {
                    diff = swing_counter - turn_start_counter;
                } else {
                    diff = (628 - turn_start_counter) + swing_counter;
                }

                if (diff >= 314) {
                    // 右转完成，回到前进状态
                    current_state = STATE_STOP;
                    is_turn_prepare_phase=1;
                    turn_executed = 1;
                }
            }
            break;

        case STATE_TURN_RIGHT:

            if (is_turn_prepare_phase==1) {
                Fish_TurnRight_Prepare();
                if (prepare_progress >= 1.0f) {
                    printf("准备结束\n");
                    is_turn_prepare_phase = 2;
                    turn_start_counter = 157;
                    turn_swing_start_time = current_time;
                    swing_counter = 157;
                }
            } if (is_turn_prepare_phase==2) {
                printf("开始左转");
                Fish_TurnRight_Swing();
                uint32_t diff;

                if (swing_counter >= turn_start_counter) {
                    diff = swing_counter - turn_start_counter;
                } else {
                    diff = (628 - turn_start_counter) + swing_counter;
                }

                if (diff >= 314) {
                    // 左转完成，回到前进状态
                    current_state = STATE_FORWARD;
                    is_turn_prepare_phase=1;
                    turn_executed = 1;
                    diff=0;
                }
                printf("diff=%d  标志位=%d\n", diff,is_turn_prepare_phase);
            }
            break;

        default:
            current_state = STATE_STOP;  // 确保默认状态是前进
            break;
    }
}


