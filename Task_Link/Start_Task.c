#include "cmsis_os.h"
#include "JY901S.h"
#include "cmsis_os2.h"
#include "main.h"
#include "ottohesl.h"
#include <stdio.h>
#include"SBUS_T.h"
#include "Start_Task.h"
void SBUS_Recevie(void *argument) {
    SBUS_Command_t *Command;
    for(;;)
    {
        if(SBUS_Process()==sbus_data.new_data_available){
            osMessageQueuePut( SBUSHandle, &Command, 0,osWaitForever);
        }
        osDelay(1);
    }
}
void GPS_Receive(void *argument) {
    for(;;)
    {
        osDelay(1);
    }
}
void JY901S_Receive(void *argument){
    jy901 *gyro = &gyro_data;
    for(;;)
    {
        
        if (Gyroscope_Process()) {
            //ottohesl_uart(&huart3,"%f,%f,%f",gyro_data.gyroscope.angle[0],gyro_data.gyroscope.angle[1],gyro_data.gyroscope.angle[2]);
            osMessageQueuePut(JY901SHandle,&gyro, 0, 0);
        }
        osDelay(10);
    }
}
void Start_Control(void *argument)
{
    jy901 *gyro;
    SBUS_Command_t *cmd;
    for(;;)
    {
        if (osMessageQueueGet(JY901SHandle,&gyro,0,osWaitForever)==osOK) {
            //开始处理姿态
            //ottohesl_uart(&huart_debug,"开始处理文件！");
            
            ottohesl_uart(&huart_debug,"%f,%f,%f",gyro->gyroscope.angle[0],gyro->gyroscope.angle[1],gyro->gyroscope.angle[2]);
        }
        if(osMessageQueueGet( SBUSHandle, &cmd, 0,osWaitForever) ==osOK){
            //开始处理SBUS命令
            Fish_ExecuteCommand();
            //ottohesl_uart(&huart_debug,"收到SBUS命令！");

        }
        osDelay(1);
    }

}
