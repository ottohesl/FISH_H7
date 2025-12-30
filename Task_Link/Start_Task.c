#include "cmsis_os.h"
#include "JY901S.h"
#include "main.h"
#include "ottohesl.h"
#include <stdio.h>
#include "Start_Task.h"
void SUBS_Recevie(void *argument) {
    for(;;)
    {
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
    for(;;)
    {
        if (osMessageQueueGet(JY901SHandle,&gyro,0,osWaitForever)==osOK) {
            //开始处理姿态
            //ottohesl_uart(&huart_debug,"开始处理文件！");
            ottohesl_uart(&huart_debug,"%f,%f,%f",gyro->gyroscope.angle[0],gyro->gyroscope.angle[1],gyro->gyroscope.angle[2]);
        }

        osDelay(1);
    }

}
