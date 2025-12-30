/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "JY901S.h"
#include"SBUS_T.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for SUBS_Task */
osThreadId_t SBUS_TaskHandle;
const osThreadAttr_t SBUS_Task_attributes = {
  .name = "SBUS_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GPS_Task */
osThreadId_t GPS_TaskHandle;
const osThreadAttr_t GPS_Task_attributes = {
  .name = "GPS_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for JY901S_Task */
osThreadId_t JY901S_TaskHandle;
const osThreadAttr_t JY901S_Task_attributes = {
  .name = "JY901S_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Control */
osThreadId_t ControlHandle;
const osThreadAttr_t Control_attributes = {
  .name = "Control",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SBUS */
osMessageQueueId_t SBUSHandle;
const osMessageQueueAttr_t SBUS_attributes = {
  .name = "SBUS"
};
/* Definitions for JY901S */
osMessageQueueId_t JY901SHandle;
const osMessageQueueAttr_t JY901S_attributes = {
  .name = "JY901S"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void SBUS_Recevie(void *argument);
void GPS_Receive(void *argument);
void JY901S_Receive(void *argument);
void Start_Control(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of SUBS */
  SBUSHandle = osMessageQueueNew (16, sizeof(SBUS_Command_t*), &SBUS_attributes);

  /* creation of JY901S */
  JY901SHandle = osMessageQueueNew (16, sizeof(jy901*), &JY901S_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SBUS_Task */
  SBUS_TaskHandle = osThreadNew(SBUS_Recevie, NULL, &SBUS_Task_attributes);

  /* creation of GPS_Task */
  GPS_TaskHandle = osThreadNew(GPS_Receive, NULL, &GPS_Task_attributes);

  /* creation of JY901S_Task */
  JY901S_TaskHandle = osThreadNew(JY901S_Receive, NULL, &JY901S_Task_attributes);

  /* creation of Control */
  ControlHandle = osThreadNew(Start_Control, NULL, &Control_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_SUBS_Recevie */
/**
  * @brief  Function implementing the SUBS_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_SUBS_Recevie */
__weak void SBUS_Recevie(void *argument)
{
  /* USER CODE BEGIN SUBS_Recevie */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END SUBS_Recevie */
}

/* USER CODE BEGIN Header_GPS_Receive */
/**
* @brief Function implementing the GPS_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GPS_Receive */
__weak void GPS_Receive(void *argument)
{
  /* USER CODE BEGIN GPS_Receive */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END GPS_Receive */
}

/* USER CODE BEGIN Header_JY901S_Receive */
/**
* @brief Function implementing the JY901S_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_JY901S_Receive */
__weak void JY901S_Receive(void *argument)
{
  /* USER CODE BEGIN JY901S_Receive */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END JY901S_Receive */
}

/* USER CODE BEGIN Header_Start_Control */
/**
* @brief Function implementing the Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Control */
__weak void Start_Control(void *argument)
{
  /* USER CODE BEGIN Start_Control */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_Control */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

