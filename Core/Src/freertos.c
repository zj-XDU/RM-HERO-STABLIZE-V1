/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
osThreadId dbugHandle;
osThreadId gimbalHandle;
osThreadId chassisHandle;
osThreadId detectHandle;
osThreadId caliHandle;
osThreadId judgeHandle;
osThreadId pidHandle;
osThreadId shootHandle;
osThreadId imuHandle;
osThreadId idelHandle;
osThreadId canHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void dbug_task(void const * argument);
extern void gimbal_task(void const * argument);
extern void chassis_task(void const * argument);
extern void detect_task(void const * argument);
extern void cali_task(void const * argument);
extern void judge_task(void const * argument);
extern void pid_task(void const * argument);
extern void shoot_task(void const * argument);
extern void imu_task(void const * argument);
void idel_task(void const * argument);
extern void can_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of dbug */
  osThreadDef(dbug, dbug_task, osPriorityBelowNormal, 0, 128);
  dbugHandle = osThreadCreate(osThread(dbug), NULL);

  /* definition and creation of gimbal */
  osThreadDef(gimbal, gimbal_task, osPriorityAboveNormal, 0, 512);
  gimbalHandle = osThreadCreate(osThread(gimbal), NULL);

  /* definition and creation of chassis */
  osThreadDef(chassis, chassis_task, osPriorityNormal, 0, 512);
  chassisHandle = osThreadCreate(osThread(chassis), NULL);

  /* definition and creation of detect */
  //osThreadDef(detect, detect_task, osPriorityBelowNormal, 0, 256);
  //detectHandle = osThreadCreate(osThread(detect), NULL);

  /* definition and creation of cali */
  //osThreadDef(cali, cali_task, osPriorityBelowNormal, 0, 512);
  //caliHandle = osThreadCreate(osThread(cali), NULL);

  /* definition and creation of judge */
  osThreadDef(judge, judge_task, osPriorityNormal, 0, 256);
  judgeHandle = osThreadCreate(osThread(judge), NULL);

  /* definition and creation of pid */
  osThreadDef(pid, pid_task, osPriorityHigh, 0, 256);
  pidHandle = osThreadCreate(osThread(pid), NULL);

  /* definition and creation of shoot */
  osThreadDef(shoot, shoot_task, osPriorityNormal, 0, 128);
  shootHandle = osThreadCreate(osThread(shoot), NULL);

  /* definition and creation of imu */
  osThreadDef(imu, imu_task, osPriorityAboveNormal, 0, 128);
  imuHandle = osThreadCreate(osThread(imu), NULL);

  /* definition and creation of idel */
  osThreadDef(idel, idel_task, osPriorityIdle, 0, 128);
  idelHandle = osThreadCreate(osThread(idel), NULL);

  /* definition and creation of can */
  osThreadDef(can, can_task, osPriorityHigh, 0, 256);
  canHandle = osThreadCreate(osThread(can), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_dbug_task */
/**
  * @brief  Function implementing the dbug thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_dbug_task */
__weak void dbug_task(void const * argument)
{
  /* USER CODE BEGIN dbug_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END dbug_task */
}

/* USER CODE BEGIN Header_idel_task */
/**
* @brief Function implementing the idel thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_idel_task */
void idel_task(void const * argument)
{
  /* USER CODE BEGIN idel_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END idel_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
