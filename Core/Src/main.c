/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "remote_control.h"
#include "bsp_can.h"
#include "ins_task.h"
#include "calibrate_task.h"
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "shoot_task.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const RC_ctrl_t *local_rc_ctrl;
const motor_measure_t * yaw;
const motor_measure_t * pitch;
const motor_measure_t * shoot1;
const motor_measure_t * shoot2;
const motor_measure_t * trigle;
uint8_t pcWriteBuffer[200];
volatile uint32_t ulHighFrequencyTimerTicks;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
   HAL_Delay(500);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	can_filter_init();
	remote_control_init();
	cali_param_init();
	//local_rc_ctrl = get_remote_control_point();
	yaw = get_yaw_gimbal_motor_measure_point();
	//pitch = get_pitch_gimbal_motor_measure_point();
	//shoot1 = get_shoot_motor_measure_point(0);
	//shoot2 = get_shoot_motor_measure_point(1);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);
	//trigle = get_yaw_gimbal_motor_measure_point();
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_UART_Receive_IT(&huart4,RxData,1);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void dbug_task(void const * argument)
{
	uint32_t xLastWakeTime;
	xLastWakeTime = osKernelSysTick();
	while(1)
	{
	
		//printf("name            count              rate\r\n");
		//U1printf("name            count              rate\r\n");
		//vTaskGetRunTimeStats((char *)&pcWriteBuffer);
		//U1printf("%s\r\n", pcWriteBuffer);
		//printf("pit_kp         %f\r\n",gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.kp);
		//printf("                    pit_ki         %f\r\n",gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.ki);
		//printf("                    pit_kd         %f\r\n",gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.kd);
		//printf("                    pit_angl_out         %f\r\n",gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.out);
		//printf("                    pit_angl_iout         %f\r\n",gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.Iout);
		//printf("                    pit_gyro_out         %f\r\n",gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid.out);
		//printf("                    pit_angle          %d\r\n",gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd);
	
	
	
		//printf("yaw_kp         %f\r\n",gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.kp);
		//printf("                    yaw_ki         %f\r\n",gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.ki);
		//printf("                    yaw_kd         %f\r\n",gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.kd);
		
		//printf("		************************************************************************************         \r\n");
		//printf("		      yaw_angl_out         %f\r\n",					   gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.out);
		//printf("                    yaw_angl_iout          %f\r\n",gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.Iout);
		//printf("                    yaw_gyro_out           %f\r\n",gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_gyro_pid.out);
		//printf("                    yaw_current_set        %d\r\n",(int)gimbal_control.yaw_current_set);
		//printf("                    yaw_angle=     %f\r\n",gimbal_control.gimbal_yaw_motor.absolute_angle);
		//printf("		      yaw_angle_set=  %f\r\n",gimbal_control.gimbal_yaw_motor.absolute_angle_set);
		//printf("		      yaw_angle_err=         %f\r\n",gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.err);
		
		//printf("		      can_time=         %d\r\n",gimbal_control.time);
		printf("set=%f\r\n",shoot_control.set_angle);
	    printf("data=%f\r\n",shoot_control.angle);
		
		//printf("                    yaw_temp               %d\r\n",(int)gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->temperate);
		
		//printf("                    can_yaw_speed          %d\r\n",(int)gimbal_control.can_yaw_speed);

		osDelayUntil(&xLastWakeTime,1);
	}

	

}


void configureTimerForRunTimeStats(void)
{
	ulHighFrequencyTimerTicks = 0ul;
}
unsigned long getRunTimeCounterValue(void)
{
	return ulHighFrequencyTimerTicks;
}
/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else if(__HAL_TIM_GET_IT_SOURCE(&htim6,TIM_IT_UPDATE)!=RESET)
  {
    ulHighFrequencyTimerTicks++;
    __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
