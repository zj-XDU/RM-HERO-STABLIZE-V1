//#include "variable.h"
#include "usart_task.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "usart.h"
/*************************外部变量定义********************* */


struct SAcc
{
	short a[3];
	short T;
};
struct SGyro
{
	short w[3];
	short T;
};
struct SAngle
{
	short Angle[3];
	short T;
};

struct bno055_euler_float_t
{
    float h; /**< Euler h float data */
    float r; /**< Euler r float data */
    float p; /**< Euler p float data */
};
struct bno055_gyro_float_t
{
    float x; /**< Gyro x float data */
    float y; /**< Gyro y float data */
    float z; /**< Gyro z float data */
};

struct shoot_data
{
	float pit_set;
	float yaw_set;
	uint8_t shoot_en;

};



struct bno055_euler_float_t euler_hpr;
struct bno055_gyro_float_t gyro_xyz;
struct shoot_data my_shoot_data;

unsigned char ucRxBuffer[250];
unsigned char ucRxBuffer_shoot[250];

unsigned char ucRxCnt = 0;
unsigned char ucRxCnt_shoot = 0;


uint8_t RxData[100];
uint8_t RxData_shoot[30];

struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 		stcAngle;

uint8_t can_send_data_gyro[8];
uint8_t can_send_data_angle[8];


void CopeSerial2Data()
{

		//LED2_TOGGLE;					//接收到数据，LED灯闪烁一下
		ucRxBuffer[ucRxCnt++]=(unsigned char)UART4->DR;	//将收到的数据存入缓冲区中
		
		
			   
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;

		}
		ucRxCnt=0;//清空缓存区
	}
	
	
	memcpy(can_send_data_gyro,&stcGyro,8);
	memcpy(can_send_data_angle,&stcAngle,8);
}



//视觉自瞄数据解包
void shoot_data_update()
{

	ucRxBuffer_shoot[ucRxCnt_shoot++]=(unsigned char)USART3->DR;	//将收到的数据存入缓冲区中
		
		
			   
	if (ucRxBuffer_shoot[0]!=0x88) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt_shoot=0;
		return;
	}
	if (ucRxCnt_shoot<10) {return;}//数据不满10个，则返回
	else
	{
		
		memcpy(&my_shoot_data.pit_set,&ucRxBuffer_shoot[1],4);
		memcpy(&my_shoot_data.yaw_set,&ucRxBuffer_shoot[5],4);
		memcpy(&my_shoot_data.shoot_en,&ucRxBuffer_shoot[9],1);
	}

}






uint32_t usart_now_time=0;
uint32_t usart_last_time=0;
uint32_t usart_time=0;
uint8_t test[4]={1,2,3,4};

//任务之一————usart_task

void usart_task(void const * argument)
{
  portTickType hero_gimbal_usart_task_pre_tick = 0;
	HAL_UART_Receive_IT(&huart4,RxData,1);
	HAL_UART_Receive_IT(&huart3,RxData_shoot,1);
  while(1)
  {
  
	usart_now_time=HAL_GetTick();
	

	osDelay(1);
        
	
	usart_time = usart_now_time - usart_last_time;
	usart_last_time = usart_now_time;
		
    osDelayUntil(&hero_gimbal_usart_task_pre_tick,2);		//该任务的执行周期是3ms
  }
  
}



uint8_t angle[12];
uint8_t gyro[12];

int usrt_data_count=0;
uint8_t flag=0;
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  
  
	//detect_hook(INS_TOE);
	if(UART4->CR1 & 20)
	{
		CopeSerial2Data();	
		
		
		usrt_data_count++;
		euler_hpr.h = (float)stcAngle.Angle[2]/32768*3.1415926f;		//YAW	
		euler_hpr.r = (float)stcAngle.Angle[0]/32768*3.1415926f;		//ROL
		euler_hpr.p = (float)stcAngle.Angle[1]/32768*3.1415926f;		//PIT
		
		gyro_xyz.x = (float)stcGyro.w[0]/32768*2000/360;			//ROL
		gyro_xyz.y = (float)stcGyro.w[1]/32768*2000/360;			//PIT
		gyro_xyz.z = (float)stcGyro.w[2]/32768*2000/360;			//YAW
		
		
		
		memcpy(angle,&euler_hpr,12);
		memcpy(gyro,&gyro_xyz,12);
		
		
	}

	
	

  HAL_UART_IRQHandler(&huart4);
  HAL_UART_Receive_IT(&huart4,RxData,1);
  /* USER CODE BEGIN UART2_IRQn 1 */

  /* USER CODE END UART2_IRQn 1 */
}


void USART3_IRQHandler(void)
{

	if(USART3->CR1 & 20)
	{
		shoot_data_update();
	}

	 HAL_UART_IRQHandler(&huart3);
	 HAL_UART_Receive_IT(&huart3,RxData_shoot,1);



}










