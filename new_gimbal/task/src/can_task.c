//#include "pid.h"
#include "can_task.h"
//#include "variable.h"
#include "CAN_receive.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32f4xx.h"
#include "bsp_can.h"
#include "can.h"

extern unsigned char ucRxBuffer_shoot[250];
/*********************************************
 * 描述：云台摩擦轮电机数据接收
 * 输入：无
 * 输出：无
 * 用到的全局变量和宏定义：无
 *********************************************/

void hero_gimbal_shoot_can_receive()
{
    //RE_DATA_shoot1 = *get_chassis_motor_measure_point(1);       //读取摩擦轮1电机数据
	//osDelay(1);
    //RE_DATA_shoot2 = *get_chassis_motor_measure_point(2);       //读取摩擦轮2电机数据
	//osDelay(1);
}



/*********************************************
 * 描述：云台pitch轴电机数据接收
 * 输入：无
 * 输出：无
 * 用到的全局变量和宏定义：无
 *********************************************/

void hero_gimbal_pitch_can_receive()
{
    //RE_DATA_pitch  = *get_yaw_gimbal_motor_measure_point();       //读取pitch轴电机数据
}



/*********************************************
 * 描述：云台摩擦轮电机电流发送
 * 输入：无
 * 输出：无
 * 用到的全局变量和宏定义：无
 *********************************************/

void hero_gimbal_can_send()
{
  CAN_cmd_chassis(0,1000,1000,0); 
 // CAN_cmd_gimbal(current_pitch, 0, 0, 0);
  
   // CAN_cmd_gimbal(300, 0, 0, 0);

}


extern uint8_t angle[12];
extern uint8_t gyro[12];

uint32_t can_now_time=0;
uint32_t can_last_time=0;
uint32_t can_time=0;

extern uint8_t can_send_data_gyro[8];
extern uint8_t can_send_data_angle[8];
uint8_t test_data[10]={1,2,3,4,5,6,7,8,9,10};
//can_task的入口函数
void can_task(void const * argument)
{
  portTickType hero_gimbal_can_task_pre_tick = 0;
  can_filter_init();
  while(1)
  {
    can_now_time=HAL_GetTick();
	

	CAN_MCU_send_gyro(can_send_data_gyro,CAN_MCU_GYRO_ID);
	//osDelay(1);
	CAN_MCU_send_gyro(can_send_data_angle,CAN_MCU_ANGLE_ID);
	osDelay(2);
	//shoot_en
	CAN_MCU_send_shoot_en(test_data[9]);
	
	//pit and yaw
	//CAN_MCU_send_shoot(ucRxBuffer_shoot);
	CAN_MCU_send_shoot(test_data);
	osDelay(2);
	
	can_time = can_now_time - can_last_time;
	can_last_time = can_now_time;
	
    //osDelayUntil(&hero_gimbal_can_task_pre_tick,7);     //执行周期是1ms
  }
  
}













