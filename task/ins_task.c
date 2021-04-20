#include "ins_task.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "detect_task.h"
#include "stm32f4xx_it.h"
#include "cmsis_os.h"

uint8_t RxData[100];
uint8_t RxData_buff[250];
unsigned char ucRxBuffer[250];
unsigned char ucRxCnt = 0;

struct bno055_euler_float_t euler_hpr;
struct bno055_gyro_float_t gyro_xyz;
struct SAcc 		stcAcc;
extern struct SGyro 		stcGyro;
extern struct SAngle 		stcAngle;




/**
  * @brief          get the euler angle, 0:yaw, 1:pitch, 2:roll unit rad
  * @param[in]      none
  * @retval         the point of INS_angle
  */
/**
  * @brief          获取欧拉角, 单位 rad
  * @param[in]      none
  * @retval         euler_hpr的指针
  */
const struct bno055_euler_float_t *get_INS_angle_point(void)
{
    return &euler_hpr;
}

/**
  * @brief          get the rotation speed, 0:x-axis, 1:y-axis, 2:roll-axis,unit rad/s
  * @param[in]      none
  * @retval         the point of INS_gyro
  */
/**
  * @brief          获取角速度， 单位 rad/s
  * @param[in]      none
  * @retval         gyro_xyz的指针
  */
const struct bno055_gyro_float_t *get_gyro_data_point(void)
{
    return &gyro_xyz;
}








//获取陀螺仪的角度和角速度数据
void imu_task(void const *pvParameters)
{
	
	portTickType ins_task_pre_tick = 0;
	while(1)
	{



		euler_hpr.h = (fp32)stcAngle.Angle[2] * 3.1415926f / (fp32)32768;		//YAW	
		euler_hpr.r = (fp32)stcAngle.Angle[0] * 3.1415926f / (fp32)32768;		//ROL
		euler_hpr.p = (fp32)stcAngle.Angle[1] * 3.1415926f / (fp32)32768;		//PIT
		
		gyro_xyz.x = (fp32)stcGyro.w[0]/32768*250*0.0174;///360;			//ROL
		gyro_xyz.y = (fp32)stcGyro.w[1]/32768*250*0.0174;//360;			//PIT
		gyro_xyz.z = (fp32)stcGyro.w[2]/32768*250*0.0174;//360;			//YAW
		
		
	
		osDelayUntil(&ins_task_pre_tick,1);									//执行频率为1ms
	
	}



}










