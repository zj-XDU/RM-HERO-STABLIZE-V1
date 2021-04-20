/*************************该文件包含云台CAN通信的相关功能函数*********************/



#ifndef _CAN_TASK_H_
#define _CAN_TASK_H_



void hero_gimbal_shoot_can_receive();       //云台摩擦轮电机数据接收
void hero_gimbal_pitch_can_receive();       //云台pitch轴电机数据接收

void hero_gimbal_can_send();                //云台摩擦轮、pitch轴电机电流发送

void can_task(void const * argument);       //can_task的入口函数








#endif