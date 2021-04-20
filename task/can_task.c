#include "can_task.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "detect_task.h"
#include "shoot_task.h"



void can_task(void const *pvParameters)
{

    portTickType can_task_pre_tick = 0;
	
	
	static int32_t time_now,time_last;
    while(1)
    {

			
		time_now = HAL_GetTick();
/*******************************************底盘任务CAN收发********************************************/
	    //make sure  one motor is online at least, so that the control CAN message can be received
        //确保至少一个电机在线， 这样CAN控制包可以被接收到
        //if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
        //{
            //when remote control is offline, chassis motor should receive zero current. 
            //当遥控器掉线的时候，发送给底盘电机零电流.
          //  if (toe_is_error(DBUS_TOE))//||toe_is_error(INS_TOE))
			//if (toe_is_error(INS_TOE))
            //{
              //  CAN_cmd_chassis(0, 0, 0, 0);
            //}
            //else
            //{
                //send control current
                //发送控制电流
                CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                                chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
            //}
       // }

		//osDelay(1);
/*******************************************云台任务CAN收发********************************************/

       // if (!(toe_is_error(YAW_GIMBAL_MOTOR_TOE) && toe_is_error(PITCH_GIMBAL_MOTOR_TOE) ))//&& toe_is_error(INS_TOE)))
        //{
          //  if (toe_is_error(DBUS_TOE)||toe_is_error(INS_TOE))
            //{
              //  CAN_cmd_gimbal(0, 0, 0, 0);
				//CAN1_cmd_gimbal(0, 0, 0, 0);//
           // }
            //else
            //{//gimbal_control.yaw_current_set
			
			//gimbal_control.yaw_current_set
					 CAN_cmd_gimbal(gimbal_control.yaw_current_set, gimbal_control.pit_current_set, shoot_control.shoot_current[0], shoot_control.shoot_current[1]);
					 CAN1_cmd_gimbal(shoot_control.shoot_current[2], 0, 0, 0);			//gimbal_control.trigle_current_set	
            //}
			gimbal_control.can_yaw_count_now++;
			gimbal_control.can_yaw_speed = (gimbal_control.can_yaw_count_now - gimbal_control.can_yaw_count_last) * 1000 / (time_now - time_last);
			gimbal_control.can_yaw_count_last = gimbal_control.can_yaw_count_now;
			
			gimbal_control.time = time_now - time_last;
			gimbal_control.angle = gimbal_control.gimbal_yaw_motor.absolute_angle;
			
			
			time_last = time_now;
        //}
		osDelay(1);
//gimbal_control.yaw_current_set

		//osDelayUntil(&can_task_pre_tick,3);
    
    }




}