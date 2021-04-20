#include "power.h"
#include "usart.h"


//超级电容指令集
uint8_t power_en_read_cmd  [11]={0x01,0x01,0x00,0x02,0x00,0x01,0x5C,0x0A,0X0D,0X0A};
uint8_t power_en_write_cmd [11]={0x01 ,0x05 ,0x00 ,0x02 ,0xFF ,0x00 ,0x2D ,0xFA,0X0D,0X0A};
uint8_t power_en_back_data[11];
uint8_t power_en_state_data[11];

uint8_t power_routin_read_cmd  [11]={0x01,0x03,0x00,0x38,0x00,0x01,0x05,0xC7,0X0D,0X0A,0x00};
uint8_t power_routin_write_cmd_1 [11]={0x01,0x06,0x00,0x38,0x00,0x01,0xC9,0xC7,0X0D,0X0A};
uint8_t power_routin_write_cmd_0 [11]={0x01,0x06,0x00,0x38,0x00,0x00,0x08,0x07,0X0D,0X0A};
uint8_t power_routin_back_data[11];
uint8_t power_routin_state_data[11] = {1,1,1,1,1,1};



//超级电容功率级使能命令封装
//0:功率级关闭
//1:功率级使能
void superpower_en()
{


	HAL_UART_Transmit(&huart2,power_en_write_cmd,10,0xff);
	HAL_UART_Receive(&huart2,power_en_back_data,11,0xff);
	HAL_Delay(200);
}
  
  


//超级电容功率路径选择
//0:电源直供
//1:超级电容供电  
void superpower_routin_1()
{

	HAL_UART_Transmit(&huart2,power_routin_write_cmd_1,10,0xff);
	HAL_UART_Receive(&huart2,power_routin_back_data,6,0xff);
	HAL_Delay(200);

}
  
  
void superpower_routin_0()
{

	HAL_UART_Transmit(&huart2,power_routin_write_cmd_0,10,0xff);
	HAL_UART_Receive(&huart2,power_routin_back_data,6,0xff);
	HAL_Delay(200);

}

//读取当前的超级电容功率级使能状态
int8_t superpower_en_read()
{
	HAL_UART_Transmit(&huart2,power_en_read_cmd,10,0xff);
	HAL_UART_Receive(&huart2,power_en_state_data,6,0xff);
	HAL_Delay(200);
	
	return power_en_state_data[5];
}

//读取当前的超级电容功率路径状态
int8_t superpower_routin_read()
{
	HAL_UART_Transmit(&huart2,power_routin_read_cmd,10,0xff);
	HAL_UART_Receive(&huart2,power_routin_state_data,6,0xff);
	HAL_Delay(200);
	
	return power_routin_state_data[5];

}



