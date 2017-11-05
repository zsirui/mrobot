#include "protocol.h"
#include "crc.h"
#include "usart.h"
#include "motor.h"
#include "sonar.h"
#include "battery.h"
#include "mpu6050.h"
#include "inv_mpu.h"

enum diretion {
	FRONT = 0,
	BACK,
	LEFT,
	RIGHT,
};

enum SendCommandType {
	SIMPLE_INFO = 0,
	WITH_SONAR_INFO,
	WITH_SIX_AXIS_SENSOR_INFO,
	ALL_INFO,
	SPEED_INFO,
	BATTERY_INFO,
	SONAR_INFO,
	SIX_AXIS_SENSOR_INFO,
};

enum ReceiveCommandType {
	SETTING_SPEED_WITH_SIMPLE_INFO = 21930,						//0x55AA
	SETTING_SPEED_WITH_SONAR_INFO = 21845,						//0x5555
	SETTING_SPEED_WITH_SIX_AXIS_SENSOR_INFO = 21925,			//0x55A5
	SETTING_SPEED_WITH_ALL_INFO = 21850,						//0x555A
	SETTING_SPEED_WITH_SPEED_INFO = 43690,						//0xAAAA
	SETTING_SPEED_WITH_BATTERY_INFO = 43605,					//0xAA55
	SETTING_SPEED_WITH_ONLY_SONAR_INFO = 43685,					//0xAAA5
	SETTING_SPEED_WITH_ONLY_SIX_AXIS_SENSOR_INFO = 43610,		//0xAA5A
};

union checkSum
{
	u8 d;
	unsigned char data[1];
}SendCheckSum, ReceiveCheckSum;

union receiveData												//接收到的数据
{
	_s32 d;														//左右轮速度
	unsigned char data[4];
}leftdata, rightdata;											//接收的左右轮数据

union receiveHeader
{
	_u16 d;
	unsigned char data[2];
}receive_command, receive_header;

union receiveEnder
{
	_u16 d;
	unsigned char data[2];
}receive_ender;

union sendCommand
{
	_u16 d;
	unsigned char data[2];
}send_command;

union odometry													//里程计数据共用体
{
	float odoemtry_float;
	unsigned char odometry_char[4];
}vel_left, vel_right, temprature, odom_yaw, odom_pitch, odom_roll, sonar_front, sonar_back, sonar_left, sonar_right;

union battery													//电池数据共用体
{
	float battery_float;
	unsigned char battery_char[4];
}electricity, battery_percentage;

short command = -1;
const unsigned char ender[2] = {0x0d, 0x0a};
const unsigned char header[2] = {0x55, 0xaa};
/////////////////////////////////////////////
//	通讯协议概述
//	[消息头(2字节)][命令(2字节)][长度(1字节)][数据(n字节，n=长度)][校验(1字节)][消息尾(2字节)]
//	消息头固定为[0x55 0xaa]，消息尾固定为[0x0d 0x0a]
//	发送用命令参数：
//	(1)0x5a 0x5a：发送速度信息和电池信息
//	(2)0x5a 0x55：发送速度信息，电池信息和超声波信息
//	(3)0x5a 0xaa：发送速度信息，电池信息和六轴传感器信息
//	(4)0x5a 0xa5：发送速度信息，电池信息，超声波信息和六轴传感器信息
//	(5)0xa5 0x5a：发送速度信息
//	(6)0xa5 0x55：发送电池信息
//	(7)0xa5 0xaa：发送超声波信息
//	(8)0xa5 0xa5：发送六轴传感器信息
//	接收用命令参数：
//	(1)0x55 0xaa：请求发送速度信息和电池信息
//	(2)0x55 0x55：请求发送速度信息，电池信息和超声波信息
//	(3)0x55 0xa5：请求发送速度信息，电池信息和六轴传感器信息
//	(4)0x55 0x5a：请求发送速度信息，电池信息，超声波信息和六轴传感器信息
//	(5)0xaa 0xaa：请求发送速度信息
//	(6)0xaa 0x55：请求发送电池信息
//	(7)0xaa 0xa5：请求发送超声波信息
//	(8)0xaa 0x5a：请求发送六轴传感器信息
/////////////////////////////////////////////

unsigned char _ID[12];												//存放芯片ID的临时变量
 
void GetChipID(void)
{
	u32 _ID0, _ID1, _ID2;
	_ID0 = *(__IO u32*)(0x1FFFF7E8);								//产品唯一身份标识寄存器（96位）
	_ID1 = *(__IO u32*)(0x1FFFF7EC);
	_ID2 = *(__IO u32*)(0x1FFFF7F0);
								  
	_ID[0] = (u8)(_ID0 & 0x000000FF);
	_ID[1] = (u8)((_ID0 & 0x0000FF00)>>8);
	_ID[2] = (u8)((_ID0 & 0x00FF0000)>>16);
	_ID[3] = (u8)((_ID0 & 0xFF000000)>>24);
	_ID[4] = (u8)(_ID1 & 0x000000FF);
	_ID[5] = (u8)((_ID1 & 0x0000FF00)>>8);
	_ID[6] = (u8)((_ID1 & 0x00FF0000)>>16);
	_ID[7] = (u8)((_ID1 & 0xFF000000)>>24);
	_ID[8] = (u8)(_ID2 & 0x000000FF);
	_ID[9] = (u8)((_ID2 & 0x0000FF00)>>8);
	_ID[10] = (u8)((_ID2 & 0x00FF0000)>>16);
	_ID[11] = (u8)((_ID2 & 0xFF000000)>>24);         
}

void SendData(short Temperature, float pitch, float roll, float yaw)
{
	int i, length = 0;
	vel_left.odoemtry_float = get_walkingmotor_lspeed_mm();
	vel_right.odoemtry_float = get_walkingmotor_rspeed_mm();
	sonar_front.odoemtry_float = sonar_get(FRONT);
	sonar_back.odoemtry_float = sonar_get(BACK);
	sonar_left.odoemtry_float = sonar_get(LEFT);
	sonar_right.odoemtry_float = sonar_get(RIGHT);
	electricity.battery_float = get_electricity();
	battery_percentage.battery_float = get_electricitypercentage();
	temprature.odoemtry_float = (float)Temperature / 100;
	odom_pitch.odoemtry_float = pitch;
	odom_roll.odoemtry_float = roll;
	odom_yaw.odoemtry_float = yaw;
	for(i = 0; i < 2; i++)
	{
		send_data[i] = header[i];
	}
	switch(command)
	{
		case SIMPLE_INFO:
			send_command.d = 23130;
			for(i = 0; i < 2; i++)
			{
				send_data[i + 2] = send_command.data[i];
			}
			length = 16;
			send_data[4] = length;
			for(i = 0; i < 4; i++)
			{
				send_data[i + 5] 	= vel_left.odometry_char[i];
				send_data[i + 9] 	= vel_right.odometry_char[i];
				send_data[i + 13]	= electricity.battery_char[i];
				send_data[i + 17]	= battery_percentage.battery_char[i];
			}
		break;
		case WITH_SONAR_INFO:
			send_command.d = 23125;
			for(i = 0; i < 2; i++)
			{
				send_data[i + 2] = send_command.data[i];
			}
			length = 32;
			send_data[4] = length;
			for(i = 0; i < 4; i++)
			{
				send_data[i + 5] 	= vel_left.odometry_char[i];
				send_data[i + 9] 	= vel_right.odometry_char[i];
				send_data[i + 13]	= electricity.battery_char[i];
				send_data[i + 17]	= battery_percentage.battery_char[i];
				send_data[i + 21] 	= sonar_front.odometry_char[i];
				send_data[i + 25] 	= sonar_back.odometry_char[i];
				send_data[i + 29]	= sonar_left.odometry_char[i];
				send_data[i + 33]	= sonar_right.odometry_char[i];
			}
		break;
		case WITH_SIX_AXIS_SENSOR_INFO:
			send_command.d = 23210;
			for(i = 0; i < 2; i++)
			{
				send_data[i + 2] = send_command.data[i];
			}
			length = 32;
			send_data[4] = length;
			for(i = 0; i < 4; i++)
			{
				send_data[i + 5] 	= vel_left.odometry_char[i];
				send_data[i + 9] 	= vel_right.odometry_char[i];
				send_data[i + 13]	= electricity.battery_char[i];
				send_data[i + 17]	= battery_percentage.battery_char[i];
				send_data[i + 21] 	= odom_pitch.odometry_char[i];
				send_data[i + 25] 	= odom_roll.odometry_char[i];
				send_data[i + 29]	= odom_yaw.odometry_char[i];
				send_data[i + 33]	= temprature.odometry_char[i];
			}
		break;
		case ALL_INFO:
			send_command.d = 23205;
			for(i = 0; i < 2; i++)
			{
				send_data[i + 2] = send_command.data[i];
			}
			length = 48;
			send_data[4] = length;
			for(i = 0; i < 4; i++)
			{
				send_data[i + 5] 	= vel_left.odometry_char[i];
				send_data[i + 9] 	= vel_right.odometry_char[i];
				send_data[i + 13]	= electricity.battery_char[i];
				send_data[i + 17]	= battery_percentage.battery_char[i];
				send_data[i + 21] 	= odom_pitch.odometry_char[i];
				send_data[i + 25] 	= odom_roll.odometry_char[i];
				send_data[i + 29]	= odom_yaw.odometry_char[i];
				send_data[i + 33]	= temprature.odometry_char[i];
				send_data[i + 37] 	= sonar_front.odometry_char[i];
				send_data[i + 41] 	= sonar_back.odometry_char[i];
				send_data[i + 45]	= sonar_left.odometry_char[i];
				send_data[i + 49]	= sonar_right.odometry_char[i];
			}
		break;
		case SPEED_INFO:
			send_command.d = 42330;
			for(i = 0; i < 2; i++)
			{
				send_data[i + 2] = send_command.data[i];
			}
			length = 8;
			send_data[4] = length;
			for(i = 0; i < 4; i++)
			{
				send_data[i + 5] = vel_left.odometry_char[i];
				send_data[i + 9] = vel_right.odometry_char[i];
			}
		break;
		case BATTERY_INFO:
			send_command.d = 42325;
			for(i = 0; i < 2; i++)
			{
				send_data[i + 2] = send_command.data[i];
			}
			length = 8;
			send_data[4] = length;
			for(i = 0; i < 4; i++)
			{
				send_data[i + 5] = electricity.battery_char[i];
				send_data[i + 9] = battery_percentage.battery_char[i];
			}
		break;
		case SONAR_INFO:
			send_command.d = 42410;
			for(i = 0; i < 2; i++)
			{
				send_data[i + 2] = send_command.data[i];
			}
			length = 16;
			send_data[4] = length;
			for(i = 0; i < 4; i++)
			{
				send_data[i + 5] 	= sonar_front.odometry_char[i];
				send_data[i + 9] 	= sonar_back.odometry_char[i];
				send_data[i + 13]	= sonar_left.odometry_char[i];
				send_data[i + 17]	= sonar_right.odometry_char[i];
			}
		break;
		case SIX_AXIS_SENSOR_INFO:
			send_command.d = 42405;
			for(i = 0; i < 2; i++)
			{
				send_data[i + 2] = send_command.data[i];
			}
			length = 16;
			send_data[4] = length;
			for(i = 0; i < 4; i++)
			{
				send_data[i + 5] 	= odom_pitch.odometry_char[i];
				send_data[i + 9] 	= odom_roll.odometry_char[i];
				send_data[i + 13]	= odom_yaw.odometry_char[i];
				send_data[i + 17]	= temprature.odometry_char[i];
			}
		break;
	}
	send_data[5 + length] = Get_Crc8(send_data, 5 + length);
	send_data[6 + length] = ender[0];
	send_data[6 + length + 1] = ender[1];
	GetChipID();
	for(i = 0; i < 12; i++)
	{
		send_data[8 + length + i] = _ID[i];
	}
#ifdef _DMA_USART
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
	DMA_Enable(DMA1_Channel2);
	while(1)
	{
		if(DMA_GetFlagStatus(DMA1_FLAG_TC2) != RESET)
		{
			DMA_ClearFlag(DMA1_FLAG_TC2);
			break;
		}
	}
#else
	for(i = 0; i < 200; i++)
	{
		USART_SendData(USART3, send_data[i]);						//发送一个字节到串口 
		while(USART_GetFlagStatus(USART3, USART_FLAG_TC) != SET);	//等待发送结束            
	}
#endif
}

void ReceiveData(short Temperature, float pitch, float roll, float yaw)
{
	int i, length, _checkSum;
	_s32 lspeed = 0, rspeed = 0;
	if(USART3_RX_STA)
	{
		for (i = 0; i < 2; i++)
		{
			receive_header.data[i] = USART3_RX_BUF[i];
		}
		if (receive_header.data[0] == header[0] && receive_header.data[1] == header[1])
		{
			for (i = 0; i < 2; i++)
			{
				receive_command.data[i] = USART3_RX_BUF[i + 2];
			}
			length = USART3_RX_BUF[4];
			_checkSum = Get_Crc8(USART3_RX_BUF, 5 + length);
			switch(receive_command.d)
			{
				case SETTING_SPEED_WITH_SIMPLE_INFO:
					if (USART3_RX_BUF[6 + length] == ender[0] && USART3_RX_BUF[6 + length + 1] == ender[1])
					{
						ReceiveCheckSum.data[0] = USART3_RX_BUF[5 + length];
						if (_checkSum == ReceiveCheckSum.d)
						{
							for(i = 0; i < 4; i++)
							{
								rightdata.data[i] = USART3_RX_BUF[i + 5];
								leftdata.data[i] = USART3_RX_BUF[i + 9];
							}
							rspeed = rightdata.d;
							lspeed = leftdata.d;
							command = SIMPLE_INFO;
						}
					}
					USART3_RX_STA = 0;						//清楚接收标志位
					Res3 = 0;
					set_walkingmotor_speed(lspeed, rspeed);
					SendData(Temperature, pitch, roll, yaw);
				break;
				case SETTING_SPEED_WITH_SONAR_INFO:
					if (USART3_RX_BUF[6 + length] == ender[0] && USART3_RX_BUF[6 + length + 1] == ender[1])
					{
						ReceiveCheckSum.data[0] = USART3_RX_BUF[5 + length];
						if (_checkSum == ReceiveCheckSum.d)
						{
							for(i = 0; i < 4; i++)
							{
								rightdata.data[i] = USART3_RX_BUF[i + 5];
								leftdata.data[i] = USART3_RX_BUF[i + 9];
							}
							rspeed = rightdata.d;
							lspeed = leftdata.d;
							command = WITH_SONAR_INFO;
						}
					}
					USART3_RX_STA = 0;						//清楚接收标志位
					Res3 = 0;
					set_walkingmotor_speed(lspeed, rspeed);
					SendData(Temperature, pitch, roll, yaw);
				break;
				case SETTING_SPEED_WITH_SIX_AXIS_SENSOR_INFO:
					if (USART3_RX_BUF[6 + length] == ender[0] && USART3_RX_BUF[6 + length + 1] == ender[1])
					{
						ReceiveCheckSum.data[0] = USART3_RX_BUF[5 + length];
						if (_checkSum == ReceiveCheckSum.d)
						{
							for(i = 0; i < 4; i++)
							{
								rightdata.data[i] = USART3_RX_BUF[i + 5];
								leftdata.data[i] = USART3_RX_BUF[i + 9];
							}
							rspeed = rightdata.d;
							lspeed = leftdata.d;
							command = WITH_SIX_AXIS_SENSOR_INFO;
						}
					}
					USART3_RX_STA = 0;						//清楚接收标志位
					Res3 = 0;
					set_walkingmotor_speed(lspeed, rspeed);
					SendData(Temperature, pitch, roll, yaw);
				break;
				case SETTING_SPEED_WITH_ALL_INFO:
					if (USART3_RX_BUF[6 + length] == ender[0] && USART3_RX_BUF[6 + length + 1] == ender[1])
					{
						ReceiveCheckSum.data[0] = USART3_RX_BUF[5 + length];
						if (_checkSum == ReceiveCheckSum.d)
						{
							for(i = 0; i < 4; i++)
							{
								rightdata.data[i] = USART3_RX_BUF[i + 5];
								leftdata.data[i] = USART3_RX_BUF[i + 9];
							}
							rspeed = rightdata.d;
							lspeed = leftdata.d;
							command = ALL_INFO;
						}
					}
					USART3_RX_STA = 0;						//清楚接收标志位
					Res3 = 0;
					set_walkingmotor_speed(lspeed, rspeed);
					SendData(Temperature, pitch, roll, yaw);
				break;
				case SETTING_SPEED_WITH_SPEED_INFO:
					if (USART3_RX_BUF[6 + length] == ender[0] && USART3_RX_BUF[6 + length + 1] == ender[1])
					{
						ReceiveCheckSum.data[0] = USART3_RX_BUF[5 + length];
						if (_checkSum == ReceiveCheckSum.d)
						{
							for(i = 0; i < 4; i++)
							{
								rightdata.data[i] = USART3_RX_BUF[i + 5];
								leftdata.data[i] = USART3_RX_BUF[i + 9];
							}
							rspeed = rightdata.d;
							lspeed = leftdata.d;
							command = SPEED_INFO;
						}
					}
					USART3_RX_STA = 0;						//清楚接收标志位
					Res3 = 0;
					set_walkingmotor_speed(lspeed, rspeed);
					SendData(Temperature, pitch, roll, yaw);
				break;
				case SETTING_SPEED_WITH_BATTERY_INFO:
					if (USART3_RX_BUF[6 + length] == ender[0] && USART3_RX_BUF[6 + length + 1] == ender[1])
					{
						ReceiveCheckSum.data[0] = USART3_RX_BUF[5 + length];
						if (_checkSum == ReceiveCheckSum.d)
						{
							for(i = 0; i < 4; i++)
							{
								rightdata.data[i] = USART3_RX_BUF[i + 5];
								leftdata.data[i] = USART3_RX_BUF[i + 9];
							}
							rspeed = rightdata.d;
							lspeed = leftdata.d;
							command = BATTERY_INFO;
						}
					}
					USART3_RX_STA = 0;						//清楚接收标志位
					Res3 = 0;
					set_walkingmotor_speed(lspeed, rspeed);
					SendData(Temperature, pitch, roll, yaw);
				break;
				case SETTING_SPEED_WITH_ONLY_SONAR_INFO:
					if (USART3_RX_BUF[6 + length] == ender[0] && USART3_RX_BUF[6 + length + 1] == ender[1])
					{
						ReceiveCheckSum.data[0] = USART3_RX_BUF[5 + length];
						if (_checkSum == ReceiveCheckSum.d)
						{
							for(i = 0; i < 4; i++)
							{
								rightdata.data[i] = USART3_RX_BUF[i + 5];
								leftdata.data[i] = USART3_RX_BUF[i + 9];
							}
							rspeed = rightdata.d;
							lspeed = leftdata.d;
							command = SONAR_INFO;
						}
					}
					USART3_RX_STA = 0;						//清楚接收标志位
					Res3 = 0;
					set_walkingmotor_speed(lspeed, rspeed);
					SendData(Temperature, pitch, roll, yaw);
				break;
				case SETTING_SPEED_WITH_ONLY_SIX_AXIS_SENSOR_INFO:
					if (USART3_RX_BUF[6 + length] == ender[0] && USART3_RX_BUF[6 + length + 1] == ender[1])
					{
						ReceiveCheckSum.data[0] = USART3_RX_BUF[5 + length];
						if (_checkSum == ReceiveCheckSum.d)
						{
							for(i = 0; i < 4; i++)
							{
								rightdata.data[i] = USART3_RX_BUF[i + 5];
								leftdata.data[i] = USART3_RX_BUF[i + 9];
							}
							rspeed = rightdata.d;
							lspeed = leftdata.d;
							command = SIX_AXIS_SENSOR_INFO;
						}
					}
					USART3_RX_STA = 0;						//清楚接收标志位
					Res3 = 0;
					set_walkingmotor_speed(lspeed, rspeed);
					SendData(Temperature, pitch, roll, yaw);
				break;
				default:
					USART3_RX_STA = 0;						//清楚接收标志位
					Res3 = 0;
				break;
			}
		}
	}
}
