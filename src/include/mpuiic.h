#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "common.h"
#include "sys.h"

// IIC
#define			I2C_PORT				GPIOB
#define 		I2C1_SCL				GPIO_Pin_6
#define 		I2C1_SDA				GPIO_Pin_7
#define 		I2C2_SCL				GPIO_Pin_10
#define 		I2C2_SDA				GPIO_Pin_11

//IO方向设置
#define MPU_SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)8<<28;} 
#define MPU_SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)3<<28;}

//IO操作函数	 
#define MPU_IIC_SCL    PBout(6) 		//SCL
#define MPU_IIC_SDA    PBout(7) 		//SDA	 
#define MPU_READ_SDA   PBin(7) 		//输入SDA 

//IIC所有操作函数
void MPU_IIC_Delay(void);				//MPU IIC延时函数
void MPU_IIC_Init(void);                //初始化IIC的IO口				 
void MPU_IIC_Start(void);				//发送IIC开始信号
void MPU_IIC_Stop(void);	  			//发送IIC停止信号
void MPU_IIC_Send_Byte(_u8 txd);			//IIC发送一个字节
_u8 MPU_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
_u8 MPU_IIC_Wait_Ack(void); 				//IIC等待ACK信号
void MPU_IIC_Ack(void);					//IIC发送ACK信号
void MPU_IIC_NAck(void);				//IIC不发送ACK信号

void IMPU_IC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
_u8 MPU_IIC_Read_One_Byte(u8 daddr,u8 addr);	  
#endif
















