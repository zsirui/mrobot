#pragma once
#include "common.h"

#define		USART_MAX_LEN  		200  				//定义最大接收字节数 200

extern		u8		send_data[USART_MAX_LEN];	  	
extern		u8		USART3_RX_BUF[USART_MAX_LEN]; 	//接收缓冲,最大USART_MAX_LEN个字节.末字节为换行符 
extern		u16		USART3_RX_STA;         			//接收状态标记
extern		u8		Res3;
extern		u8		USART2_RX_BUF[USART_MAX_LEN]; 	//接收缓冲,最大USART_MAX_LEN个字节.末字节为换行符 
extern		u16		USART2_RX_STA;         			//接收状态标记
extern		u8		Res2;

#define 		USART2_PORT				GPIOA
#define			USART2_TX				GPIO_Pin_2
#define			USART2_RX				GPIO_Pin_3
#ifdef _REMAP
#define 		USART3_PORT				GPIOC
#else
#define 		USART3_PORT				GPIOB
#endif
#define			USART3_TX				GPIO_Pin_10
#define			USART3_RX				GPIO_Pin_11
#define 		UART4_PORT				GPIOC
#define			UART4_TX				GPIO_Pin_10
#define			UART4_RX				GPIO_Pin_11

int usart2_init(u32 bound);
int usart3_init(u32 bound);
#ifdef _DMA_USART
//int usart3_dma_init(u32 bound);
void DMA_Enable(DMA_Channel_TypeDef * DMA_CHx);
#endif
#ifndef _REMAP
extern		u8		UART4_RX_BUF[USART_MAX_LEN]; 	//接收缓冲,最大USART_MAX_LEN个字节.末字节为换行符 
extern		u16		UART4_RX_STA;         			//接收状态标记
extern		u8		Res4;
int uart4_init(u32 bound);
#endif
