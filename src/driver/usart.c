#include "usart.h"
#include "stdio.h"

#ifdef _PRINT
#pragma import(__use_no_semihosting)

struct __FILE
{
	int handle;
};

FILE __stdout;

int fputc(int ch, FILE *f)
{
	while((USART3->SR&0X40)==0){};
	USART3->DR = (u8) ch;
	return ch;
}
#endif
u8 send_data[USART_MAX_LEN];
u8 USART2_RX_BUF[USART_MAX_LEN];		//接收缓冲,最大USART_MAX_LEN个字节.
u16 USART2_RX_STA = 0;					//接收状态标记
u8 Res2 = 0;
u8 USART3_RX_BUF[USART_MAX_LEN];		//接收缓冲,最大USART_MAX_LEN个字节.
u16 USART3_RX_STA = 0;					//接收状态标记
u8 Res3 = 0;

int usart3_init(u32 bound){
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
#ifdef _REMAP
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);									//将USART3局部重映射到PC10，PC11
#else
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
#endif
	//USART3_TX   GPIOB.10
	GPIO_InitStructure.GPIO_Pin = USART3_TX;												//PB.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;											//复用推挽输出
	GPIO_Init(USART3_PORT, &GPIO_InitStructure);											//初始化GPIOB.10

	//USART3_RX	  GPIOB.11初始化
	GPIO_InitStructure.GPIO_Pin = USART3_RX;												//PC.11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;									//浮空输入
	GPIO_Init(USART3_PORT, &GPIO_InitStructure);											//初始化GPIOB.11  

	//USART3 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;								//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;										//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;											//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);															//根据指定的参数初始化VIC寄存器

	//USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;												//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;								//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;									//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;										//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;			//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;							//收发模式

	USART_Init(USART3, &USART_InitStructure);												//初始化串口3
#ifdef _DMA_USART
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
#endif
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);											//开启串口接受中断
	USART_Cmd(USART3, ENABLE);																//使能串口3
#ifdef _DMA_USART
	DMA_InitTypeDef DMA_InitStructure;
	
	DMA_DeInit(DMA1_Channel2);																//将DMA的通道12寄存器重设为缺省值  串口1对应的是DMA通道5
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;							//DMA外设usart基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)send_data;									//DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;										//数据传输方向，从外设读取发送到内存
	DMA_InitStructure.DMA_BufferSize = USART_MAX_LEN;										//DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;						//外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;									//内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;					//数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;							//数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;											//工作在正常缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;									//DMA通道12拥有中优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;											//DMA通道12没有设置为内存到内存传输
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);											//根据DMA_InitStruct中指定的参数初始化DMA的通道
	DMA_Cmd(DMA1_Channel2, ENABLE);															//正式驱动DMA传输

	//DMA串口接收配置
	DMA_DeInit(DMA1_Channel3);																//将DMA的通道13寄存器重设为缺省值  串口1对应的是DMA通道5
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;							//DMA外设usart基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART3_RX_BUF;								//DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;										//数据传输方向，从外设读取发送到内存
	DMA_InitStructure.DMA_BufferSize = 16;										//DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;						//外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;									//内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;					//数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;							//数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;											//工作在正常缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;									//DMA通道13拥有中优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;											//DMA通道13没有设置为内存到内存传输
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);											//根据DMA_InitStruct中指定的参数初始化DMA的通道
	DMA_Cmd(DMA1_Channel3, ENABLE);															//正式驱动DMA传输

	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);											//开启串口DMA发送
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);											//开启串口DMA接收
#endif
	return 1;
}

#ifdef _DMA_USART
//int usart3_dma_init(u32 bound){
//	//GPIO端口设置
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	DMA_InitTypeDef DMA_InitStructure;

//#ifdef _REMAP
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

//	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);									//将USART3局部重映射到PC10，PC11
//#else
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
//#endif

//	//USART3_TX   GPIOB.10
//	GPIO_InitStructure.GPIO_Pin = USART3_TX;												//PB.10
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;											//复用推挽输出
//	GPIO_Init(USART3_PORT, &GPIO_InitStructure);											//初始化GPIOB.10

//	//USART3_RX	  GPIOB.11初始化
//	GPIO_InitStructure.GPIO_Pin = USART3_RX;												//PC.11
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;									//浮空输入
//	GPIO_Init(USART3_PORT, &GPIO_InitStructure);											//初始化GPIOB.11  

//	//USART3 NVIC 配置
//	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;								//抢占优先级3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;										//子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;											//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);															//根据指定的参数初始化VIC寄存器

//	//USART 初始化设置

//	USART_InitStructure.USART_BaudRate = bound;												//串口波特率
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;								//字长为8位数据格式
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;									//一个停止位
//	USART_InitStructure.USART_Parity = USART_Parity_No;										//无奇偶校验位
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;			//无硬件数据流控制
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;							//收发模式

//	USART_Init(USART3, &USART_InitStructure);												//初始化串口3
//	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
//	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);											//开启串口接受中断
//	USART_Cmd(USART3, ENABLE);																//使能串口3

//	//DMA串口发送配置
//	DMA_DeInit(DMA1_Channel2);																//将DMA的通道12寄存器重设为缺省值  串口1对应的是DMA通道5
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;							//DMA外设usart基地址
//	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)send_data;									//DMA内存基地址
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;										//数据传输方向，从外设读取发送到内存
//	DMA_InitStructure.DMA_BufferSize = USART_MAX_LEN;										//DMA通道的DMA缓存的大小
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;						//外设地址寄存器不变
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;									//内存地址寄存器递增
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;					//数据宽度为8位
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;							//数据宽度为8位
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;											//工作在正常缓存模式
//	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;									//DMA通道12拥有中优先级 
//	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;											//DMA通道12没有设置为内存到内存传输
//	DMA_Init(DMA1_Channel2, &DMA_InitStructure);											//根据DMA_InitStruct中指定的参数初始化DMA的通道
//	DMA_Cmd(DMA1_Channel2, ENABLE);															//正式驱动DMA传输

//	//DMA串口接收配置
//	DMA_DeInit(DMA1_Channel3);																//将DMA的通道13寄存器重设为缺省值  串口1对应的是DMA通道5
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;							//DMA外设usart基地址
//	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART3_RX_BUF;								//DMA内存基地址
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;										//数据传输方向，从外设读取发送到内存
//	DMA_InitStructure.DMA_BufferSize = USART_MAX_LEN;										//DMA通道的DMA缓存的大小
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;						//外设地址寄存器不变
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;									//内存地址寄存器递增
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;					//数据宽度为8位
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;							//数据宽度为8位
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;											//工作在正常缓存模式
//	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;									//DMA通道13拥有中优先级 
//	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;											//DMA通道13没有设置为内存到内存传输
//	DMA_Init(DMA1_Channel3, &DMA_InitStructure);											//根据DMA_InitStruct中指定的参数初始化DMA的通道
//	DMA_Cmd(DMA1_Channel3, ENABLE);															//正式驱动DMA传输

//	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);											//开启串口DMA发送
//	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);											//开启串口DMA接收

//	return 1;
//}

void DMA_Enable(DMA_Channel_TypeDef * DMA_CHx)
{
	DMA_Cmd(DMA_CHx, DISABLE);
	DMA_SetCurrDataCounter(DMA_CHx, USART_MAX_LEN);
	DMA_Cmd(DMA_CHx, ENABLE);
}
#endif

int usart2_init(u32 bound){
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	//USART2_TX   GPIOA.2
	GPIO_InitStructure.GPIO_Pin = USART2_TX; //PA.2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;											//复用推挽输出
	GPIO_Init(USART2_PORT, &GPIO_InitStructure);

	//USART2_RX	  GPIOA.3
	GPIO_InitStructure.GPIO_Pin = USART2_RX;//PA.3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;									//浮空输入
	GPIO_Init(USART2_PORT, &GPIO_InitStructure);

	//USART2 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;								//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;										//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;											//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);															//根据指定的参数初始化VIC寄存器

	//USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;												//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;								//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;									//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;										//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;			//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;							//收发模式

	USART_Init(USART2, &USART_InitStructure); 												//初始化串口2
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);											//开启串口接受中断
	USART_Cmd(USART2, ENABLE);																//使能串口2
	return 1;
}
#ifndef _REMAP

u8 UART4_RX_BUF[USART_MAX_LEN];			//接收缓冲,最大USART_MAX_LEN个字节.
u16 UART4_RX_STA = 0;					//接收状态标记
u8 Res4 = 0;

int uart4_init(u32 bound){
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	//UART4_TX   GPIOC.10
	GPIO_InitStructure.GPIO_Pin = UART4_TX;													//PC.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;											//复用推挽输出
	GPIO_Init(UART4_PORT, &GPIO_InitStructure);												//初始化GPIOC.10

	//UART4_RX	  GPIOC.11初始化
	GPIO_InitStructure.GPIO_Pin = UART4_RX;													//PC.11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;									//浮空输入
	GPIO_Init(UART4_PORT, &GPIO_InitStructure);												//初始化GPIOC.11  

	//UART4 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;								//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;										//子优先级4
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;											//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);															//根据指定的参数初始化VIC寄存器

	//USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;												//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;								//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;									//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;										//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;			//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;							//收发模式

	USART_Init(UART4, &USART_InitStructure);												//初始化串口4
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);											//开启串口接受中断
	USART_Cmd(UART4, ENABLE);																//使能串口4 
	return 1;
}

void UART4_IRQHandler(void)                	//串口4中断服务程序
{
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
		UART4_RX_BUF[Res4] = USART_ReceiveData(UART4);
		Res4++;
	}

	if((UART4_RX_BUF[Res4 - 2] == 0x0D)&&(USART2_RX_BUF[Res4 - 1] == 0x0A)) UART4_RX_STA = 1;
	if(USART_GetFlagStatus(UART4, USART_FLAG_ORE) == SET)
	{
		USART_ClearFlag(UART4, USART_FLAG_ORE);
		USART_ReceiveData(UART4);
	}
}
#endif

#ifdef _DMA_USART
void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
	{
		USART_ReceiveData(USART3);
		USART3_RX_STA = 1;
		USART_ClearITPendingBit(USART3, USART_IT_IDLE);
		DMA_Enable(DMA1_Channel3);
	}
}
#else
void USART3_IRQHandler(void)                	//串口3中断服务程序
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		USART3_RX_BUF[Res3] = USART_ReceiveData(USART3);
		Res3++;
	}

	if((USART3_RX_BUF[Res3 - 2] == 0x0D)&&(USART3_RX_BUF[Res3 - 1] == 0x0A)) USART3_RX_STA = 1;
	if(USART_GetFlagStatus(USART3, USART_FLAG_ORE) == SET)
	{
		USART_ClearFlag(USART3, USART_FLAG_ORE);
		USART_ReceiveData(USART3);
	}
}
#endif

void USART2_IRQHandler(void)                	//串口2中断服务程序
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		USART2_RX_BUF[Res2] = USART_ReceiveData(USART2);
		Res2++;
	}

	if((USART2_RX_BUF[Res2 - 2] == 0x0D)&&(USART2_RX_BUF[Res2 - 1] == 0x0A)) USART2_RX_STA = 1;
	if(USART_GetFlagStatus(USART2, USART_FLAG_ORE) == SET)
	{
		USART_ClearFlag(USART2, USART_FLAG_ORE);
		USART_ReceiveData(USART2);
	}
}
