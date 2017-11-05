#include "common.h"

static void RCC_Configuration(void);
static void NVIC_Configuration(void);
static void ADC_Configuration(void);

/*
 * 系统时钟设定函数，时间：1ms
 */
static void set_board_systick()
{
    SysTick->LOAD = SYSTICK_1MS_TICKS - 1;
    NVIC_SetPriority(SysTick_IRQn, 0);
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    softdelay_calibrate();
}
/*
 * RCC clock配置函数
 */
static void RCC_Configuration(void)
{
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(APB2PERIPH_INIT_LIST, ENABLE);
    RCC_APB1PeriphClockCmd(APB1PERIPH_INIT_LIST, ENABLE);   
}
/*
 * 中断控制器配置函数
 */
static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    set_board_systick();

    /* Enable line5 ~ line9 external interrupt. */
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);  
}
/*
 * ADC配置函数
 */
#define			ADC1_DR_Address			ADC1_BASE + 0x4c
_u32 voltage;

static void ADC_Configuration(void)
{
	DMA_InitTypeDef DMA_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&voltage;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
	DMA_Cmd(DMA1_Channel1, ENABLE);
	
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1, 10, 1, ADC_SampleTime_239Cycles5);
	
	ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
//	ADC_TempSensorVrefintCmd(ENABLE);
    ADC_ResetCalibration(ADC1);

	while (ADC_GetResetCalibrationStatus(ADC1)){};

    ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1)){};
		
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
/*
 * MCU低级初始化
 */
_s32 init_board()
{
    RCC_Configuration();
	PERFORM_IO_REMAP();
    NVIC_Configuration();
    ADC_Configuration();
    return 1;
}
