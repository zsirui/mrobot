//MCU主频
#define			CPU_FREQ				72000000L

//IO端口重映射
#define			PERFORM_IO_REMAP()		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);\
										GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE)

//APB2总线上的外设
#define APB2PERIPH_INIT_LIST  \
  ( RCC_APB2Periph_GPIOA \
  | RCC_APB2Periph_GPIOB \
  | RCC_APB2Periph_GPIOC \
  | RCC_APB2Periph_AFIO \
  | RCC_APB2Periph_ADC1 \
  )
//APB1总线上的外设
#define APB1PERIPH_INIT_LIST  \
  ( RCC_APB1Periph_USART2 \
  | RCC_APB1Periph_TIM3 \
  | RCC_APB1Periph_USART3 \
  )
