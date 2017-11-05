#pragma once
#include "types.h"

#define SYSTICK_1MS_TICKS    (CPU_FREQ/1000)

#define 	cli 					__disable_irq
#define 	sei 					__enable_irq

static _u32 enter_critical_section(void)
{
    _u32 context=__get_PRIMASK();
    cli();
    return context;
}

static void leave_critical_section(_u32 context)
{
    __set_PRIMASK(context);
}


typedef void (* abort_proc_t ) (void);
void board_abort_mode(void);
void board_set_abort_proc( abort_proc_t proc);
uint32_t getms(void);
uint64_t getus(void);
void delay(uint32_t ms);
void delay_alert(uint32_t ms);
extern void _delay_us(volatile uint32_t us);
static void _delay_ms(uint32_t ms)
{
  while(ms--)
  {
     _delay_us(1000);
  }
}
void alert(void);
void clear_alert(void);
int is_alert(void);

/*
* GPIO相关
*/
#define HIGH 1
#define LOW  0

#define _PIN_SET_1(port, pin) port->BSRR = pin
#define _PIN_SET_0(port, pin) port->BRR = pin
#define _PIN_SET(port, pin, val) _PIN_SET_##val(port, pin)

#define PIN_SET(port, pin, val) EXPAND_WRAPPER(_PIN_SET, port, pin, val)
#define PIN_READ(port, pin) ((port)->IDR & pin)

#define pinSet   GPIO_WriteBit
#define pinRead  GPIO_ReadInputDataBit

#define _RAW_PWM_SET(channel, timer, val) \
  TIM##timer->CCR##channel = val

#define RAW_PWM_SET(channel, timer, val) \
  EXPAND_WRAPPER(_RAW_PWM_SET, channel, timer, val)

static void pinMode(GPIO_TypeDef* GPIOx, uint16_t pin,
                           GPIOMode_TypeDef mode, GPIOSpeed_TypeDef GPIO_Speed)
{
  //GPIO_InitTypeDef init = { pin, GPIO_Speed, mode};
	GPIO_InitTypeDef init;
	init.GPIO_Pin = pin;
	init.GPIO_Speed = GPIO_Speed;
	init.GPIO_Mode = mode;
	GPIO_Init(GPIOx, &init);
}

/*
* ADC相关
*/
void adc_read_start(ADC_TypeDef * adc_dev, uint8_t ADC_Channel);
uint16_t adc_read_final(ADC_TypeDef * adc_dev);
uint16_t adc_read_wait(ADC_TypeDef * adc_dev);
static int adc_read_is_ready(ADC_TypeDef * adc_dev)
{
  return (adc_dev->SR & ADC_FLAG_EOC);
}
float board_get_temperature(void);
uint16_t get_adc(uint8_t ch);
uint16_t get_adc_average(uint8_t ch, uint8_t times);
#include "softdelay.h"
#include "rpstm32.h"
#include "usart.h"
