#include "common.h"
#include "watchdog.h"

static void _default_abort_proc(void);
static volatile uint8_t alert_flg = 0;
static abort_proc_t _abort_proc = _default_abort_proc;
static volatile uint32_t _systick_val = 0;
static void _default_abort_proc()
{
    while(1);
}

/*
 * 系统定时器节拍中断
 * 节拍:1ms
 */
void SysTick_Handler(void)
{
    _systick_val ++;
}

void alert(void)
{
    alert_flg = 1;
}

int is_alert(void)
{
    return alert_flg;
}

void clear_alert(void)
{
    alert_flg = 0;
}

void board_set_abort_proc( abort_proc_t proc)
{
    _abort_proc = proc;
}

void board_abort_mode()
{
    cli();
    _abort_proc();
}

/*
 * 获取毫秒总累计数
 * 单位:ms
 */
uint32_t getms()
{
    return _systick_val;
}

#define SYSTICK_uS_PER_TICK      1000L/SYSTICK_1MS_TICKS
/*
 * 获取微秒总累计数
 * 单位:us
 */
uint64_t getus()
{
    register _u32 cached_ms, cached_tick;

    _u32 context = enter_critical_section();

    cached_ms = _systick_val;
    cached_tick = SysTick->VAL;

    if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) {
        ++cached_ms;
        cached_tick = SysTick->VAL;
    }

    leave_critical_section(context);

    return (_u64)cached_ms*1000 + ((SYSTICK_1MS_TICKS-1 - cached_tick)*SYSTICK_uS_PER_TICK);

}

/*
 * 不可打断的延时函数
 * 延时单位:ms
 */
void delay(uint32_t ms)
{
    uint32_t targettime = getms() + ms;

    while( getms() < targettime);
}
/*
 * 可打断的延时函数
 * 延时单位:ms
 */
void delay_alert(uint32_t ms)
{
    uint32_t targettime = getms() + ms;
    while( getms() < targettime && (!alert_flg));
    clear_alert();
}

uint16_t get_adc(uint8_t ch)
{
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
	return ADC_GetConversionValue(ADC1);
}

uint16_t get_adc_average(uint8_t ch, uint8_t times)
{
	uint32_t temp_val = 0;
	uint8_t t;
	for(t = 0; t < times; t++)
	{
		temp_val += get_adc(ch);
		mark_watchdog();
		_delay_ms(5);
	}
	return temp_val / times;
}
