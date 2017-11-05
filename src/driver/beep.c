#include "beep.h"

static _u32 _delay = 0;
static _u32 _startMs = 0;
static _u32 _beeper_ticks = 0;
static _u8  _beeper_status = 0;
/*
 * 蜂鸣器初始化函数
 * PWM输出，频率可调
 */
int init_beep(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);

    GPIO_InitStructure.GPIO_Pin = BEEP_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BEEP_GPIO, &GPIO_InitStructure);
	
	PIN_SET(BEEP_GPIO, BEEP_PIN, LOW);
	
    TIM_TimeBaseStructure.TIM_Period = TIME_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = CPU_FREQ / TIME_PRESCALER / BEEP_INIT_HZ;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);

    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
	
	return 1;
}
/*
 * 蜂鸣器使能函数
 */
static void enable_beep(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

    GPIO_InitStructure.GPIO_Pin = BEEP_PIN;
    GPIO_Init(BEEP_GPIO, &GPIO_InitStructure);
}
/*
 * 蜂鸣器禁止函数
 */
static void disable_beep(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

    GPIO_InitStructure.GPIO_Pin = BEEP_PIN;
    GPIO_Init(BEEP_GPIO, &GPIO_InitStructure);
}
/*
 * 蜂鸣器打开函数
 */
void start_beep(void)
{
    enable_beep();
}
/*
 * 蜂鸣器关闭函数
 */
void stop_beep(void)
{
    disable_beep();
}

static _u32 HighFrequency[] = { 0, 1046, 1175, 1318, 1397, 1568, 1760, 1976 };          //高频发声
/*
 * 蜂鸣器PWM频率设置函数
 */
static void set_beep_frequency(_u32 frequency)
{
    TIM2->PSC = CPU_FREQ / TIME_PRESCALER / frequency;
}
/*
 * 蜂鸣器PWM脉宽设置函数
 */
static void set_beep_sound(_u32 sound)
{
    TIM2->CCR1 = sound;
}
/*
 * 蜂鸣器上电特定发声函数
 */
void play_poweron(void)
{
	stop_beep();
	start_beep();
	PIN_SET(BEEP_GPIO, BEEP_PIN, HIGH);
    _delay_ms(500);
	PIN_SET(BEEP_GPIO, BEEP_PIN, LOW);
	stop_beep();
    _delay_ms(1000);
	start_beep();
	PIN_SET(BEEP_GPIO, BEEP_PIN, HIGH);
    _delay_ms(500);
	PIN_SET(BEEP_GPIO, BEEP_PIN, LOW);
	stop_beep();
}

void play_poweron_frequency(void)
{
    stop_beep();
    set_beep_frequency(HighFrequency[6]);
    start_beep();
    _delay_ms(500);
    stop_beep();
    _delay_ms(800);
    set_beep_frequency(HighFrequency[6]);
    start_beep();
    _delay_ms(500);
    stop_beep();
}

/*
 * 蜂鸣器任意发声函数
 */
void beep_beeper(_u32 frequency, _u32 delay, _u8 sound)
{
    _delay = delay;
    _startMs = getms();
    stop_beep();
    set_beep_sound(sound);
    set_beep_frequency(frequency);
    start_beep();
}
/*
 * 蜂鸣器延时关闭函数
 */
void heartbeat_beep(float n)
{
    if ((getms() - _startMs) > _delay) {
        stop_beep();
    }
	if (n < 15) {
		if (getms() - _beeper_ticks < 100) {
			return;
		}
		if(_beeper_status){
			start_beep();
			PIN_SET(BEEP_GPIO, BEEP_PIN, HIGH);
			_beeper_status = _beeper_status ? 0 : 1;
			_beeper_ticks = getms();
		}else{
			PIN_SET(BEEP_GPIO, BEEP_PIN, LOW);
			stop_beep();
			_beeper_status = _beeper_status ? 0 : 1;
			_beeper_ticks = getms();
		}
	}else if (n < 30) {
		if(_beeper_status){
			if (getms() - _beeper_ticks < 15000) {
				return;
			}
			start_beep();
			PIN_SET(BEEP_GPIO, BEEP_PIN, HIGH);
			_beeper_status = _beeper_status ? 0 : 1;
			_beeper_ticks = getms();
		}else{
			if (getms() - _beeper_ticks < 500) {
				return;
			}
			PIN_SET(BEEP_GPIO, BEEP_PIN, LOW);
			stop_beep();
			_beeper_status = _beeper_status ? 0 : 1;
			_beeper_ticks = getms();
		}
	}
}

void on_abort_mode(void)
{
	while(1)
	{
		stop_beep();
//		set_beep_frequency(HighFrequency[6]);
		start_beep();
		PIN_SET(BEEP_GPIO, BEEP_PIN, HIGH);
		_delay_ms(1500);
		PIN_SET(BEEP_GPIO, BEEP_PIN, LOW);
		stop_beep();
//		set_beep_frequency(HighFrequency[3]);
		_delay_ms(100);
		start_beep();
		PIN_SET(BEEP_GPIO, BEEP_PIN, HIGH);
		_delay_ms(1500);
		PIN_SET(BEEP_GPIO, BEEP_PIN, LOW);
		stop_beep();
	}
}
