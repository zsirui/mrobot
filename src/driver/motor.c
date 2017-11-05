#include "motor.h"
#include "common.h"
#include "stdlib.h"
#include "string.h"
#include "beep.h"
#include "led.h"
#include "exti.h"
#include "watchdog.h"

#define WALKINGMOTOR_CNT       2
#define WALKINGMOTOR_LEFT_ID   0
#define WALKINGMOTOR_RIGHT_ID  1

static _u32 _motorDeltaTicks[WALKINGMOTOR_CNT];           //行走电机的累计编码器值
static _u32  _motorDistAccumulated[WALKINGMOTOR_CNT];     // 累计的行走距离，单位mm.
static float _motorDistTailing[WALKINGMOTOR_CNT];         // 累计的行走距离不到1mm的剩余值，用于下一次累计.

static _u32 _encoderTicksDelta[WALKINGMOTOR_CNT];               //detla时间内的编码器值
static _u32 _lastEncoderTicksDelta[WALKINGMOTOR_CNT];           //最近一次 detla时间内的编码器值
static float _lastOdometerSpeedAbs[WALKINGMOTOR_CNT];           //最近一次 detla时间内的速度值

static _u8 _motorCtrlStates[WALKINGMOTOR_CNT];                  //行走电机方向
static _s32 _motorSpeedMm[WALKINGMOTOR_CNT];                    //行走电机速度

static float speedLastErr[WALKINGMOTOR_CNT];
static float speedErri[WALKINGMOTOR_CNT];
static float speed_PWMOUT[WALKINGMOTOR_CNT];

static _u32 _stallFilterBlindModeTS[WALKINGMOTOR_CNT];
static _u8  _stallBitmap;
static _u32 _stallDetectorTS = 0;
static _u32 _stallDetectorFilter[WALKINGMOTOR_CNT] = {0, 0};

float Kp = 11.7;					//PID 比例因子
float Ki = 1.1;						//PID 积分因子
float Kd = 0.0;						//PID 微分因子

#define PWM_MAX 10000

static const motor_cfg_t _motor_cfg[] = {
	
	{   /**< Left walking motor configure. */
		{MOTOR_PWM_PORT, MOTOR_PWM_L_PIN, GET_TIM(MOTOR_PWM_L_ID), MOTOR_PWM_L_CHN},
		{MOTOR_L_EN_PORT, MOTOR_LF_EN, MOTOR_LB_EN},
		{ENCODER_PORT, LA_ENCODER_PIN, 0},
		{ENCODER_PORT, LB_ENCODER_PIN, 1},
		ODOMETER_EST_PULSE_PER_METER,
	},
	
	{   /**< Right walking motor configure. */
		{MOTOR_PWM_PORT, MOTOR_PWM_R_PIN, GET_TIM(MOTOR_PWM_R_ID), MOTOR_PWM_R_CHN},
		{MOTOR_R_EN_PORT, MOTOR_RF_EN, MOTOR_RB_EN},
		{ENCODER_PORT, RA_ENCODER_PIN, 6},
		{ENCODER_PORT, RB_ENCODER_PIN, 7},
		ODOMETER_EST_PULSE_PER_METER,
	},
};

int InitMotor(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = MOTOR_L_EN_PINs;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MOTOR_L_EN_PORT, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = MOTOR_R_EN_PINs;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MOTOR_R_EN_PORT, &GPIO_InitStructure);
	
	PIN_SET(MOTOR_L_EN_PORT, MOTOR_LB_EN, LOW);
	PIN_SET(MOTOR_L_EN_PORT, MOTOR_LF_EN, LOW);
	PIN_SET(MOTOR_R_EN_PORT, MOTOR_RF_EN, LOW);
	PIN_SET(MOTOR_R_EN_PORT, MOTOR_RB_EN, LOW);
	
	InitPWM();
	init_walkingmotor_odometer();
	return 1;
}

void InitPWM(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = MOTOR_PWM_PINs;
	GPIO_Init(MOTOR_PWM_PORT, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = (PWM_MAX - 1);
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = PWM_MAX;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
	
	set_walkingmotor_lduty(0, MOTOR_CTRL_STATE_RELEASE);
	set_walkingmotor_rduty(0, MOTOR_CTRL_STATE_RELEASE);
	memset(_motorSpeedMm, 0, sizeof(_motorSpeedMm));
	memset(speedLastErr, 0, sizeof(speedLastErr));
	memset(speedErri, 0, sizeof(speedErri));
}

static void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
#ifdef _DOUBLE	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#ifdef _TEST
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
}

static void encoder_l1_exti_cb(void)
{
	++_encoderTicksDelta[WALKINGMOTOR_LEFT_ID];
}

static void encoder_r1_exti_cb(void)
{
	++_encoderTicksDelta[WALKINGMOTOR_RIGHT_ID];
}

#ifdef _DOUBLE
static void encoder_l2_exti_cb(void)
{
	++_encoderTicksDelta[WALKINGMOTOR_LEFT_ID];
}

static void encoder_r2_exti_cb(void)
{
	++_encoderTicksDelta[WALKINGMOTOR_RIGHT_ID];
}
#endif

static void init_extix(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_PINs;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(ENCODER_PORT, &GPIO_InitStructure);
#ifdef _TEST
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);
#endif
	EXTILineConfig();

	NVIC_Configuration();

	GPIO_EXTILineConfig(GPIO_PortSource(_motor_cfg[WALKINGMOTOR_LEFT_ID].encoder1.port), _motor_cfg[WALKINGMOTOR_LEFT_ID].encoder1.exti);
	GPIO_EXTILineConfig(GPIO_PortSource(_motor_cfg[WALKINGMOTOR_RIGHT_ID].encoder1.port), _motor_cfg[WALKINGMOTOR_RIGHT_ID].encoder1.exti);
	exti_reg_callback(_motor_cfg[WALKINGMOTOR_LEFT_ID].encoder1.exti, EXTI_Trigger_Rising_Falling, encoder_l1_exti_cb);
	exti_reg_callback(_motor_cfg[WALKINGMOTOR_RIGHT_ID].encoder1.exti, EXTI_Trigger_Rising_Falling, encoder_r1_exti_cb);

#ifdef _DOUBLE
	GPIO_EXTILineConfig(GPIO_PortSource(_motor_cfg[WALKINGMOTOR_LEFT_ID].encoder2.port), _motor_cfg[WALKINGMOTOR_LEFT_ID].encoder2.exti);
	GPIO_EXTILineConfig(GPIO_PortSource(_motor_cfg[WALKINGMOTOR_RIGHT_ID].encoder2.port), _motor_cfg[WALKINGMOTOR_RIGHT_ID].encoder2.exti);
	exti_reg_callback(_motor_cfg[WALKINGMOTOR_LEFT_ID].encoder2.exti, EXTI_Trigger_Rising_Falling, encoder_l2_exti_cb);
	exti_reg_callback(_motor_cfg[WALKINGMOTOR_RIGHT_ID].encoder2.exti, EXTI_Trigger_Rising_Falling, encoder_r2_exti_cb);
#endif
	
#ifdef _TEST	
	GPIO_EXTILineConfig(GPIO_PortSource(GPIOB), 13);
	exti_reg_callback(13, EXTI_Trigger_Falling, test_button_callback);
#endif

}

#ifdef _TEST
void test_button_callback(void)
{
	disable_watchdog();
	_delay_ms(150);
	stop_beep();
	start_beep();
	PIN_SET(BEEP_GPIO, BEEP_PIN, HIGH);
	drv_led_set(1, 0, 0);
	set_walkingmotor_speed(150, -150);
	speedctl_heartbeat();
	_delay_ms(5000);
	drv_led_set(0, 1, 0);
	PIN_SET(BEEP_GPIO, BEEP_PIN, LOW);
	stop_beep();
	set_walkingmotor_speed(0, 0);
	speedctl_heartbeat();
	_delay_ms(2000);
	start_beep();
	PIN_SET(BEEP_GPIO, BEEP_PIN, HIGH);
	set_walkingmotor_speed(-150, 150);
	speedctl_heartbeat();
	drv_led_set(0, 0, 1);
	_delay_ms(5000);
	PIN_SET(BEEP_GPIO, BEEP_PIN, LOW);
	stop_beep();
	set_walkingmotor_speed(0, 0);
	speedctl_heartbeat();
	drv_led_set(1, 0, 1);
	_delay_ms(2000);
	enable_watchdog();
}
#endif

void init_walkingmotor_odometer(void)
{
	memset(_motorDeltaTicks, 0, sizeof(_motorDeltaTicks));
	memset(_motorDistAccumulated, 0, sizeof(_motorDistAccumulated));
	memset(_motorDistTailing, 0, sizeof(_motorDistTailing));
	
	memset(_encoderTicksDelta, 0, sizeof(_encoderTicksDelta));
	memset(_lastEncoderTicksDelta, 0, sizeof(_lastEncoderTicksDelta));
	memset(_lastOdometerSpeedAbs, 0, sizeof(_lastOdometerSpeedAbs));
	init_extix();
}

static void _set_walkingmotor_duty(_u32 id, _s32 duty, _s32 ctrl)
{
	if (id == WALKINGMOTOR_RIGHT_ID) {
		set_walkingmotor_rduty(duty, ctrl);
	}
	else
	{
		set_walkingmotor_lduty(duty, ctrl);
	} 
}

/*
 * 左行走电机释放函数
 * 不使能PWM输出和控制脚
 */
static void _release_walkingmotor_l()
{
	PIN_SET(MOTOR_PWM_PORT, MOTOR_PWM_L_PIN, LOW);
	pinMode(MOTOR_PWM_PORT, MOTOR_PWM_L_PIN, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);

	_delay_us(20);              //等效于死区

	PIN_SET(MOTOR_L_EN_PORT, MOTOR_LB_EN, LOW);
	PIN_SET(MOTOR_L_EN_PORT, MOTOR_LF_EN, LOW);
}
/*
 * 右行走电机释放函数
 * 不使能PWM输出和控制脚
 */
static void _release_walkingmotor_r()
{
	PIN_SET(MOTOR_PWM_PORT, MOTOR_PWM_R_PIN, LOW);
	pinMode(MOTOR_PWM_PORT, MOTOR_PWM_R_PIN, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);

	_delay_us(20);              //等效于死区

	PIN_SET(MOTOR_R_EN_PORT, MOTOR_RF_EN, LOW);
	PIN_SET(MOTOR_R_EN_PORT, MOTOR_RB_EN, LOW);
}
/*
 * 设置左行走电机占空比和方向函数
 */
void set_walkingmotor_lduty(int dutyCycle, int ctrl)
{
	dutyCycle = abs(dutyCycle);
	if (_motorCtrlStates[WALKINGMOTOR_LEFT_ID] != ctrl) {
		_release_walkingmotor_l();                                              //方向变化，先停止电机
	}
	if (ctrl == MOTOR_CTRL_STATE_RELEASE || ctrl == MOTOR_CTRL_STATE_BRAKE) {
		dutyCycle = 0;
	} else {
		if (dutyCycle > PWM_MAX)
			dutyCycle = PWM_MAX;
	}
	RAW_PWM_SET(MOTOR_PWM_L_CHN, MOTOR_PWM_L_ID, PWM_MAX - dutyCycle);			//设定PWM占空比=(TIM3_CCR4/ TIM3_ARR)*100

	switch (ctrl) {                                                             //根据设定方向，进行变向或停止
	case MOTOR_CTRL_STATE_RELEASE:
		break;
	case MOTOR_CTRL_STATE_BRAKE:
		PIN_SET(MOTOR_L_EN_PORT, MOTOR_LB_EN, LOW);
		PIN_SET(MOTOR_L_EN_PORT, MOTOR_LF_EN, LOW);
		break;
	case MOTOR_CTRL_STATE_FORWARD:
		PIN_SET(MOTOR_L_EN_PORT, MOTOR_LF_EN, HIGH);
		PIN_SET(MOTOR_L_EN_PORT, MOTOR_LB_EN, LOW);
		break;
	case MOTOR_CTRL_STATE_BACKWARD:
		PIN_SET(MOTOR_L_EN_PORT, MOTOR_LB_EN, HIGH);
		PIN_SET(MOTOR_L_EN_PORT, MOTOR_LF_EN, LOW);
		break;
	}
	
	if (dutyCycle && (_motorCtrlStates[WALKINGMOTOR_LEFT_ID] != ctrl)) {
		pinMode(MOTOR_PWM_PORT, MOTOR_PWM_L_PIN, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	}
	_motorCtrlStates[WALKINGMOTOR_LEFT_ID] = ctrl;
}
/*
 * 设置右行走电机占空比和方向函数
 */
void set_walkingmotor_rduty(_s32 dutyCycle,_s32 ctrl)
{
	dutyCycle = abs(dutyCycle);
	if (_motorCtrlStates[WALKINGMOTOR_RIGHT_ID] != ctrl) {
		_release_walkingmotor_r();                                              //方向变化，先停止电机
	}
	if (ctrl == MOTOR_CTRL_STATE_RELEASE || ctrl == MOTOR_CTRL_STATE_BRAKE) {
		dutyCycle = 0;
	} else {
		if (dutyCycle > PWM_MAX)
			dutyCycle = PWM_MAX;
	}
	RAW_PWM_SET(MOTOR_PWM_R_CHN, MOTOR_PWM_R_ID, PWM_MAX - dutyCycle);			//设定PWM占空比=(TIM3_CCR4/ TIM3_ARR)*100
	switch (ctrl) {                                                             //根据设定方向，进行变向或停止
	case MOTOR_CTRL_STATE_RELEASE:
		break;
	case MOTOR_CTRL_STATE_BRAKE:
		PIN_SET(MOTOR_R_EN_PORT, MOTOR_RB_EN, LOW);
		PIN_SET(MOTOR_R_EN_PORT, MOTOR_RF_EN, LOW);
		break;
	case MOTOR_CTRL_STATE_FORWARD:
		PIN_SET(MOTOR_R_EN_PORT, MOTOR_RF_EN, LOW);
		PIN_SET(MOTOR_R_EN_PORT, MOTOR_RB_EN, HIGH);
		break;
	case MOTOR_CTRL_STATE_BACKWARD:
		PIN_SET(MOTOR_R_EN_PORT, MOTOR_RB_EN, LOW);
		PIN_SET(MOTOR_R_EN_PORT, MOTOR_RF_EN, HIGH);
		break;
	}
	if (dutyCycle && (_motorCtrlStates[WALKINGMOTOR_RIGHT_ID] != ctrl)) {
		pinMode(MOTOR_PWM_PORT, MOTOR_PWM_R_PIN, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	}
	_motorCtrlStates[WALKINGMOTOR_RIGHT_ID] = ctrl;
}

static void _refresh_walkingmotor_odometer(_u32 durationMs)
{
	_u8 cnt;
	float dist_mm;
	_u32 irqSave = enter_critical_section();                                    //临界资源保护
	for (cnt = 0; cnt < WALKINGMOTOR_CNT; ++cnt) {                       		//得到这段时间内的编码器数据
		_lastEncoderTicksDelta[cnt] = _encoderTicksDelta[cnt];
		_encoderTicksDelta[cnt] = 0;
	}
	leave_critical_section(irqSave);

	if (durationMs == 0)                                                        //防止除零
		durationMs = 1;

	for (cnt = 0; cnt < WALKINGMOTOR_CNT; ++cnt) {                       		//根据这段时间内的编码器数据计算这段时间内速度，即当前速度
		dist_mm = (float)_lastEncoderTicksDelta[cnt] * (1000.0 / ODOMETER_EST_PULSE_PER_METER);
		
		_lastOdometerSpeedAbs[cnt] = dist_mm * 1000.0 / durationMs;
			
		dist_mm += _motorDistTailing[cnt];
		_motorDistAccumulated[cnt] += (_u32)dist_mm;
		_motorDistTailing[cnt] = dist_mm - (_u32)dist_mm;

		_motorDeltaTicks[cnt] += _lastEncoderTicksDelta[cnt];
	}

}

static void _control_walkingmotor_speed_byid(int id)
{
	int desiredSpdAbs;
	float speedCurrentErr;
	float speedCurrentErrd;
	const float PWM_OUT_MAX = PWM_MAX;

	if (_motorSpeedMm[id] == 0) {                                               //设定速度为0，则立即停止行走电机
		_set_walkingmotor_duty(id, 0, MOTOR_CTRL_STATE_BRAKE);
	} else {
		int desiredCtrl = (_motorSpeedMm[id] > 0) ? MOTOR_CTRL_STATE_FORWARD : MOTOR_CTRL_STATE_BACKWARD;
		
		if (desiredCtrl != _motorCtrlStates[id]) {                              //方向改变，则先停止行走电机
			if (_lastOdometerSpeedAbs[id] > 1.0f) {
				_set_walkingmotor_duty(id, 0, MOTOR_CTRL_STATE_BRAKE);
				return;
			}
			speedLastErr[id] = 0;
			speedErri[id] = 0;
		}

		desiredSpdAbs = abs(_motorSpeedMm[id]);
		speedCurrentErr = (float) desiredSpdAbs - _lastOdometerSpeedAbs[id];
		speedCurrentErrd = speedCurrentErr - speedLastErr[id];
		speedErri[id] += speedCurrentErr;
		speedLastErr[id] = speedCurrentErr;
		speed_PWMOUT[id] = (Kp * speedCurrentErr + Ki * speedErri[id] + Kd * speedCurrentErrd); //PID计算下一个PWM占空比值

		if (speed_PWMOUT[id] > PWM_OUT_MAX)
			speed_PWMOUT[id] = PWM_OUT_MAX;
		if (speed_PWMOUT[id] < 0)
			speed_PWMOUT[id] = 0;
		_set_walkingmotor_duty(id, (int) speed_PWMOUT[id], desiredCtrl);            //将PID计算得到的PWM占空比值设定
	}
}

/*
 * 获得左电机速度，单位：mm/s
 */
static _s32 get_walkingmotor_lspeed_set(void)
{
	return _motorSpeedMm[WALKINGMOTOR_LEFT_ID];
}
/*
 * 获得右电机速度，单位：mm/s
 */
static _s32 get_walkingmotor_rspeed_set(void)
{
	return _motorSpeedMm[WALKINGMOTOR_RIGHT_ID];
}

/*
 * 计算左行走电机当前速度函数
 * 单位：mm/s
 */
float get_walkingmotor_lspeed_mm(void)
{
	if (get_walkingmotor_lspeed_set() >= 0) return _lastOdometerSpeedAbs[WALKINGMOTOR_LEFT_ID];
	else return -_lastOdometerSpeedAbs[WALKINGMOTOR_LEFT_ID];
}
/*
 * 计算右行走电机当前速度函数
 * 单位：mm/s
 */
float get_walkingmotor_rspeed_mm(void)
{
	if (get_walkingmotor_rspeed_set() >= 0) return _lastOdometerSpeedAbs[WALKINGMOTOR_RIGHT_ID];
	else return -_lastOdometerSpeedAbs[WALKINGMOTOR_RIGHT_ID];
}

/*
 * 左右行走电机速度控制函数
 */
void control_walkingmotor_speed(void)
{
	size_t id;
	for (id = 0; id < WALKINGMOTOR_CNT; ++id) {
		_control_walkingmotor_speed_byid(id);
	}
}
/*
 * 设定左右电机速度，单位：mm/s
 */
void set_walkingmotor_speed(_s32 lSpeed, _s32 rSpeed)
{
	_motorSpeedMm[WALKINGMOTOR_LEFT_ID] = lSpeed;
	_motorSpeedMm[WALKINGMOTOR_RIGHT_ID] = rSpeed;
}

static _u32 speedctl_frequency = 0;
/*
 * 行走电机速度控制和反馈编码检测函数
 */
void speedctl_heartbeat(void)
{
	_u32 currentTs = getms();
	_u32 delta = currentTs - speedctl_frequency;

	if (delta >= CONF_MOTOR_HEARTBEAT_DURATION) {
		speedctl_frequency = currentTs;
		_refresh_walkingmotor_odometer(delta);                  //定时获取反馈编码值
		control_walkingmotor_speed();                           //进而进行速度控制
	}
}

void stalldetector_init(void)
{
	memset(_stallFilterBlindModeTS, 0, sizeof(_stallFilterBlindModeTS));
	memset(_stallDetectorFilter, 0, sizeof(_stallDetectorFilter));
	_stallDetectorTS = 0;
	_stallBitmap = 0;
}

static _u8 countbit (_u32 x) {
	_u32 n;

	n = (x >> 1) & 033333333333;
	x = x - n;
	n = (n >> 1) & 033333333333;
	x = x - n;
	x = (x + (x >> 3)) & 030707070707;
	return (_u8)(x % 63);
}

void stalldetector_enterBlindMode(_u8 id)
{
	_stallFilterBlindModeTS[id] = getms();
}

/*
 * 速度堵转检测
 */
void stalldetector_heartbeat(void)
{
	_u8 id;
	_u8 n;

	if (getms() - _stallDetectorTS < CONF_MOTOR_HEARTBEAT_DURATION) {
		return ;
	}
	_stallDetectorTS = getms();

	for (id = 0; id < WALKINGMOTOR_CNT; id++) {
		if (getms() - _stallFilterBlindModeTS[id] < CONF_MOTOR_STALL_BLINDMODE_DURATION) {
			_stallDetectorFilter[id] = 0;
			_stallBitmap = 0;
			continue;
		}

		_stallDetectorFilter[id] <<= 1;
		if (speed_PWMOUT[id] > 0 && _lastOdometerSpeedAbs[id] < 1.0f) {
			_stallDetectorFilter[id] |= 0x01;
		} else {
			_stallDetectorFilter[id] &= ~0x01;
		}

		n = countbit(_stallDetectorFilter[id]);
		if (n > 25) {
			/* Stall is detected. */
			_stallBitmap |= (1 << id);
//			beep_beeper(4000, 100, 2);
		} else {
			/* No stall, clean flag. */
			_stallBitmap &= ~(1 << id);
		}
	}
}
/*
 * 获取堵转映射成碰撞状态
 */
_u8 stalldetector_is_bumped(void)
{
	_u8 id;
	_u8 bump_flag = 0xff;

	for (id = 0; id < WALKINGMOTOR_CNT; id++) {
		if (_stallBitmap & (1 << id)) {
			bump_flag &= ~(1 << id);
		}
	}
	return bump_flag;
}
/*
 * 获取堵转状态
 */
_u8 stalldetector_is_stalled(void)
{
	return _stallBitmap;
}
