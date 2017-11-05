#pragma once
#include "common.h"

#define 	MOTOR_PWM_PORT			GPIOA
#define 	MOTOR_PWM_L_PIN			GPIO_Pin_10
#define 	MOTOR_PWM_L_ID			1
#define 	MOTOR_PWM_L_CHN			3

#define 	MOTOR_PWM_R_PIN			GPIO_Pin_11
#define 	MOTOR_PWM_R_ID			1
#define 	MOTOR_PWM_R_CHN			4
#define 	MOTOR_PWM_PINs			MOTOR_PWM_L_PIN | MOTOR_PWM_R_PIN

#define 	MOTOR_L_EN_PORT			GPIOC
#define 	MOTOR_LF_EN				GPIO_Pin_6
#define 	MOTOR_LB_EN				GPIO_Pin_7
#define 	MOTOR_L_EN_PINs			MOTOR_LF_EN | MOTOR_LB_EN

#define 	MOTOR_R_EN_PORT			GPIOC
#define 	MOTOR_RF_EN				GPIO_Pin_8
#define 	MOTOR_RB_EN				GPIO_Pin_9
#define 	MOTOR_R_EN_PINs			MOTOR_RF_EN | MOTOR_RB_EN

#define 	ENCODER_PORT			GPIOA
#define 	RA_ENCODER_PIN			GPIO_Pin_6
#define 	RB_ENCODER_PIN			GPIO_Pin_7
#define 	LA_ENCODER_PIN			GPIO_Pin_0
#define 	LB_ENCODER_PIN			GPIO_Pin_1

#ifdef _DOUBLE
#define		EXTI_LINEs				EXTI_Line0 | EXTI_Line1 | EXTI_Line6 | EXTI_Line7
#define		GPIO_PINs				LA_ENCODER_PIN | LB_ENCODER_PIN | RA_ENCODER_PIN | RB_ENCODER_PIN
#define		EXTILineConfig()		GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0); \
									GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1); \
									GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource6); \
									GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource7)
#define		ODOMETER_EST_PULSE_PER_METER	7648UL
#else
#define		EXTI_LINEs				EXTI_Line0 | EXTI_Line6
#define		GPIO_PINs				LA_ENCODER_PIN | RA_ENCODER_PIN
#define		EXTILineConfig()		GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);	\
									GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource6)
#define		ODOMETER_EST_PULSE_PER_METER	3824UL
#endif

//行走电机速度控制频率：60hz
#define CONF_MOTOR_HEARTBEAT_FREQ     100
#define CONF_MOTOR_HEARTBEAT_DURATION (1000/(CONF_MOTOR_HEARTBEAT_FREQ))

#define CONF_MOTOR_STALL_BLINDMODE_DURATION 600

typedef struct _motor_cfg {
	pwm_port_t  pwm;         /**< Forward control pwm port. */
	dire_port_t direction;         /**< Backward control pwm port. */
	exti_port_t encoder1;       /**< Motor encoder odometer port. */
	exti_port_t encoder2;       /**< Motor encoder odometer port. */
	_u16        speed_factor;   /**< Odometer speed factor, in pulse per meter. */
} motor_cfg_t;

enum motorCtrlState_t {
    MOTOR_CTRL_STATE_RELEASE = 0,
    MOTOR_CTRL_STATE_FORWARD = 1,
    MOTOR_CTRL_STATE_BACKWARD = 2,
    MOTOR_CTRL_STATE_BRAKE = 3,
};

void InitPWM(void);
int InitMotor(void);
void test_button_callback(void);
void init_walkingmotor_odometer(void);
float get_walkingmotor_lspeed_mm(void);
float get_walkingmotor_rspeed_mm(void);
void set_walkingmotor_lduty(int dutyCycle, int ctrl);
void set_walkingmotor_rduty(_s32 dutyCycle,_s32 ctrl);
void control_walkingmotor_speed(void);
void set_walkingmotor_speed(_s32 lSpeed, _s32 rSpeed);
void brake_walkingmotor(void);
void speedctl_heartbeat(void);
void stalldetector_init(void);
_u8 stalldetector_is_bumped(void);
_u8 stalldetector_is_stalled(void);
void stalldetector_heartbeat(void);
void stalldetector_enterBlindMode(_u8 id);
