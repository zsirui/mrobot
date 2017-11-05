#pragma once

#include "rpstm32.h"

/**
 @brief input port definitions.
 */
typedef struct _in_port {
    GPIO_TypeDef *port;         /**< input port. */
    _u16          pin;          /**< input pin. */
    _u8           level;        /**< input trigger level. */
} in_port_t;

/**
 @brief output definitions.
 */
typedef struct _out_port {
    GPIO_TypeDef *port;         /**< output port. */
    _u16          pin;          /**< output pin. */
    _u8           level;        /**< output default level. */
} out_port_t;

/**
 @brief pwm control port definitions.
 */
typedef struct _pwm_port {
    GPIO_TypeDef *port;         /**< pwm control port. */
    _u16          pin;          /**< pwm control pin. */
    TIM_TypeDef  *tim;          /**< pwm timer. */
    _u8           tim_ch;       /**< pwm channel. */
} pwm_port_t;

/**
 @brief external interrupt port definitions.
 */
typedef struct _exti_cfg {
    GPIO_TypeDef *port;         /**< EXTI port. */
    _u16          pin;          /**< EXTI pin. */
    _u8           exti;         /**< external interrupt. */
} exti_port_t;

typedef struct _dire_cfg {
    GPIO_TypeDef *port;         /**< direction port. */
    _u16          f_pin;        /**< direction pin. */
	_u16          b_pin;        /**< direction pin. */
} dire_port_t;

_s32 init_board(void);
