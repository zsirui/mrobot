#pragma once

#ifndef RPSTM32_HEADER_H
#define RPSTM32_HEADER_H

#define EXPAND_WRAPPER( __NEXTLEVEL__, ...)  __NEXTLEVEL__( __VA_ARGS__ )

// Helper Macros

#define _GET_ADC_PERIPH(x) RCC_APB2Periph_ADC##x
#define _GET_TIM(x)   TIM##x
#define GET_ADC_PERIPH(x)  EXPAND_WRAPPER(_GET_ADC_PERIPH,x)
#define GET_TIM(x)    EXPAND_WRAPPER(_GET_TIM,x)

// ST-FWlib
#include "stm32f10x.h"

// Lib-c
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "types.h"

// Board Def
#include "boarddef.h"

// Board HAL
#include "boardfunc.h"

#endif // #ifndef RPSTM32_HEADER_H
