#include "common.h"

_u32 _ticks_per_us = 8; //true for 72Mhz with 2cycles' flash delay
#define CALIBRATION_TICKS 500000UL

static void _delay_loop(volatile _u32 count)
{
    while(count--);
}
/*
 * 微秒级延时矫正函数
 */
void softdelay_calibrate()
{
	_u64 usedTime;
    _u64 startUs = getus();
    _delay_loop(CALIBRATION_TICKS);
    usedTime = getus() - startUs;

    if (!usedTime) usedTime = 1;

    _ticks_per_us = CALIBRATION_TICKS/usedTime;
    if (!_ticks_per_us) _ticks_per_us = 1;
}
/*
 * 微秒级延时函数
 */
void _delay_us(volatile uint32_t us)
{
    _delay_loop(us * _ticks_per_us);
}
