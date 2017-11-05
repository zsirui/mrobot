#ifndef __BEEP_H
#define __BEEP_H

#include "common.h"

#define BEEP_GPIO       GPIOA
#define BEEP_PIN        GPIO_Pin_15
#define BEEP_PWM        TIM2

#define TIME_PERIOD 255
#define TIME_PRESCALER (TIME_PERIOD + 1)
#define BEEP_INIT_HZ 1568

//#define BEEP_OPEN() (GPIO_SetBits(GPIOC, GPIO_Pin_12))
//#define BEEP_CLOSE() (GPIO_ResetBits(GPIOC, GPIO_Pin_12))
enum {
    BEEP_START = 1,
    BEEP_STOP = 2,
    BEEP_WORKING = 3,
};
enum {
    BEEP_POWERON = 1,
    BEEP_POWEROFF = 2,
    BEEP_POWERLOW = 3,
    BEEP_POWERCHARGE = 4,
    BEEP_PLAYWORKING = 5,
};
int init_beep(void);
void start_beep(void);
void stop_beep(void);
void play_music(void);
void play_poweron(void);
void play_poweron_frequency(void);
void beep_beeper(_u32 frequency, _u32 delay, _u8 sound);
void heartbeat_beep(float n);
void on_abort_mode(void);
#endif
