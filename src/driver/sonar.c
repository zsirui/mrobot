#include "common.h"
#include "filters.h"
#include "exti.h"
#include "led.h"
#include "sonar.h"
//#include <stdio.h>

/**
 @defgroup  sonar ultrasonic sonar module.
 @addtogroup drivers
 @{

  Resource consumption: GPIOE 5, 7 ~ 12, 15.
  EXTI line 5,7,8,9 with interrupt.
  TIM6 as counter timer.
 */
#define SONAR_TIMER         TIM6

static uint8_t g_sonar_ch = 0;  /**< Active sonar channel index. */

/**
 @brief Global sonar channel descriptors.
 */
static sonar_channel_t g_sonar[CONFIG_SONAR_CHANNEL_NUM];

/**
 @brief Global sonar channel configurations.
 */
static const sonar_cfg_t g_sonar_cfg[] = {
    {SONAR_TRIG1_PORT, SONAR_TRIG1_PIN, SONAR_ECHO1_PORT, SONAR_ECHO1_PIN, 5},
    {SONAR_TRIG2_PORT, SONAR_TRIG2_PIN, SONAR_ECHO2_PORT, SONAR_ECHO2_PIN, 7},
    {SONAR_TRIG3_PORT, SONAR_TRIG3_PIN, SONAR_ECHO3_PORT, SONAR_ECHO3_PIN, 8},
    {SONAR_TRIG4_PORT, SONAR_TRIG4_PIN, SONAR_ECHO4_PORT, SONAR_ECHO4_PIN, 9},
};

/**
 @brief Sonar echo read.
 @param ch    - sonar channel index, 0 ~ CONFIG_SONAR_CHANNEL_NUM.
 @return none.
 */
static inline uint8_t SONAR_ECHO(uint8_t ch)
{
    return GPIO_ReadInputDataBit(g_sonar_cfg[ch].echo_port, g_sonar_cfg[ch].echo_pin);
}

/**
 @brief Sonar trigger function.
 @param ch    - sonar channel index, 1 ~ CONFIG_SONAR_CHANNEL_NUM-1.
 @param level - sonar trigger level.
 @return none.
 */
static inline void SONAR_TRIG(uint8_t ch, uint8_t level)
{
    if (level == HIGH) {
        GPIO_SetBits(g_sonar_cfg[ch].trig_port, g_sonar_cfg[ch].trig_pin);
    } else {
        GPIO_ResetBits(g_sonar_cfg[ch].trig_port, g_sonar_cfg[ch].trig_pin);
    }
}

static void sonar_exti_cb(void)
{
    uint8_t  id;
    uint8_t  status;
    TIM_TimeBaseInitTypeDef tim_base;

    status = SONAR_ECHO(g_sonar_ch);
    if (status == HIGH) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
        /* Start counter timer. */
        tim_base.TIM_Period = 60000;
        tim_base.TIM_Prescaler = (SYSTICK_1MS_TICKS/1000-1);
        tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
        tim_base.TIM_CounterMode = TIM_CounterMode_Up;

        TIM_TimeBaseInit(SONAR_TIMER, &tim_base);
        TIM_SetCounter(SONAR_TIMER, 0);
        TIM_Cmd(SONAR_TIMER, ENABLE);
    } else {
        /* Stop counter timer. */
        TIM_Cmd(SONAR_TIMER, DISABLE);

        id = g_sonar[g_sonar_ch].id;
        if (id >= CONFIG_SONAR_SAMPLE_SIZE) {
            id = 0;
        }
        g_sonar[g_sonar_ch].sample[id++] = TIM_GetCounter(SONAR_TIMER);
        g_sonar[g_sonar_ch].state++;    /* Move to next state. */
        g_sonar[g_sonar_ch].id = id;
        if (g_sonar[g_sonar_ch].cnt < CONFIG_SONAR_SAMPLE_SIZE) {
            g_sonar[g_sonar_ch].cnt++;
        }
    }
}

/**
 @brief Trigger a sonar channel.
 @param ch - channel number to be triggered, 1 ~ CONFIG_SONAR_CHANNEL_NUM.
 @return none.

 */
static void sonar_trigger(uint8_t ch)
{
    if (ch >= _countof(g_sonar_cfg)) {
        return ;
    }

    /* Send trigger wave. */
    SONAR_TRIG(ch, HIGH);
    _delay_us(20);
    SONAR_TRIG(ch, LOW);

    /* Prepare for echo rising edge interrupt. */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, g_sonar_cfg[ch].exti_line);
    exti_reg_callback(g_sonar_cfg[ch].exti_line, EXTI_Trigger_Rising_Falling, sonar_exti_cb);
    return ;
}

/**
 @brief Shutdown a sonar channel.
 @param ch - channel number to be shutdown, 0 ~ CONFIG_SONAR_CHANNEL_NUM.
 @return none.

 */
static void sonar_shutdown(uint8_t ch)
{
    if (ch >= _countof(g_sonar_cfg)) {
        return ;
    }

    SONAR_TRIG(ch, LOW);
    exti_unreg_callback(g_sonar_cfg[ch].exti_line);
    return ;
}

/**
 @brief Calculate sonar distance by sample value.
 @param ch - sonar channel, 0 ~ CONFIG_SONAR_CHANNEL_NUM
 @return none.

 This distance is in mm. 
 */
float sonar_distance(uint8_t ch)
{
    uint8_t  i;
    uint32_t avg;
    float    d;

    if (ch >= _countof(g_sonar_cfg)) {
        return -1;
    }

    avg = 0;
    for (i = 0; i < g_sonar[ch].cnt; i++) {
        avg += g_sonar[ch].sample[i];
    }
    if (i > 0) {
        avg /= i;
    } else {
        avg = 0;
    }

    d = CONFIG_SONAR_COE_A + CONFIG_SONAR_COE_B * (float)(Temperature / 100);
    d *= avg / 2 / 1000.0;
    g_sonar[ch].distance = (float)d;
	return d;
}

/**
 @brief ultrasonic sonar module heartbeat.
 @param None.
 @return None.
            ___                                                         ___
 TRIG   ___|   |_______________________________________________________|   |____
            >10us _   _   _   _   _   _   _   _          at least 10ms wait
 SENSOR _________| |_| |_| |_| |_| |_| |_| |_| |________________________________
                      Send 8 40KHz wave
                                                   ________
 ECHO   __________________________________________|        |____________________
                                                  10us ~ 18ms
 */
void sonar_heartbeat(void)
{
    if (g_sonar_ch >= _countof(g_sonar_cfg)) {
        g_sonar_ch = 0;
        g_sonar[g_sonar_ch].state = SONAR_INIT;
    }

    switch (g_sonar[g_sonar_ch].state) {
    case SONAR_INIT:
        sonar_trigger(g_sonar_ch);
        g_sonar[g_sonar_ch].state++;    /* Move to next state. */
        g_sonar[g_sonar_ch].ticks = getms();
        break;
    case SONAR_MEASURE: /* Wait until measurement done. */
        if (getms() - g_sonar[g_sonar_ch].ticks > CONFIG_SONAR_TIMEOUT_MS) {
            /* Timeout. Abort measurement and move to next channel. */
//			_SONAR = -1;
            TIM_Cmd(SONAR_TIMER, DISABLE);
            sonar_shutdown(g_sonar_ch);
            g_sonar[g_sonar_ch].ticks = getms();
            g_sonar[g_sonar_ch].state = SONAR_IDLE;
			memset(g_sonar[g_sonar_ch].sample, 0, sizeof(g_sonar[g_sonar_ch].sample));
            g_sonar_ch++;
            if (g_sonar_ch >= _countof(g_sonar_cfg)) {
                g_sonar_ch = 0;
            }
//            drv_led_set(0, 0, 0);
        }
        break;
    case SONAR_DONE:    /* Measurement is done. */
        if (g_sonar[g_sonar_ch].cnt == 0) {
            break;
        }
        if (g_sonar[g_sonar_ch].cnt >= CONFIG_SONAR_SAMPLE_SIZE) {
            g_sonar[g_sonar_ch].cnt = CONFIG_SONAR_SAMPLE_SIZE;
        }
//        printf("sonar distance: %.4f", sonar_distance(g_sonar_ch));
        g_sonar[g_sonar_ch].state++;
        /* Blink the led by channel. */
//        drv_led_set((g_sonar_ch%3)==0?1:0, (g_sonar_ch%3)==1?1:0, (g_sonar_ch%3)==2?1:0);
//        sonar_dbg("ch %d, distance %d\r\n", g_sonar_ch, g_sonar[g_sonar_ch].distance);
        break;
    case SONAR_EXIT:    /* Channel measurement is done. Move to next. */
        sonar_shutdown(g_sonar_ch);
        g_sonar[g_sonar_ch].ticks = getms();
        g_sonar[g_sonar_ch].state = SONAR_IDLE;
        g_sonar_ch++;
        if (g_sonar_ch >= _countof(g_sonar_cfg)) {
            g_sonar_ch = 0;
        }
        break;
    default:
        if (getms() - g_sonar[g_sonar_ch].ticks < CONFIG_SONAR_TICKS) {
            break;
        }
        g_sonar[g_sonar_ch].state = SONAR_INIT;
        break;
    }

    return ;
}

/**
 @brief Get Sonar channel distance measurement value.
 @param ch - Sonar channel number, 1 ~ CONFIG_SONAR_CHANNEL_NUM.
 @return return disatance measured in mm.
 */
uint32_t sonar_get(uint8_t ch)
{
    if (ch >= _countof(g_sonar_cfg)) {
        return 0;
    }

    sonar_distance(ch);
    return g_sonar[ch].distance;
}

/**
 @brief Initialize ultrasonic sonar module.
 @param None.
 @return None.
 */
int sonar_init(void)
{
    uint8_t ch;

    /* These pins are pull up by default. So pull down them. */
    for (ch = 0; ch < _countof(g_sonar_cfg); ch++) {
        pinMode(g_sonar_cfg[ch].echo_port, g_sonar_cfg[ch].echo_pin,
                GPIO_Mode_IPD, GPIO_Speed_10MHz);
        pinMode(g_sonar_cfg[ch].trig_port, g_sonar_cfg[ch].trig_pin,
                GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
        SONAR_TRIG(ch, 0);
    }

    /* Enable line5 ~ line9 external interrupt. */

    memset(g_sonar, 0, sizeof(g_sonar));
	return 1;
}

/**
 @brief Clean ultrasonic sonar module.
 @param None.
 @return None.
 */
void sonar_exit(void)
{
    uint8_t ch;

    for (ch = 0; ch < _countof(g_sonar_cfg); ch++) { 
        SONAR_TRIG(ch, 0);
    }

    /* Disable line5 ~ line9 external interrupt. */

    memset(g_sonar, 0, sizeof(g_sonar));
}
