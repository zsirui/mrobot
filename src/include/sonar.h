#include <stdio.h>
#include <stdint.h>
#include "common.h"

/**
 @addtogroup sonar
 @{
 */

#ifndef _SONAR_H
#define _SONAR_H
extern short Temperature;
//extern int _SONAR;
/**
 @brief ultrasonic speed coefficient is different by temperature.

    Tempearture  -30  -20 -10  0   10  20  30 100
    Speed(m/s)   313 319  325 332 338 344 349 386

    V = 331.45 + 0.607 * T
    S = V * t / 2
  FIXME: use tempearture sensor to select speed coefficient.
 */
#define CONFIG_SONAR_COE_A              331.45  /*< Sonar distance coefficient A. */
#define CONFIG_SONAR_COE_B              0.607   /*< Sonar distance coefficient B. */
//#define CONFIG_SONAR_COE_T              20      /*< Sonar distance coefficient T at 20. */

#define CONFIG_SONAR_CHANNEL_NUM        4       /**< Sonar channel number. */
#define CONFIG_SONAR_SAMPLE_SIZE        4       /**< Sonar sample buffer size. */
#define CONFIG_SONAR_TIMEOUT_MS         50      /**< Sonar detection timeout in ms. */
#define CONFIG_SONAR_TICKS              100     /**< Sonar detection timeout in ms. */

#define CONFIG_SONAR_BLOCK_DIST         100     /**< Sonar block distance in mm. */

#define			SONAR_TRIG1_PORT		GPIOB
#define			SONAR_TRIG1_PIN			GPIO_Pin_0

#define			SONAR_ECHO1_PORT		GPIOB
#define			SONAR_ECHO1_PIN			GPIO_Pin_1

#define			SONAR_TRIG2_PORT		GPIOB
#define			SONAR_TRIG2_PIN			GPIO_Pin_3

#define			SONAR_ECHO2_PORT		GPIOB
#define			SONAR_ECHO2_PIN			GPIO_Pin_4

#define			SONAR_TRIG3_PORT		GPIOB
#define			SONAR_TRIG3_PIN			GPIO_Pin_8

#define			SONAR_ECHO3_PORT		GPIOB
#define			SONAR_ECHO3_PIN			GPIO_Pin_9

#define			SONAR_TRIG4_PORT		GPIOB
#define			SONAR_TRIG4_PIN			GPIO_Pin_14

#define			SONAR_ECHO4_PORT		GPIOB
#define			SONAR_ECHO4_PIN			GPIO_Pin_15

#if !defined(_countof)
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))
#endif

/**
 @brief sonar channel info structure.
 */
typedef struct _sonar_channel {
    uint8_t  state;                             /**< Sonar sensor state. */
    uint8_t  id;                                /**< Sonar smaple buffer id. */
    uint8_t  cnt;                               /**< Sonar smaple count. */
    uint16_t sample[CONFIG_SONAR_SAMPLE_SIZE];  /**< Sonar sample buffer. */
    uint32_t ticks;                             /**< Sonar sample ticks. */
    float distance;                          /**< Sonar sampel result. */
} sonar_channel_t;

/**
 @brief sonar channel configuration.
 */
typedef struct _sonar_cfg {
    GPIO_TypeDef *trig_port;        /**< Sonar channel trigger IO port. */
    uint16_t      trig_pin;         /**< Sonar channel tirgger IO pin. */
    GPIO_TypeDef *echo_port;        /**< Sonar channel echo input IO port. */
    uint16_t      echo_pin;         /**< Sonar channel echo input IO pin. */
    uint32_t      exti_line;        /**< Sonar channel echo input interrupt line. */
} sonar_cfg_t;

/**
 @brief Sonar sensor states.
 */
enum _sonar_state {
    SONAR_IDLE = 0,
    SONAR_INIT,
    SONAR_MEASURE,
    SONAR_DONE,
    SONAR_EXIT,
};

int sonar_init(void);
void sonar_exit(void);
void sonar_heartbeat(void);
uint32_t sonar_get(uint8_t ch);
float sonar_distance(uint8_t ch);

#endif

/** @} */
