#ifndef __EXTI_H
#define __EXTI_H

#include "common.h"
#include "types.h"
#include <stdio.h>

#define CONFIG_EXTI_MAX_NUM             16  /**< Maximum external line channels. */
#define CONFIG_EXIT_IRQ_NUM             7   /**< External irq number, irq EXTI0~EXTI4, EXTI9-5, EXTI15-10 */

/**
 @brief get gpio port source by port id.
 @param port - gpio port id.
 @return return gpio source number.
 */
static _u8 GPIO_PortSource(GPIO_TypeDef *port)
{
    if (port == NULL) {
        return 0;
    }
    if (port == GPIOA) {
        return GPIO_PortSourceGPIOA;
    } else if (port == GPIOB) {
        return GPIO_PortSourceGPIOB;
    } else if (port == GPIOC) {
        return GPIO_PortSourceGPIOC;
    } else if (port == GPIOD) {
        return GPIO_PortSourceGPIOD;
    } else if (port == GPIOE) {
        return GPIO_PortSourceGPIOE;
    } else if (port == GPIOF) {
        return GPIO_PortSourceGPIOF;
    } else if (port == GPIOG) {
        return GPIO_PortSourceGPIOG;
    } else {
        return 0;
    }
}

#define EXTI_LINE(l)        (EXTI_Line0 << (l))

/**
 @brief External line interrupt call back function.
 */
typedef void (*exti_cb_t)(void);

bool exti_reg_callback(_u8 exti, EXTITrigger_TypeDef type, exti_cb_t cb);
void exti_unreg_callback(_u8 exti);

#endif
