#include "watchdog.h"
/*
 * 看门狗使能函数
 * 打开看门狗
 */
void enable_watchdog()
{
    RCC_ClearFlag();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
    WWDG_SetPrescaler(WWDG_Prescaler_8);
    WWDG_SetWindowValue(127);
    WWDG_Enable(127);
}
/*
 * 看门狗禁止函数
 * 关闭看门狗
 */
void disable_watchdog()
{
    mark_watchdog();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, DISABLE);
}
/*
 * 喂狗函数
 */
void mark_watchdog()
{
    WWDG_SetCounter(127);
}
