#ifndef _LED_H_
#define _LED_H_

#ifdef _RGB
#define 		LED_COLOR_PORT			GPIOA
#define 		LED_COLOR_RED			GPIO_Pin_8
#define 		LED_COLOR_GREEN			GPIO_Pin_9
#define 		LED_COLOR_BLUE			GPIO_Pin_12
#define 		LED_COLOR_PINs			LED_COLOR_RED | LED_COLOR_GREEN | LED_COLOR_BLUE
#else
#define 		LED_COLOR_PORT			GPIOB
#define 		LED_COLOR_PIN			GPIO_Pin_12
#endif

int drv_led_init(void);
void drv_led_shutdown(void);
void drv_led_set(int r, int g, int b);
void led_heartbeat(float n);
#endif
