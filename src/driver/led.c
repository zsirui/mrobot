#include "common.h"
#include "led.h"
/**
 @defgroup led led driver
 @addtogroup drivers
 @{
 */
#ifdef _RGB
int drv_led_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = LED_COLOR_PINs;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(LED_COLOR_PORT, &GPIO_InitStructure);
	
	PIN_SET(LED_COLOR_PORT, LED_COLOR_RED, 0);
	PIN_SET(LED_COLOR_PORT, LED_COLOR_GREEN, 0);
	PIN_SET(LED_COLOR_PORT, LED_COLOR_BLUE, 0);
    drv_led_set(0, 0, 0);
	return 1;
}

void drv_led_set(int r, int g, int b)
{
	if(r) PIN_SET(LED_COLOR_PORT, LED_COLOR_RED, 0);
	else PIN_SET(LED_COLOR_PORT, LED_COLOR_RED, 1);
	if(g) PIN_SET(LED_COLOR_PORT, LED_COLOR_GREEN, 0);
	else PIN_SET(LED_COLOR_PORT, LED_COLOR_GREEN, 1);
	if(b) PIN_SET(LED_COLOR_PORT, LED_COLOR_BLUE, 0);
	else PIN_SET(LED_COLOR_PORT, LED_COLOR_BLUE, 1);
}

void drv_led_shutdown(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = LED_COLOR_PINs;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(LED_COLOR_PORT, &GPIO_InitStructure);
	
	PIN_SET(LED_COLOR_PORT, LED_COLOR_RED, 0);
	PIN_SET(LED_COLOR_PORT, LED_COLOR_GREEN, 0);
	PIN_SET(LED_COLOR_PORT, LED_COLOR_BLUE, 0);
    drv_led_set(0, 0, 0);
}
#else
static int led_r, led_g, led_b;

/**
 @brief 250ns delay macro.

 the delay should be within 250ns+-75ns.
 The real delay time may be a little bit different by OS level.
 The delay in OS size, is about 250ns.
 The delay in OS none, is about 260ns.
 */
#define delay_250ns()  do {    \
        __NOP();    \
        __NOP();    \
        __NOP();    \
        __NOP();    \
        __NOP();    \
        __NOP();    \
        __NOP();    \
        __NOP();    \
        __NOP();    \
        __NOP();    \
} while (0)

/**
 @brief Color led bit command.
 @param bit - bit value, 0 or 1.
 @return none.

  Bit 0 command pulse.
    ____
   |    |________________|
   0.25us      1.0us

  Bit 1 command pulse
    ________________
   |                |____|
        1.0us       0.25us

  RESET command
   |_____________________|
        >= 24us
 */
#define drv_led_bit(bit)    do {    \
    if (bit) {  \
        PIN_SET(LED_COLOR_PORT, LED_COLOR_PIN, 1);  \
        delay_250ns();  \
        delay_250ns();  \
        delay_250ns();  \
        delay_250ns();  \
        delay_250ns();  \
        delay_250ns();  \
		delay_250ns();  \
        PIN_SET(LED_COLOR_PORT, LED_COLOR_PIN, 0);  \
        delay_250ns();  \
    } else {    \
        PIN_SET(LED_COLOR_PORT, LED_COLOR_PIN, 1);  \
        delay_250ns();  \
        PIN_SET(LED_COLOR_PORT, LED_COLOR_PIN, 0);  \
        delay_250ns();  \
        delay_250ns();  \
        delay_250ns();  \
        delay_250ns();  \
        delay_250ns();  \
		delay_250ns();  \
        delay_250ns();  \
    }   \
} while (0)

/**
 @brief Color led byte command send.
 @param cmd - byte command.
 @return none.
 */
static void drv_led_cmd(uint8_t cmd)
{
    drv_led_bit(cmd & 0x80);
    drv_led_bit(cmd & 0x40);
    drv_led_bit(cmd & 0x20);
    drv_led_bit(cmd & 0x10);
    drv_led_bit(cmd & 0x08);
    drv_led_bit(cmd & 0x04);
    drv_led_bit(cmd & 0x02);
    drv_led_bit(cmd & 0x01);
}

/**
 @brief LED reset.
 @param none.
 @return none.

 Just wait for the active of new settings.
 */
static void drv_led_reset(void)
{
    _delay_us(20);
}

/**
 @brief LED color set.
 @param r - red color, 0 ~ 255.
 @param g - green color, 0 ~ 255.
 @param b - blue color, 0 ~ 255.
 @return none.
 */
void drv_led_set(int r, int g, int b)
{
    uint8_t reset = false;
    r *= 0x10;
    g *= 0x10;
    b *= 0x10;

    if (r < 0 || r > 0xFF) {
        return ;
    }
    if (g < 0 || g > 0xFF) {
        return ;
    }
    if (b < 0 || b > 0xFF) {
        return ;
    }

    if (led_r != r) {
        led_r = r;
        reset = true;
    }

    if (led_g != g) {
        led_g = g;
        reset = true;
    }

    if (led_b != b) {
        led_b = b;
        reset = true;
    }

    if (reset) {
        drv_led_cmd(led_g);
        drv_led_cmd(led_r);
        drv_led_cmd(led_b);
        /* FIXME: to make the settings active, should wait reset.
         * But it'll waste a little time. So let caller do this.
         */
//      drv_led_reset();
    }
}

/**
 @brief Color LED initialize.
 @param none.
 @return none.
 */
int drv_led_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = LED_COLOR_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_Init(LED_COLOR_PORT, &GPIO_InitStructure);
	
	PIN_SET(LED_COLOR_PORT, LED_COLOR_PIN, 0);
    drv_led_set(0, 0, 0);
    drv_led_reset();
	led_r = led_g = led_b = -1;
	return 1;
}

/**
 @brief Color LED shutdonw.
 @param none.
 @return none.
 */
void drv_led_shutdown(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = LED_COLOR_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_Init(LED_COLOR_PORT, &GPIO_InitStructure);
	
	PIN_SET(LED_COLOR_PORT, LED_COLOR_PIN, 0);
    drv_led_set(0, 0, 0);
    drv_led_reset();
    led_r = led_g = led_b = 0;
}
#endif
static _u32 _led_ticks = 0;
static _u8  _led_status = 0;

/*
 * 指示灯心跳函数
 */
void led_heartbeat(float n)
{
    if (n < 15) {
        if (getms() - _led_ticks < 250) {
            return ;
        }
        /* Blink red led when battery is less than 15%. */
        drv_led_set(_led_status, 0, 0);
        _led_status = _led_status ? 0 : 1;
        _led_ticks = getms();
        return ;
    } else if (n < 30) {
        if (getms() - _led_ticks < 250) {
            return ;
        }
        /* Blink yellow led when battery is less than 30%. */
        drv_led_set(_led_status, _led_status, 0);
        _led_status = _led_status ? 0 : 1;
        _led_ticks = getms();
        return ;
    }

	if (getms() - _led_ticks < 250) {
		return ;
	}
	drv_led_set(0, _led_status, _led_status);
	_led_status = _led_status ? 0 : 1;
	_led_ticks = getms();
}
