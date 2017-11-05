#include "common.h"
#include "battery.h"

static _u32 batteryFrequency = 0;
static float batteryElectricityPercentage = 0;
extern _u32 voltage;

int init_battery(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = BATT_DETECT_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(BATT_DETECT_PORT, &GPIO_InitStructure);
	return 1;
}

/*
 * 获取电池电压函数
 * 返回电压值，单位：mV
 */
float get_electricity(void)
{
	return (float)voltage * (3.3 / 4096) * 9090.91 - 2359.09;
}
/*
 * 获取电池容量百分比函数
 * 返回百分比0-100%
 */
float get_electricitypercentage(void)
{
    return batteryElectricityPercentage;
}

/*
 * 电池相关模块函数
 * 充电状态的判定等
 */
void heartbeat_battery(void)
{
	_u32 currentVolt;
    if ((getms() - batteryFrequency) >= 3000) {
        //3秒检测一次电池容量及计算百分比
        batteryFrequency = getms();
        currentVolt = get_electricity();
        if (currentVolt < BATTERY_VOLTAGE_EMPTY) {
            batteryElectricityPercentage = 0.0;
        } else if (currentVolt > BATTERY_VOLTAGE_FULL) {
            batteryElectricityPercentage = 100.0;
        } else {
            batteryElectricityPercentage = (float)(currentVolt - BATTERY_VOLTAGE_EMPTY) * 100 / (float)(BATTERY_VOLTAGE_FULL - BATTERY_VOLTAGE_EMPTY);
        }
    }
}
