#include "common.h"
#include <stdio.h>
#include <math.h>
#include "beep.h"
#include "battery.h"
#include "motor.h"
#include "watchdog.h"
#include "led.h"
#include "sonar.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "filters.h"
#include "protocol.h"

#define PI 3.14159265358979f
#define fastAbs(n) n > 0 ? n : -n

int _MPU6050 = 1, _MPU6050_DMP = 1, _MOTOR, _USART3, _LED, _BATTERY, _BEEP, _SONAR;
short Temperature;																//温度
float pitch, roll, yaw;															//欧拉角

/*
* 初始化板级外设函数
*/
static _s32 init_dev(void)
{
//#ifdef _PRINT
//	int _USART2;
//	_USART2 = usart2_init(115200);
//#endif
	_USART3 = usart3_init(115200);
	_LED = drv_led_init();
	_BATTERY = init_battery();													//初始化电池电量、充电相关
	_BEEP = init_beep();														//初始化蜂鸣器，用于各种声音提示
	_MOTOR = InitMotor();														//初始化两路行走电机，输出的速度分辨率为-300 ~ 300
	stalldetector_init();
	_SONAR = sonar_init();
#ifdef _ENABLE_MPU6050
	_MPU6050 = MPU_Init();
	_MPU6050_DMP = mpu_dmp_init();
#endif
	set_walkingmotor_speed(0, 0);												//速度设定 左：0mm/s 右：0mm/s
	return 1;
}

extern _u8 sdp_status;
/*
 * 模块循环处理函数
 */
static void dev_heartbeat(void)
{
#ifdef _ENABLE_MPU6050
	int get_data;
	short aacx, aacy, aacz;				//加速度传感器原始数据
	short gyrox, gyroy, gyroz;			//陀螺仪原始数据
#ifdef _PRINT
	printf("_MPU6050: %d\r\n", _MPU6050);
	printf("_MPU6050_DMP: %d\r\n", _MPU6050_DMP);
#endif
	if(_MPU6050 == 0 && _MPU6050_DMP == 0)
	{
		get_data = mpu_dmp_get_data(&pitch, &roll, &yaw);
#ifdef _PRINT
		printf("get_data: %d\r\n", get_data);
#endif
		if(get_data == 0)
		{
			Temperature = MPU_Get_Temperature();								//得到温度值
			MPU_Get_Accelerometer(&aacx, &aacy, &aacz);					//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);					//得到陀螺仪数据
			Kalman_Filter(atan2(aacx, aacy) * 180 / PI, -(float)gyroz /16.4);
#ifdef _PRINT
			printf("\r\nAccelerometer: x: %.6f, y: %.6f, z: %.6f\r\n\r\n", (float)aacx / 16384, (float)aacy / 16384, (float)aacz / 16384);
			printf("Gyroscope: x: %.6f, y: %.6f, z: %.6f\r\n\r\n", (float)gyrox / 16.4, (float)gyroy / 16.4, (float)gyroz / 16.4);
			printf("Pitch: %.2f, Roll: %.2f, Yaw: %.2f\r\n\r\n", pitch, roll, yaw);
			printf("Kalman Filter Yaw: %.2f\r\n\r\n", angle);
			printf("Temperature: %.2f\r\n\r\n", (float)Temperature / 100);
#endif
		}
	}
#endif
	heartbeat_battery();
	heartbeat_beep(get_electricitypercentage());
	ReceiveData(Temperature, pitch, roll, yaw);
	speedctl_heartbeat();
	stalldetector_heartbeat();
	sonar_heartbeat();
#ifdef _PRINT
	printf("Voltage of battery: %.2f mV\r\n", get_electricity());
	printf("Percentage of battery: %.2f%%\r\n", get_electricitypercentage());
#endif
}

/*
 * 主循环函数
 */
static _s32 loop(void)
{
	led_heartbeat(get_electricitypercentage());
	dev_heartbeat();                                            //处理各个功能模块
	return 1;
}
/*
 * 主程序函数
 */
int main(void)
{
	_delay_ms(100);												//等待上电电源稳定

	init_board();												//MCU低级初始化
	if (!init_dev()) 
	{															//所有外设初始化
		goto _on_fail;
	}

	play_poweron();												//开机发声
	enable_watchdog();

	while (loop()) 
	{
		mark_watchdog();
	}

  _on_fail:
	disable_watchdog();
	drv_led_shutdown();
	set_walkingmotor_speed(0, 0);
	cli();
	on_abort_mode();
	return 0;
}
