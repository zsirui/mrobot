#ifndef __BATTERY_H
#define __BATTERY_H

#define 	BATT_DETECT_PORT		GPIOC
#define 	BATT_DETECT_PIN			GPIO_Pin_0


#define BATTERY_VOLTAGE_FULL    ((int)(12.4 * 1000)) //mV
#define BATTERY_VOLTAGE_EMPTY   ((int)(9.25 * 1000)) //mV

int init_battery(void);
float get_electricity(void);
float get_electricitypercentage(void);
void heartbeat_battery(void);

#endif
