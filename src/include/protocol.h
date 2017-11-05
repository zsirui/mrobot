#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_
#include "types.h"
#include "stm32f10x.h"

void SendData(short Temperature, float pitch, float roll, float yaw);
void ReceiveData(short Temperature, float pitch, float roll, float yaw);

#endif
