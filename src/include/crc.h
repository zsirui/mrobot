#ifndef __CRC_H
#define __CRC_H
#include "stm32f10x.h"

u8 Get_Crc8(u8 *ptr,u16 len);
u16 Get_Crc16(u8 *puchMsg,u16 usDataLen);

#endif
