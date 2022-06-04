#ifndef OPENMV_H
#define OPENMV_H
#include "stm32f4xx.h"
void Openmv_Data_ReceiveAnalysis(uint8_t *buff,uint8_t cnt);
void Position_Calculate(void);
#endif
