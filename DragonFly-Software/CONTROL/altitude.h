#ifndef ALTITUDE_H
#define ALTITUDE_H
#include "stm32f4xx.h"

#define  ALT_Max 3.0f //定高高度

extern float height;

void Altitude_Combine(void);
void Altitude_Control(void);
void altitude_get(void);
#endif
