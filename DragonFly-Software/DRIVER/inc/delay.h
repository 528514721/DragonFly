#ifndef DELAY_H
#define DELAY_H

#include "stm32f4xx.h"

/**********延时方式选择*************/
#define NOINT_DELAY //不开中断延时
//#define INT_DELAY //开中断延时
/**********************************/

void Delay_Init(void);
void delay_ms(uint32_t nTime);
void delay_us(uint32_t ntime);
void delay(uint32_t timers);

uint32_t micros(void);
uint32_t millis(void);
#endif
