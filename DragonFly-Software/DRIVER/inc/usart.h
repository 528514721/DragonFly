#ifndef USART_H
#define USART_H

#include "stm32f4xx.h"
#include "stdio.h"

void Usart1_Init(uint32_t bound);
void Usart2_Init(uint32_t bound);

void Usart_Send(uint8_t *data, uint8_t length);
#endif
