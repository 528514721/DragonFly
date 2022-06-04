#ifndef SPI_H
#define SPI_H
#include "stm32f4xx.h"

void SPI2_Init(void);
void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler);
uint8_t SPI2_WriteReadByte(uint8_t data);
#endif
