#ifndef MOTOR_H
#define MOTOR_H
#include "stm32f4xx.h"

void MOTOR_Init(void);
void Moto_Pwm(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM);
#endif
