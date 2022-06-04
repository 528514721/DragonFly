#ifndef LED_H
#define LED_H
#include "stm32f4xx.h"

void LED_Init(void);
void LED_Run(void);
void RGB_LED_Rand(void);
void RGB_LED_Runing(void);
void RGB_LED_Off(void);
void RGB_LED_Red(void);
void RGB_LED_Orange(void);
void RGB_LED_Yellow(void);
void RGB_LED_green(void);
void RGB_LED_Cyan(void);
void RGB_LED_Blue(void);
void RGB_LED_Violet(void);
void RGB_LED_FLY(void);
void RGB_LED_White(void);
void OneNET_LED(uint8_t color[],uint8_t num);
void GYRO_Offset_LED(void);
void ACC_Offset_LED(void);
void BAR_Offset_LED(void);
void WiFi_OFFON_LED(void);
void BATT_Alarm_LED(void);

void RGB_Unlock(uint8_t N,uint8_t flag);
void RunTimer_Test(void);
void RunTimer_Test2(void);
#endif
