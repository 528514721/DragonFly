#ifndef REMOTEDATA_H
#define REMOTEDATA_H
#include "stm32f4xx.h"
void Remote_Data_ReceiveAnalysis(void);
void NRF_SingalCheck(void);
void WiFi_Data_Receive(uint8_t data);
void WiFi_Data_ReceiveAnalysis(uint8_t*buff,uint8_t cnt);

void SendToRemote(void);
#endif
