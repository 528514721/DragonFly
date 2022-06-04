#ifndef MONI_FLASH_H
#define MONI_FLASH_H
#include "stm32f4xx.h"

u8 Compare_ParamsTable(void);
void TableToParams(void);
void ParamsToTable(void);
void ParamsClearAll(void);
void PID_WriteFlash(void);
void PID_ReadFlash(void);
void PID_ClearFlash(void);
void DefaultParams(void);
void DefaultParams_WriteFlash(void);
#endif
