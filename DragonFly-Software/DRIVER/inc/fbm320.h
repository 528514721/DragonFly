#ifndef FBM320_H
#define FBM320_H
#include "stm32f4xx.h"

#define FBMAddr 0x6C   //气压计从机IIC地址
#define FBMAddr2 0x6D   //气压计从机IIC地址

/****************************************气压计寄存器地址***********************************************/
#define SPI_CTRL     0x00 //SPI通信时配置寄存器 (本工程使用IIC驱动 ，应设置为0x00)
#define FBM_ID       0x6B //FBM320 身份寄存器
#define FBM_COEFF1   0xAA //FBM320 校准寄存器
#define FBM_COEFF2   0xBB
#define FBM_COEFF3   0xD0
#define FBM_COEFF4   0xF1
#define FBM_RESET    0xE0 //FBM320 复位寄存器
#define FBM_CONFIG   0xF4 //FBM320 这配置寄存器 6:7 OSR（精度），0:5 101110:温度转化命令/110100:气压转换命令
#define DATA_MSB     0xF6 //FBM320 数据寄存器 16:23
#define DATA_CSB     0xF7 //FBM320 数据寄存器 8:15
#define DATA_LSB     0xF8 //FBM320 数据寄存器 0:7

#define OSR1024  0x00
#define OSR2048  0x40
#define OSR4096  0x80
#define OSR8192  0xC0
#define PRES_CONVERSION     0x34
#define TEMP_CONVERSION     0x2E
#define FBMRESET      0xB6
#define FBMID         0x42

extern float RPFilter;

void FBM320_Init(void);
void FBM320_Check(void);
void FBM320_Init(void);
void FBM320_GetCoeff(void);
void FBM320_Calculate(int32_t UP, int32_t UT);
int32_t Abs_Altitude(int32_t Press);
uint8_t Init_Altitude(void);
void FBM320_GetAltitude(void);

#endif
