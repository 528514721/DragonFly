#ifndef __IIC_MONI_H
#define __IIC_MONI_H
#include "stm32f4xx.h"
#include "nvicconfig.h"


/******************************注意！！！**************************************
移植此驱动，驱动DMP解算四元数时，注意从机地址要设置成0xD0
*******************************************************************************/

//IIC所有操作函数
void IIC_Init(void);        //初始化IIC的IO口				 
void IIC_Start(void);			  //发送IIC开始信号
void IIC_Stop(void);	  	  //发送IIC停止信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号
uint8_t IIC_WaitAck(void); 		 //IIC等待ACK信号

void IIC_SendByte(uint8_t data);  //IIC发送一个字节
uint8_t IIC_ReadByte(uint8_t ack);//IIC读取一个字节

uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf);
uint8_t IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t buf);
uint8_t IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);

void AT24C02_WriteLen(u8 length,u8*data);
void AT24C02_ReadLen(u8 length,uint8_t*data);
#endif





